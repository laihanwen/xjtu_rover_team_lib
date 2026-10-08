// Copyright 2026 hanwen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>

#include "auv_mapping/grid_mapper.hpp"

#include "opencv2/imgproc.hpp"

namespace {

// Competition scene constants: a dark blue pool floor, white edges and
// internal divisions, and a single yellow bottom-reference edge.
auv_mapping::GridMapperConfig competition_config() {
  auv_mapping::GridMapperConfig config;
  config.single_yellow_edge = true;
  config.white_grid_edges = true;
  return config;
}
const cv::Scalar kPoolFloor(120, 60, 20);
const cv::Scalar kWhite(255, 255, 255);
const cv::Scalar kYellow(0, 255, 255);

// Warp a canonical 500x500 grid onto a larger dark-blue canvas so the mapper
// must recover the quadrilateral under perspective rather than read an
// axis-aligned rectangle.
cv::Mat warp_to_perspective(const cv::Mat &canonical) {
  const std::array<cv::Point2f, 4> source{
      cv::Point2f(0.0F, 0.0F), cv::Point2f(499.0F, 0.0F),
      cv::Point2f(499.0F, 499.0F), cv::Point2f(0.0F, 499.0F)};
  const std::array<cv::Point2f, 4> destination{
      cv::Point2f(115.0F, 80.0F), cv::Point2f(550.0F, 115.0F),
      cv::Point2f(590.0F, 430.0F), cv::Point2f(70.0F, 455.0F)};
  const cv::Mat transform =
      cv::getPerspectiveTransform(source.data(), destination.data());
  cv::Mat perspective(520, 660, CV_8UC3, kPoolFloor);
  cv::warpPerspective(canonical, perspective, transform, perspective.size(),
                      cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
  return perspective;
}

// Draw the canonical 500x500 grid: three white edges plus one yellow edge
// (yellow_side: 0=top, 1=right, 2=bottom, 3=left), with white 2x2 internal
// divisions, over a dark-blue pool floor.
cv::Mat make_canonical_grid(const bool draw_internal_lines,
                            const int yellow_side) {
  cv::Mat canonical(500, 500, CV_8UC3, kPoolFloor);
  cv::rectangle(canonical, cv::Rect(8, 8, 484, 484), kWhite, 18);
  if (yellow_side == 0) {
    cv::line(canonical, cv::Point(8, 8), cv::Point(492, 8), kYellow, 18);
  } else if (yellow_side == 1) {
    cv::line(canonical, cv::Point(492, 8), cv::Point(492, 492), kYellow, 18);
  } else if (yellow_side == 3) {
    cv::line(canonical, cv::Point(8, 8), cv::Point(8, 492), kYellow, 18);
  } else {
    cv::line(canonical, cv::Point(8, 492), cv::Point(492, 492), kYellow, 18);
  }
  if (draw_internal_lines) {
    cv::line(canonical, cv::Point(167, 10), cv::Point(167, 490), kWhite, 8);
    cv::line(canonical, cv::Point(333, 10), cv::Point(333, 490), kWhite, 8);
    cv::line(canonical, cv::Point(10, 167), cv::Point(490, 167), kWhite, 8);
    cv::line(canonical, cv::Point(10, 333), cv::Point(490, 333), kWhite, 8);
  }
  return canonical;
}

cv::Mat make_grid(const bool draw_internal_lines = true,
                  const int yellow_side = 2) {
  return warp_to_perspective(
      make_canonical_grid(draw_internal_lines, yellow_side));
}

// A legacy-style grid whose four border edges are all yellow: no single edge
// stands out, so yellow-edge orientation must stay unconfirmed indefinitely.
cv::Mat make_all_yellow_grid() {
  cv::Mat canonical = make_canonical_grid(true, 2);
  cv::line(canonical, cv::Point(8, 8), cv::Point(492, 8), kYellow, 18);
  cv::line(canonical, cv::Point(492, 8), cv::Point(492, 492), kYellow, 18);
  cv::line(canonical, cv::Point(8, 8), cv::Point(8, 492), kYellow, 18);
  return warp_to_perspective(canonical);
}

} // namespace
#include "auv_core/semantic_map.hpp"
#include <limits>
#include <stdexcept>
static void check(bool value, const char *why) {
  if (!value)
    throw std::runtime_error(why);
}
int main() {
  auto cfg = competition_config();
  for (int side = 0; side < 4; ++side) {
    auv_mapping::GridMapper m(cfg);
    auv_mapping::GridResult g;
    for (int i = 0; i < 5; ++i)
      g = m.process(make_grid(true, side));
    check(g.geometry_valid && g.stable && g.orientation_valid &&
              g.yellow_edge >= 0,
          "white four-way grid/A2 interface failed");
    cv::Mat hsv, yellow;
    cv::cvtColor(g.rectified, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv(cv::Rect(60, 570, 480, 30)), cv::Scalar(15, 60, 60),
                cv::Scalar(40, 255, 255), yellow);
    check(cv::countNonZero(yellow) > yellow.total() * .2,
          "yellow not at bottom");
    auv_vision::ConeObservation c;
    c.row = 0;
    c.col = 0;
    c.shape = auv_vision::ConeShape::kCircle;
    c.confidence = 1;
    check(auv_core::fuse_semantic_map(g, {c}, true, {}, 1).complete,
          "confirmed map rejected");
    g.orientation_valid = false;
    check(!auv_core::fuse_semantic_map(g, {c}, true, {}, 1).complete,
          "unoriented map complete");
    g.orientation_valid = true;
    c.shape = auv_vision::ConeShape::kUnknown;
    check(!auv_core::fuse_semantic_map(g, {c}, true, {}, 1).complete,
          "unknown cone complete");
  }
  cfg.stable_frames = 1;
  cfg.yellow_oriented_frames = 5;
  auv_mapping::GridMapper m(cfg);
  for (int i = 0; i < 4; ++i) {
    auto g = m.process(make_grid());
    check(!g.orientation_valid && !g.position_valid,
          "orientation confirmed too early");
  }
  check(m.process(make_grid()).orientation_valid,
        "orientation never confirmed");
  cv::Mat shifted;
  cv::Mat transform = (cv::Mat_<double>(2, 3) << 1, 0, 30, 0, 1, 0);
  cv::warpAffine(make_grid(), shifted, transform, make_grid().size());
  auto changed = m.process(shifted);
  check(!changed.orientation_valid && !changed.position_valid,
        "corner jump did not reset orientation");
  check(!m.process(make_all_yellow_grid()).orientation_valid,
        "ambiguous yellow accepted");
  check(!m.process(cv::Mat(520, 660, CV_8UC3, cv::Scalar(0, 0, 0)))
             .geometry_valid,
        "blank frame accepted");
  check(!m.process(make_grid()).orientation_valid,
        "reset did not clear confirmation");
  for (int bad = 0; bad < 3; ++bad) {
    auto invalid = cfg;
    if (bad == 0)
      invalid.white_s_max = std::numeric_limits<double>::quiet_NaN();
    if (bad == 1)
      invalid.yellow_oriented_frames = 0;
    if (bad == 2)
      invalid.single_yellow_edge = false;
    bool rejected = false;
    try {
      auv_mapping::GridMapper x(invalid);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    check(rejected, "invalid config accepted");
  }
}
