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

#include <stdexcept>

#include "auv_vision/cucumber_detector_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

namespace
{

TEST(CucumberDetectorNode, RefusesToStartWithoutPublishedModel)
{
  rclcpp::init(0, nullptr);
  EXPECT_THROW(
    {
      const auto node = auv_vision::make_cucumber_detector_node();
      (void)node;
    },
    std::invalid_argument);
  rclcpp::shutdown();
}

}  // namespace
