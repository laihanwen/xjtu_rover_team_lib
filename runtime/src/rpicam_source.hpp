#ifndef AUV_RUNTIME__RPICAM_SOURCE_HPP_
#define AUV_RUNTIME__RPICAM_SOURCE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

struct RpicamSourceConfig
{
  int width{640};
  int height{480};
  int fps{30};
  double exposure_compensation{0.0};
  double gain{0.0};
  double brightness{0.4};
  std::string denoise{"cdn_fast"};
  int quality{50};
};

class RpicamSource
{
public:
  explicit RpicamSource(RpicamSourceConfig config);
  ~RpicamSource();

  RpicamSource(const RpicamSource &) = delete;
  RpicamSource & operator=(const RpicamSource &) = delete;

  bool open();
  bool read(cv::Mat & frame, int timeout_ms);
  void close();
  bool is_open();

private:
  bool extract_frame(cv::Mat & frame);

  RpicamSourceConfig config_;
  int read_fd_{-1};
  int child_pid_{-1};
  std::vector<std::uint8_t> buffer_;
};

#endif
