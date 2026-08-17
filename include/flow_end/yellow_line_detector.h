#pragma once

#include <opencv2/core/core.hpp>
#include <string>

namespace flow_end {

struct YellowLineDetection {
  bool valid = false;
  float center_x_px = 0.0f;
  float center_y_px = 0.0f;
  float center_error_px = 0.0f;
  float angle_deg = 0.0f;
  float longitudinal_m = 0.0f;
  float lateral_m = 0.0f;
  float confidence = 0.0f;
  int width_px = 0;
  int yellow_pixels = 0;
  std::string reason;
};

class YellowLineDetector {
 public:
  YellowLineDetector();
  void configure(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max,
                 int min_width_px, int roi_y_min, int roi_y_max,
                 float min_width_m, float max_angle_deg, float min_confidence,
                 float pixel_per_meter, float reference_y, float front_offset_m);
  YellowLineDetection detect(const cv::Mat& bgr) const;

 private:
  int h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
  int min_width_px_, roi_y_min_, roi_y_max_;
  float min_width_m_, max_angle_deg_, min_confidence_;
  float pixel_per_meter_, reference_y_, camera_to_front_offset_m_;
};

}  // namespace flow_end
