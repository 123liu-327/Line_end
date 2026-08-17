#include <flow_end/yellow_line_detector.h>
#include <flow_end/follow.h>
#include <flow_end/ImagePerspectiveInit.h>
#include <flow_end/generateLookupTable.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace flow_end {

YellowLineDetector::YellowLineDetector()
    : h_min_(12), h_max_(42), s_min_(70), s_max_(255), v_min_(70), v_max_(255),
      min_width_px_(24), roi_y_min_(120), roi_y_max_(450), min_width_m_(0.08f),
      max_angle_deg_(8.0f), min_confidence_(0.45f), pixel_per_meter_(500.0f),
      reference_y_(490.0f), camera_to_front_offset_m_(0.0f) {
  ImagePerspective_Init();
  generateLookupTable(mapx, mapy);
}

void YellowLineDetector::configure(int h_min, int h_max, int s_min, int s_max,
                                    int v_min, int v_max, int min_width_px,
                                    int roi_y_min, int roi_y_max, float min_width_m,
                                    float max_angle_deg, float min_confidence,
                                    float pixel_per_meter, float reference_y,
                                    float front_offset_m) {
  h_min_ = h_min; h_max_ = h_max; s_min_ = s_min; s_max_ = s_max;
  v_min_ = v_min; v_max_ = v_max; min_width_px_ = min_width_px;
  roi_y_min_ = roi_y_min; roi_y_max_ = roi_y_max; min_width_m_ = min_width_m;
  max_angle_deg_ = max_angle_deg; min_confidence_ = min_confidence;
  pixel_per_meter_ = std::max(1.0f, pixel_per_meter);
  min_width_px_ = std::max(min_width_px,
                           static_cast<int>(std::ceil(min_width_m_ * pixel_per_meter_)));
  reference_y_ = reference_y; camera_to_front_offset_m_ = front_offset_m;
}

YellowLineDetection YellowLineDetector::detect(const cv::Mat& input) const {
  YellowLineDetection out;
  if (input.empty()) { out.reason = "empty_image"; return out; }
  cv::Mat bgr;
  if (input.size() != cv::Size(RESULT_COL, RESULT_ROW))
    cv::resize(input, bgr, cv::Size(RESULT_COL, RESULT_ROW), 0, 0, cv::INTER_AREA);
  else bgr = input;

  cv::Mat hsv, mask, bird;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_),
              cv::Scalar(h_max_, s_max_, v_max_), mask);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                   cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3)));
  cv::remap(mask, bird, cv::Mat(RESULT_ROW, RESULT_COL, CV_32F, mapx),
            cv::Mat(RESULT_ROW, RESULT_COL, CV_32F, mapy), cv::INTER_NEAREST,
            cv::BORDER_CONSTANT, cv::Scalar(0));

  const int y0 = std::max(0, roi_y_min_);
  const int y1 = std::min(RESULT_ROW - 1, roi_y_max_);
  int best_left = -1, best_right = -1, best_y = -1;
  int best_width = 0, yellow_pixels = 0;
  float best_center_error = std::numeric_limits<float>::max();
  for (int y = y0; y <= y1; ++y) {
    int run_start = -1;
    for (int x = 0; x <= RESULT_COL; ++x) {
      const bool on = x < RESULT_COL && bird.at<unsigned char>(y, x) > 0;
      if (on) { ++yellow_pixels; if (run_start < 0) run_start = x; }
      if ((!on || x == RESULT_COL) && run_start >= 0) {
        const int right = x - 1, width = right - run_start + 1;
        const float center = 0.5f * (run_start + right);
        const float error = std::fabs(center - RESULT_COL * 0.5f);
        if (width > best_width || (width == best_width && error < best_center_error)) {
          best_left = run_start; best_right = right; best_y = y;
          best_width = width; best_center_error = error;
        }
        run_start = -1;
      }
    }
  }
  if (best_width < min_width_px_) {
    out.reason = "horizontal_span_too_short";
    out.yellow_pixels = yellow_pixels;
    return out;
  }

  const int cx = (best_left + best_right) / 2;
  const int cy = best_y;
  if (cx < 0 || cx >= RESULT_COL || cy < 0 || cy >= RESULT_ROW ||
      point_map[cy][cx][1] < 0 || point_map[cy][cx][1] >= USED_ROW) {
    out.reason = "ipm_point_unmapped";
    out.yellow_pixels = yellow_pixels;
    return out;
  }
  std::vector<cv::Point> points;
  for (int y = std::max(y0, cy - 8); y <= std::min(y1, cy + 8); ++y)
    for (int x = best_left; x <= best_right; ++x)
      if (bird.at<unsigned char>(y, x) > 0) points.emplace_back(x, y);
  float angle = 0.0f;
  if (points.size() >= 2) {
    cv::Vec4f line; cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
    angle = std::atan2(line[1], line[0]) * 180.0f / static_cast<float>(CV_PI);
    while (angle > 90.0f) angle -= 180.0f;
    while (angle < -90.0f) angle += 180.0f;
  }
  const float map_x = static_cast<float>(point_map[cy][cx][0]);
  const float map_y = static_cast<float>(point_map[cy][cx][1]);
  const float longitudinal = -(map_y - reference_y_) / pixel_per_meter_ - camera_to_front_offset_m_;
  const float lateral = -(map_x - RESULT_COL * 0.5f) / pixel_per_meter_;
  const float width_score = std::min(1.0f, best_width / (0.50f * pixel_per_meter_));
  const float angle_score = std::max(0.0f, 1.0f - std::fabs(angle) / max_angle_deg_);
  const float confidence = 0.55f * width_score + 0.45f * angle_score;
  out.center_x_px = cx; out.center_y_px = cy;
  out.center_error_px = cx - RESULT_COL * 0.5f;
  out.angle_deg = angle; out.longitudinal_m = longitudinal;
  out.lateral_m = lateral; out.confidence = confidence;
  out.width_px = best_width; out.yellow_pixels = yellow_pixels;
  out.valid = std::fabs(angle) <= max_angle_deg_ && confidence >= min_confidence_ && longitudinal > 0.0f;
  out.reason = out.valid ? "ok" : (longitudinal <= 0.0f ? "line_behind_reference" : "geometry_rejected");
  return out;
}

}  // namespace flow_end
