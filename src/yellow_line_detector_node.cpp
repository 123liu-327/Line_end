#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <flow_end/yellow_line_detector.h>
#include <sstream>
#include <iomanip>

class YellowLineDetectorNode {
 public:
  YellowLineDetectorNode() : nh_(), pnh_("~"), detector_() {
    image_topic_ = pnh_.param<std::string>("image_topic", "/usb_cam/image_raw");
    output_topic_ = pnh_.param<std::string>("output_topic", "/strict_mission/yellow_line");
    detector_.configure(
      pnh_.param("yellow_h_min", 12), pnh_.param("yellow_h_max", 42),
      pnh_.param("yellow_s_min", 70), pnh_.param("yellow_s_max", 255),
      pnh_.param("yellow_v_min", 70), pnh_.param("yellow_v_max", 255),
      pnh_.param("yellow_line_min_width_px", 24),
      pnh_.param("yellow_line_roi_y_min", 120), pnh_.param("yellow_line_roi_y_max", 450),
      pnh_.param("yellow_line_min_width_m", 0.08f),
      pnh_.param("yellow_line_max_angle_deg", 8.0f),
      pnh_.param("yellow_line_min_confidence", 0.45f),
      pnh_.param("pixel_per_meter", 500.0f),
      pnh_.param("ipm_reference_y_px", 490.0f),
      pnh_.param("camera_to_front_offset_m", 0.0f));
    sub_ = nh_.subscribe(image_topic_, 1, &YellowLineDetectorNode::imageCallback, this,
                         ros::TransportHints().tcpNoDelay());
    pub_ = nh_.advertise<std_msgs::String>(output_topic_, 10);
    ROS_INFO("yellow_line_detector using flow_end IPM: %s -> %s", image_topic_.c_str(), output_topic_.c_str());
  }
 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    std_msgs::String out;
    try {
      const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
      const flow_end::YellowLineDetection d = detector_.detect(image);
      std::ostringstream s; s << std::fixed << std::setprecision(5)
        << "{\"valid\":" << (d.valid ? "true" : "false")
        << ",\"center_x_px\":" << d.center_x_px
        << ",\"center_y_px\":" << d.center_y_px
        << ",\"center_error_px\":" << d.center_error_px
        << ",\"angle_deg\":" << d.angle_deg
        << ",\"longitudinal_m\":" << d.longitudinal_m
        << ",\"lateral_m\":" << d.lateral_m
        << ",\"confidence\":" << d.confidence
        << ",\"width_px\":" << d.width_px
        << ",\"yellow_pixels\":" << d.yellow_pixels
        << ",\"reason\":\"" << d.reason << "\""
        << ",\"stamp\":" << msg->header.stamp.toSec() << "}";
      out.data = s.str();
    } catch (const std::exception& e) {
      out.data = std::string("{\"valid\":false,\"reason\":\"") + e.what() + "\"}";
    }
    pub_.publish(out);
  }
  ros::NodeHandle nh_, pnh_; ros::Subscriber sub_; ros::Publisher pub_;
  std::string image_topic_, output_topic_;
  flow_end::YellowLineDetector detector_;
};

int main(int argc, char** argv) { ros::init(argc, argv, "yellow_line_detector"); YellowLineDetectorNode n; ros::spin(); return 0; }
