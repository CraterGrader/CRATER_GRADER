#ifndef RECTIFY__RECTIFY_NODE_HPP
#define RECTIFY__RECTIFY_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace cg {
namespace rectify {

class RectifyNode : public rclcpp::Node {

public:
  RectifyNode();
  cv::Mat K = (cv::Mat1d(3, 3) << 269.41299877, 0., 311.52540063, 0., 269.60663548, 220.38433628, 0., 0., 1.);
  cv::Mat D = (cv::Mat1d(4, 1) << -0.04884999, 0.01215806, -0.01517078, 0.00470384);
  cv::Mat R = (cv::Mat1d(3, 3) << 1., 0., 0., 0., 1., 0., 0., 0., 1.);
  //cv::Mat R = (cv::Mat1d(3, 1) << 1., 1., 1.);
  //cv::Mat P = (cv::Mat1d(3, 4) << 269.41299877, 0., 311.52540063, 0., 0., 269.60663548, 220.38433628, 0., 0., 0., 1., 0.);
private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  /* Callbacks */
  // Callback for image input
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};


}  // namespace rectify
}  // namespace cg

#endif  // RECTIFY__RECTIFY_NODE_HPP
