#include "rectify/rectify_node.hpp"

namespace cg {
namespace rectify {

RectifyNode::RectifyNode() : Node("rectify_node") {
  // Initialize publishers and subscribers
  img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/image_rect", 1
  );
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 1, std::bind(&RectifyNode::imageCallback, this, std::placeholders::_1)
  );
}

void RectifyNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  //RCLCPP_INFO(this->get_logger(), "I heard: '%i, %i, %i'", msg->height, msg->width, msg->data[0]);
  // CV Bridge Image pointer to access input image message
  cv_bridge::CvImagePtr cv_ptr;
  // Copy the input message to a cv pointer
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  
  // Define variables for calculating
  cv::Size dim(msg->width, msg->height);
  cv::Mat dst;
  cv::Mat map1;
  cv::Mat map2;

  // Undistort the image
  cv::fisheye::initUndistortRectifyMap(K, D, R, K, dim, CV_16SC2, map1, map2);
  cv::remap(cv_ptr->image, dst, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  // Create the output image
  cv_bridge::CvImage out_msg;
  out_msg.header   = msg->header; // Same timestamp and tf frame as input image
  out_msg.encoding = "bgr8"; // Same type as maps
  out_msg.image    = dst; // Set image to undistorted cv::Mat
  
  // Get output image message - dereferenced
  sensor_msgs::msg::Image out_img = *(out_msg.toImageMsg());

  // Publish Image
  img_pub_->publish(out_img);
}

}  // namespace rectify
}  // namespace cg
