#include "tagbearing/tagbearing_node.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cg {
namespace tagbearing {

TagBearingNode::TagBearingNode() : Node("tagbearing_node") {
  // Initialize publishers and subscribers
  img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/undistorted_img", 1
  );
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 1, std::bind(&TagBearingNode::imageCallback, this, std::placeholders::_1)
  );
}

void TagBearingNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "I heard: '%i, %i, %i'", msg->height, msg->width, msg->data[0]);
  //cv_bridge::CvImagePtr cv_ptr;
  //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  //cv::Mat undistorted;
  //cv::fisheye::undistortPoints(cv_ptr->image, undistorted, K, D, R, P)
  //img_pub.publish(undistorted->toImageMsg())
}

}  // namespace image
}  // namespace cg
