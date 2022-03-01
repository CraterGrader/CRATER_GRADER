#ifndef TAGBEARING__TAGBEARING_NODE_HPP
#define TAGBEARING__TAGBEARING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace cg {
namespace tagbearing {

class TagBearingNode : public rclcpp::Node {

public:
  TagBearingNode();
  //cv::InputArray K = [203.718327157626, 0, 319.5, 0, 203.718327157626, 239.5, 0, 0, 1];
  //cv::InputArray D = [0, 0, 0, 0];
  //cv::InputArray R = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  //cv::InputArray P = [203.718327157626, 0, 319.5, 0, 0, 203.718327157626, 239.5, 0, 0, 0, 1, 0];
private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  /* Callbacks */
  // Callback for image input
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};


}  // namespace tagbearing
}  // namespace cg

#endif  // TAGBEARING__TAGBEARING_NODE_HPP
