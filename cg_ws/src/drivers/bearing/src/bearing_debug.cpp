#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/string.hpp>

namespace cg {
namespace bearing {

class BearingDebugNode : public rclcpp::Node {
public:
  BearingDebugNode() : Node("bearing_debug_node") {
    // sub_cam(image_transport::create_camera_subscription(this, "/image_raw",
    //   std::bind(&BearingDebugNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
    //   declare_parameter<std::string>("image_transport", "raw"), rmw_qos_profile_sensor_data)) {
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(), std::bind(&BearingDebugNode::onCamera, this, std::placeholders::_1)
    );
    debug_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/bearing_debug", 1
    );
    
  }
private:
  // const image_transport::CameraSubscriber sub_cam;
  // void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) {
  void onCamera(const sensor_msgs::msg::Image::SharedPtr& msg) {
    std_msgs::msg::String str_msg;
    str_msg.data = "Image received";
    debug_pub_->publish(str_msg);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

} // namespace bearing
} // namespace cg

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::bearing::BearingDebugNode>());
    rclcpp::shutdown();
    return 0;
}
