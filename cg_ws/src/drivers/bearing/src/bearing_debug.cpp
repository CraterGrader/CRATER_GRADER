#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <std_msgs/msg/string.hpp>

class BearingDebugNode {
public:
  BearingDebugNode() : Node("bearing_debug_node"),
    sub_cam(image_transport::create_camera_subscription(this, "image",
      std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
      declare_parameter<std::string>("image_transport", "raw"), rmw_qos_profile_sensor_data)) {
    
    debug_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/bearing_debug", 1
    );
    
  }
private:
  const image_transport::CameraSubscriber sub_cam;
  void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci) {
    std_msgs::msg::String str_msg;
    str_msg.data = "Image received";
    debug_pub_->publish(str_msg);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::bearing::BearingDebugNode>());
    rclcpp::shutdown();
    return 0;
}
