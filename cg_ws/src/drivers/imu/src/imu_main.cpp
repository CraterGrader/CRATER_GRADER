#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "imu/vn_imu_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::imu::VnImuNode>());
    rclcpp::shutdown();
    return 0;
}
