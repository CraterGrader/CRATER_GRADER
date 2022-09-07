#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "imu/imu_base_link_conversion_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::imu::ImuBaseLinkConversionNode>());
    rclcpp::shutdown();
    return 0;
}
