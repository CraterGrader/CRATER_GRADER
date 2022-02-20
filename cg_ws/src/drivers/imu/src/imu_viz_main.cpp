#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "imu/imu_viz_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::imu::ImuVizNode>());
    rclcpp::shutdown();
    return 0;
}
