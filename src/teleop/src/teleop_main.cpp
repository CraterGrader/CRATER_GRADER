#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "teleop/teleop_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::teleop::TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
