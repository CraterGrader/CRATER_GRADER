#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "bearing/bearing_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::bearing::BearingNode>());
    rclcpp::shutdown();
    return 0;
}
