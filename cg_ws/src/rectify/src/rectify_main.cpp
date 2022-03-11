#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rectify/rectify_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::rectify::RectifyNode>());
    rclcpp::shutdown();
    return 0;
}
