#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "planning/autograder_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::planning::AutoGraderNode>());
    rclcpp::shutdown();
    return 0;
}
