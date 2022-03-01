#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "tagbearing/tagbearing_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::tagbearing::TagBearingNode>());
    rclcpp::shutdown();
    return 0;
}
