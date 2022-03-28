#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "localization/slip_estimate_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::slip::SlipEstimateNode>());
    rclcpp::shutdown();
    return 0;
}
