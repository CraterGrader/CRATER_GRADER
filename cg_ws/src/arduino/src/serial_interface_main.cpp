#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "arduino/serial_interface_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::arduino::SerialInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
