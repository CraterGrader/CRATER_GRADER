#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "uwb_beacon_rtls/uwb_beacon_rtls_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::uwb_beacon_rtls::UWBBeaconRTLSNode>());
    rclcpp::shutdown();
    return 0;
}
