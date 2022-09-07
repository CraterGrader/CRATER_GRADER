#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mapping/point_cloud_registration_node_but_fancier.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::mapping::PointCloudRegistrationNodeFancy>());
    rclcpp::shutdown();
    return 0;
}
