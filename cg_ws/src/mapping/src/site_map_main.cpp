#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mapping/site_map_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::mapping::SiteMapNode>());
    rclcpp::shutdown();
    return 0;
}
