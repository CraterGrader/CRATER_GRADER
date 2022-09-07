#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mapping/terrain_filtering_node.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<cg::mapping::TerrainFilteringNode>());
    rclcpp::shutdown();
    return 0;
}
