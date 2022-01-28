#ifndef TELEOP__TELEOP_NODE_HPP
#define TELEOP__TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>

namespace cg {
namespace teleop {

class TeleopNode : public rclcpp::Node {

public:
    TeleopNode() : Node("teleop_node") {}

};


}  // namespace teleop
}  // namespace cg

#endif  // TELEOP__TELEOP_NODE_HPP