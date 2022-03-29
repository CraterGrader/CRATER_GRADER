#include "planning/autograder_node.hpp"

namespace cg {
namespace planning {

AutoGraderNode::AutoGraderNode() : Node("autograder_node") {
  // Initialize publishers and subscribers
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TeleopNode::timerCallback, this)
  );
  // Load parameters

}

void AutoGraderNode::timerCallback() {

}


}  // namespace teleop
}  // namespace cg
