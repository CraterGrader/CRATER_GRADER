#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "localization/beacon_transformer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::uwb_beacon_rtls::BeaconTransformer>());
  rclcpp::shutdown();
  return 0;
}
