#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "localization/ts_prism_transformer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cg::total_station_rtls::TSPrismTransformer>());
  rclcpp::shutdown();
  return 0;
}
