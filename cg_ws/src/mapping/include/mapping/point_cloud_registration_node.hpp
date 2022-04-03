#ifndef MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP
#define MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cg {
namespace mapping {

class PointCloudRegistrationNode : public rclcpp::Node {

public:
  PointCloudRegistrationNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_raw_map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* Callbacks */
  void timerCallback();
  void filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr new_point_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map_;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;

  bool initial_data_received_;
  bool new_data_received_;

  int icp_max_iters_;
};


}  // namespace mapping
}  // namespace cg

#endif  // MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP
