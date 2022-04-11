#ifndef MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP
#define MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

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
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /* Callbacks */
  void timerCallback();
  void filteredPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  pcl::PointCloud<pcl::PointXYZ>::Ptr new_point_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map_;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
  geometry_msgs::msg::TransformStamped tf_;

  bool new_data_received_;

  std::string source_frame_;
  std::string target_frame_;
  int icp_max_iters_;
  double icp_max_correspondence_dist_;
  double icp_transformation_eps_;
  double icp_euclidean_fitness_epsilon_;
  // int icp_min_num_correspondences_;
  float voxel_filter_size_;
  Eigen::Affine3d matrix_;
};


}  // namespace mapping
}  // namespace cg

#endif  // MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP
