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


#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>

namespace cg {
namespace mapping {

class PointCloudRegistrationNodeFancy : public rclcpp::Node {

public:
  PointCloudRegistrationNodeFancy();

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
  // bool first_pc;

  std::string source_frame_;
  std::string target_frame_;
  int icp_max_iters_;
  float voxel_filter_size_;
  Eigen::Affine3d matrix_;

  // New Stuff

  // --------------------
  // -----Parameters-----
  // --------------------
  // SIFT Keypoint parameters
  const float min_scale = 0.01f; // the standard deviation of the smallest scale in the scale space
  const int n_octaves = 3;  // the number of octaves (i.e. doublings of scale) to compute
  const int n_scales_per_octave = 4; // the number of scales to compute within each octave
  const float min_contrast = 0.001f; // the minimum contrast required for detection

  // Sample Consensus Initial Alignment parameters (explanation below)
  const float min_sample_dist = 0.025f;
  const float max_correspondence_dist = 0.01f;
  const int nr_iters = 500;

  // ICP parameters (explanation below)
  const float max_correspondence_distance = 0.05f;
  const float outlier_rejection_threshold = 0.05f;
  const float transformation_epsilon = 0;
  const int max_iterations = 100;

  /* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by fin    ding
  * correspondences between two sets of local features
  * Inputs:
  *   source_points
  *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
  *   source_descriptors
  *     The local descriptors for each source point
  *   target_points
  *     The "target" points, i.e., the points to which the source point cloud will be aligned                     
  *   target_descriptors
  *     The local descriptors for each target point
  *   min_sample_distance
  *     The minimum distance between any two random samples
  *   max_correspondence_distance
  *     The maximum distance between a point and its nearest neighbor correspondent in order to be considered
  *     in the alignment process
  *   nr_interations
  *     The number of RANSAC iterations to perform
  * Return: A transformation matrix that will roughly align the points in source to the points in target
  */
  typedef pcl::PointWithScale PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef pcl::FPFHSignature33 LocalDescriptorT;
  typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
  Eigen::Matrix4f
  computeInitialAlignment (const PointCloudPtr & source_points, const LocalDescriptorsPtr & source_descriptors,
                          const PointCloudPtr & target_points, const LocalDescriptorsPtr & target_descriptors,
                          float min_sample_distance, float max_correspondence_distance, int nr_iterations)
  {
    pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
    sac_ia.setMinSampleDistance (min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
    sac_ia.setMaximumIterations (nr_iterations);

    sac_ia.setInputCloud (source_points);
    sac_ia.setSourceFeatures (source_descriptors);

    sac_ia.setInputTarget (target_points);
    sac_ia.setTargetFeatures (target_descriptors);

    PointCloud registration_output;
    sac_ia.align (registration_output);

    return (sac_ia.getFinalTransformation ());
  }

  /* Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,                   
  * starting with an intial guess
  * Inputs:
  *   source_points
  *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
  *   target_points
  *     The "target" points, i.e., the points to which the source point cloud will be aligned
  *   intial_alignment
  *     An initial estimate of the transformation matrix that aligns the source points to the target points
  *   max_correspondence_distance
  *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further 
  *     apart than this threshold will be ignored when computing the source-to-target transformation
  *   outlier_rejection_threshold
  *     A threshold used to define outliers during RANSAC outlier rejection
  *   transformation_epsilon
  *     The smallest iterative transformation allowed before the algorithm is considered to have converged
  *   max_iterations
  *     The maximum number of ICP iterations to perform
  * Return: A transformation matrix that will precisely align the points in source to the points in target
  */
  typedef pcl::PointXYZ ICPPointT;
  typedef pcl::PointCloud<ICPPointT> ICPPointCloud;
  typedef pcl::PointCloud<ICPPointT>::Ptr ICPPointCloudPtr;
  Eigen::Matrix4f
  refineAlignment (const ICPPointCloudPtr & source_points, const ICPPointCloudPtr & target_points,
                  const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                  float outlier_rejection_threshold, float transformation_epsilon, float max_iterations) {

    pcl::IterativeClosestPoint<ICPPointT, ICPPointT> icp;
    icp.setMaxCorrespondenceDistance (max_correspondence_distance);
    icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
    icp.setTransformationEpsilon (transformation_epsilon);
    icp.setMaximumIterations (max_iterations);

    ICPPointCloudPtr source_points_transformed (new ICPPointCloud);
    pcl::transformPointCloud (*source_points, *source_points_transformed, initial_alignment);

    icp.setInputCloud (source_points_transformed);
    icp.setInputTarget (target_points);

    ICPPointCloud registration_output;
    icp.align (registration_output);

    return (icp.getFinalTransformation () * initial_alignment);
  }


};


}  // namespace mapping
}  // namespace cg

#endif  // MAPPING__POINT_CLOUD_REGISTRATION_NODE_HPP
