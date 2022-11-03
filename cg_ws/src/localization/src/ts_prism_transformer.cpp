#include "localization/ts_prism_transformer.hpp"

using std::placeholders::_1;

namespace cg
{
  namespace total_station_rtls
  {

    TSPrismTransformer::TSPrismTransformer() : Node("ts_prism_transformer")
    {

      // Load parameters
      this->declare_parameter<int>("pub_freq", 10);
      this->get_parameter("pub_freq", pub_freq_);
      this->declare_parameter<float>("linear_translation.prism_offset_x", -0.20033);
      this->get_parameter("linear_translation.prism_offset_x", prism_offset_x_);
      this->declare_parameter<float>("linear_translation.prism_offset_y", -0.020133);
      this->get_parameter("linear_translation.prism_offset_y", prism_offset_y_);
      this->declare_parameter<float>("linear_translation.prism_offset_z", -0.75628);
      this->get_parameter("linear_translation.prism_offset_z", prism_offset_z_);

      /* Publishers and Subscribers */
      ts_prism_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/total_station_prism", pub_freq_, std::bind(&TSPrismTransformer::ts_prism_callback, this, _1));

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/imu/data/base_link", pub_freq_, std::bind(&TSPrismTransformer::imu_callback, this, _1));

      transformed_ts_prism_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/transformed_total_station_prism", 1);
      
      bearing_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/bearing", pub_freq_, std::bind(&TSPrismTransformer::bearing_callback, this, _1));

    }

    void TSPrismTransformer::bearing_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr bearing_msg) {
      got_bearing_ = true;
      latest_bearing_ = *bearing_msg;
    }

    void TSPrismTransformer::ts_prism_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr prism_msg)
    {
      updated_pose_ = *prism_msg;
      if (got_imu && got_bearing_)
      {
        // Obtain yaw angle from bearing and roll/pitch from IMU in quaternion forms
        tf2::Quaternion bearing_q(
          latest_bearing_.pose.pose.orientation.x,
          latest_bearing_.pose.pose.orientation.y,
          latest_bearing_.pose.pose.orientation.z,
          latest_bearing_.pose.pose.orientation.w);
        tf2::Quaternion imu_q(
          imu_last.orientation.x,
          imu_last.orientation.y,
          imu_last.orientation.z,
          imu_last.orientation.w);

        // Convert quaternions to roll/pitch/yaw
        tf2::Matrix3x3 bearing_m(bearing_q);
        tf2::Matrix3x3 imu_m(imu_q);
        double bearing_roll, bearing_pitch, bearing_yaw;
        double imu_roll, imu_pitch, imu_yaw;
        bearing_m.getRPY(bearing_roll, bearing_pitch, bearing_yaw);
        imu_m.getRPY(imu_roll, imu_pitch, imu_yaw);

        // Compute 3DOF orientation from map to base_link, based on raw IMU and bearing measurements
        tf2::Quaternion map_to_base_link_q;
        map_to_base_link_q.setRPY(imu_roll, imu_pitch, bearing_yaw);
        Eigen::Quaternion<double> map_to_base_link_rotation(
          map_to_base_link_q.w(),
          map_to_base_link_q.x(),
          map_to_base_link_q.y(),
          map_to_base_link_q.z()
        );

        // Get static TS prism to base_link translation from transform/parameters
        Eigen::Vector3d ts_prism_to_base_link_translation(
          ///-0.20033, -0.020133, -0.75628
          prism_offset_x_, prism_offset_y_, prism_offset_z_
        );

        // Rotate translation such that it is axis aligned with base_link
        Eigen::Vector3d rotated_ts_prism_to_base_link_translation = map_to_base_link_rotation * ts_prism_to_base_link_translation;

        // Get position of TS prism in map frame, from raw TS measurement
        Eigen::Vector3d map_to_ts_prism_translation(
          updated_pose_.pose.pose.position.x,
          updated_pose_.pose.pose.position.y,
          updated_pose_.pose.pose.position.z
        );

        // Get position of base_link in map frame by translating position of TS prism
        Eigen::Vector3d map_to_base_link_translation = map_to_ts_prism_translation + rotated_ts_prism_to_base_link_translation;
        updated_pose_.pose.pose.position.x = map_to_base_link_translation.x();
        updated_pose_.pose.pose.position.y = map_to_base_link_translation.y();
        updated_pose_.pose.pose.position.z = map_to_base_link_translation.z();
        updated_pose_.pose.pose.orientation.w = map_to_base_link_rotation.w();
        updated_pose_.pose.pose.orientation.x = map_to_base_link_rotation.x();
        updated_pose_.pose.pose.orientation.y = map_to_base_link_rotation.y();
        updated_pose_.pose.pose.orientation.z = map_to_base_link_rotation.z();

        transformed_ts_prism_pub_->publish(updated_pose_);
      } else {
        std::cout << "No imu or bearing received\n";
      }
    }

    void TSPrismTransformer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
      imu_last = *imu_msg;
      got_imu = true;
    }

  } // namespace total_station_rtls
} // namespace cg
