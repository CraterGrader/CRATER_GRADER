#include "localization/ts_prism_transformer.hpp"

using std::placeholders::_1;

namespace cg
{
  namespace total_station_rtls
  {

    TSPrismTransformer::TSPrismTransformer() : Node("ts_prism_transformer")
    {

      /* Publishers and Subscribers */
      ts_prism_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/total_station_prism", 10, std::bind(&TSPrismTransformer::ts_prism_callback, this, _1));

      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
          "/imu/data/base_link", 10, std::bind(&TSPrismTransformer::imu_callback, this, _1));

      transformed_ts_prism_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/transformed_total_station_prism", 1);
      
      bearing_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/bearing", 10, std::bind(&TSPrismTransformer::bearing_callback, this, _1));

      int pub_freq{10};
      tf_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(1000 / pub_freq),
          std::bind(&TSPrismTransformer::tf_Callback, this));

      // Tf listeners
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

        // TODO parametrize this hard-coded translation
        // Get static TS prism to base_link translation from transform/parameters
        Eigen::Vector3d ts_prism_to_base_link_translation(
          // ts_prism_transformStamped.transform.translation.x,
          // ts_prism_transformStamped.transform.translation.y,
          // ts_prism_transformStamped.transform.translation.z
          -0.20033, -0.020133, -0.75628
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

    // Update tf transforms
    // TODO investigate if tf_Callback and tf_Update are still needed
    void TSPrismTransformer::tf_Callback()
    {

      // Get Translation between base_link and prism in base_link_frame
      bool got_tf1 = tf_update(prism_frame, base_link_frame, ts_prism_transformStamped);

      // Get Orientation between map and base_link in map_frame
      bool got_tf2 = tf_update(map_frame, base_link_frame, base_link_transform);
      got_tf = got_tf1 && got_tf2;
    }

    // Update given tf transform
    bool TSPrismTransformer::tf_update(std::string toFrameRel, std::string fromFrameRel, geometry_msgs::msg::TransformStamped &transform)
    {
      try
      {
        transform = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);
        return true;
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return false;
      }
    }

  } // namespace total_station_rtls
} // namespace cg
