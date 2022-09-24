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

        tf2::Matrix3x3 bearing_m(bearing_q);
        tf2::Matrix3x3 imu_m(imu_q);
        double bearing_roll, bearing_pitch, bearing_yaw;
        double imu_roll, imu_pitch, imu_yaw;
        bearing_m.getRPY(bearing_roll, bearing_pitch, bearing_yaw);
        imu_m.getRPY(imu_roll, imu_pitch, imu_yaw);
        tf2::Quaternion map_to_base_link_q;
        map_to_base_link_q.setRPY(imu_roll, imu_pitch, bearing_yaw);

        // std::cout << "RPY: " << imu_roll*180/M_PI << ", " << imu_pitch*180/M_PI << ", " << bearing_yaw*180/M_PI << "\n";

        Eigen::Quaternion<double> map_to_base_link_rotation(
          map_to_base_link_q.w(),
          map_to_base_link_q.x(),
          map_to_base_link_q.y(),
          map_to_base_link_q.z()
        );

        Eigen::Vector3d ts_prism_to_base_link_translation(
          // ts_prism_transformStamped.transform.translation.x,
          // ts_prism_transformStamped.transform.translation.y,
          // ts_prism_transformStamped.transform.translation.z
          -0.1, 0.0, -0.8
        );

        Eigen::Vector3d rotated_ts_prism_to_base_link_translation = map_to_base_link_rotation * ts_prism_to_base_link_translation;

        Eigen::Vector3d map_to_ts_prism_translation(
          updated_pose_.pose.pose.position.x,
          updated_pose_.pose.pose.position.y,
          updated_pose_.pose.pose.position.z
        );

        Eigen::Vector3d map_to_base_link_translation = map_to_ts_prism_translation + rotated_ts_prism_to_base_link_translation;
        updated_pose_.pose.pose.position.x = map_to_base_link_translation.x();
        updated_pose_.pose.pose.position.y = map_to_base_link_translation.y();
        updated_pose_.pose.pose.position.z = map_to_base_link_translation.z();
        updated_pose_.pose.pose.orientation.w = map_to_base_link_rotation.w();
        updated_pose_.pose.pose.orientation.x = map_to_base_link_rotation.x();
        updated_pose_.pose.pose.orientation.y = map_to_base_link_rotation.y();
        updated_pose_.pose.pose.orientation.z = map_to_base_link_rotation.z();

        // // Zero out the translation to isolate the rotation between the map and baselink
        // base_link_transform.transform.translation.x = 0;
        // base_link_transform.transform.translation.y = 0;
        // base_link_transform.transform.translation.z = 0;
        // std::cout << "BEFORE\n";
        // std::cout << ts_prism_transformStamped.transform.translation.x << ", "
        //           << ts_prism_transformStamped.transform.translation.y << ", "
        //           << ts_prism_transformStamped.transform.translation.z << std::endl;
        // // ts_prism_transformStamped.transform.rotation.w = 1;
        // // ts_prism_transformStamped.transform.rotation.x = 0;
        // // ts_prism_transformStamped.transform.rotation.y = 0;
        // // ts_prism_transformStamped.transform.rotation.z = 0;
        // // Transform the base_link->prism translation by the map->base_link orientation
        // base_link_transform.header.frame_id = "total_station_prism";
        // std::cout << "Base_link_transform frame_id: " << base_link_transform.header.frame_id << ", child: " << base_link_transform.child_frame_id << std::endl;
        // std::cout << "ts_prism_transform frame_id: " << ts_prism_transformStamped.header.frame_id << ", child: " << ts_prism_transformStamped.child_frame_id << std::endl;
        // tf2::doTransform(ts_prism_transformStamped, ts_prism_transformStamped, base_link_transform);
        // std::cout << "AFTER\n";
        // std::cout << ts_prism_transformStamped.transform.translation.x << ", "
        //           << ts_prism_transformStamped.transform.translation.y << ", "
        //           << ts_prism_transformStamped.transform.translation.z << std::endl;

        // // Apply transformed base_link->prism translation to pose
        // updated_pose_.header.frame_id = "base_link";
        // // tf2::doTransform(updated_pose_, updated_pose_, base_link_transform);
        // tf2::doTransform(updated_pose_, updated_pose_, ts_prism_transformStamped);
        // updated_pose_.header.frame_id = "map";
        transformed_ts_prism_pub_->publish(updated_pose_);
      } else {
        std::cout << "No imu or bearing received\n";
      }
      // else if (got_imu)
      // {
      //   ts_prism_transformStamped.transform.rotation = imu_last.orientation;
      //   tf2::doTransform(updated_pose_, updated_pose_, ts_prism_transformStamped);
      //   transformed_ts_prism_pub_->publish(updated_pose_);
      // }
    }

    void TSPrismTransformer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
      imu_last = *imu_msg;
      got_imu = true;
    }

    // Update tf transforms
    void TSPrismTransformer::tf_Callback()
    {

      // Get Translation between base_link and prism in base_link_frame
      bool got_tf1 = tf_update(prism_frame, base_link_frame, ts_prism_transformStamped);
      // if (got_tf1) std::cout << "GOT TF1" << std::endl;

      // Get Orientation between map and base_link in map_frame
      bool got_tf2 = tf_update(map_frame, base_link_frame, base_link_transform);
      // if (got_tf2) std::cout << "GOT TF2" << std::endl;
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