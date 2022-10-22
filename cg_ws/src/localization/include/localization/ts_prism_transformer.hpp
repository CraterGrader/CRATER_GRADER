#ifndef PRISM_TRANSFORMER_NODE_HPP
#define PRISM_TRANSFORMER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include<math.h>

namespace cg {
namespace total_station_rtls {

class TSPrismTransformer : public rclcpp::Node {

public:
    TSPrismTransformer();

private:

    /* Publishers and Subscribers */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr transformed_ts_prism_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ts_prism_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr bearing_subscription_;
    rclcpp::TimerBase::SharedPtr tf_timer_{nullptr};

    /* Callbacks */
    void ts_prism_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr prism_msg);
    void bearing_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr bearing_msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    bool tf_update(std::string toFrameRel, std::string fromFrameRel, geometry_msgs::msg::TransformStamped &transform);
    void tf_Callback();

    /* Transforms */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool got_tf{false};

    // TODO turn these into ROS parameters
    std::string prism_frame = "total_station_prism";
    std::string base_link_frame = "base_link";
    std::string map_frame = "map";

    geometry_msgs::msg::PoseWithCovarianceStamped updated_pose_;
    geometry_msgs::msg::PoseWithCovarianceStamped latest_bearing_;
    sensor_msgs::msg::Imu imu_last;
    geometry_msgs::msg::TransformStamped ts_prism_transformStamped;
    geometry_msgs::msg::TransformStamped base_link_transform;
    bool got_imu{false};
    bool got_bearing_{false};

};


}  // namespace total_station_rtls
}  // namespace cg

#endif  // PRISM_TRANSFORMER_NODE_HPP
