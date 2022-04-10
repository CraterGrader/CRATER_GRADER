#ifndef BEACON_TRANSFORMER_NODE_HPP
#define BEACON_TRANSFORMER_NODE_HPP

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
namespace uwb_beacon_rtls {

class BeaconTransformer : public rclcpp::Node {

public:
    BeaconTransformer();

private:

    /* Publishers and Subscribers */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tag_0_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr tag_1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr average_tag_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr beacon_subscription_0_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr beacon_subscription_1_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::TimerBase::SharedPtr tf_timer_{nullptr};
    rclcpp::TimerBase::SharedPtr average_tag_timer_{nullptr};

    /* Callbacks */
    void beacon_callback_0(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr beacon_msg);
    void beacon_callback_1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr beacon_msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    bool tf_update(std::string toFrameRel, std::string fromFrameRel, geometry_msgs::msg::TransformStamped &transform);
    void tf_Callback();
    void average_Beacon_Callback();

    /* Transforms */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    bool got_tf{false};

    std::string tag_0_frame = "uwb_tag_0";
    std::string tag_1_frame = "uwb_tag_1";
    std::string base_link_frame = "base_link";
    std::string map_frame = "map";

    geometry_msgs::msg::PoseWithCovarianceStamped raw_pose_0_;
    geometry_msgs::msg::PoseWithCovarianceStamped updated_pose_0_;
    bool pub_tag_0{false};
    geometry_msgs::msg::PoseWithCovarianceStamped updated_pose_1_;
    geometry_msgs::msg::PoseWithCovarianceStamped raw_pose_1_;
    bool pub_tag_1{false};
    geometry_msgs::msg::PoseWithCovarianceStamped average_pose_;
    sensor_msgs::msg::Imu imu_last;
    geometry_msgs::msg::TransformStamped tag_0_transformStamped;
    geometry_msgs::msg::TransformStamped tag_1_transformStamped;
    geometry_msgs::msg::TransformStamped base_link_transform;
    bool got_imu{false};

};


}  // namespace uwb_beacon_rtls
}  // namespace cg

#endif  // BEACON_TRANSFORMER_NODE_HPP
