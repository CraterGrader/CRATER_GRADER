#include <gtest/gtest.h>
#include <motion_control/lateral_controller.hpp>

TEST(LateralControllerTest, offset_test) {

    double k = 0.1;
    double softening_constant = 0.1;

    cg::motion_control::LateralController lateral_controller(
        k,
        softening_constant);


    std::vector<cg_msgs::msg::Pose2D> pose_vector{
        cg::planning::create_pose2d(0, 1, 0),
        cg::planning::create_pose2d(0, 2, 0),
        cg::planning::create_pose2d(0, 3, 0),
    };
    std::vector<float> tool_positions{0,0,0};
    std::vector<float> velocities{1,1,1};

    cg_msgs::msg::Trajectory traj;
    traj.set__path(pose_vector);
    traj.set__tool_positions(tool_positions);
    traj.set__velocity_targets(velocities);

    nav_msgs::msg::Odometry current_state;
    current_state.pose.pose.orientation.x = 0;
    current_state.pose.pose.orientation.y = 0;
    current_state.pose.pose.orientation.z = 0; 
    current_state.pose.pose.orientation.w = 1;

    current_state.pose.pose.position.x = 1;
    current_state.pose.pose.position.y = 2;
    
    current_state.twist.twist.linear.x = 1;
    current_state.twist.twist.linear.y = 1;

    double des_steer = lateral_controller.computeSteer(
        traj, current_state);

    nav_msgs::msg::Odometry current_state_2;
    current_state_2.pose.pose.orientation.x = 0;
    current_state_2.pose.pose.orientation.y = 0;
    current_state_2.pose.pose.orientation.z = 0; 
    current_state_2.pose.pose.orientation.w = 1;
    
    current_state_2.pose.pose.position.x = -1;
    current_state_2.pose.pose.position.y = 2;
    
    current_state_2.twist.twist.linear.x = 1;
    current_state_2.twist.twist.linear.y = 1;

    double des_steer_2 = lateral_controller.computeSteer(
        traj, current_state_2);

    EXPECT_NEAR(des_steer, -des_steer_2, 1e-6f);
    EXPECT_GT(des_steer, 0);    
}


TEST(LateralControllerTest, angle_test) {

    double k = 0.1;
    double softening_constant = 0.1;

    cg::motion_control::LateralController lateral_controller(
        k,
        softening_constant);

    std::vector<cg_msgs::msg::Pose2D> pose_vector{
        cg::planning::create_pose2d(0, 1, cg::planning::deg2rad(0)),
        cg::planning::create_pose2d(1, 2, cg::planning::deg2rad(10)),
        cg::planning::create_pose2d(2, 3, cg::planning::deg2rad(20)),
    };
    std::vector<float> tool_positions{0,0,0};
    std::vector<float> velocities{1,1,1};

    cg_msgs::msg::Trajectory traj;
    traj.set__path(pose_vector);
    traj.set__tool_positions(tool_positions);
    traj.set__velocity_targets(velocities);

    nav_msgs::msg::Odometry current_state;
    current_state.pose.pose.orientation.x = 0;
    current_state.pose.pose.orientation.y = 0;
    current_state.pose.pose.orientation.z = 0; 
    current_state.pose.pose.orientation.w = 1;
    current_state.pose.pose.position.x = 1;
    current_state.pose.pose.position.y = 2;

    double des_steer = lateral_controller.computeSteer(
        traj, current_state);

    EXPECT_GT(des_steer, 0);    
}
