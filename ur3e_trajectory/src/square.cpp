#include "../include/square.hpp"

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;
    
    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    //Write your code for following the square trajectory here.
    
    
    // Create instance of pose target plan
    moveit::planning_interface::MoveGroupInterface::Plan pose_plan;
    
    geometry_msgs::Pose pose_target;
    pose_target.position.x = 0.4;
    pose_target.position.y = 0.0;
    pose_target.position.z = 1.1;
    pose_target.orientation.x = -0.707;
    pose_target.orientation.y = 0.0;
    pose_target.orientation.z = 0.0;
    pose_target.orientation.w = 0.707;

    bool pose_plan_success;
    std::string reference_frame = "base_link";
    pose_plan_success = ArmController::planToPoseTarget(planning_options, arm_move_group, pose_target, reference_frame, pose_plan);

    if(pose_plan_success){
        ROS_INFO("Moving to pose target");
        arm_move_group.execute(pose_plan);
    }

    // Create instance of cartesian plan
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;

    // Get the start Pose
    geometry_msgs::Pose start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose corner1_pose = start_pose;
    corner1_pose.position.x += 0.2;
    corner1_pose.position.y -= 0.2;
    //corner1_pose.position.z -= 0.2;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(corner1_pose);

    moveit_msgs::RobotTrajectory trajectory;
    trajectory = ArmController::planCartesianPath(start_pose, waypoints, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);

    // Get the start Pose
    start_pose = arm_move_group.getCurrentPose().pose;

    geometry_msgs::Pose corner2_pose = start_pose;
    corner2_pose.position.x -= 0.4;
    corner2_pose.position.y += 0.4;
    //corner2_pose.position.z -= 0.2;

    // Define waypoints for the cartesian path
    std::vector<geometry_msgs::Pose> waypoints2;
    waypoints2.push_back(corner2_pose);

    trajectory = ArmController::planCartesianPath(start_pose, waypoints2, reference_frame, arm_move_group);

    n.setParam("/record_pose", true);
    arm_move_group.execute(trajectory);
    n.setParam("/record_pose", false);

}