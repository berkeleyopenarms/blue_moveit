#include "moveit_arm_utils.h"
#include <csignal>

void signal_handler( int signal_num ) {
   // Terminate program
   ros::shutdown();
   exit(signal_num);
}

int main(int argc, char **argv)
{
    // Vars
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Init ROS
    ros::init(argc, argv, "left_end_effector_pose_reporter");
    ros::NodeHandle nh;

    // Bind signal handler
    std::signal(SIGINT, signal_handler);

    // Start async spinner to facilitate joint-state publishing
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Init Planning Interfaces
    moveit::planning_interface::MoveGroupInterface l_move_group("left_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Get Joint Model Groups
    const robot_state::JointModelGroup* r_joint_model_group = l_move_group.getCurrentState()->getJointModelGroup("left_arm");

    // Print all groups in the robot
    ROS_INFO("Available Planning Groups:");
    std::copy(l_move_group.getJointNames().begin(), l_move_group.getJointNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO("Interfaces initialised.");
    ROS_INFO("L Planning frame: %s", l_move_group.getPlanningFrame().c_str());

    geometry_msgs::PoseStamped left_end_pose;

    while (ros::ok()){
      left_end_pose = l_move_group.getCurrentPose("left_gripper_link");

      ROS_INFO(" ");
      ROS_INFO("LEFT POSE REPORT:");
      ROS_INFO("ORI w: %f", left_end_pose.pose.orientation.w);
      ROS_INFO("ORI x: %f", left_end_pose.pose.orientation.x);
      ROS_INFO("ORI y: %f", left_end_pose.pose.orientation.y);
      ROS_INFO("ORI z: %f", left_end_pose.pose.orientation.z);

      ROS_INFO("POS x: %f", left_end_pose.pose.position.x);
      ROS_INFO("POS y: %f", left_end_pose.pose.position.y);
      ROS_INFO("POS z: %f", left_end_pose.pose.position.z);

      ros::Duration(3).sleep();
    }

    return 0;
}
