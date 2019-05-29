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
    ros::init(argc, argv, "right_end_effector_pose_reporter");
    ros::NodeHandle nh;

    // Bind signal handler
    std::signal(SIGINT, signal_handler);

    // Start async spinner to facilitate joint-state publishing
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Init Planning Interfaces
    moveit::planning_interface::MoveGroupInterface r_move_group("right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Get Joint Model Groups
    const robot_state::JointModelGroup* r_joint_model_group = r_move_group.getCurrentState()->getJointModelGroup("right_arm");

    // Print all groups in the robot
    ROS_INFO("Available Planning Groups:");
    std::copy(r_move_group.getJointNames().begin(), r_move_group.getJointNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO("Interfaces initialised.");
    ROS_INFO("R Planning frame: %s", r_move_group.getPlanningFrame().c_str());

    geometry_msgs::PoseStamped right_end_pose;

    while (ros::ok()){
      right_end_pose = r_move_group.getCurrentPose("right_gripper_link");

      ROS_INFO(" ");
      ROS_INFO("RIGHT POSE REPORT:");
      ROS_INFO("ORI w: %f", right_end_pose.pose.orientation.w);
      ROS_INFO("ORI x: %f", right_end_pose.pose.orientation.x);
      ROS_INFO("ORI y: %f", right_end_pose.pose.orientation.y);
      ROS_INFO("ORI z: %f", right_end_pose.pose.orientation.z);

      ROS_INFO("POS x: %f", right_end_pose.pose.position.x);
      ROS_INFO("POS y: %f", right_end_pose.pose.position.y);
      ROS_INFO("POS z: %f", right_end_pose.pose.position.z);

      ros::Duration(3).sleep();
    }

    return 0;
}
