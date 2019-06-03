#include "moveit_arm_utils.h"
#include <csignal>

void signal_handler( int signal_num ) {
   // Terminate program
   ros::shutdown();
   exit(signal_num);
}

// Gripper params
const double CLOSE_POS(0.61);
const double OPEN_POS(0.1);
const double MAX_EFFORT(10.2);

// Workspace constraints
const double BASE_LINK_HEIGHT(0.965);

// Movement params
ros::Duration GOAL_TIME_TOLERANCE(15.0);
const double GOAL_TOLERANCE(0.025);
const double PRE_GRASP_APPROACH_DISTANCE(0.10);
const double PRE_GRASP_DISTANCE(0.06);
const double POST_GRASP_RETREAT_DISTANCE(0.10);

int main(int argc, char **argv)
{
    // Vars
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Init ROS
    ros::init(argc, argv, "pick_and_place_commander");
    ros::NodeHandle nh;

    // Bind signal handler
    std::signal(SIGINT, signal_handler);

    // Start async spinner to facilitate joint-state publishing
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Init Planning Interfaces
    moveit::planning_interface::MoveGroupInterface l_move_group("left_arm");
    moveit::planning_interface::MoveGroupInterface r_move_group("right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Get Joint Model Groups
    // const robot_state::JointModelGroup* l_joint_model_group = l_move_group.getCurrentState()->getJointModelGroup("left_arm");
    // const robot_state::JointModelGroup* r_joint_model_group = r_move_group.getCurrentState()->getJointModelGroup("right_arm");

    // Print all groups in the robot
    // ROS_INFO("Available Planning Groups:");
    // std::copy(l_move_group.getJointNames().begin(), l_move_group.getJointNames().end(),
    //       std::ostream_iterator<std::string>(std::cout, ", "));

    ROS_INFO("Interfaces initialised.");

    ROS_INFO("L Planning frame: %s", l_move_group.getPlanningFrame().c_str());
    ROS_INFO("R Planning frame: %s", r_move_group.getPlanningFrame().c_str());

    // Init Gripper action clients
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> right_gripper_client("right_arm/blue_controllers/gripper_controller/gripper_cmd", true);
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> left_gripper_client("left_arm/blue_controllers/gripper_controller/gripper_cmd", true);

    left_gripper_client.waitForServer();
    right_gripper_client.waitForServer();

    ROS_INFO("Gripper action clients ready!");

    // Init Arm action clients
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> right_arm_client("right_arm/blue_controllers/joint_trajectory_controller/follow_joint_trajectory", true);
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> left_arm_client("left_arm/blue_controllers/joint_trajectory_controller/follow_joint_trajectory", true);

    left_arm_client.waitForServer();
    right_arm_client.waitForServer();

    ROS_INFO("Arm action clients ready!");

    ////////////////////////////////////////////////////////////////////////////
    // POPULATE ENVIRONMENT
    ////////////////////////////////////////////////////////////////////////////

    // // Populate environment with box
    // shape_msgs::SolidPrimitive pickup_BoxShape = createBoxShape(0.07, 0.07, 0.2);
    //
    // // Note: The pose is located at the CENTER of the primitive. So adjust accordingly
    // // For URDFs, Pose is generally located at the BOTTOM of the model.
    // // Also take note that for MoveIt in this case, the fixed frame is base_link, which is different from
    // // the URDF's world_link frame.
    //
    // // Base_link height is 0.965
    // geometry_msgs::Pose box_pose;
    // box_pose.orientation.w = 1.0;
    // box_pose.position.x = 0.8;
    // box_pose.position.y = 0.4;
    // box_pose.position.z = 0.8 - BASE_LINK_HEIGHT + 0.2 / 2; // Box falls to table height at 0.8.
    //
    // addPrimitiveCollision(pickup_BoxShape, box_pose, l_move_group.getPlanningFrame(),
    //                         "pickup_box", collision_objects);

    // Populate environment with cylinder
    shape_msgs::SolidPrimitive pickup_cylinder_shape = createCylinderShape(0.025, 0.2);

    geometry_msgs::Pose cylinder_pose;
    cylinder_pose.orientation.w = 1.0;
    cylinder_pose.position.x = 0.8;
    cylinder_pose.position.y = 0.4;
    cylinder_pose.position.z = 0.8 - BASE_LINK_HEIGHT + 0.2 / 2; // Box falls to table height at 0.8.

    addPrimitiveCollision(pickup_cylinder_shape, cylinder_pose, l_move_group.getPlanningFrame(),
                            "pickup_cylinder", collision_objects);

    // Populate environment with table
    shape_msgs::SolidPrimitive table_shape = createBoxShape(1.0, 2.0, 0.075);

    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.58;
    table_pose.position.y = 0;
    table_pose.position.z = 0.8 - BASE_LINK_HEIGHT - 0.075 / 2;

    addPrimitiveCollision(table_shape, table_pose, l_move_group.getPlanningFrame(),
                            "table", collision_objects);

    // Populate planning interface
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Waiting for arm to stabilise...");
    ros::Duration(3).sleep();

    // Open grippers
    ROS_INFO("Opening grippers");
    actuateGripper(left_gripper_client, OPEN_POS, MAX_EFFORT);
    actuateGripper(right_gripper_client, OPEN_POS, MAX_EFFORT);

    ROS_INFO("MoveIt! Interfaces start states initialised!");

    // Move left arm to pick up box
    geometry_msgs::Pose box_pickup_pose;

    box_pickup_pose.orientation.w = 0.685497; // HORIzONTAL POSE IS ACTUALLY 45 degrees pitch
    box_pickup_pose.orientation.x = 0.652666;
    box_pickup_pose.orientation.y = -0.229109;
    box_pickup_pose.orientation.z = 0.227221;

    box_pickup_pose.position.x = 0.8 - PRE_GRASP_APPROACH_DISTANCE;
    box_pickup_pose.position.y = 0.4;
    box_pickup_pose.position.z = 0.8 - BASE_LINK_HEIGHT + 0.1;

    // Approach
    moveArm(left_arm_client, l_move_group, box_pickup_pose, GOAL_TIME_TOLERANCE); // Commented out to try
    actuateGripper(left_gripper_client, OPEN_POS, MAX_EFFORT);

    box_pickup_pose.position.x = 0.8 - PRE_GRASP_DISTANCE;

    // Ready for pickup
    moveArm(left_arm_client, l_move_group, box_pickup_pose, GOAL_TIME_TOLERANCE * 0.5);

    // Re-orient
    moveArm(left_arm_client, l_move_group, box_pickup_pose, GOAL_TIME_TOLERANCE);

    std::vector<std::string> object_ids;
    object_ids.push_back("pickup_box");
    object_ids.push_back("pickup_cylinder");

    planning_scene_interface.removeCollisionObjects(object_ids);

    actuateGripper(left_gripper_client, CLOSE_POS, MAX_EFFORT);

    geometry_msgs::PoseStamped left_end_pose;

    while (ros::ok){
      left_end_pose = l_move_group.getCurrentPose("left_gripper_link");

      ROS_INFO("POSE REPORT:");
      ROS_INFO("ORI w: %f", left_end_pose.pose.orientation.w);
      ROS_INFO("ORI x: %f", left_end_pose.pose.orientation.x);
      ROS_INFO("ORI y: %f", left_end_pose.pose.orientation.y);
      ROS_INFO("ORI z: %f", left_end_pose.pose.orientation.z);

      ROS_INFO("POS x: %f", left_end_pose.pose.position.x);
      ROS_INFO("POS y: %f", left_end_pose.pose.position.y);
      ROS_INFO("POS z: %f", left_end_pose.pose.position.z);

      ros::Duration(3).sleep();
    }

    // std::copy(l_move_group.getJointNames().begin(), l_move_group.getJointNames().end(),
    //       std::ostream_iterator<std::string>(std::cout, ", "));

    return 0;
}
