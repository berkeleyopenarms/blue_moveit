#include "moveit_arm_utils.h"
#include <csignal>

void signal_handler( int signal_num ) {
   // Terminate program
   ros::shutdown();
   exit(signal_num);
}

// Gripper params
const double CLOSE_POS(1.0);
const double OPEN_POS(0.0);
const double MAX_EFFORT(10.0);

// Movement params
ros::Duration GOAL_TIME_TOLERANCE(10.0);
const double GOAL_TOLERANCE(0.2);
const double PRE_GRASP_APPROACH_DISTANCE(0.10);
const double PRE_GRASP_DISTANCE(0.06);
const double POST_GRASP_RETREAT_DISTANCE(0.10);

// Planning Context params
const double BASE_LINK_HEIGHT(0.04);
const double X_OFFSET(0.02);
const double Y_OFFSET(-0.06);
const double CYLINDER_HEIGHT(0.20);
const double CYLINDER_RADIUS(0.0325);

int main(int argc, char **argv)
{
    // Vars
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Init ROS
    ros::init(argc, argv, "right_table_pick_and_place_commander");
    ros::NodeHandle nh;

    // Bind signal handler
    std::signal(SIGINT, signal_handler);

    // Start async spinner to facilitate joint-state publishing
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Init Planning Interfaces
    moveit::planning_interface::MoveGroupInterface r_move_group("right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Interfaces initialised.");
    ROS_INFO("R Planning frame: %s", r_move_group.getPlanningFrame().c_str());

    // Init Gripper action clients
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> right_gripper_client("right_arm/blue_controllers/gripper_controller/gripper_cmd", true);
    right_gripper_client.waitForServer();
    ROS_INFO("Gripper action clients ready!");

    // Init Arm action clients
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> right_arm_client("right_arm/blue_controllers/joint_trajectory_controller/follow_joint_trajectory", true);
    right_arm_client.waitForServer();
    ROS_INFO("Arm action clients ready!");

    ////////////////////////////////////////////////////////////////////////////
    // POPULATE ENVIRONMENT
    ////////////////////////////////////////////////////////////////////////////

    // Populate environment with box
    shape_msgs::SolidPrimitive table_shape = createBoxShape(2.0, 2.0, 0.04);

    // Note: The pose is located at the CENTER of the primitive. So adjust accordingly
    // For URDFs, Pose is generally located at the BOTTOM of the model.
    // Also take note that for MoveIt in this case, the fixed frame is base_link, which is different from
    // the URDF's world_link frame.

    // Construct table
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.0;
    table_pose.position.y = 0.0;
    table_pose.position.z = -BASE_LINK_HEIGHT;

    addPrimitiveCollision(table_shape, table_pose, r_move_group.getPlanningFrame(),
                            "table", collision_objects);

    // Populate environment with can
    // shape_msgs::SolidPrimitive pickup_cylinder_shape = createCylinderShape(0.035, 0.25);
    shape_msgs::SolidPrimitive pickup_cylinder_shape = createCylinderShape(CYLINDER_RADIUS, CYLINDER_HEIGHT);

    geometry_msgs::Pose r_pickup_cylinder_pose;
    r_pickup_cylinder_pose.orientation.w = 1.0;
    r_pickup_cylinder_pose.position.x = -0.465 + X_OFFSET;
    r_pickup_cylinder_pose.position.y = 0.20 + Y_OFFSET;
    r_pickup_cylinder_pose.position.z = CYLINDER_HEIGHT / 2 - BASE_LINK_HEIGHT;

    geometry_msgs::Pose l_pickup_cylinder_pose;
    l_pickup_cylinder_pose.orientation.w = 1.0;
    l_pickup_cylinder_pose.position.x = -0.465 + X_OFFSET;
    l_pickup_cylinder_pose.position.y = -0.06 + Y_OFFSET;
    l_pickup_cylinder_pose.position.z = CYLINDER_HEIGHT / 2 - BASE_LINK_HEIGHT;

    // Populate planning interface
    planning_scene_interface.addCollisionObjects(collision_objects);

    ROS_INFO("Waiting for arm to stabilise...");
    ros::Duration(3).sleep();

    // Open grippers
    ROS_INFO("Opening grippers");
    actuateGripper(right_gripper_client, OPEN_POS, MAX_EFFORT);
    actuateGripper(right_gripper_client, OPEN_POS, 0.0);

    ROS_INFO("MoveIt! Interfaces start states initialised!");

    // Move left arm to pick up box
    geometry_msgs::Pose pickup_ready_pose = createPoseMsg(0.203699, -0.224751, 0.615237, 0.727652,
                                                                 -0.311129, -0.012285, 0.464498);

    geometry_msgs::Pose l_vertical_pickup_pose = createPoseMsg(0.150959, -0.363080, 0.565928, 0.724645,
                                                                 -0.470472 + X_OFFSET, -0.059734 + Y_OFFSET, CYLINDER_HEIGHT - BASE_LINK_HEIGHT + 0.04);
    geometry_msgs::Pose r_vertical_pickup_pose = createPoseMsg(0.321572, -0.219754, 0.626336, 0.675280,
                                                                 -0.465372 + X_OFFSET, 0.202503 + Y_OFFSET, CYLINDER_HEIGHT - BASE_LINK_HEIGHT + 0.04);

    geometry_msgs::Pose l_horizontal_pickup_pose = createPoseMsg(0.140651, -0.438133, -0.492451, 0.738748, //0.007847, -0.655444, -0.321642, 0.683285,
                                                                   -0.475119 + X_OFFSET, -0.077985 + Y_OFFSET, CYLINDER_HEIGHT / 2 + 0.05);
    geometry_msgs::Pose r_horizontal_pickup_pose = createPoseMsg(0.496856, -0.007063, 0.827511, -0.261362,
                                                                   -0.440480 + X_OFFSET, 0.227093 + Y_OFFSET, CYLINDER_HEIGHT / 2);

    geometry_msgs::Pose l_vertical_up_pickup_pose = l_vertical_pickup_pose;
    geometry_msgs::Pose r_vertical_up_pickup_pose = r_vertical_pickup_pose;
    l_vertical_up_pickup_pose.position.z += 0.15;
    r_vertical_up_pickup_pose.position.z += 0.15;

    geometry_msgs::Pose l_horizontal_left_pickup_pose = l_horizontal_pickup_pose;
    geometry_msgs::Pose r_horizontal_right_pickup_pose = r_horizontal_pickup_pose;
    geometry_msgs::Pose r_horizontal_right_up_pickup_pose = r_horizontal_pickup_pose;
    geometry_msgs::Pose l_horizontal_left_up_pickup_pose = l_horizontal_pickup_pose;

    l_horizontal_left_pickup_pose.position.y -= 0.05;
    l_horizontal_left_up_pickup_pose.position.y -= 0.05;
    l_horizontal_left_up_pickup_pose.position.z += 0.15;

    r_horizontal_right_pickup_pose.position.y += 0.05;
    r_horizontal_right_up_pickup_pose.position.y += 0.05;
    r_horizontal_right_up_pickup_pose.position.z += 0.15;

    std::vector<std::string> object_ids;
    object_ids.push_back("pickup_cylinder");

    while (ros::ok){
      // LEFT VERTICAL PICK
      addPrimitiveCollision(pickup_cylinder_shape, l_pickup_cylinder_pose, r_move_group.getPlanningFrame(),
                              "pickup_cylinder", collision_objects);
      planning_scene_interface.addCollisionObjects(collision_objects);

      moveArm(right_arm_client, r_move_group, l_vertical_pickup_pose, GOAL_TIME_TOLERANCE);
      planning_scene_interface.removeCollisionObjects(object_ids);
      actuateGripper(right_gripper_client, CLOSE_POS, MAX_EFFORT);

      // CLEAR
      moveArm(right_arm_client, r_move_group, l_vertical_up_pickup_pose, GOAL_TIME_TOLERANCE);

      // RIGHT VERTICAL PLACE
      moveArm(right_arm_client, r_move_group, r_vertical_up_pickup_pose, GOAL_TIME_TOLERANCE);
      moveArm(right_arm_client, r_move_group, r_vertical_pickup_pose, GOAL_TIME_TOLERANCE);

      // CLEAR
      actuateGripper(right_gripper_client, OPEN_POS, MAX_EFFORT);
      moveArm(right_arm_client, r_move_group, r_vertical_up_pickup_pose, GOAL_TIME_TOLERANCE);

      addPrimitiveCollision(pickup_cylinder_shape, r_pickup_cylinder_pose, r_move_group.getPlanningFrame(),
      "pickup_cylinder", collision_objects);
      planning_scene_interface.addCollisionObjects(collision_objects);
      actuateGripper(right_gripper_client, OPEN_POS, 0.0);

      // RIGHT HORIZONTAL PICK
      // moveArm(right_arm_client, r_move_group, r_horizontal_right_pickup_pose, GOAL_TIME_TOLERANCE);
      moveArm(right_arm_client, r_move_group, r_horizontal_pickup_pose, GOAL_TIME_TOLERANCE);

      planning_scene_interface.removeCollisionObjects(object_ids);
      actuateGripper(right_gripper_client, CLOSE_POS, MAX_EFFORT);
      moveArm(right_arm_client, r_move_group, r_horizontal_right_up_pickup_pose, GOAL_TIME_TOLERANCE);

      // LEFT HORIZONTAL PLACE
      moveArm(right_arm_client, r_move_group, l_horizontal_left_up_pickup_pose, GOAL_TIME_TOLERANCE);
      moveArm(right_arm_client, r_move_group, l_horizontal_pickup_pose, GOAL_TIME_TOLERANCE);

      actuateGripper(right_gripper_client, OPEN_POS, MAX_EFFORT);
      actuateGripper(right_gripper_client, OPEN_POS, 0.0);

      moveArm(right_arm_client, r_move_group, l_horizontal_left_up_pickup_pose, GOAL_TIME_TOLERANCE);

      addPrimitiveCollision(pickup_cylinder_shape, l_pickup_cylinder_pose, r_move_group.getPlanningFrame(),
                              "pickup_cylinder", collision_objects);
      planning_scene_interface.addCollisionObjects(collision_objects);

      ros::Duration(1).sleep();

      moveArm(right_arm_client, r_move_group, pickup_ready_pose, GOAL_TIME_TOLERANCE);
      ros::Duration(5).sleep();
    }

    return 0;
}
