#include "moveit_arm_utils.h"

void actuateGripper(actionlib::SimpleActionClient<control_msgs::GripperCommandAction>& ac, double position, double max_effort)
{
    // Function for actuating grippers. Pass in the appropriate action client.
    control_msgs::GripperCommandGoal goal;

    goal.command.position = position;
    goal.command.max_effort = max_effort;

    ac.sendGoal(goal);
    ROS_INFO("Sent Gripper Command: Position %f | Max Effort %f", position, max_effort);

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
}

void moveArm(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
             moveit::planning_interface::MoveGroupInterface& move_group,
             geometry_msgs::Pose pose,
             const ros::Duration& goal_time_tolerance)
{
    control_msgs::FollowJointTrajectoryGoal goal;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group.setPoseTarget(pose);
    move_group.plan(plan);

    plan.trajectory_.joint_trajectory.joint_names.pop_back();

    goal.trajectory = plan.trajectory_.joint_trajectory;
    goal.goal_time_tolerance = goal_time_tolerance;

    ac.sendGoal(goal);
    ROS_INFO("Sent arm Command");

    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
}

void add_primitive_collision(const shape_msgs::SolidPrimitive& _shape_msg, const geometry_msgs::Pose& _pose,
                             const std::string& _frame_name, const std::string& _id,
                             std::vector<moveit_msgs::CollisionObject>& _collision_objects)
{
    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = _frame_name;
    collision_object.id = _id;
    collision_object.primitives.push_back(_shape_msg);
    collision_object.primitive_poses.push_back(_pose);
    collision_object.operation = collision_object.ADD;

    _collision_objects.push_back(collision_object);
}

shape_msgs::SolidPrimitive create_box_shape(const double& _x, const double& _y, const double& _z)
{
  shape_msgs::SolidPrimitive msg;
  msg.type = msg.BOX;
  msg.dimensions.resize(3);
  msg.dimensions[msg.BOX_X] = _x;
  msg.dimensions[msg.BOX_Y] = _y;
  msg.dimensions[msg.BOX_Z] = _z;
  return msg;
}

shape_msgs::SolidPrimitive create_cylinder_shape(const double& _r, const double& _h)
{
  shape_msgs::SolidPrimitive msg;
  msg.type = msg.CYLINDER;
  msg.dimensions.resize(2);
  msg.dimensions[msg.CYLINDER_RADIUS] = _r;
  msg.dimensions[msg.CYLINDER_HEIGHT] = _h;
  return msg;
}
