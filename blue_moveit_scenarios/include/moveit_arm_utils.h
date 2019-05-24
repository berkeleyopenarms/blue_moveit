#ifndef MOVEIT_ARM_UTILS
#define MOVEIT_ARM_UTILS

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "geometric_shapes/shapes.h"

void actuateGripper(actionlib::SimpleActionClient<control_msgs::GripperCommandAction>& ac, double position, double max_effort);

void moveArm(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& ac,
             moveit::planning_interface::MoveGroupInterface& move_group,
             geometry_msgs::Pose pose,
             const ros::Duration& goal_time_tolerance);

void add_primitive_collision(const shape_msgs::SolidPrimitive& _shape_msg, const geometry_msgs::Pose& _pose,
                             const std::string& _frame_name, const std::string& _id,
                             std::vector<moveit_msgs::CollisionObject>& _collision_objects);

shape_msgs::SolidPrimitive create_box_shape(const double& _x, const double& _y, const double& _z);
shape_msgs::SolidPrimitive create_cylinder_shape(const double& _r, const double& _h);

#endif
