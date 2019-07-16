## Blue MoveIt

### RViz-only MoveIt Demo

Use `rviz_demo.launch` to bring up the two-arm MoveIt stack in RViz:
```shell
$ roslaunch blue_moveit_bringup rviz_demo.launch
```
This does not require a physical arm.


### Gazebo MoveIt Demo

First, launch the Gazebo bringup (`full.launch` corresponds to the bringup for the full, two-arm setup; `right.launch` and `left.launch` should work as well):
```bash
roslaunch blue_gazebo full.launch
```

Then, launch the MoveIt stack:
```bash
roslaunch blue_moveit_bringup full.launch
```

Finally, run rviz:
```bash
roslaunch blue_bringup rviz.launch
```


### Physical Arm with MoveIt

Launching MoveIt for the physical arm is identical to doing so with Gazebo -- just replace the Gazebo launch command with the corresponding one from [blue_bringup](https://github.com/berkeleyopenarms/blue_core/).

Ensure your arm is powered on and the USB dongle is connected.

Note that the MoveIt bringup starts the controllers immediately. The arm will not be in gravity compensation mode and will actively try to maintain its starting position.

If needed, you can use one of these [helper scripts](https://github.com/berkeleyopenarms/blue_helpers) to move the arm to a position that MoveIt doesn't consider to be self-colliding. Example: `$ rosrun blue_helpers right_pickup_ready_pose_commander`


## Adjusting Controller Gains

This section is especially important for the MoveIt stack, since it uses the base ros_control `effort_controllers/JointTrajectoryController`, which has PID gains.

The configuration file is located at `blue_core/blue_controllers/config`.

- gazebo_controllers.yaml
  - For Gazebo simulated arm
- controllers.yaml
  - For physical arm

The base repo should provide good starting points for the gains, but for individual arms, it might be wise to tune them. So here are some pointers:

- Higher P gains should let the arm be more accurate in its poses, but might lead to oscillation or violent movement. The higher P goes, the less compliant the arm will be.
- Higher I gains allow the arm to more quickly converge into its target poses. But be aware that higher I gains will also cause the arm to potentially overshoot goals if the arm is obstructed along the way.
  - You can mitigate this by setting I clamps, though too restrictive a clamp will cause the arm to be unable to converge to its target pose.
- Higher D gains allow the arm to feel more 'rigid', reducing its compliance and reducing oscillations.
  - However, do note that too high a D can be quite catastrophic, as the ros_control controller does not mitigate against D kickbacks when the motor set point changes, which will cause the motors to jerk violently when they initially move.


## How the MoveIt Control Stack Works

Since this topic can get a little bit confusing, it pays to keep the signal flow documented.

The main point is that where ROS is concerned, the only node that matters is the `<SIDE>_arm/blue_controller_manager`. It abstracts away the hardware driver and will do the controlling for you by relaying messages to configured controllers.

It subscribes to the following topics:

**Gripper**

- /<SIDE>_arm/blue_controllers/gripper_controller/gripper_cmd/cancel
 * /<SIDE>_arm/blue_controllers/gripper_controller/gripper_cmd/goal
 * /<SIDE>_arm/blue_controllers/gripper_torque_controller/command

**Arm**

 * /<SIDE>_arm/blue_controllers/joint_trajectory_controller/command
 * /<SIDE>_arm/blue_controllers/joint_trajectory_controller/follow_joint_trajectory/cancel
 * /<SIDE>_arm/blue_controllers/joint_trajectory_controller/follow_joint_trajectory/goal

When it receives a message on any of these topics, it will either cause the arm the move, or cancel the current goal (if the topic is part of an action interface).

The message then goes into the controller manager and out to the controller that is currently switched on and handling that topic interface. Then the controller calculates the control output and, in the case of Blue, **passes it back to the controller manager which uses the hardware interface to write the instruction to Blue.**

![ros_control stack](http://wiki.ros.org/ros_control?action=AttachFile&do=get&target=gazebo_ros_control.png)

Image source: <http://wiki.ros.org/ros_control>


## Visualising the Launch File Call Tree

Because MoveIt involves many nested launch file calls, and it can be tedious to manually do it. Luckily, there's a [solution](<https://github.com/bponsler/roslaunch_to_dot>). This is very useful if you need to fix some bugs or change which launch files or nodes are called.

**Note:** This solution unfortunately does not visualise the parameter files that are included or called in the Blue stack.
