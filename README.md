## Blue MoveIt

### RViz Only Bringup with MoveIt!

Use this to bring up the two-arm MoveIt! stack in RViz! (Will not bring up a physical arm.)
```shell
$ roslaunch blue_moveit_bringup rviz_demo.launch
```

### Simulator Bringup In Gazebo with MoveIt!

Use this to bring up the two-arm MoveIt! stack in the Gazebo simulator! (Will not bring up a physical arm.)

This particular bringup starts a Gazebo world loaded with a table and an example pick-and-place object.

Note that pick and place will crash Gazebo if the gripper actuators touch the pick-and-place object due to how mimic joints are implemented. But aside from that all arm functionalities should work.

```shell
$ roslaunch blue_moveit_bringup gazebo_demo.launch
```

### Physical Arm with MoveIt!

Use this to bring up the physical Blue arm with the basic Blue controllers and MoveIt! integration!

Ensure your arm is powered on and the USB dongle is connected.

Note that the MoveIt! bringup starts the controllers immediately. The arm will not be in gravity compensation mode and will actively try to maintain its starting position.

You are advised to use one of the helper scripts to move the arm to a position where MoveIt! will not consider it self-colliding. Example: `$ rosrun blue_helpers right_pickup_ready_pose_commander`

**Warning:** Do not run the bringup if you've just turned on the power to the arm as this might cause the arm to violently snap to a fully zeroed position. **Wait at least 5 seconds.**

```shell
# Right Arm Setup
$ roslaunch blue_moveit_bringup right_moveit.launch param_file:=blue_params.yaml

# Left Arm Setup
$ roslaunch blue_moveit_bringup left_moveit.launch param_file:=blue_params.yaml

# Full Setup
$ roslaunch blue_moveit_bringup full_moveit.launch param_file:=blue_params.yaml
```

### Running Simulated Two-Arm Pick and Place
Note running this demo requires the blue_simulator package.

This is really easy because it's simulated. After starting the gazebo_demo, you should be able to add a MoveIt! GUI plugin interface for the RViz window that should open up to manually control the arm.

```shell
# Terminal 1
$ roslaunch blue_moveit_bringup gazebo_demo.launch

# Terminal 2
$ roslaunch blue_moveit_demos pick_and_place
```

### Running Right Arm Pick and Place Demo

Note that this particular demo does not use the table arrangement as specified in the quickstart. Instead, the arm will operate at an area opposite its starting position.

If the demo fails, the robot might be detecting that it is self-colliding. Use the manual control or rqt_joint_trajectory_controller to move it to a more favourable position.

```shell
# Terminal 1
$ roslaunch blue_moveit_bringup right_moveit.launch param_file:=blue_params.yaml

# Terminal 2
$ rosrun blue_helpers right_pickup_ready_pose_commander
$ rosrun blue_moveit_demos right_table_pick_and_place
```

## Adjusting Controller Gains

This section is especially important for the MoveIt! stack, since it uses the base ros_control `effort_controllers/JointTrajectoryController`, which has PID gains.

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


## How the MoveIt! Control Stack Works

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

When it receives a message on any of these topics, it will either cause the arm the move, or cancel the current goal (if the topic is part of an action interface.)

The message then goes into the controller manager and out to the configured controller currently switched on and handling that topic interface and relevant actuators for Blue. Then the controller calculates the control output, and, in the case of Blue, **passes it back to the controller manager which uses the hardware interface to write the instruction to Blue.**

![ros_control stack](http://wiki.ros.org/ros_control?action=AttachFile&do=get&target=gazebo_ros_control.png)

Image source: <http://wiki.ros.org/ros_control>



## Visualising the Launch File Call Tree

Because MoveIt! involves full of nested launch file calls, and it can be tedious to manually do it. Luckily, there's a [solution](<https://github.com/bponsler/roslaunch_to_dot>). This is very useful if you need to fix some bugs or change which launch files or nodes are called.

**Note:** This solution unfortunately does not visualise the parameter files that are included or called in the Blue stack.
