# REEF Controller
The REEF Controller package contains a set of simple PID controllers designed to enable attitude, velocity, and position control while integrating nicely with feedback from motion capture and the REEF Estimator node.

## Prerequisites
Requires [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)  and [ROSFlight](http://docs.rosflight.org/en/latest/user-guide/ros-setup/) to be installed. For REEF Estimator feedback, [REEF Estimator](http://192.168.1.101/AVL-Summer-18/reef_estimator) should be installed. For REEF Teleop control, [REEF Teleop](http://192.168.1.101/AVL-Summer-18/reef_teleop) should be installed.

## Installation
Simply clone **reef_controller** to the catkin workspace src directory and compile it to make sure everything works.
```
cd catkin_ws/src
git clone http://192.168.1.101/AVL-Summer-18/reef_controller
cd ../ && catkin_make
```
## Usage
REEF Controller is designed to be executed from a launchfile. **reef_controller** should be the name of the node, the package, and the type. For example,
```xml
<node name="reef_controller" pkg="reef_controller" type="reef_controller" output="screen"/>
```
For useful operation, however, several parameters should be provided, including PID gains and control modes. Before configuring a launchfile node, familiarize oneself with the parameters in the following section.

### Parameters
The internal PID controllers can be tuned by setting the gain and output clamp values in **reef_controller/params/pid.yaml**:
```xml
yaw_pid_gains: {
    p_gain_: 0.75,
    i_gain_: 0.2,
    d_gain_: 0.0,
    i_min_: -0.2,
    i_max_: 0.2,
    antiwindup_: true,
    output_min_: -0.785,
    output_max_: 0.785
}

x_pid_gains: {
    p_gain_: 0.6,
    i_gain_: 0.1,
    d_gain_: 0.0,
    i_min_: -0.25,
    i_max_: 0.25,
    antiwindup_: true,
    output_min_: -1.0,
    output_max_: 1.0
}

y_pid_gains: {
    p_gain_: 0.6,
    i_gain_: 0.1,
    d_gain_: 0.0,
    i_min_: -0.25,
    i_max_: 0.25,
    antiwindup_: true,
    output_min_: -1.0,
    output_max_: 1.0
}

z_pid_gains: {
    p_gain_: 0.75,
    i_gain_: 0.05,
    d_gain_: 0.0,
    i_min_: -0.25,
    i_max_: 0.25,
    antiwindup_: true,
    output_min_: -1.0,
    output_max_: 1.0
}

#X velocity input, pitch output
xdot_pid_gains: {
    p_gain_: 0.3,
    i_gain_: 0.025,
    d_gain_: 0.0,
    i_min_: -0.2,
    i_max_: 0.2,
    antiwindup_: true,
    output_min_: -0.3,
    output_max_: 0.3
}

#Y velocity input, roll output
ydot_pid_gains: {
    p_gain_: 0.3,
    i_gain_: 0.025,
    d_gain_: 0.0,
    i_min_: -0.2,
    i_max_: 0.2,
    antiwindup_: true,
    output_min_: -0.3,
    output_max_: 0.3
}

#Z velocity input, thrust output
zdot_pid_gains: {
    p_gain_: 0.6,
    i_gain_: 0.2,
    d_gain_: 0.0,
    i_min_: -0.6,
    i_max_: 0.6,
    antiwindup_: true,
    output_min_: 0.1,
    output_max_: 0.75
}

```

**Other parameters:**

|Name|Type|Description|Default|
|--|--|--|--|
|**control_mode**|std::string|Controller control mode setting|"attitude_altitude"|
|**enable_xy_position_controller**|bool|Enable X/Y position control override|false|
|**position_override_channel**|int|RC channel mapped to position control override|6|
|**mocap_pose_topic**|std::string|Motion capture pose topic name|mocap_ned|
|**initial_yaw_cmd**|double|Initial yaw angle for override mode|Pi|
|**initial_x_cmd**|double|Initial X position for override mode|0|
|**initial_y_cmd**|double|Initial Y position for override mode|0|
|**initial_z_cmd**|double|Initial altitude|0|


### Control Modes
 - **Attitude + Altitude**
Controls based on pitch and roll attitude, yawrate, and altitude setpoints. To enable this mode, set the **control_mode** parameter to "attitude_altitude".

 - **Velocity + Altitude**
Controls based on x and y velocity, yawrate, and altitude setpoints. To enable this mode, set the **control_mode** parameter to "velocity_altitude".
	 - **Position Control Override**
	 In the **Velocity + Altitude** control mode, X/Y/yaw position control override can be enabled if both ROSFlight RC data and motion capture feedback are available. If the **enable_xy_position_controller** parameter is set to true, the RC channel corresponding to the **position_override_channel** parameter will enable the override if its value is greater than 1500.

### ROS Topics and Messages

#### Internal Message Types
 - **ControllerState**
	 - **current**: current value of controller process variable (float64)
	 - **setpoint**: current value of controller setpoint (float64)
	 - **output**: current value of controller output (float64)
 - **PIDControllerState**
	 - **yaw**: current yaw angle PID controller state (ControllerState)
	 - **x**: current x position PID controller state (ControllerState)
	 - **y**: current y position PID controller state (ControllerState)
	 - **z**: current z position PID controller state (ControllerState)
	 - **x_dot**: current x velocity PID controller state (ControllerState)
	 - **y_dot**: current y velocity PID controller state (ControllerState)
	 - **z_dot**: current z velocity PID controller state (ControllerState)
#### Subscribed Topics
|Topic Name|Message Type|Description|
|--|--|--|
|mocap_ned|geometry_msgs::PoseStamped|Motion capture pose topic for position control|
|rc_raw|rosflight_msgs::RCRaw|ROSFlight raw RC data for position override switch|
|status|rosflight_msgs::Status|ROSFlight flight controller status messages|
|teleop_command/altitude|reef_teleop::AltitudeCommand|REEF Teleop altitude commands|
|teleop_command/attitude|reef_teleop::AttitudeCommand|REEF Teleop attitude commands|
|teleop_command/velocity|reef_teleop::VelocityCommand|REEF Teleop velocity commands|
|xyz_estimate|reef_estimator::XYZEstimate|Current state estimates for controller feedback purposes|

#### Published Topics
|Topic Name|Message Type|Description|
|--|--|--|
|controller_state|reef_controller::PIDControllerState|Current setpoints, process variables, and outputs for all PID controllers.|

### Launchfile Node Examples
Example launchfile node for Attitude + Altitude control:
```xml
<node name="reef_controller" pkg="reef_controller" type="reef_controller" output="screen">
    <rosparam file="$(find reef_controller)/params/pid.yaml" />
    <rosparam subst_value="true">
        control_mode: "attitude_altitude"
        enable_xy_position_controller: false
    </rosparam>
</node>
```
Example launchfile node for Velocity + Altitude control:
```xml
<node name="reef_controller" pkg="reef_controller" type="reef_controller" output="screen">
    <rosparam file="$(find reef_controller)/params/pid.yaml" />
    <rosparam subst_value="true">
        control_mode: "velocity_altitude"
        enable_xy_position_controller: false
    </rosparam>
</node>
```
Example launchfile node for Position Override control:
```xml
<node name="reef_controller" pkg="reef_controller" type="reef_controller" output="screen">
    <rosparam file="$(find reef_controller)/params/pid.yaml" />
    <rosparam subst_value="true">
        control_mode: "velocity_altitude"
        enable_xy_position_controller: true
        initial_x_cmd: 0.0
        initial_y_cmd: 0.0
        initial_z_cmd: -1.0
    </rosparam>
</node>
```