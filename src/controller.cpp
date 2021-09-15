#include <ros/ros.h>
#include "controller.h"

namespace reef_control
{
  Controller::Controller() :
    nh_(),
    nh_private_("~"),
    armed_(false),
    initialized_(false),
    is_flying_(false)
  {

    // Get Global Parameters
    nh_.param<double>("gravity", gravity_, 9.80665);

    ROS_ASSERT_MSG(nh_private_.getParam("max_roll", max_roll_), "[rotor_controller] - missing parameters");
    ROS_ASSERT(nh_private_.getParam("max_pitch", max_pitch_));
    ROS_ASSERT(nh_private_.getParam("max_yaw_rate", max_yaw_rate_));
    command_publisher_       = nh_.advertise<rosflight_msgs::Command>("command", 1);

    //Attitute publisher for mavros
    attitude_publisher_ = nh_.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);

    desired_state_subcriber_ = nh_.subscribe("desired_state",1,&Controller::desiredStateCallback,this);
    status_subscriber_       = nh_.subscribe("status",1,&Controller::statusCallback,this);
    is_flying_subcriber_     = nh_.subscribe("is_flying",1, &Controller::isflyingCallback,this);
    current_state_subcriber_ = nh_.subscribe("xyz_estimate", 1, &Controller::currentStateCallback,this);
    rc_in_subcriber_         = nh_.subscribe("rc_raw",1,&Controller::RCInCallback,this);
    pose_subcriber_          = nh_.subscribe("pose_stamped", 1, &Controller::poseCallback,this);
    mavors_state_subscriber_ = nh_.subscribe("mavros/state", 1, &Controller::stateCallback, this);
    time_of_previous_control_ = ros::Time(0);

  }

  void Controller::desiredStateCallback(const reef_msgs::DesiredState& msg)
  {
    desired_state_ = msg;
  }

  void Controller::currentStateCallback(const reef_msgs::XYZEstimate& msg)
  {
    current_state_.header = msg.header;
    current_state_.twist.twist.linear.x = msg.xy_plus.x_dot;
    current_state_.twist.twist.linear.y = msg.xy_plus.y_dot;
    current_state_.twist.twist.linear.z = msg.z_plus.z_dot;
    current_state_.pose.pose.position.z = msg.z_plus.z;
    computeCommand();
  }

  void Controller::poseCallback(const geometry_msgs::PoseStamped& msg)
  {
    current_state_.pose.pose.position.x = msg.pose.position.x;
    current_state_.pose.pose.position.y = msg.pose.position.y;
    current_state_.pose.pose.orientation = msg.pose.orientation;

  }

  void Controller::statusCallback(const rosflight_msgs::Status &msg)
  {
    armed_ = msg.armed;
    initialized_ = armed_;
  }

  void Controller::stateCallback(const mavros_msgs::State &msg){

    initialized_ = msg.armed;
  }

  void Controller::isflyingCallback(const std_msgs::Bool &msg)
  {
    is_flying_ = msg.data;
    initialized_ = is_flying_ && armed_;
  }

  void Controller::RCInCallback(const rosflight_msgs::RCRaw &msg)
  {

  }

  void Controller::computeCommand()
  {

    // MAVROS additions by Adam, 8 Jul 2021
    //
    mavros_msgs::AttitudeTarget att_target;
    att_target.header.stamp = current_state_.header.stamp;
    att_target.type_mask = 3; //Bitmask set to use position vs rate

    geometry_msgs::PoseStamped p;
    double x = current_state_.pose.pose.orientation.x; //Current state is a class variable.
    double y = current_state_.pose.pose.orientation.y;
    double z = current_state_.pose.pose.orientation.z;


    tf::Quaternion q;

    // Time calculation
    dt = (current_state_.header.stamp - time_of_previous_control_).toSec();
    time_of_previous_control_ = current_state_.header.stamp;
    if(dt <= 0.0000001)
    {
      // Don't do anything if dt is really close (or equal to) zero
      return;
    }

    computeCommand(current_state_ ,desired_state_,dt);

    phi_desired = desired_state_.acceleration.y;
    theta_desired = -desired_state_.acceleration.x;
    thrust = -desired_state_.acceleration.z;

    /*
    accel_out = Eigen::Vector3d(desired_state_.acceleration.x, desired_state_.acceleration.y, desired_state_.acceleration.z );
    total_accel = sqrt( pow(accel_out.x(),2) + pow(accel_out.y(),2) + pow((1 - accel_out.z()),2) );
    thrust = total_accel * hover_throttle_ ;

    if(thrust > 0.001)
    {
      phi_desired = asin(accel_out.y() / total_accel);
      theta_desired = -1.0 * asin(accel_out.x() / total_accel);
    }
    else
    {
        phi_desired = 0;
        theta_desired = 0;
    }
    */
    command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command.F = std::min(std::max(thrust, 0.0), 1.0);
    if(!desired_state_.attitude_valid && !desired_state_.altitude_only) {
      // ROS_WARN_STREAM("Coming into the right mode");
      command.ignore = 0x00;
      //Roll command
      command.x = std::min(std::max(phi_desired, -1.0 * max_roll_), max_roll_);
      //Pitch command
      command.y = std::min(std::max(theta_desired, -1.0 * max_pitch_), max_pitch_);
      //Yaw-rate command
      command.z = std::min(std::max(desired_state_.velocity.yaw, -1.0 * max_yaw_rate_), max_yaw_rate_);

        //Additions by Adam for MAVROS
        q.setRPY(command.x, -command.y, 0);
        att_target.orientation.x = q.getX();
        att_target.orientation.y = q.getY();
        att_target.orientation.z = q.getZ();
        att_target.orientation.w = q.getW();
        att_target.body_rate.z = -command.z;
        att_target.thrust = command.F;

        command.x*=57.3;
        command.y*=57.3;
        command.z*=57.3;
       ROS_WARN_STREAM("\nROLL Command \t" << command.x << "\nPITCH Command \t" << command.y << "\nYaw Command \t" << command.z
        << "\nThrust Command \t" << command.F);
  
    }   

    command_publisher_.publish(command);
    //
    //Publish to attitude target
    // ROS_INFO("Att publishing!");
    attitude_publisher_.publish(att_target);
  }

} //namespace
