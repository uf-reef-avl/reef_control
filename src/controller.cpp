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
    ROS_ASSERT(nh_private_.getParam("hover_throttle", hover_throttle_));
    ROS_ERROR("hover_throttle = %f", hover_throttle_);

    setpoint_attitude_pub_       = nh_.advertise<mavros_msgs::AttitudeTarget>("setpoint_raw/attitude", 1);

    desired_state_subcriber_ = nh_.subscribe("desired_state",1,&Controller::desiredStateCallback,this);
    status_subscriber_       = nh_.subscribe("status",1,&Controller::statusCallback,this);
    is_flying_subcriber_     = nh_.subscribe("is_flying",1, &Controller::isflyingCallback,this);
    current_state_subcriber_ = nh_.subscribe("xyz_estimate", 1, &Controller::currentStateCallback,this);
    rc_in_subcriber_         = nh_.subscribe("rc_raw",1,&Controller::RCInCallback,this);
    pose_subcriber_          = nh_.subscribe("pose_stamped", 1, &Controller::poseCallback,this);

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
    current_state_.pose.pose.position = msg.pose.position;
    current_state_.pose.pose.orientation = msg.pose.orientation;

  }

  void Controller::statusCallback(const mavros_msgs::State &msg)
  {
    armed_ = msg.armed;
    initialized_ = armed_;
  }

  void Controller::isflyingCallback(const std_msgs::Bool &msg)
  {
    is_flying_ = msg.data;
    initialized_ = is_flying_ && armed_;
  }

  void Controller::RCInCallback(const mavros_msgs::OverrideRCIn &msg)
  {

  }

  void Controller::computeCommand()
  {
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

    command.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;
    command.thrust = std::min(std::max(thrust, 0.0), 1.0);

    if(!desired_state_.attitude_valid && !desired_state_.altitude_only) {
      orient_out = rpyToQuat(std::min(std::max(theta_desired, -1.0 * max_pitch_), max_pitch_), 
                std::min(std::max(phi_desired, -1.0 * max_roll_), max_roll_),
                -std::min(std::max(desired_state_.velocity.yaw, -1.0 * max_yaw_rate_), max_yaw_rate_));
      command.orientation.x = orient_out.x();
      command.orientation.y = orient_out.y();
      command.orientation.z = orient_out.z();
      command.orientation.w = orient_out.w();

    }else if(desired_state_.altitude_only)
      command.type_mask = command.type_mask | mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    else
    {
      orient_out = rpyToQuat(theta_desired,
                             phi_desired,
                             0);
      command.orientation.x = orient_out.x();
      command.orientation.y = orient_out.y();
      command.orientation.z = orient_out.z();
      command.orientation.w = orient_out.w();
    }

    setpoint_attitude_pub_.publish(command);
  }

  Eigen::Quaterniond Controller::rpyToQuat(const double &roll, const double &pitch, const double &yaw)
  {
    return Eigen::Quaterniond(
    Eigen::AngleAxisd(-z, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-y, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()));
        
  }
} //namespace
