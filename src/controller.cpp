#include <ros/ros.h>
#include "controller.h"

namespace controller
{
  Controller::Controller() :
    nh_(),
    nh_private_("~"),
    armed_(false),
    initialized_(false),
    is_flying_(false),
    xy_control_flag(false)
  {

    // Get Global Parameters
    nh_.param<double>("gravity", gravity_, 9.80665);
    nh_.param<double>("hover_yaw", hover_yaw_, -90*M_PI/180);

    // get robot parameters
    std::string robot_namespace;
    nh_private_.param<std::string>("robot_namespace", robot_namespace, ros::this_node::getNamespace());
    ros::NodeHandle robot_nh(robot_namespace.c_str());
    ROS_ASSERT_MSG(robot_nh.getParam("pose_controller/max_roll", max_roll_), "[rotor_controller] - missing parameters in %s namespace", robot_namespace.c_str());
    ROS_ASSERT(robot_nh.getParam("pose_controller/max_pitch", max_pitch_));
    ROS_ASSERT(robot_nh.getParam("pose_controller/max_yaw_rate", max_yaw_rate_));
    ROS_ASSERT(robot_nh.getParam("pose_controller/hover_throttle", hover_throttle_));
    ROS_ERROR("hover_throttle = %f", hover_throttle_);

    command_publisher_       = nh_.advertise<rosflight_msgs::Command>("command", 1);

    desired_state_subcriber_ = nh_.subscribe("desired_state",1,&Controller::desiredStateCallback,this);
    status_subscriber_       = nh_.subscribe("status",1,&Controller::statusCallback,this);
    is_flying_subcriber_     = nh_.subscribe("is_flying",1, &Controller::isflyingCallback,this);
    current_state_subcriber_ = nh_.subscribe("relative_state", 1, &Controller::currentStateCallback,this);
    rc_in_subcriber_         = nh_.subscribe("rc_raw",1,&Controller::RCInCallback,this);
    pose_subcriber_          = nh_.subscribe("pose_stamped", 1, &Controller::poseCallback,this);

    time_of_previous_control_ = ros::Time(0);

  }

  void Controller::desiredStateCallback(const reef_msgs::DesiredState& msg)
  {
    desired_state_ = msg;
//    desired_state_.acceleration_valid = false;
//    desired_state_.pose.z = msg.pose.z;
//
//    if(xy_control_flag)
//    {
//      desired_state_.position_valid = false;
//      desired_state_.velocity_valid = true;
//      desired_state_.velocity.x = msg.velocity.x;
//      desired_state_.velocity.y = msg.velocity.y;
//      desired_state_.velocity.yaw = msg.velocity.yaw;
//    }
//    else
//    {
//      desired_state_.position_valid = true;
//      desired_state_.velocity_valid = false;
//      desired_state_.pose.x = 0;
//      desired_state_.pose.y = 0;
//      desired_state_.pose.yaw = hover_yaw_;
//    }
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
    initialized_ = is_flying_ && armed_;
  }

  void Controller::isflyingCallback(const std_msgs::Bool &msg)
  {
    is_flying_ = msg.data;
    initialized_ = is_flying_ && armed_;
  }

  void Controller::RCInCallback(const rosflight_msgs::RCRaw &msg)
  {
    if(msg.values[6] > 1500)
    {
      ROS_WARN_ONCE("ENABLED XY CONTROL!!! ");
      xy_control_flag = 1;
    }
    else
    {
      ROS_WARN_ONCE("ENABLED Z CONTROL ONLY!!! ");
      xy_control_flag = 0;
    }
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

    command.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command.ignore = 0x00;
    command.F = std::min(std::max(thrust, 0.0), 1.0);

    if(!desired_state_.attitude_valid) {
      command.x = std::min(std::max(phi_desired, -1.0 * max_roll_), max_roll_);
      command.y = std::min(std::max(theta_desired, -1.0 * max_pitch_), max_pitch_);
      command.z = std::min(std::max(desired_state_.velocity.yaw, -1.0 * max_yaw_rate_), max_yaw_rate_);
    }
    else
    {
      command.x = desired_state_.attitude.x;
      command.y = desired_state_.attitude.y;
      command.z = desired_state_.attitude.yaw;
    }

    command_publisher_.publish(command);
  }

} //namespace
