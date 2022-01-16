/**
 * @file src/velocity_smoother.cpp
 *
 * Copyright (c) 2012 Yujin Robot, Daniel Stonier, Jorge Santos, Marcus Liebhardt
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/velocity_smoother/license/LICENSE
 */
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "velocity_smoother/velocity_smoother.hpp"

/*****************************************************************************
 ** Preprocessing
 *****************************************************************************/

#define PERIOD_RECORD_SIZE    5

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace velocity_smoother {

/*********************
** Implementation
**********************/

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options) : rclcpp::Node("velocity_smoother", options)
, input_active_(false)
, last_velocity_cb_time_(this->get_clock()->now())
, pr_next_(0)
{
  double frequency = this->declare_parameter("frequency", 20.0);
  quiet_ = this->declare_parameter("quiet", false);
  decel_factor_ = this->declare_parameter("decel_factor", 1.0);
  int feedback = this->declare_parameter("feedback", static_cast<int>(NONE));

  if ((static_cast<int>(feedback) < NONE) || (static_cast<int>(feedback) > COMMANDS))
  {
    throw std::runtime_error("Invalid robot feedback type. Valid options are 0 (NONE, default), 1 (ODOMETRY) and 2 (COMMANDS)");
  }

  feedback_ = static_cast<RobotFeedbackType>(feedback);

  // Mandatory parameters
  rclcpp::ParameterValue speed_v = this->declare_parameter(
      "speed_lim_v",
      rclcpp::ParameterValue(0.8)
  );
  if (speed_v.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("speed_lim_v must be specified as a double");
  }
  speed_lim_v_ = speed_v.get<double>();

  rclcpp::ParameterValue speed_w = this->declare_parameter(
    "speed_lim_w",
    rclcpp::ParameterValue(5.4)
  );
  if (speed_w.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("speed_lim_w must be specified as a double");
  }
  speed_lim_w_ = speed_w.get<double>();

  rclcpp::ParameterValue accel_v = this->declare_parameter(
    "accel_lim_v",
    rclcpp::ParameterValue(0.3)
  );
  if (accel_v.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("accel_lim_v must be specified as a double");
  }
  accel_lim_v_ = accel_v.get<double>();

  rclcpp::ParameterValue accel_w = this->declare_parameter(
    "accel_lim_w",
    rclcpp::ParameterValue(3.5)
  );
  if (accel_w.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    throw std::runtime_error("accel_lim_w must be specified as a double");
  }
  accel_lim_w_ = accel_w.get<double>();

  // Deceleration can be more aggressive, if necessary
  decel_lim_v_ = decel_factor_*accel_lim_v_;
  decel_lim_w_ = decel_factor_*accel_lim_w_;

  // Publishers and subscribers
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("~/feedback/odometry", rclcpp::QoS(1), std::bind(&VelocitySmoother::odometryCB, this, std::placeholders::_1));
  current_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("~/feedback/cmd_vel", rclcpp::QoS(1), std::bind(&VelocitySmoother::robotVelCB, this, std::placeholders::_1));
  raw_in_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("~/input", rclcpp::QoS(1), std::bind(&VelocitySmoother::velocityCB, this, std::placeholders::_1));
  smooth_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("~/smoothed", 1);

  period_ = 1.0 / frequency;
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<uint64_t>(period_ * 1000.0)), std::bind(&VelocitySmoother::timerCB, this));

  param_cb_ =
    add_on_set_parameters_callback(std::bind(&VelocitySmoother::parameterUpdate, this,
      std::placeholders::_1));
}

VelocitySmoother::~VelocitySmoother()
{
}

void VelocitySmoother::velocityCB(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Estimate commands frequency; we do continuously as it can be very different depending on the
  // publisher type, and we don't want to impose extra constraints to keep this package flexible
  if (period_record_.size() < PERIOD_RECORD_SIZE)
  {
    period_record_.push_back((this->get_clock()->now() - last_velocity_cb_time_).seconds());
  }
  else
  {
    period_record_[pr_next_] = (this->get_clock()->now() - last_velocity_cb_time_).seconds();
  }

  pr_next_++;
  pr_next_ %= period_record_.size();
  last_velocity_cb_time_ = this->get_clock()->now();

  if (period_record_.size() <= PERIOD_RECORD_SIZE/2)
  {
    // wait until we have some values; make a reasonable assumption (10 Hz) meanwhile
    cb_avg_time_ = 0.1;
  }
  else
  {
    // enough; recalculate with the latest input
    cb_avg_time_ = median(period_record_);
  }

  input_active_ = true;

  // Bound speed with the maximum values
  target_vel_.linear.x  =
    msg->linear.x  > 0.0 ? std::min(msg->linear.x,  speed_lim_v_) : std::max(msg->linear.x,  -speed_lim_v_);
  target_vel_.angular.z =
    msg->angular.z > 0.0 ? std::min(msg->angular.z, speed_lim_w_) : std::max(msg->angular.z, -speed_lim_w_);
}

void VelocitySmoother::odometryCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (feedback_ == ODOMETRY)
  {
    current_vel_ = msg->twist.twist;
  }

  // ignore otherwise
}

void VelocitySmoother::robotVelCB(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (feedback_ == COMMANDS)
  {
    current_vel_ = *msg;
  }

  // ignore otherwise
}

void VelocitySmoother::timerCB()
{
  if ((input_active_ == true) && (cb_avg_time_ > 0.0) &&
      ((this->get_clock()->now() - last_velocity_cb_time_).seconds() > std::min(3.0*cb_avg_time_, 0.5)))
  {
    // Velocity input not active anymore; normally last command is a zero-velocity one, but reassure
    // this, just in case something went wrong with our input, or he just forgot good manners...
    // Issue #2, extra check in case cb_avg_time_ is very big, for example with several atomic commands
    // The cb_avg_time_ > 0 check is required to deal with low-rate simulated time, that can make that
    // several messages arrive with the same time and so lead to a zero median
    input_active_ = false;
    if (target_vel_.linear.x != 0.0 || target_vel_.angular.z != 0.0)
    {
      RCLCPP_WARN(get_logger(), "Velocity Smoother : input went inactive leaving us a non-zero target velocity (%f, %f), zeroing...",
                  target_vel_.linear.x,
                  target_vel_.angular.z);
      target_vel_ = geometry_msgs::msg::Twist();
    }
  }

  // check if the feedback is off from what we expect
  // don't care about min / max velocities here, just for rough checking
  double period_buffer = 2.0;

  double v_deviation_lower_bound = last_cmd_vel_linear_x_ - decel_lim_v_ * period_ * period_buffer;
  double v_deviation_upper_bound = last_cmd_vel_linear_x_ + accel_lim_v_ * period_ * period_buffer;

  double w_deviation_lower_bound = last_cmd_vel_angular_z_ - decel_lim_w_ * period_ * period_buffer;
  double w_deviation_upper_bound = last_cmd_vel_angular_z_ + accel_lim_w_ * period_ * period_buffer;

  bool v_different_from_feedback = current_vel_.linear.x < v_deviation_lower_bound || current_vel_.linear.x > v_deviation_upper_bound;
  bool w_different_from_feedback = current_vel_.angular.z < w_deviation_lower_bound || current_vel_.angular.z > w_deviation_upper_bound;

  if ((feedback_ != NONE) && (input_active_ == true) && (cb_avg_time_ > 0.0) &&
      (((this->get_clock()->now() - last_velocity_cb_time_).seconds() > 5.0*cb_avg_time_)     || // 5 missing msgs
          v_different_from_feedback || w_different_from_feedback))
  {
    // If the publisher has been inactive for a while, or if our current commanding differs a lot
    // from robot velocity feedback, we cannot trust the former; rely on robot's feedback instead
    // This might not work super well using the odometry if it has a high delay
    if (!quiet_) {
      // this condition can be unavoidable due to preemption of current velocity control on
      // velocity multiplexer so be quiet if we're instructed to do so
      RCLCPP_WARN(get_logger(), "Velocity Smoother : using robot velocity feedback %s instead of last command: %f, %f, %f",
                  std::string(feedback_ == ODOMETRY ? "odometry" : "end commands").c_str(),
                  (this->get_clock()->now() - last_velocity_cb_time_).seconds(),
                  current_vel_.linear.x  - last_cmd_vel_linear_x_,
                  current_vel_.angular.z - last_cmd_vel_angular_z_);
    }
    last_cmd_vel_linear_x_ = current_vel_.linear.x;
    last_cmd_vel_angular_z_ = current_vel_.angular.z;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  if ((target_vel_.linear.x  != last_cmd_vel_linear_x_) ||
      (target_vel_.angular.z != last_cmd_vel_angular_z_))
  {
    // Try to reach target velocity ensuring that we don't exceed the acceleration limits
    cmd_vel->linear = target_vel_.linear;
    cmd_vel->angular = target_vel_.angular;

    double v_inc, w_inc, max_v_inc, max_w_inc;

    v_inc = target_vel_.linear.x - last_cmd_vel_linear_x_;
    if ((feedback_ == ODOMETRY) && (current_vel_.linear.x*target_vel_.linear.x < 0.0))
    {
      // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
      max_v_inc = decel_lim_v_*period_;
    }
    else
    {
      max_v_inc = ((v_inc*target_vel_.linear.x > 0.0) ? accel_lim_v_ : decel_lim_v_)*period_;
    }

    w_inc = target_vel_.angular.z - last_cmd_vel_angular_z_;
    if ((feedback_ == ODOMETRY) && (current_vel_.angular.z*target_vel_.angular.z < 0.0))
    {
      // countermarch (on robots with significant inertia; requires odometry feedback to be detected)
      max_w_inc = decel_lim_w_*period_;
    }
    else
    {
      max_w_inc = ((w_inc*target_vel_.angular.z > 0.0) ? accel_lim_w_ : decel_lim_w_)*period_;
    }

    // Calculate and normalise vectors A (desired velocity increment) and B (maximum velocity increment),
    // where v acts as coordinate x and w as coordinate y; the sign of the angle from A to B determines
    // which velocity (v or w) must be overconstrained to keep the direction provided as command
    double MA = std::sqrt(    v_inc *     v_inc +     w_inc *     w_inc);
    double MB = std::sqrt(max_v_inc * max_v_inc + max_w_inc * max_w_inc);

    double Av = std::abs(v_inc) / MA;
    double Aw = std::abs(w_inc) / MA;
    double Bv = max_v_inc / MB;
    double Bw = max_w_inc / MB;
    double theta = std::atan2(Bw, Bv) - atan2(Aw, Av);

    if (theta < 0)
    {
      // overconstrain linear velocity
      max_v_inc = (max_w_inc*std::abs(v_inc))/std::abs(w_inc);
    }
    else
    {
      // overconstrain angular velocity
      max_w_inc = (max_v_inc*std::abs(w_inc))/std::abs(v_inc);
    }

    if (std::abs(v_inc) > max_v_inc)
    {
      // we must limit linear velocity
      cmd_vel->linear.x  = last_cmd_vel_linear_x_  + sign(v_inc)*max_v_inc;
    }

    if (std::abs(w_inc) > max_w_inc)
    {
      // we must limit angular velocity
      cmd_vel->angular.z = last_cmd_vel_angular_z_ + sign(w_inc)*max_w_inc;
    }
    last_cmd_vel_linear_x_ = cmd_vel->linear.x;
    last_cmd_vel_angular_z_ = cmd_vel->angular.z;
    smooth_vel_pub_->publish(std::move(cmd_vel));
  }
  else if (input_active_ == true)
  {
    // We already reached target velocity; just keep resending last command while input is active
    cmd_vel->linear.x = last_cmd_vel_linear_x_;
    cmd_vel->angular.z = last_cmd_vel_angular_z_;
    smooth_vel_pub_->publish(std::move(cmd_vel));
  }
}

rcl_interfaces::msg::SetParametersResult VelocitySmoother::parameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter & parameter : parameters)
  {
    if (parameter.get_name() == "frequency")
    {
      result.successful = false;
      result.reason = "frequency cannot be changed on-the-fly";
      break;
    }
    else if (parameter.get_name() == "quiet")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = "quiet must be a boolean";
        break;
      }

      quiet_ = parameter.get_value<bool>();
    }
    else if (parameter.get_name() == "decel_factor")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "decel_factor must be a double";
        break;
      }

      decel_factor_ = parameter.get_value<double>();
    }
    else if (parameter.get_name() == "feedback")
    {
      result.successful = false;
      result.reason = "feedback cannot be changed on-the-fly";
      break;
    }
    else if (parameter.get_name() == "speed_lim_v")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "speed_lim_v must be a double";
        break;
      }

      speed_lim_v_ = parameter.get_value<double>();
    }
    else if (parameter.get_name() == "speed_lim_w")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "speed_lim_w must be a double";
        break;
      }

      speed_lim_w_ = parameter.get_value<double>();
    }
    else if (parameter.get_name() == "accel_lim_v")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "accel_lim_v must be a double";
        break;
      }

      accel_lim_v_ = parameter.get_value<double>();
    }
    else if (parameter.get_name() == "accel_lim_w")
    {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason = "accel_lim_w must be a double";
        break;
      }

      accel_lim_w_ = parameter.get_value<double>();
    }
    else
    {
      result.successful = false;
      result.reason = "unknown parameter";
      break;
    }
  }

  if (result.successful)
  {
    // Since these rely on more than one parameter, update at the end in case
    // multiple change at once.
    decel_lim_v_ = decel_factor_*accel_lim_v_;
    decel_lim_w_ = decel_factor_*accel_lim_w_;
  }

  return result;
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(velocity_smoother::VelocitySmoother)
