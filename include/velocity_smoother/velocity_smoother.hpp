/**
 * @file include/velocity_smoother/velocity_smoother.hpp
 *
 * Copyright (c) 2012 Yujin Robot, Daniel Stonier, Jorge Santos, Marcus Liebhardt
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/velocity_smoother/license/LICENSE
 */
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_
#define VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace velocity_smoother {

/*****************************************************************************
** VelocitySmoother
*****************************************************************************/

class VelocitySmoother final : public rclcpp::Node
{
public:
  explicit VelocitySmoother(const rclcpp::NodeOptions & options);

  ~VelocitySmoother() override;
  VelocitySmoother(VelocitySmoother && c) = delete;
  VelocitySmoother & operator=(VelocitySmoother && c) = delete;
  VelocitySmoother(const VelocitySmoother & c) = delete;
  VelocitySmoother & operator=(const VelocitySmoother & c) = delete;

private:
  enum RobotFeedbackType
  {
    NONE,
    ODOMETRY,
    COMMANDS
  } feedback_;  /**< What source to use as feedback for smoothed velocity calculations */

  bool quiet_;        /**< Quieten some warnings that are unavoidable because of velocity multiplexing. **/
  double speed_lim_v_, accel_lim_v_, decel_lim_v_;
  double speed_lim_w_, accel_lim_w_, decel_lim_w_;

  geometry_msgs::msg::Twist current_vel_;
  geometry_msgs::msg::Twist target_vel_;
  double last_cmd_vel_linear_x_{0.0};
  double last_cmd_vel_angular_z_{0.0};

  double period_;
  double decel_factor_;
  bool input_active_;
  double cb_avg_time_;
  rclcpp::Time last_velocity_cb_time_;
  std::vector<double> period_record_; /**< Historic of latest periods between velocity commands */
  unsigned int pr_next_; /**< Next position to fill in the periods record buffer */

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;    /**< Current velocity from odometry */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr current_vel_sub_; /**< Current velocity from commands sent to the robot, not necessarily by this node */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_in_vel_sub_;  /**< Incoming raw velocity commands */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  smooth_vel_pub_;  /**< Outgoing smoothed velocity commands */
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  void timerCB();
  void velocityCB(const geometry_msgs::msg::Twist::SharedPtr msg);
  void robotVelCB(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odometryCB(const nav_msgs::msg::Odometry::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult parameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  double sign(double x)  { return x < 0.0 ? -1.0 : +1.0; };

  double median(std::vector<double> & values) {
    // Return the median element of an doubles vector
    std::nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  }
};

} // namespace velocity_smoother

#endif /* VELOCITY_SMOOTHER__VELOCITY_SMOOTHER_HPP_ */
