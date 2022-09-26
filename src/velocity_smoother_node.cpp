/**
 * @file src/velocity_smoother_node.cpp
 *
 * Copyright (c) 2012 Yujin Robot, Daniel Stonier, Jorge Santos, Marcus Liebhardt
 *
 * License: BSD
 *   https://raw.githubusercontent.com/kobuki-base/velocity_smoother/license/LICENSE
 */
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "kobuki_velocity_smoother/velocity_smoother.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<kobuki_velocity_smoother::VelocitySmoother>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
