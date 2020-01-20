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

#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "velocity_smoother/velocity_smoother.hpp"

int main(int argc, char** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<velocity_smoother::VelocitySmoother>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
