/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022 Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the <ORGANIZATION> nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Alexey Merzlyakov
 *********************************************************************/

#ifndef NAV2_COSTMAP_2D__COSTMAP_FILTERS__BINARY_FILTER_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_FILTERS__BINARY_FILTER_HPP_

#include <memory>
#include <string>

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace nav2_costmap_2d
{
/**
 * @class BinaryFilter
 * @brief Reads in a speed restriction mask and enables a robot to
 * dynamically adjust speed based on pose in map to slow in dangerous
 * areas. Done via absolute speed setting or percentage of maximum speed
 */
class BinaryFilter : public CostmapFilter
{
public:
  /**
   * @brief A constructor
   */
  BinaryFilter();

  /**
   * @brief Initialize the filter and subscribe to the info topic
   */
  void initializeFilter(
    const std::string & filter_info_topic);

  /**
   * @brief Process the keepout layer at the current pose / bounds / grid
   */
  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose2D & pose);

  /**
   * @brief Reset the costmap filter / topic / info
   */
  void resetFilter();

  /**
   * @brief Costmap reset (e.g. clear_entirely_* service called by recoveries).
   * Binary state, mask and subscriptions are kept as they are: the mask does not change
   * on costmap clearing, and resetting would flip the state to default for a moment.
   */
  void reset() override;

  /**
   * @brief If this filter is active
   */
  bool isActive();

private:
  /**
   * @brief Callback for the filter information
   */
  void filterInfoCallback(const nav2_msgs::msg::CostmapFilterInfo::SharedPtr msg);
  /**
   * @brief Callback for the filter mask
   */
  void maskCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  /**
   * @brief Changes binary state of filter. Sends a message with new state.
   * @param state New binary state
   */
  void changeState(const bool state);
  /**
   * @brief Publishes the binary state: default state while escaping, binary_state_ otherwise
   */
  void publishState();

  // ----- Escape -----
  // While the robot is in the mask area (non-default state), a static obstacle may block
  // the robot forever (e.g. obstacle layers are off and collision monitor stop zone is on).
  // Escape: publish default state for a while, so that the robot can avoid the obstacle.
  // Started when the robot is blocked for escape_stuck_time: velocity is requested on the
  // collision monitor input, but its output is zero (or no longer published).
  // Robot pose is not used for this, so odometry drift does not affect it.

  /**
   * @brief Checks stuck condition and escape end. Called from process() with mutex locked.
   */
  void updateEscape(const geometry_msgs::msg::Pose2D & pose);
  /**
   * @brief Starts escape. Must be called with mutex locked.
   */
  void startEscape(
    const geometry_msgs::msg::Pose2D & pose, const rclcpp::Time & now, const std::string & reason);
  /**
   * @brief Callback for the requested velocity (collision monitor input)
   */
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /**
   * @brief Callback for the output velocity (collision monitor output)
   */
  void cmdVelOutCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Working with filter info and mask
  rclcpp::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr binary_state_pub_;

  nav_msgs::msg::OccupancyGrid::SharedPtr filter_mask_;

  std::string mask_frame_;  // Frame where mask located in
  std::string global_frame_;  // Frame of currnet layer (master_grid)

  double base_, multiplier_;
  // Filter values higher than this threshold,
  // will set binary state to non-default
  double flip_threshold_;

  bool default_state_;  // Default Binary Filter state
  bool binary_state_;  // Current Binary Filter state (by mask)

  // Escape
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_sub_;
  double escape_stuck_time_;  // <= 0: automatic escape disabled
  double escape_distance_;
  double escape_timeout_;
  bool escaping_;
  rclcpp::Time escape_start_time_;
  geometry_msgs::msg::Pose2D escape_start_pose_;
  // Stuck (blocked) detection
  bool cmd_requested_;  // input velocity is non-zero
  rclcpp::Time last_cmd_time_;
  bool cmd_out_moving_;  // output velocity is non-zero
  rclcpp::Time last_cmd_out_time_;
  bool stuck_check_valid_;  // last_stuck_check_ is valid
  rclcpp::Time last_stuck_check_;
  double stuck_accum_;  // time with velocity requested but not passed to the robot
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_FILTERS__BINARY_FILTER_HPP_
