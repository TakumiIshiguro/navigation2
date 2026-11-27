// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BINARY_STATE_INACTIVE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BINARY_STATE_INACTIVE_CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_thread.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that listens to the BinaryFilter state topic and
 * returns SUCCESS only when the state has been false for at least min_inactive_time.
 * It returns FAILURE when no state has been received yet or the state is true.
 */
class IsBinaryStateInactiveCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::IsBinaryStateInactiveCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsBinaryStateInactiveCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBinaryStateInactiveCondition() = delete;

  /**
   * @brief Stops the callback thread before the subscription and the state are destroyed
   */
  ~IsBinaryStateInactiveCondition() override
  {
    callback_group_thread_.reset();
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "binary_state_topic", std::string("/binary_state"), "BinaryFilter state topic"),
      BT::InputPort<double>(
        "min_inactive_time", 35.0,
        "Time [s] the state must stay false before returning SUCCESS"),
    };
  }

private:
  /**
   * @brief Callback function for the binary state topic
   * @param msg Shared pointer to std_msgs::msg::Bool message
   */
  void binaryStateCallback(std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  // Spin the callback group in its own thread so that the time of a state change is
  // recorded when it arrives, not when this node is ticked
  std::unique_ptr<nav2_util::NodeThread> callback_group_thread_;
  std::mutex mutex_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr binary_state_sub_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::string binary_state_topic_;
  double min_inactive_time_;
  bool received_;
  bool active_;
  // Time the state was last true (or first received / turned false)
  rclcpp::Time last_active_time_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BINARY_STATE_INACTIVE_CONDITION_HPP_
