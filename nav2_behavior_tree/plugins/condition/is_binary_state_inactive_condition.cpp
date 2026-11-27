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

#include <memory>
#include <string>

#include "nav2_behavior_tree/plugins/condition/is_binary_state_inactive_condition.hpp"

namespace nav2_behavior_tree
{

IsBinaryStateInactiveCondition::IsBinaryStateInactiveCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  binary_state_topic_("/binary_state"),
  min_inactive_time_(35.0),
  received_(false),
  active_(false),
  last_active_time_(0, 0, RCL_STEADY_TIME)
{
  getInput("binary_state_topic", binary_state_topic_);
  getInput("min_inactive_time", min_inactive_time_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  callback_group_executor_->add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  // BinaryFilter publishes the state as transient_local, so the latest state is received on start
  binary_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    binary_state_topic_,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(
      &IsBinaryStateInactiveCondition::binaryStateCallback, this, std::placeholders::_1),
    sub_option);
  callback_group_thread_ = std::make_unique<nav2_util::NodeThread>(callback_group_executor_);
}

BT::NodeStatus IsBinaryStateInactiveCondition::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!received_) {
    RCLCPP_INFO(
      node_->get_logger(), "IsBinaryStateInactive: no state received on %s",
      binary_state_topic_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (active_) {
    RCLCPP_INFO(node_->get_logger(), "IsBinaryStateInactive: state is true");
    return BT::NodeStatus::FAILURE;
  }
  const double inactive_time = (steady_clock_.now() - last_active_time_).seconds();
  if (inactive_time < min_inactive_time_) {
    RCLCPP_INFO(
      node_->get_logger(), "IsBinaryStateInactive: state false for only %.1f s (< %.1f s)",
      inactive_time, min_inactive_time_);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

void IsBinaryStateInactiveCondition::binaryStateCallback(std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Count the inactive time from the last true state, or from the first received state
  if (msg->data || active_ || !received_) {
    last_active_time_ = steady_clock_.now();
  }
  active_ = msg->data;
  received_ = true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsBinaryStateInactiveCondition>(
    "IsBinaryStateInactive");
}
