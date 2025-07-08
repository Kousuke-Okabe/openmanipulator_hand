#ifndef HAND_CONTROL_NODE_HPP_
#define HAND_CONTROL_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "hand_control_interfaces/msg/move_hand.hpp"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"


class HandControlNode : public rclcpp::Node
{
public:
  using MoveHand = hand_control_interfaces::msg::MoveHand;
//   using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  HandControlNode();
  virtual ~HandControlNode();

private:
  rclcpp::Subscription<MoveHand>::SharedPtr move_hand_subscriber_;

//   rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
//   int present_position;
};

#endif  // HAND_CONTROL_NODE_HPP_
