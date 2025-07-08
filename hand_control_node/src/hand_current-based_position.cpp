#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "hand_control_interfaces/msg/move_hand.hpp"
#include "hand_control_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

HandControlNode::HandControlNode() : Node("hand_control_node"){
  RCLCPP_INFO(this->get_logger(), "Run hand_current-based_postion control node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  move_hand_subscriber_ =
    this->create_subscription<MoveHand>(
    "Hand_control",
    QOS_RKL10V,
    [this](const MoveHand::SharedPtr msg) -> void
    {
      uint8_t dxl_error = 0;

        if(msg->state == 'O') // O: Hand open, C: Hand close
            goal_position = 2000;
        else if(msg->state == 'C')
            goal_position = 400;

      // Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_POSITION,
        goal_position,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        if(msg->state == 'O')
          RCLCPP_INFO(this->get_logger(), "Hand Open [ID: %d] ", msg->id);
        else if(msg->state == 'C')
          RCLCPP_INFO(this->get_logger(), "Hand Close [ID: %d] ", msg->id);
      }
    }
    );

//   auto get_present_position =
//     [this](
//     const std::shared_ptr<GetPosition::Request> request,
//     std::shared_ptr<GetPosition::Response> response) -> void
//     {
//       // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
//       // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
//       dxl_comm_result = packetHandler->read4ByteTxRx(
//         portHandler,
//         (uint8_t) request->id,
//         ADDR_PRESENT_POSITION,
//         reinterpret_cast<uint32_t *>(&present_position),
//         &dxl_error
//       );

//       RCLCPP_INFO(
//         this->get_logger(),
//         "Get [ID: %d] [Present Position: %d]",
//         request->id,
//         present_position
//       );

//       response->position = present_position;
//     };

//   get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

HandControlNode::~HandControlNode()
{
}

void setupDynamixel(uint8_t dxl_id){
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    5,  // 5: Current-based Position Control Mode
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_control_node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("hand_control_node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_control_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("hand_control_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_control_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("hand_control_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("hand_control_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("hand_control_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto handcontrolnode = std::make_shared<HandControlNode>();
  rclcpp::spin(handcontrolnode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}
