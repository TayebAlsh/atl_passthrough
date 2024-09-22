#include "atl_passthrough/atl_passthrough_component.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
using namespace std::chrono_literals;


#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <exception>
#include <cerrno>
#include <utility>

using std::string;

constexpr static std::size_t nServos = 5;

namespace atl
{

// ///////////////////
// ATL Passthrough Component
// ///////////////////
AtlPassthroughComponent::AtlPassthroughComponent(const rclcpp::NodeOptions & options)
: Node("atl_passthrough", options)
{

  declare_parameter("servo_trim_1", servoTrim1_); // defaults to 0.0  
  declare_parameter("servo_trim_2", servoTrim2_); // defaults to 0.0  
  declare_parameter("servo_trim_3", servoTrim3_); // defaults to 0.0  
  declare_parameter("servo_trim_4", servoTrim4_); // defaults to 0.0  

  // // Get Parameter
  get_parameter("servo_trim_1", servoTrim1_);
  get_parameter("servo_trim_2", servoTrim2_);
  get_parameter("servo_trim_3", servoTrim3_);
  get_parameter("servo_trim_4", servoTrim4_);

  // Print the trim attributes loaded with parameters
  RCLCPP_INFO(get_logger(),"Main Wing trim is %.1f", servoTrim1_);
  RCLCPP_INFO(get_logger(),"Tail Servo 1 trim is %.1f", servoTrim2_);
  RCLCPP_INFO(get_logger(),"Tail Servo 2 trim is %.1f", servoTrim3_);
  RCLCPP_INFO(get_logger(),"Tail Servo 3 trim is %.1f", servoTrim4_);

  // Create a periodic timer of 1 second
  timer_ = create_wall_timer(1000ms, std::bind(&AtlPassthroughComponent::timerCb, this));

  // Create the subscriptions
  rclcpp::SensorDataQoS inputQoS;
  inputQoS.keep_last(1);
  subJoystick_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", inputQoS,
    [this](sensor_msgs::msg::Joy::SharedPtr msg) {
      subJoystickCb(std::move(msg));
    });

  // Create the publishers
  pubServos_ = create_publisher<atl_msgs::msg::ServosInput>(
    "servos_input", rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "ATL Joystick Allocation started");

  }


// Timer function to show the changes in servo trim
  void AtlPassthroughComponent::timerCb(){

    // Store current servo trim 
    lastServoTrim1_ = servoTrim1_;
    lastServoTrim2_ = servoTrim2_;
    lastServoTrim3_ = servoTrim3_;
    lastServoTrim4_ = servoTrim4_;

    // Get the parameter again to check for changes
    get_parameter("servo_trim_1", servoTrim1_);
    get_parameter("servo_trim_2", servoTrim2_);
    get_parameter("servo_trim_3", servoTrim3_);
    get_parameter("servo_trim_4", servoTrim4_);

    // Print new parameter change
    if (servoTrim1_ != lastServoTrim1_) {
    RCLCPP_INFO(get_logger(),"NEW trim for Main Wing is %.1f", servoTrim1_);
    }
    if (servoTrim2_ != lastServoTrim2_) {
    RCLCPP_INFO(get_logger(),"NEW trim for Tail Servo 1 is %.1f", servoTrim2_);
    }
    if (servoTrim3_ != lastServoTrim3_) {
    RCLCPP_INFO(get_logger(),"NEW trim for Tail Servo 2 is %.1f", servoTrim3_);
    }
    if (servoTrim4_ != lastServoTrim4_) {
    RCLCPP_INFO(get_logger(),"NEW trim for Tail SErvo 3 is %.1f", servoTrim4_);
    }
  }

// ///////////////
// Listen to joystick and create actuation allocation
// ///////////////
void AtlPassthroughComponent::subJoystickCb(sensor_msgs::msg::Joy::SharedPtr && msg)
{
  std::lock_guard lck(msgMtx_);

  // Timestamp
  const auto tNow = now();

  // Joystick Inputs in radians
  yawAxis_ = msg-> axes[0]; // Yaw
  pitchAxis_ = msg-> axes[1]; // Pitch
  rollAxis_ = msg-> axes[2]; // Roll

  // create input messages
  atl_msgs::msg::ServoInput input1Msg; // main wing
  atl_msgs::msg::ServoInput input2Msg; // down tail
  atl_msgs::msg::ServoInput input3Msg; // left tail
  atl_msgs::msg::ServoInput input4Msg; // right tail

  // Actuator allocation
  input1Msg.header.stamp = tNow;
  input2Msg.header.stamp = tNow;
  input3Msg.header.stamp = tNow;
  input4Msg.header.stamp = tNow;

  input1Msg.delta = yawAxis_ + servoTrim1_;
  input2Msg.delta =  pitchAxis_+ servoTrim2_;
  input3Msg.delta = rollAxis_ + servoTrim3_;
  // input4Msg.delta = 1.0 * yawAxis_ + 1.0 * pitchAxis_ + servoTrim4_;

  // Publisher for servo inputs
  atl_msgs::msg::ServosInput inputsMsg;
  inputsMsg.inputs.resize(4);
  
  inputsMsg.header.stamp = tNow;
  inputsMsg.inputs[0] = input1Msg;
  inputsMsg.inputs[1] = input2Msg;
  inputsMsg.inputs[2] = input3Msg;
  inputsMsg.inputs[3] = input4Msg;

  pubServos_ -> publish(std::move(inputsMsg));

}

}  // namespace atl

RCLCPP_COMPONENTS_REGISTER_NODE(atl::AtlPassthroughComponent)
