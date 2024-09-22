
#ifndef ATL_PASSTHROUGH__ATL_PASSTHROUGH_COMPONENT_HPP_
#define ATL_PASSTHROUGH__ATL_PASSTHROUGH_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <boost/fusion/adapted/struct.hpp>

#include <atl_msgs/msg/servo_input.hpp>
#include <atl_msgs/msg/servos_input.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <optional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace atl
{


// Component declaration
class AtlPassthroughComponent : public rclcpp::Node
{
public:
  explicit AtlPassthroughComponent(const rclcpp::NodeOptions &);
  

private:
  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subJoystick_;

  // Publishers
  rclcpp::Publisher<atl_msgs::msg::ServosInput>::SharedPtr pubServos_;

  // Callbacks
  void subJoystickCb(sensor_msgs::msg::Joy::SharedPtr && msg);
  void timerCb();
  
  std::mutex msgMtx_;


  // loop state variables
  float yawAxis_; // yaw axis
  float pitchAxis_; // pitch axis
  float rollAxis_; // roll axis

  // trim params
  float servoTrim1_ = 0; // main wing
  float servoTrim2_ = 0; // tail 1
  float servoTrim3_ = 0; // tail 2
  float servoTrim4_ = 0; // tail 3

  float lastServoTrim1_ = 0.1;
  float lastServoTrim2_ = -0.18; // tail 1
  float lastServoTrim3_ = 0.15;
  float lastServoTrim4_ = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;

};

}  // namespace atl


#endif  // ATL_PASSTHROUGH__ATL_PASSTHROUGH_COMPONENT_HPP_
