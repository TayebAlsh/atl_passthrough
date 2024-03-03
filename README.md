# atl_passthrough

Bridge between ROS2 Joystick and desired servo input signals

## Published topics

- `input` of type `atl_msgs/ServoInput` : Inputs to the servos

## Subscribed topics

- `joy` of type `sensor_msgs/joy` : Inputs from wireless joystick

## Parameters


## Behavior

This node subscribes to the joystick messages, converts them using an actuation allocation matrix, and publishes the desired input signal to each servo in radians.

