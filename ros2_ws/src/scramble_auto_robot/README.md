# CoRE Automatic Robot Hardware Package

This package contains a two ros2 nodes which control hardware components

## can_node
#### Subscriber
- `/can_node/motor0/target_current` \
  Data type: Float64 \
  Data range: 0 to 20 [A]

- `/can_node/motor1/target_current` \
  Data type: Float64 \
  Data range: 0 to 20 [A]

#### Publisher
- `/can_node/motor0/rpm` \
  Data type: Float64

- `/can_node/motor1/rpm` \
  Data type: Float64

## servo_node
#### Subscriber
- `/servo0/degree` \
  Data type: Float64 \
  Data range: 0 to 270
- `/servo1/degree` \
  Data type: Float64 \
  Data range: 0 to 270
  

## gpio_node
#### Publisher
- `/gpio_node/in0` \
  Data type: Bool
- `/gpio_node/in1` \
  Data type: Bool
- `/gpio_node/in2` \
  Data type: Bool
- `/gpio_node/in3` \
  Data type: Bool

# launch
```bash
ros2 launch scramble_auto_robot hardware.launch.py
```
