# ITHS Demo robot

Demo robot to showcase Contrain

### Purpose

To show that i can build a Robot interface using:
Javascript
rclnodejs
nodejs
ROS(python)

## Run simulation

`ros2 launch iths iths_sim_launch.py`<br/>

## Move robot head

`ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{data: [1,2,3]}' --once`

## Remove robot from Gazebo

`ros2 service call /delete_entity 'gazebo_msgs/DeleteEntity' '{name: "robot"}'`
