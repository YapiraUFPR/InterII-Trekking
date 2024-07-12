#!/bin/bash

timeout 10s ros2 topic pub -r 10 /flare std_msgs/msg/Bool "{data: true}"
timeout 10s ros2 topic pub -r 10 /flare std_msgs/msg/Bool "{data: false}" 
