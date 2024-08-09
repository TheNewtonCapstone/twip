#include <sstream>
#include <unistd.h>
#include <limits.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <cmath>

#include <sensor_msgs/msg/joint_state.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/string.hpp"
#include "../include/onnx_handler.hpp"


class ControllerNode : public rclcpp::Node{
    public:

        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void control_loop();
    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
        
        //ros2 messages
        sensor_msgs::msg::Imu::SharedPtr imu_state;
        sensor_msgs::msg::JointState motor_command; 

        OnnxHandler model;

        rclcpp::Time last_time;
        float prev_yaw;
        std::array<float,2> prev_actions;

}

