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
        ControllerNode(const std::string, const int, const int);

    private:
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
        
        //ros2 messages
        sensor_msgs::msg::Imu::SharedPtr imu_state;
        sensor_msgs::msg::JointState motor_cmd; 
        OnnxHandler model;

        rclcpp::Time last_time;
        rclcpp::Time last_call_back_time;

        float prev_yaw = .0f;
        std::array<float,2> prev_actions = {.0f, .0f};

        void control_loop();
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};

