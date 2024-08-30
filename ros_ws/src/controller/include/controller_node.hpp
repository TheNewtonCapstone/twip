#include <sstream>
#include <unistd.h>
#include <limits.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <cmath>
#include <stdio.h>
#include <string.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <sensor_msgs/msg/joint_state.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/string.hpp"
#include "../include/onnx_handler.hpp"
#include "../include/serial.hpp"


class ControllerNode : public rclcpp::Node{
    public:
        ControllerNode(const std::string, const int, const int);
        void control_loop();
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr );

        // Serial Communication 
        int open_serial();
        int open_serial(const std::string& ); // opens serial port and return the port number 
        int read_serial();
        int write_serial();

        std::vector<double> parse_double(char &); 
        std::string parse_int(char &);
       
        private :
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
        

        int serial_file_desc; 
        bool serial_open;

        static constexpr int serial_buffer_size = 32; // Read write buffer size is 32 bytes 
};

