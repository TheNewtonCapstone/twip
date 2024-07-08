#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node {
public:
  Controller() : Node("controller") {
    control_state = std::make_shared<sensor_msgs::msg::JointState>();
    imu_state = std::make_shared<sensor_msgs::msg::Imu>();

    control_state = std::make_shared<sensor_msgs::msg::JointState>();
    motor_state->position = {0,0};
    motor_state-> = {"ml", "mr"};

    
        
  }
  Controller

  private: 
    void control_loop();

    //ros2 interfaces 
    rclcpp::Subscription<sensor_msgs::msg::Imu> SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState> SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState> SharedPtr sub_;
    sensor_msgs



};
int main(int argc, char *argv[]) {
  printf("hello world controller package\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
