#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include <sensor_msgs/msg/joint_state.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node {
public:
  Controller() 
    :Node("controller"){
    // create ros2 messages 
    imu_state_ = std::make_shared<sensor_msgs::msg::Imu>();

    imu_quaternion_[3] = 1;
    RCLCPP_INFO(get_logger(), "Imu initialized");
    sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, 
              std::bind(&Controller::imu_callback,this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "1113 ROS imu subscriber succesffuly created ");

  }
  ~Controller(){};

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
     imu_state_ = std::move(msg);

    // Log the IMU state
    RCLCPP_INFO(get_logger(), "IMU values:");
    RCLCPP_INFO(get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f", 
                imu_state_->orientation.x, 
                imu_state_->orientation.y, 
                imu_state_->orientation.z, 
                imu_state_->orientation.w);
    RCLCPP_INFO(get_logger(), "Angular velocity - x: %f, y: %f, z: %f", 
                imu_state_->angular_velocity.x, 
                imu_state_->angular_velocity.y, 
                imu_state_->angular_velocity.z);
    RCLCPP_INFO(get_logger(), "Linear acceleration - x: %f, y: %f, z: %f", 
                imu_state_->linear_acceleration.x, 
                imu_state_->linear_acceleration.y, 
                imu_state_->linear_acceleration.z);

  }

  private: 

    //ros2 interfaces 
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    
    //ros2 messages
    sensor_msgs::msg::Imu::SharedPtr imu_state_;
    float *imu_quaternion_ = nullptr;
};

/*
 * function configures the current process to use the SCHED_FIFO scheduling policy with a specified
 * real-time priority. It initializes the scheduling parameters, sets the priority, and applies the 
 * scheduling policy using the sched_setscheduler system call. If the priority setting fails, an error 
 * message is logged; otherwise, an informational message with the current priority is logged.
 * 
 * Parameters:
 * - controller: A shared pointer to the Controller class, which provides the logging mechanism.
 * 
 * Returns:
 * - void
*/

void set_real_time_priority(std::shared_ptr<Controller> controller){
  // initialize the scheduling parameters
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = 98;


  //set the scheduling policy and priority for the current process 
  
  int ret =sched_setscheduler(getpid(), SCHED_FIFO, &param);
  if(ret != 0){
    RCLCPP_ERROR(controller ->get_logger(), "Failed to set the priority ");
  }else{
    std::string error = "Process priority is "  + std::to_string(param.sched_priority);
    RCLCPP_INFO(controller ->get_logger(), error.c_str());
    }
}

int main(int argc, char *argv[]) {
  std::cout << "Controller Node Started " << std::endl;

  rclcpp::init(argc, argv);
  auto controller = std::make_shared<Controller>();

  set_real_time_priority(controller);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(controller->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
