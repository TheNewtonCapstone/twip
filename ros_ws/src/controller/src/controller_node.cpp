#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <cstdio>

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Controller : public rclcpp::Node {
public:
  Controller() 
    :Node("controller"), memory_manager_("./../config.yaml")
   {
    // create ros2 messages 
    imu_state_ = std::make_shared<sensor_msgs::msg::Imu>();
    motor_state_ = std::make_shared<sensor_msgs::msg::JointState>();

    motor_state_->position = {0,0};
    motor_state_-> = {"ml", "mr"};

    imu_quaternion_[3] =1;
    RCLCPP_INFO(get_logger(), "Imu initialized");

    // get  the model 
    model_controller_ = memory_manager_.getController("Twip1").value();
    auto buffers = model_controller_ -> getBuffers();


    // Controller can execute transforms before and after control step
    std::vector<TransformRule<>> pre_transforms;
    std::vector<TransformRule<>> post_transforms;

    // Define pre-transforms (transforms needed before control step)
    // Observation 0 is pitch
    auto quaternion_to_pitch = [](float *in_ptr, float *out_ptr)
    {
        float &x = in_ptr[0];
        float &y = in_ptr[1];
        float &z = in_ptr[2];
        float &w = in_ptr[3];
        out_ptr[0] = atan2(2 * x * w + 2 * z * y, 1 - 2 * x * x - 2 * y * y);
    };
    RemapRule<> quaternion_rule(quaternion_to_pitch);
    pre_transforms.emplace_back(imu_quaternion_, buffers.first, quaternion_rule);

    // Observation 1 is last action
    RangeRemapRule<> last_action_rule(
        {-1, 1},
        {-1, 1},
        {0},
        {1},
        false);
    pre_transforms.emplace_back(buffers.second, buffers.first, last_action_rule);

    float max_velocity = 17.8;
    RangeRemapRule<> motor_command_rule(
        {-1, 1},
        {-max_velocity, max_velocity},
        {0, 0},
        {0, 1},
        false);
    post_transforms.emplace_back(buffers.second, motor_command_, motor_command_rule);

    // Add transforms to controller
    model_controller_->addTransforms(pre_transforms, post_transforms);
    float max_velocity = 17.8;
    RangeRemapRule<> motor_command_rule(
        {-1, 1},
        {-max_velocity, max_velocity},
        {0, 0},
        {0, 1},
        false);
    post_transforms.emplace_back(buffers.second, motor_command_, motor_command_rule);

    // Add transforms to controller
    model_controller_->addTransforms(pre_transforms, post_transforms);

    RCLCPP_INFO(get_logger(), "1112");

    sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, 
              std::bind(&Controller::imu_callback,this, std::placeholders::_1));

    // used to listen to the velocity published, later to be changed to subscribe to the topic of motor encoders
    sub_motor_state_ = create_subscription<sensor_msgs::msg::JointState>("motor_command", 10, 
              std::bind(&Controller::motor_state_callback,this, std::placeholders::_1));
        
    pub_ = create_publisher<sensor_msgs::msg::JointState>("motor_command",10);

    RCLCPP_INFO(get_logger(), "1113 ROS publisher subscriber succesffuly created ");

    last_time_ =std::chrono::high_resolution_clock::now();
    auto control_loop_time = 5ms;
    control_loop_timer_ =create_wall_timer(control_loop_time, 
                                          std::bind(&Controller::control_loop, this));
            
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
  void motor_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
    motor_state_ = std::move(msg);
    motor_position_[0] = motor_state->position[0];
    motor_position_[1] = motor_state->position[1];
  }


  void motor_command_publish(){
    command_.header.stamp = get_clock()->now();
    command_.velocity = {motor_command_[0], motor_command_[1]};
    pub_->publish(command_);
  }

  ControllerNode::control_loop(){
    auto now = std::chrono::high_resolution_clock::now();
    float dt - std::chrono::duration_cast<std::chrono::microseconds>(now - last_time_).count()/1e6;
    last_time = std::chrono::high_resolution_clock::now();
    model_controller_->run(dt);
    RCLCPP_INFO(get_logger(), std::to_string(motor_command_[0]))
    motor_command_publish();
  }
  private: 
    void control_loop();

    //ros2 interfaces 
    rclcpp::Subscription<sensor_msgs::msg::Imu> SharedPtr sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState> SharedPtr sub_motor_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState> SharedPtr sub_;
    
    //ros2 messages
    sensor_msgs::msg::Imu::SharedPtr imu_state_;
    float *imu_quaternion_ = nullptr;

    sensor_msgs::msg::JointState::SharedPtr motor_state_;
    float *motor_position_ = nullptr;

    sensor_msgs::msg::JointState command_;
    float *motor_command_ =nullptr;

    std::chrono::time_point<std::chrono:::high_resolution_clock> last_time_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    MemoryManager memory_manager_;

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
  mmset(&param, 0, sizeof(param));
  param.sched_priority = 98;


  //set the scheduling policy and priority for the current process 
  int ret =sched_setscheduler(get_pid(), SCHED_FIFO, &param);
  if(ret != 0){
    RCLCPP_ERROR(controller ->get_logger(), "FAiled to set process priority!");
  }else{
    RCLCPP_INFO(controller ->get_logger(), "Process priority is : " + std::to_string(param.sched_priority));
  }
}
int main(int argc, char *argv[]) {
  std::cout << "Controller Node Started " << std::endl;

  rclcpp::init(argc, argv);
  auto controller = std::make_shared>Controller>();

  set_real_time_priority(controler);

  rclcpp::executor::StaticSingleThreadedExecutor executor;
  executor.add_node(controller->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
