#include "../include/controller_node.hpp"



using std::placeholders::_1;
using namespace std::chrono_literals;

ControllerNode::ControllerNode(const std::string model_path, const int num_observations, const int num_actions)
  :Node("Controller Node"),
  model(model_path, num_observations, num_actions), 
  last_time(this->now()),
  prev_yaw(.0f, .0f),
  prev_actions({.0f, .0f}){ 

    sub = create_subscription<sensor_msgs::msg::Imu>("imu_data", 10, 
              std::bind(&Controller::imu_callback,this, std::placeholders::_1));

    pub = create_publisher<sensor_msgs::msg::JointState>("motor_cmd", 10);
  
    // initilize control loop timer
    timer =  create_wall_timer(100ms, std::bind(&Controller::control_loop, this));
    
    imu_state = std::make_shared<sensor_msgs::msg::Imu>();
    motor_command = std::make_shared<sensor_msgs::msg::JointState();

    motor_command.name = {"left_motor","right_motor"};
    motor_command.effort = {.0f,.0f};
  }


void ControllerNode::control_loop(){

     auto& input_buffer = model.get_input_buffer();

    // get the roll
    auto x = imu_state->orientation.x;
    auto y = imu_state->orientation.y;
    auto z = imu_state->orientation.z;
    auto w = imu_state->orientation.w;
    float roll =  std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));

    

    // calculate the angular velocity 
    // multiple methods can be choosen. 
    float curr_yaw = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    float dt = (get_clock()->now() - last_time).seconds();
    float angular_vel = (curr_yaw - prev_yaw) / dt;


    //load input in the input buffer
    input_buffer[0] = roll; 
    input_buffer[1] = yaw_velocity;
    input_buffer[2] = last_actions[0];
    input_buffer[3] = last_actions[1];

    model.run();

    //display the results 
    std::ostringstream out;
    out << "Control loop :\n";
    out << "Imu Quaternion Values\t" 
        << imu_state->orientation.x << " "
        << imu_state->orientation.y << " " 
        << imu_state->orientation.z << " " 
        << imu_state->orientation.w;

    out << "\nINPUTS:\t";
    for(auto element : model.get_input_buffer()){ 
      out << element << " ";
    }
    out << "\nOUTPUTS\t";
    for(auto element : model.get_output_buffer()){
      out << element << " ";
    }
    out << std::endl;
    
    
    last_actions[0] = model.get_output_buffer().at(0);
    last_actions[1] = model.get_output_buffer().at(1);

    last_yaw = current_yaw;
    last_time = get_clock()->now();


    motor_command.header.stamp = get_clock()->now();
    motor_command.effort = {model.get_output_buffer().at(0), model.get_output_buffer().at(1)};
    pub->publish(motor_command);


    RCLCPP_INFO(get_logger(), out.str().c_str());
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
     imu_state = std::move(msg);
  }

  float quat_to_roll(float x, float y, float z, float w){
    return std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  }

  float quat_to_yaw(float x, float y, float z, float w) {
    return std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }

  // Calculate the angular velocity around yaw
  void calculate_angular_vel() {
    float current_yaw = quat_to_yaw(
      imu_state->orientation.x,
      imu_state->orientation.y,
      imu_state->orientation.z,
      imu_state->orientation.w
    );

    float dt = (get_clock()->now() - last_time).seconds();
    yaw_velocity = (current_yaw - last_yaw) / dt;

    last_yaw = current_yaw;
    last_time = get_clock()->now();

    // RCLCPP_INFO(get_logger(), "CURRENT YAW %f", current_yaw);
    // RCLCPP_INFO(get_logger(), "Last yaw %f", last_yaw);
    // RCLCPP_INFO(get_logger(), "Yaw velocity: %f rad/s", yaw_velocity);
  }
  private: 

    //ros2 interfaces 
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
    
    //ros2 messages
    sensor_msgs::msg::Imu::SharedPtr imu_state;
    sensor_msgs::msg::JointState motor_command; 
    std::array<float, 4> imu_quaternion;
    int num_observations;
    int num_actions;
    std::string model_path;
    OnnxHandler model;

    float last_yaw;
    float yaw_velocity;
    rclcpp::Time last_time;
    std::array<float,2> last_actions;

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

  char cwd[PATH_MAX];
  if(getcwd(cwd, sizeof(cwd)) != nullptr){
    std::cout << "Current Working directory is : " << cwd << std::endl;
  }else{
    perror("ERROR getting cwd");
  }

  std::string modelpath = "src/controller/Twip.pth.onnx";
  int num_observations = 4;
  int num_actions =2;
  auto controller = std::make_shared<Controller>(modelpath, num_observations,num_actions);

  set_real_time_priority(controller);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(controller->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
