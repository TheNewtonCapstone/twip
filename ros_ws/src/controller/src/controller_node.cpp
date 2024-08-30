#include "../include/controller_node.hpp"
#include <iostream>



using std::placeholders::_1, std::vector, std::bind, std::cout, std::hex, std::array;
using namespace std::chrono_literals;

ControllerNode::ControllerNode(const std::string model_path, const int num_observations, const int num_actions)
  :Node("controller"),
  model(model_path, num_observations, num_actions), 
  last_time(this->now()),
  last_call_back_time(this->now())
  { 
    

    sub = create_subscription<sensor_msgs::msg::Imu>("imu_data", 1, 
              std::bind(&ControllerNode::imu_callback,this, std::placeholders::_1));

    pub = create_publisher<sensor_msgs::msg::JointState>("motor_cmd", 1);
  
    // initilize control loop timer
    timer =  create_wall_timer(50ms, bind(&ControllerNode::control_loop, this));
    
    imu_state = std::make_shared<sensor_msgs::msg::Imu>();

    motor_cmd.name = {"left_motor","right_motor"};
    motor_cmd.effort = {.0f,.0f};

    // serial communication 
    std::string path = "/dev/ttyTHS1";
    serial_file_desc = open_serial(path);
    if(serial_file_desc != -1){ 
      serial_open = true;
    } else {
      serial_open = false;
    };
      
  }

  void ControllerNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
     float dt  =(get_clock()->now() - last_call_back_time).seconds();
     imu_state = std::move(msg);
    std::cout << "IMU callback time step : " <<  dt << std::endl;
    last_time = get_clock()->now();
    
  }


void ControllerNode::control_loop(){
    double imu_data[2];
    char read_buf[serial_buffer_size];

    int num_bytes = read(serial_file_desc, &read_buf, serial_buffer_size);
    // if the first character is correct reject message 
    if(read_buf[0] == 0x7e && num_bytes == 19){
    printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf); 
    
    // information about the packet
    char type = read_buf[1];
    int n = read_buf[2];

    // values start at the 3rd byte
    switch (type) {
    case 'd':
        for(int i = 0; i < n; i++){
          memcpy(&imu_data[i], read_buf + ((i*8)+3), sizeof(double));
          cout << imu_data[i] << "\t";
        }
        break;
    default:
        printf("Not a double\n");
        break;
    }  
    cout << "\n";
    }
    
    //if the first byte is not a starting char

    float dt = (get_clock()->now() - last_time).seconds();
    auto& input_buffer = model.get_input_buffer();
    
    // get the roll
    auto x = imu_state->orientation.x;
    auto y = imu_state->orientation.y;
    auto z = imu_state->orientation.z;
    auto w = imu_state->orientation.w;
    auto angular_vel = imu_state->angular_velocity.z;
    //float roll =  std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    float pitch = std::asin(2.f * (w * y - z * x));

    

    // calculate the instantenous angular velocity 
    // float curr_yaw = std::atan2(2 * (w * z + y * x), 1 - 2 * (y * y + z * z));
    // float angular_vel = (curr_yaw - prev_yaw) / dt;


    //load input buffer
    input_buffer[0] = pitch; 
    input_buffer[1] = angular_vel;
    input_buffer[2] = prev_actions[0];
    input_buffer[3] = prev_actions[1];

    model.run();

    //display the results 
    std::ostringstream out;
    out << "Control loop :\n";
    out << "Time step : \t" << dt << "\n";
    out << "Imu Quaternion Values\t"
        << imu_state->orientation.x << " "
        << imu_state->orientation.y << " " 
        << imu_state->orientation.z << " " 
        << imu_state->orientation.w << " "
        << angular_vel;

    out << "\nINPUTS:\t";
    for(auto element : model.get_input_buffer()){ 
      out << element << " ";
    }
    out << "\nOUTPUTS\t";
    for(auto element : model.get_output_buffer()){
      out << element << " ";
    }
    out << std::endl;
    
    
  
    prev_actions = {model.get_output_buffer().at(0),model.get_output_buffer().at(1)};
    // prev_yaw = curr_yaw;
    last_time = get_clock()->now();


    motor_cmd.header.stamp = get_clock()->now();
    motor_cmd.effort = {model.get_output_buffer().at(0), model.get_output_buffer().at(1)};
    pub->publish(motor_cmd);

    RCLCPP_INFO(get_logger(), out.str().c_str());
    std::cout << out.str() << std::endl; 
}


/*
 * function configures the current process to use the SCHED_FIFO scheduling policy with a specified
 * real-time priority. It initializes the scheduling parameters, sets the priority, and applies the 
 * scheduling policy using the sched_setscheduler system call. If the priority setting fails, an error 
 * message is logged; otherwise, an informational message with the current priority is logged.
 * 
 * Parameters:
 * - controller: A shared pointer to the ControllerNode class, which provides the logging mechanism.
 * 
 * Returns:
 * - void
*/

void set_real_time_priority(std::shared_ptr<ControllerNode> controller){
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

// open the serial pord and return the file descriptor
int ControllerNode::open_serial(const std::string& path){

   int fd = open(path.c_str(), O_RDWR);

    // Create new termios struct, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if (tcgetattr(fd, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        printf("%d", fd);
        return -1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)
    tty.c_cc[VTIME] = 50;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -1;
    }
    return fd;

}


int main(int argc, char *argv[]) {
  std::cout << "ControllerNode Node Started " << std::endl;

  rclcpp::init(argc, argv);

  char cwd[PATH_MAX];
  if(getcwd(cwd, sizeof(cwd)) != nullptr){
    std::cout << "Current Working directory is : " << cwd << std::endl;
  }else{
    perror("ERROR getting cwd");
  }


  std::string modelpath = "src/controller/onnx/twip_1.1.0_hori.onnx";
  int num_observations = 4;
  int num_actions =2;
  auto controller = std::make_shared<ControllerNode>(modelpath, num_observations,num_actions);

  set_real_time_priority(controller);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(controller->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
