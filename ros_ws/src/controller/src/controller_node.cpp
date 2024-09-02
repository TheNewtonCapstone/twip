#include "../include/controller_node.hpp"
#include <iostream>



using std::placeholders::_1, std::vector, std::bind, std::cout, std::hex, std::array;
using namespace std::chrono_literals;

ControllerNode::ControllerNode(const std::string model_path, const int num_observations, const int num_actions)
  :Node("controller"),
  model(model_path, num_observations, num_actions),
  last_time(this->now()),
  last_call_back_time(this->now()) {

  // initilize control loop timer
  timer = create_wall_timer(50ms, bind(&ControllerNode::control_loop, this));

  // serial communication 
  std::string path = "/dev/ttyTHS1";
  serial_file_desc = open_serial(path);
  if (serial_file_desc == -1) {
    serial_open = false;
  }
  serial_open = true;
}


void ControllerNode::control_loop() {
  float dt = (get_clock()->now() - last_time).seconds();

  if (!serial_open) {
    std::cout << "Serial port is not availaible\n";
    return;
  }
  // get imu data
  int read_bytes = read_serial(this->imu_data);
  if (read_bytes == -1) return;
  auto& input_buffer = model.get_input_buffer();

  //load input buffer
  input_buffer[0] = imu_data[0];
  input_buffer[1] = imu_data[1];
  input_buffer[2] = prev_actions[0];
  input_buffer[3] = prev_actions[1];

  model.run();
  // publish motor
  int send_bytes = write_serial(this->motor_cmd);


  //display the results 
  std::ostringstream out;
  out << "Control loop :\n";
  out << "Time step : \t" << dt << "\n";
  out << "Data read : \t " << read_bytes << "\n";
  out << "Roll : \t" << imu_data[0] << "\n";
  out << "Angular velocity : \t" << imu_data[1] << "\n";
  out << "\nINPUTS:\t";
  for (auto element : model.get_input_buffer()) {
    out << element << " ";
  }
  out << "\nOUTPUTS\t";
  for (auto element : model.get_output_buffer()) {
    out << element << " ";
  }
  out << "\nData sent : \t " << send_bytes << "\n";
  out << std::endl;



  prev_actions = { model.get_output_buffer().at(0),model.get_output_buffer().at(1) };
  last_time = get_clock()->now();

  // motor_cmd.header.stamp = get_clock()->now();
  // motor_cmd.effort = { model.get_output_buffer().at(0), model.get_output_buffer().at(1) };

  // pub->publish(motor_cmd);

  // RCLCPP_INFO(get_logger(), out.str().c_str());
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

void set_real_time_priority(std::shared_ptr<ControllerNode> controller) {
  // initialize the scheduling parameters
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = 98;


  //set the scheduling policy and priority for the current process 

  int ret = sched_setscheduler(getpid(), SCHED_FIFO, &param);
  if (ret != 0) {
    RCLCPP_ERROR(controller->get_logger(), "Failed to set the priority ");
  }
  else {
    std::string error = "Process priority is " + std::to_string(param.sched_priority);
    RCLCPP_INFO(controller->get_logger(), error.c_str());
  }
}

// open the serial pord and return the file descriptor
int ControllerNode::open_serial(const std::string& path) {

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

int ControllerNode::read_serial(std::array<float, 2>& data) {
  std::array<char, serial_buffer_size> read_buf;
  int num_bytes = read(serial_file_desc, &read_buf, serial_buffer_size);

  if (num_bytes == -1) {
    std::cout << "Corrupted message, no bytes\n";
    return -1;
  }
  if (num_bytes < 18) {
    std::cout << "Corrupted message, insufficient bytes\n";
    return -1;
  }
  if (read_buf[0] != 's') {
    std::cout << "Corrupted message, Wrong starting Char\n";
    return -1;
  }

  // get the size of coppy, change depending on data type
  int size = sizeof(float) * data.size();
  int index = 3;
  std::memcpy(data.data(), read_buf.data() + index, size);
  return num_bytes;
}

int ControllerNode::write_serial(const std::array<float, 2>& data) {
  std::array<char, serial_buffer_size> send_buf;

  //clear the buffer
  for (size_t i = 0; i < send_buf.size(); i++) {
    send_buf[0] = 0x00;
  }

  send_buf[0] = 's';  // starting character
  send_buf[1] = 'f'; // the type of data being sent
  send_buf[2] = (sizeof(data) / sizeof(float));
  int size = sizeof(float) * data.size();

  // memory to memory copy of the data starting at index 3
  std::memcpy(send_buf.data() + 3, data.data(), size);

  // send last byte to signal that message is done or if there is more to come
  send_buf[19] = 'e';
  return write(serial_file_desc, send_buf.data(), sizeof(send_buf));
}



int ControllerNode::read_serial(std::array<double, 2>& data) {
  std::array<char, serial_buffer_size> read_buf;
  int num_bytes = read(serial_file_desc, &read_buf, serial_buffer_size);

  if (num_bytes == -1) {
    std::cout << "Corrupted message, no bytes\n";
    return -1;
  }
  if (num_bytes < 18) {
    std::cout << "Corrupted message, insufficient bytes\n";
    return -1;
  }
  if (read_buf[0] != 's') {
    std::cout << "Corrupted message, Wrong starting Char\n";
    return -1;
  }

  // get the size of coppy, change depending on data type
  int size = sizeof(double) * data.size();
  int index = 3;
  std::memcpy(data.data(), read_buf.data() + index, size);
  return num_bytes;
}

int ControllerNode::write_serial(const std::array<double, 2>& data) {
  std::array<char, serial_buffer_size> send_buf;

  //clear the buffer
  for (size_t i = 0; i < send_buf.size(); i++) {
    send_buf[0] = 0x00;
  }

  send_buf[0] = 's';  // starting character
  send_buf[1] = 'd'; // the type of data being sent
  send_buf[2] = (sizeof(data) / sizeof(double));
  int size = sizeof(double) * data.size();

  // memory to memory copy of the data starting at index 3
  std::memcpy(send_buf.data() + 3, data.data(), size);

  // send last byte to signal that message is done or if there is more to come
  send_buf[19] = 'e';
  return write(serial_file_desc, send_buf.data(), sizeof(send_buf));
}



int main(int argc, char* argv[]) {
  std::cout << "ControllerNode Node Started " << std::endl;

  rclcpp::init(argc, argv);

  char cwd[PATH_MAX];
  if (getcwd(cwd, sizeof(cwd)) != nullptr) {
    std::cout << "Current Working directory is : " << cwd << std::endl;
  }
  else {
    perror("ERROR getting cwd");
  }


  std::string modelpath = "src/controller/onnx/twip_1.1.0_hori.onnx";
  int num_observations = 4;
  int num_actions = 2;
  auto controller = std::make_shared<ControllerNode>(modelpath, num_observations, num_actions);

  set_real_time_priority(controller);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(controller->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
