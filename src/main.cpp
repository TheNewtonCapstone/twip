#include <iostream>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <array>
#include <string>
#include <cstring>
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <thread>
#include <chrono>
#include "onnx.hpp"

#define DEBUG_LOG
#define PID_CONTROLLER
// #define ONNX_CONTROLLER


const int RX_BUFFER_SIZE = 255;
const int TX_BUFFER_SIZE = 32;
const int FREQ = 100;
const float ROLL_SETPOINT = 0;
struct PID
{
  float kp;
  float kd;
  float ki;
  float integral;
  float prev_error;
};

PID pid = {1.55, 0.1, 0.15, .0f, .0f};

struct log_data
{
  float time;
  float dt;
  float roll;
  float error;
  float ctrl;
  int pwm;
};

log_data info = {0.f, 0.f, 0.f, 0.f, 0.f, 0};

std::chrono::steady_clock::time_point prev_time;
const std::chrono::microseconds PERIOD(1 / FREQ * 1000000);


// function prototypes
int read_serial(const int file_desc, std::vector<float> &);
int send_serial(const int file_desc, std::array<float, 2> &);
// run the control step
void control_step(const std::vector<float> &input, std::array<float, 2> &output, float dt);
int config_serial();
int torque_to_pwm(float torque);
std::string get_cwd();

int main(int argc, char const *argv[]){
int file_desc;
std::string work_dir = get_cwd();

#ifdef DEBUG_LOG
  std::ofstream logger;
  std::string log_file_path = work_dir + "/logs/log.csv";
  logger.open(log_file_path);
  logger << "time,dt,roll,error,ctrl,pwm" << std::endl;
#endif

#ifdef ONNX_CONTROLLER
  std::string model_path = work_dir + "/models/twip.pth.onnx";
  const int num_observations = 4;
  const int num_actions = 2;
  OnnxHandler model = OnnxHandler(model_path, num_observations, num_actions);
  auto &input_buffer = model.get_input_buffer();
#endif

  try {
    file_desc = config_serial();
    std::vector<float> imu_data(2, 0);
    std::array<float, 2> motor_cmd{};
    std::array<float, 2> prev_motor_cmd{};

    prev_time = std::chrono::steady_clock::now();

    while (1){
      auto current_time = std::chrono::steady_clock::now();
      info.time = std::chrono::duration<float>(current_time.time_since_epoch()).count();

      float dt = std::chrono::duration<float>(current_time - prev_time).count();
      info.dt = dt;

      int rs = read_serial(file_desc, imu_data);
      if (rs < 0)
      {
        std::cout << "Error reading serial data\n";
        continue;
      }
#ifdef ONNX_CONTROLLER
      // load buffer
      input_buffer[0] = imu_data[0];
      input_buffer[1] = imu_data[1];

      input_buffer[2] = prev_motor_cmd[0];
      input_buffer[3] = prev_motor_cmd[1];

      model.run();

      motor_cmd[0] = prev_motor_cmd[0] = model.get_output_buffer()[1];
      motor_cmd[1] = prev_motor_cmd[1] = model.get_output_buffer()[0];

      for (auto n : motor_cmd)
      {
        printf("%f\t", n);
      }

      for (auto n : prev_motor_cmd){
        printf("%f\t", n);
      }
      printf("\n");

#endif
#ifdef PID_CONTROLLER
      control_step(imu_data, motor_cmd, dt);
#endif
      int sent_bytes = send_serial(file_desc, motor_cmd);

      std::ostringstream out;
      out << "Control loop :\n";
      out << "Time step : \t" << dt << "\n";
      out << "Data read : \t " << rs << "\n";

      out << "\nINPUTS:\t";
      for (auto element : imu_data)
      {
        out << element << " ";
      }

      out << "\nOUTPUTS\t";
      for (auto element : motor_cmd)
      {
        out << element << " ";
      }

      out << "\nData sent : \t " << sent_bytes << "\n";
      out << std::endl;
    
#ifdef DEBUG_LOG
    logger << info.time << "," << info.dt << "," << info.roll << "," << info.error << "," << info.ctrl << info.pwm << std::endl;
#endif
      // std::this_thread::sleep_until(next_wake_time);
      // next_wake_time += PERIOD;
      std::cout << out.str() << std::endl;
      prev_time = current_time;
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
    close(file_desc);
  }

  return 0; // success
};

// pid control step
void control_step(const std::vector<float> &input, std::array<float, 2> &output, float dt)
{
  float current_roll = input[0];
  float error = ROLL_SETPOINT - current_roll;

  // PID calculations
  pid.integral += error * dt;

  float derivative = (error - pid.prev_error) / dt;

  float control_signal = pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;

  pid.prev_error = error;

  output[0] = -control_signal;
  output[1] = control_signal;

#ifdef DEBUG_LOG
    info.roll = current_roll;
    info.error = error;
    info.ctrl = control_signal;
    info.pwm = torque_to_pwm(control_signal);
#endif

  // for (auto& cmd : output) {
  //   if (cmd > 1.0f) cmd = 1.0f;
  //   if (cmd < -1.0f) cmd = -1.0f;
  // }
}

// only using metho

// reads from the serial port and update imu value
int read_serial(const int file_desc, std::vector<float> &imu_data)
{
  // Allocate memory for read buffer, set size according to your needs
  std::array<char, RX_BUFFER_SIZE> buffer{};
  // check the first char
  int num_bytes = read(file_desc, buffer.data(), RX_BUFFER_SIZE);
  if (num_bytes <= 0)
  {
    std::cout << "No bytes received\n";
    return -1;
  }

  printf("Received %d bytes\n", num_bytes);

  if (buffer[0] != 's' || buffer[15] != 'e')
  {
    printf("Corrupted data: %c\t %c\n", buffer[0], buffer[15]);
    return -1;
  }

#ifdef DEBUG_SERIAL
  for (int i = 0; i < num_bytes; i++)
  {
    printf("%x\t", buffer[i]);
  }
#endif
  int size = sizeof(float) * buffer[2]; // second character contains information about the size;
  int start_index = 3;
  std::memcpy(imu_data.data(), buffer.data() + start_index, size);

  // check the last char
  return num_bytes;
}

int send_serial(const int file_desc, std::array<float, 2> &data)
{
  std::array<char, TX_BUFFER_SIZE> buffer{};

  buffer[0] = 's'; // starting character
  buffer[1] = 'f'; // the type of data being sent
  buffer[2] = 2;
  int size = sizeof(float) * 2;

  // memory to memory copy of the data starting at index 3
  std::memcpy(buffer.data() + 3, data.data(), size);
  printf("\n");

  // send last byte to signal that message is done or if there is more to come
  buffer[31] = 'e';
  return write(file_desc, buffer.data(), RX_BUFFER_SIZE);
}

int config_serial()
{
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int file_desc = open("/dev/ttyTHS1", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if (tcgetattr(file_desc, &tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
  tty.c_cflag |= CS8;            // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                                                        // Disable echo
  tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
  tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
  tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B230400);
  cfsetospeed(&tty, B230400);

  // Save tty settings, also checking for error
  if (tcsetattr(file_desc, TCSANOW, &tty) != 0)
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  return file_desc;
}

int torque_to_pwm(float torque){ 
  constexpr float STALL_TORQUE = 1.05f;
  constexpr float PWM_MIN = 80;
  constexpr float PWM_MAX = 255;

  float speed = ((torque) / STALL_TORQUE);
  int pwm = PWM_MIN + static_cast<int>(speed * (PWM_MAX - PWM_MIN)); 
  return pwm;
}

std::string get_cwd(){
  std::filesystem::path cwd = std::filesystem::current_path();
  std::cout << "Current Working directory is : " << cwd << std::endl;
  std::filesystem::path parent = cwd.parent_path();
  std::cout << "Parent directory is : " << parent << std::endl;
  return parent.string();
}