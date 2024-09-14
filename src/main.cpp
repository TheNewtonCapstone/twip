#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <array>
#include <string>
#include <cstring>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <thread>
#include <chrono>


const int RX_BUFFER_SIZE = 255;
const int TX_BUFFER_SIZE = 32;
const int FREQ = 100;
const float ROLL_SETPOINT = 0;
struct PID {
  float kp;
  float kd;
  float ki;
  float integral;
  float prev_error;

};
PID pid = { 5.0f , .0f , .0f , .0f, .0f };


int read_serial(const int file_desc, std::vector<float>&);
int send_serial(const int file_desc, std::array<float, 2>&);
// run the control step
void control_step(const std::vector<float>& input, std::array<float, 2>& output);
int config_serial();



const std::chrono::microseconds PERIOD(1 / FREQ * 1000000);

int main(int argc, char const* argv[]) {
  int file_desc;
  try {
    file_desc = config_serial();
    std::vector<float> imu_data(4, 0);
    std::array<float, 2> motor_cmd{};

    auto next_wake_time = std::chrono::steady_clock::now() + PERIOD;
    while (1) {
      // auto dt = std::chrono::steady_clock::now() - next_wake_time;
      int rs = read_serial(file_desc, imu_data);
      if (rs < 0) {
        std::cout << "Error reading serial data\n";
        continue;
      }
      control_step(imu_data, motor_cmd);
      send_serial(file_desc, motor_cmd);

      for (auto n : imu_data) {
        std::cout << n << "\t";
      }

      for (auto n : motor_cmd) {
        std::cout << n << "\t";
      }
      std::cout << "\n";


      std::this_thread::sleep_until(next_wake_time);
      next_wake_time += PERIOD;
    }


  }
  catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    close(file_desc);
  }

  return 0; // success
};

void control_step(const std::vector<float>& input, std::array<float, 2>& output) {
  float current_roll = input[0];
  float error = ROLL_SETPOINT - current_roll;

  // PID calculations
  pid.integral += error * (1.0f / FREQ);
  // float derivative = (error - pid.prev_error) * FREQ;

  float control_signal = pid.kp * error;
  // + pid.ki * pid.integral + pid.kd * derivative;

  pid.prev_error = error;

  // output[0] = control_signal;
  // output[1] = -control_signal;

  output[0] = 1;
  output[1] = -1;
  for (auto& cmd : output) {
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
  }
}


// reads from the serial port and update imu value
int read_serial(const int file_desc, std::vector<float>& imu_data) {
    // Allocate memory for read buffer, set size according to your needs
  std::array<char, RX_BUFFER_SIZE> buffer{};
//check the first char 
  int num_bytes = read(file_desc, buffer.data(), RX_BUFFER_SIZE);
  if (num_bytes <= 0) {
    std::cout << "No bytes received\n";
    return -1;
  }
  if (buffer[0] != 's' && buffer[num_bytes - 1] != 'e') {
    std::cout << "corrupted data";
    return -1;
  }

#ifdef DEBUG_SERIAL
  for (int i = 0; i < num_bytes; i++) {
    printf("%x\t", buffer[i]);
  }
#endif
  int size = sizeof(float) * buffer[2];  // second character contains information about the size;
  int start_index = 3;
  std::memcpy(imu_data.data(), buffer.data() + start_index, size);

    // check the last char 
  return num_bytes;
}

int send_serial(const int file_desc, std::array<float, 2>& data) {
  std::array<char, TX_BUFFER_SIZE> buffer{};

  buffer[0] = 's';  // starting character
  buffer[1] = 'f'; // the type of data being sent
  buffer[2] = (sizeof(data) / sizeof(double));
  int size = sizeof(float) * data.size();

  // memory to memory copy of the data starting at index 3
  std::memcpy(buffer.data() + 3, data.data(), size);

  // send last byte to signal that message is done or if there is more to come
  buffer[19] = 'e';
  return write(file_desc, buffer.data(), RX_BUFFER_SIZE);
}

int config_serial() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int file_desc = open("/dev/ttyTHS1", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if (tcgetattr(file_desc, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
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

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(file_desc, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  return file_desc;

}