// System includes
#include <iostream>
#include <sstream>
#include <stdio.h>      
#include <string.h>    
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>     // POSIX terminal control definitions (struct termios)
#include <system_error>	// For throwing std::system_error
#include <sys/ioctl.h> // Used for TCGETS2, which is required for custom baud rates
#include <cassert>
// #include <asm/termios.h> // Terminal control definitions (struct termios)
// #include <asm/ioctls.h>
// #include <asm/termbits.h>
#include <algorithm>
#include <iterator>
#include "../include/serial.hpp"





SerialPort::SerialPort() {
}

SerialPort::SerialPort(const std::string& file) {
  echo = false; // disable echo mode 
  timeout_ms = default_timeout;
  read_buffer_size = read_buffer_size;
  read_buffer.reserve(read_buffer_size);

  m_serial_port = open(file.c_str(), O_RDWR);
  if (m_serial_port = !- 1)
    state = State::OPEN;

  // Create new termios struct, we call it 'tty' for convention
  termios tty;    
  // Read in existing settings, and handle any error
  if (tcgetattr(m_serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return;
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

  cfsetispeed(&tty, B1152000);
  cfsetospeed(&tty, B1152000);

  // Save tty settings, also checking for error
  if (tcsetattr(m_serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return;
  }
}

SerialPort::~SerialPort(){}

int SerialPort::receive() {
  if (m_serial_port == -1) {
    printf("Attempting to receive, but serial port is not open\n");
    return -1;
  }
  int num_bytes = read(m_serial_port, &read_buffer[0], read_buffer_size);

  // Error Handling
  if (num_bytes < 0) {
      // Read was unsuccessful
    // throw std::system_error(EFAULT, std::system_category());
  }
  else if (num_bytes == 0) {
      std::cout << "System error, device disconnected ... maybe\n";
      // throw std::system_error(EFAULT, std::system_category());
      return -1;
  }

  // check 
  if (read_buffer[0] != 0x7E) {
    std::cout << "Corrupted data \n";
      // throw std::system_error(EFAULT, std::system_category());
    return -1; //
  }
  char type = read_buffer[1]; // contains the buffer
  // switch (type) {
  // case 'd':
  //   memcpy(&value, read_buf + 3, sizeof(double));
  //   break;
  // case 'i':
  //   memcpy(&value, read_buf + 3, sizeof(int));
  //   break;
  // case 'c':
  //   memcpy(&value, read_buf + 3, sizeof(char));
  //   break;
  // case 'f':
  //   memcpy(&value, read_buf + 3, sizeof(char));
  //   break;
  // default:
  //   printf("Not a double\n");
  //   break;

  // }
  std::cout << "Received "  << num_bytes << std::endl;
  for(auto byte : read_buffer){
    std::cout << byte << "\t";
  }
  std::cout << '\n';

  return 0;
}

