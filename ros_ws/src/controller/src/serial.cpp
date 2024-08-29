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
#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <algorithm>
#include <iterator>
#include "../include/serial.hpp"





Serial::Serial() {
}

Serial::Serial(const std::string& file) {

  echo = false;
  timeout_ms = default_timeout;
  read_buffer_size = read_buffer_size;
  read_buffer.reserve(read_buffer_size);

  m_serial_port = open(file.c_str(), O_RDWR);
  if (m_serial_port = !- 1)
    state = State::OPEN;

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

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

int Serial::set_baudrate(Baudrate baudrate) {
  switch (baudrate) {
  case Baudrate::B_115200:
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    break;

  default:
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    break;
  }
}
int Serial::receive() {
  if (m_serial_port == -1) {
    printf("Attempting to receive, but serial port is not open\n");
    return -1;
  }

          // Read from file
        // We provide the underlying raw array from the readBuffer_ vector to this C api.
        // This will work because we do not delete/resize the vector while this method
        // is called
  int num_bytes = read(m_serial_port, &read_buffer[0], read_buffer_size);

  // Error Handling
  if (num_bytes < 0) {
      // Read was unsuccessful
    throw std::system_error(EFAULT, std::system_category());
  }
  else if (num_bytes == 0) {
      // n == 0 means EOS, but also returned on device disconnection. We try to get termios2 to distinguish two these two states
    struct termios2 term2;
    int rv = ioctl(m_serial_port, TCGETS2, &term2);

    if (rv != 0) {
      // system error;
      std::cout << "System error\n";
      // throw std::system_error(EFAULT, std::system_category());
      return -1;
    }
  }

  // check 
  if (read_buffer[0] != 0x7E) {
    std::cout << "Corrupted data \n";
      // throw std::system_error(EFAULT, std::system_category());
    return -2; //
  }
  int type = read_buffer[1]; // contains the buffer
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
}
