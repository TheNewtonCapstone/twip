#pragma once

#include <string>   // For handling string operations
#include <fstream>  // For file I/O operations (used to read/write to the COM port)
#include <sstream>  
#include <vector>  
// #include <asm/ioctls.h>  // For IOCTL command definitions in Linux
// #include <asm/termbits.h> // For terminal control definitions (struct termios2)
#include <cstdint>  // For using fixed-width integer types like uint8_t

enum class Baudrate {
  B_0,
  B_50,
  B_75,
  B_110,
  B_134,
  B_150,
  B_200,
  B_300,
  B_600,
  B_1200,
  B_1800,
  B_2400,
  B_4800,
  B_9600,
  B_19200,
  B_38400,
  B_57600,
  B_115200,
  B_230400,
  B_460800,
};


enum class State {
  CLOSED,
  OPEN
};




class SerialPort {
public:
  SerialPort();
  SerialPort(const std::string& file);
  SerialPort(const std::string& file, Baudrate baudrate);
  int open_port(const std::string& file); // opens serial port and return the port number 
  int set_baudrate(Baudrate Baudrate);

  int send(const char* data, size_t size);
  int receive();
  ~SerialPort();

  friend std::ostream& operator<<(std::ostream& os, const SerialPort& serial);


private:
  bool echo;
  int32_t timeout_ms;
  static constexpr int32_t  default_timeout = -1;

  std::vector<char> read_buffer;
  unsigned char read_buffer_size;
  static constexpr char default_buffer_size = 255;

  static constexpr Baudrate default_baud_rate = Baudrate::B_115200;

  int m_serial_port;

  State state;
};
