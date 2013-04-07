/**
 * @file Serial.cpp
 * @brief Simple serial interface, for example to talk to Arduino.
 * @author: Michael Kaess
 */

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>

#include "Serial.h"

using namespace std;


// open a serial port connection
void Serial::open(const string& port) {
  m_serialPort = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (m_serialPort==-1) {
    cout << "Unable to open serial port" << endl;
    exit(1);
  }
  fcntl(m_serialPort, F_SETFL,0);

  struct termios port_settings;      // structure to store the port settings in

  cfsetispeed(&port_settings, B115200);    // set baud rates
  cfsetospeed(&port_settings, B115200);

  port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, 8 data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;
  
  tcsetattr(m_serialPort, TCSANOW, &port_settings);    // apply the settings to the port
}

// send a string
void Serial::print(string str) const {
  int res = ::write(m_serialPort, str.c_str(), str.length());
}

// send an integer
void Serial::print(int num) const {
  stringstream stream;
  stream << num << endl;
  string str = stream.str();
  print(str);
}

// send a double
void Serial::print(double num) const {
  stringstream stream;
  stream << num << endl;
  string str = stream.str();
  print(str);
}

// send a float
void Serial::print(float num) const {
  print((double)num);
}
