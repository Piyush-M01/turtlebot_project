#include "../include/serial_comm.hpp"
#include <stdlib.h> 
#include<string.h>
Serial::Serial()
{
  //subscribing to the velocity topic
  sub = n.subscribe<geometry_msgs::Twist> ("/cmd_vel_filtered", 100, &Serial::callback, this);

  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  serial_port = open("/dev/ttyACM0", O_RDWR);

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
}

void Serial::callback(const geometry_msgs::Twist::ConstPtr& msg)
{

  x=std::to_string(msg->linear.x);
  z=std::to_string(msg->angular.z);
  
  // passing the values in the form  l+0.xxxxxxa+0.xxxxxx
  // + denotes positive(left/forward) values and - denotes negative(right/backward) values
  // l to a, values are for linear velocity and all values after a denotes angular velocity

  // when linear and angular velocity is positive
  
  if(msg->linear.x>=0 && msg->angular.z>=0)
  {
    unsigned char msgs[x.length()+z.length()+4];
    msgs[0]='l';
    msgs[1]='+';
    msgs[x.length()+2]='a';
    msgs[x.length()+3]='+';
    for(int i=0;i<x.length();i++)
    {
      msgs[i+2]=x[i];
    }
    for(int i=0;i<z.length();i++)
    {
      msgs[x.length()+i+4]=z[i];
    }
    write(serial_port, msgs, sizeof(msgs)); //write the values into the serial port
  }

  // when linear velocity is positive and angular velocity is negative

  else if(msg->linear.x>=0 && msg->angular.z<0)
  {
    unsigned char msgs[x.length()+z.length()+3];
    msgs[0]='l';
    msgs[1]='+';
    msgs[x.length()+2]='a';
    for(int i=0;i<x.length();i++)
    {
      msgs[i+2]=x[i];
    }
    for(int i=0;i<z.length();i++)
    {
      msgs[x.length()+i+3]=z[i];
    }
    write(serial_port, msgs, sizeof(msgs));
  }

  // when linear velocity is negative and angular velocity is positive

  else if(msg->linear.x<0 && msg->angular.z>=0)
  {
    unsigned char msgs[x.length()+z.length()+3];
    msgs[0]='l';
    msgs[x.length()+1]='a';
    msgs[x.length()+2]='+';
    for(int i=0;i<x.length();i++)
    {
      msgs[i+1]=x[i];
    }
    for(int i=0;i<z.length();i++)
    {
      msgs[x.length()+i+3]=z[i];
    }
    write(serial_port, msgs, sizeof(msgs));
  }

  // when both linear and angular velocities are negative

  else
  {
    unsigned char msgs[x.length()+z.length()+2];
    msgs[0]='l';
    msgs[x.length()+1]='a';
    for(int i=0;i<x.length();i++)
    {
      msgs[i+1]=x[i];
    }
    for(int i=0;i<z.length();i++)
    {
      msgs[x.length()+i+2]=z[i];
    }
    write(serial_port, msgs, sizeof(msgs));
  }

  
}