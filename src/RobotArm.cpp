
#include "RobotArm.h"

#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>    // POSIX terminal control definitions
#include <math.h>
#include <cstdio>
#include <cstring>
#include <vector>


/* ----------------------------------------------------------------------------------------- */
const int RobotArm::ranges[RobotArm::num_servos][RANGE_SIZE] = {  // MIN, CENTER, MAX
  {600,  1500, 2400},                                         // BASE
  {600,  1500, 2200},                                         // SHOULDER
  {600,  1250, 2200},                                         // ELBOW
  {600,  1500, 2400},                                         // WRIST
  {600,  1350, 2400},                                         // WRIST_ROTATE
  {600,  1600, 2400},                                         // GRIPPER
};


/* ----------------------------------------------------------------------------------------- */
void RobotArm::set_to_mid()
{
  // set all servos to mid position
  unsigned char cmd[] = "#0P1500S200#1P1600S200#2P1400S200#3P1500S200#4P1450S200#5P1400S250\r";     // set to mid slowly
  write(USB, cmd, strlen((char *)cmd));
}


/* ----------------------------------------------------------------------------------------- */
RobotArm::RobotArm()      	// **__here__**
:
  position {     // "center" position for each servo
    1500,
    1500,
    1250,
    1500,
    1350,
    1600
  },
  end {'T', '8', '0', '0', '\r', '\0'},     // (was 200) 800 ms to complete whole move
  outfile(0)
{
  memset(cmd, 0, bufsize);
}


/* ----------------------------------------------------------------------------------------- */
RobotArm::~RobotArm()
{
  close(USB);

  if(outfile)
    close(outfile);
}


/* ----------------------------------------------------------------------------------------- */
int RobotArm::init()
{
  /* Open File Descriptor */
  USB = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK | O_NDELAY);

  // Error Handling
  if(USB < 0) {
    std::cerr << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) << std::endl;
    return 1;
  }

  // Configure Port
  struct termios tty;
  memset(&tty, 0, sizeof tty);

  // Error Handling
  if(tcgetattr(USB, &tty) != 0) {
    std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    return 1;
  }

  /* Set Baud Rate */
  cfsetospeed (&tty, B9600);
  cfsetispeed (&tty, B9600);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;
  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_lflag     =   0;                  // no signaling chars, no echo, no canonical processing
  tty.c_oflag     =   0;                  // no remapping, no delays
  tty.c_cc[VMIN]  =   0;                  // read doesn't block
  tty.c_cc[VTIME] =   5;                  // 0.5 seconds read timeout

  tty.c_cflag     |=  CREAD | CLOCAL;                  // turn on READ & ignore ctrl lines
  tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);         // turn off s/w flow ctrl
  tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  tty.c_oflag     &=  ~OPOST;                          // make raw

  /* Flush Port, then applies attributes */
  tcflush(USB, TCIFLUSH);

  if(tcsetattr(USB, TCSANOW, &tty) != 0) {
    std::cerr << "Error " << errno << " from tcsetattr" << std::endl;
    return 1;
  }

  set_to_mid();

  return 0;
}


/* ----------------------------------------------------------------------------------------- */
int RobotArm::check_ranges(const int * ranges, int position)
{
  if(position > ranges[MAX] || position < ranges[MIN])
    return 1;     // position of servo i is minimal or maximal

  return 0;     // no error occured
}


/* ----------------------------------------------------------------------------------------- */
void RobotArm::move(int * new_pos)
{
  int i;

  for(i = 0; i < num_servos; i++) {
    if(position[i] != new_pos[i]) {       // detected move change

      // if(new_pos[i] > ranges[i][MAX]) {
      //   new_pos[i] = ranges[i][MAX];
      //
      // } else if(new_pos[i] < ranges[i][MIN]) {  // moved in safe ranges
      //   new_pos[i] = ranges[i][MIN];
      // }
      if(check_ranges(ranges[i], new_pos[i]) == 0) {  // moved in safe ranges
        position[i] = new_pos[i];         // save the new position
      }
      sprintf((char *)(cmd + (strlen((char *)cmd))), "#%dP%d", i, position[i]);
    }

  }

  if(strlen((char *)cmd) > 0) {               // command is not empty
    strcat((char *)cmd, (char *)end);         // append end of the string to command
    write(USB, cmd, strlen((char *)cmd));     // send command to hand

    if(outfile)     // write to outfile if open
      write(outfile, cmd, strlen((char *)cmd));
  }

  memset(cmd, 0, bufsize);                  // clear buffer
}


/* ----------------------------------------------------------------------------------------- */
int * RobotArm::get_pos()
{
  return position;
}


/* ----------------------------------------------------------------------------------------- */
void RobotArm::send_cmd(const char * cmd)
{
  write(USB, cmd, strlen((char *)cmd));
}


/* ----------------------------------------------------------------------------------------- */
void RobotArm::set_outfile(int fd)
{
  outfile = fd;
}





