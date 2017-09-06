
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

void RobotArm::create()
{
    int i, * tmp = get_pos();
    for(i = 0; i < num_servos; i++)
        new_pos[i] = tmp[i];        // fill new position by current one

    if(init()) {
        std::cerr << "Failed to initialize usb connection to robotic hand" << std::endl;
        //std::exit(1);
    }
    int use_leap = 1; //TODO remove hardcoded
    end[1] = (use_leap == 1 ? '2' : '4');
    std::cout << "Initialized" << std::endl;
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










void RobotArm::commandRobotUsingIK(float x, float y, float z, float wrist_degree, float wrist_rotate_degree, float open, float reward) {

      // cout <<  "\tx: " << x <<  "\ty: " << y <<  "\tz: " << z << endl;
      // cout <<  "\troll: " << roll <<  "\tpitch: " << pitch <<  "\tyaw: " << yaw << endl;
      // cout <<  "\troll: " << direction_roll <<  "\tpitch: " << direction_pitch <<  "\tyaw: " << direction_yaw << endl;
      float grip_angle_d = 0;
      float grip_angle_r = grip_angle_d  * M_PI / 180.0;    //grip angle in radians for use in calculations
      /* Base angle and radial distance from x,y coordinates */
      float bas_angle_r = atan2( x, y );
      float rdist = sqrt(( x * x ) + ( y * y ));
      /* rdist is y coordinate for the arm */
      y = rdist;
      /* Grip offsets calculated based on grip angle */
      float grip_off_z = ( sin( grip_angle_r )) * GRIPPER_VAL;
      float grip_off_y = ( cos( grip_angle_r )) * GRIPPER_VAL;
      /* Wrist position */
      float wrist_z = ( z - grip_off_z ) - BASE_HGT;
      float wrist_y = y - grip_off_y;
      /* Shoulder to wrist distance ( AKA sw ) */
      float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
      float s_w_sqrt = sqrt( s_w );
      /* s_w angle to ground */
      float a1 = atan2( wrist_z, wrist_y );
      /* s_w angle to humerus */
      float cos = (( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt );
      float a2 = acos(cos);
      /* shoulder angle */
      float shl_angle_r = a1 + a2;
      // if (isnan(shl_angle_r) || isinf(shl_angle_r))
      //   return;
      float shl_angle_d = shl_angle_r * 180.0 / M_PI;
      /* elbow angle */
      float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
      float elb_angle_d = elb_angle_r * 180.0 / M_PI;
      float elb_angle_dn = -( 180.0 - elb_angle_d );
      /* wrist angle */
      float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;

      new_pos[BASE] = ranges[BASE][CENTER]- int((( bas_angle_r * 180.0 / M_PI) * 11.11 ));
      new_pos[SHOULDER] = ranges[SHOULDER][CENTER] + int((( shl_angle_d - 90.0 ) * 6.6 ));
      new_pos[ELBOW] = ranges[ELBOW][CENTER] -  int((( elb_angle_d - 90.0 ) * 6.6 ));
      new_pos[WRIST] = ranges[WRIST][CENTER] + int(( wri_angle_d  * 11.1 / 2 ) - (wrist_degree) * 1000 - 600);
      new_pos[WRIST_ROTATE] = ranges[WRIST_ROTATE][CENTER] + int((( bas_angle_r * 180.0 / M_PI) * 11.11 ) + wrist_rotate_degree * 1200);
      new_pos[GRIPPER] = ranges[GRIPPER][MAX] - int( open * 1400);  // move GRIPPER in safe ranges

      std::cout << " " << new_pos[BASE] << " " << new_pos[SHOULDER] << " " << new_pos[ELBOW] << " " << new_pos[WRIST_ROTATE] << " " << new_pos[WRIST] <<  " " << new_pos[GRIPPER] << std::endl;

      // filtration
      if(filter != 0) {
        int * tmp_pos = get_pos();
        // cout << " " << new_pos[0] << " " << new_pos[1] << " " << new_pos[2] << " " << new_pos[3] << " " << new_pos[4] <<  " " << new_pos[5] << endl;
        // cout << " " << tmp_pos[0] << " " << tmp_pos[1] << " " << tmp_pos[2] << " " << tmp_pos[3] << " " << tmp_pos[4] <<  " " << tmp_pos[5] << endl;

        for(int i = 0; i < num_servos; i++) {
          if(fabs((tmp_pos[i] - new_pos[i]) / (double)filter_const) > filter) {      // move only if greater than filter
            move(new_pos);
            break;
          }
        }
        return;
      }
      
      move(new_pos);
}
















