
typedef enum {
  X,
  Y,
  Z
} dimension;
/* ----------------------------------------------------------------------------------------- */
typedef enum {
  MIN,
  CENTER,
  MAX,
  RANGE_SIZE    // just to know the size of enum
} range;
/* ----------------------------------------------------------------------------------------- */
typedef enum {
  BASE,         // 0
  SHOULDER,     // 1
  ELBOW,        // 2
  WRIST,        // 3
  WRIST_ROTATE, // 4
  GRIPPER       // 5
} servos;

class RobotArm {

public:
    RobotArm();
    ~RobotArm();
    
    void move(int * new_pos);                               // move the arm
    int check_ranges(const int * ranges, int position);
    void set_to_mid();
    int init();
    void send_cmd(const char * cmd);                        // send command
    void set_outfile(int fd);                               // set output file
    static const int num_servos = 6;                        // number of servos
    int * get_pos();                                        // return current position
    static const int ranges[num_servos][RANGE_SIZE];        // min and max ranges of all servos
    unsigned char end[20];                                  // end of each command
    
private:
    int USB;                                                // USB file descriptor
    static const int bufsize = 256;                         // command buffer size
    int position[num_servos];                               // current position of aech servo
    unsigned char cmd[bufsize];                             // command to send to robotic arm
    int outfile;                                            // output file

};

