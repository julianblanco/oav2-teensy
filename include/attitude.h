

/*
 * Record and Push imu for Laphable
*/
#ifndef _attitude_H_
#define _attitude_H_
#include "task.h"
class ATTITUDE : public Task
{
  // Public interface methods
public:
  ATTITUDE();
  ~ATTITUDE();
  float yaw;
  float roll;
  float pitch;
  cpp_freertos::ReadWriteLockPreferWriter lock;

  
  /**
   * Setup the imu
   *
   * This method will initialize all needed IMUs including, but not
   * limited to:
   * - BNO055
   * - BNO080
   * - MPU6050
   * - 
   */
  int setup();
  int start();


#define northKp 10
#define northKi 0
#define northKd 0
#define eastKp 10
#define eastKi 0
#define eastKd 0
#define downKp 40
#define downKi 0
#define downKd 0

#define yawKp -.1
#define yawKi 0
#define yawKd 0

#define rollKp 4
#define rollKi 0
#define rollKd 4

#define pitchKp 4
#define pitchKi 0
#define pitchKd 4

#define throtKp 10
#define throtKi 0
#define throtKd 0
};
extern ATTITUDE g_attiude;
#endif