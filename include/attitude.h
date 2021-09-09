

/*
 * Attitude
*/
#ifndef _attitude_H_
#define _attitude_H_
#include "task.h"
// #include "config.h"

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
  void PIDAttitudeControl();
private:
  float yawResponse = 0;
  float rollResponse = 0;
  float pitchResponse = 0;
  float throtResponse = 0;

  float yawError = 0;
  float rollError = 0;
  float pitchError = 0;
  float throtError = 0;

  float pYawResponse = 0;
  float pRollResponse = 0;
  float pPitchResponse = 0;
  float pThrotResponse = 0;

  float iYawResponse = 0;
  float iRollResponse = 0;
  float iPitchResponse = 0;
  float iThrotResponse = 0;

  float dYawResponse = 0;
  float dRollResponse = 0;
  float dPitchResponse = 0;
  float dThrotResponse = 0;

  float yawErrorLast = 0;
  float rollErrorLast = 0;
  float pitchErrorLast = 0;
  float throtErrorLast = 0;

  float yawKp = 0.1;
  float yawKi = 0;
  float yawKd = 0;

  float rollKp = 4;
  float rollKi = 0;
  float rollKd = 4;

  float pitchKp = 4;
  float pitchKi = 0;
  float pitchKd = 4;

  float throtKp = 10;
  float throtKi = 0;
  float throtKd = 0;
};
// extern ATTITUDE g_attitude;
#endif