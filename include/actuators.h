
/*
 * Record and Push imu for Laphable
*/

#ifndef _actuators_H_
#define _actuators_H_
#include "task.h"
#include <servo.h>
#include "DSHOT.h"
#include "ESCCMD.h"
#include "AWPID.h"
#include "ESCPID.h"
// #include "config.h"
class ACTUATORS : public Task
{
  // Public interface methods
public:
  ACTUATORS();
  ~ACTUATORS();
  /**
   * Setup the actuators
   *
   * This method will initialize all the servo objects
   * Based on the declaration in config.h, it will declare servos for each actuator controlled be pwm.
   * For a quadcopter, that would be 4 motors to control the ESCs.
   * - 
   */
  int setup();
  /**
   * Start the actuators loop. This runs after the setup function and calls update_motors() which updates the desired pwm which is written to each actuator
   *
   */
  int start();
    /**
   * fucntion updates the desired pwm which is written to each actuator
   *
   * 
   */
  void update_motors_pwm();
      /**
   * called when the craft is not armed or enters a failsafe condition. Writes the preconfigured esc value to stop all actuators or zero (in the case of ailerons etc)
   *
   * 
   */
    void update_motors_dshot();
      /**
   * called when the craft is not armed or enters a failsafe condition. Writes the preconfigured esc value to stop all actuators or zero (in the case of ailerons etc)
   *
   * 
   */
  void stop_motors();
      /**
   * fucntion that updates the desired pwm which is written to each actuator for HITL. Sends the "commanded value over serial to gazebo"
   *
   * 
   */
  void sendHITLmotorcommands();
  int ESCPID_comm_update( void );

  Servo yawMotor;
  Servo throttle;

  Servo frontLeftMotor; //object for sending pwm to the esc
  Servo frontRightMotor;
  Servo backLeftMotor;
  Servo backRightMotor;
  int yawSignal = 1500;
  int throttleSignal = 1500;
  int frontLeftMotorSignal = 1000;
  int frontRightMotorSignal = 1000;
  int backLeftMotorSignal = 1000;
  int backRightMotorSignal = 1000;

  
// Globals
float     ESCPID_Reference[ESCPID_NB_ESC] = {};
float     ESCPID_Measurement[ESCPID_NB_ESC] = {};
float     ESCPID_Control[ESCPID_NB_ESC] = {};
uint16_t  ESCPID_comm_wd = 0;

float     ESCPID_Kp[ESCPID_NB_ESC];
float     ESCPID_Ki[ESCPID_NB_ESC];
float     ESCPID_Kd[ESCPID_NB_ESC];
float     ESCPID_f[ESCPID_NB_ESC];
float     ESCPID_Min[ESCPID_NB_ESC];
float     ESCPID_Max[ESCPID_NB_ESC];

ESCPIDcomm_struct_t ESCPID_comm = {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };
Hostcomm_struct_t   Host_comm =   {
                                  ESCPID_COMM_MAGIC,
                                  {},
                                  {},
                                  {},
                                  {},
                                  {}
                                  };
};

// extern ACTUATORS g_actuators;
#endif