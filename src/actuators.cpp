#include "actuators.h"
#include "config.h"
ACTUATORS::ACTUATORS() {}
ACTUATORS::~ACTUATORS() {}

/**
   * Setup the actuators
   *
   * This method will initialize all the servo objects
   * Based on the declaration in config.h, it will declare servos for each actuator controlled be pwm.
   * For a quadcopter, that would be 4 motors to control the ESCs.
   * - 
   */
int ACTUATORS::setup()
{

#ifdef pwm
  g_actuators.frontLeftMotor.attach(3);
  g_actuators.frontRightMotor.attach(4);
  g_actuators.backLeftMotor.attach(5);
  g_actuators.backRightMotor.attach(6);

  g_actuators.frontLeftMotor.writeMicroseconds(1000);
  g_actuators.frontRightMotor.writeMicroseconds(1000);
  g_actuators.backLeftMotor.writeMicroseconds(1000);
  g_actuators.backRightMotor.writeMicroseconds(1000);
  delay(3000);

  // g_actuators.yawMotor.writeMicroseconds(1500);
  // g_actuators.throttle.writeMicroseconds(1500);
#elseif dshot
  int i;

  // Initialize USB serial link
  dshotserial.begin(ESCPID_USB_UART_SPEED);

  // Initialize PID gains
  for (i = 0; i < ESCPID_NB_ESC; i++)
  {
    ESCPID_Kp[i] = ESCPID_PID_ADAPT_GAIN * ESCPID_PID_P;
    ESCPID_Ki[i] = ESCPID_PID_ADAPT_GAIN * ESCPID_PID_I;
    ESCPID_Kd[i] = ESCPID_PID_ADAPT_GAIN * ESCPID_PID_D;
    ESCPID_f[i] = ESCPID_PID_ADAPT_GAIN * ESCPID_PID_F;
    ESCPID_Min[i] = ESCPID_PID_MIN;
    ESCPID_Max[i] = ESCPID_PID_MAX;
  }

  // Initialize PID subsystem
  AWPID_init(ESCPID_NB_ESC,
             ESCPID_Kp,
             ESCPID_Ki,
             ESCPID_Kd,
             ESCPID_f,
             ESCPID_Min,
             ESCPID_Max);

  // Initialize the CMD subsystem
  ESCCMD_init(ESCPID_NB_ESC);

  // Arming ESCs
  ESCCMD_arm_all();

  // Switch 3D mode on
  ESCCMD_3D_on();

  // Arming ESCs
  ESCCMD_arm_all();

  // Start periodic loop
  ESCCMD_start_timer();

  // Stop all motors
  for (i = 0; i < ESCPID_NB_ESC; i++)
  {
    ESCCMD_stop(i);
  }

  // Reference watchdog is initially triggered
  ESCPID_comm_wd = ESCPID_COMM_WD_LEVEL;

#endif
  Task::setup("actuators", 7);
  return 0;
}
/**
   * Start the actuators loop. This runs after the setup function and update the desired pwm which is written to each actuator
   *
   * - 
   */
int ACTUATORS::start()
{
  while (1)
  {
    // {    Serial.println("updatemoteos2");
#ifdef HITL
    sendHITLmotorcommands();

#else

    if (g_armed)
    {
//  Serial.println("updatemoteos");
#ifdef pwm
      update_motors_pwm();
#elseif dshot
      update_motors_dshot();
#endif
    }
    else
    {
      stop_motors();
    }
#endif

    LOOPFREQ(100); //hz
  }
}
void ACTUATORS::sendHITLmotorcommands()
{
  Serial.print(g_actuators.frontRightMotorSignal);
  Serial.print(',');
  Serial.print(g_actuators.frontLeftMotorSignal);
  Serial.print(',');
  Serial.print(g_actuators.backRightMotorSignal);
  Serial.print(',');
  Serial.println(g_actuators.backLeftMotorSignal);
  // Serial2.println("test");
}

void ACTUATORS::update_motors_pwm()
{
  if (g_vehicle_type == 0) //quadcopter
  {

    g_actuators.frontRightMotor.writeMicroseconds(g_actuators.frontRightMotorSignal);
    g_actuators.frontLeftMotor.writeMicroseconds(g_actuators.frontLeftMotorSignal);
    g_actuators.backRightMotor.writeMicroseconds(g_actuators.backRightMotorSignal);
    g_actuators.backLeftMotor.writeMicroseconds(g_actuators.backLeftMotorSignal);
  }
  if (g_vehicle_type == 1) //plane
  {
    g_actuators.yawMotor.writeMicroseconds(g_actuators.yawSignal);
    g_actuators.throttle.writeMicroseconds(g_actuators.throttleSignal);
  }
}


//comes from https://github.com/jacqu/teensyshot/blob/master/ESCPID/ESCPID.ino
void ACTUATORS::update_motors_dshot(){
   static int i, ret;

// Check for next timer event
ret = ESCCMD_tic();

// Bidirectional serial exchange with host
ESCPID_comm_update();

if (ret == ESCCMD_TIC_OCCURED)
{

  // Process timer event

  // Read all measurements and compute current control signal
  for (i = 0; i < ESCPID_NB_ESC; i++)
  {

    // Compute control signal only if telemetry is valid
    // In case of invalid telemetry, last control signal is sent
    // If motor is stopped, don't update PID to avoid integral term windup
    if (!ESCCMD_read_tlm_status(i))
    {

      // Update measurement
      ESCPID_Measurement[i] = ESCPID_comm.rpm[i];

      // Update control signal
      if (ESCPID_Reference[i] >= 0)
        AWPID_control(i,
                      ESCPID_Reference[i],
                      ESCPID_Measurement[i],
                      &ESCPID_Control[i]);
      else
      {
        AWPID_control(i,
                      -ESCPID_Reference[i],
                      -ESCPID_Measurement[i],
                      &ESCPID_Control[i]);
        ESCPID_Control[i] *= -1.0;
      }
    }

    // Send control signal if reference has been sufficiently refreshed
    if (ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL)
    {
      ret = ESCCMD_throttle(i, (int16_t)ESCPID_Control[i]);
    }
    else
    {
      AWPID_reset();
    }
  }

  // Update watchdog
  if (ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL)
  {
    ESCPID_comm_wd++;
  }
}
}
void ACTUATORS::stop_motors()
{
  if (g_vehicle_type == 0) //quadcopter
  {
    g_actuators.frontRightMotor.writeMicroseconds(1000);
    g_actuators.frontLeftMotor.writeMicroseconds(1000);
    g_actuators.backRightMotor.writeMicroseconds(1000);
    g_actuators.backLeftMotor.writeMicroseconds(1000);
  }
  if (g_vehicle_type == 1) //plane
  {
    g_actuators.yawMotor.writeMicroseconds(1500);
    g_actuators.throttle.writeMicroseconds(1500);
  }
}

int ACTUATORS::ESCPID_comm_update(void)
{
  static int i;
  static uint8_t *ptin = (uint8_t *)(&Host_comm),
                 *ptout = (uint8_t *)(&ESCPID_comm);
  static int ret;
  static int in_cnt = 0;

  ret = 0;

  // Read all incoming bytes available until incoming structure is complete
  while ((dshotserial.available() > 0) &&
         (in_cnt < (int)sizeof(Host_comm)))
    ptin[in_cnt++] = dshotserial.read();

  // Check if a complete incoming packet is available
  if (in_cnt == (int)sizeof(Host_comm))
  {

    // Clear incoming bytes counter
    in_cnt = 0;

    // Look for a reset command
    // If first ESC has 0xffff for PID and f gains, reset teensy
    if ((Host_comm.PID_P[0] == ESCPID_RESET_GAIN) &&
        (Host_comm.PID_I[0] == ESCPID_RESET_GAIN) &&
        (Host_comm.PID_D[0] == ESCPID_RESET_GAIN) &&
        (Host_comm.PID_f[0] == ESCPID_RESET_GAIN))
    {

      // Give time to host to close dshotserial port
      delay(ESCPID_RESET_DELAY);

      // Reset command
      SCB_AIRCR = 0x05FA0004;
    }

    // Check the validity of the magic number
    if (Host_comm.magic != ESCPID_COMM_MAGIC)
    {

      // Flush input buffer
      while (dshotserial.available())
        dshotserial.read();

      ret = ESCPID_ERROR_MAGIC;
    }
    else
    {

      // Valid packet received

      // Reset the communication watchdog
      ESCPID_comm_wd = 0;

      // Update the reference
      for (i = 0; i < ESCPID_NB_ESC; i++)
        ESCPID_Reference[i] = Host_comm.RPM_r[i];

      // Update PID tuning parameters
      for (i = 0; i < ESCPID_NB_ESC; i++)
      {

        // Gain conversion from int to float
        ESCPID_Kp[i] = ESCPID_PID_ADAPT_GAIN * Host_comm.PID_P[i];
        ESCPID_Ki[i] = ESCPID_PID_ADAPT_GAIN * Host_comm.PID_I[i];
        ESCPID_Kd[i] = ESCPID_PID_ADAPT_GAIN * Host_comm.PID_D[i];
        ESCPID_f[i] = ESCPID_PID_ADAPT_GAIN * Host_comm.PID_f[i];

        // Update PID tuning
        AWPID_tune(i,
                   ESCPID_Kp[i],
                   ESCPID_Ki[i],
                   ESCPID_Kd[i],
                   ESCPID_f[i],
                   ESCPID_Min[i],
                   ESCPID_Max[i]);
      }

      // Update output data structure values
      // If telemetry is invalid, data structure remains unmodified
      for (i = 0; i < ESCPID_NB_ESC; i++)
      {
        ESCCMD_read_err(i, &ESCPID_comm.err[i]);
        ESCCMD_read_cmd(i, &ESCPID_comm.cmd[i]);
        ESCCMD_read_deg(i, &ESCPID_comm.deg[i]);
        ESCCMD_read_volt(i, &ESCPID_comm.volt[i]);
        ESCCMD_read_amp(i, &ESCPID_comm.amp[i]);
        ESCCMD_read_rpm(i, &ESCPID_comm.rpm[i]);
      }

      // Send data structure to host
      dshotserial.write(ptout, sizeof(ESCPID_comm));

      // Force immediate transmission
      // dshotserial.send_now();
    }
  }

  return ret;
}
