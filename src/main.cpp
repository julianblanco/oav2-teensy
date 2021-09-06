/*
 * Example to demonstrate thread definition, semaphores, and thread sleep.
 */

#include <FreeRTOS_TEENSY4.h>
#include <Wire.h> //I2c communication
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "SBUS.h"
#include "read_write_lock.hpp"
#include "mavlink.h"
#include "SparkFun_BNO080_Arduino_Library.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55);
BNO080 bno080imu;

// #include <stlport.h>
// #include <Eigen30.h>
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);

uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint8_t buf0[MAVLINK_MAX_PACKET_LEN];
uint16_t channels[16];

int highPWMmotor = 1450;

bool vechicle_type = 0; //0quadcopter,1plane,2car
int RCThrottle;
int RCYaw;
int RCRoll;
int RCPitch;
int RCArm;
int RCMode;
bool mpu6050 = 1;
bool bno055 = 0;
bool bno080 = 0;
#define SDLogging 

int currentMode = 0;
bool failSafe;
bool lostFrame;
SBUS x8r(Serial1);
#define HITL 0

#define rollOffset 2.50
#define pitchOffset 3.12

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

#define throttleIdle 1100
#define HITLthrottleIdle 655
#define waypointmindistance 2

int flag_are_waypointing = 1;
// The LED is attached to pin 13 on the Teensy 4.0
const uint8_t LED_PIN = 13;

// Declare a semaphore handle.
SemaphoreHandle_t sem;

bool flag_armed = 0;
float currentNorth = 0;
float currentEast = 0;
float currentDown = 0;

float desiredNorth = 0;
float desiredEast = 0;
float desiredDown = -1;

float currentLat = 0;
float currentLong = 0;

float wplat[] = {
    39.292186,
    39.292130,
    39.291964,
    39.291709,
    39.292016};
float wplong[] = {
    -77.286096,
    -77.285766,
    -77.286192,
    -77.286213,
    -77.285894};

float desiredLat = 0;
float desiredLong = 0;

float homeLat = 0;
float homeLong = 0;

cpp_freertos::ReadWriteLockPreferWriter sensor_rwlock;

float currentYaw = 0;
float currentRoll = 0;
float currentPitch = 0;
float headingOffset = 0;
float desiredYaw = 0;
float desiredRoll = 0;
float desiredPitch = 0;
float desiredThrottle = 200;

float currentRangeSensorHeight = 0;
float heightOffset = 0;
Servo yawMotor;
Servo throttle;

Servo frontLeftMotor;
Servo frontRightMotor;
Servo backLeftMotor;
Servo backRightMotor;

int yawSignal = 1500;
int throttleSignal = 1500;
int frontLeftMotorSignal = 1000;
int frontRightMotorSignal = 1000;
int backLeftMotorSignal = 1000;
int backRightMotorSignal = 1000;

u_int32_t time1 = 0;
u_int32_t time2 = 0;
u_int32_t timebetweenparses = 0;

TaskHandle_t Handle_gyroTask;
TaskHandle_t Handle_desiredAttitudeTask;
TaskHandle_t Handle_attitudeTask;
TaskHandle_t Handle_navigationTask;
TaskHandle_t Handle_acutatorsTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_commsTask;
TaskHandle_t Handle_lidarTask;
TaskHandle_t Handle_telemetry;

const int MPU = 0x68; //MPU6050 I2C address -- pins 19 = SCL & 18 = SDA
//DECLARE GLOBAL VARIABLES
//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float roll_correction, pitch_correction;
float beta = 0.04; //madgwick filter parameter
float q0 = 1.0f;   //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
// Eigen::Matrix<double,1,3> eulerPQR;

//**************************************************************************
//helper functions s
//**************************************************************************
float correct_heading_wrap(float current_heading)
{
  // Correct 360 deg wrap around
  if (current_heading >= 360)
    current_heading -= 360;
  else if (current_heading < 0)
    current_heading += 360;

  return (current_heading);
}

float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2)
{
  float dist;
  float dLat = (float)(Lat2 - Lat1);               // difference of latitude in 1/10 000 000 degrees
  float dLon = (float)(Long2 - Long1) * cos(Lat1); //
  dist = sqrt(sq(dLat) + sq(dLon)) * 110312;

  return dist;
}

float courseToWaypoint(float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = radians(long2 - long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
} // courseToWaypoint()

void convertToNED(float startLat, float startLong, float currentLat, float currentLong, float &North, float &East, float &Down)
{
  float heading = courseToWaypoint(startLat, startLong, currentLat, currentLong);
  float distance = distanceToWaypoint(startLat, startLong, currentLat, currentLong);
  North = distance * sin(heading);
  East = distance * cos(heading);
  // Down = currentRangeSensorHeight - heightOffset;
}

float angular_diff(float target_angle, float source_angle)
{
  // Find simple difference
  float diff = target_angle - source_angle;
  if (diff > 180)
    diff -= 360;
  else if (diff < -180)
    diff += 360;

  return (diff);
}

float crossTrackError(float distance2WP, float tracklegHead, float targetHead)
{
  //convert to radians for use with sin
  tracklegHead = (3.14159265 / 180) * tracklegHead;
  targetHead = (3.14159265 / 180) * targetHead;

  //compute heading error off trackline
  float deltaHeading = tracklegHead - targetHead;

  // crosstrack distance (positive if right of track)
  float distanceXT = distance2WP * sin(deltaHeading);

  return distanceXT;
}

float crossTrackCorrection(float distanceXT, float targetHead, float distance2WP)
{
  float xtCoeff = -100; // based on experimental data from the autonomous car
  float temp = (xtCoeff * distanceXT) / distance2WP;

  if (temp > 30)
    temp = 30; // maximum allowable correction
  if (temp < -30)
    temp = -30;

  float newTargetHeading = targetHead + temp;

  if (newTargetHeading >= 360)
    newTargetHeading -= 360;
  else if (newTargetHeading < 0)
    newTargetHeading += 360;

  return newTargetHeading;
} // end crossTrackError

void gpsSample(Adafruit_GPS &gpsobject)
{
  int Fix = 0;
  int new_GPS_data = 0;
  float GpsSpeed = 0;
  float Gpsheading = 0;
  float GpsAltitude = 0;
  float GpsSat = 0;
  float Hour = 0;
  float Minute = 0;
  float Seconds = 0;
  if (gpsobject.newNMEAreceived())
  {
    if (gpsobject.parse(gpsobject.lastNMEA())) // this also sets the newNMEAreceived() flag to false
    {
      Fix = 1;
      new_GPS_data = 1;
      currentLat = gpsobject.latitudeDegrees;
      currentLong = gpsobject.longitudeDegrees;
      GpsSpeed = gpsobject.speed;
      Gpsheading = gpsobject.angle;
      GpsAltitude = gpsobject.altitude;
      GpsSat = gpsobject.satellites;
      Hour = gpsobject.hour;
      Minute = gpsobject.minute;
      Seconds = gpsobject.seconds;
    }
  }
}
float saturate(float input, float upperbound, float lowerbound)
{
  if (input > upperbound)
    input = upperbound;
  if (input < lowerbound)
    input = lowerbound;
  return input;
}

float invSqrt(float x)
{
  //Fast inverse sqrt
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int *)&x >> 1);
  float tmp = *(float *)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

void IMUinit()
{
  //DESCRIPTION: Initialize IMU I2C connection
  /*
   * Don't worry about how this works
   */
  Wire.begin(); //Initialize communication
  delay(20);
  Wire.setClock(1000000);
  delay(20);
  Wire.beginTransmission(MPU); //Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);            //Talk to the register 6B
  Wire.write(0x00);            //Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);  //End the transmission
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq)
{
  //DESCRIPTION: Attitude estimation through sensor fusion
  /*
   * This function fuses the accelerometer and gyro readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter called beta in the variable declaration section which basically
   * adjusts the weight of accelerometer and gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. 
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //compute angles
  roll_IMU = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951; //degrees
  pitch_IMU = asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;                 //degrees
  yaw_IMU = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951;  //degrees
  //   eulerPQR = derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_)
}

static void telemetry(void *pvParameters)
{

  uint8_t system_type = MAV_TYPE_GROUND_ROVER;    //MAV_TYPE_HELICOPTER;//MAV_TYPE_FIXED_WING;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC; //MAV_AUTOPILOT_ARDUPILOTMEGA
  uint8_t system_mode = MAV_MODE_MANUAL_DISARMED; //MAV_MODE_MANUAL_ARMED; //MAV_MODE_GUIDED_ARMED //MAV_MODE_GUIDED_DISARMED
  uint8_t system_state = MAV_STATE_ACTIVE;
  mavlink_message_t heartbeatMsg;
  mavlink_message_t battery_statusMsg;
  mavlink_message_t radio_statusMsg;
  mavlink_message_t local_position_nedMsg;
  mavlink_message_t global_position_intMsg;
  mavlink_message_t position_target_local_nedMsg;
  mavlink_message_t position_target_global_intMsg;
  mavlink_message_t highres_imuMsg;
  mavlink_message_t attitudeMsg;
  mavlink_message_t sys_statusMsg;
  mavlink_message_t altMsg;
  mavlink_message_t pingMsg;
  mavlink_manual_control_t manual_control;
  mavlink_set_mode_t mode;
  mavlink_heartbeat_t heartbeat;

  mavlink_system_t mavlink_system;
  float position[6];
  bool ManualMode = false;
  int32_t Latitude;
  int32_t Longitude;
  int32_t Altitude;
  int16_t Velocity;
  int64_t Microseconds;

  long parsed;
  long lastparsed;
  long timebetweenparsed;

  //Initialize Timers
  unsigned long heartbeatTimer_TX = millis();
  unsigned long heartbeatTimer_RX = millis();
  unsigned long heartbeatInterval_TX = 0.5L * 1000L;
  unsigned long heartbeatInterval_RX = 2L * 1000L;
  unsigned long currentTime = currentTime;
  int data = 0;
  mavlink_message_t receivedMsg;
  mavlink_status_t mav_status;
  while (1)
  {
    currentTime = micros();
    memset(buf, 0xFF, sizeof(buf));
    mavlink_system.sysid = 1;
    mavlink_system.compid = MAV_COMP_ID_AUTOPILOT1;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    heartbeat.custom_mode = 65536;
    heartbeat.base_mode = 81;

    // // Pack the message
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &heartbeatMsg, 2, 12, heartbeat.base_mode, heartbeat.custom_mode, heartbeat.system_status);
    // // Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &heartbeatMsg);
    // Serial2.println("Heartbeat");
    // //Write Message
    Serial2.write(buf, len);
    memset(buf, 0xFF, sizeof(buf));

    mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &global_position_intMsg, currentTime, 3, 392919390, -772862310, 10, 0xFFFF, 0xFFFF, Velocity, 0xFFFF, 7, 0, 0, 0, 0, 0); //fix_type must be 3 for some odd reason
    // /// Copy the message to send buffer
    len = mavlink_msg_to_send_buffer(buf, &global_position_intMsg);
    //Write Message
    Serial2.write(buf, len);
    memset(buf, 0xFF, sizeof(buf));

    mavlink_msg_altitude_pack(mavlink_system.sysid, mavlink_system.compid, &altMsg, currentTime, 12, 13, 14, 15, 16, 17);
    len = mavlink_msg_to_send_buffer(buf, &altMsg);
    //Write Message
    Serial2.write(buf, len);
    //Reset Buffer
    memset(buf, 0xFF, sizeof(buf));

    mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &local_position_nedMsg, 1, 1, 2, 3, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &local_position_nedMsg);
    //Write Message
    Serial2.write(buf, len);
    //Reset Buffer
    memset(buf, 0xFF, sizeof(buf));

    mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &attitudeMsg, currentTime, currentRoll * 3.14 / 180, currentPitch * 3.14 / 180, currentYaw * 3.14 / 180, 4, 5, 6);
    len = mavlink_msg_to_send_buffer(buf, &attitudeMsg);
    //Write Message
    Serial2.write(buf, len);
    //Reset Buffer
    memset(buf, 0xFF, sizeof(buf));

    mavlink_msg_highres_imu_pack(mavlink_system.sysid, mavlink_system.compid, &highres_imuMsg, currentTime, 0, 0, 0, 0, 1, 2, 1, 2, 3, 0, 0, 0, 10, 9);
    len = mavlink_msg_to_send_buffer(buf, &highres_imuMsg);
    //Write Message
    Serial2.write(buf, len);
    //Reset Buffer
    memset(buf, 0xFF, sizeof(buf));

    mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &sys_statusMsg, 1, 1, 1, 1, 2000, 1900, 1900, 0, 0, 0, 1, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &sys_statusMsg);
    //Write Message
    Serial2.write(buf, len);
    //Reset Buffer
    memset(buf, 0xFF, sizeof(buf));

    //   //Read Message
    if (Serial2.available())

    {
      data = Serial2.read();

      if (mavlink_parse_char(MAVLINK_COMM_0, data, &receivedMsg, &mav_status))
      {
        parsed = micros();
        timebetweenparsed = parsed - lastparsed;
        lastparsed = parsed;
        // Serial.print(1000000/timebetweenparsed);Serial.println(" hz");
        Serial.print("  Sys ID: ");
        Serial.print(receivedMsg.sysid, DEC);
        Serial.print("  Comp ID: ");
        Serial.print(receivedMsg.compid, DEC);
        Serial.print("  Len ID: ");
        Serial.print(receivedMsg.len, DEC);
        Serial.print("  Msg ID: ");
        Serial.print(receivedMsg.msgid, DEC);
        Serial.print("\n");
      }
    }

    // mavlink_msg_ping_pack(mavlink_system.sysid, mavlink_system.compid, &pingMsg, currentTime, 0, 0, 0);
    // len = mavlink_msg_to_send_buffer(buf, &pingMsg);
    // //Write Message
    // Serial2.write(buf, len);
    // //Reset Buffer
    // memset(buf, 0xFF, sizeof(buf));

    // time1 = micros();
    Serial.print(currentYaw, 3);
    Serial.print(',');
    Serial.print(currentRoll, 3);
    Serial.print(',');
    Serial.print(currentPitch, 3);
    Serial.print(',');
    Serial.print(desiredYaw, 3);
    Serial.print(',');
    Serial.print(desiredRoll, 3);
    Serial.print(',');
    Serial.print(desiredPitch, 3);
    Serial.print(',');
    Serial.print(desiredThrottle, 3);
    Serial.print(',');
    Serial.print(currentMode);
    Serial.print(',');
    Serial.println(flag_armed);
    time2 = time1;
    vTaskDelay(1000L * (configTICK_RATE_HZ) / 1000L);

  } //end while1
}

void loopRate(int freq)
{
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters scattered around this code.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time))
  {
    checker = micros();
  }
}

void tokenCreator(char *instr);
void stringparse(char buffer[200], int ind);
static void readComputer(void *pvParameters)
{
  while (1)
  {

    static boolean recvInProgress = false;

    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    const byte numChars = 74;
    char receivedChars[numChars]; // IF BROKEN LOOK HERE, array of chars -> string
    if (Serial.available())
    {
      rc = Serial.read();
      // Serial2.println(rc);

      if (recvInProgress == true)
      {
        if (rc != endMarker)
        {
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars)
          {
            ndx = numChars - 1;
          }
        }
        else
        {
          receivedChars[ndx] = '\0'; // terminate the string
          recvInProgress = false;
          ndx = 0;
          tokenCreator(receivedChars);
        }
      }

      else if (rc == startMarker)
      {
        recvInProgress = true;
      }
    }
    delayMicroseconds(1);
    //  vTaskDelay(( configTICK_RATE_HZ) / 1000L);
  } //end while1
}

void tokenCreator(char *instr)
{
  char *pch;
  pch = strtok(instr, ",");
  int index = 0;
  //  Serial2.println("tok");
  while (pch != NULL)
  {

    stringparse(pch, index);
    pch = strtok(NULL, ",");
    index++;
  }
}

void stringparse(char buffer[80], int ind)
{
  // Serial2.println(buffer);
  // Serial2.println(ind);
  static int choice = 0;
  if ((strcmp(buffer, "$OA008") == 0))
  {
    choice = 1;
    // Serial2.println("sucess");
    time1 = millis();
    timebetweenparses = time1 - time2;
    time2 = time1;
  }
  if ((strcmp(buffer, "$OA009") == 0))
    choice = 2;
  // Serial2.println(choice);
  if (choice == 1)
  {
    // Serial2.println("Choice1");
    if (ind == 1)
      currentYaw = atof(buffer);
    if (ind == 2)
      currentRoll = atof(buffer);
    if (ind == 3)
      currentPitch = atof(buffer);
    if (ind == 4)
      currentNorth = atof(buffer);
    if (ind == 5)
      currentEast = atof(buffer);
    if (ind == 6)
    {
      currentDown = atof(buffer);
      // Serial2.println("Yay");
    }
  }
}

void getIMUdata()
{
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro and accelerometer data
  /*
   * Reads accelerometer and gyro data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ. These values are scaled 
   * according to the IMU datasheet to put them into correct units of g's and degree/sec. A simple first-order 
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut 
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally, 
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the readings.
   */
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  float B_accel = 0.14; //0.01
  float B_gyro = 0.89;  //.9 for 1khz? //0.13 sets cutoff just past 80Hz for about 3000Hz loop rate

  //Get accel data
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); //Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AcX = (Wire.read() << 8 | Wire.read()); //X-axis value
  AcY = (Wire.read() << 8 | Wire.read()); //Y-axis value
  AcZ = (Wire.read() << 8 | Wire.read()); //Z-axis value
  AccX = AcX / 16384.0;
  AccY = AcY / 16384.0;
  AccZ = AcZ / 16384.0;
  //LP filter accelerometer data

  AccX = (1.0 - B_accel) * AccX_prev + B_accel * AccX;
  AccY = (1.0 - B_accel) * AccY_prev + B_accel * AccY;
  AccZ = (1.0 - B_accel) * AccZ_prev + B_accel * AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;

  //Get gyro data
  Wire.beginTransmission(MPU);
  Wire.write(0x43); //Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); //Read 4 registers total, each axis value is stored in 2 registers
  //For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyX = (Wire.read() << 8 | Wire.read());
  GyY = (Wire.read() << 8 | Wire.read());
  GyZ = (Wire.read() << 8 | Wire.read());
  GyroX = GyX / 131.0;
  GyroY = GyY / 131.0;
  GyroZ = GyZ / 131.0;
  //LP filter gyro data

  GyroX = (1.0 - B_gyro) * GyroX_prev + B_gyro * GyroX;
  GyroY = (1.0 - B_gyro) * GyroY_prev + B_gyro * GyroY;
  GyroZ = (1.0 - B_gyro) * GyroZ_prev + B_gyro * GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
}

void calculate_IMU_error()
{
  //DESCRIPTION: Computes IMU error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;

  //Read accelerometer values 12000 times
  int c = 0;
  while (c < 12000)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AcX = (Wire.read() << 8 | Wire.read());
    AcY = (Wire.read() << 8 | Wire.read());
    AcZ = (Wire.read() << 8 | Wire.read());
    AccX = AcX / 16384.0;
    AccY = AcY / 16384.0;
    AccZ = AcZ / 16384.0;
    // Sum all readings
    AccErrorX = AccErrorX + AccX;
    AccErrorY = AccErrorY + AccY;
    AccErrorZ = AccErrorZ + AccZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX = AccErrorX / 12000.0;
  AccErrorY = AccErrorY / 12000.0;
  AccErrorZ = AccErrorZ / 12000.0 - 1.0;
  c = 0;
  //Read gyro values 12000 times
  while (c < 12000)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();
    GyroX = GyX / 131.0;
    GyroY = GyY / 131.0;
    GyroZ = GyZ / 131.0;
    // Sum all readings
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  GyroErrorX = GyroErrorX / 12000.0;
  GyroErrorY = GyroErrorY / 12000.0;
  GyroErrorZ = GyroErrorZ / 12000.0;
}

void calibrateAttitude()
{
  //DESCRIPTION: Extra function to calibrate IMU attitude estimate on startup, can be used to warm up everything before entering main loop
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation. Originally used to eliminate additional error in the signal
   * but no longer used for that purpose as it is not needed on the Teensy. This function is what causes startup to take a few seconds
   * to boot. The roll_correction and pitch_correction values can be applied to the roll and pitch attitude estimates using 
   * correctRollPitch() in the main loop after the madgwick filter function. Again, we don't use this for that purpose but just to warm
   * up the IMU and attitude estimation algorithm.
   */
  //Warm up IMU and madgwick filter
  for (int i = 0; i <= 10000; i++)
  {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    getIMUdata();
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
  //Grab mean roll and pitch values after everything is warmed up
  for (int j = 1; j <= 2000; j++)
  {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    getIMUdata();
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);
    roll_correction = roll_IMU + roll_correction;
    pitch_correction = pitch_IMU + pitch_correction;
    loopRate(2000); //do not exceed 2000Hz
  }
  //These are applied to roll and pitch after Madgwick filter in main loop if desired using correctRollPitch()
  roll_correction = roll_correction / 2000.0;
  pitch_correction = pitch_correction / 2000.0;
}

//// Derives angular velocity vector from euler angles
// Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_)
// {
//     Eigen::Matrix<double,3,3> tfm_;             // transformation matrix

//     tfm_(0,0) = cos(e_(1)); tfm_(0,1) = 0.0; tfm_(0,2) = (-1.0*cos(e_(0))*sin(e_(1)));
//     tfm_(1,0) = 0.0; tfm_(1,1) = 1.0; tfm_(1,2) = sin(e_(0));
//     tfm_(2,0) = sin(e_(1)); tfm_(2,1) = 0.0; tfm_(2,2) = (cos(e_(0))*cos(e_(1)));

//     _derived_pqr_att = tfm_ * (e_ - _prev_derived_euler_att).transpose();       // angular velocity vector

//     return(_derived_pqr_att);

// } // end derive_ang_velocity()
void sbusParse()
{
  if (x8r.read(&channels[0], &failSafe, &lostFrame))
  {

    // write the SBUS packet to an SBUS compatible servo
    //    x8r.write(&channels[0]);
    // Serial.print(channels[0]);Serial.print(",");
    // Serial.print(channels[1]);Serial.print(",");
    // Serial.print(channels[2]);Serial.print(",");
    // Serial.print(channels[3]);Serial.print(",");
    // Serial.print(channels[4]);Serial.print(",");
    // Serial.print(channels[5]);Serial.print(",");
    // Serial.print(channels[6]);Serial.print(",");
    // Serial.print(channels[7]);Serial.print(",");
    // Serial.print(channels[8]);Serial.print(",");
    // Serial.print(channels[9]);Serial.print(",");
    // Serial.print(channels[10]);Serial.print(",");
    // Serial.print(channels[11]);Serial.print(",");
    // Serial.print(channels[12]);Serial.print(",");
    // Serial.print(channels[13]);Serial.print(",");
    // Serial.print(channels[14]);Serial.print(",");
    // Serial.print(channels[15]);Serial.print(",");
    // Serial.println(channels[16]);
    RCThrottle = channels[0]; //172-1811 1017
    RCRoll = channels[1];     //172-1811 mid988
    RCPitch = channels[2];    //172-1811 985
    RCYaw = channels[3];      //172-1811 1000
    RCMode = channels[4];     //down 992 Up172
    RCArm = channels[5];      //down 992 Up172

    if (channels[0] > 0) //if no signal.... needs to be verified across radis
    {
      if (RCArm > 500)
      {
        flag_armed = 1;
      }
      else
      {
        flag_armed = 0;
      }

      if (RCMode < 500)
      {
        currentMode = 1;
      }
      else
      {
        currentMode = 2;
      }
      if (currentMode == 1)
      {

        if (RCYaw > 1040) //(1021-1811)
        {
          desiredYaw = correct_heading_wrap(desiredYaw + (.0001 * pow(map(RCYaw, 1021, 1811, 0, 100), 2)));
        }
        if (RCYaw < 1000)
        {
          desiredYaw = correct_heading_wrap(desiredYaw - (.0001 * pow(map(RCYaw, 170, 1021, 0, 100), 2)));
        }
        desiredYaw = 0;

        desiredThrottle = map(RCThrottle, 172, 1811, 0, 300); //172-1811
        desiredRoll = map(RCRoll, 172, 1811, -20, 20);
        desiredPitch = map(RCPitch, 172, 1811, -20, 20);
      }
    }
    else //no rc signal -- failsafe
    {
      flag_armed = 0;
      currentMode = 0;
    }
  }
}
void getbno055data()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  currentYaw = correct_heading_wrap(euler.x() - headingOffset);
  currentPitch = -1 * euler.y();
  //rollInput=fmod((euler.z()+(360+90)), 360)-180;
  currentRoll = euler.z();
}

void getbno080data()
{
  if (bno080imu.dataAvailable() == true)
  {
    currentRoll = (bno080imu.getRoll()) * 180.0 / PI;   // Convert roll to degrees
    currentPitch = (bno080imu.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    currentYaw = (bno080imu.getYaw()) * 180.0 / PI;     // Convert yaw / heading to degrees
  }
}
void getSensorData(void *arg)
{
  while (1)
  {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0; // convert from microseconds to seconds
    if (mpu6050)
    {
      getIMUdata();
      Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      currentRoll = pitch_IMU;
      currentPitch = -roll_IMU;
      currentYaw = -yaw_IMU;
      // Serial.print("a");
    }
    if (bno055)
    {
      getbno055data();
    }
    if (bno080)
    {
      getbno080data();
    }

    sbusParse();
    // Serial.println("a");
    vTaskDelay((configTICK_RATE_HZ) / 1000L); //1000hz
                                              // Serial.println(dt);
  }
}
//*****************************************************************
// attitudeControl Sensor 25hz
//*****************************************************************
static void positionControl(void *pvParameters)
{
  float northError = desiredNorth - currentNorth;
  float eastError = currentEast - desiredEast;
  float downError = currentDown - desiredDown;

  float northResponse = 0;
  float eastResponse = 0;
  float downResponse = 0;

  float pNorthResponse = 0;
  float pEastResponse = 0;
  float pDownResponse = 0;

  float iNorthResponse = 0;
  float iEastResponse = 0;
  float iDownResponse = 0;

  float dNorthResponse = 0;
  float dEastResponse = 0;
  float dDownResponse = 0;

  float northErrorLast = 0;
  float eastErrorLast = 0;
  float downErrorLast = 0;
  float mixer = 0;
  while (1)
  {
    if (flag_armed)
    {
      if (currentMode == 2)
      {
        northError = currentNorth - desiredNorth;
        eastError = currentEast - desiredEast;
        downError = currentDown - desiredDown;

        pNorthResponse = northError * northKp;
        pEastResponse = eastError * eastKp;
        pDownResponse = downError * downKp;

        iNorthResponse += northError * northKi;
        iEastResponse += eastError * eastKi;
        iDownResponse += downError * downKi;

        dNorthResponse = northKd * (northError - northErrorLast) / 10;
        dEastResponse = eastKd * (eastError - eastErrorLast) / 10;
        dDownResponse = downKd * (downError - downErrorLast) / 10;

        northResponse = pNorthResponse + iNorthResponse + dNorthResponse;
        eastResponse = pEastResponse + iEastResponse + dEastResponse;
        downResponse = pDownResponse + iDownResponse + dDownResponse;

        desiredPitch = saturate(northResponse, 30, -30);
        desiredRoll = saturate(eastResponse, 30, -30);
        desiredThrottle = saturate(downResponse, 1000, -1000);
      }
      if (currentMode == 1)
      {
        //handled elsewhere
      }
    }

    vTaskDelay((configTICK_RATE_HZ) / 1000L);
    //
  }
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete(NULL);
}

//*****************************************************************
// attitudeControl Sensor 100hz
//*****************************************************************
static void attitudeControl(void *pvParameters)
{

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
  while (1)
  {
    if (flag_armed)
    {

      rollErrorLast = rollError;
      pitchErrorLast = pitchError;
      yawErrorLast = yawError;
      sensor_rwlock.ReaderLock();
      yawError = angular_diff(currentYaw, desiredYaw);
      rollError = currentRoll - desiredRoll;
      pitchError = currentPitch - desiredPitch;
      sensor_rwlock.ReaderUnlock();

      pYawResponse = yawError * yawKp;
      pRollResponse = rollError * rollKp;
      pPitchResponse = pitchError * pitchKp;
      pThrotResponse = desiredThrottle;

      iYawResponse += yawError * yawKi;
      iRollResponse += rollError * rollKi;
      iPitchResponse += pitchError * pitchKi;

      dYawResponse = yawKd * (yawError - yawErrorLast) / 100; //10microseconds
      dRollResponse = rollKd * (rollError - rollErrorLast) / 100;
      dPitchResponse = pitchKd * (pitchError - pitchErrorLast) / 100;

      yawResponse = pYawResponse + iYawResponse - dYawResponse;
      rollResponse = pRollResponse + iRollResponse - dRollResponse;
      pitchResponse = pPitchResponse + iPitchResponse - dPitchResponse;

      throtResponse = desiredThrottle;
      if (!HITL)
      {
        if (vechicle_type == 0)
        {
          if (throtResponse < 75)
          {
            frontLeftMotorSignal = 1000;
            frontRightMotorSignal = 1000;
            backLeftMotorSignal = 1000;
            backRightMotorSignal = 1000;
          }
          else
          {
            frontLeftMotorSignal = (int)(throttleIdle + throtResponse - rollResponse + pitchResponse - yawResponse);
            frontRightMotorSignal = (int)(throttleIdle + throtResponse + rollResponse + pitchResponse + yawResponse);
            backLeftMotorSignal = (int)(throttleIdle + throtResponse - rollResponse - pitchResponse + yawResponse);
            backRightMotorSignal = (int)(throttleIdle + throtResponse + rollResponse - pitchResponse - yawResponse);
          }

          // Serial.println(backRightMotorSignal);
        }
        if (vechicle_type == 1)
        {
          yawSignal = 1500 + yawResponse;
          throttleSignal = 1500 + throtResponse;
        }

        yawSignal = saturate(yawSignal, highPWMmotor, 1000);
        throttleSignal = saturate(throttleSignal, highPWMmotor, 1000);
        backRightMotorSignal = saturate(backRightMotorSignal, highPWMmotor, 1000);
        backLeftMotorSignal = saturate(backLeftMotorSignal, highPWMmotor, 1000);
        frontRightMotorSignal = saturate(frontRightMotorSignal, highPWMmotor, 1000);
        frontLeftMotorSignal = saturate(frontLeftMotorSignal, highPWMmotor, 1000);
        // Serial.println(backRightMotorSignal);
      }
      else
      {
        backRightMotorSignal = HITLthrottleIdle + throtResponse - rollResponse + pitchResponse + yawResponse;
        backLeftMotorSignal = HITLthrottleIdle + throtResponse + rollResponse + pitchResponse - yawResponse;
        frontRightMotorSignal = HITLthrottleIdle + throtResponse - rollResponse - pitchResponse - yawResponse;
        frontLeftMotorSignal = HITLthrottleIdle + throtResponse + rollResponse - pitchResponse + yawResponse;

        backRightMotorSignal = saturate(backRightMotorSignal, 600, 600);
        backLeftMotorSignal = saturate(backLeftMotorSignal, 600, 600);
        frontRightMotorSignal = saturate(frontRightMotorSignal, 600, 600);
        frontLeftMotorSignal = saturate(frontLeftMotorSignal, 600, 600);
      }
    }
    else
    {
      yawSignal = 1500;
      throttleSignal = 1500;
      backRightMotorSignal = 1000;
      backLeftMotorSignal = 1000;
      frontRightMotorSignal = 1000;
      frontLeftMotorSignal = 1000;
    }

    vTaskDelay((configTICK_RATE_HZ) / 1000L);
    // delay(100);
  }
  // delete ourselves.
  // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
  vTaskDelete(NULL);
}
//*****************************************************************
// Navigation Thread 1HZ
//*****************************************************************
static void trajectoryControl(void *pvParameters)
{
  int waypointNumber = 0;
  int numOfWaypoints = 5; //size of waypoints
  float oldHeading = 0;
  float courseBetweenWaypoints = 0;
  float distanceBetweenWaypoints = 0;
  float targetHeading = 0;
  float lastdesiredLat = 0;
  float lastdesiredLong = 0;
  while (1)
  {
    //calculates the distance to the waypoint
    float distanceToTarget = distanceToWaypoint(currentLat, currentLong, desiredLat, desiredLong);

    if (flag_are_waypointing == 1)
    {
      float XTerror = crossTrackError(distanceToTarget, courseBetweenWaypoints, oldHeading);
      if (waypointNumber > 1 && XTerror > 1)
      {
        oldHeading = courseToWaypoint(currentLat, currentLong, desiredLat, desiredLong);
        targetHeading = crossTrackCorrection(XTerror, oldHeading, distanceToTarget);
      }
      else
      {
        targetHeading = courseToWaypoint(currentLat, currentLong, desiredLat, desiredLong);
      }

      if (waypointNumber > 1)
      {
        lastdesiredLat = wplat[waypointNumber - 1];
        lastdesiredLong = wplong[waypointNumber - 1];
      }

      if (distanceToTarget < waypointmindistance)
      {
        waypointNumber = waypointNumber + 1;

        if (waypointNumber == numOfWaypoints)
        {
          waypointNumber = 1;
        }
      }

      desiredLat = wplat[waypointNumber];
      desiredLong = wplong[waypointNumber];

      distanceBetweenWaypoints = distanceToWaypoint(desiredLat, desiredLong, wplat[waypointNumber - 1], wplong[waypointNumber - 1]);
      courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber - 1], wplong[waypointNumber - 1], desiredLat, desiredLong);
    }
    vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
  }
}
//*****************************************************************
// GPS Thread 50HZ
//*****************************************************************
static void gpsThread(void *pvParameters)
{

  while (1)
  {
    GPS.read();
    gpsSample(GPS);
  }
}

//*****************************************************************
// Actuators Thread 100HZ
//*****************************************************************
static void actuarorsThread(void *pvParameters)
{

  while (1)
  {

    if (HITL)
    {
      Serial.print(frontRightMotorSignal);
      Serial.print(',');
      Serial.print(frontLeftMotorSignal);
      Serial.print(',');
      Serial.print(backRightMotorSignal);
      Serial.print(',');
      Serial.println(backLeftMotorSignal);
      // Serial2.println("test");
    }

    else
    {
      if (flag_armed)
      {
        if (vechicle_type == 1)
        {
          yawMotor.writeMicroseconds(yawSignal);
          throttle.writeMicroseconds(throttleSignal);
        }
        if (vechicle_type == 0)
        {
          frontRightMotor.writeMicroseconds(frontRightMotorSignal);
          frontLeftMotor.writeMicroseconds(frontLeftMotorSignal);
          backRightMotor.writeMicroseconds(backRightMotorSignal);
          backLeftMotor.writeMicroseconds(backLeftMotorSignal);
        }
      }
      else
      {
        if (vechicle_type == 1)
        {
          yawMotor.writeMicroseconds(1500);
          throttle.writeMicroseconds(1500);
        }
        if (vechicle_type == 0)
        {
          frontRightMotor.writeMicroseconds(1000);
          frontLeftMotor.writeMicroseconds(1000);
          backRightMotor.writeMicroseconds(1000);
          backLeftMotor.writeMicroseconds(1000);
        }
      }
    }
    vTaskDelay((configTICK_RATE_HZ) / 1000L);
  }
}

//------------------------------------------------------------------------------

void setup()
{
  // digitalWrite(LED_PIN, LOW);
  portBASE_TYPE s0, s1, s2, s3, s4, s5;

  delay(500);

  if (!HITL)
  {
    // throttle.attach(2);
    // yawMotor.attach(3);
    frontLeftMotor.attach(3);
    frontRightMotor.attach(4);
    backLeftMotor.attach(5);
    backRightMotor.attach(6);

    frontLeftMotor.writeMicroseconds(1000);
    frontRightMotor.writeMicroseconds(1000);
    backLeftMotor.writeMicroseconds(1000);
    backRightMotor.writeMicroseconds(1000);
    delay(3000);

    yawMotor.writeMicroseconds(1500);
    throttle.writeMicroseconds(1500);
    x8r.begin();
    Serial.begin(57600);
    Serial.println("Start");
    Serial2.begin(57600);
    if (mpu6050)
    {
      IMUinit();
      delay(100);
      calculate_IMU_error();
      calibrateAttitude(); //helps to warm up IMU and Madgwick filter
    }
    if (bno055)
    {
      if (!bno.begin())
      {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1)
          ;
      }

      bno.setExtCrystalUse(true);
    }
    if (bno080)
    {
      if (bno080imu.begin() == false)
      {
        Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
        while (1)
          ;
      }

      Wire.setClock(400000); //Increase I2C data rate to 400kHz

      bno080imu.enableRotationVector(1); //Send data update every 50ms
    }
    Serial.println("I2C start");

    // TaskHandle_t Handle_gyroTask;
    // TaskHandle_t Handle_desiredAttitudeTask;
    // TaskHandle_t Handle_attitudeTask;
    // TaskHandle_t Handle_navigationTask;
    // TaskHandle_t Handle_acutatorsTask;
    // TaskHandle_t Handle_monitorTask;
    // TaskHandle_t Handle_commsTask;
    // TaskHandle_t Handle_lidarTask;

    // initialize semaphore
    sem = xSemaphoreCreateCounting(1, 0);
    s5 = xTaskCreate(telemetry, NULL, configMINIMAL_STACK_SIZE, NULL, 6, &Handle_telemetry);
    s4 = xTaskCreate(getSensorData, NULL, configMINIMAL_STACK_SIZE, NULL, 5, &Handle_monitorTask);
    s3 = xTaskCreate(trajectoryControl, NULL, configMINIMAL_STACK_SIZE, NULL, 4, &Handle_desiredAttitudeTask);

    s2 = xTaskCreate(positionControl, NULL, configMINIMAL_STACK_SIZE, NULL, 3, &Handle_navigationTask);
    // create task at priority two
    s1 = xTaskCreate(attitudeControl, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &Handle_attitudeTask);
    // create task at priority one
    s0 = xTaskCreate(actuarorsThread, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &Handle_acutatorsTask);

    // check for creation errors
    if (sem == NULL || s0 != pdPASS || s1 != pdPASS || s2 != pdPASS)
    {
      Serial.println("Creation problem");
      while (1)
        ;
    }
  }
  else
  {
    Serial.begin(2000000);
    Serial2.begin(2000000);
    Serial2.println("Start HITL");

    // initialize semaphore
    sem = xSemaphoreCreateCounting(1, 0);
    s5 = xTaskCreate(telemetry, NULL, configMINIMAL_STACK_SIZE, NULL, 6, &Handle_telemetry);

    s4 = xTaskCreate(trajectoryControl, NULL, configMINIMAL_STACK_SIZE, NULL, 5, &Handle_navigationTask);
    //   // create task at priority two
    s3 = xTaskCreate(positionControl, NULL, configMINIMAL_STACK_SIZE, NULL, 4, &Handle_desiredAttitudeTask);

    s2 = xTaskCreate(attitudeControl, NULL, configMINIMAL_STACK_SIZE, NULL, 3, &Handle_attitudeTask);
    // create task at priority two
    s1 = xTaskCreate(actuarorsThread, NULL, configMINIMAL_STACK_SIZE, NULL, 2, &Handle_acutatorsTask);
    // create task at priority one
    s0 = xTaskCreate(readComputer, NULL, configMINIMAL_STACK_SIZE, NULL, 1, &Handle_commsTask);

    // check for creation errors
    if (sem == NULL || s0 != pdPASS || s1 != pdPASS || s2 != pdPASS || s3 != pdPASS || s4 != pdPASS)
    { //|| s5 != pdPASS ) {
      Serial2.println("Creation problem");
      while (1)
        ;
    }
  }

  Serial.println("Starting the scheduler !");

  // start scheduler
  vTaskStartScheduler();
  Serial2.println("Insufficient RAM");
  while (1)
    ;
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{
  // Not used.
}