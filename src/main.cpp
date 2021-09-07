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








//------------------------------------------------------------------------------

void setup()
{
  // digitalWrite(LED_PIN, LOW);
  portBASE_TYPE s0, s1, s2, s3, s4, s5;

  delay(500);

  if (!HITL)
  {
    Serial.begin(57600);
    Serial.println("Start");
    Serial2.begin(57600);
    init_motors();
    init_radios();
   
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