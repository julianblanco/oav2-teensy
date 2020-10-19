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
#include "read_write_lock.hpp"

#define HTIL 1


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

#define yawKp 0.5
#define yawKi 0
#define yawKd 0
#define rollKp 20
#define rollKi 0
#define rollKd 0
#define pitchKp 20
#define pitchKi 0
#define pitchKd 0

#define throtKp 10
#define throtKi 0
#define throtKd 0

#define throttleIdle 1100

#define waypointmindistance 2
// The LED is attached to pin 13 on the Teensy 4.0
const uint8_t LED_PIN = 13;

// Declare a semaphore handle.
SemaphoreHandle_t sem;



float currentNorth = 0;
float currentEast = 0;
float currentDown = 0;

float desiredNorth = 0;
float desiredEast = 0;
float desiredDown = -5;

float currentLat = 0;
float currentLong = 0;

float wplat[] = {
    39.292186,
    39.292130,
    39.291964,
    39.291709,
    39.292016
};
float wplong[] = {
    -77.286096,
    -77.285766,
    -77.286192,
    -77.286213,
    -77.285894
};

float desiredLat = 0;
float desiredLong = 0;

float homeLat = 0;
float homeLong = 0;

cpp_freertos::ReadWriteLockPreferWriter sensor_rwlock;

float currentYaw = 0;
float currentRoll = 0;
float currentPitch = 0;

float desiredYaw = 0;
float desiredRoll = 0;
float desiredPitch = 0;
float desiredThrottle = 0;

float currentRangeSensorHeight = 0;
float heightOffset = 0;

Servo frontLeftMotor;
Servo frontRightMotor;
Servo backLeftMotor;
Servo backRightMotor;

int frontLeftMotorSignal = 1000;
int frontRightMotorSignal = 1000;
int backLeftMotorSignal = 1000;
int backRightMotorSignal = 1000;

TaskHandle_t Handle_gyroTask;
TaskHandle_t Handle_desiredAttitudeTask;
TaskHandle_t Handle_attitudeTask;
TaskHandle_t Handle_navigationTask;
TaskHandle_t Handle_acutatorsTask;
TaskHandle_t Handle_monitorTask;
TaskHandle_t Handle_commsTask;
TaskHandle_t Handle_lidarTask;

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
float q0 = 1.0f; //initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
Eigen::Matrix<double,1,3> eulerPQR;

//**************************************************************************
//helper functions s
//**************************************************************************

float distanceToWaypoint(float Lat1, float Long1, float Lat2, float Long2) {
    float dist;
    float dLat = (float)(Lat2 - Lat1); // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(Long2 - Long1) * cos(Lat1); //
    dist = sqrt(sq(dLat) + sq(dLon)) * 110312;

    return dist;

}

float courseToWaypoint(float lat1, float long1, float lat2, float long2) {
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
    if (a2 < 0.0) {
        a2 += TWO_PI;
    }
    return degrees(a2);
} // courseToWaypoint()

void convertToNED(float startLat, float startLong, float currentLat, float currentLong, float & North, float & East, float & Down) {
    float heading = courseToWaypoint(startLat, startLong, currentLat, currentLong);
    float distance = distanceToWaypoint(startLat, startLong, currentLat, currentLong);
    North = distance * sin(heading);
    East = distance * cos(heading);
    // Down = currentRangeSensorHeight - heightOffset;
}

float angular_diff(float target_angle, float source_angle) {
    // Find simple difference
    float diff = target_angle - source_angle;
    if (diff > 180)
        diff -= 360;
    else if (diff < -180)
        diff += 360;

    return (diff);
}

float crossTrackError(float distance2WP, float tracklegHead, float targetHead) {
    //convert to radians for use with sin
    tracklegHead = (3.14159265 / 180) * tracklegHead;
    targetHead = (3.14159265 / 180) * targetHead;

    //compute heading error off trackline
    float deltaHeading = tracklegHead - targetHead;

    // crosstrack distance (positive if right of track)
    float distanceXT = distance2WP * sin(deltaHeading);

    return distanceXT;
}

float crossTrackCorrection(float distanceXT, float targetHead, float distance2WP) {
    float xtCoeff = -100; // based on experimental data from the autonomous car
    float temp = (xtCoeff * distanceXT) / distance2WP;

    if (temp > 30) temp = 30; // maximum allowable correction
    if (temp < -30) temp = -30;

    float newTargetHeading = targetHead + temp;

    if (newTargetHeading >= 360) newTargetHeading -= 360;
    else if (newTargetHeading < 0) newTargetHeading += 360;

    return newTargetHeading;
} // end crossTrackError

void gpsSample(Adafruit_GPS & gpsobject) {
    int Fix = 0;
    int new_GPS_data = 0;
    float GpsSpeed = 0;
    float Gpsheading = 0;
    float GpsAltitude = 0;
    float GpsSat = 0;
    float Hour = 0;
    float Minute = 0;
    float Seconds = 0;
    if (gpsobject.newNMEAreceived()) {
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
float saturate(float input, float upperbound, float lowerbound) {
    if (input > upperbound) input = upperbound;
    if (input < lowerbound) input = lowerbound;
    return input;
}


float invSqrt(float x) {
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
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU I2C connection
  /*
   * Don't worry about how this works
   */
  Wire.begin(); //Initialize communication
  delay(20);
  Wire.setClock(1000000); 
  delay(20);
  Wire.beginTransmission(MPU); //Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); //Talk to the register 6B
  Wire.write(0x00); //Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //End the transmission
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
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
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

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
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
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
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
  eulerPQR = derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_)
}


void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters scattered around this code.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void readComputer(){
    
}
void sendComputer(){
    
}
void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro and accelerometer data
  /*
   * Reads accelerometer and gyro data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ. These values are scaled 
   * according to the IMU datasheet to put them into correct units of g's and degree/sec. A simple first-order 
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut 
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally, 
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  float B_accel = 0.14; //0.01
   float B_gyro = 0.89;//.9 for 1khz? //0.13 sets cutoff just past 80Hz for about 3000Hz loop rate

    
    
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
    
    AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
    AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
    AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
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
   
    GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
    GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
    GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
    //Correct the outputs with the calculated error values
    GyroX = GyroX - GyroErrorX;
    GyroY = GyroY - GyroErrorY;
    GyroZ = GyroZ - GyroErrorZ;
    


}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  
  //Read accelerometer values 12000 times
  int c = 0;
  while (c < 12000) {
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
  while (c < 12000) {
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

void calibrateAttitude() {
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
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
  //Grab mean roll and pitch values after everything is warmed up
  for (int j = 1; j <= 2000; j++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt);
    roll_correction = roll_IMU + roll_correction;
    pitch_correction = pitch_IMU + pitch_correction;
    loopRate(2000); //do not exceed 2000Hz
  }
  //These are applied to roll and pitch after Madgwick filter in main loop if desired using correctRollPitch()
  roll_correction = roll_correction/2000.0;
  pitch_correction = pitch_correction/2000.0;
}

//// Derives angular velocity vector from euler angles
Eigen::Matrix<double,1,3> derive_ang_velocity(const Eigen::Ref<const Eigen::Matrix<double,1,3>>& e_)
{
    Eigen::Matrix<double,3,3> tfm_;             // transformation matrix

    tfm_(0,0) = cos(e_(1)); tfm_(0,1) = 0.0; tfm_(0,2) = (-1.0*cos(e_(0))*sin(e_(1)));
    tfm_(1,0) = 0.0; tfm_(1,1) = 1.0; tfm_(1,2) = sin(e_(0));
    tfm_(2,0) = sin(e_(1)); tfm_(2,1) = 0.0; tfm_(2,2) = (cos(e_(0))*cos(e_(1)));

    _derived_pqr_att = tfm_ * (e_ - _prev_derived_euler_att).transpose();       // angular velocity vector

    return(_derived_pqr_att);

} // end derive_ang_velocity()

void getSensorData(void* arg)
{
    while (1) {
          prev_time = current_time;      
          current_time = micros();      
          dt = (current_time - prev_time)/1000000.0; // convert from microseconds to seconds 
          getIMUdata();
          Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, dt); //updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
          vTaskDelay((10L * configTICK_RATE_HZ) / 10000L);//1000hz
        // Serial.println(dt);

  } 
}
//------------------------------------------------------------------------------
/*
 * 50hz
 */

 void trajectoryControl(void* arg) {

 double radius_ = 2.0;          // circle radius
  while (1) {
    
    // References:
    // https://gamedev.stackexchange.com/questions/9607/moving-an-object-in-a-circular-path#:~:text=You%20can%20do%20that%20using,radius%20is%20its%20radius.
    // https://math.stackexchange.com/questions/26329/formula-to-move-the-object-in-circular-path
    // Fast Nonlinear Model Predictive Control for Multicopter Attitude Tracking on SO(3)

    // TODO: have the yaw vector always pointing to the origin, maybe by transforming the euler att to world frame?
    // TODO: either fix the random spikes in roll/pitch/yaw derived measurements or filter them
    // TODO: figure out how to generate a constant desired velocity to be tracked with this trajectory method

   
    if(!_start_traj) {
        _desired_pos << 1.0, 0.0, _desired_pos(2);
    }

    if(!_start_traj & ((_desired_pos - _sensor_pos).lpNorm<2>() < 0.01))
    {
        // std::cout << "Starting Circling Trajectory... " << std::endl;
        _start_traj = 1;
        _traj_start_time = sim_time;
    } else if(_start_traj){
        _traj_time = sim_time - _traj_start_time;
        _desired_pos << radius_*cos(_traj_time), radius_*sin(_traj_time), _desired_pos(2);
        _desired_vel << -radius_*sin(_traj_time), radius_*cos(_traj_time), 0.0;
        _desired_acc << -radius_*cos(_traj_time), -radius_*sin(_traj_time), 0.0;
    }

    // Sleep for 40 milliseconds.
    vTaskDelay((40L * configTICK_RATE_HZ) / 1000L);

  }
}

//*****************************************************************
// positionControl Sensor 100hz
//*****************************************************************
static void positionControl(void * pvParameters) {

       Eigen::Array3d acc_des_ = _desired_acc + (1.0*_Kd_pos.cwiseProduct(_desired_vel - _derived_lin_vel))
                                + (1.0*_Kp_pos.cwiseProduct(_desired_pos - _sensor_pos));
    while (1) {
        if (flag_armed) {
 

    // calculate desired euler attitudes for the attitude controller
    desiredEulerATT_rwlock.WriterLock();
    _desired_euler_att(0) = (1.0/_gravity) * ((acc_des_(0)*sin(_desired_euler_att(2))) + (acc_des_(1)*cos(_desired_euler_att(2))));
    _desired_euler_att(1) = (1.0/_gravity) * ((acc_des_(0)*-1.0*cos(_desired_euler_att(2))) + (acc_des_(1)*sin(_desired_euler_att(2))));
    _desired_euler_att(2) = _derived_euler_att(2);                    // desired yaw is forward-facing
     desiredEulerATT_rwlock.WriterUnLock();
//    _desired_euler_att(2) = _orig_desired_euler_att(2);     // yaw is a free variable

    // For testing the attitude controller; circumvents the position controller except for hover/altitude controller
//    _desired_euler_att(0) = _orig_desired_euler_att(0);
//    _desired_euler_att(1) = _orig_desired_euler_att(1);
//    _desired_euler_att(2) = _orig_desired_euler_att(2);

    _desired_tot_thrust_delta = (_mass / (8.0 * _motor_force_const * _hover_point)) * acc_des_(2);      // hover/altitude control

        }

        myDelayMs(40); //25HZ

    }
    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    vTaskDelete(NULL);
}

//*****************************************************************
// attitudeControl Sensor 100hz
//*****************************************************************
static void attitudeControl(void * pvParameters) {

    Eigen::Array3d att_deltas_;
    Eigen::Matrix<double,4,1> all_deltas_;
    while (1) {
        if (flag_armed) {
            sensor_rwlock.ReaderLock();
            desiredEulerATT_rwlock.ReaderLock();
            att_deltas_(0) = 1.0*(_Kp_ang(0) * (_desired_euler_att(0) - _derived_euler_att(0)))
                                    + 1.0*(_Kd_ang(0) * (_desired_pqr_att(0) - _derived_pqr_att(0)));
            att_deltas_(1) = 1.0*(_Kp_ang(1) * (_desired_euler_att(1) - _derived_euler_att(1)))
                                    + 1.0*(_Kd_ang(1) * (_desired_pqr_att(1) - _derived_pqr_att(1)));
            att_deltas_(2) = 1.0*(_Kp_ang(2) * (_desired_euler_att(2) - _derived_euler_att(2)))
                                    + 1.0*(_Kd_ang(2) * (_desired_pqr_att(2) - _derived_pqr_att(2)));

            _final_att_deltas = att_deltas_;        // for logging purposes
            sensor_rwlock.ReaderUnlock();
            desiredEulerATT_rwlock.ReaderUnock();
            // 4x1 vector to be multiplied by the thrust mapping matrix
            
            //all_deltas_ << (_hover_point + _desired_tot_thrust_delta), att_deltas_(0), att_deltas_(1), att_deltas_(2);

            desired_thrust =  (_motor_mapping * all_deltas_);          // derive desired rotor rates

            backRightMotorSignal = saturate(backRightMotorSignal, 2000, 1000);
            backLeftMotorSignal = saturate(backLeftMotorSignal, 2000, 1000);
            frontRightMotorSignal = saturate(frontRightMotorSignal, 2000, 1000);
            frontLeftMotorSignal = saturate(frontLeftMotorSignal, 2000, 1000);
            // Sleep for 200 milliseconds.
            vTaskDelay((200L * configTICK_RATE_HZ) / 1000L);
        }
  
    }
    // delete ourselves.
    // Have to call this or the system crashes when you reach the end bracket and then get scheduled.
    vTaskDelete(NULL);
}

//*****************************************************************
// Navigation Thread 1HZ
//*****************************************************************
static void navigationThread(void * pvParameters) {
    int waypointNumber = 0;
    int numOfWaypoints = 5; //size of waypoints
    float oldHeading = 0;
    float courseBetweenWaypoints = 0;
    float distanceBetweenWaypoints = 0;
    float targetHeading = 0;
    float lastdesiredLat = 0;
    float lastdesiredLong = 0;
    while (1) {
        //calculates the distance to the waypoint
        float distanceToTarget = distanceToWaypoint(currentLat, currentLong, desiredLat, desiredLong);

        if (flag_are_waypointing == 1) {
            float XTerror = crossTrackError(distanceToTarget, courseBetweenWaypoints, oldHeading);
            if (waypointNumber > 1 && XTerror > 1) {
                oldHeading = courseToWaypoint(currentLat, currentLong, desiredLat, desiredLong);
                targetHeading = crossTrackCorrection(XTerror, oldHeading, distanceToTarget);
            } else {
                targetHeading = courseToWaypoint(currentLat, currentLong, desiredLat, desiredLong);
            }

            if (waypointNumber > 1) {
                lastdesiredLat = wplat[waypointNumber - 1];
                lastdesiredLong = wplong[waypointNumber - 1];
            }

            if (distanceToTarget < waypointmindistance) {
                waypointNumber = waypointNumber + 1;

                if (waypointNumber == numOfWaypoints) {
                    waypointNumber = 1;
                }
            }

            desiredLat = wplat[waypointNumber];
            desiredLong = wplong[waypointNumber];

            distanceBetweenWaypoints = distanceToWaypoint(desiredLat, desiredLong, wplat[waypointNumber - 1], wplong[waypointNumber - 1]);
            courseBetweenWaypoints = courseToWaypoint(wplat[waypointNumber - 1], wplong[waypointNumber - 1], desiredLat, desiredLong);
        }
        myDelayMs(1000);
    }

}
//*****************************************************************
// GPS Thread 50HZ
//*****************************************************************
static void gpsThread(void * pvParameters) {

    while (1) {
        GPS.read();
        gpsSample(GPS);
    }

}

//*****************************************************************
// Actuators Thread 100HZ
//*****************************************************************
static void actuarorsThread(void * pvParameters) {

    while (1) {
        if(HITL)
        {
            Serial.print(frontRightMotorSignal);Serial.print(',');
            Serial.print(frontLeftMotorSignal);Serial.print(',');
            Serial.print(backRightMotorSignal);Serial.print(',');
            Serial.println(backLeftMotorSignal);
        }

        else{        
            if (flag_armed) {
                frontRightMotor.writeMicroseconds(frontRightMotorSignal);
                frontLeftMotor.writeMicroseconds(frontLeftMotorSignal);
                backRightMotor.writeMicroseconds(backLeftMotorSignal);
                backLeftMotor.writeMicroseconds(backRightMotorSignal);
            } else {
                frontRightMotor.writeMicroseconds(1000);
                frontLeftMotor.writeMicroseconds(1000);
                backRightMotor.writeMicroseconds(1000);
                backLeftMotor.writeMicroseconds(1000);

            }
        }
        myDelayMs(10);
    }

}

//------------------------------------------------------------------------------



void setup() {
  // digitalWrite(LED_PIN, LOW);
  portBASE_TYPE s0, s1, s2;
  delay(2000);

  if(!HITL){
  Serial.begin(57600);
  Serial.println("Start");
  IMUinit();
  delay(100);
  calculate_IMU_error();
  calibrateAttitude(); //helps to warm up IMU and Madgwick filter
  Serial.println("I2C start");
  
  // initialize semaphore
  sem = xSemaphoreCreateCounting(1, 0);
  s4 = xTaskCreate(getSensorData , NULL, configMINIMAL_STACK_SIZE, NULL, 5, & Handle_commsTask);
  s3 = xTaskCreate(trajectoryControl, NULL, configMINIMAL_STACK_SIZE, NULL, 4, & Handle_monitorTask);

  s2 = xTaskCreate(positionControl, NULL, configMINIMAL_STACK_SIZE, NULL, 3, & Handle_monitorTask);
  // create task at priority two
  s1 = xTaskCreate(attitudeControl, NULL, configMINIMAL_STACK_SIZE, NULL, 2, & Handle_commsTask);
  // create task at priority one
  s0 = xTaskCreate(actuarorsThread, NULL, configMINIMAL_STACK_SIZE, NULL, 1, & Handle_lidarTask);
  }
  else
  { 
  Serial.begin(2000000);
  Serial.println("Start");
  // initialize semaphore
  sem = xSemaphoreCreateCounting(1, 0);
  s5 = xTaskCreate(trajectoryControl, NULL, configMINIMAL_STACK_SIZE, NULL, 6, & Handle_monitorTask);

  s4 = xTaskCreate(positionControl, NULL, configMINIMAL_STACK_SIZE, NULL, 5, & Handle_monitorTask);
  // create task at priority two
  s3 = xTaskCreate(attitudeControl, NULL, configMINIMAL_STACK_SIZE, NULL, 4, & Handle_commsTask);
  s2 = xTaskCreate(readComputer, NULL, configMINIMAL_STACK_SIZE, NULL, 3, & Handle_monitorTask);
  // create task at priority two
  s1 = xTaskCreate(sendComputer, NULL, configMINIMAL_STACK_SIZE, NULL, 2, & Handle_commsTask);
  // create task at priority one
  s0 = xTaskCreate(readComputer, NULL, configMINIMAL_STACK_SIZE, NULL, 1, & Handle_lidarTask);
  }
  




  // check for creation errors
  if (sem== NULL || s0 != pdPASS || s1 != pdPASS || s2 != pdPASS ) {
    Serial.println("Creation problem");
    while(1);
  }

  Serial.println("Starting the scheduler !");

  // start scheduler
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);
}
//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop() {
  // Not used.
}