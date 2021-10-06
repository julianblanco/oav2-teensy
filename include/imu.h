/*
 * imu interface methods and imu loop task during Flightcontroller running
*/
#ifndef _imu_H_
#define _imu_H_

#include "task.h"
#include <Wire.h>
#include <stdint.h>
// #include "config.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <utility/imumaths.h>
#include "read_write_lock.hpp"
#include "helper.h"

class IMU : public Task
{
  // Public interface methods
public:
  IMU();                                        // contstuctor
  ~IMU();                                       // decontstuctor
  float yaw;                                    //yaw for the craft, in degrees
  float roll;                                   //roll for the craft, in degrees
  float pitch;                                  //pitch for the craft, in degrees
  Adafruit_BNO055 bno055imu;                    //class object for the bno055, see adafruits documentation
  BNO080 bno080imu;                             //class object for the bno055, see sparkfuns documentation
  imu::Vector<3> euler;                         //vector to temporarily hold data coming from the bno055
  cpp_freertos::ReadWriteLockPreferWriter lock; //lock to prevent rw issues
  float headingOffset;

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
  /**
   * Start the IMU loop. This runs after the setup function and will lock the IMU Lock, call \n getIMUdata, then lock the IMU lock
   *
   * - 
   */
  int start();

  /**
  * This will begin sampling the IMUs based on what IMU (or IMUs) are declared
  */
  void getIMUdata();
  /**
  * This will get data from the mpu6050 at the standard address and then call madgwick. roll, pitch and yaw come from the output of the madgwick filter. It is called  by getIMUdata in the imu sampling loop. \n\n This function updates the roll, pitch, yaw variables in the class object
  */
  void getmpu6050data();
  /**
  * This will get roll,pitch, and yaw data from the bno080. It is called  by getIMUdata in the imu sampling loop. \n\n This function updates the roll, pitch, yaw variables in the class object
  */
  void getbno080data();
  /**
  * This will get roll, pitch, and yaw data from the bno055. It is called  by getIMUdata in the imu sampling loop. \n\n This function updates the roll, pitch, yaw variables in the class object
  */
  void getbno055data();

  // Private internal methods
private:
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
  /**
   * Calibrate mpu6050 offset
   *
   * to be filled in
   *
   * @param None
   * @return Zero on success
   */
  int calibratempu6050Attitude();

  /**
   * This function fuses the accelerometer and gyro readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter called beta in the variable declaration section which basically
   * adjusts the weight of accelerometer and gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. 
   *
   * @param gx gyro x in degrees
   * @param gy gyro x in degrees
   * @param gz gyro x in degrees
   * @param ax gyro x in degrees
   * @param ay gyro x in degrees
   * @param az gyro x in degrees
   * @param invSampleFreq The length of the path array.
   * @return 0 on success or -1 
   */
  void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);

  /**
   * Start the mpu6050
   *
   * Starts the mpu6050

   *
   * @param None 
   */
  void mpu6050init();
  /**
   * Stop sampling process and upload all channel data to FTP server.
   *
   * @param recording_dir Path to recording directory on both SD and FTP
   * @param data_file Open handles to SD files which are being sampled to
   */
  void calculate_mpu6050_IMU_error();
  void requestdatafrompu6050();

  /**
   * The following initialization routines are called by setup().
   *
   * Each method sets up/configures a necessary subsystem, and returns zero
   * on success. Non-zero return values triggger a panic().
   */
  int init_bno055();
  int init_bno080();
  int init_mpu6050();

  /**
   * Display a panic message and error code and halt the processor.
   *
   * This method **does not return**. It is used in exceptional conditions to
   * cause the process to halt immediately and provide some debugging output to
   * serial.
   *
   * @param message A message to dump to serial
   * @param code An error code to dump to serial (only used if non-zero)
   */
  [[noreturn]] void panic(const char *message, int code) const;

  /**
   * Dump a formatted log message to the serial port.
   *
   * This method takes arguments similar to printf(). The given format string
   * cannot exceed 256 characters in length and will be truncated to this length
   * when output.
   *
   * @param fmt A printf-style format string
   * @param ... Variable arguments matching the given format string
   */
  void log(const char *fmt, ...) const;

  // Private internal variables used directly by our imu

  // #if ! CONFIG_DISABLE_NETWORK
  //   FTP<EthernetClient> m_ftp;

  // // Private internal variables not used by the imu directly
  // private:
  //   uint8_t m_network_heap[CONFIG_NETWORK_HEAP_SIZE];
  // #endif

  // // Static variables used for audio shenanigans
  // private:
  //   static DMAMEM audio_block_t m_audio_queue_buffer[CONFIG_AUDIO_BUFFER_SIZE];
};


#endif