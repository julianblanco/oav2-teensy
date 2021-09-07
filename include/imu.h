/*
 * Record and Push imu for Laphable
*/
#ifndef _imu_H_
#define _imu_H_

#include <Wire.h>
#include <stdint.h>
#include "config.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFun_BNO080_Arduino_Library.h>

class imu
{
// Public interface methods
public:
  imu();
  ~imu();
  float yaw;
  float roll;
  float pitch;
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
   * Execute the main loop for the imu.
   *
   * This will begin sampling and uploading results as needed/available.
  */
  void run();

// Private internal methods
private:

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
  int Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);

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

  void getmpu6050data();
  void  getbno080data();
 void  getbno055data();

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
  [[noreturn]] void panic(const char* message, int code) const;

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
  void log(const char* fmt, ...) const;

// Private internal variables used directly by our imu
private:
  CONFIG_SD_CONTROLLER m_sd;
  unsigned long m_next_recording;
  unsigned long m_first_recording;

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