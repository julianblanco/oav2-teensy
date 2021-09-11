/*
 * Record and Push imu for Laphable
*/
#ifndef _telemetry_H_
#define _telemetry_H_
#include "mavlink.h"
#include "SBUS.h"
#include "task.h"
#include "SdFat.h"
#include <Arduino.h>
// #include "config.h"

class TELEMETRY : public Task
{
  // Public interface methods
public:
  TELEMETRY();
  ~TELEMETRY();
  int start();
  int setup();
  int mavlink_send_and_parse();
  void sbusParse();
  int init_sdcard();

void quickdebug();
SbusRx x8r;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint8_t buf0[MAVLINK_MAX_PACKET_LEN];
// uint16_t channels[16];
int RCThrottle;
int RCYaw;
int RCRoll;
int RCPitch;
int RCArm;
int RCMode;
bool failSafe;
bool lostFrame;
long timeelapsed;
long lastread;
#define CONFIG_RECORDING_DIRECTORY         "/rec%d"
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
  private:

  
// Use SDIO for SD transfer
#define CONFIG_SD_USE_SDIO                 1
// SD Card FAT File System Type
#define CONFIG_SD_FAT_TYPE                 3
// Network Heap Size
#define CONFIG_NETWORK_HEAP_SIZE           (1024*120)
// Watchdog warning timeout (seconds, 1->128)
#define CONFIG_WATCHDOG_WARNING_TIMEOUT    30
// Watchdog reset timeout (seconds, 1->128, greater than warning)
#define CONFIG_WATCHDOG_RESET_TIMEOUT      60
// Serial baud rate
#define CONFIG_SERIAL_BAUD                 9600
// LED used to indicate sampling
#define CONFIG_LED                         LED_BUILTIN
// Length of time to sample (milliseconds)
#define CONFIG_RECORDING_LENGTH               5000
// Length of time to sleep between sampling (milliseconds)
#define CONFIG_HOLD_LENGTH                 25000

// Size of the audio queue buffer
#define CONFIG_AUDIO_BUFFER_SIZE           256
// Roll off old recordings when SD card is full
#define CONFIG_SD_CARD_ROLLOFF             0
// Whether to use Ethernet/FTP
#define CONFIG_DISABLE_NETWORK             1

#if CONFIG_SD_USE_SDIO
// FIFO is faster than DMA according to documentation
#  define CONFIG_SD                        SdioConfig(FIFO_SDIO)
#else
// SD Card SS Pin is defined by the board in some cases
#  ifndef SDCARD_SS_PIN
#    define CONFIG_SD_CS_PIN               SS
#  else
#    define CONFIG_SD_CS_PIN               SDCARD_SS_PIN
#  endif
// SPI Clock Frequency
#  define CONFIG_SPI_CLOCK                 SD_SCK_MHZ(50)
// Arguments to begin()
#  define CONFIG_SD                        SdSpiConfig(CONFIG_SD_CS_PIN, DEDICATED_SPI, CONFIG_SPI_CLOCK)
#endif

/*********************************************************
 The following blocks utilize the above configuration to
 verify some basic format or value constraints and build
 more complex configurations from the above basic configs.
*********************************************************/

#if CONFIG_SD_FAT_TYPE == 0
#define CONFIG_SD_CONTROLLER                      SdFat
#define CONFIG_SD_FILE                            FsFile
#elif CONFIG_SD_FAT_TYPE == 1
#define CONFIG_SD_CONTROLLER                      SdFat32
#define CONFIG_SD_FILE                            FsFile32
#elif CONFIG_SD_FAT_TYPE == 2
#define CONFIG_SD_CONTROLLER                      SdExFat
#define CONFIG_SD_FILE                            ExFile
#elif CONFIG_SD_FAT_TYPE == 3
#define CONFIG_SD_CONTROLLER                      SdFs
#define CONFIG_SD_FILE                            FsFile
#else
#error invalid sd fat type
#endif

CONFIG_SD_CONTROLLER m_sd;
CONFIG_SD_FILE file;
CONFIG_SD_FILE datafile;
  unsigned long m_next_recording;
  unsigned long m_first_recording;
    /**
   * Upload a file from the local SD card to the FTP server
   *
   * The given filename is used as a path on both the local SD card and within
   * the remote FTP server home directory and the directory should already exist.
   *
   * @param filename The name of the a file to upload to the FTP server
   * @return Zero on success, negative on internal error and positive on FTP error
   */
  

  /**
   * Create a new folder within the SD card which doesn't already exist. This
   * method will first chdir back to root, then locate a non-existent folder
   * name which follows the `CONFIG_RECORDING_DIRECTORY` preprocessor directive.
   *
   * The passed path array will be filled with the name of the new directory and
   * will be null-terminated. If the new directory name cannot fit in the provided
   * string, -1 is returned.
   *
   * Even on failure, the given character array will be modified while searching
   * for a non-existent directory path.
   *
   * @param path A writable buffer of `len` length to hold the new directory path.
   * @param len The length of the path array.
   * @return 0 on success or -1 if the path cannot fit in the provided buffer.
   */
  int generate_new_dir(char* path, size_t len);

  /**
   * Start the logging process
   *
   * This will generate a new recording directory and store it in the given buffer
   * and also initialize the given data file list with open handles to the channel
   * data files. The data file array must be the same length as CONFIG_CHANNEL_COUNT.
   *
   * @param recording_dir A buffer to hold the path to the new recording directory
   * @param length The length of the recording_dir buffer
   * @param data_file A list of CONFIG_SD_FILE objects where we can open the data files.
   */
  void start_logging(char* recording_dir, size_t length, CONFIG_SD_FILE* data_file);

  /**
   * Stop sampling process and upload all channel data to FTP server.
   *
   * @param recording_dir Path to recording directory on both SD and FTP
   * @param data_file Open handles to SD files which are being sampled to
   */
  void stop_logging(const char* recording_dir);

  /**
   * The following initialization routines are called by setup().
   *
   * Each method sets up/configures a necessary subsystem, and returns zero
   * on success. Non-zero return values triggger a panic().
   */

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
};
// extern TELEMETRY g_telemetry;
#endif
