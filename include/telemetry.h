/*
 * Record and Push imu for Laphable
*/
#ifndef _radio_H_
#define _radio_H_
#include "mavlink.h"
#include "SBUS.h"
#include "task.h"
#include "SdFat.h"
#include <Arduino.h>

class TELEMETRY : public Task
{
  // Public interface methods
public:
  TELEMETRY();
  ~TELEMETRY();
  int start();
  int setup();
  int mavlink_send_and_parse();
SBUS x8r(Serial1);
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint8_t buf0[MAVLINK_MAX_PACKET_LEN];
uint16_t channels[16];
int RCThrottle;
int RCYaw;
int RCRoll;
int RCPitch;
int RCArm;
int RCMode;
bool failSafe;
bool lostFrame;
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
  CONFIG_SD_CONTROLLER m_sd;
  unsigned long m_next_recording;
  unsigned long m_first_recording;
};
extern TELEMETRY g_telemetry;
#endif
