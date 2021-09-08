/*
 * Record and Push imu for Laphable
*/
#ifndef _radio_H_
#define _radio_H_
#include "mavlink.h"
#include "SBUS.h"
#include "task.h"
#include <Arduino.h>

class TELEMETRY : public Task
{
  // Public interface methods
public:
  TELEMETRY();
  ~TELEMETRY();
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
};
#endif
