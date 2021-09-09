#include "telemetry.h"


TELEMETRY::TELEMETRY()
    :
{
}
TELEMETRY::~TELEMETRY() {}

int TELEMETRY::setup()
{
//
#ifdef MPU6050
  mpu6050init();
#endif
#ifdef BNO055
//shit
#endif
  Task::setup("telemetry", 1);
}

int TELEMETRY::start()
{
  while (1)
  {
    sbusParse();

    vTaskDelay(500*(configTICK_RATE_HZ) / 1000L);
  }
}
int mavlink_send_and_parse()
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
   
    time2 = time1;


}

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
    g_telemetry.RCThrottle = channels[0]; //172-1811 1017
    g_telemetry.RCRoll = channels[1];     //172-1811 mid988
    g_telemetry.RCPitch = channels[2];    //172-1811 985
    g_telemetry.RCYaw = channels[3];      //172-1811 1000
    g_telemetry.RCMode = channels[4];     //down 992 Up172
    g_telemetry.RCArm = channels[5];      //down 992 Up172

    if (channels[0] > 0) //if no signal.... needs to be verified across radis
    {
      if (g_telemetry.RCArm > 500)
      {
        flag_armed = 1;
      }
      else
      {
        flag_armed = 0;
      }

      if (g_telemetry.RCMode < 500)
      {
        currentMode = 1;
      }
      else
      {
        currentMode = 2;
      }
      if (currentMode == 1)
      {

        if (g_telemetry.RCYaw > 1040) //(1021-1811)
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




void quickdebug()
{
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
}