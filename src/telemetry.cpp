#include "telemetry.h"
#include "config.h"
TELEMETRY::TELEMETRY() : x8r(&sbsuserial) {}
TELEMETRY::~TELEMETRY() {}

int TELEMETRY::setup()
{
  //

  Serial.begin(115200);
  // Serial1.begin(57600);
  Serial2.begin(115200);
  x8r.Begin();
#ifdef MPU6050
  mpu6050init();
#endif
#ifdef BNO055
//shit
#endif
  Task::setup("telemetry", 4);
}

int TELEMETRY::start()
{
  int count = 0;
  while (1)
  {
    count++;
    sbusParse();
    if (count > 10)
    {
      quickdebug();
      count = 0;
    }
    // vTaskDelay(100*(configTICK_RATE_HZ) / 1000L);
    LOOPFREQ(100);//hz
  }
}

void TELEMETRY::sbusParse()
{
  // Serial.println("sbus");
  if (x8r.Read())
  {


    // for (int i = 0; i < x8r.rx_channels().size(); i++) {
    //   Serial.print(x8r.rx_channels()[i]);
    //   Serial.print(",");
    // }
    // Serial.println();
    auto l_channels = x8r.rx_channels();
    RCThrottle = l_channels[0]; //172-1811 1017
    RCRoll = l_channels[1];     //172-1811 mid988
    RCPitch = l_channels[2];    //172-1811 985
    RCYaw = l_channels[3];      //172-1811 1000
    RCMode = l_channels[4];     //down 992 Up172
    RCArm = l_channels[5];      //down 992 Up172

    if (l_channels[0] > 0) //if no signal.... needs to be verified across radis
    {
      if (RCArm > 500)
      {
        g_armed = 1;
      }
      else
      {
        g_armed = 0;
      }

      if (RCMode < 500)
      {
        g_current_mode = 1;
      }
      else
      {
        g_current_mode = 2;
      }
      if (g_current_mode == 1)
      {

        if (RCYaw > 1040) //(1021-1811)
        {
          g_navigation.desiredYaw = correct_heading_wrap(g_navigation.desiredYaw + (.0001 * pow(map(g_telemetry.RCYaw, 1021, 1811, 0, 100), 2)));
        }
        if (RCYaw < 1000)
        {
          g_navigation.desiredYaw = correct_heading_wrap(g_navigation.desiredYaw - (.0001 * pow(map(g_telemetry.RCYaw, 170, 1021, 0, 100), 2)));
        }
        g_navigation.desiredYaw = 0;

        g_navigation.desiredThrottle = map(g_telemetry.RCThrottle, 172, 1811, 0, 300); //172-1811
        g_navigation.desiredRoll = map(g_telemetry.RCRoll, 172, 1811, -20, 20);
        g_navigation.desiredPitch = map(g_telemetry.RCPitch, 172, 1811, -20, 20);
      }
      lastread = millis();
    }
    else //no rc signal -- failsafe
    {
      timeelapsed = millis() - lastread;

      if (timeelapsed > 2000)
      {
        Serial.println("lost connection");
        g_armed = 0;
        g_current_mode = 0;
      }
    }
  }
}
int TELEMETRY::mavlink_send_and_parse()
{

  float currentTime = micros();
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
  mavlinkserial.write(buf, len);
  memset(buf, 0xFF, sizeof(buf));

  mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &global_position_intMsg, currentTime, 3, 392919390, -772862310, 10, 0xFFFF, 0xFFFF, Velocity, 0xFFFF, 7, 0, 0, 0, 0, 0); //fix_type must be 3 for some odd reason
  // /// Copy the message to send buffer
  len = mavlink_msg_to_send_buffer(buf, &global_position_intMsg);
  //Write Message
  mavlinkserial.write(buf, len);
  memset(buf, 0xFF, sizeof(buf));

  mavlink_msg_altitude_pack(mavlink_system.sysid, mavlink_system.compid, &altMsg, currentTime, 12, 13, 14, 15, 16, 17);
  len = mavlink_msg_to_send_buffer(buf, &altMsg);
  //Write Message
  mavlinkserial.write(buf, len);
  //Reset Buffer
  memset(buf, 0xFF, sizeof(buf));

  mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &local_position_nedMsg, 1, 1, 2, 3, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &local_position_nedMsg);
  //Write Message
  mavlinkserial.write(buf, len);
  //Reset Buffer
  memset(buf, 0xFF, sizeof(buf));

  mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &attitudeMsg, currentTime, g_attitude.roll * 3.14 / 180, g_attitude.pitch * 3.14 / 180, g_attitude.yaw * 3.14 / 180, 4, 5, 6);
  len = mavlink_msg_to_send_buffer(buf, &attitudeMsg);
  //Write Message
  mavlinkserial.write(buf, len);
  //Reset Buffer
  memset(buf, 0xFF, sizeof(buf));

  mavlink_msg_highres_imu_pack(mavlink_system.sysid, mavlink_system.compid, &highres_imuMsg, currentTime, 0, 0, 0, 0, 1, 2, 1, 2, 3, 0, 0, 0, 10, 9);
  len = mavlink_msg_to_send_buffer(buf, &highres_imuMsg);
  //Write Message
  mavlinkserial.write(buf, len);
  //Reset Buffer
  memset(buf, 0xFF, sizeof(buf));

  mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &sys_statusMsg, 1, 1, 1, 1, 2000, 1900, 1900, 0, 0, 0, 1, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &sys_statusMsg);
  //Write Message
  mavlinkserial.write(buf, len);
  //Reset Buffer
  memset(buf, 0xFF, sizeof(buf));

  //   //Read Message
  if (mavlinkserial.available())

  {
    data = mavlinkserial.read();

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
    // time1 = millis();
    // timebetweenparses = time1 - time2;
    // time2 = time1;
  }
  if ((strcmp(buffer, "$OA009") == 0))
    choice = 2;
  // Serial2.println(choice);
  if (choice == 1)
  {
    // Serial2.println("Choice1");
    if (ind == 1)
      g_imu.yaw = atof(buffer);
    if (ind == 2)
      g_imu.roll = atof(buffer);
    if (ind == 3)
      g_imu.yaw = atof(buffer);
    if (ind == 4)
      g_navigation.currentNorth = atof(buffer);
    if (ind == 5)
      g_navigation.currentEast = atof(buffer);
    if (ind == 6)
    {
      g_navigation.currentDown = atof(buffer);
      // Serial2.println("Yay");
    }
  }
}

void TELEMETRY::quickdebug()
{

  Serial.print(g_imu.roll, 3);
  Serial.print(',');
  Serial.print(g_imu.pitch, 3);
  Serial.print(',');
  Serial.print(g_imu.yaw, 3);
  Serial.print(',');
  Serial.print(g_navigation.desiredYaw, 3);
  Serial.print(',');
  Serial.print(g_navigation.desiredRoll, 3);
  Serial.print(',');
  Serial.print(g_actuators.frontLeftMotorSignal);
  Serial.print(',');
  Serial.print(g_navigation.desiredThrottle, 3);
  Serial.print(',');
  Serial.print(g_current_mode);
  Serial.print(',');
  Serial.println(g_armed);
}

int TELEMETRY::init_sdcard()
{
  // const char *const animation = "\\|/-";
  // int frame = 0;

  // // Wait for an SD card to be inserted
  // this->log("[-] waiting for sd card insertion...");
  while (!m_sd.begin(CONFIG_SD))
  {
    // Silly, but erases previous message, allows us to give a lil spinny-boi
    // this->log("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
    // this->log("[%c] waiting for sd card insertion...", animation[frame]);
    // frame = (frame + 1) % 4;
    delay(1000);
  }

  // Again, silly, but I like it, okay?
  // this->log("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
  // this->log("                                    ");
  // this->log("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
  // this->log("[+] initialized sd card\n");

  // We track the recording file. If drop off is disabled, then this never changes.
  if (m_sd.exists("/first_recording"))
  {
    char buffer[64];
    CONFIG_SD_FILE file = m_sd.open("/first_recording", O_RDONLY);
    file.read(buffer, 64);
    file.close();

    m_first_recording = atoi(buffer);
  }
  else
  {
    m_first_recording = 0;
    // Touch the marker file
    CONFIG_SD_FILE file = m_sd.open("/first_recording", O_WRONLY | O_CREAT);
    file.write("0\n", 2);
    file.close();
  }

  // This is updated after every recording, and helps start up times when the SD card
  // has a lot of recordings (e.g. >1k).
  if (m_sd.exists("/next_recording"))
  {
    char buffer[64];
    CONFIG_SD_FILE file = m_sd.open("/next_recording", O_RDONLY);
    file.read(buffer, 64);
    file.close();

    m_next_recording = atoi(buffer);
  }
  else
  {
    m_next_recording = 0;
    CONFIG_SD_FILE file = m_sd.open("/next_recording", O_WRONLY | O_CREAT);
    file.write("0\n", 2);
    file.close();
  }

  // this->log("[+] first saved recording: %ld\n", m_first_recording);
  // this->log("[+] next recording slot: %ld\n", m_next_recording);

  return 0;
}

int TELEMETRY::generate_new_dir(char *recording_dir, size_t length)
{
  size_t needed;
  size_t blocks_left = m_sd.freeClusterCount() * m_sd.sectorsPerCluster();

  // // Check if we have enough for this recording + 2 blocks for accounting information
  // while (blocks_left < (CONFIG_RECORDING_TOTAL_BLOCKS + 2))
  // {
#if CONFIG_SD_CARD_ROLLOFF
  char channel_path[256];

  for (int id = m_first_recording;; id++)
  {
    // Produce a new folder path
    needed = snprintf(recording_dir, length, CONFIG_RECORDING_DIRECTORY, id);

    // Ignore non-existent entries
    if (!m_sd.exists(recording_dir))
      continue;

    // Remove channel data
    for (int ch = 0;; ch++)
    {
      snprintf(channel_path, 256, CONFIG_CHANNEL_PATH, recording_dir, ch);
      if (!m_sd.exists(channel_path))
        break;
      m_sd.remove(channel_path);
    }

    // Remove the recording directory
    m_sd.rmdir(recording_dir);

    // Update first recording ID
    m_first_recording = id + 1;

    // Update first recording tracker on disk
    CONFIG_SD_FILE file = m_sd.open("/first_recording", O_WRONLY);
    char buffer[64];
    size_t len = snprintf(buffer, 64, "%ld\n", m_first_recording);
    file.write(buffer, len);
    file.close();

    break;
  }
#else
  // this->panic("sd card full and rolloff disabled!", -1);
#endif
  // }

  for (int id = m_next_recording;; id++)
  {
    // Produce a new folder path
    // needed = snprintf(recording_dir, length, CONFIG_RECORDING_DIRECTORY, id);

    // Could we fit it in our buffer?
    if (needed > length)
    {
      return 1;
    }

    // Does it exist?
    if (!m_sd.exists(recording_dir))
    {
      // Create the new directory
      m_sd.mkdir(recording_dir);
      // Increment counter
      m_next_recording = id + 1;

      // Update the next recording tracker on disk
      CONFIG_SD_FILE file = m_sd.open("/next_recording", O_WRONLY);
      char buffer[64];
      size_t len = snprintf(buffer, 64, "%ld\n", m_next_recording);
      file.write(buffer, len);
      file.close();

      return 0;
    }
  }
}
