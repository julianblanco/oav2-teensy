#include "flightcontroller.h"



flightcontroller::flightcontroller()
  : 
{ }
flightcontroller::~flightcontroller() { }

void flightcontroller::getsensordata()

void flightcontroller::imuLoop(void *arg)
{
  while (1)
  {
    
    if (mpu6050)
    {
      getmpu6050data();
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

    
    // Serial.println("a");
    vTaskDelay((configTICK_RATE_HZ) / 1000L); //1000hz
                                              // Serial.println(dt);
  }
}

int flightcontroller::slow_sensorsloop()
{
    GPS.read();
    gpsSample(GPS);
    sbusParse();
    //updatebarometer;
    //updatelidar();

}

int flightcontroller::generatetrajectoryloop()
{

}

int flightcontroller::updatemotors()
{
  
//*****************************************************************
// Actuators Thread 100HZ
//*****************************************************************
static void actuarorsThread(void *pvParameters)
{

  while (1)
  {

    if (HITL)
    {
      sendHITLmotorcommands();
    }

    else
    {
      if (flag_armed)
      {
        updatemotors();
      }
      else
      {
        stopmotors();
      }
    }
    vTaskDelay((configTICK_RATE_HZ) / 1000L);
  }
}


}

int flightcontroller::telemetry_and_logging()
{}

  
int flightcontroller::setup()
{
  int code = 0;

  code = this->init_serial();
  if( code != 0 ) this->panic("serial initialization failed", code);

  this->log("[-] waiting five seconds for startup sequence...");
  delay(5000);

  code = this->init_sdcard();
  if( code != 0 ) this->panic("sdcard initialization failed", code);

   code = this->init_imu();
  if( code != 0 ) this->panic("serial initialization failed", code);

  



  return 0;
}

int flightcontroller::generate_new_dir(char* recording_dir, size_t length)
{
  size_t needed;
  size_t blocks_left = m_sd.freeClusterCount() * m_sd.sectorsPerCluster();

  // Check if we have enough for this recording + 2 blocks for accounting information
  while ( blocks_left < (CONFIG_RECORDING_TOTAL_BLOCKS + 2) ) {
#if CONFIG_SD_CARD_ROLLOFF
    char channel_path[256];

    for(int id = m_first_recording; ; id++){
      // Produce a new folder path
      needed = snprintf(recording_dir, length, CONFIG_RECORDING_DIRECTORY, id);

      // Ignore non-existent entries
      if( ! m_sd.exists(recording_dir) ) continue;

      // Remove channel data
      for(int ch = 0; ; ch++) {
        snprintf(channel_path, 256, CONFIG_CHANNEL_PATH, recording_dir, ch);
        if( !m_sd.exists(channel_path) ) break;
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
    this->panic("sd card full and rolloff disabled!", -1);
#endif
  }

  for(int id = m_next_recording; ; id++) {
    // Produce a new folder path
    needed = snprintf(recording_dir, length, CONFIG_RECORDING_DIRECTORY, id);

    // Could we fit it in our buffer?
    if( needed > length ) {
      return 1;
    }

    // Does it exist?
    if( ! m_sd.exists(recording_dir) ) {
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


#if ! CONFIG_DISABLE_NETWORK

int init_bnO055()
{

}
int flightcontroller::init_imu()
{

 if (mpu6050)
    {
      mpu6050init();
      delay(100);
      calculate_IMU_error();
      calibrateAttitude(); //helps to warm up IMU and Madgwick filter
    }
    if (bno055)
    {
      bno050init();
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
}

#endif

int flightcontroller::init_sdcard()
{
  const char* const animation = "\\|/-";
  int frame = 0;

  // Wait for an SD card to be inserted
  this->log("[-] waiting for sd card insertion...");
  while( ! m_sd.begin(CONFIG_SD) ) {
    // Silly, but erases previous message, allows us to give a lil spinny-boi
    this->log("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
    this->log("[%c] waiting for sd card insertion...", animation[frame]);
    frame = (frame + 1) % 4;
    delay(1000);
  }

  // Again, silly, but I like it, okay?
  this->log("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
  this->log("                                    ");
  this->log("\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
  this->log("[+] initialized sd card\n");

  // We track the recording file. If drop off is disabled, then this never changes.
  if( m_sd.exists("/first_recording") ) {
    char buffer[64];
    CONFIG_SD_FILE file = m_sd.open("/first_recording", O_RDONLY);
    file.read(buffer,64);
    file.close();

    m_first_recording = atoi(buffer);
  } else {
    m_first_recording = 0;
    // Touch the marker file
    CONFIG_SD_FILE file = m_sd.open("/first_recording", O_WRONLY | O_CREAT);
    file.write("0\n", 2);
    file.close();
  }

  // This is updated after every recording, and helps start up times when the SD card
  // has a lot of recordings (e.g. >1k).
  if( m_sd.exists("/next_recording") ) {
    char buffer[64];
    CONFIG_SD_FILE file = m_sd.open("/next_recording", O_RDONLY);
    file.read(buffer,64);
    file.close();

    m_next_recording = atoi(buffer);
  } else {
    m_next_recording = 0;
    CONFIG_SD_FILE file = m_sd.open("/next_recording", O_WRONLY | O_CREAT);
    file.write("0\n", 2);
    file.close();
  }

  this->log("[+] first saved recording: %ld\n", m_first_recording);
  this->log("[+] next recording slot: %ld\n", m_next_recording);

  return 0;
}

int flightcontroller::init_radio()
{
  x8r.begin();


  this->log("[+] initialized radios\n");

  return 0;
}

int flightcontroller::init_motors()
{
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

  this->log("[+] initialized audio controller\n");

  return 0;
}



[[noreturn]] void flightcontroller::panic(const char* message, int code) const
{
  this->log("panic: %s: %d\n", message, code);
  exit(1);
}

void flightcontroller::log(const char* fmt, ...) const
{
  char buffer[256];
  va_list args;

  // Render the formatted string to buffer
  va_start(args, fmt);
  vsnprintf(buffer, 256, fmt, args);
  va_end(args);

  // Dump the string over serial
  Serial.print(buffer);
}