#include "flightcontroller.h"



flightcontroller::flightcontroller()
  : 
{ }
flightcontroller::~flightcontroller() { }

void flightcontroller::getsensordata()


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


  



#if ! CONFIG_DISABLE_NETWORK

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