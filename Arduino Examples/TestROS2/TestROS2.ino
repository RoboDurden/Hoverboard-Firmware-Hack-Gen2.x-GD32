//    For testing of RemoteROS2 using an ESP32 instead of full stack ROS2 setup.
//    Only for basic tests of RemoteROS2 and its serial protocol.
//    To use with ROS2, use this driver: https://github.com/hoverboard-robotics/hoverboard-driver/tree/humble (use humble branch)
//    This is a copy of SpeedTest.ino for RemoteROS2 (with some extentions).
//
//    Tested with ESP32-WROOM-32 ("ESP32 Dev Module" in Arduino IDE) and Hoverboard-TX to pin PB6 and Hoverboard-RX to pin PB7
//    Also use "New Line" in Serial Monitor in Arduino IDE (or your favorite terminal).
//
//    PB6 (Hoverboard-TX) and PB7 (Hoverboard-RX) can handle 5V I/O-Level :-)
//
//    please share feedback to https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
#define _DEBUG      // debug output to first hardware serial port
//#define DEBUG_RX    // additional hoverboard-rx debug output

#define ESP32       // comment out if using Arduino

#define SEND_MILLIS 50   // send commands to hoverboard every SEND_MILLIS millisesonds

#include "util.h"
#include "ros2serial.h"

#define BAUDRATE 19200   // 19200 is default for RemoteROS2
//#define BAUDRATE 115200   // better for receving high frequency

#ifndef LED_BUILTIN
  //#define LED_BUILTIN 4   // ESP32_2432S028 like "HoverBike CYD LVGL"
  #define LED_BUILTIN 2   // ESP32-WROOM-32
#endif

#ifdef ESP32
  //const int pinRX = 39, pinTX = 37;   // Wemos S2 Mini
  //const int pinRX = 16, pinTX = 17;    // Wemos Lolin32
  //const int pinRX = 16, pinTX = 17;    // ESP32-2432S028
  const int pinRX = 16, pinTX = 17;    // ESP32-WROOM-32
  
  #define oSerialHover Serial1    // ESP32
#else
  #include <SoftwareSerial.h>    // not compatible with RCReceiver because of interrupt conflicts.
  const int pinRX = 9, pinTX = 8;
  SoftwareSerial oSerialHover(pinRX,pinTX); // RX, TX 
  #define oSerialHover Serial    // Arduino
#endif

SerialHover2Server oHoverFeedback;
SerialServer2HoverConfig oHoverConfig;

void setup()
{
  #ifdef _DEBUG
    Serial.begin(115200);
    Serial.println("Hello Hoverboard Gen2.target.board :-)");
  #endif
  
  #ifdef ESP32
    // Serial interface, baud, RX GPIO, TX GPIO
    // Note: The GPIO numbers will not necessarily correspond to the
    // pin number printed on the PCB. Refer to your ESP32 documentation for pin to GPIO mappings.
    HoverSetupEsp32(oSerialHover,BAUDRATE,pinRX,pinTX); 
  #else
    HoverSetupArduino(oSerialHover,BAUDRATE);    //  8 Mhz Arduino Mini too slow for 115200 !!!
  #endif

  oHoverConfig.iDriveMode = 1;
  pinMode(LED_BUILTIN, OUTPUT);
}

#define SPEED_MODE_CONSTANT 0
#define SPEED_MODE_ZIGZAG 1
#define SPEED_MODE_ZIGZAG_STEER 2
int speedMode=SPEED_MODE_ZIGZAG;
int iMax = 30;   // sending a (flattened) zigzag curve with iMax amplitude. can be changed with 'max'. Unit RPM. 30 RPM => 512 revs/s*1024
int iPeriod = 3;  // 3 = 3 seconds period of the zigzag curve
boolean CheckConsole()
{
   if (!Serial.available())  // if there is terminal data comming from user
    return false;
    
  String sReceived = Serial.readStringUntil('\n');
  Serial.println(sReceived);
  String sCmd = ShiftValue(sReceived, " ");
  if (sCmd == sReceived) {
    sCmd = ShiftValue(sReceived, "\n"); // For commands with no argument, eg. "c", "zz", "zzs". Use "New Line" in Serial Monitor (or your favorite terminal).
  }
   
  if ( (sCmd == "m") || (sCmd == "mode"))
  {
    int iMode = abs(ShiftValue(sReceived, "\n").toInt());
    if (iMode < 4)
    {
      oHoverConfig.iDriveMode = iMode;
      if (oHoverConfig.iDriveMode == 0) iMax = CLAMP(iMax,-1000,1000);
    }
  }
  else if (sCmd == "c")
  {
    speedMode = SPEED_MODE_CONSTANT;
    Serial.println("speedMode:CONSTANT");
  }
  else if ( (sCmd == "zz") || (sCmd == "zigzag"))
  {
    speedMode = SPEED_MODE_ZIGZAG;
    Serial.println("speedMode:ZIGZAG");
  }
  else if (sCmd == "zzs")
  {
    speedMode = SPEED_MODE_ZIGZAG_STEER;
    Serial.println("speedMode:ZIGZAG_STEER");
  }
  else if ( (sCmd == "bl") || (sCmd == "batlow"))
  {
    oHoverConfig.fBattEmpty = ShiftValue(sReceived, "\n").toFloat();
  }
  else if ( (sCmd == "bh") || (sCmd == "bathi"))
  {
    oHoverConfig.fBattFull = ShiftValue(sReceived, "\n").toFloat();
  }
  else if (sCmd == "max")
  {
    iMax = abs(ShiftValue(sReceived, "\n").toInt());
    if (oHoverConfig.iDriveMode == 0) iMax = CLAMP(iMax,-1001,1001);
    Serial.print("iMax:");Serial.println(iMax);
  }
  else if (sCmd == "period")
  {
    iPeriod = CLAMP(abs(ShiftValue(sReceived, "\n").toInt()),1,30);
    Serial.print("iPeriod:");Serial.println(iPeriod);
  }
  else
  {
    Serial.print("unknown command: ");
    Serial.print(sCmd); Serial.print("\t value:"); Serial.println(ShiftValue(sReceived, "\n"));
  }
  
  return false;
}

unsigned long iLast = 0;
unsigned long iNext = 0;
void loop()
{
  unsigned long iNow = millis();
  digitalWrite(LED_BUILTIN, (iNow%1000) < 200);

  // For speedMode SPEED_MODE_CONSTANT
  int iSpeed = iMax;
  int iSteer = 0;

  if (speedMode == SPEED_MODE_ZIGZAG) {
    float fScaleMax = 1.6*(CLAMP(iPeriod,3,9)/9.0);  // to flatten the sine curve to send constant iMax for some time
    iSpeed = CLAMP((fScaleMax*iMax/100) * (ABS( (int)((iNow/iPeriod+250) % 1000) - 500) - 250),-iMax,iMax);   // repeats from +300 to -300 to +300 :-)
    iSteer = 0;
  }
  else if (speedMode == SPEED_MODE_ZIGZAG_STEER) {
    float fScaleMax = 1.6*(CLAMP(iPeriod,3,9)/9.0);  // to flatten the sine curve to send constant iMax for some time
    iSpeed = CLAMP((fScaleMax*iMax/100) * (ABS( (int)((iNow/iPeriod+250) % 1000) - 500) - 250),-iMax,iMax);   // repeats from +300 to -300 to +300 :-)
    iSteer = 1 * (ABS( (int)((iNow/400+100) % 400) - 200) - 100);   // repeats from +100 to -100 to +100 :-)
  }

  boolean bReceived;   
  while (bReceived = Receive(oSerialHover,oHoverFeedback))
  {
    DEBUGT("millis",iNow-iLast);
    DEBUGT("iSpeed",iSpeed);
    //DEBUGT("iSteer",iSteer);
    HoverLog(oHoverFeedback);
    iLast = iNow;
  }

  if (iNow > iNext)
  {
    iNext = iNow + SEND_MILLIS/2;
    if (!CheckConsole())
    {
      //DEBUGT("sending iSpeed",iSpeed)

      //if (bReceived)  // Reply only when you receive data
      HoverSend(oSerialHover,iSteer,iSpeed);
    }
  }

}
