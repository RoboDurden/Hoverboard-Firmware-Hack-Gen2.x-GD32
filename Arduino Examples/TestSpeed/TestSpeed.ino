//    Tested with Arduino Pro Mini 3.3V and Hoverboard-TX to pin 9 and Hoverboard-RX to pin 8
//
//    PB6 (Hoverboard-TX) and PB7 (Hoverboard-RX) can handle 5V I/O-Level :-)
//
//    please share feedback to https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x
#define _DEBUG      // debug output to first hardware serial port
//#define DEBUG_RX    // additional hoverboard-rx debug output

#define ESP32       // comment out if using Arduino


//#define REMOTE_UARTBUS  // one serial bus to control them all :-)

#define MPU_Data

#define SEND_MILLIS 50   // send commands to hoverboard every SEND_MILLIS millisesonds

#include "util.h"
#include "hoverserial.h"

//#define BAUDRATE 19200   // 19200 is default on hoverboard side because Arudino Nano SoftwareSerial can not do 115200
#define BAUDRATE 115200

#ifndef LED_BUILTIN
  #define LED_BUILTIN 4
#endif

#ifdef ESP32
  //const int pinRX = 39, pinTX = 37;   // Wemos S2 Mini
  //const int pinRX = 16, pinTX = 17;    // Wemos Lolin32
  const int pinRX = 16, pinTX = 17;    // ESP32-2432S028
  
  #define oSerialHover Serial1    // ESP32
#else
  #include <SoftwareSerial.h>    // not compatible with RCReceiver because of interrupt conflicts.
  const int pinRX = 9, pinTX = 8;
  SoftwareSerial oSerialHover(pinRX,pinTX); // RX, TX 
  #define oSerialHover Serial    // Arduino
#endif

SerialHover2Server oHoverFeedback;

void setup()
{
  #ifdef _DEBUG
    Serial.begin(115200);
    Serial.println("Hello Hoverbaord Gen2.target.board :-)");
  #endif
  
  #ifdef ESP32
    // Serial interface, baud, RX GPIO, TX GPIO
    // Note: The GPIO numbers will not necessarily correspond to the
    // pin number printed on the PCB. Refer to your ESP32 documentation for pin to GPIO mappings.
    HoverSetupEsp32(oSerialHover,BAUDRATE,pinRX,pinTX); 
    
  #else
    HoverSetupArduino(oSerialHover,BAUDRATE);    //  8 Mhz Arduino Mini too slow for 115200 !!!
  #endif

  pinMode(LED_BUILTIN, OUTPUT);
}

uint8_t  iSendId = 0;   // only for UartBus
int iLog = -1;  // -1: print log of all slaves, 0: only print log of slaveId 0
int iMax = 500;   // sending a (flattened) zigzag curve with iMax amplitude. can be changed with 'max'
int iPeriod = 3;  // 3 = 3 seconds period of the zigzag curve

boolean CheckConsole()
{
   if (!Serial.available())  // if there is terminal data comming from user
    return false;
    
  String sReceived = Serial.readStringUntil('\n');
  Serial.println(sReceived);
  boolean bSend = false;
  int  iSendTo = -1;
  String sCmd = ShiftValue(sReceived, " ");

  #ifdef REMOTE_UARTBUS
    if (isUInt(sCmd))
    {
      iSendTo = sCmd.toInt();
      DEBUGT("send to",iSendTo)
      sCmd = ShiftValue(sReceived, " ");
    }
  #endif
    
  if ( (sCmd == "m") || (sCmd == "mode"))
  {
    int iMode = abs(ShiftValue(sReceived, "\n").toInt());
    if (iMode < 4)
    {
      oHoverConfig.iDriveMode = iMode;
      if (oHoverConfig.iDriveMode == 0) iMax = CLAMP(iMax,-1000,1000);
      bSend = true;
    }
  }
  else if ( (sCmd == "bl") || (sCmd == "batlow"))
  {
    oHoverConfig.fBattEmpty = ShiftValue(sReceived, "\n").toFloat();
    bSend = true;
  }
  else if ( (sCmd == "bh") || (sCmd == "bathi"))
  {
    oHoverConfig.fBattFull = ShiftValue(sReceived, "\n").toFloat();
    bSend = true;
  }
#ifdef REMOTE_UARTBUS
  else if ( (sCmd == "si") || (sCmd == "slave"))
  {
    oHoverConfig.iSlaveNew = ShiftValue(sReceived, "\n").toInt();
    bSend = true;
  }
#endif
  else if ( (sCmd == "l") || (sCmd == "log"))
  {
    iLog = ShiftValue(sReceived, "\n").toInt();
  }
  else if (sCmd == "max")
  {
    iMax = abs(ShiftValue(sReceived, "\n").toInt());
    if (oHoverConfig.iDriveMode == 0) iMax = CLAMP(iMax,-1001,1001);
  }
  else if (sCmd == "period")
  {
    iPeriod = CLAMP(abs(ShiftValue(sReceived, "\n").toInt()),1,30);
  }
  else
  {
    Serial.print("unkown command: "); 
  }
  Serial.print(sCmd); Serial.print("\t value:"); Serial.println(ShiftValue(sReceived, "\n"));
  
  if (bSend)
  {
    #ifdef REMOTE_UARTBUS
      for (int iTo=0; iTo<4; iTo++)
      {
        if (  (iSendTo<0) || (iSendTo == iTo)  )
        {
          oHoverConfig.iSlave = iTo;
          HoverSendData(oSerialHover,oHoverConfig);
          HoverLogConfig(oHoverConfig);
        }
      }
    #else
       HoverSendData(oSerialHover,oHoverConfig);
       HoverLogConfig(oHoverConfig);
    #endif
    return true;
  }
  return false;
}

unsigned long iLast = 0;
unsigned long iNext = 0;
unsigned long iTimeNextState = 3000;
uint8_t  wState = 1;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
void loop()
{
  unsigned long iNow = millis();
  digitalWrite(LED_BUILTIN, (iNow%1000) < 200);

  float fScaleMax = 1.6*(CLAMP(iPeriod,3,9)/9.0);  // to flatten the sine curve to send constant iMax for some time

  int iSpeed = CLAMP((fScaleMax*iMax/100) * (ABS( (int)((iNow/iPeriod+250) % 1000) - 500) - 250),-iMax,iMax);   // repeats from +300 to -300 to +300 :-)
  
  int iSteer = 1 * (ABS( (int)((iNow/400+100) % 400) - 200) - 100);   // repeats from +100 to -100 to +100 :-)
  

  if (iNow > iTimeNextState)
  {
    iTimeNextState = iNow + 3000;
    wState = wState << 1;
    if (wState == 64) wState = 1;  // remove this line to test Shutoff = 128
  }
  
  boolean bReceived;   
  while (bReceived = Receive(oSerialHover,oHoverFeedback))
  {
    #ifdef REMOTE_UARTBUS
      if (  (iLog < 0) || (iLog == oHoverFeedback.iSlave)  )
    #endif
    {
      DEBUGT("millis",iNow-iLast);
      DEBUGT("iSpeed",iSpeed);
      //DEBUGT("iSteer",iSteer);
      HoverLog(oHoverFeedback);
      iLast = iNow;
    }
  }

  if (iNow > iNext)
  {
    iNext = iNow + SEND_MILLIS/2;
    if (!CheckConsole())
    {
      //DEBUGT("sending iSpeed",iSpeed)
      #ifdef REMOTE_UARTBUS
          switch(iSendId++)
          {
          case 0: // left motor
            HoverSend(oSerialHover,0,CLAMP(iSpeed + iSteer,-iMax,iMax),wState);  // hoverboard will answer immediatly on having received this message ...
            break;
          case 1: // right motor
            HoverSend(oSerialHover,1,-CLAMP(iSpeed - iSteer,-iMax,iMax),wState);  // hoverboard will answer immediatly on having received this message ...
            iSendId = 0;
            break;
          }
        #else
          //if (bReceived)  // Reply only when you receive data
           HoverSend(oSerialHover,iSteer,iSpeed,wState,wState);
        #endif
      }
    }

}
