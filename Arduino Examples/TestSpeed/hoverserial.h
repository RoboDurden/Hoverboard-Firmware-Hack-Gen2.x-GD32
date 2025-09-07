// hoverserial.h v20250813
/*
// Variables todo
uint8_t upperLEDMaster = 0;
uint8_t lowerLEDMaster = 0;
uint8_t mosfetOutMaster = 0;
uint8_t upperLEDSlave = 0;
uint8_t lowerLEDSlave = 0;
uint8_t mosfetOutSlave = 0;
uint8_t beepsBackwards = 0;
uint8_t activateWeakening = 0;
*/

template <typename O,typename I> void HoverSetupEsp32(O& oSerial, I iBaud, I gpio_RX, I gpio_TX)
{
  // Starts the serial connection using the baud, protocol, GPIO RX, GPIO TX.
  // These are the GPIO numbers; not necessarily the pin number printed on the PCB.
  oSerial.begin(iBaud, SERIAL_8N1, gpio_RX, gpio_TX);
}
template <typename O,typename I> void HoverSetupArduino(O& oSerial, I iBaud)
{
  oSerial.begin(iBaud);
}

uint16_t CalcCRC(uint8_t *ptr, int count)
{
  uint16_t  crc;
  uint8_t i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
      {
        crc = crc << 1 ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    } while(--i);
  }
  return (crc);
}


#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication

  template <typename O,typename D> void HoverSendData(O& oSerial, D& oData)
  {
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(oData)-2); // first bytes except crc
    oSerial.write((uint8_t*) &oData, sizeof(oData)); 
    //DEBUGN(oData.iSlave, sizeof(oData)); 
  }

  template <typename O,typename D> void HoverSendRawData(O& oSerial, D& oData)
  {
    oSerial.write((uint8_t*) &oData, sizeof(oData));
  }

#ifdef REMOTE_UARTBUS
  typedef struct __attribute__((packed, aligned(1))) {
     uint16_t cStart = START_FRAME;    //  = '/';
     uint8_t iSlave;    //  the slave id this message is sent from
     int16_t iSpeed;   // 100* km/h
     uint16_t iVolt;    // 100* V
     int16_t iAmp;   // 100* A
     int32_t iOdom;    // hall steps
     uint16_t checksum;
  } SerialHover2Server;
  
  //typedef struct{   // new version
  //   uint16_t cStart = START_FRAME;   // new version
  typedef struct __attribute__((packed, aligned(1))) {
     uint8_t  cStart = '/';
     uint8_t  iDataType = 0;    //  unique id for this data struct
     uint8_t  iSlave;       //  contains the slave id this message is intended for
     int16_t  iSpeed = 0;
     uint8_t  wState = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint16_t checksum;
  } SerialServer2Hover;
  
  typedef struct __attribute__((packed, aligned(1))) {
     uint8_t  cStart = '/';
     uint8_t  iDataType = 1;    //  unique id for this data struct
     uint8_t  iSlave;       //  contains the slave id this message is intended for
     int16_t  iSpeed = 0;
     int16_t  iSteer = 0;
     uint8_t  wState = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint8_t  wStateSlave = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint16_t checksum;
  } SerialServer2HoverMaster;
  
  typedef struct __attribute__((packed, aligned(1))) { 
     uint8_t cStart     = '/';      //  unique id for this data struct
     uint8_t  iDataType = 2;  //  unique id for this data struct
     uint8_t  iSlave;     //  contains the slave id this message is intended for
     float  fBattFull     = 42.0;    // 10s LiIon = 42.0;
     float  fBattEmpty    = 27.0;    // 10s LiIon = 27.0;
     uint8_t  iDriveMode  = 2;      //  MM32: 0=COM_VOLT, 1=COM_SPEED, 2=SINE_VOLT, 3=SINE_SPEED
     int8_t   iSlaveNew   = -1;      //  if >= 0 contains the new slave id saved to eeprom
     uint16_t checksum;
  } SerialServer2HoverConfig;


  template <typename O,typename I> void HoverSend(O& oSerial, uint8_t iSlave, I iSpeed, uint8_t  wState=32)
  {
    //DEBUGT("iSteer",iSteer);DEBUGN("iSpeed",iSpeed);
    SerialServer2Hover oData;
    oData.iSlave    = iSlave;
    oData.iSpeed    = (int16_t)iSpeed;
    oData.wState    = wState;
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover)-2); // first bytes except crc
    oSerial.write((uint8_t*) &oData, sizeof(SerialServer2Hover)); 
    //DebugOut((uint8_t*) &oData, sizeof(oData)); 
  }
  
  void HoverLog(SerialHover2Server& oData)
  {
    DEBUGT("iSlave",oData.iSlave);
    DEBUGT("iOdom",oData.iOdom);
    DEBUGT("\tiSpeed",(float)oData.iSpeed/100.0);
    DEBUGT("\tiAmp",(float)oData.iAmp/100.0);
    DEBUGN("\tiVolt",(float)oData.iVolt/100.0);
  }
#elif defined (REMOTE_ROS2)

  typedef struct __attribute__((packed, aligned(1))) {
    uint16_t start;         // START_FRAME 0xABCD
    int16_t  cmd1;          // Not used
    int16_t  cmd2;          // Not used
    int16_t  speedR_meas;   // Unit: RPM
    int16_t  speedL_meas;   // Unit: RPM
    int16_t  wheelR_cnt;    // Range: 0-ENCODER_MAX
    int16_t  wheelL_cnt;    // Range: 0-ENCODER_MAX
    int16_t  left_dc_curr;  // Unit: 0.01 A
    int16_t  right_dc_curr; // Unit: 0.01 A
    int16_t  batVoltage;    // Unit: 0.01 V
    int16_t  boardTemp;     // Unit: 0.1 degrees
    uint16_t cmdLed;        // Not used
    uint16_t checksum;      // checksum of all other attributes
  } SerialFeedback;
  typedef SerialFeedback SerialHover2Server;

  typedef struct __attribute__((packed, aligned(1))) {
   uint16_t start; // START_FRAME_PID 0xEFCD
   int16_t  kp;    // kp*1000  (fixed-point)
   int16_t  ki;    // ki*1000  (fixed-point)
   int16_t  kd;    // kd*10000 (fixed-point)
   int16_t  fs;    // FILTER_SHIFT [10..13]
   int16_t  iDrivingMode; //0=pwm, 1=speed in revs/s*1024, 2=torque in NewtonMeter*1024, 3=iOdometer
   uint16_t checksum;
  } SerialPidConfig;

  typedef struct __attribute__((packed, aligned(1))) {
   uint16_t start; // START_FRAME 0xABCD
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
  } SerialCommand;

  // Copy from REMOTE_UART (not supported to send yet, just for storing data locally on ESP32)
  typedef struct __attribute__((packed, aligned(1))) {
     uint8_t cStart     = '/';      //  unique id for this data struct
     uint8_t  iDataType = 2;  //  unique id for this data struct
     float  fBattFull     = 42.0;    // 10s LiIon = 42.0;
     float  fBattEmpty    = 27.0;    // 10s LiIon = 27.0;
     uint8_t  iDriveMode  = 0;      //  MM32: 0=COM_VOLT, 1=COM_SPEED, 2=SINE_VOLT, 3=SINE_SPEED
     uint16_t checksum;
  } SerialServer2HoverConfig;

  template <typename O,typename I> void HoverSend(O& oSerial, I iSteer, I iSpeed,uint8_t  wStateMaster=32, uint8_t  wStateSlave=0)
  {
    //DEBUGT("iSteer",iSteer);DEBUGN("iSpeed",iSpeed);
    SerialCommand oData;
    oData.start    = START_FRAME;
    oData.speed    = (int16_t)iSpeed;
    oData.steer    = (int16_t)iSteer;
    oData.checksum = (uint16_t)(oData.start ^ oData.steer ^ oData.speed);
    oSerial.write((uint8_t*) &oData, sizeof(SerialCommand));
    //DebugOut((uint8_t*) &oData, sizeof(oData));
  }

  void HoverLog(SerialHover2Server& oData)
  {
    /*
    unsigned long iNow = millis();
    DEBUGT("iNow",iNow);
    DEBUGT("cmd1",oData.cmd1);
    DEBUGT("cmd2",oData.cmd2);
    DEBUGT("speedR_meas",oData.speedR_meas);
    DEBUGT("speedL_meas",oData.speedL_meas);
    DEBUGT("wheelR_cnt",oData.wheelR_cnt);
    DEBUGT("wheelL_cnt",oData.wheelL_cnt);
    DEBUGT("left_dc_curr",oData.left_dc_curr);
    DEBUGT("right_dc_curr",oData.right_dc_curr);
    DEBUGT("batVoltage",oData.batVoltage);
    DEBUGT("boardTemp",oData.boardTemp);
    DEBUGN("cmdLed",oData.cmdLed);
    */

    DEBUGT("speed",oData.cmdLed);
    DEBUGT("iOdom",oData.wheelL_cnt);
    DEBUGT("revs32",oData.speedL_meas);//DEBUGT("revs32x1024",oData.speedL_meas);
    DEBUGT("inputFilterPwm",oData.cmd1);
    DEBUGT("outputFilterPwm",oData.cmd2);
    DEBUGT("revs32",oData.speedR_meas);//DEBUGT("revs32x4096",oData.speedR_meas);
    DEBUGT("sCSLog",oData.batVoltage);//DEBUGT("speedCounterSlowLog",oData.batVoltage);
    DEBUGT("pwmSlave",oData.right_dc_curr);
    DEBUGT("currentDC",oData.left_dc_curr);//DEBUGT("currentDC1000",oData.left_dc_curr);
    DEBUGT("buzzerTimer",oData.wheelR_cnt);
    DEBUGN("msTicks",oData.boardTemp);
  }


#else //REMOTE_UART

  typedef struct __attribute__((packed, aligned(1))) {
     uint16_t cStart = START_FRAME;    //  = '/';
     int16_t iSpeedL;   // 100* km/h
     int16_t iSpeedR;   // 100* km/h
     uint16_t iVolt;    // 100* V
     int16_t iAmpL;   // 100* A
     int16_t iAmpR;   // 100* A
     int32_t iOdomL;    // hall steps
     int32_t iOdomR;    // hall steps
    #ifdef MPU_Data
      int16_t     iGyroX;
      int16_t     iGyroY;
      int16_t     iGyroZ; 
      int16_t     iAccelX;
      int16_t     iAccelY;
      int16_t     iAccelZ;
      int16_t     iTemperature;
    #endif
     
     uint16_t checksum;
  } SerialHover2Server;
  
  //typedef struct{   // new version
  //   uint16_t cStart = START_FRAME;   // new version
  typedef struct __attribute__((packed, aligned(1))) {  // old version
     uint8_t cStart = '/';                              // old version
     int16_t  iSpeed = 0;
     int16_t  iSteer = 0;
     uint8_t  wStateMaster = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint8_t  wStateSlave = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
     uint16_t checksum;
  } SerialServer2Hover;

  typedef struct __attribute__((packed, aligned(1))) { 
     uint8_t cStart     = '/';      //  unique id for this data struct
     uint8_t  iDataType = 2;  //  unique id for this data struct
     float  fBattFull     = 42.0;    // 10s LiIon = 42.0;
     float  fBattEmpty    = 27.0;    // 10s LiIon = 27.0;
     uint8_t  iDriveMode  = 0;      //  MM32: 0=COM_VOLT, 1=COM_SPEED, 2=SINE_VOLT, 3=SINE_SPEED
     uint16_t checksum;
  } SerialServer2HoverConfig;

  template <typename O,typename I> void HoverSend(O& oSerial, I iSteer, I iSpeed,uint8_t  wStateMaster=32, uint8_t  wStateSlave=0)
  {
    //DEBUGT("iSteer",iSteer);DEBUGN("iSpeed",iSpeed);
    SerialServer2Hover oData;
    oData.iSpeed    = (int16_t)iSpeed;
    oData.iSteer    = (int16_t)iSteer;
    oData.wStateMaster  = wStateMaster;
    oData.wStateSlave   = wStateSlave;
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover)-2); // first bytes except crc
    oSerial.write((uint8_t*) &oData, sizeof(SerialServer2Hover)); 
    //DebugOut((uint8_t*) &oData, sizeof(oData)); 
  }
  
  template <typename O,typename I> void HoverSendLR(O& oSerial, I iSpeedLeft, I iSpeedRight) // -1000 .. +1000
  {
    // speed coeff in config.h must be 1.0 : (DEFAULT_)SPEED_COEFFICIENT   16384
    // steer coeff in config.h must be 0.5 : (DEFAULT_)STEER_COEFFICIENT   8192 
    HoverSend(oSerial,iSpeedRight - iSpeedLeft,(iSpeedLeft + iSpeedRight)/2);
  }
  
  void HoverLog(SerialHover2Server& oData)
  {
    DEBUGT("iOdomL",oData.iOdomL);
    DEBUGT("\tiOdomR",oData.iOdomR);
    DEBUGT("\tiSpeedL",(float)oData.iSpeedL/100.0);
    DEBUGT(" iSpeedR",(float)oData.iSpeedR/100.0);
    DEBUGT("\tiAmpL",(float)oData.iAmpL/100.0);
    DEBUGT(" iAmpR",(float)oData.iAmpR/100.0);
    DEBUGN("\tiVolt",(float)oData.iVolt/100.0);
    #ifdef MPU_Data
      DEBUGT("iGyroX",oData.iGyroX);
      DEBUGT("iGyroY",oData.iGyroY);
      DEBUGT("iGyroZ",oData.iGyroZ); 
      DEBUGT("iAccelX",oData.iAccelX);
      DEBUGT("iAccelY",oData.iAccelY);
      DEBUGT("iAccelZ",oData.iAccelZ);
      DEBUGN("iTemperature",oData.iTemperature);
    #endif
    
  }

  void HoverDebug(SerialHover2Server& oData)
  {
    DEBUGTX("0",oData.iVolt);
    DEBUGTB("1",oData.iAmpL);
    DEBUGTB("2",oData.iAmpR);
    DEBUGTB("3",oData.iSpeedL);
    DEBUGN("4",oData.iSpeedR);
  }

  void HoverDebug2(SerialHover2Server& oData)
  {
    DEBUGT("0",oData.iVolt);
    DEBUGT("1",oData.iAmpL);
    DEBUGT("2",oData.iAmpR);
    DEBUGT("3",oData.iSpeedL);
    DEBUGT("4",oData.iSpeedR);
    DEBUGT("5",oData.iOdomL);
    DEBUGN("6",oData.iOdomR);
  }

#endif

void HoverLogConfig(SerialServer2HoverConfig& oConfig)
{
  #ifdef REMOTE_UARTBUS
    DEBUGT("config for iSlave",oConfig.iSlave);
    DEBUGT("iSlaveNew",oConfig.iSlaveNew);
  #endif
  DEBUGT("fBattFull",oConfig.fBattFull);
  DEBUGT("fBattEmpty",oConfig.fBattEmpty);
  DEBUGN("iDriveMode",oConfig.iDriveMode);
}


SerialServer2HoverConfig oHoverConfig;




void DebugOut(uint8_t aBuffer[], uint8_t iSize)
{
  for (int i=0; i<iSize; i++)
  {
    uint8_t c = aBuffer[i];
    Serial.print((c < 16) ? " 0" : " ");Serial.print(c,HEX); 
  }
  Serial.println();
}


  

#ifdef DEBUG_RX
  unsigned long iLastRx = 0;
#endif

//boolean Receive(Serial& oSerial, SerialFeedback& Feedback)
template <typename O,typename OF> boolean Receive(O& oSerial, OF& Feedback)
{
  int iTooMuch = oSerial.available() - sizeof(SerialHover2Server) + 1;
  int8_t bFirst = 1;
  while (iTooMuch >= bFirst )
  {
    byte c = oSerial.read();  // Read the incoming byte
    iTooMuch--;

    #ifdef DEBUG_RX
      //if (millis() > iLastRx + 50)  Serial.println();
      Serial.print((c < 16) ? " 0" : " ");Serial.print(c,HEX); 
      iLastRx = millis();
    #endif
    
    if (bFirst) // test first START byte
    {
      if (c == (byte)START_FRAME) //if (c == 0xCD)
      {
        bFirst = 0;
      }
    }
    else  // test second START byte
    {
      if (c == START_FRAME >>8 ) //if (c == 0xAB)
      {
        //DEBUGT(" avail",oSerial.available())
        SerialHover2Server tmpFeedback;
        byte* p = (byte *)&tmpFeedback+2; // start word already read
        for (int i = sizeof(SerialHover2Server); i>2; i--)  
          *p++    = oSerial.read();

        //while(oSerial.available()) oSerial.read();
        #ifdef DEBUG_RX
          //Serial.print(" -> ");
          //HoverLog(tmpFeedback);
        #endif

        #ifdef REMOTE_ROS2
        uint16_t checksum = START_FRAME ^  tmpFeedback.cmd1 ^ tmpFeedback.cmd2 ^ tmpFeedback.speedR_meas ^ tmpFeedback.speedL_meas ^
          tmpFeedback.wheelR_cnt ^ tmpFeedback.wheelL_cnt ^ tmpFeedback.left_dc_curr ^ tmpFeedback.right_dc_curr ^ tmpFeedback.batVoltage ^
          tmpFeedback.boardTemp ^ tmpFeedback.cmdLed;
        #else
        uint16_t checksum = CalcCRC((byte *)&tmpFeedback, sizeof(SerialHover2Server)-2);
        #endif
        if (checksum == tmpFeedback.checksum)
        {
            memcpy(&Feedback, &tmpFeedback, sizeof(SerialHover2Server));
            #ifdef DEBUG_RX
              Serial.println(" :-)");
            #endif
            return true;
        }
        #ifdef DEBUG_RX
          Serial.print(tmpFeedback.checksum, HEX);
          Serial.print(" != ");
          Serial.print(checksum,HEX);
          Serial.println(" :-(");
        #endif
        return false;       
      }
      if (c != (byte)START_FRAME) //if (c != 0xCD)
        bFirst = 1;
    }
  }
  return false;
}
