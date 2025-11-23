// ros2serial.h

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

#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication

  template <typename O,typename D> void HoverSendData(O& oSerial, D& oData)
  {
    //oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(oData)-2); // first bytes except crc
    oSerial.write((uint8_t*) &oData, sizeof(oData)); 
  }

  template <typename O,typename D> void HoverSendRawData(O& oSerial, D& oData)
  {
    oSerial.write((uint8_t*) &oData, sizeof(oData));
  }

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
   uint16_t start; // START_FRAME 0xABCD
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
  } SerialCommand;

  // Copy from TestSpeed.ino (not supported to send, just for storing data locally on ESP32)
  typedef struct __attribute__((packed, aligned(1))) {
     uint8_t cStart     = '/';      //  unique id for this data struct
     uint8_t  iDataType = 2;  //  unique id for this data struct
     float  fBattFull     = 42.0;    // 10s LiIon = 42.0;
     float  fBattEmpty    = 27.0;    // 10s LiIon = 27.0;
     uint8_t  iDriveMode  = 0;      //  MM32: 0=COM_VOLT, 1=COM_SPEED, 2=SINE_VOLT, 3=SINE_SPEED
     uint16_t checksum;
  } SerialServer2HoverConfig;

  template <typename O,typename I> void HoverSend(O& oSerial, I iSteer, I iSpeed)
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
  }

void HoverLogConfig(SerialServer2HoverConfig& oConfig)
{
  DEBUGT("fBattFull",oConfig.fBattFull);
  DEBUGT("fBattEmpty",oConfig.fBattEmpty);
  DEBUGN("iDriveMode",oConfig.iDriveMode);
}

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

        uint16_t checksum = START_FRAME ^  tmpFeedback.cmd1 ^ tmpFeedback.cmd2 ^ tmpFeedback.speedR_meas ^ tmpFeedback.speedL_meas ^
          tmpFeedback.wheelR_cnt ^ tmpFeedback.wheelL_cnt ^ tmpFeedback.left_dc_curr ^ tmpFeedback.right_dc_curr ^ tmpFeedback.batVoltage ^
          tmpFeedback.boardTemp ^ tmpFeedback.cmdLed;

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
