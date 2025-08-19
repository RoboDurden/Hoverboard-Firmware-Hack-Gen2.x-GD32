
#define REMOTE_BAUD 115200		// increase default baud rate from 19200 to higher speed

#define SEND_IMU_DATA // send the IMU data with RemoteUart or RemoteUartBus
//#define MPU_6050		// active this if you do not SEND_IMU_DATA but only need imu data for your pilot code



// Schmitt clip thresholds in raw ADC/LSB units (tune these)
#define CLIP_THR_HI      3      // enter +1 if (x - baseline) >= +10
#define CLIP_THR_LO     -3      // enter -1 if (x - baseline) <= -10

//void PilotInit();

void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave);
