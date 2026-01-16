#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/comms.h"
#include "../Inc/bldc.h"
#include "stdio.h"
#include "string.h"

#ifdef REMOTE_ROS2

#ifdef IMU_ENABLE
  #define SEND_IMU_DATA
  #include "../Inc/mpu6050.h"
  extern MPU_Data mpuData;
#endif

#pragma pack(1)

// Only master communicates with steerin device
#ifdef MASTER_OR_SINGLE

static int16_t iReceivePos = -1;

// From setup.c:
extern uint8_t usart0_rx_buf[1];
extern uint8_t usart1_rx_buf[1];
extern uint8_t usart2_rx_buf[1];

// From main.c:
extern uint8_t bRemoteTimeout; 	// any Remote can set this to 1 to disable motor (with soft brake)
extern int32_t steer;
extern int32_t speed;
extern DataSlave oDataSlave;

// From bldc.c:
extern int32_t revs32;
extern int32_t iOdom;
extern float batteryVoltage; 							// global variable for battery voltage
extern float currentDC; 									// global variable for current dc
extern float realSpeed; 									// global variable for real Speed
extern int32_t bldc_inputFilterPwm;
extern int32_t bldc_outputFilterPwm;

// from https://github.com/alex-makarov/hoverboard-firmware-hack-FOC/blob/master/Src/bldc.c
// Changed to input int32_t since iOdom is int32_t.
int16_t modulo32(int32_t m, int16_t rest_classes)
{
  return (((m % rest_classes) + rest_classes) %rest_classes);
}

#define START_FRAME 0xABCD
#define START_FRAME_MSB 0xAB
#define START_FRAME_LSB 0xCD

typedef struct {
   uint16_t start; // START_FRAME 0xABCD
   int16_t  steer; // Difference between left and right wheel speed. Unit: RPM
   int16_t  speed; // Average of left and right wheel speed. Unit: RPM
   uint16_t checksum;
} SerialCommand;

static uint8_t aReceiveBuffer[sizeof(SerialCommand)];

#define ENCODER_MAX 9000 // Max value of wheelR_cnt/wheelL_cnt before it wraps to 0

typedef struct {
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


#define START_FRAME_IMU 0xACDC

typedef struct {
	uint16_t start; // START_FRAME_IMU=0xACDC
	uint16_t imuId; // 0=imu0(Master board), 1=imu1(Slave board)
	int16_t accelX;
	int16_t accelY;
	int16_t accelZ;
	int16_t gyroX;
	int16_t gyroY;
	int16_t gyroZ;
	uint16_t checksum;
} SerialImu;

uint32_t iTimeLastRx = 0;
uint32_t iTimeNextTx = 0;

// Send frame to steer device
// Called from main() every 2*DELAY_IN_MAIN_LOOP = 10ms
void RemoteUpdate(void)
{
	if (millis() - iTimeLastRx > LOST_CONNECTION_STOP_MILLIS)
	{
		bRemoteTimeout = 1;
		speed = steer = 0;
	}

#ifdef SEND_IMU_DATA
	if (MPU_ReadAll() == SUCCESS)
	{
		//Send IMU data to ROS2
		SerialImu imu;
		imu.start = START_FRAME_IMU;
		imu.imuId = 0; // 0=imu0(Master board), 1=imu1(Slave board)
		imu.accelX = mpuData.accel.x;
		imu.accelY = mpuData.accel.y;
		imu.accelZ = mpuData.accel.z;
		imu.gyroX = mpuData.gyro.x;
		imu.gyroY = mpuData.gyro.y;
		imu.gyroZ = mpuData.gyro.z;

		imu.checksum = (uint16_t)(
		  imu.start ^
		  imu.imuId ^
		  imu.accelX ^
		  imu.accelY ^
		  imu.accelZ ^
		  imu.gyroX ^
		  imu.gyroY ^
		  imu.gyroZ);

		SendBuffer(USART_REMOTE, (uint8_t*) &imu, sizeof(imu));
	}
#endif

	if (millis() < iTimeNextTx)	
		return;
	iTimeNextTx = millis() + SEND_INTERVAL_MS; // Called every 10ms so precision is ~10ms
	
	//Send feedback to ROS2
	SerialFeedback feedback;
	feedback.start = START_FRAME;
	feedback.cmd1 = bldc_inputFilterPwm;  // Not used by ROS2 hoverboard-driver
	feedback.cmd2 = bldc_outputFilterPwm; // Not used by ROS2 hoverboard-driver
	feedback.cmdLed = revs32;             // Not used by ROS2 hoverboard-driver
	feedback.batVoltage = (int16_t) (batteryVoltage * 100); // Unit: 0.01 V
	feedback.boardTemp = 250; // Dummy value (250 => 25 degrees)
	feedback.speedL_meas = (int16_t) (realSpeed * 1000); // TODO: Base on revs32/revs32x instead! 1000 is just random scale-factor...
	feedback.wheelL_cnt = modulo32(iOdom, ENCODER_MAX);
	feedback.left_dc_curr = (int16_t) (currentDC * 100);

#ifdef MASTER
	feedback.speedR_meas = (int16_t) (oDataSlave.realSpeed * 1000); // TODO: Base on revs32/revs32x instead! 1000 is just random scale-factor...
	feedback.wheelR_cnt = modulo32(oDataSlave.iOdom, ENCODER_MAX);
	feedback.right_dc_curr = (int16_t) (oDataSlave.currentDC * 100);
#else // SINGLE (SLAVE not possible due to ifdef MASTER_OR_SINGLE at top of this file)
	feedback.speedR_meas = 0;
	feedback.wheelR_cnt = 0;
	feedback.right_dc_curr = 0;
#endif

	feedback.checksum = (uint16_t)(
		feedback.start ^
		feedback.cmd1 ^
		feedback.cmd2 ^
		feedback.speedR_meas ^
		feedback.speedL_meas ^
		feedback.wheelR_cnt ^
		feedback.wheelL_cnt ^
		feedback.left_dc_curr ^
		feedback.right_dc_curr ^
		feedback.batVoltage ^
		feedback.boardTemp ^
		feedback.cmdLed);
	SendBuffer(USART_REMOTE, (uint8_t*) &feedback, sizeof(feedback));
}


// Update USART steer input
void RemoteCallback(void)
{
	uint8_t cRead = USART_REMOTE_BUFFER[0];

	if (iReceivePos<0) {
		// Protocol is in little-endian byte order, i.e. LSB is sent before MSB on wire.
		// (little-endian is used by eg. STM32F103, Raspberry Pi, Intel (x86/x86-64), ESP32, etc.)
		if (cRead == START_FRAME_LSB) { // First Start character is captured
			iReceivePos=0;
		}
	}
	else if (iReceivePos==1) {
		if (cRead != START_FRAME_MSB) { // Second Start character NOT captured, start over
			iReceivePos=-1;
		}
	}

	// Data reading has begun, save in aReceiveBuffer until whole message is received, then compare checksum.
	if (iReceivePos >= 0) {
		aReceiveBuffer[iReceivePos++] = cRead;
		if (iReceivePos == sizeof(SerialCommand))
		{
			SerialCommand* pData = (SerialCommand*) aReceiveBuffer;
			uint16_t checksum = (uint16_t)(pData->start ^ pData->steer ^ pData->speed);
			if (pData->checksum == checksum)
			{
				iTimeLastRx = millis();
				bRemoteTimeout = 0;

				speed = pData->speed;
				steer = pData->steer;

				ResetTimeout();	// Reset the pwm timout to avoid stopping motors
			}
			iReceivePos = -1;
		}
	}	
}


#endif // MASTER_OR_SINGLE

#if DRIVING_MODE != 1
  #error "RemoteROS2 assumes DRIVING_MODE == 1"
#endif
// Outputs pPwmMaster and pPwmSlave in revs/s*1024 (fixed-point with 10 bit fraction)
// speed and steer in RPM unit (some kind of average and difference between left and right wheel speed)
// Reverse of hoverboard-driver code: https://github.com/hoverboard-robotics/hoverboard-driver/blob/humble/hardware/hoverboard_driver.cpp#L465
void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave)
{
	*pPwmMaster = ( (speed + (steer/2)) *1024) / 60;	// Divide by 60 due to RPM to revs(revolution per second)
	*pPwmSlave  = ( (speed - (steer/2)) *1024) / 60;	// Divide by 60 due to RPM to revs(revolution per second)
}


#endif // REMOTE_ROS2