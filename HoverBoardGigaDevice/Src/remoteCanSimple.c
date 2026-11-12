// ODrive CANSimple protocol over UART for the GD32 firmware, modelled on the
// MM32 controller's Src/cansimple.c but reduced to what this firmware can do:
// speed / torque / position / pwm commands, arm/disarm and telemetry. There is
// no FOC here, so the current-loop messages are not accepted and the values that
// cannot be measured are answered as NaN. See Inc/remoteCanSimple.h for the frame
// format and the full list of supported messages.

#include "../Inc/defines.h"
#include "../Inc/it.h"
#include "../Inc/comms.h"
#include "../Inc/bldc.h"
#include "string.h"

#ifdef REMOTE_CANSIMPLE

#define START 0xA5
#define BROADCAST 0x3F		// node 63

// ODrive CANSimple command ids (cmd = frame byte 2, bit 7 = request)
enum{
	CMD_GET_VERSION = 0x00, CMD_HEARTBEAT = 0x01, CMD_ESTOP = 0x02, CMD_GET_ERROR = 0x03,
	CMD_SET_AXIS_STATE = 0x07, CMD_GET_ENCODER_ESTIMATES = 0x09,
	CMD_SET_CONTROLLER_MODE = 0x0B, CMD_SET_INPUT_POS = 0x0C, CMD_SET_INPUT_VEL = 0x0D,
	CMD_SET_INPUT_TORQUE = 0x0E, CMD_SET_LIMITS = 0x0F,
	CMD_GET_IQ = 0x14, CMD_GET_TEMPERATURE = 0x15,
	CMD_GET_BUS_VOLTAGE_CURRENT = 0x17, CMD_CLEAR_ERRORS = 0x18,
	CMD_SET_ABSOLUTE_POSITION = 0x19,
	CMD_GET_TORQUES = 0x1C, CMD_GET_POWERS = 0x1D, CMD_SET_INPUT_VOLTAGE = 0x1E,
};

// ODrive axis states / control modes
enum{ AXIS_IDLE = 1, AXIS_CLOSED_LOOP = 8 };
enum{ CM_VOLTAGE = 0, CM_TORQUE = 1, CM_VELOCITY = 2, CM_POSITION = 3 };

// the per-USART rx byte buffers (setup.c); USART_REMOTE_BUFFER aliases the selected one
extern uint8_t usart0_rx_buf[1];
extern uint8_t usart1_rx_buf[1];
extern uint8_t usart2_rx_buf[1];

// shared firmware globals (as the other remotes declare them)
extern uint8_t bRemoteTimeout; 	// any Remote can set this to 1 to disable motor (with soft brake)
extern uint8_t iDrivingMode;	//  0=pwm, 1=speed revs/s*1024, 2=torque Nm*1024, 3=iOdometer
extern int32_t speed;			// the command handed to SetBldcInput (meaning depends on iDrivingMode)
extern int32_t steer;
extern int32_t iOdom;			// position in hall steps
extern float batteryVoltage;
extern float currentDC;
extern float realSpeed;			// revs/s

static uint32_t iTimeLastRx = 0;
static uint8_t  armed = 0;		// ODrive axis state: 1 = closed loop, 0 = idle
static int32_t  posOffset = 0;	// iOdom at turns == 0 (Set_Absolute_Position)

static float nanF(void){ return __builtin_nanf(""); }
static float rdF(const uint8_t* p){ float f; memcpy(&f, p, 4); return f; }
static uint32_t rdU32(const uint8_t* p){ return p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static void wrF(uint8_t* p, float f){ memcpy(p, &f, 4); }
static void wrU32(uint8_t* p, uint32_t v){ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static int32_t clampF(float f, int32_t lim){ return f>lim ? lim : (f<-lim ? -lim : (int32_t)(f>=0 ? f+0.5f : f-0.5f)); }

static float posTurns(void){ return (float)(iOdom-posOffset)/(float)HALL_STEPS_PER_TURN; }

#ifdef MCU_TEMP
// MCU internal temperature sensor (ADC channel 16). It is read through the ADC
// *inserted* group, software-triggered once per RemoteUpdate (~10 ms): the regular
// DMA scan that feeds the motor control loop (v_batt, current) is left untouched,
// the inserted group only briefly borrows the ADC for one 239.5-cycle conversion.
// Uncalibrated, roughly +-5 C: datasheet-typical V25 / slope, Vref assumed 3.3 V.
#define ADC_VREF   3.3f
#define TEMP_V25   1.45f		// sensor voltage at 25 C (typ)
#define TEMP_SLOPE 0.0041f		// V per C (typ 4.1 mV/C)
static uint16_t tempRaw = 0;
static uint8_t  tempStarted = 0;
static void mcuTempSample(void)		// read the previous conversion, start the next
{
	static uint8_t inited = 0;
	if(!inited){
		adc_tempsensor_vrefint_enable();		// setup.c had disabled the sensor
		TARGET_adc_channel_length_config(ADC_INSERTED_CHANNEL, 1);
		TARGET_adc_inserted_channel_config(0, ADC_CHANNEL_16, ADC_SAMPLETIME_239POINT5);
		TARGET_adc_external_trigger_source_config(ADC_INSERTED_CHANNEL, TARGET_ADC_EXTTRIG_INSERTED_SW);
		TARGET_adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
		inited = 1;
	}
	if(tempStarted) tempRaw = TARGET_adc_inserted_data_read(ADC_INSERTED_CHANNEL_0);
	tempStarted = 1;
	TARGET_adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
}
static float mcuTempC(void)
{
	float v = (float)tempRaw * (ADC_VREF/4095.0f);
	return (TEMP_V25 - v)/TEMP_SLOPE + 25.0f;
}
#endif

//---------------------------------------------------------------------------- send a reply frame
static void reply(uint8_t cmd, const uint8_t* data, uint8_t len)
{
	uint8_t f[14];
	f[0] = START; f[1] = (uint8_t)SLAVE_ID; f[2] = cmd; f[3] = len;
	memcpy(&f[4], data, len);
	uint16_t c = CalcCRC(&f[1], 3+len);		// CRC-16/XMODEM over node id .. payload
	f[4+len] = (uint8_t)c; f[5+len] = (uint8_t)(c>>8);
	SendBuffer(USART_REMOTE, f, (uint8_t)(6+len));
}

//---------------------------------------------------------------------------- arm / disarm
static uint8_t setAxisState(uint32_t st)
{
	if(st==AXIS_IDLE){ armed = 0; bRemoteTimeout = 1; speed = 0; }
	else if(st==AXIS_CLOSED_LOOP){ armed = 1; bRemoteTimeout = 0; }
	return armed;
}

// ODrive control mode -> this firmware's iDrivingMode (DriverInit re-tunes the PID)
static void setControlMode(uint32_t cm)
{
	static const uint8_t map[4] = { 0, 2, 1, 3 };	// voltage->pwm, torque->torque, velocity->speed, position->iOdom
	if(cm>3) return;
	speed = 0;				// do not carry a stale command into the new mode
	DriverInit(map[cm]);
}

static void inputArrived(void)		// a set point arrived: feed the timeout watchdog
{
	iTimeLastRx = millis();
	if(armed){ bRemoteTimeout = 0; ResetTimeout(); }
}

// Set_Input_Vel / _Torque / _Voltage (k 0,1,2): turns/s, Nm, fraction -> the `speed` command,
// taken only in the matching driving mode; returns 1 if taken
static uint8_t inputCmd(uint8_t k, float f)
{
	static const uint8_t inMode[3] = { 1, 2, 0 };	// required iDrivingMode for vel / torque / voltage
	if(iDrivingMode != inMode[k]) return 0;
	if(k==0)      speed = clampF(f*(float)CANSIMPLE_REVS_SCALE,   32767);
	else if(k==1) speed = clampF(f*(float)CANSIMPLE_TORQUE_SCALE, 32767);
	else          speed = clampF(f*(float)CANSIMPLE_PWM_SCALE,    CANSIMPLE_PWM_SCALE);
	return 1;
}

static void setInputPos(float turns)		// position set point, turns (iDrivingMode 3)
{
	if(iDrivingMode != 3) return;
	speed = clampF(turns*(float)HALL_STEPS_PER_TURN + (float)posOffset, 2000000000);
}

static void setAbsolutePosition(float turns)	// the wheel is at `turns` now
{
	posOffset = iOdom - clampF(turns*(float)HALL_STEPS_PER_TURN, 2000000000);
}

//---------------------------------------------------------------------------- Get_ payloads
// returns the payload length, or 0xFF if the command is not a readable one
static uint8_t getMsg(uint8_t cmd, uint8_t* d)
{
	switch(cmd)
	{
		case CMD_GET_VERSION:
			d[0] = 2; d[1] = 0; d[2] = 0; d[3] = 0;		// protocol 2, hardware unknown
			d[4] = FW_VERSION_MAJOR; d[5] = FW_VERSION_MINOR; d[6] = FW_VERSION_REVISION; d[7] = 1;
			return 8;
		case CMD_HEARTBEAT:
			wrU32(d, 0);								// no axis error
			d[4] = armed ? AXIS_CLOSED_LOOP : AXIS_IDLE;
			d[5] = 0; d[6] = 0; d[7] = 0;
			return 8;
		case CMD_GET_ERROR:
			wrU32(d, 0); wrU32(d+4, 0);
			return 8;
		case CMD_GET_ENCODER_ESTIMATES:
			wrF(d, posTurns());							// turns
			wrF(d+4, realSpeed);						// turns/s (= revs/s)
			return 8;
		case CMD_GET_IQ:								// no current loop
			wrF(d, nanF()); wrF(d+4, nanF());
			return 8;
		case CMD_GET_TEMPERATURE:
			#ifdef MCU_TEMP
				wrF(d, mcuTempC());					// fet = MCU internal sensor
			#else
				wrF(d, nanF());						// no sensor on this target
			#endif
			wrF(d+4, nanF());						// motor temperature: no sensor
			return 8;
		case CMD_GET_BUS_VOLTAGE_CURRENT:
			wrF(d, batteryVoltage); wrF(d+4, currentDC);
			return 8;
		case CMD_GET_TORQUES:							// no FOC torque estimate
			wrF(d, nanF()); wrF(d+4, nanF());
			return 8;
		case CMD_GET_POWERS:
			wrF(d, batteryVoltage*currentDC);			// electrical power, W
			wrF(d+4, nanF());							// mechanical power unknown
			return 8;
	}
	return 0xFF;
}

//---------------------------------------------------------------------------- dispatch one frame
static void handle(uint8_t node, uint8_t cmdByte, const uint8_t* d, uint8_t len)
{
	uint8_t cmd = cmdByte&0x1F, request = cmdByte&0x80;
	uint8_t answer = node!=BROADCAST;
	uint8_t out[8];

	if(request){
		uint8_t n = getMsg(cmd, out);
		if(n!=0xFF && answer) reply(cmd, out, n);
		return;
	}
	switch(cmd)
	{
		case CMD_ESTOP:
			armed = 0; bRemoteTimeout = 1; speed = 0;
			break;
		case CMD_SET_AXIS_STATE:
			if(len>=4){ if(setAxisState(rdU32(d))) ResetTimeout(); }
			break;
		case CMD_SET_CONTROLLER_MODE:
			if(len>=8) setControlMode(rdU32(d));		// input_mode (d+4) ignored
			break;
		case CMD_SET_INPUT_POS:
			if(len>=4){ setInputPos(rdF(d)); inputArrived(); }
			break;
		case CMD_SET_INPUT_VEL: case CMD_SET_INPUT_TORQUE: case CMD_SET_INPUT_VOLTAGE:
			if(len>=4){
				inputCmd(cmd==CMD_SET_INPUT_VEL ? 0 : (cmd==CMD_SET_INPUT_TORQUE ? 1 : 2), rdF(d));
				inputArrived();
			}
			break;
		case CMD_CLEAR_ERRORS:
			bRemoteTimeout = 0;
			break;
		case CMD_SET_ABSOLUTE_POSITION:
			if(len>=4) setAbsolutePosition(rdF(d));
			break;
		// Set_Limits, Set_Pos_Gain, Set_Vel_Gains, RxSdo, Reboot: not supported, ignored
	}
}

//---------------------------------------------------------------------------- byte-by-byte parser
void RemoteCallback(void)
{
	static uint8_t buf[14];
	static uint8_t pos = 0;
	uint8_t c = USART_REMOTE_BUFFER[0];

	if(pos==0){
		if(c!=START) return;		// wait for a start byte
		buf[pos++] = c;
		return;
	}
	buf[pos++] = c;
	if(pos==4 && buf[3]>8){ pos = 0; return; }		// length out of range: resync
	if(pos<4 || pos < 6+buf[3]) return;				// still collecting

	uint8_t len = buf[3];
	pos = 0;
	uint16_t crc = (uint16_t)(buf[4+len] | buf[5+len]<<8);
	if(crc != CalcCRC(&buf[1], 3+len)) return;
	if(buf[1]!=SLAVE_ID && buf[1]!=BROADCAST) return;
	handle(buf[1], buf[2], &buf[4], len);
}

//---------------------------------------------------------------------------- periodic timeout check
void RemoteUpdate(void)
{
	#ifdef MCU_TEMP
		mcuTempSample();		// refresh the MCU temperature (~10 ms cadence)
	#endif
	if(millis() - iTimeLastRx > LOST_CONNECTION_STOP_MILLIS){
		bRemoteTimeout = 1;		// soft brake
		speed = 0;
	}
}

#endif	// REMOTE_CANSIMPLE
