#ifndef REMOTE_CANSIMPLE_H
#define REMOTE_CANSIMPLE_H

// ODrive CANSimple protocol carried over UART, as the MM32 controller uses it
// (Hoverboard-Firmware-Hack-Gen2.x-MM32, Src/cansimple.c). The message ids,
// payload layouts and units are ODrive's CANSimple (docs.odriverobotics.com,
// can-protocol); float32 in physical units: turns, turns/s, Nm, A, V.
//
// A CAN frame becomes a UART frame:
//    0xA5 | node id | cmd id (bit 7 = request, the CAN RTR bit) | length 0..8 | payload | CRC-16 (LE)
// CRC-16/XMODEM (poly 0x1021, init 0) over node id .. payload = CalcCRC() in comms.c.
// Node 63 is broadcast: every board acts on it, none answers.
//
// This GD32 firmware has no FOC / current loop, so the FOC-only messages are not
// accepted: Set_Pos_Gain, Set_Vel_Gains and RxSdo/TxSdo are ignored, and the
// unmeasurable Get_ values (Iq, motor temperature, torques) answer NaN. Supported:
// Get_Version, Heartbeat, Estop, Get_Error, Set_Axis_State, Get_Encoder_Estimates,
// Set_Controller_Mode, Set_Input_Pos/Vel/Torque, Set_Input_Voltage (0x1E, pwm),
// Get_Temperature, Get_Bus_Voltage_Current, Clear_Errors, Set_Absolute_Position,
// Get_Iq, Get_Torques, Get_Powers.

#ifndef REMOTE_BAUD
	#define REMOTE_BAUD 19200		// as the other UART remotes in this repo; override in config.h / defines_*.h
#endif

#define LOST_CONNECTION_STOP_MILLIS 500		// set bRemoteTimeout + speed 0 when 500 ms no command received

#ifndef SLAVE_ID
	#define SLAVE_ID 0		// this board's CANSimple node id (0..62); 63 = broadcast
#endif

// Get_Temperature reports the MCU's internal temperature sensor (ADC channel 16) on
// the GD32 families that have it; other targets answer NaN. See remoteCanSimple.c.
#if defined(GD32F130) || defined(GD32F103) || defined(GD32E230)
	#define MCU_TEMP
#endif

// firmware version reported by Get_Version
#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 0
#define FW_VERSION_REVISION 0

// control-command scaling to the firmware's `speed` (see driver.c / config.h DRIVING_MODE)
#define CANSIMPLE_REVS_SCALE   1024		// velocity: `speed` = turns/s * 1024   (iDrivingMode 1)
#define CANSIMPLE_TORQUE_SCALE 1024		// torque:   `speed` = Nm * 1024        (iDrivingMode 2)
#define CANSIMPLE_PWM_SCALE    1000		// voltage:  `speed` = fraction * 1000  (iDrivingMode 0, permille pwm)
#define HALL_STEPS_PER_TURN    90		// hall steps per motor revolution (one step = 4 deg) for position/turns

#endif	// REMOTE_CANSIMPLE_H
