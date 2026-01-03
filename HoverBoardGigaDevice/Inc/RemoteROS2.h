#ifndef REMOTE_ROS2_H
#define REMOTE_ROS2_H

#include <stdint.h>

#define STAND_STILL_THRESHOLD 10

#define REMOTE_BAUD 115200

#define SEND_INTERVAL_MS	100	// sending SerialFeedback data every 100 ms to ROS2

#define LOST_CONNECTION_STOP_MILLIS 500		// set speed to 0 when 500 ms no command received

#define IMU_LP 0 // Disable IMU Low-pass filter

#undef PILOT_DEFAULT
void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave);

#endif // REMOTE_ROS2_H
