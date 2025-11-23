#ifndef REMOTE_ROS2_H
#define REMOTE_ROS2_H

#include <stdint.h>

#define STAND_STILL_THRESHOLD 10

#define REMOTE_BAUD 19200

#define SEND_INTERVAL_MS	100	// sending SerialFeedback data every 100 ms to ROS2

#define LOST_CONNECTION_STOP_MILLIS 500		// set speed to 0 when 500 ms no command received

#undef PILOT_DEFAULT
void 	Pilot(int16_t* pPwmMaster, int16_t* pPwmSlave);

#endif // REMOTE_ROS2_H
