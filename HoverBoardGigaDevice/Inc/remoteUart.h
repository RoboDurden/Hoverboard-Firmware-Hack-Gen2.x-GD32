#ifndef REMOTE_UART_H
#define REMOTE_UART_H

#ifndef REMOTE_BAUD 
	#define REMOTE_BAUD 19200
#endif

#define SEND_INTERVAL_MS	100	// sending SerialHover2Server data every 100 ms to ESP32/Arduino

#define LOST_CONNECTION_STOP_MILLIS 500		// set speed to 0 when 500 ms no command received


// Only master communicates with steering device
#ifdef MASTER




#endif	// MASTER

#endif