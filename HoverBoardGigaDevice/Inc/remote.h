#ifndef REMOTE_H
#define REMOTE_H

	extern int16_t iConfigMode;	// 0 = default behavior, some RemotXY can enter config mode on BUTTON longpress at startup

	void RemoteCallback(void);		// must be implemented by all remotes
	void RemoteUpdate(void);			// must be implemented by all remotes


	#if defined(REMOTE_UART)
		#include "../Inc/remoteUart.h"
	#elif defined(REMOTE_UARTBUS)
		#include "../Inc/remoteUartBus.h"
	#elif defined(REMOTE_CRSF)
		#include "../Inc/remoteCrsf.h"
	#elif defined(REMOTE_DUMMY)
		#include "../Inc/remoteDummy.h"
	#elif defined(REMOTE_AUTODETECT)
		#include "../Inc/remoteAutodetect.h"
	#elif defined(REMOTE_ADC)
		#include "../Inc/remoteAdc.h"
	#endif

#endif