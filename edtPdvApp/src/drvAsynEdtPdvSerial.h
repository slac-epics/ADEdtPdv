/**********************************************************************
* Asyn device support using EDT framegrabber serial interface via CamLink 
**********************************************************************/

#ifndef DRVASYNEDTCAMERA_H
#define DRVASYNEDTCAMERA_H

#include "edtinc.h"

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Define additional pasynUser->auxStatus error masks */
//	#define ASYN_ERROR_PARITY  0x0001
//	#define ASYN_ERROR_FRAMING 0x0002

//
// This structure holds the hardware-specific information
// for a single asyn link.	There is one for each camera.
//
typedef struct ttyController
{
	char			*	portName;
	int					m_addr;
	bool				m_Connected;
	PdvDev			*	m_pPdvDev;
	unsigned long		nRead;
	unsigned long		nWritten;
	double				readTimeout;
	double				writeTimeout;
	asynInterface		common;
	asynInterface		octet;
	epicsMutexId		m_serialLock;
}	ttyController_t;

ttyController_t	*	drvAsynEdtPdvSerialPortConfigure(	const char		*	portName,
														unsigned int		priority,
														int					autoConnect,
														int					noProcessEos );

asynStatus drvAsynEdtPdvSerialPortConnect(				ttyController_t	*	tty,
														PdvDev			*	pPdvDev	);

asynStatus drvAsynEdtPdvSerialPortDisconnect(			ttyController_t	*	tty		);

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  /* DRVASYNEDTCAMERA_H */
