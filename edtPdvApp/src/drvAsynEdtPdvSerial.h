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

int drvAsynEdtPdvSerialPortConfigure(	const char	*	cameraName,
                               			unsigned int	priority,
										int				noAutoConnect,
                               			int				noProcessEos,
										PdvDev		*	pPdvDev );

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  /* DRVASYNEDTCAMERA_H */
