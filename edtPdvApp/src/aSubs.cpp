#if 0
#include <iostream>
#include <iomanip>
#include <math.h>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <sstream>
#endif

#include <iocsh.h>
#include <registryFunction.h>
#include <epicsExport.h>
// #include <epicsThread.h>
// #include <dbFldTypes.h>
#include <aSubRecord.h>
// #include <dbAddr.h>
// #include <dbAccess.h>
// #include <dbScan.h>
// #include <recGbl.h>

using namespace		std;

#if 0
extern "C" long Up900Shutter_Init(	aSubRecord	*	pSub	)
{
	static	bool	isInitialized	= false;

	// No need to do this more than once
	if ( isInitialized )
		return 0;

	isInitialized	= true;

	return 0;
}
#endif


//	Up900Shutter_Process
//	Computes EdtAcquireTime_RBV and EdtTriggerMode_RBV from UP900 raw shutter values
//	Inputs:
//		A:	LONG, Up900ReadRawShutter
//		B:	LONG, Up900ShutterMode
//
//	Outputs
//		A:	DOUBLE, EdtAcquireTime_RBV, sec
//		B:	LONG,	EdtTriggerMode_RBV,	mbbo
//
extern "C" long Up900Shutter_Process( aSubRecord	*	pSub	)
{
	int			status		= 0;
	double		acquireTime	= 0;

	// Get input value pointers
	epicsInt32	*	pRawShutterVal	= static_cast<epicsInt32 *>( pSub->a );
	epicsInt32	*	pShutterModeVal	= static_cast<epicsInt32 *>( pSub->b );
	double		*	pPulseWidthVal	= static_cast<double	 *>( pSub->c );
	double		*	pPulseWidthScale= static_cast<double	 *>( pSub->d );
	
	// Get output value pointers
	double		*	pAcquireTimeVal	= static_cast<double *>(		pSub->vala );
	epicsInt32	*	pTriggerModeVal	= static_cast<epicsInt32 *>(	pSub->valb );
	
	if ( pShutterModeVal == NULL )
		return 0;
	if ( *pShutterModeVal == 0 )
	{
		//	NM, Normal FreeRun Mode
		switch ( *pRawShutterVal )
		{
			default:
			case 0:		acquireTime	= 1.0/15;		break;
			case 1:		acquireTime	= 1.0/30;		break;
			case 2:		acquireTime	= 1.0/60;		break;
			case 3:		acquireTime	= 1.0/125;		break;
			case 4:		acquireTime	= 1.0/250;		break;
			case 5:		acquireTime	= 1.0/500;		break;
			case 6:		acquireTime	= 1.0/1000;		break;
			case 7:		acquireTime	= 1.0/2000;		break;
			case 8:		acquireTime	= 1.0/3000;		break;
			case 9:		acquireTime	= 1.0/4000;		break;
			case 10:	acquireTime	= 1.0/5000;		break;
			case 11:	acquireTime	= 1.0/6000;		break;
			case 12:	acquireTime	= 1.0/7500;		break;
			case 13:	acquireTime	= 1.0/10000;	break;
			case 14:	acquireTime	= 1.0/15000;	break;
			case 15:	acquireTime	= 1.0/31000;	break;
		}
		if ( pAcquireTimeVal != NULL )
			*pAcquireTimeVal	= acquireTime;
		if ( pTriggerModeVal != NULL )
			*pTriggerModeVal	= 0;	// FreeRun
	}
	else
	{
		epicsInt32	triggerMode = 1;	// External

		//	AM, Async Triggered Mode
		switch ( *pRawShutterVal )
		{
			default:
			case 0:		acquireTime	= 10.0;			break;
			case 1:		acquireTime	= 1.0/60;		break;
			case 2:		acquireTime	= 1.0/125;		break;
			case 3:		acquireTime	= 1.0/250;		break;
			case 4:		acquireTime	= 1.0/500;		break;
			case 5:		acquireTime	= 1.0/1000;		break;
			case 6:		acquireTime	= 1.0/2000;		break;
			case 7:		acquireTime	= 1.0/3000;		break;
			case 8:		acquireTime	= 1.0/4000;		break;
			case 9:		acquireTime	= 1.0/5000;		break;
			case 10:	acquireTime	= 1.0/6000;		break;
			case 11:	acquireTime	= 1.0/7500;		break;
			case 12:	acquireTime	= 1.0/10000;	break;
			case 13:	acquireTime	= 1.0/15000;	break;
			case 14:	acquireTime	= 1.0/31000;	break;
			case 15:	// Pulse Width Mode
				if ( pPulseWidthVal != NULL && pPulseWidthScale != NULL && *pPulseWidthScale != 0.0 )
					acquireTime = *pPulseWidthVal / *pPulseWidthScale;
				triggerMode = 2;	// Pulse
				break;
		}
		if ( pAcquireTimeVal != NULL )
			*pAcquireTimeVal	= acquireTime;
		if ( pTriggerModeVal != NULL )
			*pTriggerModeVal	= triggerMode;
	}

	return status;
}


// Register aSub functions
extern "C"
{
// epicsRegisterFunction(	Up900Shutter_Init		);
epicsRegisterFunction(	Up900Shutter_Process	);
}

