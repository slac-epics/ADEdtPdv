//
//	Asyn device support using EDT framegrabber serial interface via CamLink
//

#include <string.h>
#include <errno.h>

#include <cantProceed.h>
#include <errlog.h>
#include <iocsh.h>
#include <epicsAssert.h>
#include <epicsMutex.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsTime.h>
#include <epicsExport.h>

#include "asynDriver.h"
#include "asynOctet.h"

// #include "edtPdvCamera.h"
#include "drvAsynEdtPdvSerial.h"

#include "edtinc.h"

/*
 * This structure holds the hardware-specific information
 * for a single asyn link.	There is one for each camera.
 */
typedef struct
{
	asynUser		*	pasynUser;
	char			*	portName;
	bool				m_Connected;
//	edtPdvCamera		*	m_pCamera;
	PdvDev			*	m_pPdvDev;
	unsigned long		nRead;
	unsigned long		nWritten;
	double				readTimeout;
	double				writeTimeout;
	asynInterface		common;
	asynInterface		octet;
	epicsMutexId		m_serialLock;
}	ttyController_t;

/*
 * Close a connection
 */
static void
closeConnection(asynUser *pasynUser,ttyController_t *tty)
{
	if ( tty->m_Connected )
	{
		asynPrint(pasynUser, ASYN_TRACE_FLOW,
						"Close %s connection.\n", tty->portName );
		//	close(tty->fd);
		tty->m_Connected = false;
		pasynManager->exceptionDisconnect(pasynUser);
	}
}

/*
 * Report link parameters
 */
static void
report(void *drvPvt, FILE *fp, int details)
{
	ttyController_t *tty = (ttyController_t *)drvPvt;

	assert(tty);
	fprintf(	fp, "Camera %s: %s\n",
				tty->portName,
				tty->m_Connected ? "Connected" : "Disconnected");
	if (details >= 1)
	{
		fprintf(fp, "	 Characters written: %lu\n", tty->nWritten);
		fprintf(fp, "		Characters read: %lu\n", tty->nRead);
	}
}

/*
 * Create a link
 */
static asynStatus
connectIt(void *drvPvt, asynUser *pasynUser)
{
	ttyController_t *tty = (ttyController_t *)drvPvt;

	/*
	 * Sanity check
	 */
	assert(tty);
	if ( tty->m_Connected )
	{
		epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
							"%s: Link already open!", tty->portName );
		return asynError;
	}
	asynPrint(pasynUser, ASYN_TRACE_FLOW,
							"Open connection to %s\n", tty->portName );

	/*
	 * Open connection to camera
	 */
#if 0
// No need to get a pCamera pointer, as we already have m_pPdvDev, but
// we should find a way to force a close and re-open of the camera connection
	edtPdvCamera	*	pCamera = edtPdvCamera::CameraFindByName( tty->portName );
	if ( pCamera == NULL )
	{
		epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
							": Can't find camera %s\n", tty->portName );
		tty->m_Connected = false;
		return asynError;
	}
#endif
	tty->m_Connected = true;

	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"Opened connection to %s\n", tty->portName );
	pasynManager->exceptionConnect(pasynUser);
	return asynSuccess;
}


static asynStatus
disconnect(void *drvPvt, asynUser *pasynUser)
{
	ttyController_t *tty = (ttyController_t *)drvPvt;

	assert(tty);
	asynPrint(pasynUser, ASYN_TRACE_FLOW,
									"%s disconnect\n", tty->portName );
	closeConnection(pasynUser,tty);
	return asynSuccess;
}


/*
 * Write to the serial line
 */
static asynStatus writeIt(
	void *drvPvt, asynUser *pasynUser,
	const char *data, size_t numchars,size_t *nbytesTransfered)
{
	ttyController_t *tty = (ttyController_t *)drvPvt;
	int nleft = numchars;
	asynStatus status = asynSuccess;

	assert(tty);
	asynPrint(pasynUser, ASYN_TRACE_FLOW,
							"%s write.\n", tty->portName );
	asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, data, numchars,
							"%s write %zd\n", tty->portName, numchars);
	if ( !tty->m_Connected )
	{
		epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
								"%s disconnected:", tty->portName );
		return asynError;
	}
	if (numchars == 0)
	{
		*nbytesTransfered = 0;
		return asynSuccess;
	}
	if (tty->writeTimeout != pasynUser->timeout)
	{
		tty->writeTimeout = pasynUser->timeout;
	}
	nleft = numchars;

	epicsMutexLock(tty->m_serialLock);
	int	pdv_status = pdv_serial_command( tty->m_pPdvDev, (char *)data );
	epicsMutexUnlock(tty->m_serialLock);
	if ( pdv_status == 0 ) {
		tty->nWritten += nleft;
		data += nleft;
		nleft	= 0;
	}
	if (tty->writeTimeout == 0) {
		status = asynTimeout;
	} else if ( pdv_status != 0 ) {
		epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
					"%s write error: %s",
					tty->portName, strerror(errno));
		closeConnection(pasynUser,tty);
		status = asynError;
	}

	*nbytesTransfered = numchars - nleft;
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"wrote %lu to %s, return %s\n",
				(unsigned long)*nbytesTransfered,
				tty->portName,
				pasynManager->strStatus(status)	);
	return status;
}

/*
 * Read from the serial line
 */
static asynStatus readIt(
	void			*	drvPvt,
	asynUser		*	pasynUser,
	char			*	data,
	size_t				maxchars,
	size_t			*	nbytesTransfered,
	int				*	gotEom	)
{
	ttyController_t	*	tty = (ttyController_t *)drvPvt;
	int					nRead			= 0;
	asynStatus			status			= asynSuccess;
	int timeout_ms;

	assert(tty);
	asynPrint(pasynUser, ASYN_TRACE_FLOW,
			"%s read.\n", tty->portName );
	if ( !tty->m_Connected )
	{
		epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
								"%s disconnected:", tty->portName );
		return asynError;
	}
	if (maxchars <= 0)
	{
		epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
			"%s maxchars %d Why <=0?\n",tty->portName,(int)maxchars);
		return asynError;
	}
	if ( tty->readTimeout != pasynUser->timeout ) {
		tty->readTimeout = pasynUser->timeout;
	}
	timeout_ms = static_cast<int>(tty->readTimeout*1000);
	if (gotEom) *gotEom = 0;

	for (;;) {
		int nAvailToRead;

		epicsMutexLock(tty->m_serialLock);
		/*
		 * Let's get the timeout_ms semantics right:
		 *	> 0 = wait for this many milliseconds.
		 *	  0 = don't wait, just read what you have.
		 *	< 0 = wait forever.
		 * In pdv_serial_wait, 0 = wait for the default time (1 sec?).
		 */
		if (timeout_ms)
			nAvailToRead = pdv_serial_wait( tty->m_pPdvDev,
											(timeout_ms > 0) ? timeout_ms : 0, maxchars );
		else
			nAvailToRead = pdv_serial_get_numbytes(tty->m_pPdvDev);
		epicsMutexUnlock(tty->m_serialLock);

		if( nAvailToRead > 0 ) {
			int		nToRead	= nAvailToRead;
			if( nToRead > static_cast<int>(maxchars) )
				nToRead = static_cast<int>(maxchars);
			epicsMutexLock(tty->m_serialLock);
			nRead = pdv_serial_read( tty->m_pPdvDev, data, nToRead );
			epicsMutexUnlock(tty->m_serialLock);
		}

		if( nRead > 0 ) {
			asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, data, nRead,
						"%s read %d\n", tty->portName, nRead);
			tty->nRead += nRead;
			break;			/* If we have something, we're done. */
		} else {
			if (	(nAvailToRead < 0 || nRead < 0)
						&&	(errno != EWOULDBLOCK)
						&&	(errno != EINTR)
						&&	(errno != EAGAIN) ) {
				epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,
							"%s read error: %s",
							tty->portName, strerror(errno));
				closeConnection(pasynUser,tty);
				status = asynError;
				break;		/* If we have an error, we're done. */
			}
		}
		if (timeout_ms >= 0)
			break;			/* If we aren't waiting forever, we're done. */
	}

	if (nRead == 0 && (status == asynSuccess))	/* If we don't have anything, not even an error	*/
		status = asynTimeout;					/* then we must have timed out.					*/
	*nbytesTransfered = nRead;
	/* If there is room add a null byte */
	if ( nRead < static_cast<int>(maxchars) )
		data[nRead] = 0;
	else if (gotEom)
		*gotEom = ASYN_EOM_CNT;
	asynPrint(	pasynUser, ASYN_TRACE_FLOW, "%s read %zd, status %d\n",
				tty->portName, *nbytesTransfered, status	);
	return status;
}

/*
 * Flush pending input
 */
static asynStatus
flushIt(void *drvPvt,asynUser *pasynUser)
{
	ttyController_t *tty = (ttyController_t *)drvPvt;

	assert(tty);
	asynPrint(pasynUser, ASYN_TRACE_FLOW, "%s flush\n", tty->portName );
	if ( tty->m_Connected )
	{
		//	tcflush(tty->fd, TCIFLUSH);
	}
	return asynSuccess;
}

/*
 * Clean up a ttyController
 */
static void
ttyCleanup(ttyController_t *tty)
{
	if (tty)
	{
		if ( tty->m_Connected )
		{
			//	close(tty->fd);
			tty->m_Connected	= false;
		}
		free(tty->portName);
		free(tty->portName );
		free(tty);
	}
}

/*
 * asynCommon methods
 */
static const struct asynCommon drvAsynEdtPdvSerialPortAsynCommon =
{
	report,
	connectIt,
	disconnect
};


/*
 * Configure and register a generic serial device
 */
int
drvAsynEdtPdvSerialPortConfigure(
	const char		*	cameraName,
	unsigned int		priority,
	int					noAutoConnect,
	int					noProcessEos,
	PdvDev			*	pPdvDev	)
{
	ttyController_t	*	tty;
	asynStatus			status;
	int					nbytes;
	asynOctet		*	pasynOctet;

	/*
	 * Check arguments
	 */
	if ( cameraName == NULL || *cameraName == '\0' )
	{
		printf("Camera name missing or empty.\n");
		return -1;
	}

	/*
	 * Create a driver
	 */
	nbytes = sizeof(*tty) + sizeof(asynOctet);
	tty = (ttyController_t *)callocMustSucceed(1, nbytes,
		 "drvAsynEdtPdvSerialPortConfigure()");
	pasynOctet = (asynOctet *)(tty +1);
	tty->m_Connected	= false;
	tty->m_pPdvDev		= pPdvDev;
	tty->portName		= epicsStrDup(cameraName);
    tty->m_serialLock	= epicsMutexMustCreate();

	/*
	 *	Link with higher level routines
	 */
	tty->common.interfaceType = asynCommonType;
	tty->common.pinterface	= (void *)&drvAsynEdtPdvSerialPortAsynCommon;
	tty->common.drvPvt = tty;
	if (pasynManager->registerPort(	tty->portName,
									ASYN_CANBLOCK,
									!noAutoConnect,
									priority,
									0) != asynSuccess)
	{
		printf("drvAsynEdtPdvSerialPortConfigure: Can't register myself.\n");
		ttyCleanup(tty);
		return -1;
	}
	status = pasynManager->registerInterface(tty->portName,&tty->common);
	if(status != asynSuccess)
	{
		printf("drvAsynEdtPdvSerialPortConfigure: Can't register common.\n");
		ttyCleanup(tty);
		return -1;
	}
	pasynOctet->read	= readIt;
	pasynOctet->write	= writeIt;
	pasynOctet->flush	= flushIt;
	tty->octet.interfaceType = asynOctetType;
	tty->octet.pinterface		= pasynOctet;
	tty->octet.drvPvt = tty;
	status = pasynOctetBase->initialize(	tty->portName,	&tty->octet,
											(noProcessEos ? 0 : 1),
											(noProcessEos ? 0 : 1), 1 );
	if ( status != asynSuccess )
	{
		printf("drvAsynEdtPdvSerialPortConfigure: Can't register octet.\n");
		ttyCleanup(tty);
		return -1;
	}
	tty->pasynUser = pasynManager->createAsynUser(0,0);
	status = pasynManager->connectDevice(tty->pasynUser,tty->portName,-1);
	if(status != asynSuccess)
	{
		printf("connectDevice failed %s\n",tty->pasynUser->errorMessage);
		ttyCleanup(tty);
		return -1;
	}
	return 0;
}

#if 0
/*
 * IOC shell command registration
 */
static const iocshArg drvAsynEdtPdvSerialPortConfigureArg0 = { "port name",iocshArgString};
static const iocshArg drvAsynEdtPdvSerialPortConfigureArg1 = { "priority",iocshArgInt};
static const iocshArg drvAsynEdtPdvSerialPortConfigureArg2 = { "disable auto-connect",iocshArgInt};
static const iocshArg drvAsynEdtPdvSerialPortConfigureArg3 = { "noProcessEos",iocshArgInt};
static const iocshArg *drvAsynEdtPdvSerialPortConfigureArgs[] =
{
	&drvAsynEdtPdvSerialPortConfigureArg0, &drvAsynEdtPdvSerialPortConfigureArg1,
	&drvAsynEdtPdvSerialPortConfigureArg2, &drvAsynEdtPdvSerialPortConfigureArg3,
};
static const iocshFuncDef drvAsynEdtPdvSerialPortConfigureFuncDef =
					{ "drvAsynEdtPdvSerialPortConfigure", 4, drvAsynEdtPdvSerialPortConfigureArgs };
static void	drvAsynEdtPdvSerialPortConfigureCallFunc( const iocshArgBuf * args )
{
	drvAsynEdtPdvSerialPortConfigure(	args[0].sval, args[1].ival,
									args[2].ival, args[3].ival	);
}

/*
 * This routine is called before multitasking has started, so there's
 * no race condition in the test/set of firstTime.
 */
static void
drvAsynEdtPdvSerialPortRegisterCommands(void)
{
	static int firstTime = 1;
	if (firstTime)
	{
		iocshRegister(&drvAsynEdtPdvSerialPortConfigureFuncDef,drvAsynEdtPdvSerialPortConfigureCallFunc);
		firstTime = 0;
	}
}
epicsExportRegistrar(drvAsynEdtPdvSerialPortRegisterCommands);
#endif
