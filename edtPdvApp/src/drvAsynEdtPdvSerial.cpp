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

#include "asynDriver.h"
#include "asynOctet.h"

// #include "edtPdvCamera.h"
extern int	EDT_PDV_DEBUG;
#include "drvAsynEdtPdvSerial.h"

#include "edtinc.h"

using namespace	std;

static const char *	driverName	= "EdtPdvSerial";

/*
 * Close a connection
 */
static void
ttyCloseConnection( asynUser *	pasynUser, ttyController_t * tty )
{
	const char	*	functionName	= "ttyCloseConnection";
	if ( EDT_PDV_DEBUG >= 1 )
		printf(  "tty %s: Close %s serial connection.\n", functionName, tty->portName );
	if ( tty->m_Connected )
	{
		asynPrint(	pasynUser, ASYN_TRACE_FLOW,
					"asynPrint "
					"%s: Close %s serial connection.\n", functionName, tty->portName );
		//	close(tty->fd);
		tty->m_Connected = false;
		//	tty->m_pPdvDev	 = NULL;
		// Signal asynManager that we are disconnected
		int  status = pasynManager->exceptionDisconnect( tty->pasynUser );
		if ( status != asynSuccess )
			asynPrint(	tty->pasynUser, ASYN_TRACE_ERROR,
						"asynPrint "
						"%s %s port %s: Error calling pasynManager->exceptionDisconnect, error=%s\n",
						driverName, functionName, tty->portName, tty->pasynUser->errorMessage );
	}
}

/*
 * Report link parameters
 */
static void
report( void * drvPvt, FILE * fp, int details )
{
	ttyController_t *tty = (ttyController_t *)drvPvt;

	assert(tty);
	int			connected	= 0;
	pasynManager->isConnected( tty->pasynUser, &connected );
	fprintf(	fp, "Camera serial port %s: %s\n",
				tty->portName,
				tty->m_Connected ? "Connected" : "Disconnected" );
	if ( tty->m_Connected && !connected )
	{
		fprintf(	fp, "Warning, Camera serial port %s thinks it's %s, but asynManager says %s\n",
					tty->portName,
					tty->m_Connected	? "Connected" : "Disconnected",
					connected			? "Connected" : "Disconnected"	);
	}
	if ( details >= 1 )
	{
		fprintf( fp, "	 Characters written: %lu\n", tty->nWritten );
		fprintf( fp, "		Characters read: %lu\n", tty->nRead );
	}
}

/*
 * Create a link
 */
static asynStatus
ttyConnectIt( void * drvPvt, asynUser * pasynUser)
{
	const char	*	functionName	= "ttyConnectIt";
	ttyController_t *tty = (ttyController_t *)drvPvt;

	if ( EDT_PDV_DEBUG >= 3 )
		printf(	"%s: %s ...\n", functionName, tty->portName );

	/*
	 * Sanity check
	 */
	assert(tty);
	if ( tty->m_pPdvDev == NULL )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s %s: Camera Device not available!\n", functionName, tty->portName );
		return asynError;
	}

#if 0
	if ( tty->m_Connected )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s %s: Serial Link already open!\n", functionName, tty->portName );
		return asynError;
	}
#endif

	int			connected	= 0;
	pasynManager->isConnected( tty->pasynUser, &connected );
	if ( !connected || !tty->m_Connected )
	{
		/*
		 * Open connection to camera
		 */
		tty->m_Connected = true;

		if ( EDT_PDV_DEBUG >= 1 )
			printf(	"tty %s: Opened connection to %s\n", functionName, tty->portName );
		asynPrint(	pasynUser, ASYN_TRACE_FLOW,
					"asynPrint "
					"%s: Opened connection to %s\n", functionName, tty->portName );

		// Signal asynManager that we are connected
		int  status = pasynManager->exceptionConnect( tty->pasynUser );
		if ( status != asynSuccess )
			asynPrint(	tty->pasynUser, ASYN_TRACE_ERROR,
						"asynPrint "
						"%s %s port %s: Error calling pasynManager->exceptionConnect, error=%s\n",
						driverName, functionName, tty->portName, tty->pasynUser->errorMessage );
	}
	return asynSuccess;
}


static asynStatus
ttyDisconnect( void * drvPvt, asynUser * pasynUser )
{
	const char	*	functionName	= "ttyDisconnect";
	ttyController_t *tty = (ttyController_t *)drvPvt;

	assert(tty);
	if ( EDT_PDV_DEBUG >= 1 )
		printf(	"%s: %s\n", functionName, tty->portName );
	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: %s\n", functionName, tty->portName );
	ttyCloseConnection( pasynUser, tty );
	return asynSuccess;
}


/*
 * Write to the serial line
 */
static asynStatus ttyWriteIt(
	void			*	drvPvt,
	asynUser		*	pasynUser,
	const char		*	data,
	size_t				numchars,
	size_t			*	nbytesTransfered )
{
	const char		*	functionName	= "ttyWriteIt";
	ttyController_t *tty = (ttyController_t *)drvPvt;
	int nleft = numchars;
	asynStatus status = asynSuccess;

	assert(tty);
	if ( EDT_PDV_DEBUG >= 3 )
		printf(	"%s: %s write %zu chars\n", functionName, tty->portName, numchars );
	asynPrintIO( pasynUser,	ASYN_TRACE_FLOW | ASYN_TRACEIO_DRIVER, data, numchars,
				"asynPrintIO "
				"%s: %s write %zu chars\n", functionName, tty->portName, numchars );
	if ( !tty->m_Connected )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s disconnected!\n", functionName, tty->portName );
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

	// Note: We wouldn't need this serialLock if we could avoid
	// trying to use pdv_serial_command from the edtPdv driver code.
	// This driver is designed to be used from DTYP "stream" PV's, and
	// the streamdevice asynDriver owns the pdv serial channel
	// and manages any bytes read from that channel, using it's
	// own layer of mutex protection.
	//
	// Calling pdv_serial_command from the driver is also wrong as
	// it requires finding another way besides streamdevice via this
	// asynEdtPdvSerial driver to handle protocol differences between
	// the many camera models we may need to support.
	epicsMutexLock( tty->m_serialLock );
	int	pdv_status = pdv_serial_command( tty->m_pPdvDev, (char *)data );
	epicsMutexUnlock( tty->m_serialLock );
	if ( pdv_status == 0 )
	{
		tty->nWritten += nleft;
		data += nleft;
		nleft	= 0;
	}
	if (tty->writeTimeout == 0)
	{
		status = asynTimeout;
	}
	else if ( pdv_status != 0 )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s write error: %s\n",
						functionName, tty->portName, strerror(errno));
		ttyCloseConnection( pasynUser, tty );
		status = asynError;
	}

	*nbytesTransfered = numchars - nleft;
	if ( EDT_PDV_DEBUG >= 3 )
		printf(	"%s: wrote %zu chars to %s, return %s\n",
				functionName,
				(unsigned long)*nbytesTransfered,
				tty->portName,
				pasynManager->strStatus(status)	);
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: wrote %zu to %s, return %s\n",
				functionName,
				*nbytesTransfered,
				tty->portName,
				pasynManager->strStatus(status)	);
	return status;
}

/*
 * Read from the serial line
 */
static asynStatus ttyReadIt(
	void			*	drvPvt,
	asynUser		*	pasynUser,
	char			*	data,
	size_t				maxchars,
	size_t			*	nbytesTransfered,
	int				*	gotEom	)
{
	const char		*	functionName	= "ttyReadIt";
	ttyController_t	*	tty = (ttyController_t *)drvPvt;
	int					nRead			= 0;
	asynStatus			status			= asynSuccess;
	int timeout_ms;

	assert(tty);
	if ( EDT_PDV_DEBUG >= 3 )
		printf(  "%s: %s read up to %zu chars\n", functionName, tty->portName, maxchars );
	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: %s read up to %zu chars\n", functionName, tty->portName, maxchars );
	if ( !tty->m_Connected )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s Error: %s disconnected:", functionName, tty->portName );
		return asynError;
	}
	if ( maxchars == 0 )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s maxchars is 0! Why?\n", functionName, tty->portName );
		return asynError;
	}
	if( tty->readTimeout != pasynUser->timeout )
	{
		tty->readTimeout = pasynUser->timeout;
	}
	timeout_ms = static_cast<int>(tty->readTimeout*1000);
	if ( gotEom )
		*gotEom = 0;

	for (;;)
	{
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

		if( nAvailToRead > 0 )
		{
			int		nToRead	= nAvailToRead;
			if( nToRead > static_cast<int>(maxchars) )
				nToRead = static_cast<int>(maxchars);
			epicsMutexLock(tty->m_serialLock);
			nRead = pdv_serial_read( tty->m_pPdvDev, data, nToRead );
			epicsMutexUnlock(tty->m_serialLock);
		}

		if( nRead > 0 )
		{
			asynPrintIO(	pasynUser, ASYN_TRACEIO_DRIVER, data, nRead,
							"asynPrint "
							"%s: %s read %d\n", functionName, tty->portName, nRead);
			tty->nRead += nRead;
			break;			/* If we have something, we're done. */
		}
		else
		{
			if (	(nAvailToRead < 0 || nRead < 0)
					&&	(errno != EWOULDBLOCK)
					&&	(errno != EINTR)
					&&	(errno != EAGAIN) )
			{
				epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
								"%s: %s read error: %s\n",
								functionName, tty->portName, strerror(errno));
				ttyCloseConnection( pasynUser, tty );
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
	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: %s read %zu, status %d\n",
				functionName, tty->portName, *nbytesTransfered, status	);
	return status;
}

/*
 * Flush pending input
 */
static asynStatus
ttyFlushIt( void * drvPvt, asynUser *	pasynUser )
{
	const char	*	functionName	= "ttyFlushIt";
	ttyController_t *tty = (ttyController_t *)drvPvt;

	assert(tty);
	if ( EDT_PDV_DEBUG >= 1 )
		printf(	"%s: %s\n", functionName, tty->portName );
	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: %s\n", functionName, tty->portName );
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
	const char	*	functionName	= "ttyCleanup";
	if (tty)
	{
		if ( EDT_PDV_DEBUG >= 1 )
			printf(  "tty %s: %s\n", functionName, tty->portName );
		if ( tty->m_Connected )
		{
			//	close(tty->fd);
			tty->m_Connected	= false;
			// Signal asynManager that we are disconnected
			int	status	= pasynManager->exceptionDisconnect( tty->pasynUser );
			if ( status != asynSuccess )
				asynPrint(	tty->pasynUser, ASYN_TRACE_ERROR,
							"asynPrint "
							"%s %s port %s: Error calling pasynManager->exceptionDisconnect, error=%s\n",
							driverName, functionName, tty->portName, tty->pasynUser->errorMessage );
		}
		tty->m_pPdvDev		= NULL;
		free(tty->portName);
		free(tty);
	}
}

/*
 * asynCommon methods
 */
static const struct asynCommon drvAsynEdtPdvSerialPortAsynCommon =
{
	report,
	ttyConnectIt,
	ttyDisconnect
};


/*
 * Configure and register a generic serial device
 */
ttyController_t	*
drvAsynEdtPdvSerialPortConfigure(
	const char		*	portName,
	unsigned int		priority,
	int					autoConnect,
	int					noProcessEos )
{
	const char	*	functionName	= "drvAsynEdtPdvSerialPortConfigure";
	ttyController_t	*	tty;
	asynStatus			status;
	int					nbytes;
	asynOctet		*	pasynOctet;

	if ( EDT_PDV_DEBUG >= 1 )
		printf(  "tty %s: %s\n", functionName, tty->portName );

	/*
	 * Check arguments
	 */
	if ( portName == NULL || *portName == '\0' )
	{
		printf("portName missing or empty.\n");
		return NULL;
	}

	/*
	 * Create a driver
	 */
	nbytes = sizeof(*tty) + sizeof(asynOctet);
	tty = (ttyController_t *)callocMustSucceed( 1, nbytes, functionName );
	pasynOctet = (asynOctet *)(tty +1);
	tty->m_Connected	= false;
	tty->m_pPdvDev		= NULL;
	tty->portName		= epicsStrDup(portName);
	tty->m_addr			= 0;
    tty->m_serialLock	= epicsMutexMustCreate();

	/*
	 *	Link with higher level routines
	 */
	tty->common.interfaceType = asynCommonType;
	tty->common.pinterface	= (void *)&drvAsynEdtPdvSerialPortAsynCommon;
	tty->common.drvPvt = tty;
	if ( pasynManager->registerPort(	tty->portName,
										ASYN_CANBLOCK,
										autoConnect,
										priority, 0		) != asynSuccess )
	{
		printf(	"tty %s: %s registerPort error!\n",
				functionName, tty->portName );
		ttyCleanup(tty);
		return NULL;
	}
	status = pasynManager->registerInterface( tty->portName, &tty->common );
	if ( status != asynSuccess )
	{
		printf(	"tty %s: %s Can't register common interface!\n",
				functionName, tty->portName );
		ttyCleanup(tty);
		return NULL;
	}
	pasynOctet->read			= ttyReadIt;
	pasynOctet->write			= ttyWriteIt;
	pasynOctet->flush			= ttyFlushIt;
	tty->octet.interfaceType	= asynOctetType;
	tty->octet.pinterface		= pasynOctet;
	tty->octet.drvPvt			= tty;
	status = pasynOctetBase->initialize(	tty->portName,	&tty->octet,
											(noProcessEos ? 0 : 1),
											(noProcessEos ? 0 : 1), 1 );
	if ( status != asynSuccess )
	{
		printf(	"tty %s: %s Can't register octet interface!\n",
				functionName, tty->portName );
		ttyCleanup(tty);
		return NULL;
	}
	tty->pasynUser = pasynManager->createAsynUser(0,0);
	if ( EDT_PDV_DEBUG )
		printf(  "Camera serial port: %s created!\n", tty->portName );

	status = pasynManager->connectDevice( tty->pasynUser, tty->portName, tty->m_addr );
	if ( status != asynSuccess )
	{
		printf( "connectDevice failed %s\n", tty->pasynUser->errorMessage );
		ttyCleanup( tty );
		return NULL;
	}
	return tty;
}


/*
 * Connect to the EDTPdv serial device
 */
asynStatus
drvAsynEdtPdvSerialPortConnect(
	ttyController_t	*	tty,
	PdvDev			*	pPdvDev	)
{
	const char	*	functionName	= "drvAsynEdtPdvSerialPortConnect";
	if ( tty == NULL )
		return asynError;
	if ( EDT_PDV_DEBUG >= 1 )
		printf(  "tty %s: %s\n", functionName, tty->portName );

	tty->m_pPdvDev		= pPdvDev;
	if ( pPdvDev == NULL )
	{
		printf(  "tty %s Error on %s: NULL PdvDev ptr!\n", functionName, tty->portName );
		tty->m_Connected	= false;

		// Signal asynManager that we are disconnected
		int  status	= pasynManager->exceptionDisconnect( tty->pasynUser );
		if ( status != asynSuccess )
			asynPrint(	tty->pasynUser, ASYN_TRACE_ERROR,
						"asynPrint "
						"%s %s port %s: Error calling pasynManager->exceptionDisconnect, error=%s\n",
						driverName, functionName, tty->portName, tty->pasynUser->errorMessage );
		return asynError;
	}

	tty->m_Connected	= true;
	if ( EDT_PDV_DEBUG >= 1 )
		printf(  "tty %s: %s connected\n", functionName, tty->portName );

	// Signal asynManager that we are connected
	int  status = pasynManager->exceptionConnect( tty->pasynUser );
	if ( status != asynSuccess )
		asynPrint(	tty->pasynUser, ASYN_TRACE_ERROR,
					"asynPrint "
					"%s %s port %s: Error calling pasynManager->exceptionConnect, error=%s\n",
					driverName, functionName, tty->portName, tty->pasynUser->errorMessage );
	return asynSuccess;
}


/*
 * Disconnect from the EDTPdv serial device
 */
asynStatus
drvAsynEdtPdvSerialPortDisconnect(
	ttyController_t	*	tty	)
{
	const char	*	functionName	= "drvAsynEdtPdvSerialPortDisconnect";
	if ( tty == NULL )
		return asynError;

	tty->m_Connected	= false;
	//	tty->m_pPdvDev		= NULL;
	// Signal asynManager that we are disconnected
	int  status = pasynManager->exceptionDisconnect( tty->pasynUser );
	if ( status != asynSuccess )
		asynPrint(	tty->pasynUser, ASYN_TRACE_ERROR,
					"asynPrint "
					"%s %s port %s: Error calling pasynManager->exceptionDisconnect, error=%s\n",
					driverName, functionName, tty->portName, tty->pasynUser->errorMessage );
	if ( EDT_PDV_DEBUG >= 1 )
		printf(  "tty %s: %s\n", functionName, tty->portName );
	return asynSuccess;
}
