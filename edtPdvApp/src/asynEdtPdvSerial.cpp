//
//	asynEdtPdvSerial driver
//
//	Asyn device support using EDT framegrabber serial interface via CamLink
//

//	#include <string.h>
//	#include <errno.h>

//	#include <cantProceed.h>
//	#include <errlog.h>
//	#include <iocsh.h>
//	#include <epicsAssert.h>
//	#include <epicsMutex.h>
//	#include <epicsStdio.h>
//	#include <epicsString.h>
//	#include <epicsThread.h>
//	#include <epicsTime.h>

//	#include "asynDriver.h"
//	#include "asynOctet.h"

// #include "edtPdvCamera.h"
#include "asynEdtPdvSerial.h"

#include "edtinc.h"

extern int	DEBUG_EDT_PDV;

#define	MAX_ADDR		1
#define	NUM_PARAMS		1

using namespace	std;

///	Constructor
asynEdtPdvSerial::asynEdtPdvSerial(
	const char			*	portName,
	int						priority,		// 0 = default 50, high is 90
	int						autoConnect,	// 0 = no auto-connect
	int						maxBuffers,		// 0 = unlimited
	size_t					maxMemory,		// 0 = unlimited
	int						stackSize		// 0 = default 1MB
	)	:
	asynPortDriver(			portName,
							MAX_ADDR,
							NUM_PARAMS,
							asynOctetMask,		// Interface mask
							asynOctetMask,		// Interrupt mask
							ASYN_CANBLOCK,
							autoConnect,
							priority,
							stackSize	),
	m_pPdvDev(				NULL		),
	m_pasynUser(			NULL		),
	m_inputEosOctet(		NULL		),
	m_inputEosLenOctet(		0			),
	m_outputEosOctet(		NULL		),
	m_outputEosLenOctet(	0			),
	m_fConnected(			false		)
{
	const char		*	functionName	= "asynEdtPdvSerial::asynEdtPdvSerial";
	//	asynStatus			status;
	//	int					nbytes;
	//	asynOctet		*	pasynOctet;

	if ( DEBUG_EDT_PDV >= 1 )
		printf(  "%s: %s\n", functionName, portName );

	/*
	 * Check arguments
	 */
	if ( portName == NULL || *portName == '\0' )
	{
		printf("portName missing or empty.\n");
	}

}

/// virtual Destructor
asynEdtPdvSerial::~asynEdtPdvSerial()
{
}


//
//	asynPortDriver function overrides
//
asynStatus	asynEdtPdvSerial::connect(
	asynUser			*	pasynUser	)
{
    static const char	*	functionName	= "asynEdtPdvSerial::connect";
    epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize, "%s:\n", functionName );

	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"%s port %s\n", functionName, this->portName );

	if ( m_pPdvDev == NULL )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s pdvDev disconnected!\n", functionName, this->portName );
		return asynError;
	}

	m_fConnected	= true;

	// Signal asynManager that we are connected
	int  status = pasynManager->exceptionConnect( pasynUser );
	if ( status != asynSuccess )
		asynPrint(	pasynUser, ASYN_TRACE_ERROR,
					"%s port %s: Error calling pasynManager->exceptionConnect, error=%s\n",
					functionName, this->portName, pasynUser->errorMessage );

	return asynSuccess;
}


asynStatus	asynEdtPdvSerial::disconnect(
	asynUser			*	pasynUser	)
{
    static const char	*	functionName	= "asynEdtPdvSerial::disconnect";
    epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize, "%s:\n", functionName );

	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"%s port %s\n", functionName, this->portName );

	m_fConnected	= false;

	// Signal asynManager that we are disconnected
	int  status = pasynManager->exceptionDisconnect( pasynUser );
	if ( status != asynSuccess )
		asynPrint(	pasynUser, ASYN_TRACE_ERROR,
					"%s port %s: Error calling pasynManager->exceptionDisconnect, error=%s\n",
					functionName, this->portName, pasynUser->errorMessage );

	return asynSuccess;
}


asynStatus
asynEdtPdvSerial::pdvDevConnected(
	PdvDev			*	pPdvDev	)
{
	asynStatus			status			= asynSuccess;
	const char		*	functionName	= "asynEdtPdvSerial::pdvDevConnected";

	if ( DEBUG_EDT_PDV >= 1 )
		printf( "%s: %s Connecting %s\n", functionName, this->portName,
				(pPdvDev != NULL ? pdv_get_camera_model( pPdvDev ) : "NULL") );

	m_pPdvDev	= pPdvDev;
	if ( pPdvDev == NULL )
	{
		m_fConnected	= false;
	}
	else
	{
		m_fConnected	= true;

		// Create a temporary asynUser for autoConnect control
		asynUser	*	pAsynUserTmp = pasynManager->createAsynUser(0,0);
		pAsynUserTmp->userPvt = this;
		pasynManager->autoConnect( pAsynUserTmp, 1 );
	}

	return status;
}


asynStatus
asynEdtPdvSerial::pdvDevDisconnected(
	PdvDev			*	pPdvDev	)
{
	asynStatus			status			= asynSuccess;
	const char		*	functionName	= "asynEdtPdvSerial::pdvDevDisconnected";
	if ( DEBUG_EDT_PDV >= 1 )
		printf( "%s: %s Disconnecting\n", functionName, this->portName );

	m_fConnected	= false;
	m_pPdvDev		= pPdvDev;
	return status;
}


asynStatus	asynEdtPdvSerial::readOctet(
	asynUser			*	pasynUser,
	char				*	value,
	size_t					nBytesReadMax,
	size_t				*	pnRead,
	int					*	eomReason	)
{
	asynStatus				status			= asynSuccess;
    static const char	*	functionName	= "asynEdtPdvSerial::readOctet";
    const char			*	reasonName		= "unknownReason";

	getParamName( 0, pasynUser->reason, &reasonName );
	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"%s: %s nBytesReadMax %zu, reason %d %s\n",
				functionName, this->portName, nBytesReadMax, pasynUser->reason, reasonName );

	if ( pnRead )
		*pnRead = 0;
	if ( eomReason )
		*eomReason = ASYN_EOM_EOS;

	if ( m_pPdvDev == NULL )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s pdvDev disconnected!\n", functionName, this->portName );
		return asynError;
	}
	if ( !m_fConnected )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s Error: %s disconnected:", functionName, this->portName );
		return asynError;
	}
	if ( nBytesReadMax == 0 )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s nBytesReadMax is 0! Why?\n", functionName, this->portName );
		return asynError;
	}

	size_t		nRead	= 0;
	for (;;)
	{
//		epicsMutexLock(tty->m_serialLock);
		/*
		 * Let's get the timeout_ms semantics right:
		 *	> 0 = wait for this many milliseconds.
		 *	  0 = don't wait, just read what you have.
		 *	< 0 = wait forever.
		 * Note: Above was for edt_unix module
		 * Now we need to follow streamdevice usage: <= 0 is don't wait, > 0 wpecifies delay in sec
		 * In pdv_serial_wait, 0 = wait for the default time (1 sec?).
		 */
		int nAvailToRead;
		if ( pasynUser->timeout > 0 )
		{
			int		nMsTimeout	= static_cast<int>( pasynUser->timeout * 1000 );
			nAvailToRead = pdv_serial_wait( m_pPdvDev, nMsTimeout, nBytesReadMax );
		}
		else
			nAvailToRead = pdv_serial_get_numbytes( m_pPdvDev );
//		epicsMutexUnlock(tty->m_serialLock);

		if( nAvailToRead > 0 )
		{
			int		nToRead	= nAvailToRead;
			if( nToRead > static_cast<int>(nBytesReadMax) )
				nToRead = static_cast<int>(nBytesReadMax);
//			epicsMutexLock(tty->m_serialLock);
			nRead = pdv_serial_read( this->m_pPdvDev, value, nToRead );
//			epicsMutexUnlock(tty->m_serialLock);
		}

		if( nRead > 0 )
		{
			asynPrintIO(	pasynUser, ASYN_TRACEIO_DRIVER, value, nRead,
							"%s: %s read %zu of %d\n", functionName, this->portName,
							nRead, nAvailToRead );
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
								functionName, this->portName, strerror(errno)	);
				status = asynError;
				break;		/* If we have an error, we're done. */
			}
		}
		if ( pasynUser->timeout > 0 )
			break;			/* If we aren't waiting forever, we're done. */
	}

	if ( nRead == 0 && (status == asynSuccess))	/* If we don't have anything, not even an error	*/
	{
		status = asynTimeout;					/* then we must have timed out.					*/
		if ( eomReason )
			*eomReason = ASYN_EOM_EOS;
	}

	*pnRead = nRead;
	/* If there is room add a null byte */
	if ( nRead < nBytesReadMax )
	{
		value[nRead] = 0;
		if ( eomReason )
			*eomReason = ASYN_EOM_EOS;
	}
	else if ( nRead == nBytesReadMax )
	{
		if ( eomReason )
			*eomReason = ASYN_EOM_CNT;
	}

	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"%s: %s read %zu, status %d, value %s\n",
				functionName, this->portName, nRead, status, value	);

	// Call the parameter callbacks
    callParamCallbacks();

    return status;
}

asynStatus	asynEdtPdvSerial::writeOctet(
	asynUser			*	pasynUser,
	const char			*	value,
	size_t					maxChars,
	size_t				*	pnWritten	)
{
	asynStatus				status			= asynSuccess;
    static const char	*	functionName	= "asynEdtPdvSerial::writeOctet";
    const char			*	reasonName		= "unknownReason";
	
	getParamName( 0, pasynUser->reason, &reasonName );
	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"%s: %s maxChars %zu, reason %d %s\n",
				functionName, this->portName, maxChars, pasynUser->reason, reasonName );

	if ( pnWritten )
		*pnWritten = 0;

	if ( m_pPdvDev == NULL )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s pdvDev disconnected!\n", functionName, this->portName );
		return asynError;
	}
	if ( !m_fConnected )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s Error: %s disconnected:", functionName, this->portName );
		return asynError;
	}

	if ( maxChars == 0 )
		return asynSuccess;

	// Note: 
	// This driver is designed to be used from DTYP "stream" PV's.
	// The streamdevice asynDriver owns the pdv serial channel
	// and manages any bytes read from that channel, using it's
	// own layer of mutex protection.
	//
	// Calling pdv_serial functions from outside this driver is dangerous
	// and also wrong as it requires finding another way besides
	// streamdevice to handle protocol differences between
	// the many camera models we may need to support.
	int		pdv_status	= pdv_serial_write( m_pPdvDev, value, maxChars );

	if ( pdv_status == 0 )
	{
		*pnWritten = maxChars;

		asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
					"%s: wrote %zu to %s: %s\n",
					functionName, *pnWritten, this->portName, value	);
		asynPrintIO(	pasynUser, ASYN_TRACEIO_DRIVER, value, *pnWritten,
						"%s: %s wrote %zu\n", functionName, this->portName, *pnWritten );
	}
	else if ( pdv_status != 0 )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s write error: %s\n",
						functionName, this->portName, strerror(errno)	);
		status = asynError;
	}

    callParamCallbacks();

    return status;
}


#if 0
asynStatus asynEdtPdvSerial::flushOctet(
	asynUser			*	pasynUser	)
{
static const char	*	functionName	= "asynEdtPdvSerial::flushOctet";
double     savetimeout = pasynUser->timeout;
char       buffer[100]; 
size_t     nbytesTransfered;

pasynUser->timeout = .05;
while(1) {
nbytesTransfered = 0;
readOctet(pasynUser, buffer, sizeof(buffer), &nbytesTransfered, 0);
if (nbytesTransfered==0) break;
asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER,
buffer, nbytesTransfered, "%s:%s\n", driverName, functionName);
}
pasynUser->timeout = savetimeout;
return asynSuccess;
}


// Do we need these?
asynStatus asynEdtPdvSerial::setInputEosOctet(
	asynUser		*	pasynUser,
	const char		*	eos,
	int					eosLen	)

asynStatus asynEdtPdvSerial::getInputEosOctet(
	asynUser		*	pasynUser,
	char			*	eos,
	int					eosSize,
	int				*	eosLen	)

asynStatus	asynEdtPdvSerial::setOutputEosOctet(
	asynUser		*	pasynUser,
	const char		*	eos,
	int					eosLen	)

asynStatus	asynEdtPdvSerial::getOutputEosOctet(
	asynUser		*	pasynUser,
	char			*	eos,
	int					eosSize,
	int				*	eosLen	)
#endif

void asynEdtPdvSerial::report( FILE * fp, int details )
{
    fprintf(	fp, "EDT PDV camera serial port %s: %s\n",
				this->portName, m_fConnected ? "Connected" : "Disconnected" );
    fprintf(	fp, "EDT PDV camera serial port %s: camera model %s\n",
				this->portName,
				(m_pPdvDev != NULL ? pdv_get_camera_model( m_pPdvDev ) : "None") );

	int			connected	= 0;
	pasynManager->isConnected( this->pasynUserSelf, &connected );
	if ( m_fConnected && !connected )
	{
		fprintf(	fp, "Warning, Camera serial port %s thinks it's %s, but asynManager says %s\n",
					portName,
					m_fConnected	? "Connected" : "Disconnected",
					connected		? "Connected" : "Disconnected"	);
	}

#if 0
    if ( details > 0 )
	{
        fprintf( fp, "\n" );
    }
#endif

    /* Call the base class method */
    asynPortDriver::report( fp, details );
}

//	Private member variables
