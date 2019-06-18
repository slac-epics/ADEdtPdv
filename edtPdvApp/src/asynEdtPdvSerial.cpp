//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ADEdtPdv'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'ADEdtPdv', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
//
//	asynEdtPdvSerial driver
//
//	Asyn device support using EDT framegrabber serial interface via CamLink
//

#include "epicsExport.h"
#include "epicsStdio.h"
#include "asynEdtPdvSerial.h"

#include "edtinc.h"

int	DEBUG_EDT_SER	= 2;

#define	MAX_ADDR		1

using namespace	std;

static const char * driverName = "asynEdtPdvSerial";

/// TODO: What if we passed EDT unit and ch # here and let the serial open
///	a separate pPdvEdv instance for serial use.   Might be able to use it
/// independent of primary camera pPdvEdv instance being reopened for
/// different config files.
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
							asynOctetMask,		// Interface mask
							asynOctetMask,		// Interrupt mask
							ASYN_CANBLOCK,
							autoConnect,
							priority,
							stackSize	),
	m_pPdvDev(				NULL		),
	m_pasynUserStream(		NULL		),
	m_inputEosOctet(		NULL		),
	m_inputEosLenOctet(		0			),
	m_outputEosOctet(		NULL		),
	m_outputEosLenOctet(	0			),
	m_fConnected(			false		),
	m_fInputFlushNeeded(	true		),
	m_serialLock(						)
{
	const char		*	functionName	= "asynEdtPdvSerial::asynEdtPdvSerial";
	//	asynStatus			status;
	//	int					nbytes;
	//	asynOctet		*	pasynOctet;

	// Create mutexes
    m_serialLock	= epicsMutexMustCreate();

	if ( DEBUG_EDT_SER >= 1 )
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

	epicsMutexLock(m_serialLock);
	if ( m_pPdvDev == NULL )
	{
		m_fConnected	= false;
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s pdvDev disconnected!\n", functionName, this->portName );
		epicsMutexUnlock(m_serialLock);
		return asynError;
	}
	m_fConnected	= true;
	epicsMutexUnlock(m_serialLock);

	int			connected	= 0;
	pasynManager->isConnected( pasynUserSelf, &connected );
	if ( !connected )
	{
		// Signal asynManager that we're connected.
		int  status = pasynManager->exceptionConnect( pasynUser );
		if ( status != asynSuccess )
			asynPrint(	pasynUser, ASYN_TRACE_ERROR,
						"%s port %s: Error calling pasynManager->exceptionConnect, error=%s\n",
						functionName, this->portName, pasynUser->errorMessage );
	}

	return asynSuccess;
}


asynStatus	asynEdtPdvSerial::disconnect(
	asynUser			*	pasynUser	)
{
    static const char	*	functionName	= "asynEdtPdvSerial::disconnect";
    epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize, "%s:\n", functionName );

	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"%s port %s\n", functionName, this->portName );

	epicsMutexLock(m_serialLock);
    //commented out as the user can connect or disconnect using the AsynIO fields
    //m_pPdvDev		= NULL;
	m_fConnected	= false;
	epicsMutexUnlock(m_serialLock);

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

	if ( DEBUG_EDT_SER >= 1 )
		printf( "%s: %s Connecting %s\n", functionName, this->portName,
				(pPdvDev != NULL ? pdv_get_camera_model( pPdvDev ) : "NULL") );

	epicsMutexLock(m_serialLock);
	m_pPdvDev	= pPdvDev;
	if ( pPdvDev == NULL )
	{
		m_fConnected	= false;
		epicsMutexUnlock(m_serialLock);
		return asynError;
	}

	epicsMutexUnlock(m_serialLock);

    connect(this->pasynUserSelf);
	// Create a temporary asynUser for autoConnect control
	//asynUser	*	pAsynUserTmp = pasynManager->createAsynUser(0,0);
	//pAsynUserTmp->userPvt = this;
	//pasynManager->autoConnect( pAsynUserTmp, 1 );

	return status;
}


asynStatus
asynEdtPdvSerial::pdvDevDisconnected(
	PdvDev			*	pPdvDev	) // TODO: Remove pPdvDev param
{
	asynStatus			status			= asynSuccess;
	const char		*	functionName	= "asynEdtPdvSerial::pdvDevDisconnected";
	if ( DEBUG_EDT_SER >= 1 )
		printf( "%s: %s Disconnecting ...\n", functionName, this->portName );

	epicsMutexLock(m_serialLock);
	if ( DEBUG_EDT_SER >= 3 )
		printf( "%s: %s Have serial lock ...\n", functionName, this->portName );

	if ( pasynManager->exceptionDisconnect( this->pasynUserSelf ) != asynSuccess )
	{
        asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s %s: error calling pasynManager->exceptionDisconnect, error=%s\n",
					driverName, functionName, this->pasynUserSelf->errorMessage );
	}
	m_fConnected	= false;
	m_pPdvDev		= NULL;
	epicsMutexUnlock(m_serialLock);

	if ( DEBUG_EDT_SER >= 3 )
		printf( "%s: %s Disconnected\n", functionName, this->portName );
	return status;
}

bool isAscii( const char * pBuf, int sBuf )
{
	if ( pBuf == NULL || sBuf == 0 )
		return false;

	const char	*	pBufEnd	= pBuf + sBuf;
	while ( pBuf < pBufEnd )
	{
		char	next = *pBuf++;
		if ( next <= 0 || next >= 0x7F )
			return false;
	}
	return true;
}


asynStatus	asynEdtPdvSerial::readOctet(
	asynUser			*	pasynUser,
	char				*	pBuffer,
	size_t					nBytesReadMax,
	size_t				*	pnRead,
	int					*	eomReason	)
{
	asynStatus				status			= asynSuccess;
    static const char	*	functionName	= "asynEdtPdvSerial::readOctet";
	int						nAvailToRead	= 0;
 
	if ( pnRead )
		*pnRead = 0;
	if ( eomReason )
		*eomReason = ASYN_EOM_EOS;

	if ( nBytesReadMax == 0 )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s nBytesReadMax is 0! Why?\n", functionName, this->portName );
		return asynError;
	}

	char			*	pReadBuffer	= pBuffer;
	size_t				sReadBuffer	= nBytesReadMax;

	int		nRead	= 0;
	for (;;)
	{
		epicsMutexLock(m_serialLock);
		if ( DEBUG_EDT_SER >= 4 )
			printf( "%s: %s Have serial lock, nBytesReadMax %zu, sReadBuffer %zu, timeout %.3f ...\n",
					functionName, this->portName, nBytesReadMax, sReadBuffer, pasynUser->timeout );
		/*
		 * Follow streamdevice usage on timeout: <= 0 is don't wait, > 0 specifies delay in sec
		 * In pdv_serial_wait, 0 = wait for the default time (1 sec?).
		 */
		if ( m_pPdvDev && m_fConnected )
		{
            int nMsTimeout = 200; // Default and max timeout
			if ( pasynUser->timeout > 0 )
				nMsTimeout	= static_cast<int>( pasynUser->timeout * 1000 );
			nAvailToRead = pdv_serial_wait( m_pPdvDev, nMsTimeout, sReadBuffer );
		}
		if( nAvailToRead > 0 )
		{
			int		nToRead	= nAvailToRead;
			if( nToRead > static_cast<int>(sReadBuffer) )
			{
				if ( DEBUG_EDT_SER >= 3 )
					printf( "%s: %s Clipping nAvailToRead %d to sReadBuffer %zu\n",
							functionName, this->portName, nAvailToRead, sReadBuffer );
				nToRead = static_cast<int>(sReadBuffer);
			}
			asynPrint(	pasynUser, ASYN_TRACE_FLOW,
						"%s: %s nToRead %d\n", functionName, this->portName, nToRead );

			epicsMutexLock(m_serialLock);
			if ( DEBUG_EDT_SER >= 3 )
				printf( "%s: %s Have serial lock, reading %d ...\n", functionName, this->portName, nToRead );
			if ( m_pPdvDev && m_fConnected )
				nRead = pdv_serial_read( m_pPdvDev, pReadBuffer, nToRead );
			else
				nRead = -1;
			epicsMutexUnlock(m_serialLock);
			if ( DEBUG_EDT_SER >= 3 )
				printf( "%s: %s Released serial lock, read %d ...\n", functionName, this->portName, nRead );
		}
		else
		{
            // nAvailToRead <=0 so nothing to do here... fly away!
            *pnRead = 0;
            epicsMutexUnlock(m_serialLock);
            if ( DEBUG_EDT_SER >= 4 )
                printf( "%s: %s Released serial lock, nRead=0, nAvailToRead %d ...\n", functionName, this->portName, nAvailToRead );

            return asynSuccess;
        }

        epicsMutexUnlock(m_serialLock);
		if ( DEBUG_EDT_SER >= 4 )
			printf( "%s: %s Released serial lock, Read %d, nAvailToRead %d ...\n", functionName, this->portName, nRead, nAvailToRead );


		// If we read something
		if( nRead > 0 )
		{
			// Make sure we have a valid ascii response, and not garbage on the camlink Rx/Tx lines
			if ( isAscii( pBuffer, strlen(pBuffer) ) == false )
			{
				epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize, "Invalid ascii response!" );
				asynPrint(	pasynUser, ASYN_TRACE_ERROR,
							"%s port %s: Read error: %s\n",
							functionName, this->portName, pasynUser->errorMessage );
				status = asynError;
				m_fConnected = false;
				m_fInputFlushNeeded = true;
				pasynManager->exceptionDisconnect( pasynUser );
				if ( eomReason )
					*eomReason = ASYN_EOM_EOS;
				break;		/* If we have an error, we're done. */
			}

			break;			/* If we have something, we're done. */
		}

		// Handle errors
		if (	(nAvailToRead < 0 || nRead < 0)
				&&	(errno != EWOULDBLOCK)
				&&	(errno != EINTR)
				&&	(errno != EAGAIN) )
		{
			epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
							"%s: %s read error: %s\n",
							functionName, this->portName, strerror(errno)	);
			status = asynError;
			m_fInputFlushNeeded = true;
			break;		/* If we have an error, we're done. */
		}
		if ( pasynUser->timeout > 0 )
			break;			/* If we aren't waiting forever, we're done. */
	}	// end forever loop


	if ( nRead == 0 && (pasynUser->timeout > 0) && (status == asynSuccess))	/* If we don't have anything, not even an error	*/
	{
		status = asynTimeout;					/* then we must have timed out.					*/
		if ( eomReason )
			*eomReason = ASYN_EOM_EOS;
	}

	*pnRead = nRead;
	if ( nRead < static_cast<int>( sReadBuffer ) )
	{
		/* If there is room add a null byte */
		pBuffer[nRead] = 0;
		if ( eomReason )
			*eomReason = ASYN_EOM_EOS;
	}
	else if ( nRead == static_cast<int>( sReadBuffer ) )
	{
		if ( eomReason )
			*eomReason = ASYN_EOM_CNT;
	}

	if ( *pnRead > 0 )
	{
		if ( isAscii( pBuffer, strlen(pBuffer) ) )
		{
			if ( DEBUG_EDT_SER >= 3 )
				printf( "%s: %s Read %zu: %s\n", functionName, this->portName, *pnRead, pBuffer );
			asynPrintIO(	pasynUser, ASYN_TRACEIO_DRIVER, pBuffer, nRead,
							"%s: %s  read  %d of %d\n",
							functionName, this->portName, nRead, nAvailToRead );
			asynPrint(		pasynUser, ASYN_TRACE_FLOW,
							"%s: %s  read  %zu, status %d, Buffer: %s\n",
							functionName, this->portName, *pnRead, status, pBuffer	);
		}
		else
		{
			if ( DEBUG_EDT_SER >= 3 )
				printf( "%s: %s Read %zu\n", functionName, this->portName, *pnRead );
			asynPrint(		pasynUser, ASYN_TRACE_FLOW,
							"%s: %s read %zu, status %d\n",
							functionName, this->portName, *pnRead, status );
		}

		// TODO: Do we need callParamCallbacks() here?
		// Call the parameter callbacks
		callParamCallbacks();
	}

    return status;
}

asynStatus	asynEdtPdvSerial::writeOctet(
	asynUser			*	pasynUser,
	const char			*	pBuffer,
	size_t					maxChars,
	size_t				*	pnWritten	)
{
	asynStatus				status			= asynSuccess;
    static const char	*	functionName	= "asynEdtPdvSerial::writeOctet";
    // const char			*	reasonName		= "unknownReason";

	// getParamName( 0, pasynUser->reason, &reasonName );
	asynPrint(	pasynUser, ASYN_TRACE_FLOW,
				"%s: %s maxChars %zu\n", functionName, this->portName, maxChars );

	if ( pnWritten )
		*pnWritten = 0;

	if ( maxChars == 0 )
		return asynSuccess;

	if ( m_pPdvDev == NULL )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s pdvDev disconnected!\n", functionName, this->portName );
		m_fInputFlushNeeded = true;
		return asynError;
	}
	if ( !m_fConnected )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s Error: %s disconnected:", functionName, this->portName );
		m_fInputFlushNeeded = true;
		return asynError;
	}

	// See if we need to flush the serial input buffer
	if ( m_fInputFlushNeeded )
	{
		// Flush the read buffer
		char		flushBuf[1000];
		int			nAvailToRead = 0;
		epicsMutexLock( m_serialLock );
		if ( m_pPdvDev )
		{
			nAvailToRead = pdv_serial_wait( m_pPdvDev, 500, 1000 );
			if ( nAvailToRead > 0 )
				nAvailToRead = pdv_serial_read( m_pPdvDev, flushBuf, nAvailToRead );
		}
		epicsMutexUnlock( m_serialLock );
		printf( "%s: Flushed %d bytes\n", functionName, nAvailToRead );
		m_fInputFlushNeeded = false;
	}

	const char		*	pSendBuffer	= pBuffer;
	size_t				sSendBuffer	= maxChars;
	uint16_t			requestId	= 0xFFFF;

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
	int		pdv_status	= -1;
	epicsMutexLock( m_serialLock );
	if ( requestId != 0xFFFF )
	{
		if ( DEBUG_EDT_SER >= 3 )
			printf( "REQUESTID %-5hu: Sending  %zu bytes\n", requestId, sSendBuffer );
	}
	if ( m_pPdvDev )
		pdv_status = pdv_serial_write( m_pPdvDev, pSendBuffer, sSendBuffer );
	epicsMutexUnlock( m_serialLock );

	if ( pdv_status == 0 )
	{
		*pnWritten = sSendBuffer;

		if ( isAscii( pBuffer, strlen(pBuffer) ) )
		{
			asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
						"%s: wrote %zu to %s: %s\n",
						functionName, *pnWritten, this->portName, pBuffer	);
			asynPrintIO(pasynUser, ASYN_TRACEIO_DRIVER, pBuffer, *pnWritten,
						"%s: %s wrote %zu\n", functionName, this->portName, *pnWritten );
		}
		else
		{
			asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
						"%s: wrote %zu to %s\n",
						functionName, *pnWritten, this->portName );
		}
	}
	else if ( pdv_status != 0 )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s write error: %s\n",
						functionName, this->portName, strerror(errno)	);
		status = asynError;
		m_fInputFlushNeeded = TRUE;
	}

	// TODO: Do we need callParamCallbacks() here?
	// Call the parameter callbacks
    callParamCallbacks();

    return status;
}


#if 0
# TODO: Should we support flushOctet?
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

    if ( details >= 2 )
	{
		int			connected	= 0;
		pasynManager->isConnected( this->pasynUserSelf, &connected );
		if ( m_fConnected && !connected )
		{
			fprintf(	fp, "Warning, Camera serial port %s thinks it's %s, but asynManager says %s\n",
						portName,
						m_fConnected	? "Connected" : "Disconnected",
						connected		? "Connected" : "Disconnected"	);
		}
	}
    if ( details >= 1 )
	{
        fprintf( fp, "InputEosOctet  0x%p, len %d\n", m_inputEosOctet,  m_inputEosLenOctet  );
        fprintf( fp, "OutputEosOctet 0x%p, len %d\n", m_outputEosOctet, m_outputEosLenOctet );

		/* Call the base class method */
		asynPortDriver::report( fp, details );
    }
}

//	Private member variables

extern "C"
{
	epicsExportAddress( int, DEBUG_EDT_SER );
}
