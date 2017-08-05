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

#include "asynEdtPdvSerial.h"
#include "GenCpPacket.h"

#include "edtinc.h"

extern int	DEBUG_EDT_PDV;

#define	MAX_ADDR		1
#define	NUM_PARAMS		1

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
							NUM_PARAMS,
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
	m_fInputFlushNeeded(	false		),
	m_serialLock(						),
	m_GenCPRegAddr(			0LL			),
	m_GenCPRequestId(		0			),
	m_GenCPResponseType(	0			),
	m_GenCPResponseCount(	0			),
	m_GenCPResponseSize(	0			),
	m_GenCPResponsePending(				)
{
	const char		*	functionName	= "asynEdtPdvSerial::asynEdtPdvSerial";
	//	asynStatus			status;
	//	int					nbytes;
	//	asynOctet		*	pasynOctet;

	// Create mutexes
    m_serialLock	= epicsMutexMustCreate();

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

	if ( DEBUG_EDT_PDV >= 1 )
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
	if ( DEBUG_EDT_PDV >= 1 )
		printf( "%s: %s Disconnecting ...\n", functionName, this->portName );

	epicsMutexLock(m_serialLock);
	if ( DEBUG_EDT_PDV >= 3 )
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

	if ( DEBUG_EDT_PDV >= 3 )
		printf( "%s: %s Disconnected\n", functionName, this->portName );
	return status;
}

bool isAscii( char * pBuf, int sBuf )
{
	if ( pBuf == NULL || sBuf == 0 )
		return false;

	char	*	pBufEnd	= pBuf + sBuf;
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
	char					genCpResponseBuffer[EDT_GENCP_RESPONSE_MAX];
    
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
	
	// TODO: Could the GenCP processing be implemented as an interposeInterface?
	bool				fGenCP		= FALSE;
	char			*	pReadBuffer	= pBuffer;
	size_t				sReadBuffer	= nBytesReadMax;
	GenCpReadMemAck		genCpReadMemAck;
	GenCpWriteMemAck	genCpWriteMemAck;
	if ( strncmp( m_pPdvDev->dd_p->serial_trigger, "GenCP", MAXSER ) == 0 )
	{
		fGenCP		= TRUE;
		size_t		nBytesPending = strlen( m_GenCPResponsePending );
		if ( nBytesPending > 0 )
		{
			// Unable to return entire response on last call
			// Typically because streamdevice sets nBytesReadMax to 1 on first call
			*pnRead = nBytesPending;
			strncpy( pBuffer, m_GenCPResponsePending, nBytesReadMax );
			m_GenCPResponsePending[0] = '\0';
			if ( eomReason )
				*eomReason = ASYN_EOM_EOS;

			if ( DEBUG_EDT_PDV >= 3 )
				printf( "%s: %s Read pending %zu: %s\n", functionName, this->portName, nBytesPending, pBuffer );
			asynPrint(	pasynUser, ASYN_TRACE_FLOW,
						"%s: %s read pending %zu, status %d, Buffer: %s\n",
						functionName, this->portName, nBytesPending, status, pBuffer	);

			// Call the parameter callbacks
			callParamCallbacks();
			return asynSuccess;
		}

		switch ( m_GenCPResponseType )
		{
		case EDT_GENCP_TY_RESP_ACK:
			pReadBuffer	= reinterpret_cast<char *>( &genCpWriteMemAck );
			sReadBuffer	= m_GenCPResponseSize;
			break;
		case EDT_GENCP_TY_RESP_STRING:
		case EDT_GENCP_TY_RESP_UINT:
		case EDT_GENCP_TY_RESP_INT:
		case EDT_GENCP_TY_RESP_FLOAT:
		case EDT_GENCP_TY_RESP_DOUBLE:
			pReadBuffer	= reinterpret_cast<char *>( &genCpReadMemAck );
			sReadBuffer	= m_GenCPResponseSize;
			break;
		}
	}

	int		nRead	= 0;
	for (;;)
	{
		epicsMutexLock(m_serialLock);
		if ( DEBUG_EDT_PDV >= 4 )
			printf( "%s: %s Have serial lock, nBytesReadMax %zu, sReadBuffer %zu, timeout %e ...\n",
					functionName, this->portName, nBytesReadMax, sReadBuffer, pasynUser->timeout );
		/*
		 * Follow streamdevice usage on timeout: <= 0 is don't wait, > 0 specifies delay in sec
		 * In pdv_serial_wait, 0 = wait for the default time (1 sec?).
		 */
		if ( m_pPdvDev && m_fConnected )
		{
            int nMsTimeout = 500; // Default timeout of 500 milliseconds
			if ( pasynUser->timeout > 0 )
				nMsTimeout	= static_cast<int>( pasynUser->timeout * 1000 );
			nAvailToRead = pdv_serial_wait( m_pPdvDev, nMsTimeout, sReadBuffer );
		}
		if( nAvailToRead > 0 )
		{
			int		nToRead	= nAvailToRead;
			if( nToRead > static_cast<int>(sReadBuffer) )
			{
				if ( DEBUG_EDT_PDV >= 3 )
					printf( "%s: %s Clipping nAvailToRead %d to sReadBuffer %zu\n",
							functionName, this->portName, nAvailToRead, sReadBuffer );
				nToRead = static_cast<int>(sReadBuffer);
			}
			asynPrint(	pasynUser, ASYN_TRACE_FLOW,
						"%s: %s nToRead %d\n", functionName, this->portName, nToRead );

			epicsMutexLock(m_serialLock);
			if ( DEBUG_EDT_PDV >= 3 )
				printf( "%s: %s Have serial lock, reading %d ...\n", functionName, this->portName, nToRead );
			if ( m_pPdvDev && m_fConnected )
				nRead = pdv_serial_read( m_pPdvDev, pReadBuffer, nToRead );
			else
				nRead = -1;
			epicsMutexUnlock(m_serialLock);
			if ( DEBUG_EDT_PDV >= 3 )
				printf( "%s: %s Released serial lock, read %d ...\n", functionName, this->portName, nRead );
		}
		else
		{
            // nAvailToRead <=0 so nothing to do here... fly away!
            *pnRead = 0;
            epicsMutexUnlock(m_serialLock);
            if ( DEBUG_EDT_PDV >= 4 )
                printf( "%s: %s Released serial lock, nRead=0, nAvailToRead %d ...\n", functionName, this->portName, nAvailToRead );

            return asynSuccess;
        }

        epicsMutexUnlock(m_serialLock);
		if ( DEBUG_EDT_PDV >= 4 )
			printf( "%s: %s Released serial lock, nAvailToRead %d ...\n", functionName, this->portName, nAvailToRead );


		// If we read something
		if( nRead > 0 )
		{
			// Make sure we have a valid ascii response, and not garbage on the camlink Rx/Tx lines
			if ( !fGenCP && isAscii( pBuffer, strlen(pBuffer) ) == false )
			{
				epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize, "Invalid ascii response!" );
				asynPrint(	pasynUser, ASYN_TRACE_ERROR,
							"%s port %s: Read error: %s\n",
							functionName, this->portName, pasynUser->errorMessage );
				status = asynError;
				m_fConnected = false;
				m_fInputFlushNeeded = TRUE;
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
			m_fInputFlushNeeded = TRUE;
			break;		/* If we have an error, we're done. */
		}
		if ( pasynUser->timeout > 0 )
			break;			/* If we aren't waiting forever, we're done. */
	}	// end forever loop

	if ( DEBUG_EDT_PDV >= 4 )
		printf( "%s: %s Read %d\n", functionName, this->portName, nRead );

	if ( nRead == 0 && (pasynUser->timeout > 0) && (status == asynSuccess))	/* If we don't have anything, not even an error	*/
	{
		status = asynTimeout;					/* then we must have timed out.					*/
		if ( eomReason )
			*eomReason = ASYN_EOM_EOS;
	}

	if ( fGenCP )
	{
		size_t					nBytesRead;
		GENCP_STATUS			genStatus;
		GenCpReadMemAck		*	pReadAck	= reinterpret_cast<GenCpReadMemAck *>(	pReadBuffer );
		GenCpWriteMemAck	*	pWriteAck	= reinterpret_cast<GenCpWriteMemAck *>(	pReadBuffer );

		switch ( m_GenCPResponseType )
		{
		case EDT_GENCP_TY_RESP_ACK:
			genStatus = GenCpValidateWriteMemAck( pWriteAck, m_GenCPRequestId );
			if ( genStatus != GENCP_STATUS_SUCCESS )
			{
				// TODO: Add status code to error msg translation here
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "ERR %d (0x%X)\n", genStatus, genStatus );
				fprintf( stderr, "%s: ReadMemString Validate Error: %d (0x%X)\n", functionName, genStatus, genStatus );
				m_fInputFlushNeeded = TRUE;
				return asynError;
			}
			strncpy( genCpResponseBuffer, "OK\n", EDT_GENCP_RESPONSE_MAX );
			break;
		case EDT_GENCP_TY_RESP_STRING:
			snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "R0x%LX=", m_GenCPRegAddr );
			genStatus = GenCpProcessReadMemAck( pReadAck, m_GenCPRequestId-1, genCpResponseBuffer + strlen(genCpResponseBuffer), (size_t)(EDT_GENCP_RESPONSE_MAX - strlen(genCpResponseBuffer)), &nBytesRead );
			if ( genStatus != GENCP_STATUS_SUCCESS )
			{
				// TODO: Add status code to error msg translation here
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "ERR %d (0x%X)\n", genStatus, genStatus );
				fprintf( stderr, "%s: ReadMemString Validate Error: %d (0x%X)\n", functionName, genStatus, genStatus );
				m_fInputFlushNeeded = TRUE;
				return asynError;
			}
			strcat( genCpResponseBuffer, "\n" );
			break;
		case EDT_GENCP_TY_RESP_UINT:
			switch ( m_GenCPResponseCount )
			{
			case 16:
				uint16_t	valueUint16;
				genStatus = GenCpProcessReadMemAck( pReadAck, m_GenCPRequestId-1, &valueUint16 );
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "R0x%llX=%hu (0x%02hX)\n", m_GenCPRegAddr, valueUint16, valueUint16 );
				break;
			case 32:
				uint32_t	valueUint32;
				genStatus = GenCpProcessReadMemAck( pReadAck, m_GenCPRequestId-1, &valueUint32 );
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "R0x%llX=%u (0x%04X)\n", m_GenCPRegAddr, valueUint32, valueUint32 );
				break;
			case 64:
				uint64_t	valueUint64;
				genStatus = GenCpProcessReadMemAck( pReadAck, m_GenCPRequestId-1, &valueUint64 );
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "R0x%llX=%llu (0x%08llX)\n", m_GenCPRegAddr,
						(long long unsigned int) valueUint64, (long long unsigned int) valueUint64 );
				break;
			default:
				genStatus	= GENCP_STATUS_INVALID_PARAM;
				break;
			}
			if ( genStatus != GENCP_STATUS_SUCCESS )
			{
				// TODO: Add status code to error msg translation here
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "ERR %d (0x%X)\n", genStatus, genStatus );
				fprintf( stderr, "%s: Uint ProcessReadMem Error: %d (0x%X)\n", functionName, genStatus, genStatus );
				m_fInputFlushNeeded = TRUE;
				return asynError;
			}
			break;
		case EDT_GENCP_TY_RESP_FLOAT:
			switch ( m_GenCPResponseCount )
			{
			case 32:
				float		floatValue;
				genStatus = GenCpProcessReadMemAck( pReadAck, m_GenCPRequestId-1, &floatValue );
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "R0x%llX=%f\n", m_GenCPRegAddr, floatValue );
				break;
			case 64:
				double		doubleValue;
				genStatus = GenCpProcessReadMemAck( pReadAck, m_GenCPRequestId-1, &doubleValue );
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "R0x%llX=%lf\n", m_GenCPRegAddr, doubleValue );
				break;
			default:
				genStatus	= GENCP_STATUS_INVALID_PARAM;
				break;
			}
			if ( genStatus != GENCP_STATUS_SUCCESS )
			{
				// TODO: Add status code to error msg translation here
				snprintf( genCpResponseBuffer, EDT_GENCP_RESPONSE_MAX, "ERR %d (0x%X)\n", genStatus, genStatus );
				fprintf( stderr, "%s: Uint ProcessReadMem Error: %d (0x%X)\n", functionName, genStatus, genStatus );
				m_fInputFlushNeeded = TRUE;
				return asynError;
			}
			break;
		case EDT_GENCP_TY_RESP_INT:
		default:
			fprintf( stderr, "%s: Unsupported response type: %d\n", functionName, m_GenCPResponseType );
			return asynError;
			break;
		}
	}

	if ( fGenCP )
	{
		size_t		nBytesResponse = strlen( genCpResponseBuffer );
		if ( nBytesResponse > nBytesReadMax )
		{
			// Copy requested number of characters to return buffer
			strncpy( pBuffer, genCpResponseBuffer, nBytesReadMax );
			*pnRead = nBytesReadMax;
			if ( eomReason )
				*eomReason = ASYN_EOM_CNT;
			// Save remaining response characters for next call to readOctet
			strncpy( m_GenCPResponsePending, genCpResponseBuffer + nBytesReadMax, EDT_GENCP_RESPONSE_MAX - 1 );
		}
		else
		{
			// Copy response to return buffer
			strncpy( pBuffer, genCpResponseBuffer, nBytesReadMax );
			*pnRead = nBytesResponse;
			if ( eomReason )
				*eomReason = ASYN_EOM_EOS;
		}
	}
	else
	{
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
	}

	if ( *pnRead > 0 )
	{
		if ( DEBUG_EDT_PDV >= 3 )
			printf( "%s: %s Read %zu: %s\n", functionName, this->portName, *pnRead, pBuffer );
		asynPrintIO(	pasynUser, ASYN_TRACEIO_DRIVER, pBuffer, nRead,
						"%s: %s read %d of %d\n", functionName, this->portName,
						nRead, nAvailToRead );
		asynPrint(	pasynUser, ASYN_TRACE_FLOW,
					"%s: %s read %zu, status %d, Buffer: %s\n",
					functionName, this->portName, *pnRead, status, pBuffer	);

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
		m_fInputFlushNeeded = TRUE;
		return asynError;
	}
	if ( !m_fConnected )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s Error: %s disconnected:", functionName, this->portName );
		m_fInputFlushNeeded = TRUE;
		return asynError;
	}

	// See if we need to flush the serial input buffer
	if ( m_fInputFlushNeeded )
	{
		// Flush the read buffer
		char		flushBuf[1000];
		(void) pdv_serial_read( m_pPdvDev, flushBuf, 1000 );
		m_fInputFlushNeeded = FALSE;
	}

	bool				fGenCP		= FALSE;
	const char		*	pSendBuffer	= pBuffer;
	size_t				sSendBuffer	= maxChars;
	GenCpReadMemPacket	genCpReadMemPacket;
	GenCpWriteMemPacket	genCpWriteMemPacket;
	if ( strncmp( m_pPdvDev->dd_p->serial_trigger, "GenCP", MAXSER ) == 0 )
	{
		GENCP_STATUS	genStatus;
		char			cGetSet;	// '?' is a Get, '=' is a Set
		unsigned int	cmdCount;
		char		*	pString;
		double			doubleValue	= 0.0;
		int64_t			intValue	= 0LL;
		uint64_t		regAddr		= 0LL;
		int				scanCount	= -1;
		char		*	pEqualSign	= strchr( pBuffer, '=' );

		fGenCP		= TRUE;	// GenCP Protocol
		m_GenCPResponsePending[0] = '\0';

		// Parse the simple streamdevice ascii protocol and replace it w/ a GenCpReadMemPacket.
		switch ( *pBuffer )
		{
		case 'C':
			scanCount = sscanf( pBuffer, "C%u %Li %c", &cmdCount, (long long int *) &regAddr, &cGetSet );
			if ( scanCount == 3 && cGetSet == '=' && cmdCount > 0 )
			{
				assert( pEqualSign != NULL );
				pString		= pEqualSign + 1;
				genStatus	= GenCpInitWriteMemPacket(	&genCpWriteMemPacket, m_GenCPRequestId++, regAddr,
														cmdCount, pString, &sSendBuffer );
				pSendBuffer	= reinterpret_cast<char *>( &genCpWriteMemPacket );
				m_GenCPResponseCount	= cmdCount;
				m_GenCPResponseType		= EDT_GENCP_TY_RESP_ACK;
				m_GenCPResponseSize		= sizeof(GenCpWriteMemAck);
			}
			else if ( scanCount == 3 && cGetSet == '?' && cmdCount > 0 )
			{
				genStatus	= GenCpInitReadMemPacket( &genCpReadMemPacket, m_GenCPRequestId++, regAddr, cmdCount );
				pSendBuffer	= reinterpret_cast<char *>( &genCpReadMemPacket );
				sSendBuffer	= sizeof(genCpReadMemPacket);
				m_GenCPResponseCount	= cmdCount;
				m_GenCPResponseType		= EDT_GENCP_TY_RESP_STRING;
				m_GenCPResponseSize		= sizeof(GenCpSerialPrefix) + sizeof(GenCpCCDAck) + cmdCount;
			}
			else
				scanCount = -1;
			break;

		case 'U':
			scanCount = sscanf( pBuffer, "U%u %Li %c%Li", &cmdCount, (long long int *) &regAddr, &cGetSet, (long long int *) &intValue );
			asynPrint(	pasynUser, ASYN_TRACE_FLOW,
						"%s %s: scanCount=%d, cmdCount=%u, regAddr=0x%llX, cGetSet=%c, intValue=%lld, command: %s\n",
						functionName, this->portName, scanCount, cmdCount, (long long unsigned int) regAddr, cGetSet, (long long int) intValue, pBuffer );
			if ( scanCount == 4 && cGetSet == '=' && cmdCount > 0 )
			{
				uint16_t	value16	= static_cast<uint16_t>( intValue );
				uint32_t	value32	= static_cast<uint32_t>( intValue );
				uint64_t	value64	= static_cast<uint64_t>( intValue );
				assert( pEqualSign != NULL );
				switch ( cmdCount )
				{
				case 16:
					genStatus = GenCpInitWriteMemPacket(	&genCpWriteMemPacket, m_GenCPRequestId++, regAddr,
															value16, &sSendBuffer );
					break;
				case 32:	
					genStatus = GenCpInitWriteMemPacket(	&genCpWriteMemPacket, m_GenCPRequestId++, regAddr,
															value32, &sSendBuffer );
					break;
				case 64:	
					genStatus = GenCpInitWriteMemPacket(	&genCpWriteMemPacket, m_GenCPRequestId++, regAddr,
															value64, &sSendBuffer );
					break;
				}
				pSendBuffer	= reinterpret_cast<char *>( &genCpWriteMemPacket );
				m_GenCPResponseCount	= cmdCount;
				m_GenCPResponseType		= EDT_GENCP_TY_RESP_ACK;
				m_GenCPResponseSize		= sizeof(GenCpWriteMemAck);
			}
			else if ( scanCount == 3 && cGetSet == '?' && cmdCount > 0 )
			{
				genStatus	= GenCpInitReadMemPacket( &genCpReadMemPacket, m_GenCPRequestId++, regAddr, cmdCount / 8 );
				pSendBuffer	= reinterpret_cast<char *>( &genCpReadMemPacket );
				sSendBuffer	= sizeof(genCpReadMemPacket);
				m_GenCPResponseCount	= cmdCount;
				m_GenCPResponseType		= EDT_GENCP_TY_RESP_UINT;
				m_GenCPResponseSize		= sizeof(GenCpSerialPrefix) + sizeof(GenCpCCDAck) + cmdCount / 8;
			}
			else
				scanCount = -1;
			break;

		case 'F':
			scanCount = sscanf( pBuffer, "F%u %Li %c%lf", &cmdCount, (long long int *) &regAddr, &cGetSet, &doubleValue );
			asynPrint(	pasynUser, ASYN_TRACE_FLOW,
						"%s %s: scanCount=%d, cmdCount=%u, regAddr=0x%llX, cGetSet=%c, doubleValue=%lf, command: %s\n",
						functionName, this->portName, scanCount, cmdCount, (long long unsigned int) regAddr, cGetSet, doubleValue, pBuffer );
			if ( scanCount == 4 && cGetSet == '=' && cmdCount > 0 )
			{
				float	floatValue	= static_cast<float>( doubleValue );
				assert( pEqualSign != NULL );
				switch ( cmdCount )
				{
				case 32:	
					genStatus = GenCpInitWriteMemPacket(	&genCpWriteMemPacket, m_GenCPRequestId++, regAddr,
															floatValue, &sSendBuffer );
					break;
				case 64:	
					genStatus = GenCpInitWriteMemPacket(	&genCpWriteMemPacket, m_GenCPRequestId++, regAddr,
															doubleValue, &sSendBuffer );
					break;
				}
				pSendBuffer	= reinterpret_cast<char *>( &genCpWriteMemPacket );
				m_GenCPResponseCount	= cmdCount;
				m_GenCPResponseType		= EDT_GENCP_TY_RESP_ACK;
				m_GenCPResponseSize		= sizeof(GenCpWriteMemAck);
			}
			else if ( scanCount == 3 && cGetSet == '?' && cmdCount > 0 )
			{
				genStatus	= GenCpInitReadMemPacket( &genCpReadMemPacket, m_GenCPRequestId++, regAddr, cmdCount / 8 );
				pSendBuffer	= reinterpret_cast<char *>( &genCpReadMemPacket );
				sSendBuffer	= sizeof(genCpReadMemPacket);
				m_GenCPResponseCount	= cmdCount;
				if ( cmdCount == 32 )
					m_GenCPResponseType		= EDT_GENCP_TY_RESP_FLOAT;
				else
					m_GenCPResponseType		= EDT_GENCP_TY_RESP_DOUBLE;
				m_GenCPResponseSize		= sizeof(GenCpSerialPrefix) + sizeof(GenCpCCDAck) + cmdCount / 8;
			}
			else
				scanCount = -1;
			break;

		case 'I':
		default:
			break;
		}
		m_GenCPRegAddr = regAddr;

		if ( scanCount == -1 )
		{
			epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
							"%s: %s Invalid GenCP command: %s\n",
							functionName, this->portName, pBuffer	);
			m_fInputFlushNeeded = TRUE;
			return asynError;
		}

		asynPrint(	pasynUser, ASYN_TRACE_FLOW,
					"%s %s: responseType=%u, responseCount=%u, responseSize=%u\n",
					functionName, this->portName, m_GenCPResponseType, m_GenCPResponseCount, m_GenCPResponseSize );
	}

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
	if ( m_pPdvDev )
		pdv_status = pdv_serial_write( m_pPdvDev, pSendBuffer, sSendBuffer );
	epicsMutexUnlock( m_serialLock );

	if ( pdv_status == 0 )
	{
		if ( fGenCP )
			*pnWritten = strlen( pBuffer );
		else
			*pnWritten = sSendBuffer;

		asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
					"%s: wrote %zu to %s: %s\n",
					functionName, *pnWritten, this->portName, pBuffer	);
		asynPrintIO(	pasynUser, ASYN_TRACEIO_DRIVER, pBuffer, *pnWritten,
						"%s: %s wrote %zu\n", functionName, this->portName, *pnWritten );
	}
	else if ( pdv_status != 0 )
	{
		epicsSnprintf(	pasynUser->errorMessage, pasynUser->errorMessageSize,
						"%s: %s write error: %s\n",
						functionName, this->portName, strerror(errno)	);
		status = asynError;
		m_fInputFlushNeeded = TRUE;
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
