#include "GenTL.h"
#include "GenCpPacket.h"
#include "GenCpRegister.h"
#include <asm/byteorder.h>	// For __cpu_to_be64() and variants
#include <assert.h>
#include <stdio.h>
#include <string.h>


/// GenCpCheckSum16() from Allied Vision Goldeye G/CL Features Reference V1.2.0
/// Assumes packet data exists in memory in big endian format and the host is little endian.
/// Return value is host format
uint16_t GenCpChecksum16( uint8_t* pBuffer, uint32_t nNumBytes )
{
	uint32_t nChecksum = 0;
	uint32_t nByteCounter;
	uint32_t nNumBytesEven = nNumBytes & ~(sizeof(uint16_t) - 1);

	// for reasons of performance, this function is limited to 64Kb length.
	// Since the GenCP standard recommends to have packets <= 1Kb, this should not be a problem.
	assert(nNumBytes < 65535);
	for (nByteCounter = 0; nByteCounter < nNumBytesEven; nByteCounter += sizeof(uint16_t))
	{
		// Two bytes from pBuffer are interpreted as a big endian 16 bit value
		// Couldn't we just do this?
		uint16_t	nCurVal2=	__be16_to_cpup( reinterpret_cast<__be16*>( pBuffer + nByteCounter ) );
		uint16_t	nCurVal	=	(	(( (uint16_t) pBuffer[nByteCounter    ] ) << 8)
								|	 ( (uint16_t) pBuffer[nByteCounter + 1] ) );
		if ( nCurVal2 != nCurVal )
			printf( "Error: nCurVal=0x%04X, nCurVal2=0x%04X\n", nCurVal, nCurVal2 );
		nChecksum += (uint32_t) nCurVal;
	}
	if ((nNumBytes & (sizeof(uint16_t) - 1)) != 0)
	{
		// special case: buffer length is odd number
		nChecksum += (((uint32_t) pBuffer[nNumBytesEven]) << 8);
	}
	while ((nChecksum & 0xFFFF0000) != 0)
	{
		nChecksum = (nChecksum & 0xFFFF) + (nChecksum >> 16);
	}
	return(~((uint16_t) nChecksum));
}

/// GenCpInitReadMemPacket()
GENCP_STATUS	GenCpInitReadMemPacket(
	GenCpReadMemPacket		*	pPacket,
	uint16_t					requestId,
	uint64_t					regAddr,
	size_t						numBytes )
{
	if ( pPacket == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;

	pPacket->serialPrefix.prefixPreamble	= __cpu_to_be16( GENCP_SERIAL_PREAMBLE );
	pPacket->serialPrefix.prefixChannelId	= 0;
	pPacket->ccd.ccdFlags					= __cpu_to_be16( GENCP_CCD_FLAG_REQACK );
	pPacket->ccd.ccdCommandId				= __cpu_to_be16( GENCP_ID_READMEM_CMD );
	pPacket->ccd.ccdScdLength				= __cpu_to_be16( sizeof( GenCpSCDReadMem ) );
	pPacket->ccd.ccdRequestId				= __cpu_to_be16( requestId );
	pPacket->scd.scdRegAddr					= __cpu_to_be64( regAddr );
	pPacket->scd.scdReserved				= 0;
	pPacket->scd.scdReadSize				= __cpu_to_be16( numBytes );

	// Compute CCD and SCD Checksums
	uint32_t	ckSumCCD	= GenCpChecksum16(	reinterpret_cast<uint8_t *>( &pPacket->serialPrefix.prefixChannelId ),
												sizeof(uint16_t) + sizeof(GenCpCCDRequest) );
	uint32_t	ckSumSCD	= GenCpChecksum16(	reinterpret_cast<uint8_t *>( &pPacket->serialPrefix.prefixChannelId ),
												sizeof(uint16_t) + sizeof(GenCpCCDRequest) + sizeof(GenCpSCDReadMem) );
	pPacket->serialPrefix.prefixCkSumCCD	= __cpu_to_be16( ckSumCCD );
	pPacket->serialPrefix.prefixCkSumSCD	= __cpu_to_be16( ckSumSCD );

	return GENCP_STATUS_SUCCESS;
}

/// GenCpValidateReadMemAck()
GENCP_STATUS	GenCpValidateReadMemAck(
	GenCpReadMemAck			*	pPacket	)
{
	const	char 			*	funcName = "GenCpValidateReadMemAck";
	if ( pPacket == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;

	uint16_t	prefixPreamble	= __be16_to_cpu( pPacket->serialPrefix.prefixPreamble );
	uint16_t	ccdRequestId	= __be16_to_cpu( pPacket->ccd.ccdRequestId );
	if ( prefixPreamble	!= GENCP_SERIAL_PREAMBLE )
	{
		fprintf( stderr, "%s Error: Req %u, Invalid preamble, 0x%02X\n", funcName, ccdRequestId, prefixPreamble );
		return GENCP_STATUS_INVALID_PARAM | GENCP_SC_ERROR;
	}

	// Validate CCD Checksum
	uint32_t	ckSumCCD	= GenCpChecksum16(	reinterpret_cast<uint8_t *>( &pPacket->serialPrefix.prefixChannelId ),
												sizeof(uint16_t) + sizeof(GenCpCCDAck) );
	if ( ckSumCCD != __be16_to_cpu( pPacket->serialPrefix.prefixCkSumCCD ) )
	{
		fprintf( stderr, "%s Error: Req %u, Packet CCD cksum, 0x%04X, computed 0x%04X\n", funcName,
				ccdRequestId, ckSumCCD, __be16_to_cpu( pPacket->serialPrefix.prefixCkSumCCD ) );
		return GENCP_STATUS_INVALID_PARAM | GENCP_SC_ERROR;
	}

	// Validate SCD Checksum
	uint32_t	ckSumSCD	= GenCpChecksum16(	reinterpret_cast<uint8_t *>( &pPacket->serialPrefix.prefixChannelId ),
												sizeof(uint16_t) + sizeof(GenCpCCDAck) + __be16_to_cpu( pPacket->ccd.ccdScdLength ) );
	if ( ckSumSCD != __be16_to_cpu( pPacket->serialPrefix.prefixCkSumSCD ) )
	{
		fprintf( stderr, "%s Error: Req %u, Packet SCD cksum, 0x%04X, computed 0x%04X\n", funcName,
				ccdRequestId, ckSumSCD, __be16_to_cpu( pPacket->serialPrefix.prefixCkSumSCD ) );
		return GENCP_STATUS_INVALID_PARAM | GENCP_SC_ERROR;
	}

	uint16_t	ccdCommandId	= __be16_to_cpu( pPacket->ccd.ccdCommandId );
	if ( ccdCommandId	!= GENCP_ID_READMEM_ACK )
	{
		fprintf( stderr, "%s Error: Req %u, Invalid commandId, 0x%02X\n", funcName, ccdRequestId, ccdCommandId );
		return GENCP_STATUS_INVALID_PARAM | GENCP_SC_ERROR;
	}

	uint16_t	ccdScdLength	= __be16_to_cpu( pPacket->ccd.ccdScdLength );
	if ( ccdScdLength > GENCP_READMEM_MAX_BYTES )
	{
		fprintf( stderr, "%s Error: Req %u, SCD Length %u greater than max %u\n", funcName,
				ccdRequestId, ccdScdLength, GENCP_READMEM_MAX_BYTES );
		return GENCP_STATUS_INVALID_PARAM | GENCP_SC_ERROR;
	}

	uint16_t	ccdStatus		= __be16_to_cpu( pPacket->ccd.ccdStatusCode );
	uint16_t	ccdStatusCode	= ccdStatus & GENCP_SC_CODE_MASK;
	if ( ccdStatus & GENCP_SC_ERROR )
	{
		// uint16_t	ccdStatusNS		= ccdStatusCode & GENCP_SC_NAMESPACE_MASK;
		// if ( ccdStatusNS == GENCP_SC_NAMESPACE_GENCP )
		//		fprintf( stderr, "%s Error: Req %u, StatusCode Error %u: %s\n", funcName, ccdRequestId, ccdStatusCode,
		//				GenCpStatusCodeToString(ccdStatusCode) );
		fprintf( stderr, "%s Error: Req %u, StatusCode Error %u\n", funcName, ccdRequestId, ccdStatusCode );
		return ccdStatusCode;
	}

	return GENCP_STATUS_SUCCESS;
}

/// GenCpProcessReadMemAck() char buffer
GENCP_STATUS	GenCpProcessReadMemAck(
	GenCpReadMemAck			*	pPacket,
	char					*	pBuffer,
	size_t						sBuffer,
	size_t					*	pnBytesRead )
{
	const	char 			*	funcName = "GenCpProcessReadMemAck";
	if ( pPacket == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;
	if ( pBuffer == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;
	if ( sBuffer == 0 )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;
	if ( pnBytesRead != NULL )
		*pnBytesRead = 0;

	GENCP_STATUS	statusCode	= GenCpValidateReadMemAck( pPacket );
	if ( statusCode	!= GENCP_STATUS_SUCCESS )
	{
		return statusCode;
	}

	uint16_t	ccdScdLength	= __be16_to_cpu( pPacket->ccd.ccdScdLength );
	uint16_t	ccdRequestId	= __be16_to_cpu( pPacket->ccd.ccdRequestId );
	if ( ccdScdLength >= sBuffer )
	{
		fprintf( stderr, "%s Error: Req %u, SCD Length %d >= sBuffer %zu\n", funcName,
				ccdRequestId, ccdScdLength, sBuffer );
		return GENCP_STATUS_INVALID_PARAM | GENCP_SC_ERROR;
	}

	memcpy( reinterpret_cast<void *>(pBuffer),
			reinterpret_cast<void *>(&pPacket->scd.scdReadData[0]), ccdScdLength );
	if ( pnBytesRead != NULL )
		*pnBytesRead = ccdScdLength;
	return GENCP_STATUS_SUCCESS;
}

/// GenCpProcessReadMemAck() 16 bit reg
GENCP_STATUS	GenCpProcessReadMemAck(
	GenCpReadMemAck			*	pPacket,
	uint16_t				*	pReg16 )
{
	// const	char 			*	funcName = "GenCpProcessReadMemAck";
	if ( pPacket == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;
	if ( pReg16 == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;

	GENCP_STATUS	statusCode	= GenCpValidateReadMemAck( pPacket );
	if ( statusCode	!= GENCP_STATUS_SUCCESS )
	{
		return statusCode;
	}

	if ( pReg16 != NULL )
	{
		__be16	*	pBigEndianReg16	= reinterpret_cast<__be16 *>( &pPacket->scd.scdReadData[0] );
		*pReg16 = __be16_to_cpu( *pBigEndianReg16 );
	}
	return GENCP_STATUS_SUCCESS;
}

/// GenCpProcessReadMemAck() 32 bit reg
GENCP_STATUS	GenCpProcessReadMemAck(
	GenCpReadMemAck			*	pPacket,
	uint32_t				*	pReg32 )
{
	const	char 			*	funcName = "GenCpProcessReadMemAck";
	if ( pPacket == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;
	if ( pReg32 == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;

	GENCP_STATUS	statusCode	= GenCpValidateReadMemAck( pPacket );
	if ( statusCode	!= GENCP_STATUS_SUCCESS )
	{
		fprintf( stderr, "%s Error: %u\n", funcName, statusCode );
		// return statusCode;
	}

	if ( pReg32 != NULL )
	{
		__be32	*	pBigEndianReg32	= reinterpret_cast<__be32 *>( &pPacket->scd.scdReadData[0] );
		*pReg32 = __be32_to_cpu( *pBigEndianReg32 );
	}
	return GENCP_STATUS_SUCCESS;
}


/// GenCpProcessReadMemAck() 64 bit reg
GENCP_STATUS	GenCpProcessReadMemAck(
	GenCpReadMemAck			*	pPacket,
	uint64_t				*	pReg64 )
{
	// const	char 			*	funcName = "GenCpProcessReadMemAck";
	if ( pPacket == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;
	if ( pReg64 == NULL )
		return GENCP_STATUS_GENERIC_ERROR | GENCP_SC_ERROR;

	GENCP_STATUS	statusCode	= GenCpValidateReadMemAck( pPacket );
	if ( statusCode	!= GENCP_STATUS_SUCCESS )
	{
		return statusCode;
	}

	if ( pReg64 != NULL )
	{
		__be64	*	pBigEndianReg64	= reinterpret_cast<__be64 *>( &pPacket->scd.scdReadData[0] );
		*pReg64 = __be64_to_cpu( *pBigEndianReg64 );
	}
	return GENCP_STATUS_SUCCESS;
}

