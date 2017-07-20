#include "GenCpPacket.h"
#include "GenCpRegister.h"
#include <asm/byteorder.h>	// For __cpu_to_be64() and variants
#include <assert.h>
#include <stdio.h>


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

