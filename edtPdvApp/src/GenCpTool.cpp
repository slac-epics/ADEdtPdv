#include "edtinc.h"
#include "pciload.h"
#include "GenCpPacket.h"


// GenCp Request ID, start at 0, increment each request
static uint16_t		localGenCpRequestId	= 0;

void usage( char * msg )
{
    printf( "%s", msg );
    printf( "Usage: \n" );
    printf(
       "    -h              - Help message\n"
       "    --help          - Help message\n"
       "    -c N            - channel #\n"
       "    --channel N     - channel #\n"
       "    -u N            - Unit number (default 0)\n"
       "    --unit N        - Unit number (default 0)\n"
       "    --readXml       - Read XML GeniCam file and dump to stdout\n"
       "    --U32 Addr      - Read 32 bit unsigned value from address\n"
       "    -v              - Verbose\n"
    );
}

GENCP_STATUS PdvGenCpReadUint(
    EdtDev			*	pPdv,
	uint64_t			regAddr,
	size_t				numBytes,
	uint64_t		*	pnResult	)
{
	const char		*	functionName = "EdtGenCpReadUint";
	GENCP_STATUS		status;
	GenCpReadMemPacket	readMemPacket;
	GenCpReadMemAck		ackPacket;
	uint64_t			result = static_cast<unsigned int>( -1 );

	if ( pnResult != NULL )
		*pnResult = result;

	status = GenCpInitReadMemPacket( &readMemPacket, localGenCpRequestId++, regAddr, numBytes );
	if ( status != GENCP_STATUS_SUCCESS )
	{
		fprintf( stderr, "GenCP Error: %d\n", status );
		return status;
	}

	status = pdv_serial_write( pPdv, reinterpret_cast<char *>( &readMemPacket ), sizeof(readMemPacket) );
	int		nMsTimeout		= 500;
	size_t	nBytesReadMax	= sizeof(ackPacket);
	int		nAvailToRead	= pdv_serial_wait( pPdv, nMsTimeout, nBytesReadMax );

	int     nRead = 0;
	if ( nAvailToRead > 0 )
	{
		int     nToRead = nAvailToRead;
		if( nToRead > static_cast<int>( nBytesReadMax ) )
		{
			printf( "%s: Clipping nAvailToRead %d to nBytesReadMax %zu\n",
					functionName, nAvailToRead, nBytesReadMax );
			nToRead = static_cast<int>(nBytesReadMax);
		}
		nRead = pdv_serial_read( pPdv, reinterpret_cast<char *>(&ackPacket), nToRead );
	}

	if ( nRead <= 0 )
	{
		fprintf( stderr, "%s Error: Timeout with no reply!\n", functionName );
		return GENCP_STATUS_MSG_TIMEOUT | GENCP_SC_ERROR;
	}

	if ( numBytes == 2 )
	{
		uint16_t	result16 = 0xFF;

		status = GenCpProcessReadMemAck( &ackPacket, &result16 );
		if ( status != GENCP_STATUS_SUCCESS )
		{
			fprintf( stderr, "GenCP Validate Error: %d\n", status );
			return status;
		}
		result = static_cast<uint64_t>( result16 );
	}
	else if ( numBytes == 4 )
	{
		uint32_t	result32 = static_cast<unsigned int>( -1 );

		status = GenCpProcessReadMemAck( &ackPacket, &result32 );
		if ( status != GENCP_STATUS_SUCCESS )
		{
			fprintf( stderr, "GenCP Validate Error: %d\n", status );
			return status;
		}
		result = static_cast<uint64_t>( result32 );
	}
	else if ( numBytes == 8 )
	{
		uint64_t	result64 = static_cast<unsigned int>( -1 );

		status = GenCpProcessReadMemAck( &ackPacket, &result64 );
		if ( status != GENCP_STATUS_SUCCESS )
		{
			fprintf( stderr, "GenCP Validate Error: %d\n", status );
			return status;
		}
		result = static_cast<uint64_t>( result64 );
	}

	if ( pnResult != NULL )
		*pnResult = result;
	return GENCP_STATUS_SUCCESS;
}


GENCP_STATUS EdtGenCpReadUint(
	unsigned int		iUnit,
	unsigned int		iChannel,
	uint64_t			regAddr,
	size_t				numBytes,
	uint64_t		*	pnResult	)
{
	GENCP_STATUS		status;
    EdtDev			*	pPdv;
	uint64_t			result = static_cast<unsigned int>( -1 );

	if ( pnResult != NULL )
		*pnResult = result;

    /* open a handle to the device     */
    pPdv = pdv_open_channel(EDT_INTERFACE, iUnit, iChannel);
    if ( pPdv == NULL )
    {
        pdv_perror( EDT_INTERFACE );
        return GENCP_STATUS_INVALID_PARAM | GENCP_SC_ERROR;
    }
	pdv_serial_read_enable( pPdv );

	status = PdvGenCpReadUint( pPdv, regAddr, numBytes, &result );

	pdv_close( pPdv );

	if ( status != GENCP_STATUS_SUCCESS )
	{
		fprintf( stderr, "Error reading %zu bytes from regAddr 0x%08lX\n", numBytes, regAddr );
	}
	else
	{
		printf( "regAddr 0x%08lX = %lu\n", regAddr, result );
	}

	if ( pnResult != NULL )
		*pnResult = result;
	return GENCP_STATUS_SUCCESS;
}


int main( int argc, char **argv )
{
	int				status;
    unsigned int	channel = 0;
    unsigned int	unit = 0;
    bool	     	verbose = FALSE;

    for ( int iArg = 1; iArg < argc; iArg++ )
    {
		if	(	strcmp( argv[iArg], "-c" ) == 0
			||	strcmp( argv[iArg], "--channel" ) == 0 )
		{
			if ( iArg >= argc )
			{
				usage( "Error: Missing channel number.\n" );
				exit( -1 );
			}
			channel = atoi( argv[++iArg] );
		}
		else if (	strcmp( argv[iArg], "-u" ) == 0
				||	strcmp( argv[iArg], "--unit" ) == 0 )
        {
			if ( iArg >= argc )
			{
				usage( "Error: Missing unit number.\n" );
				exit( -1 );
			}
			unit = atoi( argv[++iArg] );
		}
		else if (	strncmp( argv[iArg], "--U", 3 ) == 0 )
		{
			if ( iArg >= argc )
			{
				usage( "Error: Missing address.\n" );
				exit( -1 );
			}

       		//   --U32 Addr  Read 32 bit unsigned value from address\n"
			uint64_t		result64;
			unsigned int	numBits		= atoi( &argv[iArg][3] );
			unsigned int	numBytes	= numBits / 8;
			if ( numBytes == 0 || numBytes > 8 )
			{
				fprintf( stderr, "Invalid number of bits for --U option: %s\n", argv[iArg] );
				exit( 1 );
			}
			iArg++;
			uint64_t		regAddr		= strtoull( argv[iArg], NULL, 0 );
			//if ( sscanf( argv[iArg], "%LX", &regAddr ) != 1 )
			//{
			//	fprintf( stderr, "Invalid reg addr for --U option: %s\n", argv[iArg] );
			//	exit( 1 );
			//}
			status = EdtGenCpReadUint( unit, channel, regAddr, numBytes, &result64 );
		}
		else if (	strcmp( argv[iArg], "-v" ) == 0
				||	strcmp( argv[iArg], "--verbose" ) == 0 )
        {
			verbose = 1;
		}
		else if (	strcmp( argv[iArg], "-h" ) == 0
				||	strcmp( argv[iArg], "--help" ) == 0 )
		{
			usage( "" );
			exit( 0 );
		}
		else
		{
			fprintf( stderr, "unknown option: %s\n", argv[0] );
			usage( "" );
			exit( 1 );
		}
    }

    return (0);
}
