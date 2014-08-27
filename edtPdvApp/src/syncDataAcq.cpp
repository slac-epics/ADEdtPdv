#include <iocsh.h>
#include <callback.h>
#include <dbScan.h>
#include <dbAccess.h>
#include <cantProceed.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <registryFunction.h>
#include <errlog.h>
#include <epicsVersion.h>
#include <unistd.h>
#include "syncDataAcq.h"
#include "evrTime.h"

using namespace		std;

int sync_debug = 2;
int sync_cnt   = 200;
#define SYNC_DEBUG(n)        (sync_debug > (n) && sync_cnt > 0 && --sync_cnt)
#define SYNC_DEBUG_ALWAYS(n) (sync_debug > (n))
#define SYNC_ERROR(level, msg)     \
        if (SYNC_DEBUG(level)) {   \
            printf msg;            \
            DebugPrint(m_pDataObject);\
            fflush(stdout);        \
        }                          \
        SetSynced( false );

syncDataAcq::syncDataAcq( void )
	:	m_AcquireTimeout(		-1.0		),
		m_fSynced(				false		),
		m_EventNumber(			0			),
		m_PolicyBadTimeStamp(	USE_OBJECT	),
		m_PolicyUnsynced(		USE_OBJECT	)

{
}

//	syncDataAcq::Poll()
//	Loops:
//		Acquires a new data object
//		Checks for synchronization
//		Timestamps and queues data object if synced
//	Never returns
int syncDataAcq::Poll(void)
{
    SetSynced( false );

	// Acquire loop
	edtImage		syncDataObject;
	edtImage	*	pDataObject	= &syncDataObject;
    while(TRUE)
	{
		// Delete the current data object, if any
        ReleaseData( pDataObject );

		// Acquire a new data object, waits forever
        int status = AcquireData( pDataObject, m_AcquireTimeout );
		if ( status != syncDataAcq::SYNC_OBJ_OK )
			continue;

		// Check for image errors
		status  =   CheckData( pDataObject );
		if ( status != syncDataAcq::SYNC_OBJ_OK )
		{
			continue;
		}

		// Check for sync
		bool    isSynced = CheckSync( pDataObject );
		if ( isSynced == false )
		{
			if ( GetPolicyUnsynced() == syncDataAcq::SKIP_OBJECT )
				continue;
		}

		//  Get image timestamp
		epicsTimeStamp  tsEvent;
		int             pulseID;
		status = GetTimeStampAndPulse(	pDataObject,	GetEventNumber(),
										&tsEvent,		&pulseID );
		if ( status != 0 )
		{
			if ( GetPolicyBadTimeStamp() == syncDataAcq::SKIP_OBJECT )
				continue;
		}

        QueueData( pDataObject, &tsEvent, pulseID );
    }
    return 0;
}


//	Returns status: 0 = OK, -1 on error
//	If status is OK and a pulse number is found,
//	the pulse number number is returned via pPulseNumRet
int syncDataAcq::GetTimeStampAndPulse(
	edtImage		*	pDataObject,
	int					eventNumber,
	epicsTimeStamp	*	pDest,
	int				*	pPulseNumRet )
{
	if ( pDest == NULL )
	{
		SetSynced( false );
		return -1;
	}
	if ( pPulseNumRet != NULL )
		*pPulseNumRet  = -1;

	// Just get the latest timestamp for the specified event code
	int	status	= epicsTimeGetEvent( pDest, eventNumber );
	if ( pPulseNumRet )
		*pPulseNumRet  = GetPulseID( pDataObject, pDest );

	if ( status != 0 )
	{
		SetSynced( false );
		return SYNC_OBJ_ERROR;
	}
	return SYNC_OBJ_OK;
}

