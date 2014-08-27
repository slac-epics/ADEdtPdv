#ifndef SYNC_OBJECT_H
#define SYNC_OBJECT_H
#include <string>
#include <math.h>
#include "NDArray.h"

class edtImage
{
 public:
	edtImage( )
		:	m_pNDArray(	NULL	)
	{
	}
	virtual ~edtImage( )
	{
	}

	int				CheckData( )
	{
		if ( m_pNDArray == NULL )
			return -1;
		return 0;
	}
	NDArray		*	GetNDArrayPtr(	)
	{
		return m_pNDArray;
	}

	void ReleaseNDArray( )
	{
		if( m_pNDArray != NULL )
		{
			m_pNDArray->release();
			m_pNDArray	= NULL;
		}
	}

	void SetNDArrayPtr( NDArray * pNDArray )
	{
		assert( m_pNDArray == NULL );
		m_pNDArray	= pNDArray;
	}

private:
	NDArray		*	m_pNDArray;
};

// TODO: Morph into a template class
// Possible approach could be:
//	class syncData< dataClass, devClass >
//	{
//		syncData<DataClass,devClass>( devClass * pDev, int eventNumber = -1 );
//		SetUnsyncedPolicy( fRejectUnsynced );
//		SetBadTimestampPolicy( fRejectBadTimestamp );
//		SetEventNumber( eventNumber );
//		Shutdown( ) { m_fExitApp = true; }
//		AcquireData( )	// In high priority thread
//		{
//		  while( !m_fExitApp )
//		  {
//			if ( pDev->NeedReconfig() )
//			{  pDev->Reconfigure(); continue; }
//			pData = pDev->AcquireData( );
//			pData->CheckData(	);
//			pDev->CheckSync(	pData );
//			pDev->Timestamp(	pData );
//			QueueData(	pData );
//		  }
//		}
//		ProcessData( )	// In med-high priority thread
//		{
//		  while( !m_fExitApp )
//		  {
//			pConfigGenSub->Process();
//			pData = DequeData( );
//			pDev->ProcessData(	m_pData );
//			pDev->ReleaseData(	m_pData );
//		  }
//		}
//	  private:
//		dataClass	*	m_pData;
//		devClass	*	m_pDev;
//	}
//	with instantiations like
//	syncData	edtSyncData< edtImage, edtCamera >
//	or
//	syncData	rawSyncData< rawData, rawDataSource >
class syncDataAcq
{
 public:
	enum SyncBehaviour	{ SKIP_OBJECT	= 0, USE_OBJECT		= 1 };
	enum SyncStatus		{ SYNC_OBJ_OK	= 0, SYNC_OBJ_ERROR	= 1 };
	enum AttributeMask	{ CanSkip		= 1, HasCount	= 2,	HasTime	= 4 };
	syncDataAcq( );
	virtual ~syncDataAcq()
	{
	}
	// Acquire timeout in sec, 0 = don't wait, -1 = forever
	virtual int		AcquireData(	edtImage		*	pDataObject,
									double				acquireTimeout )
	{
		if ( pDataObject == NULL || isnan(acquireTimeout) )
		{
			SetSynced( false );
			return SYNC_OBJ_ERROR;
		}
		return SYNC_OBJ_OK;
	};
	virtual int		QueueData(		edtImage		*	pDataObject,
									epicsTimeStamp	*	pTimeStamp,
									int					pulseID )
	{
		if ( pDataObject == NULL )
			return SYNC_OBJ_ERROR;
		return SYNC_OBJ_OK;
	};
	virtual void	ReleaseData(	edtImage		*	)
	{
	};
	virtual int				CheckData(	edtImage	*	pDataObject )
	{
		if ( pDataObject == NULL )
		{
			SetSynced( false );
			return SYNC_OBJ_ERROR;
		}
		return SYNC_OBJ_OK;
	};

	// See if we're synced
	virtual bool				CheckSync(	edtImage	*	pDataObject )
	{
		// Stubbed implementation has no specific sync criteria
		SetSynced( true );
		return m_fSynced;
	};

	bool IsSynced( edtImage * pDataObject )
	{
		return m_fSynced;
	};
	
	virtual void SetSynced( bool fSynced )	// TODO: Make this private
	{
		m_fSynced = fSynced;
	};

	virtual const char	*	Name(		void			)	{ return ""; };
	virtual int				CountIncr(	edtImage	*	)	{ return -1; };
	virtual int				FidDiff(	edtImage	*	)	{ return -1; };
	virtual int				Attributes( void)				{ return CanSkip; };

	virtual void DebugPrint(	edtImage	*	)
	{
	};

	int		GetEventNumber( ) const
	{
		return m_EventNumber;
	}

	int		SetEventNumber( int	eventNumber );

	enum SyncBehaviour GetPolicyBadTimeStamp( ) const
	{
		return m_PolicyBadTimeStamp;
	}
	int SetPolicyBadTimeStamp( enum SyncBehaviour value )
	{
		if ( value < 0 || value > SKIP_OBJECT )
			return SYNC_OBJ_ERROR;
		m_PolicyBadTimeStamp	= value;
		return SYNC_OBJ_OK;
	}

	enum SyncBehaviour GetPolicyUnsynced( ) const
	{
		return m_PolicyUnsynced;
	}
	int SetPolicyUnsynced( enum SyncBehaviour value )
	{
		if ( value < 0 || value > SKIP_OBJECT )
			return SYNC_OBJ_ERROR;
		m_PolicyUnsynced	= value;
		return SYNC_OBJ_OK;
	}

	int		Poll(void);		// Default routine to do synchronization (never returns!)

	int		TimeStampImage(	edtImage		*	pDataObject,
							int					eventNumber,
							epicsTimeStamp	*	pDest,
							int				*	pPulseNumRet );

	int		RequestResync(	void );

	virtual	int	GetPulseID(
		edtImage				*,
		const epicsTimeStamp	*	pTimeStamp )
	{
		int		pulseID	= -1;
		if ( pTimeStamp != NULL )
		{
			// Default pulse ID is assumed to be the
			// least 17 bits in the epicsTimeStamp nsec field
			pulseID	= pTimeStamp->nsec & 0x1FFFF; 
		}
		return pulseID;
	}

 private:
	double				m_AcquireTimeout;		// Acquire timeout in sec
	bool				m_fSynced;				// true when synced
	int					m_EventNumber;			// Trigger event number
	enum SyncBehaviour 	m_PolicyBadTimeStamp;	// Policy on invalid timestamp
	enum SyncBehaviour	m_PolicyUnsynced;		// Policy when unsynced
//	edtImage		*	m_pDataObject;			// Ptr to current data object
//	double			*	m_pDelayVal;			// Ptr to Expected delay between trigger and reception.
//	epicsUInt32		 *	m_pEventCode;			// Ptr to Event of interest.
//	epicsUInt32		 *	m_pGenCount;			// Ptr to Generation of event, if triggered.
//	std::string			m_syncpv;				// Name of the synchronization status PV
//	int					m_have_syncpv;
//	DBADDR				m_addr;
//	int					m_lastdatafid;
//	int					m_lasttsfid;
//	int					m_lastdelayfid;
//	double				m_lastdelay;
//	unsigned int		m_gen;
//	int					m_eventvalid;
//	int					m_tsfid;
//	unsigned long long	m_idx;
};

#endif // SYNC_OBJECT_H
