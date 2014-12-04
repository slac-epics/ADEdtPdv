#ifndef SYNC_OBJECT_H
#define SYNC_OBJECT_H
#include <string>
#include <math.h>
#include "NDArray.h"

#define	ACQ_TRACE( level, pFormat... ) if ( level > GetTraceLevel() ) tracePrint( pFormat )

/*	Macro declarations */
#define ACQUIRE_THREAD_PRIORITY	(epicsThreadPriorityMedium)
#define ACQUIRE_THREAD_STACK		(0x20000)

// TODO: Move this somewhere else
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

///	template class syncDataAcq< devClass, dataClass >
/// Provides a thread based acquisition and timestamping loop
/// with common logic for thread safety, reconfiguration,
/// timestamping, and shutdown.
/// Policy based configuration for whether or not to reject data objects
///	Example instantiations:
///		syncDataAcq	edtSyncDataAcq< edtImage, edtCamera >
///		syncDataAcq	rawSyncDataAcq< rawData, rawDataSource >
template < class Dev, class Data >	class syncDataAcq
{
 public:
	enum SyncBehaviour	{ SKIP_OBJECT	= 0, USE_OBJECT		= 1 };
	enum SyncStatus		{ SYNC_OBJ_OK	= 0, SYNC_OBJ_ERROR	= 1 };
	enum AttributeMask	{ CanSkip		= 1, HasCount	= 2,	HasTime	= 4 };

	syncDataAcq( Dev	&	rDevice, const std::string	&	strName	)
	:	m_Device(				rDevice		),
		m_Name(					strName		),
		m_fExitThread(			false		),
		m_TraceLevel(			1			),
		m_fEnabled(				false		),
		m_fAcquiring(			false		),
		m_threadId(				NULL		),
		m_acquireEvent(			NULL		),
		m_acquireTimeout(		1.0			),
		m_reconfigDelay(		1.0			),

		m_fSynced(				false		),
		m_EventNumber(			0			),
		m_PolicyBadTimeStamp(	USE_OBJECT	),
		m_PolicyUnsynced(		USE_OBJECT	)
	{
		static const char	*	functionName = "syncDataAcq::syncDataAcq";
		printf( "%s: Creating acquire thread from thread %s\n", functionName, epicsThreadGetNameSelf() );
		// Create acquisition thread
		m_threadId		= epicsThreadMustCreate( m_Name.c_str(), ACQUIRE_THREAD_PRIORITY, ACQUIRE_THREAD_STACK,
												(EPICSTHREADFUNC)syncDataAcq::ThreadStart, (void *) this );

		printf( "%s: Creating acquire event\n", functionName );
		// Create an event for signaling acquisition thread
		m_acquireEvent  = epicsEventMustCreate(	epicsEventEmpty );

	}

	virtual ~syncDataAcq()
	{
		Shutdown();
	}

	bool	IsAcquiring() const
	{
		return m_fAcquiring;
	}

	bool	IsEnabled() const
	{
		return m_fEnabled;
	}

	void SetEnabled( bool fEnabled = true )
	{
		m_fEnabled = fEnabled;
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



#if 0
	//	Returns status: 0 = OK, -1 on error
	//	If status is OK and a pulse number is found,
	//	the pulse number number is returned via pPulseNumRet
	template < class Dev, class Data > int syncDataAcq< Dev, Data >::TimeStampImage(
		edtImage		*	pDataObject,
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
		int	status	= epicsTimeGetEvent( pDest, m_EventNumber );
		if ( pPulseNumRet )
			*pPulseNumRet  = GetPulseID( pDataObject, pDest );

		if ( status != 0 )
		{
			SetSynced( false );
			return SYNC_OBJ_ERROR;
		}
		return SYNC_OBJ_OK;
	}
#endif

	void	SignalAcquireEvent( )
	{
		static const char	*	functionName = "syncDataAcq::SignalAcquireEvent";
		ACQ_TRACE(	1,	"%s: Signal Acquire Event in thread %s\n", functionName, epicsThreadGetNameSelf() );
		printf( "%s: Signal Acquire Event in thread %s\n", functionName, epicsThreadGetNameSelf() );
		epicsEventSignal( m_acquireEvent );
	}

	//	syncDataAcq::acquireSyncData()
	//	Loops:
	//		Acquires a new data object
	//		Checks for synchronization
	//		Timestamps and queues data object if synced
	//	Never returns
	void	acquireSyncData( )
	{
		static const char	*	functionName = "syncDataAcq::acquireSyncData";

		// Also create a data object for EDT image data
		//	edtSyncData		edtImageObject;
		//	edtSyncData	*	pImage	= &edtImageObject;
		Data		edtImageObject;
		Data	*	pImage	= &edtImageObject;

		printf( "%s: Entering forever loop in thread %s\n", functionName, epicsThreadGetNameSelf() );
		ACQ_TRACE(	1,	"%s: Entering forever loop in thread %s\n", functionName, epicsThreadGetNameSelf() );

		//	Forever loop until app exits
		while ( m_fExitThread == false )
		{

		try
			{
				// Just spin till acquisition is enabled
				if ( !m_fEnabled )
				{
					ACQ_TRACE(	2,	"%s: Signal Acquisition disabled\n", functionName, epicsThreadGetNameSelf() );
					epicsThreadSleep( m_reconfigDelay * 5 );
					continue;
				}

				// See if the camera needs to be configured
				int	status	= 0;
				if ( m_Device.NeedsReconfigure() )
				{
					//	Wait a bit before configuring in case multiple
					//	params are being changed.
					//	Also keeps from chewing up cpu time on failed
					//	reconfigure attempts.
					epicsThreadSleep( m_reconfigDelay );
					m_Device.Reconfigure();
					continue;
				}

				// Update the sync object w/ the latest pdvDev ptr
				//	pSyncObj->SetPdvDev( m_pPdvDev );

				// Wait till we have something to do
				// Use a timeout so we check periodically to see if we need to reconfigure
				status = epicsEventWaitWithTimeout( m_acquireEvent, m_acquireTimeout );
				if ( status == epicsEventWaitTimeout )
				{
					ACQ_TRACE(	5,	"%s: Timeout on m_acquireEvent in thread %s\n", functionName, epicsThreadGetNameSelf() );
					continue;
				}
				printf( "%s: Acquire Loop acquiring %d images\n", functionName, m_Device.GetAcquireCount() );

				if ( m_Device.NeedsReconfigure() )
				{
					ACQ_TRACE(	1,	"%s: Reconfig pending in thread %s\n", functionName, epicsThreadGetNameSelf() );
					continue;
				}

				if ( m_Device.GetAcquireCount() != 0 )
				{
					ACQ_TRACE(	1,	"%s: Image acquisition requested in thread %s\n", functionName, epicsThreadGetNameSelf() );

					//	Start camera
					status = m_Device.CameraStart();
					if ( status != 0 )
						continue;
				}

				if ( m_Device.InAcquireMode() )
					ACQ_TRACE(	5,	"%s: Entering acquire loop: Acquire Count %d\n", functionName,
									m_Device.GetAcquireCount() );
				while ( m_Device.InAcquireMode() )
				{
					m_fAcquiring	= true;

					//	Release the image data at the top so error handling can
					//	just bail w/ a continue.  NULL pImage is OK.
					//	pSyncObj->ReleaseData( pImage );
					ACQ_TRACE(	4,	"%s: Release old image\n", functionName );
					m_Device.ReleaseData( pImage );

					// See if we should stop acquiring
					if ( m_Device.NeedsReconfigure() )
					{
						ACQ_TRACE( 3, "%s: Halting acquire pending reconfiguration ...\n", functionName );
						break;
					}
					if ( m_Device.GetAcquireCount() == 0 )
					{
						ACQ_TRACE( 3, "%s: Halting acquire as count is complete.\n", functionName );
						break;
					}

					// TODO: Use a genSub to configure ROI, eventNum, sync params, timeouts, etc
					// TODO: Make it SCAN "I/O Intr" and proccess it here to check params and
					//	update status once per loop
					//	Process( pGenSubRec );

					ACQ_TRACE( 4, "%s: Acquiring new image ...\n", functionName );
					// Wait for a new image
					// int	status = pSyncObj->AcquireData( pImage, m_acquireTimeout );
					// if ( status != syncObject::SYNC_OBJ_OK )
					// 	continue;
					status = m_Device.AcquireData( pImage );
					if ( status != 0 )
					{
						ACQ_TRACE( 3, "%s: AcquireData error %d\n", functionName, status );
						continue;
					}

					// Check for image errors
					//	status	=	pSyncObj->CheckData( pImage );
					//	if ( status != syncObject::SYNC_OBJ_OK )
						//	continue;
					ACQ_TRACE( 4, "%s: Checking data ...\n", functionName );
					status	=	pImage->CheckData( );
					if ( status != 0 )
						continue;

					// Check for sync
					ACQ_TRACE( 4, "%s: Checking sync ...\n", functionName );
#if 0
					setIntegerParam( ADStatus, ADStatusCorrect );
					callParamCallbacks( 0, 0 );
#else
					bool	isSynced = m_Device.IsSynced( pImage );
#endif
					if ( isSynced == false )
					{
						if ( GetPolicyUnsynced() == SKIP_OBJECT )
							continue;
					}

					//	Get image timestamp
					ACQ_TRACE( 4, "%s: Getting timestamp ...\n", functionName );
					epicsTimeStamp	tsEvent;
					int				pulseID;
					status = m_Device.TimeStampImage( pImage, &tsEvent, &pulseID );
					if ( status != 0 )
					{
						if ( GetPolicyBadTimeStamp() == SKIP_OBJECT )
							continue;
					}

					ACQ_TRACE( 4, "%s: ProcessData ...\n", functionName );
					// Process the image data
					m_Device.ProcessData( pImage, &tsEvent, pulseID );
				}
				m_fAcquiring = false;
				// Do we need to do this? m_Device.SetAcquireMode( false );
			}
		catch ( std::exception & e )
			{
			// What to do?
			printf( "Acquire loop handling exception: %s\n", e.what() );
			continue;
			}
		}	// End of acquire loop

		printf( "%s: Exiting forever loop in thread %s\n", functionName, epicsThreadGetNameSelf() );
		// We only return if the app is exiting
		return;
	}

	///	Tell acquisition loop to shutdown and exit thread
	int		Shutdown( )
	{
		m_fExitThread = true;
		return 0;
	}

	int		TimeStampImage(	edtImage		*	pDataObject,
							epicsTimeStamp	*	pDest,
							int				*	pPulseNumRet );

	int		RequestResync(	void );

	// Trace level: 0=none, 1=sparse, 2=more, 3=lots, ...
	void	SetTraceLevel(	unsigned int level )
	{
		m_TraceLevel	= level;
	}
	unsigned int GetTraceLevel( )
	{
		return m_Device.GetTraceLevel();
	}

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

	int	tracePrint( const char	*	pFormat, ... )
	{
		va_list		pvar;
		int			nout	= 0;

		va_start( pvar, pFormat );
		m_Device.traceVPrint( pFormat, pvar );
		va_end( pvar );

		return nout;
	}


private:	//	Private class functions

///	Thread routine for epics camera
/// For each epics camera, a new thread is spawned, with this
/// function as the first function called by the thread.
/// Typically, this will call a camera function that loops
/// handling successive images, returning only on error.
	static	int		ThreadStart( syncDataAcq	*	pSyncDataAcquirer )
	{
		static const char	*	functionName = "syncDataAcq::ThreadStart";
		if ( pSyncDataAcquirer  == NULL )
		{
			errlogPrintf(	"%s: ERROR, SyncDataAcq polling thread failed! NULL ptr to sync object!\n",
							functionName );
			return -1;
		}
		pSyncDataAcquirer->acquireSyncData( );
		return 0;
	}

private:	//	Private member functions

 	// No default copy constructor
	syncDataAcq< Dev, Data >( const syncDataAcq< Dev, Data > & );
 	// No assignment operator
	syncDataAcq< Dev, Data > & operator = ( const syncDataAcq< Dev, Data > & );

 // Private member variables
 private:
 	Dev			&	m_Device;			// Pointer to data acquisition device
 	std::string		m_Name;				// Name is arbitrary, used to identify thread
	bool			m_fExitThread;		// Set true to ask the acquisition thread to exit
	unsigned int	m_TraceLevel;		// Trace level: 0=none, 1=sparse, 2=more, 3=lots, ...
	bool			m_fEnabled;			// True if acquisition is enabled.  Does nothing when not enabled
	bool			m_fAcquiring;		// True while acquiring images, false when idle
	epicsThreadId	m_threadId;			// Thread identifier for polling thread
	epicsEventId	m_acquireEvent;		// Used to signal when image acquisition is needed

	double			m_acquireTimeout;	// Timeout in seconds for image acquisition (will retry after cking for reconfig)
	double			m_reconfigDelay;	// Delay in seconds after failed reconfiguration before retry

	//	Old member variables from timesync.h
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
