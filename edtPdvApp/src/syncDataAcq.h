//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ADEdtPdv'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'ADEdtPdv', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
#ifndef SYNC_OBJECT_H
#define SYNC_OBJECT_H
#include <string>
#include <math.h>
#include "NDArray.h"

//	Synchronous data acquisition trace flags
//	Uses asynTrace mechanism, so they share the
//	same mask parameter, currently an int.
//	Asyn uses the first 6 bits
//  Starting from an arbitrary bit 9 position
//	Can be changed here if it conflicts w/ another module
#define	ACQ_TRACE_START_STOP	0x01	// Enables trace msgs related to stopping and starting acquisition
#define	ACQ_TRACE_SYNC_UNSYNC	0x02	// Enables trace msgs related to changes in sync status
#define	ACQ_TRACE_DETAIL		0x04	// Enables capture by capture trace details at up to 120hz

#define	ACQ_TRACE( level, pFormat... ) if ( (level) & GetTraceLevel() ) tracePrint( pFormat )

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

	void SetEnabled( bool fEnabled = true )
	{
		m_fEnabled = fEnabled;
	}

	bool	IsAcquiring() const
	{
		return m_fAcquiring;
	}

	virtual const char	*	Name(		void			)	{ return ""; };
	virtual int				CountIncr(	edtImage	*	)	{ return -1; };
	virtual int				FidDiff(	edtImage	*	)	{ return -1; };
	virtual int				Attributes( void)				{ return CanSkip; };

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


	void	SignalAcquireEvent( )
	{
		static const char	*	functionName = "syncDataAcq::SignalAcquireEvent";
		ACQ_TRACE(	ACQ_TRACE_START_STOP,	"%s: Signal Acquire Event in thread %s\n", functionName, epicsThreadGetNameSelf() );
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

		ACQ_TRACE(	ACQ_TRACE_START_STOP,	"%s: Entering forever loop in thread %s\n", functionName, epicsThreadGetNameSelf() );

		//	Forever loop until app exits
		while ( m_fExitThread == false )
		{

		try
			{
				// Just spin on the reconfig delay till acquisition is enabled
				if ( !m_fEnabled )
				{
					ACQ_TRACE(	ACQ_TRACE_DETAIL, "%s: Image Acquisition loop not enabled\n", functionName );
					epicsThreadSleep( m_reconfigDelay );
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

				// Wait till we have something to do
				// Use a timeout so we check periodically to see if we need to reconfigure
				status = epicsEventWaitWithTimeout( m_acquireEvent, m_acquireTimeout );
				if ( status == epicsEventWaitTimeout )
				{
					ACQ_TRACE(	ACQ_TRACE_DETAIL,	"%s: Timeout on m_acquireEvent in thread %s\n",
								functionName, epicsThreadGetNameSelf() );
				}

				if ( m_Device.NeedsReconfigure() )
				{
					ACQ_TRACE(	ACQ_TRACE_START_STOP,	"%s: Reconfig pending in thread %s\n",
								functionName, epicsThreadGetNameSelf() );
					continue;
				}

				if ( m_Device.InAcquireMode() && m_Device.GetAcquireCount() != 0 )
				{
					// Go acquire some images!
					status = acquireLoop( pImage );
					if ( status != 0 )
						continue;
				}
			}
		catch ( std::exception & e )
			{
			// What to do?
			printf( "Acquire loop handling exception: %s\n", e.what() );
			printf( "Continuing Acquire loop ...\n" );
			continue;
			}
		}	// End of acquire loop

		printf( "%s: Exiting forever loop in thread %s\n", functionName, epicsThreadGetNameSelf() );
		// We only return if the app is exiting
		return;
	}

	//	syncDataAcq::acquireLoop()
	//	Loops:
	//		Acquires a new data object
	//		Checks for synchronization
	//		Timestamps and queues data object if synced
	//	Returns when acquisition is complete or reconfig needed.
	int	acquireLoop(  Data * pImage )
	{
		static const char	*	functionName = "syncDataAcq::acquireLoop";

		ACQ_TRACE(	ACQ_TRACE_START_STOP,	"%s: Entering acquire loop: Acquire Count %d\n",
					functionName, m_Device.GetAcquireCount() );


        m_Device.ResetSyncCounters();

		//	Start acquisition
		int status = m_Device.StartAcquisition();
		if ( status != 0 )
			return -1;

		while ( m_Device.InAcquireMode() )
		{
			m_fAcquiring	= true;

			//	Release the image data at the top so error handling can
			//	just bail w/ a continue.  NULL pImage is OK.
			ACQ_TRACE(	ACQ_TRACE_DETAIL,	"%s: Release old image\n", functionName );
			m_Device.ReleaseData( pImage );

			// See if we should stop acquiring
			if ( m_Device.NeedsReconfigure() )
			{
				ACQ_TRACE( ACQ_TRACE_START_STOP, "%s: Halting acquire pending reconfiguration ...\n", functionName );
				return 0;
			}
			if ( m_Device.GetAcquireCount() == 0 )
			{
				ACQ_TRACE( ACQ_TRACE_START_STOP, "%s: Halting acquire as count is complete.\n", functionName );
				return 0;
			}

			ACQ_TRACE( ACQ_TRACE_DETAIL, "%s: Acquiring new image ...\n", functionName );
			// Wait for a new image
			status = m_Device.AcquireData( pImage );
			if ( status != 0 )
			{
				// Failed to acquire an image!
				ACQ_TRACE( ACQ_TRACE_DETAIL, "%s: AcquireData error %d\n", functionName, status );
				// Should we return here to avoid trying to acquire data too often?
				return status;
				// Looks like Opal is ok w/ continue here
				// but Pulnix needs a restart if the trigger goes away and comes back
				// continue;
			}

			// Check for image errors
			status	= m_Device.CheckData( pImage );
			if ( status != 0 )
			{
				ACQ_TRACE( ACQ_TRACE_DETAIL, "%s: Rejected invalid data ...\n", functionName );
				continue;
			}

            m_Device.IncrSyncTotalCount();

			//	Get image timestamp
			epicsTimeStamp	tsEvent;
			int				pulseID;
			status = m_Device.TimeStampImage( pImage, &tsEvent, &pulseID );
			if ( status != 0 )
			{
                m_Device.IncrSyncBadTSCount();
				if ( GetPolicyBadTimeStamp() == SKIP_OBJECT )
				{
					ACQ_TRACE( ACQ_TRACE_DETAIL, "%s: Bad TimeStamp, skipping object ...\n", functionName );
					continue;
				}
			}

			if ( GetPolicyUnsynced() == SKIP_OBJECT )
			{
				// Check for sync
				if ( m_Device.IsSynced( pImage, &tsEvent, pulseID ) == false )
				{
                    m_Device.IncrSyncBadSyncCount();
					ACQ_TRACE( ACQ_TRACE_DETAIL, "%s: Unsynced, skipping object ...\n", functionName );
					continue;
				}
			}

			ACQ_TRACE( ACQ_TRACE_DETAIL, "%s: ProcessData for pulse id %d ...\n", functionName, pulseID );            
			// Process the image data
			m_Device.ProcessData( pImage, &tsEvent, pulseID );
		}
		m_fAcquiring = false;
		return 0;
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

	int	tracePrint( const char	*	pFormat, ... )
	{
		va_list		pvar;
		int			nout	= 0;

		va_start( pvar, pFormat );
		// m_Device.traceVPrint( pFormat, pvar );
		vprintf( pFormat, pvar );
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
