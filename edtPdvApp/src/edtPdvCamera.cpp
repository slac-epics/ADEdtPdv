//////////////////////////////////////////////////////////////////////////////
// This file is part of 'ADEdtPdv'.
// It is subject to the license terms in the LICENSE.txt file found in the 
// top-level directory of this distribution and at: 
//    https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. 
// No part of 'ADEdtPdv', including this file, 
// may be copied, modified, propagated, or distributed except according to 
// the terms contained in the LICENSE.txt file.
//////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
// Filename: edtPdvCamera.cpp
// Description: EPICS device support for cameras using EDT framegrabbers
//              via EDT's PDV software library
// Author:
//		Bruce Hill, SLAC National Accelerator Lab, July 2014
/////////////////////////////////////////////////////////////////////////////

//	Standard headers
#include <iostream>

//	EPICS headers
#include <iocsh.h>
#include <callback.h>
#include <dbScan.h>
#include <dbAccess.h>
#include <cantProceed.h>
#include <epicsEvent.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <epicsVersion.h>
#include <errlog.h>
#include <registryFunction.h>
#include <unistd.h>

// AreaDetector headers
#include "ADDriver.h"

// ADEdtPdv headers
#include "edtPdvCamera.h"
//#include "edtSync.h"
#include "syncDataAcq.h"
#include "asynEdtPdvSerial.h"
#include "promheader.h"

//	PCDS headers
//	#include "evrTime.h"

#ifdef	USE_DIAG_TIMER
#include "HiResTime.h"
#include "ContextTimer.h"
#else
#define CONTEXT_TIMER(a)
#endif	//	USE_DIAG_TIMER

using namespace		std;

static	const char *		driverName	= "EdtPdv";

// Diagnostic timers
// View and reset via iocsh cmds.
// From iocsh, type: help *Context*

int		DEBUG_EDT_PDV	= 2;

// int		fEnableFrameSync	= PDV_FRAMESYNC_OFF;
int		fEnableFrameSync	= PDV_FRAMESYNC_ON;
int		fCheckFrameSync		= 0;

//	t_HiResTime		imageCaptureCumTicks	= 0LL;
//	unsigned long	imageCaptureCount		= 0L;

///	Camera map - Stores ptr to all edtPdvCamera instances indexed by name
map<string, edtPdvCamera *>	edtPdvCamera::ms_cameraMap;

const char * EdtModeToString( edtPdvCamera::EdtMode_t	edtMode )
{
	const char	*	pstrEdtMode;
	switch( edtMode )
	{
	default:							pstrEdtMode	= "Invalid!";	break;
	case edtPdvCamera::EDTMODE_BASE:	pstrEdtMode	= "Base";		break;
	case edtPdvCamera::EDTMODE_MEDIUM:	pstrEdtMode	= "Medium";		break;
	case edtPdvCamera::EDTMODE_FULL:	pstrEdtMode	= "Full";		break;
	}
	return pstrEdtMode;
}

const char * TrigLevelToString( int	trigLevel )
{
	const char	*	pstrTrigLevel;
	switch( trigLevel )
	{
	default:	pstrTrigLevel	= "Invalid!";	break;
	case 0:		pstrTrigLevel	= "Edge";		break;
	case 1:		pstrTrigLevel	= "Level";		break;
	case 2:		pstrTrigLevel	= "Sync";		break;
	}
	return pstrTrigLevel;
}

const char * TriggerModeToString( edtPdvCamera::TriggerMode_t	tyTriggerMode )
{
	const char	*	pstrTriggerMode;
	switch( tyTriggerMode )
	{
	default:								pstrTriggerMode	= "Invalid!";	break;
	case edtPdvCamera::TRIGMODE_FREERUN:	pstrTriggerMode	= "FreeRun";	break;
	case edtPdvCamera::TRIGMODE_EXT:		pstrTriggerMode	= "External";	break;
	case edtPdvCamera::TRIGMODE_PULSE:		pstrTriggerMode	= "Pulse";		break;
	}
	return pstrTriggerMode;
}

//
// edtPdvCamera functions
//

///	edtPdvCamera constructor
edtPdvCamera::edtPdvCamera(
	const char			*	cameraName,
	int						unit,
	int						channel,
	const char			*	modelName,
	const char			*	edtMode,
	int						maxBuffers,				// 0 = unlimited
	size_t					maxMemory,				// 0 = unlimited
	int						priority,				// 0 = default 50, high is 90
	int						stackSize			)	// 0 = default 1 MB
	:	ADDriver(			cameraName,	
							N_PDV_DRV_ADDR_MAX,
							NUM_EDT_PARAMS,
							maxBuffers, maxMemory,
							asynOctetMask,	0,	// Supports an asynOctect interface w/ no interrupts
							ASYN_CANBLOCK,	1,	// ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1
							priority, stackSize	),
		m_fAcquireMode(		false			    ),
		m_fExitApp(			false			    ),
		m_fReconfig(		true			    ),
		m_fReopen(			true			    ),
		m_NumMultiBuf(		N_PDV_MULTIBUF_DEF	),
		m_pPdvDev(			NULL				),
		m_unit(				unit				),
		m_channel(			channel				),
		m_CameraClass(							),
		m_CameraInfo(							),
		m_CameraModel(		modelName			),
		m_CameraName(		cameraName	        ),
		m_ConfigFile(							),
		m_DrvVersion(							),
		m_LibVersion(							),
		m_ModelName(		modelName			),
		m_SerialPort(							),
		m_ClCurWidth(		0					),
		m_ClCurHeight(		0					),
		m_ClMaxWidth(		0					),
		m_ClMaxHeight(		0					),
		m_ClNumBits(		0					),
		m_ClHTaps(			0					),
		m_ClVTaps(			0					),
		m_tyInterlace(	PDV_INTLV_IN_PDV_LIB	),
		m_EdtMode(			EDTMODE_BASE		),
		m_TriggerMode(		TRIGMODE_FREERUN	),
		m_TriggerModeReq(	TRIGMODE_FREERUN	),
		m_BinX(				1					),
		m_BinY(				1					),
		m_MinX(				0					),
		m_MinY(				0					),
		m_SizeX(			0					),
		m_SizeY(			0					),
		// Do we need ADBase ROI values here?  m_BinX, m_BinY, m_MinX, m_SizeY, ...
		m_Gain(				0					),
		m_HwHRoi(			0					),
		m_HwVRoi(			0					),
		
		m_ArrayCounter(		0					),
		m_acquireCount(		0					),
		m_fiducial(			0					),

		m_ReCfgCnt(			0					),
		m_reconfigLock(		NULL				),
		
		m_trigLevel(		0					),
		m_EdtDebugLevel(	1					),
	//	m_EdtDebugMsgLevel(	0xFFF				),
		m_EdtDebugMsgLevel(	0x000				),

#ifdef	USE_DIAG_TIMER
		m_ReAcquireTimer(	"ReAcquire"			),
		m_ReArmTimer(		"ReArm"				),
		m_ProcessImageTimer("ProcessImage"		),
#endif	//	USE_DIAG_TIMER
		m_ioscan(			NULL				),
		m_pAsynSerial(		NULL				),
        m_SerialDisable (   1                   )
{
	static const char	*	functionName = "edtPdvCamera:edtPdvCamera";

	// Create mutexes
    m_reconfigLock	= epicsMutexMustCreate();

    // Initialize I/O Intr processing
    scanIoInit( &m_ioscan );
    if ( m_ioscan == NULL )
        asynPrint( this->pasynUserSelf, ASYN_TRACE_FLOW,
			"asynPrint "
            "%s %s: ERROR, scanIoInit failed!\n",
            driverName, functionName );

	// Serial port name set based on camera name
	m_SerialPort	=	m_CameraName;
	m_SerialPort	+=	".SER";

	if ( strcmp( edtMode, "Medium" ) == 0 )
		m_EdtMode	= EDTMODE_MEDIUM;
	else if ( strcmp( edtMode, "Full" ) == 0 )
		m_EdtMode	= EDTMODE_FULL;

    // Configure an asyn port for serial commands
	unsigned int		serPriority		= 0;
	int					autoConnect		= 0;
    m_pAsynSerial = new asynEdtPdvSerial(	m_SerialPort.c_str(), serPriority,	autoConnect	);

	// Create EDT parameters shared by all EDT based cameras
	// This group gives access to PDV library values of interest
	createParam( EdtClassString,		asynParamOctet,		&EdtClass		);
	createParam( EdtDebugString,		asynParamInt32,		&EdtDebug		);
	createParam( EdtDebugMsgString,		asynParamInt32,		&EdtDebugMsg	);
	createParam( EdtDrvVersionString,	asynParamOctet,		&EdtDrvVersion	);
	createParam( EdtHSkipString,		asynParamInt32,		&EdtHSkip		);
	createParam( EdtHSizeString,		asynParamInt32,		&EdtHSize		);
	createParam( EdtHTapsString,		asynParamInt32,		&EdtHTaps		);
	createParam( EdtHwHRoiString,		asynParamInt32,		&EdtHwHRoi		);
	createParam( EdtHwVRoiString,		asynParamInt32,		&EdtHwVRoi		);
	createParam( EdtModeString,			asynParamInt32,		&EdtMode		);
	createParam( EdtOverrunString,		asynParamInt32,		&EdtOverrun		);
	createParam( EdtVSkipString,		asynParamInt32,		&EdtVSkip		);
	createParam( EdtVSizeString,		asynParamInt32,		&EdtVSize		);
	createParam( EdtVTapsString,		asynParamInt32,		&EdtVTaps		);
	createParam( EdtLibVersionString,	asynParamOctet,		&EdtLibVersion	);
	createParam( EdtMultiBufString,		asynParamInt32,		&EdtMultiBuf	);
	createParam( EdtInfoString,			asynParamOctet,		&EdtInfo		);
	createParam( EdtTrigLevelString,	asynParamInt32,		&EdtTrigLevel	);
	createParam( EdtReCfgCntString,		asynParamInt32,		&EdtReCfgCnt	);

	// This group provides a way to have serial readbacks get reflected in
	// their ADBase class equivalents, for example
	// SerAcquireTime	=>	ADAcquireTime 
	createParam( EdtSerAcquireTimeString,	asynParamFloat64,	&SerAcquireTime	);
	createParam( EdtSerBinXString,			asynParamInt32,		&SerBinX		);
	createParam( EdtSerBinYString,			asynParamInt32,		&SerBinY		);
	createParam( EdtSerGainString,			asynParamFloat64,	&SerGain		);
	createParam( EdtSerMinXString,			asynParamInt32,		&SerMinX		);
	createParam( EdtSerMinYString,			asynParamInt32,		&SerMinY		);
	createParam( EdtSerSizeXString,			asynParamInt32,		&SerSizeX		);
	createParam( EdtSerSizeYString,			asynParamInt32,		&SerSizeY		);
	createParam( EdtSerTriggerModeString,	asynParamInt32,		&SerTriggerMode	);

    createParam( EdtSerDisableString,             asynParamInt32,     &SerDisable     );

    createParam( EdtSyncTotalCntString,     asynParamInt32,     &SyncTotal  );
    createParam( EdtSyncBadTSCntString,     asynParamInt32,     &SyncBadTS  );
    createParam( EdtSyncBadSyncCntString,   asynParamInt32,     &SyncBadSync  );

	// Get the EDT mode from the mbbo PV
	int		paramValue	= static_cast<int>( m_EdtMode );
	setIntegerParam( EdtMode,		paramValue );

	// Get the EDT PDV debug levels and multibuf number (should be in autosave)
	getIntegerParam( EdtDebug,		&m_EdtDebugLevel	);
	getIntegerParam( EdtDebugMsg,	&m_EdtDebugMsgLevel	);
	getIntegerParam( EdtMultiBuf,	&m_NumMultiBuf  );

    if (m_NumMultiBuf < 1) {
        printf( "%s: Setting MultiBuf to default %d\n", functionName, N_PDV_MULTIBUF_DEF);
        m_NumMultiBuf = N_PDV_MULTIBUF_DEF;
    }

	// Create an EDT Sync object
	// TODO: Should we just make this a member object?
	printf( "%s: Creating syncDataAcq object in thread %s\n", functionName, epicsThreadGetNameSelf() );
	syncDataAcq<edtPdvCamera, edtImage>		*	pSyncDataAcquirer	= NULL;
	pSyncDataAcquirer	= new syncDataAcq<edtPdvCamera, edtImage>( *this, m_CameraName );

	// Make it available as a member variable
	m_pSyncDataAcquirer	= pSyncDataAcquirer;

	// Set default policies
	m_pSyncDataAcquirer->SetPolicyUnsynced(		syncDataAcq<edtPdvCamera, edtImage>::SKIP_OBJECT );
	m_pSyncDataAcquirer->SetPolicyBadTimeStamp(	syncDataAcq<edtPdvCamera, edtImage>::SKIP_OBJECT );

    // Install exit hook for clean shutdown
    epicsAtExit( (EPICSTHREADFUNC)edtPdvCamera::ExitHook, (void *) this );
}

///	edtPdvCamera Destructor
edtPdvCamera::~edtPdvCamera( )
{
	Shutdown();

	disconnect( this->pasynUserSelf );

	// Cleanup driver
	delete m_pSyncDataAcquirer;
	m_pSyncDataAcquirer	= NULL;

	epicsMutexDestroy(	m_reconfigLock );
}


int edtPdvCamera::CreateCamera(
	const char *	cameraName,
	int				unit,
	int				channel,
	const char *	modelName,
	const char *	edtMode		)
{
    static const char	*	functionName = "edtPdvCamera::CreateCamera";

    /* Parameters check */
    if (  cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf(	"%s %s: ERROR, NULL or zero length camera name. Check parameters to edtPdvConfig()!\n",
            			driverName, functionName );
        return  -1;
    }

    if ( CameraFindByName(cameraName) != NULL )
    {
        errlogPrintf(	"%s %s: ERROR, Camera name %s already in use!\n",
						driverName, functionName, cameraName );
        return -1;
    }

    if (  IsCameraChannelUsed( unit, channel ) )
    {
        errlogPrintf(	"%s %s: ERROR, Unit %d channel %d already in use!\n",
						driverName, functionName, unit, channel	);
        return -1;
    }

    if (  modelName == NULL || strlen(modelName) == 0 )
    {
        errlogPrintf(	"%s %s: ERROR, NULL or zero length camera configuration.\n",
						driverName, functionName );
        return  -1;
    }

    if ( DEBUG_EDT_PDV )
        cout << "Creating edtPdvCamera: " << string(cameraName) << endl;
    edtPdvCamera	* pCamera = new edtPdvCamera( cameraName, unit, channel, modelName, edtMode );
    assert( pCamera != NULL );

    int	status	= pCamera->ConnectCamera( );
	if ( status != 0 )
        errlogPrintf( "edtPdvConfig failed for camera %s!\n", cameraName );

	// TODO: This should be in the constructor and add call
	//	to CameraRemove in the destructor
	CameraAdd( pCamera );
    return 0;
}


int edtPdvCamera::ShowAllCameras( int level )
{
	if ( level <= 0 )
		return 0;

	map<string, edtPdvCamera *>::iterator	it;
	for ( it = ms_cameraMap.begin(); it != ms_cameraMap.end(); ++it )
	{
		edtPdvCamera		*	pCamera	= it->second;
		pCamera->CameraShow( level );
    }

    return 0;
}


bool edtPdvCamera::IsCameraChannelUsed( unsigned int unit,  unsigned int channel )
{
	map<string, edtPdvCamera *>::iterator	it;
	for ( it = ms_cameraMap.begin(); it != ms_cameraMap.end(); ++it )
	{
		edtPdvCamera		*	pCamera	= it->second;
        if ( unit == pCamera->m_unit && channel == pCamera->m_channel )
			return true;
    }

    return false;
}


edtPdvCamera	*	edtPdvCamera::CameraFindByName( const string & name )
{
	map<string, edtPdvCamera *>::iterator	it	= ms_cameraMap.find( name );
	if ( it == ms_cameraMap.end() )
		return NULL;
	return it->second;
}

void edtPdvCamera::CameraAdd(		edtPdvCamera * pCamera )
{
	assert( CameraFindByName( pCamera->m_CameraName ) == NULL );
	if ( DEBUG_EDT_PDV )
		cout << "CameraAdd: " << pCamera->m_CameraName << endl;
	ms_cameraMap[ pCamera->m_CameraName ]	= pCamera;
}

void edtPdvCamera::CameraRemove(	edtPdvCamera * pCamera )
{
	ms_cameraMap.erase( pCamera->m_CameraName );
}

int edtPdvCamera::CameraShow( int level )
{
    if ( level < 0 )
		return 0;

	cout << "\tCamera " << m_CameraName	<< " is installed on Unit " << m_unit << " Channel " << m_channel << endl;
	if ( level >= 1 )
	{
		cout	<< "\t\tType: "			<< m_CameraClass
				<< " "					<< m_CameraModel
				<< ", configuration: " 	<< m_ConfigFile << endl;
		cout	<< "\t\tMax Res: "		<< m_ClMaxWidth
				<<	" x "				<< m_ClMaxHeight
				<< ", "					<< m_ClNumBits
				<< "bits" << endl;
	}
	if ( level >= 2 && m_pPdvDev != NULL )
	{
		int     stat;
    	stat = edt_reg_read( m_pPdvDev, PDV_STAT );
		cout	<< "\t\tStatus: 0x%X"	<< stat
				<< endl;
	}
    return 0;
}

void edtPdvCamera::ExitHook(void * arg)
{
	edtPdvCamera	*	pCam = static_cast<edtPdvCamera *>( arg );
	if( pCam != NULL )
		pCam->Shutdown();
}

void edtPdvCamera::Shutdown( )
{
	epicsMutexLock(	m_reconfigLock );
	m_acquireCount = 0;
	setIntegerParam(ADAcquire, 0);
	if ( m_pPdvDev != NULL )
	{
		//	edt_abort_dma( m_pPdvDev );
		pdv_timeout_restart( m_pPdvDev, 0 );
		pdv_close( m_pPdvDev );
		m_pPdvDev	= NULL;
	}
	epicsMutexUnlock(	m_reconfigLock );
}

///	Connects driver to device
asynStatus edtPdvCamera::ConnectCamera( )
{
    static const char	*	functionName	= "edtPdvCamera::ConnectCamera";
    asynStatus				status			= asynSuccess;

	if ( DEBUG_EDT_PDV >= 1 )
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );

    // Disable the Serial Communication
    SetSerDisable(1);

	// Initialize (or re-initialize) EDT framegrabber connection
	Reopen( );

	if ( m_pPdvDev == NULL )
	{
		printf( "%s: %s failed to initialize PdvDev camera!\n", functionName, m_CameraName.c_str() );
        return asynError;
	}
	if ( m_pAsynSerial == NULL )
	{
		printf( "%s: %s failed to initialize PdvDev camera serial port!\n", functionName, m_CameraName.c_str() );
        return asynError;
	}

	epicsMutexLock(	m_reconfigLock );
	status = m_pAsynSerial->pdvDevConnected( m_pPdvDev );
	// m_pAsynUser = m_pAsynSerial->pdvDevConnect( m_pPdvDev );
	epicsMutexUnlock(	m_reconfigLock );

	if ( status != asynSuccess )
        return asynError;

    // Enable the Serial Communication
    SetSerDisable(0);
	UpdateStatus( ADStatusIdle );

    return status;
}


//	Disconnects driver from device
asynStatus edtPdvCamera::DisconnectCamera( )
{
    static const char	*	functionName	= "edtPdvCamera::DisconnectCamera";
    int						status			= asynSuccess;

	if ( DEBUG_EDT_PDV >= 1 )
		printf(	"%s %s: disconnecting camera %s\n", 
				driverName, functionName, m_CameraName.c_str() );
    asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW,
				"asynPrint "
          		"%s %s: disconnecting camera %s\n", 
				driverName, functionName, m_CameraName.c_str() );

	SetAcquireMode( false );
	// Wait for acquire loop to terminate
	// TODO: Make this safer
	while ( m_pSyncDataAcquirer != NULL && m_pSyncDataAcquirer->IsAcquiring() )
	{
		epicsThreadSleep(0.1);
	}

    // Disable the Serial Communication
    SetSerDisable(1);

    // Block reconfigured until serial device is disconnected
	epicsMutexLock(	m_reconfigLock );
	m_pAsynSerial->pdvDevDisconnected( NULL );
	// m_pAsynSerial->pdvDevDisConnect( );
	// freeAsynUser( m_pAsynUser );

    if ( m_pPdvDev )
	{
		// Halt any image acquires in progress
		pdv_timeout_restart( m_pPdvDev, 0 );

		// Note: pdv_close(), not edt_close()!
		pdv_close( m_pPdvDev );
		m_pPdvDev	= NULL;
	}
	epicsMutexUnlock(	m_reconfigLock );
 
    return static_cast<asynStatus>( status );
    //	return asynSuccess;
}

/// Overriding ADDriver::connect
///	Connects driver to device
asynStatus edtPdvCamera::connect( asynUser *	pasynUser )
{
    static const char	*	functionName	= "edtPdvCamera::connect";

	if ( DEBUG_EDT_PDV >= 1 )
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );

	// The guts are in ConnectCamera(), which doesn't need a pasynUser ptr
	asynStatus	status	= ConnectCamera();
    if ( status != asynSuccess )
	{
		int			connected	= 0;
		pasynManager->isConnected( pasynUser, &connected );
		if ( connected )
		{
			// Signal asynManager that we are disconnected
			UpdateStatus( ADStatusDisconnected );
			if ( pasynManager->exceptionDisconnect( pasynUser ) != asynSuccess )
			{
				asynPrint(	pasynUser, ASYN_TRACE_ERROR,
							"%s %s: error calling pasynManager->exceptionDisconnect, error=%s\n",
							driverName, functionName, pasynUser->errorMessage );
			}
		}
    	return status;
    }

	// Signal asynManager that we are connected
    status = pasynManager->exceptionConnect( pasynUser );
    if ( status != asynSuccess )
	{
        asynPrint(	pasynUser, ASYN_TRACE_ERROR,
					"%s %s: error calling pasynManager->exceptionConnect, error=%s\n",
					driverName, functionName, pasynUser->errorMessage );
        return asynError;
    }
	if ( DEBUG_EDT_PDV >= 1 )
		printf(	"%s %s: EDT Framegrabber %s 0 connected!\n", 
				driverName, functionName, m_CameraName.c_str() );
    asynPrint(	pasynUser, ASYN_TRACE_FLOW, 
				"asynPrint "
				"%s %s: EDT Framegrabber %s 0 connected!\n", 
				driverName, functionName, m_CameraName.c_str() );
    return asynSuccess;
}

/// Overriding ADDriver::disconnect
///	Disconnects driver from device
asynStatus edtPdvCamera::disconnect( asynUser *	pasynUser )
{
    static const char	*	functionName	= "edtPdvCamera::disconnect";

	// The guts are in DisconnectCamera(), which doesn't need a pasynUser ptr
	int	status	= DisconnectCamera();
    if ( status != asynSuccess )
	{
        asynPrint(	pasynUser, ASYN_TRACE_ERROR,
					"%s: error calling DisconnectCamera, error=%s\n",
					functionName, pasynUser->errorMessage );
    }

	// Set ADStatus to ADStatusDisconnected
	UpdateStatus( ADStatusDisconnected );

    asynPrint(	pasynUser, ASYN_TRACE_FLOW, 
				"asynPrint "
				"%s %s: EDT Framegrabber disconnected %s\n", 
				driverName, functionName, m_CameraName.c_str() );
	if ( DEBUG_EDT_PDV >= 1 )
		printf(	"%s %s: EDT Framegrabber %s 0 disconnected!\n", 
				driverName, functionName, m_CameraName.c_str() );

	// Process callbacks to update status
    callParamCallbacks( 0, 0 );

	// Signal asynManager that we are disconnected
    status = pasynManager->exceptionDisconnect( pasynUser );
    if ( status != asynSuccess )
	{
        asynPrint(	pasynUser, ASYN_TRACE_ERROR,
					"%s %s: error calling pasynManager->exceptionDisconnect, error=%s\n",
					driverName, functionName, pasynUser->errorMessage );
    }

    return asynSuccess;
}


int edtPdvCamera::Reconfigure( )
{
    static const char	*	functionName = "edtPdvCamera::Reconfigure";
	CONTEXT_TIMER( "Reconfigure" );

	if ( DEBUG_EDT_PDV >= 1 )
	{
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );
	}

	int		status	= 0;
	epicsMutexLock(		m_reconfigLock );
    
    // Disable the Serial Communication
    SetSerDisable(1); 
    m_pAsynSerial->pdvDevDisconnected( NULL );

	UpdateStatus( ADStatusInitializing );
	if ( m_fReopen )
	{
        // Clear reopen flag up front so it can be set again by another thread if needed
		m_fReopen	= false;
		status	= edtPdvCamera::_Reopen( );
	}

	// Clear reconfig flag up front so it can be set again by another thread if needed
	m_fReconfig = false;
	status	= edtPdvCamera::_Reconfigure( );
	if ( status != 0 )
	{
		// Reconfigure failed, request another
		m_fReconfig	= true;
	}

	if ( DEBUG_EDT_PDV >= 1 )
	{
		printf( "%s: %s done in thread %s\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );
	}
	epicsMutexUnlock(	m_reconfigLock );

	// TODO: Find a safe place to do this
	// Restart acquisition if acquire mode still on
	// if ( m_fAcquireMode )
	// 	SetAcquireMode( m_fAcquireMode );

	if ( status != 0 )
	{
        asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s %s: Reconfigure error! errorMsg=%s\n",
					driverName, functionName, this->pasynUserSelf->errorMessage );
	}
    else
    {
        UpdateStatus( ADStatusIdle );
		
        m_ReCfgCnt++;
		setIntegerParam( EdtReCfgCnt, m_ReCfgCnt );
        if ( m_fReconfig )
        {
            asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s %s: Reconfigure succeeded, but Reconfig flag has already been set again!\n",
                        driverName, functionName );
        }
    }

	return status;
}


int edtPdvCamera::Reopen( )
{
    static const char	*	functionName = "edtPdvCamera::ReReopen";
	CONTEXT_TIMER( "ReReopen" );

	if ( DEBUG_EDT_PDV >= 1 )
	{
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );
	}

	int		status	= 0;
	epicsMutexLock(		m_reconfigLock );

	UpdateStatus( ADStatusInitializing );

	// Clear reopen flag up front so it can be set again by another thread if needed
	m_fReopen	= false;
	status	= edtPdvCamera::_Reopen( );
	if ( status != 0 )
	{
		// Reopen failed, request another
		m_fReopen	= true;
	}
	m_fReconfig = true;
	epicsMutexUnlock(	m_reconfigLock );

	if ( status != 0 )
	{
        asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s %s: Reopen error! errorMsg=%s\n",
					driverName, functionName, this->pasynUserSelf->errorMessage );
	}
	else if ( m_fReopen )
	{
        asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s %s: Reopen succeeded, but Reopen flag has already been set again!\n",
					driverName, functionName );
	}
	return status;
}


//	Internal version of reconfigure
//	Don't call without holding m_reconfigLock!
int edtPdvCamera::_Reconfigure( )
{
    static const char	*	functionName = "edtPdvCamera::_Reconfigure";
	CONTEXT_TIMER( "_Reconfigure" );

    if ( m_pPdvDev == NULL || m_fReopen )
	{
		m_fReopen = true;
        return -1;
	}

	if ( DEBUG_EDT_PDV >= 1 )
	{
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );
	}

	// Read the config file
    Edtinfo			edtinfo;
	Dependent	*	pDD;			// Pointer to PDV dependent information.
    pDD = pdv_alloc_dependent();
    if ( pDD == NULL )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_ERROR, 
					"asynPrint "
					"%s %s: ERROR, Cannot allocate PDV Dependent structure!\n",
					driverName,		functionName );
        return -1;
    }

	//	Update m_ConfigFile based on TriggerMode, changing the current
	//	Internal/External choices to FreeRun (f.cfg), Trigger (t.cfg), or Pulse (p.cfg)
	m_ConfigFile = "db/";
	m_ConfigFile += m_ModelName;
	switch( m_TriggerModeReq )
	{
	default:
	case TRIGMODE_FREERUN:
		m_ConfigFile += "f.cfg";
		break;
	case TRIGMODE_EXT:
		m_ConfigFile += "t.cfg";
		break;
	case TRIGMODE_PULSE:
		m_ConfigFile += "p.cfg";
		break;
	}

	if ( DEBUG_EDT_PDV >= 1 )
	{
		printf( "%s: %s Reading config file %s ...\n", functionName, m_CameraName.c_str(), m_ConfigFile.c_str() );
	}

	// Read the config file
    if ( pdv_readcfg( m_ConfigFile.c_str(), pDD, &edtinfo ) )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_ERROR, 
					"asynPrint "
					"%s %s: ERROR, Invalid camera config file %s!\n",
					driverName,		functionName,	m_ConfigFile.c_str() );
        return -1;
    }

	if ( DEBUG_EDT_PDV >= 2 )
	{
		printf( "%s: Calling pdv_initcam ...\n", functionName );
	}
	// Initialize the camera
	// m_pPdvDev should already have it's own Dependent data in dd_p,
	// but because of how pdv_initcam() works, we must supply a different one
	// with the new configuration information.
	assert( m_pPdvDev->dd_p != NULL );
	assert( m_pPdvDev->dd_p != pDD  );
    if ( pdv_initcam( m_pPdvDev, pDD, m_unit, &edtinfo, m_ConfigFile.c_str(), NULL, 1 ) )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_ERROR,
					"asynPrint "
					"%s %s: ERROR, pdv_initcam failed!\n",
					driverName,		functionName	);
        return -1;
    }

    // Enable the Serial Communication
    m_pAsynSerial->pdvDevConnected( m_pPdvDev );
    SetSerDisable(0); 

	double		cameraInitCamDelay	= 0.0;
	if ( cameraInitCamDelay > 0.0 )
	{
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: Delaying %f sec for pdv_initcam\n", functionName, cameraInitCamDelay );
		epicsThreadSleep( cameraInitCamDelay );
	}

	// Fetch the camera manufacturer and model and write them to ADBase parameters
    m_CameraClass	= pdv_get_camera_class(	m_pPdvDev );
    m_CameraModel	= pdv_get_camera_model(	m_pPdvDev );
    m_CameraInfo	= pdv_get_camera_info(	m_pPdvDev );

	// Update EDT PDV asyn parameters
	setStringParam( ADModel,		m_CameraModel.c_str()	);
	setStringParam( ADManufacturer, m_CameraClass.c_str()	);
    setStringParam( EdtClass, 		m_CameraClass.c_str()	);
    setStringParam( EdtInfo,		m_CameraInfo.c_str()	);
	
	// Fetch the EDT driver and library versions and make sure they match
    char		buf[MAX_STRING_SIZE];
    edt_get_driver_version(	m_pPdvDev, buf, MAX_STRING_SIZE );
	m_DrvVersion = buf;
	size_t end_of_vers = m_DrvVersion.find( " " );
	if ( end_of_vers != string::npos )
	{
		// The driver version has a date on the end that we don't care about
		m_DrvVersion.erase( m_DrvVersion.find( " " ) );
	}
	setStringParam( EdtDrvVersion, m_DrvVersion.c_str()	);

    edt_get_library_version( m_pPdvDev, buf, MAX_STRING_SIZE );
	m_LibVersion = buf;
	setStringParam( EdtLibVersion, m_LibVersion.c_str()	);

	if ( m_DrvVersion.find(m_LibVersion) == string::npos )
	{
		printf( 
					"%s %s: ERROR, EDT driver version %s does not match lib version %s!\n",
					driverName, functionName, m_DrvVersion.c_str(), m_LibVersion.c_str() );
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_ERROR, 
					"asynPrint "
					"%s %s: ERROR, EDT driver version %s does not match lib version %s!\n",
					driverName, functionName, m_DrvVersion.c_str(), m_LibVersion.c_str() );
        return -1;
    }
	if ( DEBUG_EDT_PDV >= 2 )
	{
		printf( "EDT Driver  version: %s\n", m_DrvVersion.c_str() ); 
		printf( "EDT Library version: %s\n", m_LibVersion.c_str() );
	}

	// Fetch the full image geometry parameters and write them to ADBase parameters
    m_ClMaxWidth	= pdv_get_width(	m_pPdvDev );
    m_ClMaxHeight	= pdv_get_height(	m_pPdvDev );
	setIntegerParam( ADMaxSizeX,		m_ClMaxWidth	);
	setIntegerParam( ADMaxSizeY,		m_ClMaxHeight	);

	// Fetch the pixel depth and update ADBase DataType and BitsPerPixel
    m_ClNumBits		= pdv_get_depth(	m_pPdvDev );
	if ( m_ClNumBits <= 8 )
	{
		setIntegerParam( NDDataType,		NDUInt8	);
	}
	else if ( m_ClNumBits <= 16 )
	{
		setIntegerParam( NDDataType,		NDUInt16 );
	}
#ifdef NDBitsPerPixelString
	setIntegerParam( NDBitsPerPixel,	m_ClNumBits		);
#endif

	// setIntegerParam( NDArrayCallbacks,	1	);

	// See if we support deinterlacing raw images
	switch ( pDD->swinterlace )
	{
	//	TODO:	Add support for more interlace types as needed
	//	case PDV_BYTE_INTLV:
	//	case PDV_WORD_INTLV:
	case PDV_WORD_INTLV_MIDTOP_LINE:
		m_tyInterlace = pDD->swinterlace;
		break;
	default:
		m_tyInterlace = PDV_INTLV_IN_PDV_LIB;
		break;
	}
	
	// Save the number of horiz and vert taps
    m_ClHTaps		= pDD->htaps == NOT_SET ? 1 : pDD->htaps;
    m_ClVTaps		= pDD->vtaps == NOT_SET ? 1 : pDD->vtaps;

	// Set the number of horizontal and vertical taps
	setIntegerParam( EdtHTaps,		m_ClHTaps	);
	setIntegerParam( EdtVTaps,		m_ClVTaps	);

	free( pDD );

	if ( DEBUG_EDT_PDV >= 3 )
	{
		printf( "%s: Calling pdv_flush_fifo( ) ...\n", functionName );
	}
    pdv_flush_fifo( m_pPdvDev );

	if ( DEBUG_EDT_PDV >= 3 )
	{
		printf( "%s: Calling pdv_multibuf( %d ) ...\n", functionName, m_NumMultiBuf );
	}
    if ( pdv_multibuf( m_pPdvDev, m_NumMultiBuf ) )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW,
					"asynPrint "
					"%s %s: ERROR, pdv_multibuf() failed for %s\n",
					driverName,		functionName,	m_CameraName.c_str() );
        return -1;
	}

	//	Using default timeout based on exposure time
	pdv_set_timeout( m_pPdvDev, -1 );

	// Send the serial configuration requests
	setIntegerParam( SerTriggerMode,	m_TriggerModeReq	);
	setIntegerParam( SerBinX,			m_BinXReq	);
	setIntegerParam( SerBinY,			m_BinYReq	);
	setIntegerParam( SerMinX,			m_MinXReq	);
	setIntegerParam( SerMinY,			m_MinYReq	);
	setIntegerParam( SerSizeX,			m_SizeXReq	);
	setIntegerParam( SerSizeY,			m_SizeYReq	);
	// Don't think SerGain needs to be set here.
	// Should be OK to just FLNK it in edtPdvBase.template
	// setDoubleParam(  SerGain,			m_Gain		);

#ifdef	SETUP_ROI_IN_RECONFIG
	SetupROI();

	if ( fEnableFrameSync && pdv_framesync_mode(m_pPdvDev) != PDV_FRAMESYNC_OFF )
	{
		int framesync = pdv_enable_framesync(	m_pPdvDev, pdv_framesync_mode(m_pPdvDev) );
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: framesync enable %s\n", functionName, framesync == 0 ? "succeeded" : "failed" );
	}
	else
	{
		fCheckFrameSync = 0;
	}
#endif

    return 0;
}


int edtPdvCamera::SetupROI( )
{
    static const char	*	functionName = "edtPdvCamera::SetupROI";

	if (	(	GetSizeX() < m_ClMaxWidth  )
		||	(	GetSizeY() < m_ClMaxHeight ) )
	{
		// Setup PDV ROI image transfer
		// Note: If the camera has HW ROI, we set hskip and/or vskip to zero as
		// it is assumed that serial commands are used to configure the camera HW ROI
		int		hskip	= HasHwHRoi() ? 0 : GetMinX();
		int		vskip	= HasHwVRoi() ? 0 : GetMinY();

		// EDT Horiz and Vert Active line count must be a multiple of the number of CamLink taps
		// Pad up to next largest size 
		int		hactv	= ( (GetSizeX()+m_ClHTaps-1) / m_ClHTaps ) * m_ClHTaps;
		int		vactv	= ( (GetSizeY()+m_ClVTaps-1) / m_ClVTaps ) * m_ClVTaps;
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: Setting PDV ROI to hskip %d, hactv %d, vskip %d, vactv %d\n",
					functionName,	hskip, hactv, vskip, vactv );
		pdv_set_roi(	m_pPdvDev,	hskip, hactv, vskip, vactv );
		pdv_enable_roi(	m_pPdvDev, 1	);
		m_ClCurWidth	= hactv;
		m_ClCurHeight	= vactv;
	}
	else
	{
		int		hskip	= 0;
		int		vskip	= 0;
		// EDT Horiz and Vert Active line count must be a multiple of the number of CamLink taps
		// Pad up to next largest size 
		int		hactv	= ( (m_ClMaxWidth  + m_ClHTaps - 1) / m_ClHTaps ) * m_ClHTaps;
		int		vactv	= ( (m_ClMaxHeight + m_ClVTaps - 1) / m_ClVTaps ) * m_ClVTaps;
		assert(	hactv == (int) m_ClMaxWidth	);
		assert(	vactv == (int) m_ClMaxHeight	);
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: Disabling PDV ROI, restoring hskip %d, hactv %d, vskip %d, vactv %d\n",
					functionName,	hskip, hactv, vskip, vactv );
		pdv_set_roi(	m_pPdvDev,	hskip, hactv, vskip, vactv );
		pdv_enable_roi(	m_pPdvDev, 0	);
		m_ClCurWidth	= hactv;
		m_ClCurHeight	= vactv;
	}
	return 0;
}

//	Internal version of reopen
//	Don't call without holding m_reconfigLock!
int edtPdvCamera::_Reopen( )
{
    static const char	*	functionName = "edtPdvCamera::_Reopen";
	CONTEXT_TIMER( "_Reopen" );

	double		cameraReOpenDelay	= 0.0;
	if ( cameraReOpenDelay > 0.0 )
	{
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: Delaying %f sec\n", functionName, cameraReOpenDelay );
		epicsThreadSleep( cameraReOpenDelay );
	}

	// Set the EDT PDV debug levels
	if ( DEBUG_EDT_PDV >= 1 )
	{
		printf( "%s: Setting Pdv_debug = %d, Edt msg level debug = %d\n",
				functionName,  m_EdtDebugLevel, m_EdtDebugMsgLevel );
	}

	// Close old PdvDev if needed
	if ( m_pPdvDev )
	{
		if ( DEBUG_EDT_PDV >= 3 )
			printf( "%s: %s pdv_timeout_restart\n", functionName, m_CameraName.c_str() );
		pdv_timeout_restart( m_pPdvDev, 0 );
		// epicsThreadSleep( 0.1 );
		if ( DEBUG_EDT_PDV >= 3 )
			printf( "%s: %s pdv_close\n", functionName, m_CameraName.c_str() );
		pdv_close( m_pPdvDev );
		m_pPdvDev	= NULL;
		if ( DEBUG_EDT_PDV >= 1 )
			printf( "%s: %s old PdvDev closed.\n", functionName, m_CameraName.c_str() );
	}

	// Open the camera channel
	// Note: Do not use edt_open_channel() here!
	// Although pdv_open_channel() and edt_open_channel() both return ptrs
	// to the same underlying C structure, they behave differently in regards
	// to what memory they allocate and free and how some of the fields
	// are initialized.
	if ( DEBUG_EDT_PDV >= 1 )
		printf( "%s: %s Reopening PdvDev ...\n", functionName, m_CameraName.c_str() );
    m_pPdvDev = pdv_open_channel( "pdv", m_unit, m_channel );
    if ( m_pPdvDev == NULL )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW, 
					"%s %s: ERROR, Unable to open camera for EDT card %u, channel %u\n",
					driverName,		functionName, m_unit, m_channel );
        return -1;
    }

    char    fpga_name[128];
    printf( "Unit %d, Chan %d, Mode: %s\n",
			m_unit, m_channel, EdtModeToString( m_EdtMode ) );
    printf( "Boot sector FPGA header: \"%s\"\n",
			get_pci_fpga_header( m_pPdvDev , fpga_name));
	if ( m_EdtMode == EDTMODE_FULL && !strstr( fpga_name, "_fm" ) )
	{
	    printf( "\n\nERROR: Wrong firmware. You need to use full-mode FPGA version for this camera!\n\n" );
		pdv_close( m_pPdvDev );
		m_pPdvDev	= NULL;
        return -1;
    }
	if ( m_EdtMode != EDTMODE_FULL && strstr( fpga_name, "_fm" ) )
	{
	    printf( "\n\nERROR: Wrong firmware. You need to use a non-full-mode FPGA version for this camera!\n\n" );
		pdv_close( m_pPdvDev );
		m_pPdvDev	= NULL;
        return -1;
    }

	// Diagnostics
	if ( DEBUG_EDT_PDV >= 1 )
		printf(	"%s %s: %s framegrabber opened on card %u, ch %u\n",
				driverName, functionName, m_CameraName.c_str(), m_unit, m_channel );
	asynPrint(	this->pasynUserSelf,	ASYN_TRACEIO_DRIVER,
				"%s %s: %s framegrabber opened on card %u, ch %u\n",
				driverName, functionName, m_CameraName.c_str(), m_unit, m_channel );
    return 0;
}


int edtPdvCamera::UpdateADConfigParams( )
{
    static const char	*	functionName	= "edtPdvCamera::UpdateADConfigParams";
	if ( DEBUG_EDT_PDV >= 2 )
		printf( "%s: %s ...\n", functionName, m_CameraName.c_str() );

	if ( m_pPdvDev == NULL )
	{
		printf( "%s Error: Framegrabber %s not connected!\n", functionName, m_CameraName.c_str() );
		return -1;
	}


//	setIntegerParam( NDArraySizeX,		GetSizeX()	);
//	setIntegerParam( NDArraySizeY,		GetSizeY()	);
//	setIntegerParam( NDArraySize,		GetSizeX() * GetSizeY() );

//	if ( DEBUG_EDT_PDV >= 2 )
//	{
//		printf( "Camera %s: Image size %zu x %zu pixels, %u bits/pixel\n",
//				m_CameraName.c_str(), m_SizeXReq, m_SizeYReq, m_ClNumBits );
//	}

	// Diagnostics
	// TODO: Find a way to add new trace bits so we can combine these as
	//	asynPrint(	this->pasynUserSelf,	ASYN_TRACEIO_EDT, ... )
	//	Is it this simple?
	//	#define	ASYN_TRACEIO_EDT 0x40
	//	and add a custom asyn trace screen for gui 
//	if ( DEBUG_EDT_PDV >= 1 )
//		printf(
//				"%s %s: Camera %s ready on card %u, ch %u, %zu x %zu pixels, %u bits/pixel\n",
//				driverName, functionName, m_CameraName.c_str(), m_unit, m_channel
//				, GetSizeX(), GetSizeY(), m_ClNumBits
//				);
//	asynPrint(	this->pasynUserSelf,	ASYN_TRACEIO_DRIVER,
//				"%s %s: Camera %s ready on card %u, ch %u, %zu x %zu pixels, %u bits/pixel\n",
//				driverName, functionName, m_CameraName.c_str(), m_unit, m_channel
//				, GetSizeX(), GetSizeY(), m_ClNumBits
//				);
    return 0;
}


int edtPdvCamera::UpdateEdtDebugParams( )
{
	// Update PDV library Debug parameters
	setIntegerParam(	EdtDebug,		m_EdtDebugLevel	);
	// It's OK to call pdv_setdebug( NULL, level )
	pdv_setdebug(		m_pPdvDev, 		m_EdtDebugLevel	);

	// EDT MSG Levels
	//            INFO2   INFO1   WARN    FATAL
    // EDT Apps: 0x0008  0x0004  0x0002  0x0001
    // EDT  Lib: 0x0080  0x0040  0x0020  0x0010
    // PDV  Lib: 0x0800  0x0400  0x0200  0x0100
	// Always show fatal error msgs
	int	debugLevel	= EDT_MSG_FATAL;
	if ( m_EdtDebugLevel >= 1 )
		debugLevel += EDT_MSG_WARNING;
	if ( m_EdtDebugLevel >= 2 )
		debugLevel += EDT_MSG_INFO_1;
	if ( m_EdtDebugLevel >= 3 )
		debugLevel += EDT_MSG_INFO_2;
	debugLevel |= m_EdtDebugMsgLevel;
	edt_msg_set_level(	edt_msg_default_handle(),	debugLevel	);

    return 0;
}

int edtPdvCamera::SetEdtDebugLevel( int value )
{
	// Update PDV library Debug parameters
	m_EdtDebugLevel = value;
	setIntegerParam(	EdtDebug,		m_EdtDebugLevel	);
	UpdateEdtDebugParams();
	return 0;
}

int edtPdvCamera::SetEdtDebugMsgLevel( int value )
{
	// Update PDV library Debug parameters
	m_EdtDebugMsgLevel = value;
	setIntegerParam(	EdtDebugMsg,	m_EdtDebugMsgLevel	);
	UpdateEdtDebugParams();
	return 0;
}

int edtPdvCamera::GetEdtDebugLevel( )
{
	return pdv_debug_level(	);
}

int edtPdvCamera::GetEdtDebugMsgLevel( )
{
	return edt_msg_default_level( );
}

int edtPdvCamera::SetSerDisable( int value )
{
    printf("******************** DEBUG HUGO: SetSerDisable to: %d\n", value);
    m_SerialDisable = value;
	asynStatus		status	= setIntegerParam( SerDisable, m_SerialDisable );
	if( status == asynSuccess )
		status = callParamCallbacks( 0, 0 );
	return status;
}

int edtPdvCamera::ResetSyncCounters()
{
    m_SyncTotal = 0;
    m_SyncBadTS = 0;
    m_SyncBadSync = 0;

    return asynSuccess;
}

int edtPdvCamera::IncrSyncTotalCount()
{
    m_SyncTotal += 1;
	asynStatus		status	= setIntegerParam( SyncTotal, m_SyncTotal );
	if( status == asynSuccess )
		status = callParamCallbacks( 0, 0 );
	return status;
}

int edtPdvCamera::IncrSyncBadTSCount()
{
    m_SyncBadTS += 1;
	asynStatus		status	= setIntegerParam( SyncBadTS, m_SyncBadTS );
	if( status == asynSuccess )
		status = callParamCallbacks( 0, 0 );
	return status;
}

int edtPdvCamera::IncrSyncBadSyncCount()
{
    m_SyncBadSync += 1;
	asynStatus		status	= setIntegerParam( SyncBadSync, m_SyncBadSync );
	if( status == asynSuccess )
		status = callParamCallbacks( 0, 0 );
	return status;
}

asynStatus	edtPdvCamera::UpdateStatus( int	newStatus	)
{
	if ( DEBUG_EDT_PDV >= 4 )
	{
    	static const char	*	functionName = "edtPdvCamera::UpdateStatus";
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );
	}
	CONTEXT_TIMER( "edtPdvCamera-UpdateStatus" );
	//	Context timer shows these next two calls take about 20us
	asynStatus		status	= setIntegerParam( ADStatus, newStatus );
	if( status == asynSuccess )
		status = callParamCallbacks( 0, 0 );
	return status;
}

// Set NDArrayCounter
asynStatus edtPdvCamera::SetArrayCounter( int value )
{
	CONTEXT_TIMER( "edtPdvCamera-SetArrayCounter" );
	m_ArrayCounter = value;
	asynStatus	status	= setIntegerParam( NDArrayCounter,	m_ArrayCounter );
	if( status == asynSuccess )
		status = callParamCallbacks( 0, 0 );
	return status;
}

// Increment NDArrayCounter
asynStatus edtPdvCamera::IncrArrayCounter()
{
	CONTEXT_TIMER( "edtPdvCamera-IncrArrayCounter" );
	m_ArrayCounter++;
	asynStatus	status	= setIntegerParam(	NDArrayCounter,	m_ArrayCounter	);
	if( status == asynSuccess )
		status = callParamCallbacks( 0, 0 );
	return status;
}

asynStatus	edtPdvCamera::SetAcquireMode( int fAcquire )
{
    static const char	*	functionName = "edtPdvCamera::SetAcquireMode";
	if ( fAcquire == 0 )
	{
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: Stopping acquisition on camera %s\n", 
					functionName, m_CameraName.c_str() );
		// Stop acquisition
		m_fAcquireMode = false;
		m_acquireCount = 0;
		// TODO: Is this the right place to call pdv_timeout_restart? No this funciton just requests we stop acquire
		// pdv_timeout_restart( m_pPdvDev, 0 );

		UpdateStatus( ADStatusIdle	);
	}
	else
	{
		epicsMutexLock(	m_reconfigLock );
		if ( DEBUG_EDT_PDV >= 1 )
			printf(	"%s: Starting acquisition on camera %s\n", 
					functionName, m_CameraName.c_str() );
		if( m_pSyncDataAcquirer != NULL )
		{
			m_pSyncDataAcquirer->SetEnabled();
			if ( m_TriggerModeReq == TRIGMODE_FREERUN )
			{
				// Use any images we can get
				m_pSyncDataAcquirer->SetPolicyUnsynced( syncDataAcq<edtPdvCamera, edtImage>::USE_OBJECT	);
				m_pSyncDataAcquirer->SetPolicyBadTimeStamp( syncDataAcq<edtPdvCamera, edtImage>::USE_OBJECT	);
			}
			else
			{
				// Skip images when unsynce or invalid timestamp
				m_pSyncDataAcquirer->SetPolicyUnsynced( syncDataAcq<edtPdvCamera, edtImage>::SKIP_OBJECT	);
				m_pSyncDataAcquirer->SetPolicyBadTimeStamp( syncDataAcq<edtPdvCamera, edtImage>::SKIP_OBJECT	);
			}
		}
		// UpdateAcquireCount()
		{
		// See how many images we need to acquire
		int			imageMode;
		int			numImages;
		getIntegerParam( ADImageMode,	&imageMode	);
		getIntegerParam( ADNumImages,	&numImages	);

		switch( imageMode )
		{
		case ADImageSingle:
			m_acquireCount = 1;
			break;
		case ADImageMultiple:
			m_acquireCount = numImages;
			break;
		case ADImageContinuous:
			m_acquireCount = -1;
			break;
		}
		}
		m_fAcquireMode = true;
		if( m_pSyncDataAcquirer != NULL )
			m_pSyncDataAcquirer->SignalAcquireEvent();
		epicsMutexUnlock(	m_reconfigLock );
	}
	setIntegerParam( ADAcquire, m_fAcquireMode );
	return asynSuccess;
}

int edtPdvCamera::StartAcquisition( )
{
    static const char	*	functionName = "edtPdvCamera::StartAcquisition";
	CONTEXT_TIMER( "StartAcquisition" );

#ifndef	SETUP_ROI_IN_RECONFIG
	SetupROI();

	if ( fEnableFrameSync && pdv_framesync_mode(m_pPdvDev) != PDV_FRAMESYNC_OFF )
	{
		int framesync = pdv_enable_framesync(	m_pPdvDev, pdv_framesync_mode(m_pPdvDev) );
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: framesync enable %s\n", functionName, framesync == 0 ? "succeeded" : "failed" );
	}
	else
	{
		fCheckFrameSync = 0;
	}
#endif	//	SETUP_ROI_IN_RECONFIG

	double		cameraStartDelay	= 0.25;
	if ( cameraStartDelay > 0.0 )
	{
		if ( DEBUG_EDT_PDV >= 2 )
			printf(	"%s: Delaying %f sec\n", functionName, cameraStartDelay );
		epicsThreadSleep( cameraStartDelay );
	}

	if ( m_fReconfig || !InAcquireMode() )
    	return -1;

	if ( DEBUG_EDT_PDV >= 2 )
		printf(	"%s: Acquire image from %zu,%zu size %zux%zu\n", functionName,
				GetMinX(), GetMinY(), GetSizeX(), GetSizeY()	);

	// Clear NumImagesCounter and start acquisition
	setIntegerParam( ADNumImagesCounter, 0 );
	UpdateStatus( ADStatusAcquire );

	if ( DEBUG_EDT_PDV >= 3 )
		printf( "%s: Calling pdv_start_images( pPdvDev, %d ) ...\n", functionName, m_NumMultiBuf );

	// Start grabbing images
	pdv_start_images( m_pPdvDev, m_NumMultiBuf );

	if ( DEBUG_EDT_PDV >= 1 )
        printf(	"%s: Start acquire, count = %d\n",
				functionName, m_acquireCount );
	asynPrint(	this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
        		"%s: Start acquire, count = %d\n",
				functionName, m_acquireCount );
    return 0;
}

// Interleave function definitions

///
///	DeIntlvMidTopLine16( )
/// Takes a full line from middle, then iterates to the top
/// The next line is from the middle + 1, then iterates to the bottom.
/// DMA Buffer has alternating rows from the middle of the image,
///	iterating out to the top and bottom.
/// Example for 400 line image
///		dmaRow		imgRow
///		398			0
///		396			1
///		394			2
///		...			...
///		2			198
///		0			199
///		1			200
///		3			201
///		...			...
///		397			398
///		399			399
///
int edtPdvCamera::DeIntlvMidTopLine16( NDArray * pNDArray, void	*	pRawData )
{
	assert( pNDArray		!= NULL );
	assert( pNDArray->pData	!= NULL );
	assert( pRawData		!= NULL );
    static const char	*	functionName	= "edtPdvCamera::DeIntlvMidTopLine16";
	if ( DEBUG_EDT_PDV >= 4 )
	{
		printf( "%s: DMA   size %zu x %zu pixels, %u bits/pixel\n", functionName,
				m_ClCurWidth, m_ClCurHeight, m_ClNumBits );
		printf(	"%s: Image size %zu x %zu pixels, offset(%zu,%zu)\n", functionName,
				GetSizeX(), GetSizeY(), GetMinX(), GetMinY()	);
	}
	CONTEXT_TIMER( "DeIntlvMidTopLine16" );

	epicsUInt16	*	pDmaBuffer	= (epicsUInt16 *) pRawData;
	epicsUInt16	*	pArrayData	= (epicsUInt16 *) pNDArray->pData;
	size_t			halfHeight	= m_ClCurHeight / 2;

	/* first copy upper half, middle to top */
	for (	size_t	imgRow = 0;	imgRow < halfHeight;	imgRow++	)
	{
		size_t			dmaRow		= m_ClCurHeight - 2 - imgRow * 2;
		epicsUInt16	*	pPixelSrc	= pDmaBuffer + dmaRow * m_ClCurWidth;
		epicsUInt16	*	pPixelDst	= pArrayData + imgRow * GetSizeX();
		memcpy( pPixelDst, pPixelSrc, GetSizeX() * 2 );
	}

	/* next copy lower half, middle to bottom */
	for (	size_t	imgRow = halfHeight;	imgRow < m_ClCurHeight;	imgRow++	)
	{
		size_t			dmaRow		= (imgRow - halfHeight) * 2 + 1;
		epicsUInt16	*	pPixelSrc	= pDmaBuffer + dmaRow * m_ClCurWidth;
		epicsUInt16	*	pPixelDst	= pArrayData + imgRow * GetSizeX();
		memcpy( pPixelDst, pPixelSrc, GetSizeX() * 2 );
	}
	return(0);
}


///
///	DeIntlvRoiOnly16( )
/// De-interleave as is from top to bottom, allowing only for HW ROI
///
int edtPdvCamera::DeIntlvRoiOnly16( NDArray * pNDArray, void	*	pRawData )
{
	assert( pNDArray		!= NULL );
	assert( pNDArray->pData	!= NULL );
	assert( pRawData		!= NULL );
    // static const char	*	functionName	= "edtPdvCamera::DeIntlvRoiOnly16";
	CONTEXT_TIMER( "DeIntlvRoiOnly16" );
	// Image already de-interleaved in PDV library, just memcpy it here.
	// memcpy( pNDArray->pData, pRawData, nBytes );
	epicsUInt16	*	pDmaBuffer	= (epicsUInt16 *) pRawData;
	epicsUInt16	*	pArrayData	= (epicsUInt16 *) pNDArray->pData;
	for (	size_t	row = 0;	row < m_ClCurHeight;	row ++	)
	{
		epicsUInt16	*	pPixelSrc	= pDmaBuffer + row * m_ClCurWidth;
		epicsUInt16	*	pPixelDst	= pArrayData + row * GetSizeX();
		memcpy( pPixelDst, pPixelSrc, GetSizeX() * 2 );
	}
	return 0;
}

int edtPdvCamera::AcquireData( edtImage	*	pImage )
{
    static const char	*	functionName = "edtPdvCamera::AcquireData";
	CONTEXT_TIMER( "AcquireData" );
	assert( m_pPdvDev != NULL );

	//	double				timeOutSec	= 0.5;
	//	pdv_set_timeout( m_pPdvDev, static_cast<int>(timeOutSec * 1000) );
	//	Using default timeout based on exposure time
	//	pdv_set_timeout( m_pPdvDev, -1 );
	UpdateStatus( ADStatusWaiting );

#ifdef	USE_DIAG_TIMER
	// The diag. ReAcquireTimer times the interval between calls to pdv_wait_images_*
	m_ReAcquireTimer.StopTimer( );
#endif	//	USE_DIAG_TIMER

	// Wait for a new image
	u_int				pdvTimestamp[2];
	bool				fRaw		= ( m_tyInterlace != PDV_INTLV_IN_PDV_LIB );
	unsigned char	*	pPdvBuffer	= pdv_wait_images_timed_raw( m_pPdvDev, 1, pdvTimestamp, fRaw );
	//	unsigned char	*	pPdvBuffer	= pdv_wait_images_raw( m_pPdvDev, 1 );
	//	Caution, this raw image will get over-written if we're not done processing it
	//	before the edt ring buffer wraps.

#ifdef	USE_DIAG_TIMER
	// Update diagnostic timers
	m_ReArmTimer.StartTimer( );
	m_ReAcquireTimer.StartTimer( );
	m_ProcessImageTimer.StartTimer( );
#endif	//	USE_DIAG_TIMER

	// See if we exited acquire mode while waiting for an image
	if ( m_fReconfig || !InAcquireMode() )
	{
		if ( DEBUG_EDT_PDV >= 1 )
			printf(	"%s: AcquireMode cancelled in thread %s\n", functionName, epicsThreadGetNameSelf() );

		// TODO: Is this the right place to call pdv_timeout_restart? Yes, just returned from pdv_wait but need to exit loop
		if ( DEBUG_EDT_PDV >= 3 )
			printf(	"%s: Calling pdv_timeout_restart\n", functionName );
		pdv_timeout_restart( m_pPdvDev, 0 );
		return asynError;
	}

	// Decrement acquire count
	if( m_acquireCount > 0 )
		m_acquireCount--;

	int	edtWaitStatus	= edt_get_wait_status( m_pPdvDev );
	if ( pPdvBuffer == NULL || edtWaitStatus != EDT_WAIT_OK )
	{
#ifdef	USE_DIAG_TIMER
		m_ReArmTimer.StopTimer( );
#endif	//	USE_DIAG_TIMER
		if ( DEBUG_EDT_PDV >= 1 )
			printf(	"%s: Image Timeout: Failed to acquire image!\n", functionName );
		pdv_timeout_restart( m_pPdvDev, 0 );
		return asynError;
	}

	if ( m_acquireCount != 0 )
	{	// edtPdvCamera::ReArm()
#ifdef	USE_DIAG_TIMER
		m_ReArmTimer.StopTimer( );
#endif	//	USE_DIAG_TIMER
		//	Continue the pipeline by starting the next image
		CONTEXT_TIMER( "pdv_start_image" );
		if ( DEBUG_EDT_PDV >= 4 )
			printf(	"%s: Calling pdv_start_image in thread %s\n", functionName, epicsThreadGetNameSelf() );
		// TODO: Consider delaying this in free run mode
		pdv_start_image( m_pPdvDev );
	}

	if ( fCheckFrameSync )
	if ( m_TriggerMode != TRIGMODE_FREERUN && pdv_framesync_mode( m_pPdvDev ) != PDV_FRAMESYNC_OFF )
	{
		CONTEXT_TIMER( "AcquireData-pdv_check_framesync" );
    	u_int frame_counter;
    	int	 framesync_status = pdv_check_framesync( m_pPdvDev, pPdvBuffer, &frame_counter );
		if ( framesync_status == 0 )
		{
			if ( DEBUG_EDT_PDV >= 5 )
				printf(	"%s: IRIG2 framesync counter %d\n", functionName, frame_counter );
		}
		if ( framesync_status < 0 )
		{
			if ( DEBUG_EDT_PDV >= 3 )
				printf(	"%s: pdv framesync not enabled!\n", functionName );
		}
		else if ( framesync_status > 0 )
		{
			if ( DEBUG_EDT_PDV >= 3 )
				printf(	"%s: Lost framesync!\n", functionName );
			// TODO: Add cameraLostFrameSyncCounter
			// setIntegerParam(cameraLostFrameSyncCounter, lostFrameSyncCounter++);

			// Restart acquisition on lost framesync!
			pdv_timeout_restart( m_pPdvDev, 0 );	// Bailing on pdv_check_framesync error!
			return asynError;
		}
	}

	// This counts camera frames whether we're aqcuiring or not.
	// If the camera is in freerun or being triggered, it should increment.
	int		nCamFramesSinceReset = pdv_cl_get_fv_counter( m_pPdvDev );
	pdv_cl_reset_fv_counter( m_pPdvDev );
	if ( DEBUG_EDT_PDV >= 4 )
		printf(	"%s: %d camera frames since last capture\n", functionName, nCamFramesSinceReset );
	// TODO: Consider using nCamFramesSinceReset as an additional timestamp sync factor

	UpdateStatus( ADStatusReadout );

	// Lock NDArrayPool driver
	lock();

	//	TODO: We haven't tried to get a timestamp yet,
	//	so we don't know yet if this is a frame we want.
	//	Can we check timestamp before transferring image from DMA buffer?`

	// Allocate an NDArray from the pool
    NDArray		*	pNDArray = AllocNDArray();
	if ( pNDArray != NULL )
	{
		// Transfer the raw DMA buffer to the NDArray
		int status = LoadNDArray( pNDArray, (void *) pPdvBuffer );
		if ( status != 0 )
		{
			pNDArray->release( );
			pNDArray = NULL;
		}
		else
		{
			pImage->SetNDArrayPtr( pNDArray );
			if ( DEBUG_EDT_PDV >= 5 && pNDArray != NULL )
			{
				// Write NDArray report to stdout
				// if details param >= 5, calls NDAttributeList::report()
				// NDArray::report( FILE * fp, int details );
				pNDArray->report( stdout, DEBUG_EDT_PDV );
			}
		}
	}
#ifdef	USE_DIAG_TIMER
	m_ProcessImageTimer.StopTimer( );
#endif	//	USE_DIAG_TIMER

	unlock();

	{
	CONTEXT_TIMER( "AcquireData-wrapup" );

	// Increment NumImagesCounter
	//	TODO: Replace this pattern w/ local m_numImagesCounter
	//	m_numImagesCounter++
	//	setIntegerParam( ADNumImagesCounter, m_numImagesCounter );
	int		numImagesCounter;
	getIntegerParam(	ADNumImagesCounter,	&numImagesCounter	);
	numImagesCounter++;
	setIntegerParam(	ADNumImagesCounter,	numImagesCounter	);

	// See if we're done
	if ( m_acquireCount == 0 )
	{
		SetAcquireMode( false );
		if ( DEBUG_EDT_PDV >= 1 )
			printf(	"%s: Image acquisition completed in thread %s\n", functionName, epicsThreadGetNameSelf() );
		asynPrint(	this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
					"%s: Image acquisition completed in thread %s\n", functionName, epicsThreadGetNameSelf() );
		// pdv_timeout_restart( m_pPdvDev, 0 );
	}
	}
	return 0;
}

//
//	Allocate and prepare an NDArray to receive an image
//	Note: Driver must be locked before calling this routine
NDArray * edtPdvCamera::AllocNDArray( )
{
    static const char	*	functionName = "edtPdvCamera::AllocNDArray";
	CONTEXT_TIMER( "AcquireData-NDArrayPool-Update" );

	// TODO: Handle color images and 32 bit images
	NDDataType_t	pixelType		= NDUInt8;
	if ( m_ClNumBits > 8 )
	{
		pixelType		= NDUInt16;
	}

	// Allocate an NDArray
    const int		ndims	= 2;
    size_t			dims[ndims];
    dims[0]			= GetSizeX();
    dims[1]			= GetSizeY();
	assert( dims[0] != 0 );
	assert( dims[1] != 0 );

	// assert( m_pNDArray == NULL );
    NDArray		*	pNDArray = pNDArrayPool->alloc( ndims, dims, pixelType, 0, NULL );
	if ( pNDArray == NULL )
	{
        errlogPrintf( "%s: NULL NDArray ptr!\n", functionName );
		return NULL;
	}
	if ( pNDArray->pData == NULL )
	{
		pNDArray->release( );
		pNDArray = NULL;
        errlogPrintf( "%s: NULL NDArray->pData!\n", functionName );
		return NULL;
	}

	// Set the NDArray parameters
	assert( pNDArray			!= NULL );
	pNDArray->ndims				= ndims;
#ifdef NDBitsPerPixel
	pNDArray->bitsPerElement	= m_ClNumBits;
#endif

	// Update each dimension's settings from the RBV values
	pNDArray->dims[0].size		= GetSizeX();
	pNDArray->dims[0].offset	= GetMinX();
	pNDArray->dims[0].binning	= GetBinX();
	pNDArray->dims[1].size		= GetSizeY();
	pNDArray->dims[1].offset	= GetMinY();
	pNDArray->dims[1].binning	= GetBinY();
	if ( DEBUG_EDT_PDV >= 4 )
		printf(	"%s: NDArray %zux%zu pixels from offset (%zu,%zu)\n",	functionName,
				pNDArray->dims[0].size,		pNDArray->dims[1].size,
				pNDArray->dims[0].offset,	pNDArray->dims[1].offset	);
	return pNDArray;
}

//
//	Load image from DMA buffer to NDArray
//	Note: Driver must be locked before calling this routine
int edtPdvCamera::LoadNDArray( NDArray * pNDArray, void * pRawData )
{
    static const char	*	functionName = "edtPdvCamera::LoadNDArray";
	int		status = 0;

	// See if we support deinterlacing raw images
	switch ( m_tyInterlace )
	{
	case PDV_WORD_INTLV_MIDTOP_LINE:
		DeIntlvMidTopLine16( pNDArray, pRawData );
		break;
	case PDV_INTLV_IN_PDV_LIB:
		DeIntlvRoiOnly16( pNDArray, pRawData );
		break;
	default:
        errlogPrintf( "%s: Invalid interlace type %d!\n", functionName, m_tyInterlace );
		status = -1;
		break;
	}

	return status;
}


int	edtPdvCamera::ProcessData(
	edtImage		*	pImage,
	epicsTimeStamp	*	pTimeStamp,
	int					pulseID	)
{
    static const char	*	functionName = "edtPdvCamera::ProcessData";
	if ( pImage == NULL )
		return -1;

	UpdateStatus( ADStatusSaving );
	this->lock();
    NDArray	*	pNDArray = pImage->GetNDArrayPtr( );
	if ( pNDArray != NULL )
	{
		CONTEXT_TIMER( "edtPdvCamera-ProcessData" );
		// Set the NDArray EPICS timestamp and unique ID
		if ( DEBUG_EDT_PDV >= 4 )
			printf(	"%s: Timestamp image w/ pulseID %d, 0x%X\n", functionName, pulseID, pulseID );
		
		// NDArray has two timestamps.
		//	epicsTimeStamp	NDArray::epicsTS	is epics sec and ns relative to 1990
		pNDArray->epicsTS	= *pTimeStamp;
		pNDArray->uniqueId	= pulseID;

		// Compute POSIX timeStamp in floating point
		//	double			NDArray::timeStamp	is seconds since 1970
		pNDArray->timeStamp	= POSIX_TIME_AT_EPICS_EPOCH + pTimeStamp->secPastEpoch
							+ pTimeStamp->nsec * 1e-9;

		//
		// asynPortDriver also allows us to provide custom routines:
		//	getTimeStamp(epicsTimeStamp * pTS)
		//	setTimeStamp(const epicsTimeStamp * pTS)
		// Not sure if these will be needed.
		//
		// asyn PV's normally get their timestamp from
		//	epicsTimeStamp	asynUser::timestamp
		//
		// In AD plugins such as StdArray, pasynUser->timestamp is set from pNDArray->epicsTS
		// before calling the interrupt callback, which appears to
		// set pwf->time = pasynUser->timestamp in devAsynXXXArray.h
		//
		// NDArray::timeStamp is used to set double param NDTimeStamp
		//

		/* Update our frame count */
		IncrArrayCounter();

		int	arrayCallbacks;
		getIntegerParam( NDArrayCallbacks, &arrayCallbacks );
		if ( arrayCallbacks )
		{
			// Do NDArray callbacks unlocked to avoid deadlocks if the plugin
			// tries to lock the driver.
			this->unlock();
			if ( DEBUG_EDT_PDV >= 4 )
				printf(	"%s: Processing image callbacks ...\n", functionName );
			doCallbacksGenericPointer( pNDArray, NDArrayData, 0 );
			this->lock();
		}

		if ( DEBUG_EDT_PDV >= 4 )
			printf(	"%s: Processing parameter callbacks ...\n", functionName );

		// Call parameter callbacks
		callParamCallbacks();
	}
	this->unlock();
	return 0;
}

#ifndef	INVALID_PULSE
#define	INVALID_PULSE	0x1FFFF
#endif	//	INVALID_PULSE
bool	 edtPdvCamera::IsSynced(
	edtImage		*	pImage,
	epicsTimeStamp	*	pTimeStamp,
	int					pulseID		)
{
	CONTEXT_TIMER( "edtPdvCamera-IsSynced" );
	if ( pImage == NULL )
		return false;
	if ( m_TriggerMode == TRIGMODE_FREERUN )
		return true;
	if ( pTimeStamp == NULL )
		return false;
	if ( pulseID == INVALID_PULSE )
		return false;
	return true;
}


// CheckData returns 0 on OK, non-zero on error
int	 edtPdvCamera::CheckData(	edtImage	*	pImage	)
{
	CONTEXT_TIMER( "edtPdvCamera-CheckData" );
	if ( pImage == NULL || m_pPdvDev == NULL )
		return -1;

	// Check for data overruns in the framegrabber
	int		nOverrun	= pdv_overrun( m_pPdvDev );
	setIntegerParam( EdtOverrun, nOverrun );

	// Check for timeouts in the framegrabber
	int		timeoutCount	= 0;
	// getIntegerParam( EdtTimeouts, &timeoutCount );
	int		timeouts		= pdv_timeouts( m_pPdvDev );
	if ( timeouts > timeoutCount )
	{
		// setIntegerParam( EdtTimeouts, timeouts );
		// setIntegerParam( EdtTimeout, 1 );
	}
	else
	{
		// setIntegerParam( EdtTimeout, 0 );
	}

	return 0;
}

void edtPdvCamera::ReleaseData(	edtImage	*	pImage	)
{
	CONTEXT_TIMER( "ReleaseData" );
	UpdateStatus( ADStatusIdle );
	if ( pImage == NULL )
		return;
	this->lock();
	pImage->ReleaseNDArray();
	this->unlock();
}

int	edtPdvCamera::TimeStampImage(
	edtImage		*	pImage,
	epicsTimeStamp	*	pDest,
	int				*	pPulseNumRet	)
{
    static const char	*	functionName = "edtPdvCamera::TimeStampImage";
	if ( pImage == NULL )
		return -1;
	if ( pDest == NULL )
		return -1;
	epicsTimeStamp		newEvrTime;

	// This call to asynPortDriver::updateTimeStamp causes
	// asyn to call the registered timeStampSource function.
	// defaultTimeStampSource just calls epicsTimeGetCurrent.
	// Use asyn's registerTimeStampSource to register a new
	// timeStampSource function from code, or Kukhee's
	// registerUserTimeStampSource function from areaDetectorSupp
	// to register one from iocsh using the function's name.
	{
	CONTEXT_TIMER( "TimeStampImage-updateTimeStamp" );
	updateTimeStamp( &newEvrTime );
	}

	//	TODO:	Create a subclass of epicsTime which knows how to get
	//			and set pulseID's in an epicsTimeStamp, i.e. SLAC's pulseId=(ts.nSec & 0x1FFFF);
	//	We can't construct an epicsTime directly from our epicsTimeStamp as we sometimes
	//	set the nSec field > 1e9
	epicsTimeStamp	newTimeStamp( newEvrTime );
	if (	newTimeStamp.secPastEpoch	== m_priorTimeStamp.secPastEpoch
		&&	newTimeStamp.nsec			== m_priorTimeStamp.nsec )
	{
		char	acBuffer[32];
		epicsTimeToStrftime( acBuffer, 32, "%02H:%02M:%02S.%3f", &newTimeStamp );
		int	pulseId = newEvrTime.nsec & 0x1FFFF;
		asynPrint(	pasynUserSelf,	ASYN_TRACE_FLOW,
					"%s: Duplicate TimeStamp %s, pulseID %d\n", functionName, acBuffer, pulseId );
		return -1;
	}

	m_priorTimeStamp	= newTimeStamp;
	*pDest				= newTimeStamp;

	// TODO: Is there any way to make this less SLAC specific?
	// If not, maybe we just drop it, as no one really
	// needs uniqueId to be the pulse number
	if ( pPulseNumRet != NULL )
		*pPulseNumRet = pDest->nsec & 0x1FFFF;

	if ( m_TriggerMode != TRIGMODE_FREERUN )
	{
		if ( (pDest->nsec & 0x1FFFF) == 0x1FFFF )
		{
			char	acBuffer[32];
			epicsTimeToStrftime( acBuffer, 32, "%H:%M:%S.%04f", pDest );
			asynPrint(	pasynUserSelf,	ASYN_TRACE_FLOW,
						"%s: TimeStamp %s, invalid pulseID 0x%X\n", functionName, acBuffer, pDest->nsec & 0x1FFFF );
			return -1;
		}
	}

	return 0;
}


int	edtPdvCamera::RequestSizeX(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::RequestSizeX";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, HW ROI width cannot be set to 0!\n",
        	    		functionName );
		return asynError;
	}
	if( m_SizeXReq != value )
	{
		m_SizeXReq	= value;
		m_fReconfig	= true;
		if ( DEBUG_EDT_PDV >= 2 )
		{
			printf(	"%s: Requesting SizeX %zu ...\n", functionName, value );
		}
	}
	return asynSuccess;
}

int	edtPdvCamera::SetSizeX(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetSizeX";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, HW ROI width cannot be set to 0!\n",
        	    		functionName );
		// return asynError;
		value = m_SizeX > 0 ? m_SizeX : m_ClMaxWidth;
	}
	// Allow setting SizeX if m_ClMaxWidth hasn't been configured yet
	// Will be clipped later if needed
	if ( m_ClMaxWidth != 0 && value > m_ClMaxWidth )
	{
        errlogPrintf(	"%s: ERROR, HW ROI width %zu > max %zu!\n",
        	    		functionName, value, m_ClMaxWidth );
		// return asynError;
		value = m_ClMaxWidth;
	}
	if ( DEBUG_EDT_PDV >= 2 )
	{
		printf(	"%s: Set SizeX %zu ...\n", functionName, value );
	}
	if( m_SizeX != value )
		m_fReconfig	= true;
	m_SizeX	= value;

	// Update the AreaDetector SizeX parameters
	setIntegerParam( ADSizeX,		m_SizeX	);
	setIntegerParam( NDArraySizeX,	m_SizeX	);
	setIntegerParam( NDArraySize,	m_SizeX * m_SizeY );

//	int	status = (asynStatus) UpdateADConfigParams( );
	return asynSuccess;
}

int	edtPdvCamera::RequestSizeY(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::RequestSizeY";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, HW ROI height cannot be set to 0!\n",
        	    		functionName );
		return asynError;
	}
	if( m_SizeYReq != value )
	{
		m_SizeYReq	= value;
		m_fReconfig	= true;
		if ( DEBUG_EDT_PDV >= 2 )
		{
			printf(	"%s: Requesting SizeY %zu ...\n", functionName, value );
		}
	}
	return asynSuccess;
}

int	edtPdvCamera::SetSizeY(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetSizeY";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, HW ROI height cannot be set to 0!\n",
        	    		functionName );
		//	return asynError;
		value = m_SizeY > 0 ? m_SizeY : m_ClMaxHeight;
	}
	// Allow setting SizeY if m_ClMaxHeight hasn't been configured yet
	// Will be clipped later if needed
	if ( m_ClMaxHeight != 0 && value > m_ClMaxHeight )
	{
        errlogPrintf(	"%s: ERROR, ROI height %zu > max %zu!\n",
        	    		functionName, value, m_ClMaxHeight );
		//	return asynError;
		value = m_ClMaxHeight;
	}
	if ( DEBUG_EDT_PDV >= 2 )
	{
		printf(	"%s: Set SizeY %zu ...\n", functionName, value );
	}
	if( m_SizeY != value )
		m_fReconfig	= true;
	m_SizeY		= value;

	// Update the AreaDetector parameter
	setIntegerParam( ADSizeY,		m_SizeY	);
	setIntegerParam( NDArraySizeY,	m_SizeY	);
	setIntegerParam( NDArraySize,	m_SizeX * m_SizeY );

//	int	status = (asynStatus) UpdateADConfigParams( );
	return asynSuccess;
}

int	edtPdvCamera::RequestMinX(	size_t	value	)
{
	static const char	*	functionName	= "edtPdvCamera::RequestMinX";
	if( m_MinXReq != value )
	{
		m_MinXReq	= value;
		m_fReconfig	= true;
		if ( DEBUG_EDT_PDV >= 2 )
		{
			printf(	"%s: Requesting MinX %zu ...\n", functionName, value );
		}
	}
	return asynSuccess;
}

int	edtPdvCamera::SetMinX(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetMinX";
	if ( value > (m_ClMaxWidth - 1) )
	{
        errlogPrintf(	"%s: ERROR, ROI start %zu > max %zu!\n",
        	    		functionName, value, (m_ClMaxWidth - 1) );
		return asynError;
	}
	if ( DEBUG_EDT_PDV >= 2 )
	{
		printf(	"%s: Set MinX %zu ...\n", functionName, value );
	}
	if( m_MinX != value )
		m_fReconfig	= true;
	m_MinX	= value;

	// Update the AreaDetector parameter
	setIntegerParam( ADMinX,	m_MinX	);
//	int	status = (asynStatus) UpdateADConfigParams( );
	return asynSuccess;
}

int	edtPdvCamera::RequestMinY(	size_t	value	)
{
	static const char	*	functionName	= "edtPdvCamera::RequestMinY";
	if( m_MinYReq != value )
	{
		m_MinYReq	= value;
		m_fReconfig	= true;
		if ( DEBUG_EDT_PDV >= 2 )
		{
			printf(	"%s: Requesting MinY %zu ...\n", functionName, value );
		}
	}
	return asynSuccess;
}

int	edtPdvCamera::SetMinY(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetMinY";
	if ( value > (m_ClMaxHeight - 1) )
	{
        errlogPrintf(	"%s: ERROR, ROI start %zu > max %zu!\n",
        	    		functionName, value, (m_ClMaxHeight - 1) );
		return asynError;
	}
	if ( DEBUG_EDT_PDV >= 2 )
	{
		printf(	"%s: Set MinY %zu ...\n", functionName, value );
	}
	if( m_MinY != value )
		m_fReconfig	= true;
	m_MinY		=  value;

	// Update the AreaDetector parameter
	setIntegerParam( ADMinY,	m_MinY	);
//	int	status = (asynStatus) UpdateADConfigParams( );
	return asynSuccess;
}

int	edtPdvCamera::RequestBinX(	unsigned int	value	)
{
    static const char	*	functionName	= "edtPdvCamera::RequestBinX";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, HW ROI binning %u == 0!\n",
        	    		functionName, value );
		return asynError;
	}
	if( m_BinXReq != value )
	{
		m_BinXReq	= value;
		m_fReconfig	= true;
	}
	return asynSuccess;
}

int	edtPdvCamera::SetBinX(	unsigned int	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetBinX";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, ROI bin %u == 0!\n",
        	    		functionName, value );
		return asynError;
	}
	m_BinX		= value;

	// Update the AreaDetector parameter
	setIntegerParam( ADBinX,	m_BinX	);
//	int	status = (asynStatus) UpdateADConfigParams( );
	return asynSuccess;
}

int	edtPdvCamera::RequestBinY(	unsigned int	value	)
{
    static const char	*	functionName	= "edtPdvCamera::RequestBinY";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, HW ROI binning %u == 0!\n",
        	    		functionName, value );
		return asynError;
	}
	if( m_BinYReq != value )
	{
		m_BinYReq	= value;
		m_fReconfig	= true;
	}
	return asynSuccess;
}

int	edtPdvCamera::SetBinY(	unsigned int	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetBinY";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, ROI bin %u == 0!\n",
        	    		functionName, value );
		return asynError;
	}
	m_BinY		= value;

	// Update the AreaDetector parameter
	setIntegerParam( ADBinY,	m_BinY	);
//	int	status = (asynStatus) UpdateADConfigParams( );
	return asynSuccess;
}

int	edtPdvCamera::RequestTriggerMode(	int	value	)
{
	static const char	*	functionName	= "edtPdvCamera::RequestTriggerMode";
	TriggerMode_t	tyTriggerMode	= static_cast<TriggerMode_t>( value );
	if ( DEBUG_EDT_PDV >= 1 )
		printf(	"%s: Requesting trigger mode %s ...\n",
				functionName, TriggerModeToString( tyTriggerMode ) );
	switch ( tyTriggerMode )
	{
	default:
		m_TriggerModeReq = TRIGMODE_FREERUN;
		break;
	case TRIGMODE_FREERUN:
	case TRIGMODE_EXT:
	case TRIGMODE_PULSE:
		m_TriggerModeReq = tyTriggerMode;
		break;
	}

	// Request a reread of the configuration file
	// as many EDT based cameras use different EDT config files
	// for different trigger modes.
	m_fReopen	= true;

	// Make sure we enable the synchronous data acquisition
	// This jump starts it after boot when autosave updates trigger mode
	if( m_pSyncDataAcquirer != NULL )
		m_pSyncDataAcquirer->SetEnabled();
	return asynSuccess;
}

int	edtPdvCamera::SetTriggerMode(	int	value	)
{
	static const char	*	functionName	= "edtPdvCamera::SetTriggerMode";
	TriggerMode_t	tyTriggerMode	= static_cast<TriggerMode_t>( value );
	if ( tyTriggerMode == m_TriggerMode )
	{
		if ( DEBUG_EDT_PDV >= 1 )
			printf(	"%s: Trigger mode already %s.\n",
					functionName, TriggerModeToString( tyTriggerMode ) );
	}
	else
	{
		if ( DEBUG_EDT_PDV >= 1 )
			printf(	"%s: Setting trigger mode to %s ...\n",
					functionName, TriggerModeToString( tyTriggerMode ) );
		switch ( tyTriggerMode )
		{
		default:
			m_TriggerMode = TRIGMODE_FREERUN;
			break;
		case TRIGMODE_FREERUN:
		case TRIGMODE_EXT:
		case TRIGMODE_PULSE:
			m_TriggerMode = tyTriggerMode;
			break;
		}
	}

	// Update the AreaDetector parameter
	setIntegerParam( ADTriggerMode,	m_TriggerMode	);
	return asynSuccess;
}

int		edtPdvCamera::SetGain( double gain )
{
	int status = asynSuccess;
	
	if ( m_Gain != gain )
	{
		if ( m_pPdvDev != NULL )
			// We could drop support for calling pdv_set_gain, as it just sends a
			// serial command from the cfg file that we can support via protocol and db files
			status = pdv_set_gain( m_pPdvDev, static_cast<int>(gain) );
    	m_Gain	= gain;
	}

	// Update the AreaDetector parameter
//	setDoubleParam( ADGain, m_Gain );
	return status;
}

/** Report status of the driver.
  * Prints details about the driver if details > 0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details If >0 then driver details are printed.
  */
void edtPdvCamera::report( FILE * fp, int details )
{
    fprintf(	fp, "EDT Framegrabber port %s: %s\n",
				this->portName, m_pPdvDev ? "Connected" : "Disconnected" );

	int			connected	= 0;
	pasynManager->isConnected( this->pasynUserSelf, &connected );
	if ( m_pPdvDev && !connected )
	{
		fprintf(	fp, "Warning, Framegrabber port %s thinks it's %s, but asynManager says %s\n",
					portName,
					m_pPdvDev	? "Connected" : "Disconnected",
					connected	? "Connected" : "Disconnected"	);
	}

    if ( details > 0 )
	{
        fprintf( fp, "  Camera name:       %s\n",	m_CameraName.c_str() );
        fprintf( fp, "  Camera model:      %s\n",	m_CameraModel.c_str() );
        fprintf( fp, "  Config model:      %s\n",	m_ModelName.c_str() );
        fprintf( fp, "  Config file:       %s\n",	m_ConfigFile.c_str() );
        fprintf( fp, "  Drv Version:       %s\n",	m_DrvVersion.c_str() );
        fprintf( fp, "  Lib Version:       %s\n",	m_LibVersion.c_str() );
        fprintf( fp, "  Unit:			   %u\n",	m_unit );
        fprintf( fp, "  Channel:		   %u\n",	m_channel );
        fprintf( fp, "  NumBuffers:        %u\n",	m_NumMultiBuf );

        fprintf( fp, "  Sensor bits:       %u\n",	m_ClNumBits );
        fprintf( fp, "  Sensor width:      %zd\n",	m_ClMaxWidth );
        fprintf( fp, "  Sensor height:     %zd\n",	m_ClMaxHeight );
        fprintf( fp, "  Horiz taps:        %d\n",	m_ClHTaps );
        fprintf( fp, "  Vert  taps:        %d\n",	m_ClVTaps );
        fprintf( fp, "  Mode:              %s\n",	EdtModeToString( m_EdtMode ) );
        fprintf( fp, "  Trig Level:        %s\n",	TrigLevelToString( m_trigLevel ) );
        fprintf( fp, "  PDV DebugLevel:    %u\n",	m_EdtDebugLevel );
        fprintf( fp, "  PDV DebugMsgLevel: %u\n",	m_EdtDebugMsgLevel );
		fprintf( fp, "  asyn TraceLevel:   %u\n",	GetTraceLevel() );
        fprintf( fp, "  Frame Count:       %u\n",	m_ArrayCounter );
        fprintf( fp, "  Serial Disabled:   %u\n",   m_SerialDisable );

        fprintf( fp, "\n" );

		/* Call the base class method */
		ADDriver::report( fp, details - 1 );
    }
}

#if 0
asynStatus edtPdvCamera::readFloat64(	asynUser *	pasynUser, epicsFloat64	value )
{
    static const char	*	functionName	= "edtPdvCamera::readFloat64";
    const char			*	reasonName		= "unknownReason";
	getParamName( 0, pasynUser->reason, &reasonName );
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: Reason %d %s, value %lf\n", functionName, pasynUser->reason, reasonName, value );

//    if ( pasynUser->reason == SerAcquireTime )
//	{
//		setDoubleParam( ADAcquireTime, value );
//	}
    if ( pasynUser->reason == SerGain )
	{
		setDoubleParam( ADGain, value );
	}

    callParamCallbacks();

    return asynStatus(0);
}
#endif

// TODO: Don't think we need this function as it's main purpose is to query params from ADCore
asynStatus edtPdvCamera::readInt32(	asynUser *	pasynUser, epicsInt32	* pValueRet )
{
    static const char	*	functionName	= "edtPdvCamera::readInt32";
    const char			*	reasonName		= "unknownReason";
	getParamName( 0, pasynUser->reason, &reasonName );
	if ( DEBUG_EDT_PDV >= 5 )
		printf(	"%s: Reason %d %s\n", functionName, pasynUser->reason, reasonName );
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"%s: Reason %d %s\n", functionName, pasynUser->reason, reasonName );

    if ( pasynUser->reason == SerDisable    ) setIntegerParam( SerDisable, *pValueRet );
//	if ( pasynUser->reason == EdtTrigLevel ) setIntegerParam( EdtTrigLevel, *pValueRet );

	// Call base class
	// asynStatus	status	= asynPortDriver::readInt32( pasynUser, pValueRet );
	asynStatus	status	= ADDriver::readInt32( pasynUser, pValueRet );
    return status;
}

asynStatus edtPdvCamera::writeInt32(	asynUser *	pasynUser, epicsInt32	value )
{
    static const char	*	functionName	= "edtPdvCamera::writeInt32";
    const char			*	reasonName		= "unknownReason";
	getParamName( 0, pasynUser->reason, &reasonName );
	if ( DEBUG_EDT_PDV >= 2 )
		printf(	"%s: Reason %d %s, value %d\n", functionName, pasynUser->reason, reasonName, value );
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"%s: Reason %d %s, value %d\n", functionName, pasynUser->reason, reasonName, value );

	int	status	= asynSuccess;
	//
	// EDT implements these AD parameters
	//
	if (		pasynUser->reason == ADAcquire ) {
		status	= SetAcquireMode( value );
	} else if ( pasynUser->reason == ADBinX		) {
		status	= RequestBinX(	value	);
    } else if ( pasynUser->reason == ADBinY		) {
		status	= RequestBinY(	value	);
    } else if ( pasynUser->reason == ADMinX		) {
		status	= RequestMinX(	value	);
    } else if ( pasynUser->reason == ADMinY		) {
		status	= RequestMinY(	value	);
    } else if ( pasynUser->reason == ADSizeX		) {
		status	= RequestSizeX(	value	);
    } else if ( pasynUser->reason == ADSizeY		) {
		status	= RequestSizeY(	value	);
    } else if ( pasynUser->reason == ADTriggerMode	) {
		status	= RequestTriggerMode(	value );
    } else if ( pasynUser->reason == ADImageMode) {
		// Get current imageMode from ADDriver
		int			imageMode;
		status	= getIntegerParam( ADImageMode,	&imageMode	);

		if ( imageMode != value )
		{
			int			numImages;
			getIntegerParam( ADNumImages,	&numImages	);

			// Image Capture mode changed
			imageMode = value;
			status	= setIntegerParam( ADImageMode,	value );

			// Update acquire count
			switch( imageMode )
			{
			case ADImageSingle:
				m_acquireCount = 1;
				break;
			case ADImageMultiple:
				m_acquireCount = numImages;
				break;
			case ADImageContinuous:
				m_acquireCount = -1;
				break;
			}
			if ( DEBUG_EDT_PDV >= 1 )
				printf(	"%s: Setting acquire count to %d\n", 
						functionName, m_acquireCount );
		}

	} else if ( pasynUser->reason == ADNumImages ) {
		// Get prior values
		int			numImages;
		status	= getIntegerParam( ADNumImages,	&numImages	);

		if ( numImages != value )
		{
			// Image capture count changed
			status	= setIntegerParam( ADNumImages,	value );

			// Update acquire count
			int			imageMode;
			getIntegerParam( ADImageMode,	&imageMode	);
			switch( imageMode )
			{
			case ADImageSingle:
				m_acquireCount = 1;
				break;
			case ADImageMultiple:
				m_acquireCount = value;
				break;
			case ADImageContinuous:
				m_acquireCount = -1;
				break;
			}
			if ( DEBUG_EDT_PDV >= 1 )
				printf(	"%s: Updating acquire count to %d\n", 
						functionName, m_acquireCount );
		}

	//
	// EDT implements these ND parameters
	//
	} else if ( pasynUser->reason == NDArrayCounter	) {
		status = SetArrayCounter( value );
	//
	// No more overrides of ND and AD parameters
	// Any below FIRST_EDT_PARAM get handled by base class ADDriver
	//
    } else if (	pasynUser->reason < FIRST_EDT_PARAM ) {
		status	= ADDriver::writeInt32( pasynUser, value );
	//
	// Start of EDT specific parameters
	//
    } else if ( pasynUser->reason == EdtTrigLevel		) {
		status = setIntegerParam( EdtTrigLevel, value );
    } else if ( pasynUser->reason == EdtHwHRoi			) {
		m_HwHRoi = value;
		status	 = 0;
    } else if ( pasynUser->reason == EdtHwVRoi			) {
		m_HwVRoi = value;
		status	 = 0;
    } else if ( pasynUser->reason == EdtReCfgCnt		) {
		m_ReCfgCnt = value;
		status	 = 0;
	} else if ( pasynUser->reason == SerBinX			) {
		status = SetBinX(	value );
	} else if ( pasynUser->reason == SerBinY			) {
		status = SetBinY(	value );
    } else if ( pasynUser->reason == SerTriggerMode	) {
		status = SetTriggerMode(	value );
    } else if ( pasynUser->reason == SerMinX			) {
		status = SetMinX(	value );
    } else if ( pasynUser->reason == SerMinY			) {
		status = SetMinY(	value );
    } else if ( pasynUser->reason == SerSizeX			) {
		status = SetSizeX(	value );
    } else if ( pasynUser->reason == SerSizeY			) {
		status = SetSizeY(	value );
    } else if ( pasynUser->reason == EdtDebug			) {
		status = SetEdtDebugLevel(		value );
    } else if ( pasynUser->reason == EdtDebugMsg		) {
		status = SetEdtDebugMsgLevel(	value );
 	} else if ( pasynUser->reason == SerDisable      ) {
        status = SetSerDisable(value);
    }

    callParamCallbacks( 0, 0 );

	/* Report any errors */
	if (status)
		asynPrint(	pasynUser, ASYN_TRACE_ERROR,
					"%s:writeInt32 error, status=%d function=%d %s, value=%d\n",
					functionName, status, pasynUser->reason, reasonName, value);
	else 
		asynPrint(	pasynUser, ASYN_TRACEIO_DRIVER,
					"%s:writeInt32: function=%d %s, value=%d\n",
					functionName, pasynUser->reason, reasonName, value);

    return (asynStatus) status;
}

asynStatus edtPdvCamera::writeFloat64(	asynUser *	pasynUser, epicsFloat64	value )
{
    static const char	*	functionName	= "edtPdvCamera::writeFloat64";
    const char			*	reasonName		= "unknownReason";
	getParamName( 0, pasynUser->reason, &reasonName );
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: Reason %d %s, value %lf\n", functionName, pasynUser->reason, reasonName, value );

	int		status	= asynSuccess;
    if (		pasynUser->reason < FIRST_EDT_PARAM ) {
		status	= ADDriver::writeFloat64( pasynUser, value );
    } else if ( pasynUser->reason == SerAcquireTime ) {
		status	= setDoubleParam( ADAcquireTime, value );
	} else if ( pasynUser->reason == ADGain) {
		status	= SetGain(	value	);
	} else if ( pasynUser->reason == SerGain ) {
		status	= setDoubleParam( ADGain, value );
	}

    callParamCallbacks();

    return (asynStatus) status;
}


int		edtPdvCamera::traceVPrint( const char	*	pFormat, va_list pvar )
{
	if ( DEBUG_EDT_PDV >= 4 )
	{
		static const char	*	functionName = "edtPdvCamera:traceVPrint";
		printf(	"%s Format: %s\n", functionName, pFormat );
	}
	return pasynTrace->vprint( this->pasynUserSelf, GetTraceLevel(), pFormat, pvar );
}

unsigned int edtPdvCamera::GetTraceLevel()
{
//	return pasynTrace->getTraceMask( this->pasynUserSelf );
	return DEBUG_EDT_PDV;
}


extern "C" int resetImageTiming( )
{
//	imageCaptureCumTicks	= 0LL;
//	imageCaptureCount		= 0L;
	return 0;
}

extern "C" int showImageTiming( )
{
#if 0
	double	cumDur	= HiResTicksToSeconds( imageCaptureCumTicks );
	printf(	"imageCaptureCumTicks = %llu\n",	imageCaptureCumTicks	);
	printf(	"imageCaptureCount    = %lu\n",		imageCaptureCount	);
	printf(	"imageCaptureCumDur   = %4.3f sec\n",	cumDur	);
	if ( imageCaptureCount != 0 )
		printf(	"imageCaptureAvgDur   = %4.3f ms\n\n", 1000.0 * cumDur / imageCaptureCount	);
#endif

	return 0;
}

extern "C" int
edtPdvConfig(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	modelName,
	const char	*	edtMode	)
{
    if (  cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf( "NULL or zero length camera name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }
    if (  modelName == NULL || strlen(modelName) == 0 )
    {
        errlogPrintf( "NULL or zero length config name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }
    if (  edtMode == NULL || strlen(edtMode) == 0 )
    {
        errlogPrintf( "NULL or zero length EDT mode.\nUsage: edtPdvConfig(name,unit,chan,config,mode)\n");
        return  -1;
    }
    if ( edtPdvCamera::CreateCamera( cameraName, unit, channel, modelName, edtMode ) != 0 )
    {
        errlogPrintf( "edtPdvConfig failed for camera %s, config %s, mode %s!\n", cameraName, modelName, edtMode );
		if ( DEBUG_EDT_PDV >= 4 )
        	epicsThreadSuspendSelf();
        return -1;
    }
    return 0;
}

extern "C" int
edtPdvConfigFull(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	modelName,
	const char	*	edtMode,
	int				maxBuffers,				// 0 = unlimited
	size_t			maxMemory,				// 0 = unlimited
	int				priority,				// 0 = default 50, high is 90
	int				stackSize			)	// 0 = default 1 MB
{
    if (  cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf( "NULL or zero length camera name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }
    if (  modelName == NULL || strlen(modelName) == 0 )
    {
        errlogPrintf( "NULL or zero length config name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }
    if (  edtMode == NULL || strlen(edtMode) == 0 )
    {
        errlogPrintf( "NULL or zero length EDT mode.\nUsage: edtPdvConfig(name,unit,chan,config,mode)\n");
        return  -1;
    }

    if ( edtPdvCamera::CreateCamera( cameraName, unit, channel, modelName, edtMode ) != 0 )
    {
        errlogPrintf( "edtPdvConfig failed for camera %s!\n", cameraName );
		if ( DEBUG_EDT_PDV >= 4 )
        	epicsThreadSuspendSelf();
        return -1;
    }
    return 0;
}

// Register function:
//		int resetImageTiming( void )
static const iocshFuncDef   resetImageTimingFuncDef	= { "resetImageTiming", 0, NULL };
static int  resetImageTimingCallFunc( const iocshArgBuf * args )
{
	return static_cast<int>( resetImageTiming( ) );
}
void resetImageTimingRegister(void)
{
	iocshRegister( &resetImageTimingFuncDef, reinterpret_cast<iocshCallFunc>(resetImageTimingCallFunc) );
}

//epicsRegisterFunction( resetImageTiming );
epicsExportRegistrar( resetImageTimingRegister );

// Register function:
//		int showImageTiming( void )
static const iocshFuncDef   showImageTimingFuncDef	= { "showImageTiming", 0, NULL };
static int  showImageTimingCallFunc( const iocshArgBuf * args )
{
	return static_cast<int>( showImageTiming( ) );
}
void showImageTimingRegister(void)
{
	iocshRegister( &showImageTimingFuncDef, reinterpret_cast<iocshCallFunc>(showImageTimingCallFunc) );
}

//epicsRegisterFunction( showImageTiming );
epicsExportRegistrar( showImageTimingRegister );

// Register Function:
//	int edtPdvConfig( const char * cameraName, int unit, int channel, const char * modelName )
static const iocshArg		edtPdvConfigArg0	= { "name",			iocshArgString };
static const iocshArg		edtPdvConfigArg1	= { "unit",			iocshArgInt };
static const iocshArg		edtPdvConfigArg2	= { "channel",		iocshArgInt };
static const iocshArg		edtPdvConfigArg3	= { "modelName",	iocshArgString };
static const iocshArg		edtPdvConfigArg4	= { "edtMode",		iocshArgString };
static const iocshArg	*	edtPdvConfigArgs[5]	=
{
	&edtPdvConfigArg0, &edtPdvConfigArg1, &edtPdvConfigArg2, &edtPdvConfigArg3, &edtPdvConfigArg4
};
static const iocshFuncDef   edtPdvConfigFuncDef	= { "edtPdvConfig", 5, edtPdvConfigArgs };
static int  edtPdvConfigCallFunc( const iocshArgBuf * args )
{
    return edtPdvConfig( args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].sval );
}
void edtPdvConfigRegister(void)
{
	iocshRegister( &edtPdvConfigFuncDef, reinterpret_cast<iocshCallFunc>(edtPdvConfigCallFunc) );
}

// Register Function:
//	int edtPdvConfigFull( const char * cameraName, int unit, int channel, const char * modelName, int, size_t, int, int  )
static const iocshArg		edtPdvConfigFullArg0	= { "name",			iocshArgString };
static const iocshArg		edtPdvConfigFullArg1	= { "unit",			iocshArgInt };
static const iocshArg		edtPdvConfigFullArg2	= { "channel",		iocshArgInt };
static const iocshArg		edtPdvConfigFullArg3	= { "cfgFile",		iocshArgString };
static const iocshArg		edtPdvConfigFullArg4	= { "edtMode",		iocshArgString };
static const iocshArg		edtPdvConfigFullArg5	= { "maxBuffers",	iocshArgInt };
static const iocshArg		edtPdvConfigFullArg6	= { "maxMemory",	iocshArgInt };
static const iocshArg		edtPdvConfigFullArg7	= { "priority",		iocshArgInt };
static const iocshArg		edtPdvConfigFullArg8	= { "stackSize",	iocshArgInt };
// There has to be a better way to handle triggerPV, delayPV, and syncPV
//static const iocshArg		edtPdvConfigFullArgX	= { "triggerPV",	iocshArgString };
//static const iocshArg		edtPdvConfigFullArgX	= { "delayPV",		iocshArgString };
//static const iocshArg		edtPdvConfigFullArgX	= { "syncPV",		iocshArgString };
static const iocshArg	*	edtPdvConfigFullArgs[9]	=
{
	&edtPdvConfigFullArg0, &edtPdvConfigFullArg1, &edtPdvConfigFullArg2, &edtPdvConfigFullArg3,
	&edtPdvConfigFullArg4, &edtPdvConfigFullArg5, &edtPdvConfigFullArg6, &edtPdvConfigFullArg7,
	&edtPdvConfigFullArg8
};
static const iocshFuncDef   edtPdvConfigFullFuncDef	= { "edtPdvConfigFull", 9, edtPdvConfigFullArgs };
static int  edtPdvConfigFullCallFunc( const iocshArgBuf * args )
{
    return edtPdvConfigFull(
		args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].sval,
		args[5].ival, args[6].ival, args[7].ival, args[8].ival	);
}
void edtPdvConfigFullRegister(void)
{
	iocshRegister( &edtPdvConfigFullFuncDef, reinterpret_cast<iocshCallFunc>(edtPdvConfigFullCallFunc) );
}

// Register Function:
//	int edt_set_verbosity( int level )
static const iocshArg		edt_set_verbosityArg0		= { "level",	iocshArgInt };
static const iocshArg	*	edt_set_verbosityArgs[1]	= { &edt_set_verbosityArg0 };
static const iocshFuncDef   edt_set_verbosityFuncDef	= { "edt_set_verbosity", 1, edt_set_verbosityArgs };
static int  edt_set_verbosityCallFunc( const iocshArgBuf * args )
{
	edt_set_verbosity( args[0].ival );
	return 0;
}

void edt_set_verbosityRegister(void)
{
	iocshRegister( &edt_set_verbosityFuncDef, reinterpret_cast<iocshCallFunc>(edt_set_verbosityCallFunc) );
}

extern "C"
{
	epicsExportRegistrar( edtPdvConfigRegister );
	epicsExportRegistrar( edtPdvConfigFullRegister );
	epicsExportRegistrar( edt_set_verbosityRegister );
	epicsExportAddress( int, DEBUG_EDT_PDV );
}
