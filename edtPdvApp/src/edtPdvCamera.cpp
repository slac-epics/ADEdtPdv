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

//	PCDS headers
#include "evrTime.h"
#include "HiResTime.h"
#include "ContextTimer.h"
//#include "camRecord.h"
//#include "Image.h"
//#include "timesync.h"

using namespace		std;

static	const char *		driverName	= "EdtPdv";

// Diagnostic timers
// View and reset via iocsh cmds.
// From iocsh, type: help *Context*

int		EDT_PDV_DEBUG	= 2;

//	t_HiResTime		imageCaptureCumTicks	= 0LL;
//	unsigned long	imageCaptureCount		= 0L;

///	Camera map - Stores ptr to all edtPdvCamera instances indexed by name
map<string, edtPdvCamera *>	edtPdvCamera::ms_cameraMap;

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
//
// edtPdvCamera functions
//

///	edtPdvCamera constructor
edtPdvCamera::edtPdvCamera(
	const char			*	cameraName,
	int						unit,
	int						channel,
	const char			*	modelName,
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
		m_fAcquireMode(		FALSE			    ),
		m_fExitApp(			FALSE			    ),
		m_fReconfig(		true			    ),
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
		m_width(			0					),
		m_height(			0					),
		m_numOfBits(		0					),
		m_imageSize(		0					),
		m_dmaSize(			0					),
		m_tyInterlace(	PDV_INTLV_IN_PDV_LIB	),
		m_trigLevel(		0					),
		m_EdtDebugLevel(	1					),
	//	m_EdtDebugMsgLevel(	0xFFF				),
		m_EdtDebugMsgLevel(	0x000				),
		m_EdtHSkip(			0					),
		m_EdtHSize(			0					),
		m_EdtVSkip(			0					),
		m_EdtVSize(			0					),
		m_TriggerMode(		TRIGMODE_FREERUN	),
		m_BinX(				1					),
		m_BinY(				1					),
		m_MinX(				0					),
		m_MinY(				0					),
		m_SizeX(			0					),
		m_SizeY(			0					),
		// Do we need ADBase ROI values here?  m_BinX, m_BinY, m_MinX, m_SizeY, ...
		m_Gain(				0					),
		
		m_ArrayCounter(		0					),
		m_acquireCount(		0					),
		m_fiducial(			0					),
		m_reconfigLock(		NULL				),

		m_ioscan(			NULL				),
		m_pAsynSerial(		NULL				),
		m_ReAcquireTimer(	"ReAcquire"			),
		m_ReArmTimer(		"ReArm"				),
		m_ProcessImageTimer("ProcessImage"		)
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

    // Configure an asyn port for serial commands
	unsigned int		serPriority		= 0;
	int					autoConnect		= 0;
//	int					noProcessEos	= 0;
//    m_ttyPort = drvAsynEdtPdvSerialPortConfigure(	m_SerialPort.c_str(), serPriority,
//													autoConnect,	noProcessEos );
//    if ( m_ttyPort == NULL )
//	{
//       printf( "Error: Unable to configure asyn serial port for %s.\n", m_SerialPort.c_str() );
//      asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW, 
//					"asynPrint "
//					"%s %s: ERROR, drvAsynEdtPdvSerialPortConfigure failed!\n",
//					driverName,		functionName );
//    }
    m_pAsynSerial = new asynEdtPdvSerial(	m_SerialPort.c_str(), serPriority,	autoConnect	);

    // NOTE: If we needed to, could we get the ttyController ptr via findAsynPortDriver()?
	//	ttyController	*	tty	= (ttyController *) findAsynPortDriver( m_SerialPort.c_str() );

	// Create EDT parameters shared by all EDT based cameras
	// This group gives access to PDV library values of interest
	createParam( EdtClassString,		asynParamOctet,		&EdtClass		);
	createParam( EdtDebugString,		asynParamInt32,		&EdtDebug		);
	createParam( EdtDebugMsgString,		asynParamInt32,		&EdtDebugMsg	);
	createParam( EdtDrvVersionString,	asynParamOctet,		&EdtDrvVersion	);
	createParam( EdtHSkipString,		asynParamInt32,		&EdtHSkip		);
	createParam( EdtHSizeString,		asynParamInt32,		&EdtHSize		);
	createParam( EdtVSkipString,		asynParamInt32,		&EdtVSkip		);
	createParam( EdtVSizeString,		asynParamInt32,		&EdtVSize		);
	createParam( EdtLibVersionString,	asynParamOctet,		&EdtLibVersion	);
	createParam( EdtMultiBufString,		asynParamInt32,		&EdtMultiBuf	);
	createParam( EdtInfoString,			asynParamOctet,		&EdtInfo		);
	createParam( EdtTrigLevelString,	asynParamInt32,		&EdtTrigLevel	);

	// This group provides a way to have serial readbacks get reflected in
	// their ADBase class equivalents, for example
	// SerAcquireTime	=>	ADAcquireTime 
	createParam( EdtSerAcquireTimeString,	asynParamFloat64,	&SerAcquireTime	);
	createParam( EdtSerMinXString,			asynParamInt32,		&SerMinX		);
	createParam( EdtSerMinYString,			asynParamInt32,		&SerMinY		);
	createParam( EdtSerSizeXString,			asynParamInt32,		&SerSizeX		);
	createParam( EdtSerSizeYString,			asynParamInt32,		&SerSizeY		);
	createParam( EdtSerTriggerModeString,	asynParamInt32,		&SerTriggerMode	);

	// Get the EDT PDV debug levels and multibuf number (should be in autosave)
	getIntegerParam( EdtDebug,		&m_EdtDebugLevel	);
	getIntegerParam( EdtDebugMsg,	&m_EdtDebugMsgLevel	);
	getIntegerParam( EdtMultiBuf,	&m_NumMultiBuf	);

	// Create an EDT Sync object
	// TODO: Should we just make this a member object?
	printf( "%s: Creating syncDataAcq object in thread %s\n", functionName, epicsThreadGetNameSelf() );
	syncDataAcq<edtPdvCamera, edtImage>		*	pSyncDataAcquirer	= NULL;
	pSyncDataAcquirer	= new syncDataAcq<edtPdvCamera, edtImage>( *this, m_CameraName );

	// Make it available as a member variable
	m_pSyncDataAcquirer	= pSyncDataAcquirer;

#if 0
    // Install exit hook for clean shutdown
    epicsAtExit( (EPICSTHREADFUNC)edtPdvCamera::ExitHook, (void *) this );
#endif
}

///	edtPdvCamera Destructor
edtPdvCamera::~edtPdvCamera( )
{
	disconnect( this->pasynUserSelf );

	// Cleanup for shutdown
	delete m_pSyncDataAcquirer;
	m_pSyncDataAcquirer	= NULL;

	epicsMutexDestroy(	m_reconfigLock );
}


int edtPdvCamera::CreateCamera( const char * cameraName, int unit, int channel, const char * modelName	)
{
    static const char	*	functionName = "edtPdvCamera::CreateCamera";

    /* Parameters check */
    if (  cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf(	"%s %s: ERROR, NULL or zero length camera name. Check parameters to edtPdvConfig()!\n",
            			driverName, functionName );
        return  -1;
    }

    if ( edtPdvCamera::CameraFindByName(cameraName) != NULL )
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

    if ( EDT_PDV_DEBUG )
        cout << "Creating edtPdvCamera: " << string(cameraName) << endl;
    edtPdvCamera	* pCamera = new edtPdvCamera( cameraName, unit, channel, modelName );
    assert( pCamera != NULL );

    int	status	= pCamera->ConnectCamera( );
	if ( status != 0 )
        errlogPrintf( "edtPdvConfig failed for camera %s!\n", cameraName );

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
	if ( EDT_PDV_DEBUG )
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
		cout	<< "\t\tResolution: "	<< m_width
				<<	" * "				<< m_height
				<< ", "					<< m_numOfBits
				<< "bits" << endl;
	}
	if ( level >= 2 )
	{
		int     stat;
    	stat = edt_reg_read( m_pPdvDev, PDV_STAT );
		cout	<< "\t\tStatus: 0x%X"	<< stat
				<< endl;
	}
    return 0;
}

#if 0
void edtPdvCamera::ExitHook(void)
{
	exit_loop = 1;
	this->lock();
	m_acquireCount = 0;
	setIntegerParam(ADAcquire, 0);
	//	edt_abort_dma( m_pPdvDev );
	pdv_timeout_restart( m_pPdvDev, 0 );
	epicsThreadSleep(2.);	// Exiting
	pdv_close(m_pPdvDev);
	m_pPdvDev	= NULL;
	this->unlock();
}
#endif

///	Connects driver to device
asynStatus edtPdvCamera::ConnectCamera( )
{
    static const char	*	functionName	= "edtPdvCamera::ConnectCamera";
    asynStatus				status			= asynSuccess;

	if ( EDT_PDV_DEBUG >= 1 )
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );

	// Initialize (or re-initialize) camera
	m_fReconfig	= true;
	Reconfigure( );

	if ( m_pPdvDev == NULL || m_fReconfig )
	{
		printf( "%s: %s failed to initialize PdvDev camera!\n", functionName, m_CameraName.c_str() );
        return asynError;
	}
	if ( m_pAsynSerial == NULL )
	{
		printf( "%s: %s failed to initialize PdvDev camera serial port!\n", functionName, m_CameraName.c_str() );
        return asynError;
	}

	if ( m_pAsynSerial->pdvDevConnected( m_pPdvDev ) != asynSuccess )
        return asynError;

	UpdateStatus( ADStatusIdle );

	// Write the configuration parameters to the AreaDetector parameter PV's UpdateADConfigParams( );
	status = (asynStatus) UpdateADConfigParams( );
	if ( status != asynSuccess )
        asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
					"asynPrint "
					"%s %s: error calling UpdateADConfigParams, error=%s\n",
					driverName, functionName, this->pasynUserSelf->errorMessage );

	if ( EDT_PDV_DEBUG >= 2 )
	{
		printf( "Camera %s: Image size %zu x %zu pixels, %u bits/pixel\n",
				m_CameraName.c_str(), m_width, m_height, m_numOfBits );
	}

    return status;
}


//	Disconnects driver from device
asynStatus edtPdvCamera::DisconnectCamera( )
{
    static const char	*	functionName	= "edtPdvCamera::DisconnectCamera";
    int						status			= asynSuccess;

	if ( EDT_PDV_DEBUG >= 1 )
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

	// TODO: Do we need to use a lock to protect against crashes here?
	m_pAsynSerial->pdvDevDisconnected( NULL );

    if ( m_pPdvDev )
	{
		// TODO: Halt any image acquires in progress

		// Abort DMA requests
		//	edt_abort_dma( m_pPdvDev );
		pdv_timeout_restart( m_pPdvDev, 0 );

		// Note: pdv_close(), not edt_close()!
		pdv_close( m_pPdvDev );
		m_pPdvDev	= NULL;
	}
 
    return static_cast<asynStatus>( status );
    //	return asynSuccess;
}

/// Overriding asynPortDriver::connect
///	Connects driver to device
asynStatus edtPdvCamera::connect( asynUser *	pasynUser )
{
    static const char	*	functionName	= "edtPdvCamera::connect";

	if ( EDT_PDV_DEBUG >= 1 )
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
	if ( EDT_PDV_DEBUG >= 1 )
		printf(	"%s %s: Camera %s 0 connected!\n", 
				driverName, functionName, m_CameraName.c_str() );
    asynPrint(	pasynUser, ASYN_TRACE_FLOW, 
				"asynPrint "
				"%s %s: Camera %s 0 connected!\n", 
				driverName, functionName, m_CameraName.c_str() );
    return asynSuccess;
}

/// Overriding asynPortDriver::disconnect
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
				"%s %s: Camera disconnected %s\n", 
				driverName, functionName, m_CameraName.c_str() );
	if ( EDT_PDV_DEBUG >= 1 )
		printf(	"%s %s: Camera %s 0 disconnected!\n", 
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

	if ( EDT_PDV_DEBUG >= 1 )
	{
		printf( "%s: %s in thread %s ...\n", functionName, m_CameraName.c_str(), epicsThreadGetNameSelf() );
	}

	int		status	= 0;
	epicsMutexLock(		m_reconfigLock );
	if ( m_fReconfig )
	{
		UpdateStatus( ADStatusInitializing );

		// Clear reconfig flag up front so it can be set again by another thread if needed
		m_fReconfig = false;

		// Close old PdvDev is needed
		if ( m_pPdvDev )
		{
			if ( EDT_PDV_DEBUG >= 1 )
				printf( "%s: %s Closing old PdvDev prior to reconfig\n", functionName, m_CameraName.c_str() );
			pdv_close( m_pPdvDev );
			m_pPdvDev	= NULL;
		}

		status	= edtPdvCamera::_Reconfigure( );
		if ( status != 0 )
		{
			// Reconfigure failed, request another
			m_fReconfig	= true;
		}
		else
		{
			UpdateStatus( ADStatusIdle );
		}
	}
	epicsMutexUnlock(	m_reconfigLock );

	if ( status != 0 )
	{
        asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s %s: Reconfigure error! errorMsg=%s\n",
					driverName, functionName, this->pasynUserSelf->errorMessage );
	}
	else if ( m_fReconfig )
	{
        asynPrint(	this->pasynUserSelf, ASYN_TRACE_ERROR,
					"%s %s: Reconfigure succeeded, but Reconfig flag has already been set again!\n",
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

	// Set the EDT PDV debug levels
	pdv_setdebug(		NULL,				 		m_EdtDebugLevel	);
	edt_msg_set_level(	edt_msg_default_handle(),	m_EdtDebugMsgLevel	);

	// Open the camera channel
	// Note: Do not use edt_open_channel() here!
	// Although pdv_open_channel() and edt_open_channel() both return ptrs
	// to the same underlying C structure, they behave differently in regards
	// to what memory they allocate and free and how some of the fields
	// are initialized.
    m_pPdvDev = pdv_open_channel( "pdv", m_unit, m_channel );
    if ( m_pPdvDev == NULL )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW, 
					"%s %s: ERROR, Unable to open camera for EDT card %u, channel %u\n",
					driverName,		functionName, m_unit, m_channel );
        return -1;
    }

	// Read the config file
    Edtinfo			edtinfo;
	Dependent	*	pDD;			// Pointer to PDV dependent information.
    pDD = pdv_alloc_dependent();
    if ( pDD == NULL )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW, 
					"asynPrint "
					"%s %s: ERROR, Cannot allocate PDV Dependent structure!\n",
					driverName,		functionName );
        return -1;
    }

	//	Update m_ConfigFile based on TriggerMode, changing the current
	//	Internal/External choices to FreeRun (f.cfg), Trigger (t.cfg), or Pulse (p.cfg)
	m_ConfigFile = "db/";
	m_ConfigFile += m_ModelName;
	switch( m_TriggerMode )
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

	if ( EDT_PDV_DEBUG >= 1 )
	{
		printf( "%s: %s Reconfiguring from %s ...\n", functionName, m_CameraName.c_str(), m_ConfigFile.c_str() );
	}

	// Read the config file
    if ( pdv_readcfg( m_ConfigFile.c_str(), pDD, &edtinfo ) )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW, 
					"asynPrint "
					"%s %s: ERROR, Invalid camera config file %s!\n",
					driverName,		functionName,	m_ConfigFile.c_str() );
        return -1;
    }

	// Initialize the camera
	// m_pPdvDev should already have it's own Dependent data in dd_p,
	// but because of how pdv_initcam() works, we must supply a different one
	// with the new configuration information.
	assert( m_pPdvDev->dd_p != NULL );
	assert( m_pPdvDev->dd_p != pDD  );
    if ( pdv_initcam( m_pPdvDev, pDD, m_unit, &edtinfo, m_ConfigFile.c_str(), NULL, 1 ) )
	{
        asynPrint(	this->pasynUserSelf,	ASYN_TRACE_FLOW,
					"asynPrint "
					"%s %s: ERROR, pdv_initcam failed!\n",
					driverName,		functionName	);
        return -1;
    }

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

	free( pDD );

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

	// Camera is reconfigured and ready to use!
	m_fReconfig		= false;
    m_CameraClass	= pdv_get_camera_class(	m_pPdvDev );
    m_CameraModel	= pdv_get_camera_model(	m_pPdvDev );
	m_CameraInfo	= pdv_get_camera_info(	m_pPdvDev );
    m_width			= pdv_get_width(		m_pPdvDev );
    m_height		= pdv_get_height(		m_pPdvDev );
    m_numOfBits		= pdv_get_depth(		m_pPdvDev );
    m_EdtHSkip		= 0;
    m_EdtHSize		= m_width;
    m_EdtVSkip		= 0;
    m_EdtVSize		= m_height;

	// Update AreaDetector parameters
	// For many cameras, this will trigger serial commands
	// to update camera or EDT ROI settings
	setIntegerParam( ADBinX,		m_BinX	);
	setIntegerParam( ADBinY,		m_BinY	);
	setIntegerParam( ADMinX,		m_MinX	);
	setIntegerParam( ADMinY,		m_MinY	);
	setIntegerParam( ADSizeX,		m_SizeX );
	setIntegerParam( ADSizeY,		m_SizeY );
	setIntegerParam( NDArraySizeX,	m_SizeX	);
	setIntegerParam( NDArraySizeY,	m_SizeY	);

	// Diagnostics
	if ( EDT_PDV_DEBUG >= 1 )
		printf(	"%s %s: Camera %s ready on card %u, ch %u, %zu x %zu pixels, %u bits/pixel\n",
				driverName, functionName, m_CameraName.c_str(),
				m_unit, m_channel, GetSizeX(), GetSizeY(), m_numOfBits );
	asynPrint(	this->pasynUserSelf,	ASYN_TRACEIO_DRIVER,
				"%s %s: Camera %s ready on card %u, ch %u, %zu x %zu pixels, %u bits/pixel\n",
				driverName, functionName, m_CameraName.c_str(),
				m_unit, m_channel, GetSizeX(), GetSizeY(), m_numOfBits );
    return 0;
}


int edtPdvCamera::UpdateADConfigParams( )
{
    static const char	*	functionName	= "edtPdvCamera::UpdateADConfigParams";
	if ( EDT_PDV_DEBUG >= 2 )
		printf( "%s: %s ...\n", functionName, m_CameraName.c_str() );

	if ( m_pPdvDev == NULL )
	{
		printf( "%s Error: Camera %s not connected!\n", functionName, m_CameraName.c_str() );
		return -1;
	}

	// Fetch the camera manufacturer and model and write them to ADBase parameters
    m_CameraClass	= pdv_get_camera_class(	m_pPdvDev );
    m_CameraModel	= pdv_get_camera_model(	m_pPdvDev );
	setStringParam( ADManufacturer, m_CameraClass.c_str() );
	setStringParam( ADModel,		m_CameraModel.c_str() );

	// Fetch the full image geometry parameters and write them to ADBase parameters
    m_width			= pdv_get_width(	m_pPdvDev );
	setIntegerParam( ADMaxSizeX,		m_width		);
    m_height		= pdv_get_height(	m_pPdvDev );
	setIntegerParam( ADMaxSizeY,		m_height	);
    m_numOfBits		= pdv_get_depth(	m_pPdvDev );
	if ( m_numOfBits <= 8 )
		setIntegerParam( NDDataType,	NDUInt8	);
	else if ( m_numOfBits <= 16 )
		setIntegerParam( NDDataType,	NDUInt16	);
	setIntegerParam( NDArrayCallbacks,	1	);

	// TODO: Move these to SetSizeX(), ...
	setIntegerParam( NDArraySizeX,		m_width		);	// TODO: Fix for ROI
	setIntegerParam( NDArraySizeY,		m_height	);	// TODO: Fix for ROI
	m_imageSize		= m_width * m_height;
	setIntegerParam( NDArraySize,		m_imageSize );

	//	TODO: Do we need these?
    //	m_Gain			= pdv_get_gain(			m_pPdvDev );
    //	m_dmaSize		= pdv_get_dmasize(		m_pPdvDev );

	// Update EDT PDV asyn parameters
    m_CameraClass	= pdv_get_camera_class(	m_pPdvDev );
    setStringParam( EdtClass, m_CameraClass.c_str()	);
    m_CameraInfo	= pdv_get_camera_info(	m_pPdvDev );
    setStringParam( EdtInfo,	m_CameraInfo.c_str()	);

    char		buf[MAX_STRING_SIZE];
    if ( edt_get_driver_version(	m_pPdvDev, buf, MAX_STRING_SIZE ) )
	{
        m_DrvVersion = buf;
		setStringParam( EdtDrvVersion, m_DrvVersion.c_str()	);
	}
    if ( edt_get_library_version(	m_pPdvDev, buf, MAX_STRING_SIZE ) )
	{
        m_LibVersion = buf;
		setStringParam( EdtLibVersion, m_LibVersion.c_str()	);
	}

	// Update PDV library Debug parameters
	setIntegerParam(	EdtDebug,		m_EdtDebugLevel	);
	pdv_setdebug(		m_pPdvDev, 		m_EdtDebugLevel	);
	setIntegerParam(	EdtDebugMsg,	m_EdtDebugMsgLevel	);
	edt_msg_set_level(	edt_msg_default_handle(),	m_EdtDebugMsgLevel	);

//	m_EdtDebugLevel	= pdv_debug_level(	m_pPdvDev );
//	pdv_setdebug(		m_pPdvDev, 		m_EdtDebugLevel	);
//	getIntegerParam(	EdtDebug,		&m_EdtDebugLevel	);
//	getIntegerParam(	EdtDebug,		&m_EdtDebugLevel	);
//	m_EdtDebugMsgLevel	= edt_msg_default_level( );
//	edt_msg_set_level(	edt_msg_default_handle(),	m_EdtDebugMsgLevel	);
//	getIntegerParam(	EdtDebugMsg,	&m_EdtDebugMsgLevel	);
//	getIntegerParam(	EdtDebugMsg,	&m_EdtDebugMsgLevel	);
    return 0;
}


asynStatus	edtPdvCamera::UpdateStatus( int	newStatus	)
{
	CONTEXT_TIMER( "edtPdvCamera-UpdateStatus" );
	//	Context timer shows these next two calls take about 20us
	asynStatus		status	= setIntegerParam( ADStatus, ADStatusIdle );
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
		if ( m_fAcquireMode && EDT_PDV_DEBUG >= 3 )
			printf(	"%s: Stopping acquisition on camera %s\n", 
					functionName, m_CameraName.c_str() );
		// Stop acquisition
		m_fAcquireMode = false;
		m_acquireCount = 0;
		// TODO: Is this the right place to call pdv_timeout_restart?
		pdv_timeout_restart( m_pPdvDev, 0 );
		//	edt_abort_dma( m_pPdvDev ); done by pdv_timeout_restart

		UpdateStatus( ADStatusIdle	);
	}
	else
	{
		if ( EDT_PDV_DEBUG >= 3 )
			printf(	"%s: Starting acquisition on camera %s\n", 
					functionName, m_CameraName.c_str() );
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
	}
	setIntegerParam( ADAcquire, m_fAcquireMode );
	return asynSuccess;
}

int edtPdvCamera::CameraStart( )
{
    static const char	*	functionName = "edtPdvCamera::CameraStart";
	CONTEXT_TIMER( "CameraStart" );

	// Cleanup any pending transfers
	// (Helps keep synchronized images if app is restarted)
    pdv_timeout_restart( m_pPdvDev, FALSE );

	double		cameraStartDelay	= 0.1;
	if ( cameraStartDelay > 0.0 )
	{
		if ( EDT_PDV_DEBUG >= 2 )
			printf(	"%s: Delaying %f sec\n", functionName, cameraStartDelay );
		epicsThreadSleep( cameraStartDelay );
	}

	if ( EDT_PDV_DEBUG >= 2 )
		printf(	"%s: Acquire image from %zu,%zu size %zux%zu\n", functionName,
				GetMinX(), GetMinY(), GetSizeX(), GetSizeY()	);
	if (	(	GetSizeX() < GetWidth() )
		||	(	GetSizeY() < GetHeight() )	)
	{
		// Setup PDV ROI image transfer
		// Note: We don't use MinY in setting up the PDV image grab as
		// the ORCA handles the Y offset and Y size, always transfers full rows,
		// and reads the resulting image to row 0
		int		hskip	= GetMinX();
		int		vskip	= 0;
		int		hactv	= GetSizeX();
		int		vactv	= GetSizeY();
		if ( EDT_PDV_DEBUG >= 2 )
			printf(	"%s: Setting PDV ROI to hskip %d, hactv %d, vskip %d, vactv %d\n",
					functionName,	hskip, hactv, vskip, vactv );
		pdv_set_roi(	m_pPdvDev,	hskip, hactv, vskip, vactv );
		pdv_enable_roi(	m_pPdvDev, 1	);
	}
	else
	{
		int		hskip	= 0;
		int		vskip	= 0;
		int		hactv	= GetWidth();
		int		vactv	= GetHeight();
		if ( EDT_PDV_DEBUG >= 2 )
			printf(	"%s: Disabling PDV ROI, restoring hskip %d, hactv %d, vskip %d, vactv %d\n",
					functionName,	hskip, hactv, vskip, vactv );
		pdv_set_roi(	m_pPdvDev,	hskip, hactv, vskip, vactv );
		pdv_enable_roi(	m_pPdvDev, 0	);
	}

	int framesync = pdv_enable_framesync(	m_pPdvDev, PDV_FRAMESYNC_ON	);
	if ( EDT_PDV_DEBUG >= 2 )
		printf(	"%s: framesync enable %s\n", functionName, framesync == 0 ? "succeeded" : "failed" );


	// TODO: Is this the right place for these?
	setIntegerParam( ADNumImagesCounter, 0 );
	UpdateStatus( ADStatusAcquire );

	// Start grabbing images
    pdv_start_images( m_pPdvDev, m_NumMultiBuf );

	if ( EDT_PDV_DEBUG >= 1 )
        printf(	"%s: Start acquire, count = %d\n",
				functionName, m_acquireCount );
	asynPrint(	this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
        		"%s: Start acquire, count = %d\n",
				functionName, m_acquireCount );
    return 0;
}

// Interleave function definitions
// Implemented in Pdv Library's pdv_interlace.c
extern "C" int deIntlv_MidTop_Line16(u_short *src, int width, int rows, u_short *dest);


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

	// The diag. ReAcquireTimer times the interval between calls to pdv_wait_images_*
	m_ReAcquireTimer.StopTimer( );

	// Wait for a new image
	u_int				pdvTimestamp[2];
	bool				fRaw		= ( m_tyInterlace != PDV_INTLV_IN_PDV_LIB );
	unsigned char	*	pPdvBuffer	= pdv_wait_images_timed_raw( m_pPdvDev, 1, pdvTimestamp, fRaw );
	//	unsigned char	*	pPdvBuffer	= pdv_wait_images_raw( m_pPdvDev, 1 );

	// Update diagnostic timers
	m_ReArmTimer.StartTimer( );
	m_ReAcquireTimer.StartTimer( );
	m_ProcessImageTimer.StartTimer( );

	// See if we exited acquire mode while waiting for an image
	if ( !InAcquireMode() )
	{
		if ( EDT_PDV_DEBUG >= 1 )
			printf(	"%s: AcquireMode cancelled in thread %s\n", functionName, epicsThreadGetNameSelf() );
		return asynError;
	}

	// Decrement acquire count
	if( m_acquireCount > 0 )
		m_acquireCount--;
	if ( m_acquireCount != 0 )
	{	// edtPdvCamera::ReArm()
		m_ReArmTimer.StopTimer( );
		//	Continue the pipeline by starting the next image
		CONTEXT_TIMER( "pdv_start_image" );
		pdv_start_image( m_pPdvDev );
	}

	if ( pPdvBuffer == NULL )
	{
		if ( EDT_PDV_DEBUG >= 1 )
			printf(	"%s: Failed to acquire image!\n", functionName );
		return asynError;
	}

	//	Caution, this raw image will get over-written if we're not done processing it
	//	before the edt ring buffer wraps.
	
	// if ( m_fCheckFrameSync )
	{
		CONTEXT_TIMER( "AcquireData-pdv_check_framesync" );
    	u_int frame_counter;
    	int	 framesync_status = pdv_check_framesync( m_pPdvDev, (u_char*)pPdvBuffer, &frame_counter );
		if ( framesync_status == 0 )
		{
			if ( EDT_PDV_DEBUG >= 5 )
				printf(	"%s: framesync counter %d\n", functionName, frame_counter );
		}
		if ( framesync_status < 0 )
		{
			if ( EDT_PDV_DEBUG >= 3 )
				printf(	"%s: framesync not working!\n", functionName );
		}
		else if ( framesync_status > 0 )
		{
			if ( EDT_PDV_DEBUG >= 3 )
				printf(	"%s: Lost framesync!\n", functionName );
			// setIntegerParam(cameraLostFrameSyncCounter, lostFrameSyncCounter++);
		}
	}

	//	TODO:	Can we move the rest of this code to ProcessImage and avoid needless buffer
	//			copies of dark images?

	// This counts camera frames whether we're aqcuiring or not.
	// If the camera is in freerun or being triggered, it should increment.
	//int		nCamFramesSinceReset = pdv_cl_get_fv_counter( m_pPdvDev );
	//pdv_cl_reset_fv_counter( m_pPdvDev );

	//	Saving the image to the NDArrayPool 
	//	Note: Prior version had NDArrayPool point to the EDT raw image buffer
	//	for zero-copy image handling.
	//	However, since AreaDetector is based on plugins which process the NDArrayPool
	//	buffers at a later time, I don't think it's safe to leave the data in
	//	the EDT buffer as they have no overrun checks like NDArrayPool does.

	UpdateStatus( ADStatusReadout );

    NDArray	*	pNDArray = NULL;
	//	Lock NDArrayPool update
	{
	CONTEXT_TIMER( "AcquireData-NDArrayPool-Update" );
	lock();
	// TODO: Handle color images!
    const int		ndims	= 2;
    size_t			dims[ndims];
	NDDataType_t	pixelType		= NDUInt8;
	unsigned int	bytesPerPixel	= 1;
	if ( m_numOfBits > 8 )
	{
		pixelType =	NDUInt16;
		bytesPerPixel	= 2;
	}
	m_imageSize		= GetSizeX() * GetSizeY();
    dims[0]			= GetSizeX();	//	m_width;
    dims[1]			= GetSizeY();	//	m_height;
	assert( dims[0] != 0 );
	assert( dims[1] != 0 );
    pNDArray = pNDArrayPool->alloc( ndims, dims, pixelType, 0, NULL );
	if ( pNDArray == NULL )
	{
        errlogPrintf( "%s: NULL pNDArray!\n", functionName );
		unlock();
		return -1;
	}
	if ( pNDArray->pData == NULL )
	{
        errlogPrintf( "%s: NULL pNDArray->pData!\n", functionName );
		unlock();
		return -1;
	}

	if ( EDT_PDV_DEBUG >= 4 )
		printf(	"%s: Transferring %zu bytes from DMA buf %p to NDArray buf %p\n",
				functionName, m_imageSize * bytesPerPixel, pPdvBuffer, pNDArray->pData );
	// See if we support deinterlacing raw images
	switch ( m_tyInterlace )
	{
	case PDV_WORD_INTLV_MIDTOP_LINE:
		{
		CONTEXT_TIMER( "AcquireData-deIntlv_MidTop_Line16" );
		deIntlv_MidTop_Line16(	(u_short *) pPdvBuffer, GetSizeX(), GetSizeY(),
								(u_short *) pNDArray->pData );
		}
		break;
	case PDV_INTLV_IN_PDV_LIB:
		{
		CONTEXT_TIMER( "AcquireData-memcpy" );
		// Image already de-interleaved in PDV library, just memcpy it here.
		memcpy( pNDArray->pData, pPdvBuffer, m_imageSize * bytesPerPixel );
		}
		break;
	default:
        errlogPrintf( "%s: NULL pNDArray->pData!\n", functionName );
		unlock();
		return -1;
		break;
	}

	pNDArray->ndims				= ndims;
	// Update each dimension's settings from the RBV values
	pNDArray->dims[0].size		= GetSizeX();
	pNDArray->dims[0].offset	= GetMinX();
	pNDArray->dims[0].binning	= GetBinX();
	pNDArray->dims[1].size		= GetSizeY();
	pNDArray->dims[1].offset	= GetMinY();
	pNDArray->dims[1].binning	= GetBinY();

	pImage->SetNDArrayPtr( pNDArray );
	unlock();
	}

	{
	CONTEXT_TIMER( "AcquireData-wrapup" );
	if ( EDT_PDV_DEBUG >= 4 )
		// Write NDArray report to stdout
		// if details param >= 5, calls NDAttributeList::report()
		// NDArray::report( FILE * fp, int details );
		pNDArray->report( stdout, EDT_PDV_DEBUG + 1 );

	// Increment NumImagesCounter
	//	TODO: Replace this silly pattern w/ local m_numImagesCounter that sets ADNumImagesCounter
	int		numImagesCounter;
	getIntegerParam(	ADNumImagesCounter,	&numImagesCounter	);
	numImagesCounter++;
	setIntegerParam(	ADNumImagesCounter,	numImagesCounter	);

	// See if we're done
	if ( m_acquireCount == 0 )
	{
		SetAcquireMode( false );
		if ( EDT_PDV_DEBUG >= 1 )
			printf(	"%s: Image acquisition completed in thread %s\n", functionName, epicsThreadGetNameSelf() );
		asynPrint(	this->pasynUserSelf, ASYN_TRACEIO_DRIVER,
					"%s: Image acquisition completed in thread %s\n", functionName, epicsThreadGetNameSelf() );
		pdv_timeout_restart( m_pPdvDev, 0 );
	}
	}
	return 0;
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
		// Set the NDArray EPICS timestamp and unique ID
		if ( EDT_PDV_DEBUG >= 3 )
			printf(	"%s: Timestamp image w/ pulseID %d\n", functionName, pulseID );
		pNDArray->epicsTS	= *pTimeStamp;
		pNDArray->uniqueId	= pulseID;

		//
		// asynPortDriver also allows us to provide custom routines:
		//	getTimeStamp(epicsTimeStamp * pTS)
		//	setTimeStamp(const epicsTimeStamp * pTS)
		// Not sure if these will be needed.
		//
		// NDArray has two timestamps.
		//	double			NDArray::timeStamp	is seconds since 1970
		//	epicsTimeStamp	NDArray::epicsTS	is epics sec and ns relative to 1990
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

		// Do NDArray callbacks unlocked to avoid deadlocks if the plugin
		// tries to lock the driver.
		this->unlock();
		if ( EDT_PDV_DEBUG >= 4 )
			printf(	"%s: Processing image callbacks ...\n", functionName );
		doCallbacksGenericPointer( pNDArray, NDArrayData, 0 );
		this->lock();

		if ( EDT_PDV_DEBUG >= 4 )
			printf(	"%s: Processing parameter callbacks ...\n", functionName );

		// Call parameter callbacks
		callParamCallbacks();
	}
	this->unlock();
	return 0;
}

bool	 edtPdvCamera::IsSynced(	edtImage	*	pImage	)
{
	if ( pImage == NULL )
		return false;
	return true;
}


void edtPdvCamera::ReleaseData(	edtImage	*	pImage	)
{
	UpdateStatus( ADStatusIdle );
	if ( pImage == NULL )
		return;
	CONTEXT_TIMER( "ReleaseData" );
	this->lock();
	pImage->ReleaseNDArray();
	this->unlock();
}

int	edtPdvCamera::TimeStampImage(
	edtImage		*	pImage,
	epicsTimeStamp	*	pDest,
	int				*	pPulseNumRet	)
{
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

	epicsTime			newTimeStamp( newEvrTime );;
	if ( newTimeStamp == m_priorTimeStamp )
		return -1;
	m_priorTimeStamp	= newTimeStamp;
	*pDest				= newTimeStamp;

	// TODO: Is there any way to make this less SLAC specific?
	// If not, maybe we just drop it, as no one really
	// needs pNDArray->uniqueId to be the pulse number
	if ( pPulseNumRet != NULL )
		*pPulseNumRet = pDest->nsec & 0x1FFFF;
	return 0;
}


int	edtPdvCamera::SetSizeX(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetSizeX";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, ROI width %zu == 0!\n",
        	    		functionName, value );
		return asynError;
	}
	if ( value > m_width )
	{
        errlogPrintf(	"%s: ERROR, ROI width %zu > max %zu!\n",
        	    		functionName, value, m_width );
		return asynError;
	}
	if ( m_SizeX != value )
	{
		m_SizeX		= value;
		m_fReconfig	= true;
	}
	return asynSuccess;
}

int	edtPdvCamera::SetSizeY(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetSizeY";
	if ( value == 0 )
	{
        errlogPrintf(	"%s: ERROR, ROI height %zu == 0!\n",
        	    		functionName, value );
		return asynError;
	}
	if ( value > m_height )
	{
        errlogPrintf(	"%s: ERROR, ROI height %zu > max %zu!\n",
        	    		functionName, value, m_height );
		return asynError;
	}
	if ( m_SizeY != value )
	{
		m_SizeY		= value;
		m_fReconfig	= true;
	}
	return asynSuccess;
}

int	edtPdvCamera::SetMinX(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetMinX";
	if ( value > (m_width - 1) )
	{
        errlogPrintf(	"%s: ERROR, ROI start %zu > max %zu!\n",
        	    		functionName, value, (m_width - 1) );
		return asynError;
	}
	if ( m_MinX != value )
	{
		m_MinX	= value;
		m_fReconfig	= true;
	}
	return asynSuccess;
}

int	edtPdvCamera::SetMinY(	size_t	value	)
{
    static const char	*	functionName	= "edtPdvCamera::SetMinY";
	if ( value > (m_height - 1) )
	{
        errlogPrintf(	"%s: ERROR, ROI start %zu > max %zu!\n",
        	    		functionName, value, (m_height - 1) );
		return asynError;
	}
	if ( m_MinY != value )
	{
		m_MinY		=  value;
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
	if ( m_BinX != value )
	{
		m_BinX		= value;
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
	if ( m_BinY != value )
	{
		m_BinY		= value;
		m_fReconfig	= true;
	}
	return asynSuccess;
}

int	edtPdvCamera::SetTriggerMode(	int	value	)
{
//	static const char	*	functionName	= "edtPdvCamera::SetTriggerMode";
	TriggerMode_t	tyTriggerMode	= static_cast<TriggerMode_t>( value );
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
	setIntegerParam( ADTriggerMode,	m_TriggerMode	);
	return asynSuccess;
}

int		edtPdvCamera::SetGain( double gain )
{
	int status = asynSuccess;
	
	if ( m_Gain != gain )
	{
		status = pdv_set_gain( m_pPdvDev, static_cast<int>(gain) );
    	m_Gain	= gain;
	}
	setDoubleParam( ADGain, m_Gain );
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
    fprintf(	fp, "EDT PDV camera port %s: %s\n",
				this->portName, m_pPdvDev ? "Connected" : "Disconnected" );

	int			connected	= 0;
	pasynManager->isConnected( this->pasynUserSelf, &connected );
	if ( m_pPdvDev && !connected )
	{
		fprintf(	fp, "Warning, Camera port %s thinks it's %s, but asynManager says %s\n",
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

        fprintf( fp, "  Sensor bits:       %u\n",	m_numOfBits );
        fprintf( fp, "  Sensor width:      %zd\n",	m_width );
        fprintf( fp, "  Sensor height:     %zd\n",	m_height );
        fprintf( fp, "  Image size:        %zu\n",	m_imageSize );
        fprintf( fp, "  DMA size:          %zu\n",	m_dmaSize );
        fprintf( fp, "  ROI Horiz skip:    %d\n",	m_EdtHSkip );
        fprintf( fp, "  ROI Horiz size:    %d\n",	m_EdtHSize );
        fprintf( fp, "  ROI Vert  skip:    %d\n",	m_EdtVSkip );
        fprintf( fp, "  ROI Vert  size:    %d\n",	m_EdtVSize );
        fprintf( fp, "  Trig Level:        %s\n",	TrigLevelToString( m_trigLevel ) );
        fprintf( fp, "  PDV DebugLevel:    %u\n",	m_EdtDebugLevel );
        fprintf( fp, "  PDV DebugMsgLevel: %u\n",	m_EdtDebugMsgLevel );
        fprintf( fp, "  Frame Count:       %u\n",	m_ArrayCounter );

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

    if ( pasynUser->reason == SerAcquireTime )
	{
		setDoubleParam( ADAcquireTime, value );
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
	if ( EDT_PDV_DEBUG >= 5 )
		printf(	"%s: Reason %d %s\n", functionName, pasynUser->reason, reasonName );
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"%s: Reason %d %s\n", functionName, pasynUser->reason, reasonName );

//	if ( pasynUser->reason == EdtTrigLevel ) setIntegerParam( EdtTrigLevel, *pValueRet );

	// Call base class
	asynStatus	status	= asynPortDriver::readInt32( pasynUser, pValueRet );
    return status;
}

asynStatus edtPdvCamera::writeInt32(	asynUser *	pasynUser, epicsInt32	value )
{
    static const char	*	functionName	= "edtPdvCamera::writeInt32";
    const char			*	reasonName		= "unknownReason";
	getParamName( 0, pasynUser->reason, &reasonName );
	if ( EDT_PDV_DEBUG >= 2 )
		printf(	"%s: Reason %d %s, value %d\n", functionName, pasynUser->reason, reasonName, value );
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"%s: Reason %d %s, value %d\n", functionName, pasynUser->reason, reasonName, value );

    if ( pasynUser->reason == ADAcquire )
	{
		return SetAcquireMode( value );
    }


    if ( pasynUser->reason == ADBinX)	SetBinX(	value	);
    if ( pasynUser->reason == ADBinY)	SetBinY(	value	);
    if ( pasynUser->reason == ADMinX)	SetMinX(	value	);
    if ( pasynUser->reason == ADMinY)	SetMinY(	value	);
    if ( pasynUser->reason == ADSizeX)	SetSizeX(	value	);
    if ( pasynUser->reason == ADSizeY)	SetSizeY(	value	);

    if ( pasynUser->reason == ADImageMode)
	{
		// Get prior values
		int			imageMode;
		// TODO: Why are we asking ADManager what our driver parameter values are? Add m_ImageMode, ...
		getIntegerParam( ADImageMode,	&imageMode	);

		if ( imageMode != value )
		{
			int			numImages;
			getIntegerParam( ADNumImages,	&numImages	);

			// Capture mode changed
			imageMode = value;
			setIntegerParam( ADImageMode,	value );

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
			if ( EDT_PDV_DEBUG >= 1 )
				printf(	"%s: Setting acquire count to %d\n", 
						functionName, m_acquireCount );
		}
	}

    if ( pasynUser->reason == ADNumImages )
	{
		// Get prior values
		int			numImages;
		getIntegerParam( ADNumImages,	&numImages	);

		if ( numImages != value )
		{
			// Image capture count changed
			setIntegerParam( ADNumImages,	value );

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
			if ( EDT_PDV_DEBUG >= 1 )
				printf(	"%s: Updating acquire count to %d\n", 
						functionName, m_acquireCount );
		}
	}

    if ( pasynUser->reason == NDArrayCounter	)
	{
		return SetArrayCounter( value );
	}
    if ( pasynUser->reason == EdtTrigLevel		)	setIntegerParam( EdtTrigLevel, value );

    if ( pasynUser->reason == ADTriggerMode		)	SetTriggerMode(	value );
    if ( pasynUser->reason == SerTriggerMode	)	SetTriggerMode(	value );
    if ( pasynUser->reason == SerMinX			)	SetMinX(	value );
    if ( pasynUser->reason == SerMinY			)	SetMinY(	value );
    if ( pasynUser->reason == SerSizeX			)	SetSizeX(	value );
    if ( pasynUser->reason == SerSizeY			)	SetSizeY(	value );
 
    callParamCallbacks( 0, 0 );

    return asynStatus(0);
}

asynStatus edtPdvCamera::writeFloat64(	asynUser *	pasynUser, epicsFloat64	value )
{
    static const char	*	functionName	= "edtPdvCamera::writeFloat64";
    const char			*	reasonName		= "unknownReason";
	getParamName( 0, pasynUser->reason, &reasonName );
	asynPrint(	pasynUser,	ASYN_TRACE_FLOW,
				"asynPrint "
				"%s: Reason %d %s, value %lf\n", functionName, pasynUser->reason, reasonName, value );

    if ( pasynUser->reason == SerAcquireTime )
	{
		setDoubleParam( ADAcquireTime, value );
	}
    if ( pasynUser->reason == ADGain)	SetGain(	value	);

    callParamCallbacks();

    return asynStatus(0);
}


int		edtPdvCamera::traceVPrint( const char	*	pFormat, va_list pvar )
{
	if ( pasynTrace->getTraceMask( this->pasynUserSelf ) & ASYN_TRACEIO_DRIVER )
		return pasynTrace->vprint( this->pasynUserSelf, ASYN_TRACEIO_DRIVER, pFormat, pvar );
	return 0;
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
	const char	*	modelName	)
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
    if ( edtPdvCamera::CreateCamera( cameraName, unit, channel, modelName ) != 0 )
    {
        errlogPrintf( "edtPdvConfig failed for camera %s, config %s!\n", cameraName, modelName );
		if ( EDT_PDV_DEBUG >= 4 )
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

    if (  cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf( "NULL or zero length camera name. Check parameters to edtPdvConfig()!\n");
        return  -1;
    }
    if ( edtPdvCamera::CreateCamera( cameraName, unit, channel, modelName ) != 0 )
    {
        errlogPrintf( "edtPdvConfig failed for camera %s!\n", cameraName );
		if ( EDT_PDV_DEBUG >= 4 )
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
static const iocshArg	*	edtPdvConfigArgs[4]	=
{
	&edtPdvConfigArg0, &edtPdvConfigArg1, &edtPdvConfigArg2, &edtPdvConfigArg3
};
static const iocshFuncDef   edtPdvConfigFuncDef	= { "edtPdvConfig", 4, edtPdvConfigArgs };
static int  edtPdvConfigCallFunc( const iocshArgBuf * args )
{
    return edtPdvConfig( args[0].sval, args[1].ival, args[2].ival, args[3].sval );
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
static const iocshArg		edtPdvConfigFullArg4	= { "maxBuffers",	iocshArgInt };
static const iocshArg		edtPdvConfigFullArg5	= { "maxMemory",	iocshArgInt };
static const iocshArg		edtPdvConfigFullArg6	= { "priority",		iocshArgInt };
static const iocshArg		edtPdvConfigFullArg7	= { "stackSize",	iocshArgInt };
// There has to be a better way to handle triggerPV, delayPV, and syncPV
//static const iocshArg		edtPdvConfigFullArgX	= { "triggerPV",	iocshArgString };
//static const iocshArg		edtPdvConfigFullArgX	= { "delayPV",		iocshArgString };
//static const iocshArg		edtPdvConfigFullArgX	= { "syncPV",		iocshArgString };
static const iocshArg	*	edtPdvConfigFullArgs[8]	=
{
	&edtPdvConfigFullArg0, &edtPdvConfigFullArg1, &edtPdvConfigFullArg2, &edtPdvConfigFullArg3,
	&edtPdvConfigFullArg4, &edtPdvConfigFullArg5, &edtPdvConfigFullArg6, &edtPdvConfigFullArg7
};
static const iocshFuncDef   edtPdvConfigFullFuncDef	= { "edtPdvConfigFull", 8, edtPdvConfigFullArgs };
static int  edtPdvConfigFullCallFunc( const iocshArgBuf * args )
{
    return edtPdvConfigFull(
		args[0].sval, args[1].ival, args[2].ival, args[3].sval,
		args[4].ival, args[5].ival, args[6].ival, args[7].ival	);
}
void edtPdvConfigFullRegister(void)
{
	iocshRegister( &edtPdvConfigFullFuncDef, reinterpret_cast<iocshCallFunc>(edtPdvConfigFullCallFunc) );
}

// Register Function:
//	int edt_set_verbosity( int level )
static const iocshArg		edt_set_verbosityArg0	= { "level",	iocshArgInt };
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
	epicsExportAddress( int, EDT_PDV_DEBUG );
}
