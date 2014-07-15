/////////////////////////////////////////////////////////////////////////////
// Filename: edtPdvCamera.cpp
// Description: EPICS device support for cameras using EDT framegrabbers
//              via EDT's PDV software library
// Author: Bruce Hill, SLAC National Accelerator Lab, July 11 2014
/////////////////////////////////////////////////////////////////////////////

//	Standard headers
#include <iostream>

//	EPICS headers
#include <iocsh.h>
#include <callback.h>
#include <dbScan.h>
#include <dbAccess.h>
#include <cantProceed.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <registryFunction.h>
#include <errlog.h>
#include <epicsVersion.h>
#include <unistd.h>

// AreaDetector headers
#include "ADDriver.h"

// ADEdtPdv headers
#include "edtPdvCamera.h"
#include "drvAsynEdtPdvSerial.h"

//	PCDS headers
//#include "HiResTime.h"
//#include "ContextTimer.h"
//#include "camRecord.h"
//#include "Image.h"
//#include "evrTime.h"
//#include "timesync.h"

using namespace		std;

static const char *	driverName	= "prosilica";

int		EDT_PDV_DEBUG	= 2;

#define	N_PDV_BUF_DEFAULT	(IMGQBUFSIZ+4)	//	Default number of pdv buffer to use

//	t_HiResTime		imageCaptureCumTicks	= 0LL;
//	unsigned long	imageCaptureCount		= 0L;

///	Camera map - Stores ptr to all edtPdvCamera instances indexed by name
map<string, edtPdvCamera *>	edtPdvCamera::ms_cameraMap;

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


int edtPdvCamera::StartAllCameras()
{
    static const char	*	functionName = "edtPdvCamera::StartAllCameras";
	map<string, edtPdvCamera *>::iterator	it;

	for ( it = ms_cameraMap.begin(); it != ms_cameraMap.end(); ++it )
	{
		edtPdvCamera		*	pCamera	= it->second;
		if ( pCamera->CameraStart() != 0 )
		{
			errlogPrintf(	"%s:%s: ERROR, CameraStart() call failed for camera %s!\n", 
							driverName, functionName, pCamera->m_CameraName.c_str() );
			epicsThreadSuspendSelf();
			return -1;
		}
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

	cout << "\tCamera " << m_CameraName	<< " is installed on PMC " << m_unit << " Channel " << m_channel << endl;
	if ( level >= 1 )
	{
		cout	<< "\t\tType: "			<< m_CameraClass
				<< " "					<< m_CameraModel
				<< ", configuration: " 	<< m_ConfigName << endl;
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
	acquire = 0;
	edt_abort_dma(m_pPdvDev);
	epicsThreadSleep(2.);
	pdv_close(m_pPdvDev);
}
#endif


///	Thread routine for epics camera
/// For each epics camera, a new thread is spawned, with this
/// function as the first function called by the thread.
/// Typically, this will call a camera function that loops
/// handling successive images, returning only on error.
int edtPdvCamera::ThreadStart(edtPdvCamera * pCamera)
{
    static const char	*	functionName = "edtPdvCamera::ThreadStart";
    if(pCamera == NULL)
    {
        errlogPrintf(	"%s:%s: ERROR, Camera polling thread failed! NULL pCamera!\n",
        	    		driverName, functionName );
        return -1;
    }

#if 0
    edtSyncObject *sobj = new edtSyncObject(pCamera);
    return sobj->poll();
#else
	errlogPrintf(	"%s:%s: ERROR, Camera polling loop not implemented yet!\n",
					driverName, functionName );
	return -1;
#endif
}


int edtPdvCamera::CameraStart( )
{
#if 0
    // Install exit hook for clean shutdown
    epicsAtExit( (EPICSTHREADFUNC)edtPdvCamera::ExitHook, (void *) this );
#endif

    /* Create thread */
    m_ThreadId	= epicsThreadMustCreate(	m_CameraName.c_str(), CAMERA_THREAD_PRIORITY, CAMERA_THREAD_STACK,
											(EPICSTHREADFUNC)edtPdvCamera::ThreadStart, (void *) this );

    return 0;
}

extern "C" int
edtPdvConfig(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	cfgName	)
{
    if( cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf( "NULL or zero length camera name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }
    if( cfgName == NULL || strlen(cfgName) == 0 )
    {
        errlogPrintf( "NULL or zero length config name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }
    if ( edtPdvCamera::CreateCamera( cameraName, unit, channel, cfgName ) != 0 )
    {
        errlogPrintf( "edtPdvConfig failed for camera %s, config %s!\n", cameraName, cfgName );
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
	const char	*	cfgName,
	int				maxBuffers,				// 0 = unlimited
	size_t			maxMemory,				// 0 = unlimited
	int				priority,				// 0 = default 50, high is 90
	int				stackSize			)	// 0 = default 1 MB
{
    if( cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf( "NULL or zero length camera name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }
    if( cfgName == NULL || strlen(cfgName) == 0 )
    {
        errlogPrintf( "NULL or zero length config name.\nUsage: edtPdvConfig(name,unit,chan,config)\n");
        return  -1;
    }

    if( cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf( "NULL or zero length camera name. Check parameters to edtPdvConfig()!\n");
        return  -1;
    }
    if ( edtPdvCamera::CreateCamera( cameraName, unit, channel, cfgName ) != 0 )
    {
        errlogPrintf( "edtPdvConfig failed for camera %s!\n", cameraName );
		if ( EDT_PDV_DEBUG >= 4 )
        	epicsThreadSuspendSelf();
        return -1;
    }
    return 0;
}
#if 0
	// Remnants from edt_unix's epicsCamInit()
    DBADDR addr;
    static double zero = 0.0;
    double *dval = &zero;

    if (trigger && !dbNameToAddr(trigger, &addr)) {
        trig = (epicsUInt32 *) addr.pfield;
        gen = trig + MAX_EV_TRIGGERS;
    } else {
        if (trigger) {
            printf("\n\n\nNo PV trigger named %s!\n\n\n", trigger);
            fflush(stdout);
        }
        trig = NULL;
        gen = NULL;
    }

    if (delay && !dbNameToAddr(delay, &addr)) {
        dval = (double *) addr.pfield;
    }
#endif


int edtPdvCamera::CreateCamera( const char * cameraName, int unit, int channel, const char * cfgName	)
{
    static const char	*	functionName = "edtPdvCamera::CreateCamera";

    /* Parameters check */
    if( cameraName == NULL || strlen(cameraName) == 0 )
    {
        errlogPrintf(	"%s:%s: ERROR, NULL or zero length camera name. Check parameters to edtPdvConfig()!\n",
            			driverName, functionName );
        return  -1;
    }

    if ( edtPdvCamera::CameraFindByName(cameraName) != NULL )
    {
        errlogPrintf(	"%s:%s: ERROR, Camera name %s already in use!\n",
						driverName, functionName, cameraName );
        return -1;
    }

    if( IsCameraChannelUsed( unit, channel ) )
    {
        errlogPrintf(	"%s:%s: ERROR, PMC %d channel %d already in use!\n",
						driverName, functionName, unit, channel	);
        return -1;
    }

    if( cfgName == NULL || strlen(cfgName) == 0 )
    {
        errlogPrintf(	"%s:%s: ERROR, NULL or zero length camera configuration filename.\n",
						driverName, functionName );
        return  -1;
    }

    if ( EDT_PDV_DEBUG )
        cout << "CreateCamera: " << string(cameraName) << endl;
    edtPdvCamera	* pCamera = new edtPdvCamera( cameraName, unit, channel, cfgName );
    assert( pCamera != NULL );
    int	status	= pCamera->InitCamera( );
	if ( status != 0 )
    {
        errlogPrintf( "edtPdvConfig failed for camera %s!\n", cameraName );
		delete pCamera;
        return -1;
    }

	CameraAdd( pCamera );
    return 0;
}


edtPdvCamera::edtPdvCamera(
	const char			*	cameraName,
	int						unit,
	int						channel,
	const char			*	cfgName,
	int						maxBuffers,				// 0 = unlimited
	size_t					maxMemory,				// 0 = unlimited
	int						priority,				// 0 = default 50, high is 90
	int						stackSize			)	// 0 = default 1 MB
	:	ADDriver(			cameraName,		1,
							NUM_EDT_PDV_PARAMS, maxBuffers, maxMemory,
							asynOctetMask,	0,	// Supports an asynOctect interface w/ no interrupts
							ASYN_CANBLOCK,	1,	// ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1
							priority, stackSize	),
		m_reconfig(			FALSE			    ),
		m_NumBuffers(		N_PDV_BUF_DEFAULT	),
		m_NumPdvTimeouts(	0					),
		m_pPdvDev(			NULL				),
		m_pDD(				NULL			    ),
		m_unit(				unit				),
		m_channel(			channel				),
		m_CameraClass(							),
		m_CameraInfo(							),
		m_CameraModel(							),
		m_CameraName(		cameraName	        ),
		m_ConfigName(		cfgName				),
		m_DrvVersion(							),
		m_width(			0					),
		m_height(			0					),
		m_numOfBits(		0					),
		m_imageSize(		0					),
	//	m_dmaSize(			0					),
		m_gain(				0					),
		m_timeStampEvent(	0					),
		m_frameCounts(		0					),
		m_Fiducial(			0					),
		m_ThreadId(			NULL				),
//		m_imgqrd(			0					),
//		m_imgqwr(			0					),
//		m_imgqBuf(								),
#ifdef USE_EDT_ROI
		m_serial_init(      NULL                ),
		m_HWROI_X(                   0          ),
		m_HWROI_XNP(                 0          ),
		m_HWROI_Y(                   0          ),
		m_HWROI_YNP(                 0          ),
		m_HWROI_gen(                 0          ),
#endif	// USE_EDT_ROI
		m_ioscan(				NULL			)
{
}

// Destructor
edtPdvCamera::~edtPdvCamera( )
{
}

int edtPdvCamera::InitCamera( )
{
    static const char	*	functionName = "InitCamera";
    Edtinfo					edtinfo;
    int						n;

    // Initialize I/O Intr processing
    scanIoInit( &m_ioscan );
    if ( m_ioscan == NULL )
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s: ERROR, scanIoInit failed!\n",
            driverName, functionName );

#if 0
    edt_msg_set_level( edt_msg_default_handle(), DEBUG1 | DEBUG2 );
#endif

    m_pDD = pdv_alloc_dependent();
    if (!m_pDD) {
        printf("\n\n\nCannot allocate Dependent structure?!?\n\n\n");
        return -1;
    }
    if (pdv_readcfg(m_ConfigName.c_str(), m_pDD, &edtinfo)) {
        printf("\n\n\nCannot pdv_readcfg(%s)?!?\n\n\n", m_ConfigName.c_str());
        return -1;
    }
#ifdef	USE_EDT_ROI
    if (m_pDD->foi_init[0])
        m_serial_init = &m_pDD->serial_init[strlen(m_pDD->serial_init)];
    SetHWROIinit(0, 0, m_pDD->width, m_pDD->height);
#endif	//	USE_EDT_ROI
    EdtDev		*	edt = edt_open_channel("pdv", m_unit, m_channel);
    if (!edt) {
        printf("\n\n\nCannot run edt_open_channel?!?\n\n\n");
        return -1;
    }
    if (pdv_initcam(edt, m_pDD, m_unit, &edtinfo, m_ConfigName.c_str(), NULL, 1)) {
        printf("\n\n\nCannot run pdv_initcam?!?\n\n\n");
        return -1;
    }
    if (m_pDD->foi_init[n = strlen(m_pDD->foi_init) - 1] == ':') {
        m_pDD->foi_init[n] = 0;   /* Delete the final colon.  pdv_initcam needs it, but we don't! */
    }

    m_pPdvDev = edt;  /* The EDT manual claims the handles are the same!  And they are! */
    if ( EDT_PDV_DEBUG )
        printf("pdv_open_channel(pdv,%d,%d) succeeded\n",m_unit,m_channel);

    char		buf[MAX_STRING_SIZE];
    if ( edt_get_driver_version(	m_pPdvDev, buf, MAX_STRING_SIZE ) )
        m_DrvVersion = buf;
    if ( edt_get_library_version(	m_pPdvDev, buf, MAX_STRING_SIZE ) )
        m_LibVersion = buf;

	// Fetch the camera manufacturer and model and write them to ADBase
    m_CameraClass	= pdv_get_camera_class(	m_pPdvDev );
    m_CameraModel	= pdv_get_camera_model(	m_pPdvDev );
	setStringParam( ADManufacturer, m_CameraClass.c_str() );
	setStringParam( ADModel,		m_CameraModel.c_str() );

	// Fetch the full image geometry parameters and write them to ADBase
    m_width			= pdv_get_width(	m_pPdvDev );
    m_height		= pdv_get_height(	m_pPdvDev );
    m_numOfBits		= pdv_get_depth(	m_pPdvDev );
	if ( m_numOfBits <= 8 )
		setIntegerParam(	NDDataType,	NDUInt8	);
	else if ( m_numOfBits <= 16 )
		setIntegerParam( NDDataType,	NDUInt16	);
	setIntegerParam( NDArrayCallbacks,	1	);
	setIntegerParam( NDArraySizeX,		m_width	);
	setIntegerParam( NDArraySizeY,		m_height	);
	setIntegerParam( NDArraySize,		m_width * m_height	); 

	//	TODO: Do we need these?
    //	m_gain			= pdv_get_gain(			m_pPdvDev );
	//	m_imageSize		= pdv_get_imagesize(	m_pPdvDev );
    //	m_dmaSize		= pdv_get_dmasize(		m_pPdvDev );

	// Create ADBase parameters
	createParam( EdtPdvClassString,		asynParamOctet,		&m_PdvParamClass	);
	createParam( EdtPdvDebugString,		asynParamInt32,		&m_PdvParamDebug	);
	createParam( EdtPdvDebugMsgString,	asynParamInt32,		&m_PdvParamDebugMsg	);
	createParam( EdtPdvInfoString,		asynParamOctet,		&m_PdvParamInfo		);
//	createParam( EdtPdvFloat1String,	asynParamFloat64,	&m_PdvParamFloat1	);

	// Update EdtPdv asyn parameters
    m_CameraClass	= pdv_get_camera_class(	m_pPdvDev );
    setStringParam( m_PdvParamClass, m_CameraClass.c_str()	);
    m_CameraInfo	= pdv_get_camera_info(	m_pPdvDev );
    setStringParam( m_PdvParamInfo,	m_CameraInfo.c_str()	);

//	m_PdvDebugLevel	= pdv_debug_level(	m_pPdvDev );
//	pdv_setdebug(		m_pPdvDev, 		m_PdvDebugLevel	);
//	setIntegerParam(	m_PdvParamDebug,	m_PdvDebugLevel	);
//	getIntegerParam(	m_PdvParamDebug,	&m_PdvDebugLevel	);

//#include edt_error.h
//	m_PdvMsgDebugLevel	= edt_msg_default_level( );
//	edt_msg_set_level(	edt_msg_default_handle(),	m_PdvDebugMsgLevel	);
//	setIntegerParam(	m_PdvParamDebugMsg,	 m_PdvDebugMsgLevel	);
//	getIntegerParam(	m_PdvParamDebugMsg,	&m_PdvDebugMsgLevel	);

    if (	m_width		== 0
		||	m_height	== 0
		||	m_numOfBits	== 0
	//	||	m_dmaSize	== 0
		||	m_imageSize	== 0	)
    {
        errlogPrintf( "camera %s has an invalid image configuration\n", m_CameraName.c_str() );
        return -1;
    }

	if ( EDT_PDV_DEBUG >= 1 )
		printf( "Camera %s: image size=%u: %u*%u pixels, %u bits/pixel\n",
				m_CameraName.c_str(), m_imageSize, m_width, m_height, m_numOfBits );

    pdv_set_timeout( m_pPdvDev, 0 );

#if 0
	if ( EDT_PDV_DEBUG >= 1 )
		printf( "edtPdvCamera::InitCamera: Initializing %d image queue buffers\n", IMGQBUFSIZ );

    /* Initialize image queue buffer */
    unsigned int	loop;
    for ( loop = 0; loop < IMGQBUFSIZ; loop++ )
	{
        Image	*	pImage	= NULL;
        if ( m_numOfBits <= 8 )
            pImage	= new ByteImage( this, loop + 500, m_numOfBits, NULL, 1, false, false );
        else if ( m_numOfBits <= 16 )
            pImage	= new WordImage( this, loop + 600, m_numOfBits, NULL, 1, false, false );
        assert( pImage != NULL && pImage->GetPtrImageData() == NULL );
        m_imgqBuf.push_back( pImage );
    }

    m_imgqwr	= 0;
    m_imgqrd	= 0;
#endif

    m_waitLock	= epicsMutexMustCreate();

    // All resources allocated

    // Flush serial line
    int cnt = pdv_serial_get_numbytes(m_pPdvDev);
    if ( cnt > 0 )
	{
        char		buf[256];
        printf( "Flushing %d bytes in serial line of %s.\n", cnt, m_CameraName.c_str() );
        pdv_serial_read( m_pPdvDev, buf, cnt );
    }

    // Configure an asyn port for serial commands
    if ( drvAsynEdtPdvSerialPortConfigure( m_CameraName.c_str(), 0, 0, 0, m_pPdvDev ) < 0 ) {
        printf( "Error: Unable to configure asyn serial port for %s.\n", m_CameraName.c_str() );
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
            "%s:%s: ERROR, drvAsynEdtPdvSerialPortConfigure failed!\n",
            driverName, functionName );
        return -1;
    }

    return 0;
}


#if 0
Image	*	edtPdvCamera::GetCurImageBuf( )
{
    Image	*	pImage;
//	epicsMutexLock(m_historyBufMutexLock);
    /* current image is from image queue buffer */
    pImage = m_imgqBuf[ (m_imgqwr - 1) & IMGQBUFMASK ];
//	epicsMutexUnlock(m_historyBufMutexLock);
	return pImage;
}

Image	*	edtPdvCamera::GetNextImageBuf(unsigned int &imgqrd)
{
    Image	*	pImage;	/* current image buffer, could be image queue or circular buffer */
    epicsMutexLock(m_historyBufMutexLock);
    /* current image is from image queue buffer */
	if (m_imgqwr - imgqrd >= IMGQBUFSIZ)
	{
		imgqrd = m_imgqwr - IMGQBUFSIZ + 2;  // We've fallen too far behind, just catch up a bit!
	}
	if (imgqrd == m_imgqwr)
	{
		imgqrd--;                            // We're getting ahead, probably because we have
											 // previously fallen behind.  Just reserve the last.
	}
	pImage = m_imgqBuf[ imgqrd++ & IMGQBUFMASK ];
    epicsMutexUnlock(m_historyBufMutexLock);
	return pImage;
}
#endif


int		edtPdvCamera::GetGain( )
{
	m_gain = pdv_get_gain( m_pPdvDev );
	return m_gain;
}

int		edtPdvCamera::SetGain( int gain )
{
	int status = 0;
	
	if ( m_gain != gain )
	{
		status = pdv_set_gain( m_pPdvDev, gain );
    	m_gain	= gain;
	}
	return status;
}

#ifdef	USE_EDT_ROI
///	Enable/disable ROI
void	edtPdvCamera::SetEnableROI( bool	fEnableROI )
{
	if ( m_fEnableROI != fEnableROI )
	{
		m_fEnableROI	= fEnableROI;
		post_event( EVENT_HIST_IMAGE );
	}
}


static void print_msg(char *msg, char *buf)
{
    printf("\n\n\n%s |", msg);
    while (*buf) {
        char c = *buf++;
        if (c == 10 || (c >= ' ' && c < 127)) {
            putchar(c);
        } else {
            switch (c) {
            case 0x06:
		printf("<ACK>");
		break;
	    case 0x15:
		printf("<NAK>");
		break;
	    case 0x02:
		printf("<STX>");
		break;
	    case 0x03:
		printf("<ETX>");
		break;
	    default:
		printf("<%02x>", c);
            }
        }
    }
    printf("|\n\n\n\n");
}

/* Return 1 if a change, 0 otherwise */
int edtPdvCamera::SetHWROIinit(int x, int y, int xnp, int ynp)
{
    int *p1, *p2, *p3, *p4, full, ret;
    int cam_width, cam_height;

    if (m_pPdvDev) {
        cam_width = m_pPdvDev->dd_p->cam_width;
        cam_height = m_pPdvDev->dd_p->cam_height;
    } else {
        cam_width = m_pDD->width;
        cam_height = m_pDD->height;
    }

    switch (m_pDD->foi_init[0]) {
    case 0:    /* No HWROI support! */
        m_HWROI_X = 0;
        m_HWROI_Y = 0;
        m_HWROI_XNP = cam_width;
        m_HWROI_YNP = cam_height;
        return 0;
    case '1':
        full = 0;
        p1 = &y;
        p2 = &ynp;
        break;
    case '2':
        full = 0;
        p1 = &ynp;
        p2 = &y;
        break;
    case '3':
        full = 1;
        p1 = &x;
        p2 = &y;
        p3 = &xnp;
        p4 = &ynp;
        break;
    case '4':
        full = 1;
        p1 = &x;
        p2 = &y;
        p3 = &xnp;
        p4 = &ynp;
        break;
    default:
        /* Error! */
        return 0;
    }
    if (y < 0)
        y = 0;
    if (y >= cam_height)
        y = cam_height - 1;
    if (ynp > cam_height - y)
        ynp = cam_height - y;
    if (full) {
        if (x < 0)
            x = 0;
        if (x >= cam_width)
            x = cam_width - 1;
        if (xnp > cam_width - x)
            xnp = cam_width - x;
        sprintf(m_serial_init, m_pDD->foi_init + 1, *p1, *p2, *p3, *p4);
    } else {
        x = 0;
        xnp = cam_width;
        sprintf(m_serial_init, m_pDD->foi_init + 1, *p1, *p2);
    }
    ret= ((int) m_HWROI_X != x || (int) m_HWROI_XNP != xnp || 
          (int) m_HWROI_Y != y || (int) m_HWROI_YNP != ynp);
    m_HWROI_X = x;
    m_HWROI_XNP = xnp;
    m_HWROI_Y = y;
    m_HWROI_YNP = ynp;
    return ret;
}

static char *strip_crlf(char *str)
{
    static char    scRetStr[256];
    char   *p = str, *s = scRetStr;

    while (*p)
    {
	if (*p == '\r') {
	    *s++ = '\\';
            *s++ = 'r';
	} else if (*p == '\n') {
	    *s++ = '\\';
            *s++ = 'n';
	} else
	    *s++ = *p;
	++p;
    }

    *s = '\0';

    return scRetStr;
}

int edtPdvCamera::SetHWROI(int x, int y, int xnp, int ynp)
{
    char resp[257];
    int ret;

    if (SetHWROIinit(x, y, xnp, ynp)) {
        epicsMutexLock(m_resetLock);
        m_reconfig = TRUE;

        /*
         * The resetLock serves two purposes:
         *     1) It keeps the asyn driver from doing pdv_serial_* operations.
         *     2) It makes the image acquisition thread hang after getting a timeout.
         * So, next step... force a timeout!
         */
        edt_do_timeout(m_pPdvDev);
        epicsMutexLock(m_waitLock);
        /*
         * Now, we know that the CameraPoll thread must be waiting for resetLock, because
         * we have the waitLock!
         */

        /* Change the camera size */
  	edt_msg(DEBUG2, "%s ", m_serial_init);
        pdv_serial_command(m_pPdvDev, m_serial_init);
        ret = pdv_serial_wait(m_pPdvDev, m_pPdvDev->dd_p->serial_timeout, 16);
        pdv_serial_read(m_pPdvDev, resp, 256);
	edt_msg(DEBUG2, " <%s>", strip_crlf(resp));
        edt_msg(DEBUG2, "\n");

        /* Clear the stale buffers */
        for (int loop = 0; loop < IMGQBUFSIZ; loop++ )
            m_imgqBuf[loop]->SetPtrImageData(NULL);

        m_reconfig = FALSE;
        epicsMutexUnlock(m_waitLock);
        epicsMutexUnlock(m_resetLock);
    }
    return 0;
}
#endif	//	USE_EDT_ROI

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
//	int edtPdvConfig( const char * cameraName, int unit, int channel, const char * cfgName )
static const iocshArg		edtPdvConfigArg0	= { "name",		iocshArgString };
static const iocshArg		edtPdvConfigArg1	= { "unit",		iocshArgInt };
static const iocshArg		edtPdvConfigArg2	= { "channel",	iocshArgInt };
static const iocshArg		edtPdvConfigArg3	= { "cfgFile",	iocshArgString };
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
//	int edtPdvConfigFull( const char * cameraName, int unit, int channel, const char * cfgName, int, size_t, int, int  )
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
