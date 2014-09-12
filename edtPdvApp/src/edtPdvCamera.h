#ifndef	EDT_PDV_CAMERA_H
#define	EDT_PDV_CAMERA_H
/** ADDriver for cameras using EDT framegrabbers via EDT's PDV software library **/

/*	Macro declarations */
#define CAMERA_THREAD_PRIORITY	(epicsThreadPriorityMedium)
#define CAMERA_THREAD_STACK		(0x20000)


#ifdef __cplusplus

#include <map>
#include <string>
#include <vector>
#include <epicsMutex.h>
#include <epicsThread.h>
#include <dbScan.h>
#include "ADDriver.h"
#include "edtinc.h"
//	#include "HiResTime.h"
//	#include "timesync.h"

//class	dataObject;
class	edtImage;
class	asynEdtPdvSerial;
//	struct	ttyController;

#define N_PDV_DRV_ADDR_MAX		4
#define N_PDV_MULTIBUF_DEF		4

// Camera operation data structure definition
class edtPdvCamera : public ADDriver
{
//	friend class edtSyncObject;
public:		//	Public member functions

	///	Constructor
	edtPdvCamera(	const char			*	cameraName,
					int						unit,
					int						channel,
					const char			*	modelName,
					int						maxBuffers	= 0,	// 0 = unlimited
					size_t					maxMemory	= 0,	// 0 = unlimited
					int						priority	= 0,	// 0 = default 50, high is 90
					int						stackSize	= 0	);	// 0 = default 1MB

	/// Destructor
	virtual ~edtPdvCamera();

	enum TriggerMode_t { TRIGMODE_FREERUN, TRIGMODE_EXT, TRIGMODE_PULSE };

	///	Update AreaDetector params related to camera configuration
	int UpdateADConfigParams( );

	/// Open a fresh connection to the camera
	/// Closes any existing EDT objects,
	///	re-reads the configuration file,
	/// and reopens the connection.
    asynStatus ConnectCamera( );

	/// Close the EDT PDV camera connections
    asynStatus DisconnectCamera( );

    /// These methods are overwritten from asynPortDriver
    virtual asynStatus connect(		asynUser	* pasynUser	);
    virtual asynStatus disconnect(	asynUser	* pasynUser	);

    /// These are the methods that we override from ADDriver
#if 0
    virtual asynStatus readFloat64(	asynUser	*	pasynUser,	epicsFloat64	value	);
#endif
    virtual asynStatus readInt32(	asynUser	*	pasynUser,	epicsInt32		value	);
    virtual asynStatus writeInt32(	asynUser	*	pasynUser,	epicsInt32		value	);
    virtual asynStatus writeFloat64(asynUser	*	pasynUser,	epicsFloat64	value	);
    void	report(	FILE	*	fp,	int	details	);
 
	/// Registered with epicsAtExit() for clean disconnect
    static void shutdown(	void	*	arg	);

	///	Get camera class, typically the manufacturer
	const std::string	&	GetCameraClass( ) const
	{
		return m_CameraClass;
	}

	///	Get camera Model
	const std::string	&	GetCameraModel( ) const
	{
		return m_CameraModel;
	}

	///	Get camera name
	const std::string	&	GetCameraName( ) const
	{
		return m_CameraName;
	}

	///	Get camera serial port name
	const std::string	&	GetSerialPortName( ) const
	{
		return m_SerialPort;
	}

	///	Get Driver Version
	const std::string	&	GetDrvVersion( ) const
	{
		return m_DrvVersion;
	}

	///	Get Library Version
	const std::string	&	GetLibVersion( ) const
	{
		return m_LibVersion;
	}

	bool	IsAcquiring()
	{
		return m_fAcquiring;
	}

	bool	InAcquireMode()
	{
		return m_fAcquireMode;
	}
	asynStatus	SetAcquireMode( int fAcquireMode );
	bool	GetAcquireMode() const
	{
		return m_fAcquireMode;
	}

//	Image		*	GetCurImageBuf( );
//	Image		*	GetNextImageBuf(unsigned int &);

	int				SetGain( double gain );
	double			GetGain( ) const
	{
		return m_Gain;
	}

	int				SetBinX(	unsigned int	value	);
	unsigned int	GetBinX( ) const
	{
		return m_BinX;
	}

	int				SetBinY(	unsigned int	value	);
	unsigned int	GetBinY( ) const
	{
		return m_BinY;
	}

	int		SetMinX(	size_t	value	);
	size_t	GetMinX( ) const
	{
		return m_MinX;
	}

	int		SetMinY(	size_t	value	);
	size_t	GetMinY( ) const
	{
		return m_MinY;
	}

	int		SetSizeX(	size_t	value	);
	size_t	GetSizeX( ) const
	{
		if ( m_SizeX == 0 )
			return m_width;
		return m_SizeX;
	}

	int		SetSizeY(	size_t	value	);
	size_t	GetSizeY( ) const
	{
		if ( m_SizeY == 0 )
			return m_height;
		return m_SizeY;
	}

	int				SetTriggerMode(	int	value	);
	TriggerMode_t	GetTriggerMode( ) const
	{
		return m_TriggerMode;
	}


	size_t	GetWidth( ) const
	{
		return m_width;
	}

	size_t	GetHeight( ) const
	{
		return m_height;
	}

	unsigned int	GetNumBits( ) const
	{
		return m_numOfBits;
	}

	/// Get frame count
	int		GetFrameCount( ) const
	{
		return m_frameCounts;
	}

	/// Set frame count
	int		SetFrameCount( int count )
	{
		m_frameCounts	= 0;
		return 0;
	}

	/// Return camera image size in bytes
	size_t	GetImageSize( ) const
	{
		return m_imageSize;
	}

	/// Return camera pixel count
	size_t	GetNumPixels( ) const
	{
		return m_width * m_height;
	}

#undef	USE_EDT_ROI
#ifdef	USE_EDT_ROI
	///	Enable/disable ROI
	void			SetEnableROI( bool	fEnableROI );
	int SetHWROI(int, int, int, int);
	int SetHWROIinit(int, int, int, int);
	int HWROI_X(void)   { return m_HWROI_X; }
	int HWROI_XNP(void) { return m_HWROI_XNP; }
	int HWROI_Y(void)   { return m_HWROI_Y; }
	int HWROI_YNP(void) { return m_HWROI_YNP; }
#endif	//	USE_EDT_ROI

	/// Get event number to use for timestamping camera images
    int				GetTimeStampEvent( ) const
	{
		return m_timeStampEvent;
	}

	/// Set event number to use for timestamping camera images
    void			SetTimeStampEvent( int timeStampEvent )
	{
		m_timeStampEvent	= timeStampEvent;
	}

	/// Get last fiducial timestamp id
    int				GetFiducial( ) const
	{
		return m_fiducial;
	}

	/// Set fiducial timestamp id
    void			SetFiducial( int fiducial )
	{
		m_fiducial	= fiducial;
	}

	IOSCANPVT		GetIoScan( ) const
	{
		return m_ioscan;
	}

	///	Show Camera info on stdout
	int						CameraShow( int level );

	///	Start Camera
	int						CameraStart( );

	///	Acquire next image from the camera
	int						AcquireData(	edtImage	*	pImage	);

	///	Acquire images from camera
	void					acquireLoop(	);

	///	Reconfigure camera (reread config file and re-initialize connection)
	int						Reconfigure(	);

	bool					IsSynced(		edtImage		*	);

	void					ReleaseData(	edtImage		*	);

	int						ProcessData(	edtImage		*	pImage,
											epicsTimeStamp	*	pTimeStamp,
											int					pulseID		);

	int						TimeStampImage(	edtImage		*	pImage,
											int					eventNumber,
											epicsTimeStamp	*	pDest,
											int				*	pPulseNumRet	);


public:		//	Public class functions

	static int				CreateCamera( const char * cameraName, int unit, int channel, const char * modelName );

	static edtPdvCamera	*	CameraFindByName( const std::string & name );

	PdvDev				*	GetPdvDev( ) const
	{
		return m_pPdvDev;
	}

	static	int				ShowAllCameras( int level );

	static	int				StartAllCameras( );

	static bool				IsCameraChannelUsed( unsigned int unit,  unsigned int channel );

private:	//	Private member functions
	//	Internal version of reconfigure
	//	Don't call without holding m_reconfigLock!
	int						_Reconfigure( );

private:	//	Private class functions
	static	void			CameraAdd(		edtPdvCamera * pCamera );
	static	void			CameraRemove(	edtPdvCamera * pCamera );
	static	int				ThreadStart(	edtPdvCamera * pCamera );

public:		//	Public member variables	(Make these private!)

protected:	//	Protected member variables
	bool			m_fAcquireMode;		// Set true to start acquiring images, false to halt
	bool			m_fAcquiring;		// True while acquiring images, false when idle
	bool			m_fExitApp;			// Set true to shutdown ioc
	bool			m_fReconfig;		// Are we currently reconfiguring the ROI?
	int				m_NumMultiBuf;		// Number of pdv multi buffers configured

private:	//	Private member variables
	PdvDev		* 	m_pPdvDev;		// Ptr to EDT digital video device

	unsigned int	m_unit;			// index of EDT DV C-LINK PMC card
	unsigned int	m_channel;		// channel on  EDT DV C-LINK PMC card

	std::string		m_CameraClass;	// Manufacturer of camera
	std::string		m_CameraInfo;	// camera info string
	std::string		m_CameraModel;	// model name as reported by camera
	std::string		m_CameraName;	// name of this camera, must be unique
	std::string		m_ConfigFile;	// current configuration file for camera
	std::string		m_DrvVersion;	// Driver Version
	std::string		m_LibVersion;	// Library Version
	std::string		m_ModelName;	// Configuration model name for camera (selected in st.cmd)
	std::string		m_SerialPort;	// name of camera's serial port

	size_t			m_width;		// number of column of this camera
	size_t			m_height;		// number of row of this camera
	unsigned int	m_numOfBits;	// number of bits of this camera

	size_t			m_imageSize;	// image size in byte
	size_t			m_dmaSize;		// dma size of image in byte, usually same as imageSize

	unsigned int	m_trigLevel;		// Ext. Trigger Mode (0=Edge,1=Level,2=Sync)

	int				m_PdvDebugLevel;	// PDV library debug level
	int				m_PdvDebugMsgLevel;	// PDV library debug msg level

	TriggerMode_t	m_TriggerMode;

	// HW ROI and binning parameters from ADBase
	size_t			m_BinX;
	size_t			m_BinY;
	size_t			m_MinX;
	size_t			m_MinY;
	size_t			m_SizeX;
	size_t			m_SizeY;

	// Gain value for camera
	double			m_Gain;

	unsigned int	m_timeStampEvent;	// Event number to use for timestamping images
	int				m_frameCounts;		// debug information to show trigger frequency
	int				m_acquireCount;		// How many images to acquire
	unsigned int	m_fiducial;			// Fiducial ID from last timestamped image

	epicsMutexId	m_reconfigLock;		// Protect against more than one thread trying to reconfigure the device
//	epicsMutexId	m_resetLock;		// From edt_unix HW ROI support
//	epicsMutexId	m_waitLock;			// From edt_unix HW ROI support
	epicsThreadId	m_threadId;			// Thread identifier for polling thread
	epicsEventId	m_acquireEvent;		// Used to signal when image acquisition is needed

	double			m_acquireTimeout;	// Timeout in seconds for image acquisition (will retry after cking for reconfig)
	double			m_reconfigDelay;	// Delay in seconds after failed reconfiguration before retry

#define IMGQBUFSIZ				4
#define IMGQBUFMASK				3
//	std::vector<Image *>	m_imgqBuf;	// image queue buffer
//	unsigned int			m_imgqrd;	// Image Queue Read pointer
//	unsigned int			m_imgqwr;	// Image Queue Write pointer

#ifdef	USE_EDT_ROI
	char				*	m_serial_init;	// The serial command to put us in the current HWROI mode.
											// (This is in the PdvDev->dd_p->serial_init buffer!)
	unsigned int			m_HWROI_X;
	unsigned int			m_HWROI_XNP;
	unsigned int			m_HWROI_Y;
	unsigned int			m_HWROI_YNP;
	unsigned int			m_HWROI_gen;
#endif	//	USE_EDT_ROI

	#define FIRST_EDT_PDV_PARAM PdvClass
	int		PdvClass;
	int		PdvDebug;
	int		PdvDebugMsg;
	int		PdvDrvVersion;
	int		PdvInfo;
	int		PdvLibVersion;
	int		PdvMultiBuf;
	int		PdvTrigLevel;

	// Serial front-end params for ADBase parameters
	int		SerAcquireTime;
	int		SerMinX;
	int		SerMinY;
	int		SerSizeX;
	int		SerSizeY;
	int		SerTriggerMode;
	#define LAST_EDT_PDV_PARAM  SerTriggerMode

	IOSCANPVT				m_ioscan;
	asynEdtPdvSerial	*	m_pAsynSerial;
	//	ttyController	*	m_ttyPort;

private:	//	Private class variables
	static	std::map<std::string, edtPdvCamera *>	ms_cameraMap;
};

/* EDT PDV Parameters, common to all EDT PDV cameras */
#define NUM_EDT_PDV_PARAMS ((int)(&LAST_EDT_PDV_PARAM - &FIRST_EDT_PDV_PARAM + 1))

#endif /* __cplusplus */

#define EdtPdvClassString		"EDT_PDV_CLASS"
#define EdtPdvDebugString		"EDT_PDV_DEBUG"
#define EdtPdvDebugMsgString	"EDT_PDV_DEBUG_MSG"
#define EdtPdvDrvVersionString	"EDT_PDV_DRV_VERSION"
#define EdtPdvInfoString		"EDT_PDV_INFO"
#define EdtPdvLibVersionString	"EDT_PDV_LIB_VERSION"
#define EdtPdvMultiBufString	"EDT_PDV_MULTIBUF"
#define EdtPdvTrigLevelString	"EDT_PDV_TRIG_LEVEL"

// This group provides a way to have serial readbacks get reflected in
// their ADBase class equivalents, for example
// SerAcquireTime	=>	ADAcquireTime 
#define EdtSerAcquireTimeString	"EDT_SER_ACQUIRE_TIME"
#define EdtSerMinXString		"EDT_MIN_X"
#define EdtSerMinYString		"EDT_MIN_Y"
#define EdtSerSizeXString		"EDT_SIZE_X"
#define EdtSerSizeYString		"EDT_SIZE_Y"
#define EdtSerTriggerModeString	"EDT_SER_TRIGGER_MODE"

/*	Diagnostic variables	*/
extern int				EDT_PDV_DEBUG;
extern unsigned long	imageCaptureCount;

/* "C" linkage Configuration functions for iocsh */
extern "C" int	edtPdvConfig(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	modelName		);
extern "C" int	edtPdvConfigFull(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	modelName,
	int				maxBuffers,		// 0 = unlimited
	size_t			maxMemory,		// 0 = unlimited
	int				priority,		// 0 = default 50, high is 90
	int				stackSize	);	// 0 = default 1 MB

#endif	/*	EDT_PDV_CAMERA_H	*/
