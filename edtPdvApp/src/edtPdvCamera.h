#ifndef	EDT_PDV_CAMERA_H
#define	EDT_PDV_CAMERA_H
/** ADDriver for cameras using EDT framegrabbers via EDT's PDV software library **/



#ifdef __cplusplus

#include <map>
#include <string>
#include <vector>
#include <epicsMutex.h>
#include <epicsThread.h>
#include <dbScan.h>
#include "ADDriver.h"
#include "edtinc.h"
#include "syncDataAcq.h"

#ifdef	USE_DIAG_TIMER
#include "HiResTime.h"
#include "ContextTimerMax.h"
#endif	//	USE_DIAG_TIMER

//class	dataObject;
class	edtImage;
class	asynEdtPdvSerial;
//	struct	ttyController;

#define N_PDV_DRV_ADDR_MAX		4
#define N_PDV_MULTIBUF_DEF		4

#define	PDV_INTLV_IN_PDV_LIB	-1

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
					const char			*	edtMode,
					int						maxBuffers	= 0,	// 0 = unlimited
					size_t					maxMemory	= 0,	// 0 = unlimited
					int						priority	= 0,	// 0 = default 50, high is 90
					int						stackSize	= 0	);	// 0 = default 1MB

	/// Destructor
	virtual ~edtPdvCamera();

	enum TriggerMode_t { TRIGMODE_FREERUN, TRIGMODE_EXT, TRIGMODE_PULSE };

	enum EdtMode_t { EDTMODE_BASE, EDTMODE_MEDIUM, EDTMODE_FULL };

	///	Update AreaDetector params related to camera configuration
	int UpdateADConfigParams( );

	///	Update EDT Debug params
	int UpdateEdtDebugParams( );

	/// Open a fresh connection to the camera
	/// Closes any existing EDT objects,
	///	re-reads the configuration file,
	/// and reopens the connection.
    asynStatus ConnectCamera( );

	/// Close the EDT PDV camera connections
    asynStatus DisconnectCamera( );

	asynStatus		UpdateStatus( int	newStatus	);

    asynUser	*	GetAsynUser()
	{
		return pasynUserSelf;	// TODO: Is this safe?
	}

    int	GetAcquireCount()
	{
		return m_acquireCount;
	}

    /// These methods are overwritten from asynPortDriver
    virtual asynStatus connect(		asynUser	* pasynUser	);
    virtual asynStatus disconnect(	asynUser	* pasynUser	);

    /// These are the methods that we override from ADDriver
#if 0
    virtual asynStatus readFloat64(	asynUser	*	pasynUser,	epicsFloat64 *	value	);
#endif
    virtual asynStatus readInt32(	asynUser	*	pasynUser,	epicsInt32	 *	value	);
    virtual asynStatus writeInt32(	asynUser	*	pasynUser,	epicsInt32		value	);
    virtual asynStatus writeFloat64(asynUser	*	pasynUser,	epicsFloat64	value	);
    void	report(	FILE	*	fp,	int	details	);

	/// Registered with epicsAtExit() for clean disconnect
	static void ExitHook( void * pThis );
 
 	/// Shutdown driver
	void Shutdown( );

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
		if ( m_pSyncDataAcquirer == NULL )
			return false;
		return m_pSyncDataAcquirer->IsAcquiring();
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

	int				RequestBinX(	unsigned int	value	);
	int				SetBinX(		unsigned int	value	);
	unsigned int	GetBinX( ) const
	{
		return m_BinX;
	}

	int				RequestBinY(	unsigned int	value	);
	int				SetBinY(		unsigned int	value	);
	unsigned int	GetBinY( ) const
	{
		return m_BinY;
	}

	int		RequestMinX(	size_t	value	);
	int		SetMinX(		size_t	value	);
	size_t	GetMinX( ) const
	{
		return m_MinX;
	}

	int		RequestMinY(	size_t	value	);
	int		SetMinY(		size_t	value	);
	size_t	GetMinY( ) const
	{
		return m_MinY;
	}

	int		RequestSizeX(	size_t	value	);
	int		SetSizeX(		size_t	value	);
	size_t	GetSizeX( ) const
	{
		if ( m_SizeX == 0 )
			return m_ClMaxWidth;
		return m_SizeX;
	}

	int		RequestSizeY(	size_t	value	);
	int		SetSizeY(		size_t	value	);
	size_t	GetSizeY( ) const
	{
		if ( m_SizeY == 0 )
			return m_ClMaxHeight;
		return m_SizeY;
	}

	EdtMode_t GetEdtMode( ) const
	{
		return m_EdtMode;
	}

	int				RequestTriggerMode(	int	value	);
	int				SetTriggerMode(	int	value	);
	TriggerMode_t	GetTriggerMode( ) const
	{
		return m_TriggerMode;
	}

	unsigned int	GetNumBits( ) const
	{
		return m_ClNumBits;
	}

	/// Get frame count
	int		GetArrayCounter( ) const
	{
		return m_ArrayCounter;
	}

	/// Increment frame count
	asynStatus		IncrArrayCounter( );

	/// Set frame count
	asynStatus		SetArrayCounter( int value );

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

	///	Start Camera image acquisition
	int						StartAcquisition( );

	///	Acquire next image from the camera
	int						AcquireData(	edtImage	*	pImage	);

	///	Check for a valid image, returns 0 on success, error code on error
	int						CheckData(		edtImage	*	pImage	);

	///	Returns true if device needs reconfiguring
	bool					NeedsReconfigure(	)
	{
		return m_fReconfig || m_fReopen;
	}

	///	Reconfigure camera (reread config file and re-initialize connection)
	/// Takes the reconfigure lock to make it thread safe
	int						Reconfigure(	);

	///	Reopen EDT driver (re-initialize connection)
	/// Can be called from any thread to open or reopen the EDT driver connection
	/// Takes the reconfigure lock to make it thread safe
	int		Reopen(	);

	bool	IsSynced(		edtImage		*	pImage,
							epicsTimeStamp	*	pTimeStamp,
							int					pulseID		);

	void	ReleaseData(	edtImage		*	);

	int		ProcessData(	edtImage		*	pImage,
							epicsTimeStamp	*	pTimeStamp,
							int					pulseID		);

	int		TimeStampImage(	edtImage		*	pImage,
							epicsTimeStamp	*	pDest,
							int				*	pPulseNumRet	);

	//
	//	De-interleave routines to handle copying raw image data from DMA buffers
	//  to NDArray's, cropping for HW ROI as needed.
	//

	/// De-interleave ROI line by line from the middle outwards to the top
	/// and bottom lines w/ 16 bit pixels
	int		DeIntlvMidTopLine16(	NDArray	*	pNDArray, void	*	pRawData	);

	/// De-interleave as is from top to bottom, allowing only for HW ROI
	int		DeIntlvRoiOnly16(		NDArray	*	pNDArray, void	*	pRawData	);

	/// GetEdtDebugLevel
	int		GetEdtDebugLevel( );

	/// GetEdtDebugMsgLevel
	int		GetEdtDebugMsgLevel( );

	/// SetEdtDebugLevel
	int		SetEdtDebugLevel( int value );

	/// SetEdtDebugMsgLevel
	int		SetEdtDebugMsgLevel( int value );

	// Trace level for diagnostics
	unsigned int GetTraceLevel()
	{
		return pasynTrace->getTraceMask( this->pasynUserSelf );
	}

	int		traceVPrint( const char	*	pFormat, va_list pvar );

public:		//	Public class functions

	static int				CreateCamera(	const char *	cameraName, int unit, int channel,
											const char *	modelName,	const char * edtMode );

	static edtPdvCamera	*	CameraFindByName( const std::string & name );

	PdvDev				*	GetPdvDev( ) const
	{
		return m_pPdvDev;
	}

	static	int				ShowAllCameras( int level );

	static bool				IsCameraChannelUsed( unsigned int unit,  unsigned int channel );

private:	//	Private member functions
	//	Internal version of reconfigure
	//	Don't call without holding m_reconfigLock!
	int		_Reconfigure( );
	int		_Reopen( );

	//	NDArray routines
	//	Don't call without holding driver lock!
	NDArray *	AllocNDArray(	);
	int			LoadNDArray(	NDArray	*	pNDArray, void	*	pRawData	);

private:	//	Private class functions
	static	void			CameraAdd(		edtPdvCamera * pCamera );
	static	void			CameraRemove(	edtPdvCamera * pCamera );

public:		//	Public member variables	(Make these private!)

protected:	//	Protected member variables
	bool			m_fAcquireMode;		// Set true to start acquiring images, false to halt
	bool			m_fExitApp;			// Set true to shutdown ioc
	bool			m_fReconfig;		// True when we need to reconfigure the ROI or other camera parameters
	bool			m_fReopen;			// True when we need to reread the configuration file
	int				m_NumMultiBuf;		// Number of pdv multi buffers configured

private:	//	Private member variables
	PdvDev		* 	m_pPdvDev;		// Ptr to EDT digital video device

	unsigned int	m_unit;			// index of EDT DV C-LINK PMC card
	unsigned int	m_channel;		// channel on  EDT DV C-LINK PMC card

	epicsTimeStamp	m_priorTimeStamp;	// Last timestamp for this event number

	std::string		m_CameraClass;	// Manufacturer of camera
	std::string		m_CameraInfo;	// camera info string
	std::string		m_CameraModel;	// model name as reported by camera
	std::string		m_CameraName;	// name of this camera, must be unique
	std::string		m_ConfigFile;	// current configuration file for camera
	std::string		m_DrvVersion;	// Driver Version
	std::string		m_LibVersion;	// Library Version
	std::string		m_ModelName;	// Configuration model name for camera (selected in st.cmd)
	std::string		m_SerialPort;	// name of camera's serial port

	size_t			m_ClCurWidth;	// CamLink connection cur width  in pixels
	size_t			m_ClCurHeight;	// CamLink connection cur height in pixels
	size_t			m_ClMaxWidth;	// CamLink connection max width  in pixels
	size_t			m_ClMaxHeight;	// CamLink connection max height in pixels
	unsigned int	m_ClNumBits;	// CamLink connection bits  per pixel
	int				m_ClHTaps;		// CamLink connection horiz taps
	int				m_ClVTaps;		// CamLink connection vert  taps

	int				m_tyInterlace;	// Interlace type for DMA transfer

	EdtMode_t		m_EdtMode;

	TriggerMode_t	m_TriggerMode;
	TriggerMode_t	m_TriggerModeReq;

	// HW ROI and binning parameters from ADBase
	size_t	m_BinX,		m_BinXReq,		m_BinY,		m_BinYReq;
	size_t	m_MinX,		m_MinXReq,		m_MinY,		m_MinYReq;
	size_t	m_SizeX,	m_SizeXReq,		m_SizeY,	m_SizeYReq;

	// Holds currently alloc'd NDArray ptr
	// Must hold NDArrayDriver lock() while != NULL
    // NDArray		*	m_pNDArray;

	// Gain value for camera
	double			m_Gain;

	int				m_ArrayCounter;		// Frame count
	int				m_acquireCount;		// How many images to acquire
	unsigned int	m_fiducial;			// Fiducial ID from last timestamped image

	epicsMutexId	m_reconfigLock;		// Protect against more than one thread trying to reconfigure the device
	syncDataAcq<edtPdvCamera, edtImage>		*	m_pSyncDataAcquirer;

	unsigned int	m_trigLevel;		// Ext. Trigger Mode (0=Edge,1=Level,2=Sync)

	int				m_EdtDebugLevel;	// PDV library debug level
	int				m_EdtDebugMsgLevel;	// PDV library debug msg level

	// These variables hold the asyn parameter index numbers for each parameter
	#define FIRST_EDT_PARAM EdtClass
	int		EdtClass;
	int		EdtDebug;
	int		EdtDebugMsg;
	int		EdtDrvVersion;
	int		EdtHSkip;
	int		EdtHSize;
	int		EdtHTaps;
	int		EdtMode;
	int		EdtOverrun;
	int		EdtVSkip;
	int		EdtVSize;
	int		EdtVTaps;
	int		EdtInfo;
	int		EdtLibVersion;
	int		EdtMultiBuf;
	int		EdtTrigLevel;

	// Serial front-end params for ADBase parameters
	int		SerAcquireTime;
	int		SerBinX;
	int		SerBinY;
	int		SerGain;
	int		SerMinX;
	int		SerMinY;
	int		SerSizeX;
	int		SerSizeY;
	int		SerTriggerMode;
	#define LAST_EDT_PARAM  SerTriggerMode

#ifdef	USE_DIAG_TIMER
	ContextTimerMax			m_ReAcquireTimer;
	ContextTimerMax			m_ReArmTimer;
	ContextTimerMax			m_ProcessImageTimer;
#endif	//	USE_DIAG_TIMER

	IOSCANPVT				m_ioscan;
	asynEdtPdvSerial	*	m_pAsynSerial;
	//	ttyController	*	m_ttyPort;

private:	//	Private class variables
	static	std::map<std::string, edtPdvCamera *>	ms_cameraMap;
};

/* EDT PDV Parameters, common to all EDT PDV cameras */
#define NUM_EDT_PARAMS ((int)(&LAST_EDT_PARAM - &FIRST_EDT_PARAM + 1))

#endif /* __cplusplus */

#define EdtClassString		"EDT_CLASS"
#define EdtDebugString		"EDT_DEBUG"
#define EdtDebugMsgString	"EDT_DEBUG_MSG"
#define EdtDrvVersionString	"EDT_DRV_VERSION"
#define EdtHSkipString		"EDT_HSKIP"
#define EdtHSizeString		"EDT_HSIZE"
#define EdtHTapsString		"EDT_HTAPS"
#define EdtModeString		"EDT_MODE"
#define EdtOverrunString	"EDT_OVERRUN"
#define EdtVSkipString		"EDT_VSKIP"
#define EdtVSizeString		"EDT_VSIZE"
#define EdtVTapsString		"EDT_VTAPS"
#define EdtInfoString		"EDT_INFO"
#define EdtLibVersionString	"EDT_LIB_VERSION"
#define EdtMultiBufString	"EDT_MULTIBUF"
#define EdtTrigLevelString	"EDT_TRIG_LEVEL"

// This group provides a way to have serial readbacks get reflected in
// their ADBase class equivalents, for example
// SerAcquireTime	=>	ADAcquireTime 
#define EdtSerAcquireTimeString	"EDT_SER_ACQUIRE_TIME"
#define EdtSerBinXString		"EDT_BIN_X"
#define EdtSerBinYString		"EDT_BIN_Y"
#define EdtSerGainString		"EDT_GAIN"
#define EdtSerMinXString		"EDT_MIN_X"
#define EdtSerMinYString		"EDT_MIN_Y"
#define EdtSerSizeXString		"EDT_SIZE_X"
#define EdtSerSizeYString		"EDT_SIZE_Y"
#define EdtSerTriggerModeString	"EDT_SER_TRIGGER_MODE"

/*	Diagnostic variables	*/
extern int				DEBUG_EDT_PDV;
extern unsigned long	imageCaptureCount;

/* "C" linkage Configuration functions for iocsh */
extern "C" int	edtPdvConfig(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	modelName,
	const char	*	edtMode		);
extern "C" int	edtPdvConfigFull(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	modelName,
	const char	*	edtMode,
	int				maxBuffers,		// 0 = unlimited
	size_t			maxMemory,		// 0 = unlimited
	int				priority,		// 0 = default 50, high is 90
	int				stackSize	);	// 0 = default 1 MB

#endif	/*	EDT_PDV_CAMERA_H	*/
