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

// Camera operation data structure definition
class edtPdvCamera : public ADDriver
{
//	friend class edtSyncObject;
public:		//	Public member functions

	///	Constructor
	edtPdvCamera(	const char			*	cameraName,
					int						unit,
					int						channel,
					const char			*	cfgName,
					int						maxBuffers	= 0,	// 0 = unlimited
					size_t					maxMemory	= 0,	// 0 = unlimited
					int						priority	= 0,	// 0 = default 50, high is 90
					int						stackSize	= 0	);	// 0 = default 1MB

	/// Destructor
	virtual ~edtPdvCamera();

	///	Initializer function
	int InitCamera( );

    /// These methods are overwritten from asynPortDriver
    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);

    /// These are the methods that we override from ADDriver
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

//	Image		*	GetCurImageBuf( );
//	Image		*	GetNextImageBuf(unsigned int &);

	int				GetGain( );

	int				SetGain( int gain );

	unsigned int	GetWidth( ) const
	{
		return m_width;
	}

	unsigned int	GetHeight( ) const
	{
		return m_height;
	}

	unsigned int	GetNumBits( ) const
	{
		return m_numOfBits;
	}

	/// Get frame count
	int				GetFrameCount( ) const
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
	unsigned int	GetImageSize( ) const
	{
		return m_imageSize;
	}

	/// Return camera pixel count
	unsigned int	GetNumPixels( ) const
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
		return m_Fiducial;
	}

	/// Set fiducial timestamp id
    void			SetFiducial( int fiducial )
	{
		m_Fiducial	= fiducial;
	}

	IOSCANPVT		GetIoScan( ) const
	{
		return m_ioscan;
	}

	///	Show Camera info on stdout
	int						CameraShow( int level );

	///	Start Camera
	int						CameraStart( );

public:		//	Public class functions
	static int				CreateCamera( const char * cameraName, int unit, int channel, const char * cfgName );

	static edtPdvCamera	*	CameraFindByName( const std::string & name );

	PdvDev				*	GetPdvDev( ) const
	{
		return m_pPdvDev;
	}

	static	int				ShowAllCameras( int level );

	static	int				StartAllCameras( );

	static bool				IsCameraChannelUsed( unsigned int unit,  unsigned int channel );

private:	//	Private class functions
	static	void			CameraAdd(		edtPdvCamera * pCamera );
	static	void			CameraRemove(	edtPdvCamera * pCamera );
	static	int				ThreadStart(	edtPdvCamera * pCamera );

public:		//	Public member variables	(Make these private!)

protected:	//	Protected member variables
	bool			m_reconfig;				// Are we currently reconfiguring the ROI?
	unsigned int	m_NumBuffers;		// Number of pdv buffers configured
	unsigned int	m_NumPdvTimeouts;	// Number of pdv timeouts

private:	//	Private member variables
	PdvDev		* 	m_pPdvDev;		// Ptr to EDT digital video device
	Dependent	*	m_pDD;			// Pointer to PDV dependent information.

	unsigned int	m_unit;			// index of EDT DV C-LINK PMC card
	unsigned int	m_channel;		// channel on  EDT DV C-LINK PMC card

	std::string		m_CameraClass;	// Manufacturer of camera
	std::string		m_CameraInfo;	// camera info string
	std::string		m_CameraModel;	// model name of camera
	std::string		m_CameraName;	// name of this camera, must be unique
	std::string		m_ConfigName;	// configuration name for camera
	std::string		m_DrvVersion;	// Driver Version
	std::string		m_LibVersion;	// Library Version

	unsigned int	m_width;		// number of column of this camera
	unsigned int	m_height;		// number of row of this camera
	unsigned int	m_numOfBits;	// number of bits of this camera

	unsigned int	m_imageSize;	// image size in byte
	unsigned int	m_dmaSize;		// dma size of image in byte, usually same as imageSize
	
	int				m_PdvDebugLevel;	// PDV library debug level
	int				m_PdvMsgDebugLevel;	// PDV library debug level

	// HW ROI and binning parameters from ADBase
	int				m_binX;
	int				m_binY;
	int				m_minX;
	int				m_minY;
	int				m_sizeX;
	int				m_sizeY;
	int				m_maxSizeX;
	int				m_maxSizeY;

	// Gain value for camera
	int				m_gain;

	unsigned int	m_timeStampEvent;	// Event number to use for timestamping images
	unsigned long	m_numClippedPixels;	// Number of clipped pixels from brightness shift

	int				m_frameCounts;		// debug information to show trigger frequency
	unsigned int	m_Fiducial;			// Fiducial ID from last timestamped image

	epicsMutexId	m_waitLock;			// Acquire thread is running lock (used during reconfig)
	epicsThreadId	m_ThreadId;			// Thread identifier for polling thread

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

	#define FIRST_EDT_PDV_PARAM m_PdvParamClass
	int		m_PdvParamClass;
	int		m_PdvParamDebug;
	int		m_PdvParamDebugMsg;
	int		m_PdvParamInfo;
	#define LAST_EDT_PDV_PARAM  m_PdvParamInfo

	IOSCANPVT		m_ioscan;

private:	//	Private class variables
	static	std::map<std::string, edtPdvCamera *>	ms_cameraMap;
};

/* EDT PDV Parameters, common to all EDT PDV cameras */
#define NUM_EDT_PDV_PARAMS ((int)(&LAST_EDT_PDV_PARAM - &FIRST_EDT_PDV_PARAM + 1))

#endif /* __cplusplus */

#define EdtPdvClassString		"EDT_PDV_CLASS"
#define EdtPdvDebugString		"EDT_PDV_DEBUG"
#define EdtPdvDebugMsgString	"EDT_PDV_DEBUG_MSG"
#define EdtPdvInfoString		"EDT_PDV_INFO"

/*	Diagnostic variables	*/
extern int				EDT_PDV_DEBUG;
extern unsigned long	imageCaptureCount;

/* "C" linkage Configuration functions for iocsh */
extern "C" int	edtPdvConfig(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	cfgName		);
extern "C" int	edtPdvConfigFull(
	const char	*	cameraName,
	int				unit,
	int				channel,
	const char	*	cfgName,
	int				maxBuffers,		// 0 = unlimited
	size_t			maxMemory,		// 0 = unlimited
	int				priority,		// 0 = default 50, high is 90
	int				stackSize	);	// 0 = default 1 MB

#endif	/*	EDT_PDV_CAMERA_H	*/
