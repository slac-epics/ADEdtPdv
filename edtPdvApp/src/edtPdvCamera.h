#ifndef	EDT_PDV_CAMERA_H
#define	EDT_PDV_CAMERA_H

#include <map>
#include <string>
#include <vector>
#include <epicsMutex.h>
#include <epicsThread.h>
#include <dbScan.h>
#include "edtinc.h"
//	#include "HiResTime.h"
//	#include "timesync.h"

//	Forward declarations

#define CAMERA_THREAD_PRIORITY	(epicsThreadPriorityMedium)
#define CAMERA_THREAD_STACK		(0x20000)

// Camera operation data structure definition
class edtPdvCamera
{
//	friend class edtSyncObject;
public:		//	Public member functions

	///	Constructor
	edtPdvCamera(	const	std::string	&	cameraName,
				int						unit,
				int						channel,
				const	std::string	&	cfgName	);

	///	Initializer function
	int InitCamera( );

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
	static int				CreateCamera( char * cameraName, int unit, int channel, char * cfgName );

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

	std::string		m_CameraName;	// name of this camera, must be unique
	std::string		m_CameraClass;	// Manufacturer of camera
	std::string		m_CameraModel;	// model name of camera
	std::string		m_ConfigName;	// configuration name for camera
	std::string		m_DrvVersion;	// Driver Version
	std::string		m_LibVersion;	// Library Version

	unsigned int	m_width;		// number of column of this camera
	unsigned int	m_height;		// number of row of this camera
	unsigned int	m_numOfBits;	// number of bits of this camera

	unsigned int	m_imageSize;	// image size in byte
	unsigned int	m_dmaSize;		// dma size of image in byte, usually same as imageSize

	int				m_gain;			// Gain value for camera

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

	IOSCANPVT		m_ioscan;

private:	//	Private class variables
	static	std::map<std::string, edtPdvCamera *>	ms_cameraMap;
};

//	Diagnostic variables
extern int				EDT_PDV_DEBUG;
extern unsigned long	imageCaptureCount;

#endif	//	EDT_PDV_CAMERA_H	*/
