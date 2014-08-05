//
//	asynEdtPdvSerial.h
//
//	Header file for asynEdtPdvSerial class.
//	It inherits from asynPortDriver and implements
//	serial I/O to EDT framegrabber based camlink cameras
//	via the EDT PDV serial API
//
#ifndef	asynEdtPdvSerial_H
#define	asynEdtPdvSerial_H

#include "asynPortDriver.h"

#include "edtinc.h"

///	asynEdtPdvSerial class
class asynEdtPdvSerial : public asynPortDriver
{
//	friend class edtSyncObject;
public:		//	Public member functions

	///	Constructor
	asynEdtPdvSerial(	const char			*	portName,
						int						priority	= 0,	// 0 = default 50, high is 90
						int						autoConnect	= 0,	// 0 = no auto-connect
						int						maxBuffers	= 0,	// 0 = unlimited
						size_t					maxMemory	= 0,	// 0 = unlimited
						int						stackSize	= 0		// 0 = default 1MB
						);

	/// Destructor
	virtual ~asynEdtPdvSerial();

	//
	//	asynPortDriver function overrides
	//
    virtual asynStatus	connect(	asynUser	* pasynUser	);

    virtual asynStatus	disconnect(	asynUser	* pasynUser	);

	virtual asynStatus	readOctet(
		asynUser		*	pasynUser,
		char			*	value,
		size_t				maxChars,
		size_t			*	nActual,
		int				*	eomReason	);

	virtual asynStatus	writeOctet(
		asynUser		*	pasynUser,
		const char		*	value,
		size_t				maxChars,
		size_t			*	nActual	);

	asynStatus	pdvDevConnected(
		PdvDev			*	pPdvDev	);

	asynStatus	pdvDevDisconnected(
		PdvDev			*	pPdvDev	);

#if 0
	// Do we need these?
	virtual asynStatus flushOctet(
		asynUser		*	pasynUser	);

	virtual asynStatus setInputEosOctet(
		asynUser		*	pasynUser,
		const char		*	eos,
		int					eosLen	);

	virtual asynStatus getInputEosOctet(
		asynUser		*	pasynUser,
		char			*	eos,
		int					eosSize,
		int				*	eosLen	);

	virtual asynStatus	setOutputEosOctet(
		asynUser		*	pasynUser,
		const char		*	eos,
		int					eosLen	);

	virtual asynStatus	getOutputEosOctet(
		asynUser		*	pasynUser,
		char			*	eos,
		int					eosSize,
		int				*	eosLen	);
#endif

	//	Private member variables
	PdvDev			*	m_pPdvDev;
	asynUser		*	m_pasynUser;
	char			*	m_inputEosOctet;
	int					m_inputEosLenOctet;
	char			*	m_outputEosOctet;
	int					m_outputEosLenOctet;
	bool				m_connected;

#if 0

    void	report(	FILE	*	fp,	int	details	);
 
	char			*	portName;
	int					m_addr;
	unsigned long		nRead;
	unsigned long		nWritten;
	double				readTimeout;
	double				writeTimeout;
	asynInterface		common;
	asynInterface		octet;
	epicsMutexId		m_serialLock;
#endif
};

#endif	//	asynEdtPdvSerial_H
