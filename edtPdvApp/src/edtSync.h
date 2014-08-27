#ifndef EDT_SYNC_H
#define EDT_SYNC_H

#include <string>
#include "NDArray.h"
#include "epicsTime.h"
#include "syncObject.h"
#include "edtinc.h"

// edtSyncData class, derived from timesync module's dataObject
class edtImage:	public dataObject
{
public:
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

class	edtPdvCamera;


// edtSyncObject class, derived from timesync module's syncObject
class edtSyncObject: public syncObject
{
 public:
	edtSyncObject(	edtPdvCamera	* 	pCam	);
	virtual ~edtSyncObject()						 {}
	virtual int		AcquireData(	edtImage *	pDataObject );
	virtual int		CheckData(		dataObject *	pDataObject );
	virtual bool	CheckSync(		dataObject *	pDataObject );
	virtual int		GetTimestampAndPulse( int eventNumber, epicsTimeStamp * pDest, int * pPulseNumRet );
	virtual int     QueueData(      dataObject      *   pDataObject,
									epicsTimeStamp  *   pTimeStamp,
									int                 pulseID );

	virtual void	ReleaseData(	dataObject		*	pDataObject );

	void SetPdvDev( PdvDev * pPdvDev )
	{
		m_pPdvDev	= pPdvDev;
	}

 private:
	edtPdvCamera	* 	m_pCam;
	PdvDev			*	m_pPdvDev;
};

#endif // EDT_SYNC_H
