#ifndef EDT_SYNC_H
#define EDT_SYNC_H

#include <string>
#include "epicsTime.h"
#include "syncObject.h"

class edtSyncData:	public dataObject
{
};


class edtSyncObject: public syncObject
{
 public:
	edtSyncObject(	);
	virtual ~edtSyncObject()						 {}
	virtual int		AcquireData(	dataObject *	pDataObject, double	timeOutSec );
	virtual int		CheckData(		dataObject *	pDataObject );
	virtual bool	CheckSync(		dataObject *	pDataObject );
	virtual int		GetTimestampAndPulse( int eventNumber, epicsTimeStamp * pDest, int * pPulseNumRet );
	virtual int     QueueData(      dataObject      *   pDataObject,
									epicsTimeStamp  *   pTimeStamp,
									int                 pulseID );

	virtual void	ReleaseData(	dataObject *	pDataObject );

 private:
};

#endif // EDT_SYNC_H
