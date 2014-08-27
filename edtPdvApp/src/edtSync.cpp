// #include <iocsh.h>
// #include <callback.h>
// #include <dbScan.h>
// #include <dbAccess.h>
// #include <cantProceed.h>
// #include <epicsThread.h>
// #include <epicsExport.h>
// #include <registryFunction.h>
// #include <errlog.h>
// #include <epicsVersion.h>
// #include <unistd.h>

#include "edtSync.h"
#include "edtPdvCamera.h"
// #include "evrTime.h"

using namespace		std;

// Forward declarations
class		edtPdvCamera;

edtSyncObject::edtSyncObject(	edtPdvCamera	* 	pCam	)
	:	syncObject(				),
		m_pCam(			pCam	)
{
}


int edtSyncObject::AcquireData(
	edtImage		*	pDataObject,
	double				timeOutSec	)
{
	return m_pCam->AcquireData( pDataObject );
}

