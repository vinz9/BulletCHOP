#include "CHOP_CPlusPlusBase.h"

#include <btBulletDynamicsCommon.h>



// To get more help about these functions, look at CHOP_CPlusPlusBase.h


class BulletCHOP : public CHOP_CPlusPlusBase
{
public:
	BulletCHOP(const OP_NodeInfo *info);
	virtual ~BulletCHOP();

	virtual void		getGeneralInfo(CHOP_GeneralInfo*) override;
	virtual bool		getOutputInfo(CHOP_OutputInfo*) override;
	virtual const char*	getChannelName(int index, void* reserved) override;

	virtual void		execute(const CHOP_Output*,
									OP_Inputs*,
								void* reserved) override;


	virtual int			getNumInfoCHOPChans() override;
	virtual void		getInfoCHOPChan(int index,
								OP_InfoCHOPChan* chan) override;

	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize) override;
	virtual void		getInfoDATEntries(int index,
									int nEntries,
									OP_InfoDATEntries* entries) override;

	virtual void		setupParameters(OP_ParameterManager* manager) override;
	virtual void		pulsePressed(const char* name) override;

	void worldSetup();

	void worldDestroy();

	void addRigidBox(btVector3 pos, btVector3 rot, btVector3 scale, btScalar mass);
	void addRigidSphere(btVector3 pos, btVector3 rot, btScalar radius, btScalar mass);
	void addKineBox(btVector3 pos, btVector3 rot, btVector3 scale);

	void addPlane(btVector3 planeNormal, btScalar planeConstant);

	void initRigidSpheres(OP_Inputs* inputs);
	void initRigidBoxes(OP_Inputs* inputs);
	void initKineBoxes(OP_Inputs* inputs);
	void initPlanes(OP_Inputs* inputs);
	void updateKineBoxes(OP_Inputs* inputs);

	void removeBodies();

	btScalar	getDeltaTimeMicroseconds()
	{
		btScalar dt = (btScalar)clock.getTimeMicroseconds();
		clock.reset();
		return dt;
	}

private:

	// We don't need to store this pointer, but we do for the example.
	// The CHOP_NodeInfo class store information about the node that's using
	// this instance of the class (like its name).
	const OP_NodeInfo		*myNodeInfo;

	// In this example this value will be incremented each time the execute()
	// function is called, then passes back to the CHOP 
	int						 myExecuteCount;

	float pos;
	float ms;

	btBroadphaseInterface* broadphase;
	btDefaultCollisionConfiguration* collisionConfiguration;
	btCollisionDispatcher* dispatcher;
	btSequentialImpulseConstraintSolver* solver;
	btDiscreteDynamicsWorld* dynamicsWorld;
	
	btClock clock;

	btAlignedObjectArray<btCollisionShape*>	rigidSpheresShapes;
	btAlignedObjectArray<btCollisionShape*>	rigidBoxesShapes;
	btAlignedObjectArray<btCollisionShape*> kineBoxesShapes;
	btAlignedObjectArray<btCollisionShape*> staticPlanesShapes;


	btAlignedObjectArray<int> rigidSpheresIds;
	btAlignedObjectArray<int> rigidBoxesIds;
	btAlignedObjectArray<int> kineBoxesIds;
	btAlignedObjectArray<int> staticPlanesIds;

	enum { TX, TY, TZ, RX, RY, RZ , SPEED, QX, QY, QZ, QW};


	int frame = 0;

	
};
