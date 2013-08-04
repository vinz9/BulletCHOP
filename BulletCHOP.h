#include "CHOP_CPlusPlusBase.h"

#include <btBulletDynamicsCommon.h>

/*

This example file implements a class that does 2 different things depending on
if a CHOP is connected to the CPlusPlus CHOPs input or not.
The example is timesliced, which is the more complex way of working.

If an input is connected the node will output the same number of channels as the
input and divide the first 'N' samples in the input channel by 2. 'N' being the current
timeslice size. This is noteworthy because if the input isn't changing then the output
will look wierd since depending on the timeslice size some number of the first samples
of the input will get used.

If no input is connected then the node will output a smooth sine wave at 120hz.
*/


// To get more help about these functions, look at CHOP_CPlusPlusBase.h

/*class KinematicMotionState : public btMotionState {
public:
    KinematicMotionState(const btTransform &initialpos) { mPos1 = initialpos; }
    virtual ~ KinematicMotionState() { }
   
    virtual void getWorldTransform(btTransform &worldTrans) const { worldTrans = mPos1; }
    //void setKinematicPos(btTransform &currentPos) { mPos1 = currentPos; }
    virtual void setWorldTransform(const btTransform &worldTrans) { }
 
protected:
    btTransform mPos1;
};*/

class BulletCHOP : public CHOP_CPlusPlusBase
{
public:
	BulletCHOP(const CHOP_NodeInfo *info);
	virtual ~BulletCHOP();

	virtual void		getGeneralInfo(CHOP_GeneralInfo *);
	virtual bool		getOutputInfo(CHOP_OutputInfo*);
	virtual const char*	getChannelName(int index, void* reserved);

	virtual void		execute(const CHOP_Output*,
								const CHOP_InputArrays*,
								void* reserved);


	virtual int			getNumInfoCHOPChans();
	virtual void		getInfoCHOPChan(int index,
										CHOP_InfoCHOPChan *chan);

	virtual bool		getInfoDATSize(CHOP_InfoDATSize *infoSize);
	virtual void		getInfoDATEntries(int index,
											int nEntries,
											CHOP_InfoDATEntries *entries);

	void worldSetup();

	void worldDestroy();

	void addBody(btVector3 pos, btVector3 rot, btVector3 scale, btScalar mass);

	void addPlane(btVector3 pos, btVector3 rot);

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
	const CHOP_NodeInfo		*myNodeInfo;

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

	btAlignedObjectArray<btCollisionShape*>	collisionShapes;

	
};
