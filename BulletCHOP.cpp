#include "BulletCHOP.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
int
GetCHOPAPIVersion(void)
{
	// Always return CHOP_CPLUSPLUS_API_VERSION in this function.
	return CHOP_CPLUSPLUS_API_VERSION;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo *info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new BulletCHOP(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase *instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (BulletCHOP*)instance;
}

};


BulletCHOP::BulletCHOP(const OP_NodeInfo *info) : myNodeInfo(info)
{
	myExecuteCount = 0;
	pos = 0;
	ms = 0;

	worldSetup();

}

BulletCHOP::~BulletCHOP()
{

	worldDestroy();

}


void BulletCHOP::worldSetup(){

	collisionConfiguration = new btDefaultCollisionConfiguration;
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver;
	
	broadphase = new btDbvtBroadphase();
	
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	
	dynamicsWorld->setGravity(btVector3(0,-10,0));
	
    btContactSolverInfo& info = dynamicsWorld->getSolverInfo();
	info.m_splitImpulse = 1;
	info.m_splitImpulsePenetrationThreshold = -0.02;

	rigidBoxesShapes.resize(0);
	rigidSpheresShapes.resize(0);
	kineBoxesShapes.resize(0);
	staticPlanesShapes.resize(0);

	rigidBoxesIds.resize(0);
	rigidSpheresIds.resize(0);
	kineBoxesIds.resize(0);

}

void BulletCHOP::worldDestroy(){

	removeBodies();

	delete dynamicsWorld;
	delete solver;
	delete broadphase;
	delete dispatcher;
	delete collisionConfiguration;
}

void
BulletCHOP::getGeneralInfo(CHOP_GeneralInfo *ginfo)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->timeslice = false;
	//ginfo->inputMatchIndex = 0;
}

bool
BulletCHOP::getOutputInfo(CHOP_OutputInfo *info)
{

	info->numChannels = 11;
	info->sampleRate = 60;

	if (rigidBoxesIds.size()>0)
	{
		info->numSamples = rigidBoxesIds.size();
	}
	else
	{
		info->numSamples = 1;
	}

	return true;
}

const char*
BulletCHOP::getChannelName(int index, void* reserved)
{
	const char* name = "";

	switch (index) {
	case TX:
		name = "tx";
		break;
	case TY:
		name = "ty";
		break;
	case TZ:
		name = "tz";
		break;
	case RX:
		name = "rx";
		break;
	case RY:
		name = "ry";
		break;
	case RZ:
		name = "rz";
		break;
	case SPEED:
		name = "speed";
		break;
	case QX:
		name = "qx";
		break;
	case QY:
		name = "qy";
		break;
	case QZ:
		name = "qz";
		break;
	case QW:
		name = "qw";
		break;

	}

	return name;
}

/*void BulletCHOP::addBody(btVector3 pos, btVector3 rot, btVector3 scale, btScalar mass){

	btCollisionShape* colShape = new btBoxShape(0.5*scale);

	//btCollisionShape* colShape = new btSphereShape(0.5*scale.getX());

	collisionShapes.push_back(colShape);
		
	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass,localInertia);

	btMatrix3x3 rotMat;
	rotMat.setEulerZYX(rot.x(),rot.y(),rot.z());
		
	startTransform.setOrigin(pos);
	startTransform.setBasis(rotMat);

			
	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
					
	if(isDynamic == 0){
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		body->setActivationState(DISABLE_DEACTIVATION);
	}

	dynamicsWorld->addRigidBody(body);

}*/

void BulletCHOP::addKineBox(btVector3 pos, btVector3 rot, btVector3 scale) {

	btCollisionShape* colShape = new btBoxShape(0.5*scale);

	kineBoxesShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btVector3 localInertia(0, 0, 0);

	btMatrix3x3 rotMat;
	rotMat.setEulerZYX(rot.x(), rot.y(), rot.z());

	startTransform.setOrigin(pos);
	startTransform.setBasis(rotMat);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(0, myMotionState, colShape, localInertia);

	btRigidBody* body = new btRigidBody(rbInfo);

	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	body->setActivationState(DISABLE_DEACTIVATION);

	kineBoxesIds.push_back(dynamicsWorld->getCollisionObjectArray().size());

	dynamicsWorld->addRigidBody(body);

}

void BulletCHOP::addRigidSphere(btVector3 pos, btVector3 rot, btScalar radius, btScalar mass) {

	btCollisionShape* colShape = new btSphereShape(radius);

	rigidSpheresShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btVector3 localInertia(0, 0, 0);

	colShape->calculateLocalInertia(mass, localInertia);

	btMatrix3x3 rotMat;
	rotMat.setEulerZYX(rot.x(), rot.y(), rot.z());

	startTransform.setOrigin(pos);
	startTransform.setBasis(rotMat);


	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);

	btRigidBody* body = new btRigidBody(rbInfo);

	rigidSpheresIds.push_back(dynamicsWorld->getCollisionObjectArray().size());

	dynamicsWorld->addRigidBody(body);

}

void BulletCHOP::addRigidBox(btVector3 pos, btVector3 rot, btVector3 scale, btScalar mass) {

	btCollisionShape* colShape = new btBoxShape(0.5*scale);

	rigidBoxesShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btVector3 localInertia(0, 0, 0);

	colShape->calculateLocalInertia(mass, localInertia);

	btMatrix3x3 rotMat;
	rotMat.setEulerZYX(rot.x(), rot.y(), rot.z());

	startTransform.setOrigin(pos);
	startTransform.setBasis(rotMat);


	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);

	btRigidBody* body = new btRigidBody(rbInfo);

	rigidBoxesIds.push_back(dynamicsWorld->getCollisionObjectArray().size());

	dynamicsWorld->addRigidBody(body);

}

void BulletCHOP::addPlane(btVector3 planeNormal, btScalar planeConstant){

	
	btCollisionShape* groundShape = new btStaticPlaneShape(planeNormal, planeConstant);
	
	staticPlanesShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	//groundTransform.setOrigin(pos);
	//groundTransform.setOrigin(btVector3(0,-50,0));


	btScalar mass(0.);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static

	btVector3 localInertia(0,0,0);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	//add the body to the dynamics world
	dynamicsWorld->addRigidBody(body);
	

}

void BulletCHOP::removeBodies(){
	int i;
	for (i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<rigidSpheresShapes.size();j++)
	{
		delete rigidSpheresShapes[j];
	}
	rigidSpheresShapes.resize(0);
	rigidSpheresIds.resize(0);

	//delete collision shapes
	for (int j = 0; j<rigidBoxesShapes.size(); j++)
	{
		delete rigidBoxesShapes[j];
	}
	rigidBoxesShapes.resize(0);
	rigidBoxesIds.resize(0);

	//delete collision shapes
	for (int j = 0; j<kineBoxesShapes.size(); j++)
	{
		delete kineBoxesShapes[j];
	}
	kineBoxesShapes.resize(0);
	kineBoxesIds.resize(0);


}

void BulletCHOP::initRigidSpheres(OP_Inputs* inputs) {

	if (inputs->getParCHOP("Spheresrigidchop")) {

		const OP_CHOPInput* spheresRigidInput = inputs->getParCHOP("Spheresrigidchop");

		for (int i = 0; i < spheresRigidInput->numSamples; i++) {

			btVector3 pos = btVector3(
				spheresRigidInput->getChannelData(0)[i],
				spheresRigidInput->getChannelData(1)[i],
				spheresRigidInput->getChannelData(2)[i]);

			btVector3 rot = 0.017453*btVector3(
				spheresRigidInput->getChannelData(3)[i],
				spheresRigidInput->getChannelData(4)[i],
				spheresRigidInput->getChannelData(5)[i]);

			btScalar radius = spheresRigidInput->getChannelData(6)[i];

			addRigidSphere(pos, rot, radius, 1);
		}
	}
}

void BulletCHOP::initRigidBoxes(OP_Inputs* inputs) {

	if (inputs->getParCHOP("Boxesrigidchop")) {
		const OP_CHOPInput* boxesRigidInput = inputs->getParCHOP("Boxesrigidchop");

		for (int i = 0; i < boxesRigidInput->numSamples; i++) {

			btVector3 pos = btVector3(
				boxesRigidInput->getChannelData(0)[i],
				boxesRigidInput->getChannelData(1)[i],
				boxesRigidInput->getChannelData(2)[i]);

			btVector3 rot = 0.017453*btVector3(
				boxesRigidInput->getChannelData(3)[i],
				boxesRigidInput->getChannelData(4)[i],
				boxesRigidInput->getChannelData(5)[i]);

			btVector3 scale = btVector3(
				boxesRigidInput->getChannelData(6)[i],
				boxesRigidInput->getChannelData(7)[i],
				boxesRigidInput->getChannelData(8)[i]);

			addRigidBox(pos, rot, scale, 1);
		}
	}
}


void BulletCHOP::initKineBoxes(OP_Inputs* inputs) {

	if (inputs->getParCHOP("Boxeskinechop")) {
		const OP_CHOPInput* boxesKineInput = inputs->getParCHOP("Boxeskinechop");

		for (int i = 0; i < boxesKineInput->numSamples; i++) {

			btVector3 pos = btVector3(
				boxesKineInput->getChannelData(0)[i],
				boxesKineInput->getChannelData(1)[i],
				boxesKineInput->getChannelData(2)[i]);

			btVector3 rot = 0.017453*btVector3(
				boxesKineInput->getChannelData(3)[i],
				boxesKineInput->getChannelData(4)[i],
				boxesKineInput->getChannelData(5)[i]);

			btVector3 scale = btVector3(
				boxesKineInput->getChannelData(6)[i],
				boxesKineInput->getChannelData(7)[i],
				boxesKineInput->getChannelData(8)[i]);

			addKineBox(pos, rot, scale);
		}
	}
}

void BulletCHOP::updateKineBoxes(OP_Inputs* inputs) {

	if (inputs->getParCHOP("Boxeskinechop") && kineBoxesIds.size()>0) {

		const OP_CHOPInput* boxesKineInput = inputs->getParCHOP("Boxeskinechop");

		for (int i = 0; i < boxesKineInput->numSamples; i++) {

			int index = kineBoxesIds[i];

			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[index];
			btRigidBody* body = btRigidBody::upcast(obj);

			btVector3 pos = btVector3(
				boxesKineInput->getChannelData(0)[i],
				boxesKineInput->getChannelData(1)[i],
				boxesKineInput->getChannelData(2)[i]);

			btVector3 rot = 0.017453*btVector3(
				boxesKineInput->getChannelData(3)[i],
				boxesKineInput->getChannelData(4)[i],
				boxesKineInput->getChannelData(5)[i]);

			btTransform trans;

			trans.setOrigin(pos);

			btMatrix3x3 rotMat;
			rotMat.setEulerZYX(rot.x(), rot.y(), rot.z());

			trans.setBasis(rotMat);

			body->getMotionState()->setWorldTransform(trans);
		}
	}
}

void BulletCHOP::initPlanes(OP_Inputs* inputs) {

	if (inputs->getParCHOP("Colplaneschop")) {

		const OP_CHOPInput* colPlanesInput = inputs->getParCHOP("Colplaneschop");

		for (int i = 0; i < colPlanesInput->numSamples; i++) {

			btVector3 pos = btVector3(
				colPlanesInput->getChannelData(0)[i],
				colPlanesInput->getChannelData(1)[i],
				colPlanesInput->getChannelData(2)[i]);

		}
	}
}


void BulletCHOP::execute(const CHOP_Output* output,
								OP_Inputs* inputs,
									void* reserved)
{
	myExecuteCount++;

	double gravity[3];
	inputs->getParDouble3("Gravity", gravity[0], gravity[1], gravity[2]);

	dynamicsWorld->setGravity(btVector3(gravity[0], gravity[1], gravity[2]));

	int reset = inputs->getParInt("Reset");

	if (reset == 1) {

		removeBodies();

		frame = 0;

		//initRigidSpheres(inputs);
		initRigidBoxes(inputs);

		initKineBoxes(inputs);
		
	} 

	else 

	{

		updateKineBoxes(inputs);

		ms = getDeltaTimeMicroseconds() / 1000000.f;

		/*float maxSubsteps = float(inputs->getParInt("Maxsubsteps"));
		float fps = float(inputs->getParInt("Fps"));
		dynamicsWorld->stepSimulation(ms, maxSubsteps, 1.f / fps);*/

		float substeps = float(inputs->getParInt("Substeps"));
		float fps = float(inputs->getParInt("Fps"));

		btScalar elapsedTime = 1.0 / fps;
		btScalar fixedTimestep = elapsedTime / substeps;

		dynamicsWorld->stepSimulation(elapsedTime, int(substeps + 1), fixedTimestep);

		int iterations = inputs->getParInt("Iterations");
		dynamicsWorld->getSolverInfo().m_numIterations = iterations;

		for (int i = 0; i < rigidBoxesIds.size(); i++){

			int index = rigidBoxesIds[i];

			btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[index];
			btRigidBody* body = btRigidBody::upcast(obj);

			btTransform trans;
			body->getMotionState()->getWorldTransform(trans);
	
			output->channels[TX][i] = float(trans.getOrigin().getX());
			output->channels[TY][i] = float(trans.getOrigin().getY());
			output->channels[TZ][i] = float(trans.getOrigin().getZ());

			btScalar rotX, rotY, rotZ;

			//caution about X and Z
			trans.getBasis().getEulerZYX(rotZ,rotY,rotX);

			output->channels[RX][i] = float(rotX)*57.2958;
			output->channels[RY][i] = float(rotY)*57.2958;
			output->channels[RZ][i] = float(rotZ)*57.2958;

			btVector3 vel = body->getLinearVelocity();

			output->channels[SPEED][i] = pow(vel.x(),2)+pow(vel.y(),2)+pow(vel.z(),2);

			btQuaternion rot = trans.getRotation();

			output->channels[QX][index] = float(rot.x());
			output->channels[QY][index] = float(rot.y());
			output->channels[QZ][index] = float(rot.z());
			output->channels[QW][index] = float(rot.w());

		}

	}
	
}

int
BulletCHOP::getNumInfoCHOPChans()
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP. In this example we are just going to send one channel.
	return 2;
}

void
BulletCHOP::getInfoCHOPChan(int index, OP_InfoCHOPChan* chan)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.

	if (index == 0)
	{
		chan->name = "executeCount";
		chan->value = myExecuteCount;
	}

		if (index == 1)
	{
		chan->name = "ms";
		chan->value = ms;
	}
}

bool		
BulletCHOP::getInfoDATSize(OP_InfoDATSize *infoSize)
{
	infoSize->rows = 2;
	infoSize->cols = 2;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
BulletCHOP::getInfoDATEntries(int index,
										int nEntries,
										OP_InfoDATEntries *entries)
{
	if (index == 0)
	{
		// It's safe to use static buffers here because Touch will make it's own
		// copies of the strings immediately after this call returns
		// (so the buffers can be reuse for each column/row)
		static char tempBuffer1[4096];
		static char tempBuffer2[4096];
		// Set the value for the first column
		strcpy(tempBuffer1, "executeCount");
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
		sprintf(tempBuffer2, "%d", myExecuteCount);
		entries->values[1] = tempBuffer2;


	}

	if (index == 1)
	{
		// It's safe to use static buffers here because Touch will make it's own
		// copies of the strings immediately after this call returns
		// (so the buffers can be reuse for each column/row)
		static char tempBuffer1[4096];
		static char tempBuffer2[4096];
		// Set the value for the first column
		strcpy(tempBuffer1, "ms");
		entries->values[0] = tempBuffer1;

		// Set the value for the second column
		sprintf(tempBuffer2, "%f", ms);
		entries->values[1] = tempBuffer2;


	}
}

void BulletCHOP::setupParameters(OP_ParameterManager* manager)
{
	
	// Reset
	{
		OP_NumericParameter	np;

		np.name = "Reset";
		np.label = "Reset";
		np.defaultValues[0] = 0.0;

		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}


	// Maxsubsteps
	/*{
		OP_NumericParameter	np;

		np.name = "Maxsubsteps";
		np.label = "Max Substeps";
		//np.page = "Solver";
		np.defaultValues[0] = 10;

		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}*/

	// Fps
	{
		OP_NumericParameter	np;

		np.name = "Fps";
		np.label = "FPS";
		//np.page = "Solver";
		np.defaultValues[0] = 60;

		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Substeps
	{
		OP_NumericParameter	np;

		np.name = "Substeps";
		np.label = "Substeps";
		//np.page = "Solver";
		np.defaultValues[0] = 1;


		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Iterations
	{
		OP_NumericParameter	np;

		np.name = "Iterations";
		np.label = "Iterations";
		//np.page = "Solver";
		np.defaultValues[0] = 10;

		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}
	
	//Gravity
	{
		OP_NumericParameter	np;

		np.name = "Gravity";
		np.label = "Gravity";

		np.defaultValues[0] = 0.0;
		np.defaultValues[1] = -9.8;
		np.defaultValues[2] = 0.0;


		OP_ParAppendResult res = manager->appendXYZ(np);
		assert(res == OP_ParAppendResult::Success);
	}


	//Spheresrigidbodieschop
	/*{
		OP_StringParameter sp;

		sp.name = "Spheresrigidchop";
		sp.label = "Spheres Rigid Bodies CHOP";
		//sp.page = "Collisions";

		OP_ParAppendResult res = manager->appendCHOP(sp);
		assert(res == OP_ParAppendResult::Success);
	}*/

	//Boxesrigidchop
	{
	OP_StringParameter sp;

	sp.name = "Boxesrigidchop";
	sp.label = "Boxes Rigid Bodies CHOP";
	//sp.page = "Collisions";

	OP_ParAppendResult res = manager->appendCHOP(sp);
	assert(res == OP_ParAppendResult::Success);
	}

	//Boxeskinebodieschop
	{
		OP_StringParameter sp;

		sp.name = "Boxeskinechop";
		sp.label = "Boxes Kinematic Bodies CHOP";
		//sp.page = "Collisions";

		OP_ParAppendResult res = manager->appendCHOP(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	//Colplaneschop
	/*{
	OP_StringParameter sp;

	sp.name = "Colplaneschop";
	sp.label = "Collision Planes CHOP";
	sp.page = "Collisions";

	OP_ParAppendResult res = manager->appendCHOP(sp);
	assert(res == OP_ParAppendResult::Success);
	}*/

}

void BulletCHOP::pulsePressed(const char* name)
{
	/*if (!strcmp(name, "Reset"))
	{
		myOffset = 0.0;
	}*/
}
