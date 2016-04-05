
#ifndef BT_FRACTURE_BODY
#define BT_FRACTURE_BODY

class btCollisionShape;
class btDynamicsWorld;
class btCollisionWorld;
class btCompoundShape;
class btManifoldPoint;

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#define CUSTOM_FRACTURE_TYPE (btRigidBody::CO_USER_TYPE+1)



struct btConnection
{
	bool operator==(const btConnection& con)
	{
		return this->m_childIndex0 == con.m_childIndex0 &&
		       this->m_childIndex1 == con.m_childIndex1 &&
		       this->m_childShape0 == con.m_childShape0 &&
		       this->m_childShape1 == con.m_childShape1 &&
		       this->m_strength == con.m_strength;
	}

	btCollisionShape*	m_childShape0;
	btCollisionShape*	m_childShape1;
	int	m_childIndex0;
	int	m_childIndex1;
	btScalar	m_strength;
	//btVector3	m_location; //this must be local to the "Parent" compound
	//better calc this "live" from the objects themselves, so also keep ob pointers here
	btCollisionObject* m_obA;
	btCollisionObject* m_obB;
	btCollisionObject* m_parent;
	btScalar m_id;
};

struct btPropagationParameter
{
	btScalar m_impulse_dampening;
	btScalar m_directional_factor;
	btScalar m_minimum_impulse;
	btScalar m_stability_factor;
};

class btFractureBody : public btRigidBody
{
	//connections
public:

	btDynamicsWorld*	m_world;
	btAlignedObjectArray<btScalar>	m_masses;
	btAlignedObjectArray<btConnection> m_connections;
	btHashMap<btHashInt, btAlignedObjectArray<int> > *m_connection_map;
	btPropagationParameter m_propagationParameter;

	btFractureBody(	const btRigidBodyConstructionInfo& constructionInfo, btPropagationParameter parameter)
		:btRigidBody(constructionInfo),
		m_world(0),
		m_connection_map(NULL),
		m_propagationParameter(parameter)
	{
		m_masses.push_back(constructionInfo.m_mass);
		m_internalType=CUSTOM_FRACTURE_TYPE+CO_RIGID_BODY;
	}



	///btRigidBody constructor for backwards compatibility. 
	///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
	btFractureBody(	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia, btScalar* masses, int numMasses, btDynamicsWorld*	world, btPropagationParameter parameter)
		:btRigidBody(mass,motionState,collisionShape,localInertia),
		m_world(world),
		m_propagationParameter(parameter)
	{

		for (int i=0;i<numMasses;i++)
			m_masses.push_back(masses[i]);

		m_internalType=CUSTOM_FRACTURE_TYPE+CO_RIGID_BODY;
		m_connection_map = new btHashMap<btHashInt, btAlignedObjectArray<int> >();
	}

	void setWorld(btDynamicsWorld *world)
	{
		m_world = world;
	}

	void	recomputeConnectivity(btCollisionWorld* world);

	void	recomputeConnectivityByConstraints(btCollisionWorld* world);


	static btCompoundShape* shiftTransform(btCompoundShape* boxCompound,btScalar* masses,btTransform& shift, btVector3& principalInertia);
	
	static btCompoundShape* shiftTransformDistributeMass(btCompoundShape* boxCompound,btScalar mass,btTransform& shift);
	
	static bool collisionCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1);

	~btFractureBody()
	{
		delete m_connection_map;
	}
};


void fractureCallback(btDynamicsWorld* world, btScalar timeStep);
void glueCallback(btDynamicsWorld* world, btScalar timeStep);

#endif //BT_FRACTURE_BODY
