#ifndef _BT_FRACTURE_DYNAMICS_WORLD_H
#define _BT_FRACTURE_DYNAMICS_WORLD_H

#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btHashMap.h"
#include <string>
#include <sstream>

class btFractureBody;
class btCompoundShape;
class btTransform;
struct btPropagationParameter;

//callback to retrieve id from user application (blender)
typedef void (*IdCallback)(void *userPtr, int* object_id, int* shard_id);
typedef btRigidBody* (*ShapeBodyCallback)(void *userPtr);
char *to_str(int objectId, int shardId);

///The btFractureDynamicsWorld class enabled basic glue and fracture of objects. 
///If/once this implementation is stablized/tested we might merge it into btDiscreteDynamicsWorld and remove the class.
class btFractureDynamicsWorld : public btDiscreteDynamicsWorld
{
	btFractureBody* addNewBody(const btTransform& oldTransform, btScalar* masses, btCompoundShape* oldCompound, btPropagationParameter &param);

	bool	m_addBroadPhaseHandle;

public:

	btFractureDynamicsWorld ( btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration, IdCallback callback, ShapeBodyCallback shapebodycallback);

	~btFractureDynamicsWorld();

	btAlignedObjectArray<btCompoundConstraint*>	m_compoundConstraints;
	btAlignedObjectArray<btFractureBody*> m_fractureBodies;
	bool	m_fracturingMode;

	IdCallback m_idCallback;

	ShapeBodyCallback m_shapeBodyCallback;

	btHashMap<btHashString, int> *m_childIndexHash;

	virtual void	addRigidBody(btRigidBody* body);

	virtual void	removeRigidBody(btRigidBody* body);

	void	solveConstraints(btContactSolverInfo& solverInfo);

	virtual void addConstraint(btTypedConstraint *constraint, bool disableCollisionsBetweenLinkedBodies);

	virtual void removeConstraint(btTypedConstraint *constraint);

	///either fracture or glue (!fracture)
	void	setFractureMode(bool fracture)
	{
		m_fracturingMode = fracture;
	}

	bool getFractureMode() const { return m_fracturingMode;}

	///normally those callbacks are called internally by the 'solveConstraints'
	void glueCallback();

	///normally those callbacks are called internally by the 'solveConstraints'
	void fractureCallback();

	void updateBodies();

	void propagateDamage(btFractureBody *body, btScalar *impulse, int connection_index, bool *needsBreakingCheck,
	                     const btVector3 &direction, int *depth);

	void breakNeighborhood(btFractureBody *body, int connection_index);

	void	breakDisconnectedParts( btFractureBody* fracObj);

	virtual void updateAabbs();
};

#endif //_BT_FRACTURE_DYNAMICS_WORLD_H

