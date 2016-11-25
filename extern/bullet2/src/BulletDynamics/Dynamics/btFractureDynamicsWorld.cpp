
#include <cstdlib>
#include <ctime>

#include "btBulletDynamicsCommon.h"
#include "btFractureDynamicsWorld.h"
#include "btFractureBody.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include "BulletCollision/CollisionDispatch/btUnionFind.h"

btFractureDynamicsWorld::btFractureDynamicsWorld (btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration, IdCallback callback, ShapeBodyCallback shapebodycallback)
:btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration),
m_fracturingMode(true),
m_idCallback(callback),
m_shapeBodyCallback(shapebodycallback),
m_addBroadPhaseHandle(false)
{
	m_childIndexHash = new btHashMap<btHashString, int>();
}

char* to_str(int objectId, int shardId)
{
	std::ostringstream oss;
	oss << objectId << shardId;
	return strdup(oss.str().c_str());
}

btFractureDynamicsWorld::~btFractureDynamicsWorld()
{
	delete m_childIndexHash;

	//clean up remaining objects (before inefficient collision world destructor gets run... ?)
	int i;

	// first just clear ALL pairs
	btOverlappingPairCache *paircache = getBroadphase()->getOverlappingPairCache();
	btBroadphasePairArray& pairs = paircache->getOverlappingPairArray();

	//printf("Pair size: %d\n", pairs.size());
	for (i=0;i<pairs.size();i++)
	{
		paircache->cleanOverlappingPair(pairs[i], m_dispatcher1);
		paircache->removeOverlappingPair(pairs[i].m_pProxy0, pairs[i].m_pProxy1, m_dispatcher1);
	}

	// then all proxies, should be 2*N instead of N^2
	for (i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* collisionObject= m_collisionObjects[i];

		btBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
		if (bp)
		{
			//
			// only clear the cached algorithms
			//
			//getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
			//printf("Destroy Proxy: %d\n", i);
			getBroadphase()->destroyProxy(bp,m_dispatcher1);
			collisionObject->setBroadphaseHandle(0);
		}
	}
}

void btFractureDynamicsWorld::updateBodies()
{
	if (m_idCallback && m_shapeBodyCallback)
	{
		btFractureDynamicsWorld *fworld = (btFractureDynamicsWorld*)this;
		int i = 0, size = fworld->m_fractureBodies.size();
		for (i=0;i<size;i++)
		{
			btFractureBody* body = fworld->m_fractureBodies[i];
			if (body->getCollisionShape()->isCompound())
			{
				btCompoundShape *shape = (btCompoundShape*)body->getCollisionShape();
				int j, num_children = shape->getNumChildShapes();
				for (j=0;j<num_children;j++)
				{
					btCollisionShape *cshape = (btCollisionShape*)shape->getChildShape(j);
					//btTransform trans = shape->getChildTransform(j) * body->getWorldTransform();

					//find out to which body the shape belonged originally... and update it

					int objectIndexA, shardIndexA; //, objectIndexB, shardIndexB;
					m_idCallback(cshape->getUserPointer(), &objectIndexA, &shardIndexA);
					//m_idCallback(body->getUserPointer(), &objectIndexB, &shardIndexB );

					m_childIndexHash->insert(to_str(objectIndexA, shardIndexA), j);

					//if ((objectIndexA == objectIndexB) && (shardIndexA != shardIndexB))
					btTransform trans;
					btRigidBody *fbody = m_shapeBodyCallback(cshape->getUserPointer());

					trans = body->getWorldTransform()*shape->getChildTransform(j);
					fbody->setWorldTransform(trans);
				}
			}
			else
			{
				btCollisionShape *cshape = (btCollisionShape*)body->getCollisionShape();

				//non compounds have a child index of -1, (and will have a broadphase handle)
				int objectIndexA, shardIndexA;
				m_idCallback(cshape->getUserPointer(), &objectIndexA, &shardIndexA);
				m_childIndexHash->insert(to_str(objectIndexA, shardIndexA), -1);
			}
		}
	}
}

void	btFractureDynamicsWorld::updateAabbs()
{
	BT_PROFILE("updateAabbs");

	//btTransform predictedTrans;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btCollisionObject* colObj = m_collisionObjects[i];

		//only update aabb of active objects
		if ((m_forceUpdateAllAabbs || colObj->isActive()) && colObj->getBroadphaseHandle() != NULL)
		{
			updateSingleAabb(colObj);
		}
	}
}


void btFractureDynamicsWorld::glueCallback()
{

	int numManifolds = getDispatcher()->getNumManifolds();

	///first build the islands based on axis aligned bounding box overlap


	//btAlignedObjectArray<int> objectIds;
	//btHashMap<btHashInt, btAlignedObjectArray<int> >objectMap;

	int index = 0;
	{
		int i;
		for (i=0;i<getCollisionObjectArray().size(); i++)
		{
			btCollisionObject*   collisionObject= getCollisionObjectArray()[i];
		//	btRigidBody* body = btRigidBody::upcast(collisionObject);
			//Adding filtering here
#ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION
//			if (!collisionObject->isStaticOrKinematicObject())
			if (collisionObject->getInternalType() & CUSTOM_FRACTURE_TYPE)
			{
				//int objectId, shardId;
				collisionObject->setIslandTag(index++);
				/*btFractureBody *body = (btFractureBody*)collisionObject;
				m_idCallback(body->getUserPointer(),&objectId, &shardId);
				if (objectId > -1)
				{
					btAlignedObjectArray<int> *objects = objectMap.find(objectId);
					if (!objects)
					{
						btAlignedObjectArray<int> obj;
						obj.push_back(i);
						objectMap.insert(objectId, obj);
						objectIds.push_back(objectId);
					}
					else
					{
						objects->push_back(i);
					}
				}*/
			}
			else
			{
				collisionObject->setIslandTag(-1);
			}
#else
			collisionObject->setIslandTag(i);
			index=i+1;
#endif
		}
	}
	btUnionFind unionFind;
	unionFind.reset(index);

	//int numElem = unionFind.getNumElements();

	printf("Manifolds: %d\n", numManifolds);

	//if (numManifolds > 0)
	{
		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* manifold = getDispatcher()->getManifoldByIndexInternal(i);
#if 0
			if (!manifold->getNumContacts())
			{
				//printf("Manifold %d has NO CONTACT POINT!!\n", i);
			//	continue;
			}

			btScalar minDist = 1e30f;
			for (int v=0;v<manifold->getNumContacts();v++)
			{
				minDist = btMin(minDist,manifold->getContactPoint(v).getDistance());
			}
			if (minDist>0.f)
				continue;
#endif

			btCollisionObject* colObj0 = (btCollisionObject*)manifold->getBody0();
			btCollisionObject* colObj1 = (btCollisionObject*)manifold->getBody1();
			int tag0 = (colObj0)->getIslandTag();
			int tag1 = (colObj1)->getIslandTag();

			btRigidBody* body0 = btRigidBody::upcast(colObj0);
			btRigidBody* body1 = btRigidBody::upcast(colObj1);

			int objectId0, objectId1, shardId0, shardId1;
			m_idCallback(body0->getUserPointer(),&objectId0, &shardId0);
			m_idCallback(body1->getUserPointer(),&objectId1, &shardId1);

			//if (!colObj0->isStaticOrKinematicObject() && !colObj1->isStaticOrKinematicObject())
			if (objectId0 == objectId1 && objectId0 > -1 && tag0 > -1 && tag1 > -1)
			{
				unionFind.unite(tag0, tag1);
			}
		}
	}
#if 0
	//else //fallback if no manifolds are there (kinematic)
	{
		int numConstraints = m_compoundConstraints.size();
		for (int i=0;i<numConstraints;i++)
		{
			btTypedConstraint *con = m_compoundConstraints[i];

			btCollisionObject* colObj0 = (btCollisionObject*)&con->getRigidBodyA();
			btCollisionObject* colObj1 = (btCollisionObject*)&con->getRigidBodyB();
			int tag0 = (colObj0)->getIslandTag();
			int tag1 = (colObj1)->getIslandTag();
			btFractureBody* body0 = (btFractureBody*)colObj0;
			btFractureBody* body1 = (btFractureBody*)colObj1;

			int objectId0, shardId0, objectId1, shardId1;
			m_idCallback(body0->getUserPointer(),&objectId0, &shardId0);
			m_idCallback(body1->getUserPointer(),&objectId1, &shardId1);

			//if (!colObj0->isStaticOrKinematicObject() && !colObj1->isStaticOrKinematicObject())
			if (objectId0 == objectId1 && objectId0 > -1)
			{
				unionFind.unite(tag0, tag1);
			}
		}
	}
#endif

#if 0
	for (int ai=0;ai<objectIds.size();ai++)
	{
		btUnionFind unionFind;
		btAlignedObjectArray<int> *objects = objectMap.find(objectIds[ai]);
		unionFind.reset(index);

		for (int j=0;j<objects->size();j++)
		{
			btCollisionObject* collisionObject = getCollisionObjectArray()[objects->at(j)];
			if (collisionObject->getInternalType() & CUSTOM_FRACTURE_TYPE)
			{
				//ensure 1 compound per object, so shard id 0 becomes parent always.... sure that it is first ?
				int objectId, shardId, islandTag;
				islandTag = collisionObject->getIslandTag();
				btFractureBody *body = (btFractureBody*)collisionObject;
				m_idCallback(body->getUserPointer(),&objectId, &shardId);
				if (objectId > -1 /*&& islandTag < objects->size()*/)
				{
					unionFind.unite(objectId, islandTag);
				}
			}
		}
#endif

	int numElem = unionFind.getNumElements();

	index=0;
	for (int ai=0;ai<getCollisionObjectArray().size();ai++)
	{
		btCollisionObject* collisionObject=  getCollisionObjectArray()[ai];
		//if (!collisionObject->isStaticOrKinematicObject())
		if (collisionObject->getInternalType() & CUSTOM_FRACTURE_TYPE)
		{
			int tag = unionFind.find(index);

			collisionObject->setIslandTag( tag);

			//Set the correct object offset in Collision Object Array
#if STATIC_SIMULATION_ISLAND_OPTIMIZATION
			unionFind.getElement(index).m_sz = ai;
#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION

			index++;
		}
	}
	unionFind.sortIslands();



	int endIslandIndex=1;
	int startIslandIndex;

	btAlignedObjectArray<btCollisionObject*> removedObjects;
	btAlignedObjectArray<btCollisionObject*> newObjects;

	///iterate over all islands
	for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
	{
		int islandId = unionFind.getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (unionFind.getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
		{
		}

		int fractureObjectIndex = -1;

		int numObjects=0;

		int idx;
		for (idx=startIslandIndex;idx<endIslandIndex;idx++)
		{
			int i = unionFind.getElement(idx).m_sz;

			/*if (i >= getCollisionObjectArray().size())
			{
				fractureObjectIndex = -1;
				continue;
			}*/


			btCollisionObject* colObj0 = getCollisionObjectArray()[i];
			/*if (colObj0->getInternalType()& CUSTOM_FRACTURE_TYPE)
			{*/
				fractureObjectIndex = i;
			/*}*
			else
			{
				fractureObjectIndex = -1;
				continue;
			}*/

			btRigidBody* otherObject = btRigidBody::upcast(colObj0);
			if (!otherObject || !otherObject->getInvMass()) /*!(otherObject->getInternalType() & CUSTOM_FRACTURE_TYPE))*/
			{
				printf("Col Obj %d NOT THERE ?!\n", i);
				continue;
			}

			numObjects++;
		}

		///Then for each island that contains at least two objects and one fracture object
		if (fractureObjectIndex>=0 && numObjects>1)
		{

			btFractureBody* fracObj = (btFractureBody*)getCollisionObjectArray()[fractureObjectIndex];

			///glueing objects means creating a new compound and removing the old objects
			///delay the removal of old objects to avoid array indexing problems
			removedObjects.push_back(fracObj);
			//m_fractureBodies.remove(fracObj);

			btAlignedObjectArray<btScalar> massArray;

			btAlignedObjectArray<btVector3> oldImpulses;
			btAlignedObjectArray<btVector3> oldCenterOfMassesWS;

			if (!fracObj->getInvMass())
			{
				oldImpulses.push_back(btVector3(0, 0, 0));
			}
			else
			{
				oldImpulses.push_back(fracObj->getLinearVelocity()/1./fracObj->getInvMass());
			}

			oldCenterOfMassesWS.push_back(fracObj->getCenterOfMassPosition());

			btScalar totalMass = 0.f;


			btCompoundShape* compound = new btCompoundShape(false);
			if (fracObj->getCollisionShape()->isCompound())
			{
				btTransform tr;
				tr.setIdentity();
				btCompoundShape* oldCompound = (btCompoundShape*)fracObj->getCollisionShape();
				for (int c=0;c<oldCompound->getNumChildShapes();c++)
				{
					compound->addChildShape(oldCompound->getChildTransform(c),oldCompound->getChildShape(c));
					massArray.push_back(fracObj->m_masses[c]);
					totalMass+=fracObj->m_masses[c];
				}

			} else
			{
				btTransform tr;
				tr.setIdentity();
				compound->addChildShape(tr,fracObj->getCollisionShape());
				massArray.push_back(fracObj->m_masses[0]);
				totalMass+=fracObj->m_masses[0];
			}

			for (idx=startIslandIndex;idx<endIslandIndex;idx++)
			{

				int i = unionFind.getElement(idx).m_sz;

				if (i==fractureObjectIndex)
					continue;

				btCollisionObject* otherCollider = getCollisionObjectArray()[i];
				btRigidBody* otherObject = btRigidBody::upcast(otherCollider);

				//don't glue/merge with static objects right now, otherwise everything gets stuck to the ground
				///todo: expose this as a callback
				if (!otherObject || !otherObject->getInvMass())/*|| !(otherObject->getInternalType() & CUSTOM_FRACTURE_TYPE)*/
				{
					continue;
				}


				if (!otherObject->getInvMass())
				{
					oldImpulses.push_back(btVector3(0, 0, 0));
				}
				else
				{
					oldImpulses.push_back(otherObject->getLinearVelocity()*(1.f/otherObject->getInvMass()));
				}
				oldCenterOfMassesWS.push_back(otherObject->getCenterOfMassPosition());

				removedObjects.push_back(otherObject);
				//m_fractureBodies.remove((btFractureBody*)otherObject);

				btScalar curMass = 0.f;
				if (otherObject->getInvMass())
					curMass = 1.f/otherObject->getInvMass();


				if (otherObject->getCollisionShape()->isCompound())
				{
					btTransform tr;
					btCompoundShape* oldCompound = (btCompoundShape*)otherObject->getCollisionShape();
					for (int c=0;c<oldCompound->getNumChildShapes();c++)
					{
						tr = fracObj->getWorldTransform().inverseTimes(
								 otherObject->getWorldTransform()*oldCompound->getChildTransform(c));
						compound->addChildShape(tr,oldCompound->getChildShape(c));
						massArray.push_back(curMass/(btScalar)oldCompound->getNumChildShapes());

					}
				} else
				{
					btTransform tr;
					tr = fracObj->getWorldTransform().inverseTimes(otherObject->getWorldTransform());
					compound->addChildShape(tr,otherObject->getCollisionShape());
					massArray.push_back(curMass);
				}
				totalMass+=curMass;
			}


			//btFractureBody* fracObj = (btFractureBody*)getCollisionObjectArray()[fractureObjectIndex];

			btTransform shift;
			shift.setIdentity();
			btCompoundShape* newCompound = btFractureBody::shiftTransformDistributeMass(compound,totalMass,shift);
			int numChildren = newCompound->getNumChildShapes();
			btAssert(numChildren == massArray.size());

			btVector3 localInertia;
			newCompound->calculateLocalInertia(totalMass,localInertia);
			btFractureBody* newBody = new btFractureBody(totalMass,0,newCompound,localInertia, &massArray[0],
														 numChildren,this, fracObj->m_propagationParameter);
			//newBody->recomputeConnectivity(this);
			//newBody->recomputeConnectivityByConstraints(this);
			newBody->setWorldTransform(fracObj->getWorldTransform()*shift);

			int objectIndex, shardIndex;
			//pass user pointer from old compound parent to new one
			newBody->setUserPointer(fracObj->getUserPointer());
			m_idCallback(newBody->getUserPointer(), &objectIndex, &shardIndex);

			for (int i=0; i<numChildren; i++)
			{
				int obIndex, shIndex;
				btCollisionShape *cshape = newCompound->getChildShape(i);
				m_idCallback(cshape->getUserPointer(), &obIndex, &shIndex);
				m_childIndexHash->insert(to_str(obIndex, shIndex), i);
			}

			newBody->recomputeConnectivityByConstraints(this);

			//now the linear/angular velocity is still zero, apply the impulses

			for (int i=0;i<oldImpulses.size();i++)
			{
				btVector3 rel_pos = oldCenterOfMassesWS[i]-newBody->getCenterOfMassPosition();
				const btVector3& imp = oldImpulses[i];
				newBody->applyImpulse(imp, rel_pos);
			}

			m_addBroadPhaseHandle = true;
			addRigidBody(newBody);
			m_addBroadPhaseHandle = false;

			//newbody is a compound parent, hmmmm, so set its childindex to 0 or -1
			m_childIndexHash->insert(to_str(objectIndex, shardIndex), -1);

			newObjects.push_back(newBody);

		}
	}

	//HERE remove ALL pairs where proxies are NOT in
	btOverlappingPairCache *paircache = getBroadphase()->getOverlappingPairCache();
	btBroadphasePairArray& pairs = paircache->getOverlappingPairArray();

	//printf("Pair size: %d\n", pairs.size());
	for (int i=0;i<pairs.size();i++)
	{
		bool isValid = true;
		for (int j=0; j<newObjects.size();j++)
		{
			btBroadphaseProxy* bp = newObjects[j]->getBroadphaseHandle();

			if (bp == pairs[i].m_pProxy0 || bp == pairs[i].m_pProxy1)
			{
				isValid = false;
				break;
			}
		}

		if (!isValid)
		{
			paircache->cleanOverlappingPair(pairs[i], m_dispatcher1);
			paircache->removeOverlappingPair(pairs[i].m_pProxy0, pairs[i].m_pProxy1, m_dispatcher1);
		}
	}

	//remove the objects from the world at the very end,
	//otherwise the island tags would not match the world collision object array indices anymore
	while (removedObjects.size())
	{
		btCollisionObject* otherCollider = removedObjects[removedObjects.size()-1];
		removedObjects.pop_back();

		btRigidBody* otherObject = btRigidBody::upcast(otherCollider);
		if (!otherObject /* || !otherObject->getInvMass()*/)
			continue;

		btBroadphaseProxy* bp = otherCollider->getBroadphaseHandle();
		if (bp)
		{
			//printf("Destroy Proxy: %d\n", i);
			getBroadphase()->destroyProxy(bp,m_dispatcher1);
			otherCollider->setBroadphaseHandle(0);
		}

		removeRigidBody(otherObject);
	}
}


struct	btFracturePair
{
	btFractureBody* m_fracObj;
	btAlignedObjectArray<int>	m_contactManifolds;
};



void btFractureDynamicsWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	// todo: after fracture we should run the solver again for better realism
	// for example
	//	save all velocities and if one or more objects fracture:
	//	1) revert all velocties
	//	2) apply impulses for the fracture bodies at the contact locations
	//	3)and run the constaint solver again

	btDiscreteDynamicsWorld::solveConstraints(solverInfo);

	//hmm maybe skip CompoundConstraints here :) (and solve regular ones only)
	//just remove from regular solver list and add to compound "solver" list
	//or dont enter into regular list but other list :
	//keep track in "constraints" via set enabled, disabled, and skip in next re-compounding iteration
	//avoiding N^2 searches

	//use compound children for drawing meshislands and filling the cache (in blender)

	fractureCallback();
}

btFractureBody* btFractureDynamicsWorld::addNewBody(const btTransform& oldTransform,btScalar* masses, btCompoundShape* oldCompound, btPropagationParameter& param)
{
	int i;

	btTransform shift;
	shift.setIdentity();
	btVector3 localInertia;
	btCompoundShape* newCompound = btFractureBody::shiftTransform(oldCompound,masses,shift,localInertia);
	btScalar totalMass = 0;
	for (i=0;i<newCompound->getNumChildShapes();i++)
		totalMass += masses[i];
	newCompound->calculateLocalInertia(totalMass,localInertia);

	btFractureBody* newBody = new btFractureBody(totalMass,0,newCompound,localInertia, masses,newCompound->getNumChildShapes(), this, param);
	//newBody->recomputeConnectivity(this);
	newBody->recomputeConnectivityByConstraints(this);

	newBody->setCollisionFlags(newBody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	newBody->setWorldTransform(oldTransform*shift);
	m_addBroadPhaseHandle = true;
	addRigidBody(newBody);
	m_addBroadPhaseHandle = false;
	return newBody;
}

void btFractureDynamicsWorld::addConstraint(btTypedConstraint *constraint, bool disableCollisionsBetweenLinkedBodies)
{
	if (constraint->getConstraintType() == COMPOUND_CONSTRAINT_TYPE)
	{
		//keep those dummy constraints away from solver, they are just there to track their breaking state manually
		btCompoundConstraint *con = (btCompoundConstraint*)constraint;
		m_compoundConstraints.push_back(con);
		//constraint->getRigidBodyA().addConstraintRef(constraint);
		//constraint->getRigidBodyB().addConstraintRef(constraint);
	}
	else
	{
		btDiscreteDynamicsWorld::addConstraint(constraint, disableCollisionsBetweenLinkedBodies);
	}
}

void btFractureDynamicsWorld::removeConstraint(btTypedConstraint *constraint)
{
	if (constraint->getConstraintType() == COMPOUND_CONSTRAINT_TYPE)
	{
		btCompoundConstraint *con = (btCompoundConstraint*)constraint;
		m_compoundConstraints.remove(con);
		//constraint->getRigidBodyA().removeConstraintRef(constraint);
		//constraint->getRigidBodyB().removeConstraintRef(constraint);
	}
	else
	{
		btDiscreteDynamicsWorld::removeConstraint(constraint);
	}
}


void btFractureDynamicsWorld::addRigidBody(btRigidBody* body)
{
	bool addBroadPhaseHandle = true;

	if (body->getInternalType() & CUSTOM_FRACTURE_TYPE)
	{
		int objectId, shardId;
		btFractureBody* fbody = (btFractureBody*)body;
		m_fractureBodies.push_back(fbody);
		m_idCallback(fbody->getUserPointer(), &objectId, &shardId);
		//if (objectId > 0 && shardId > 1)
		//	addBroadPhaseHandle = false;
	}

	//m_addBroadPhaseHandle is an override switch (for new, fractured objects)
	if (addBroadPhaseHandle || m_addBroadPhaseHandle)
	{
		btDiscreteDynamicsWorld::addRigidBody(body);
	}
	else
	{
		//inlined from DiscreteDynamicsWorld::addRigidbody(if broadphase handle is omitted)
		if (!body->isStaticOrKinematicObject() && !(body->getFlags() &BT_DISABLE_WORLD_GRAVITY))
		{
			body->setGravity(m_gravity);
		}

		if (body->getCollisionShape())
		{
			if (!body->isStaticObject())
			{
				m_nonStaticRigidBodies.push_back(body);
			} else
			{
				body->setActivationState(ISLAND_SLEEPING);
			}

			btCollisionObject* collisionObject = (btCollisionObject*)body;

			//its inside a compound and collision is disabled, so need no broadphase handle for children
			//removing the broadphase stuff is N^2 and slow, only adding if necessary !
			collisionObject->setBroadphaseHandle(NULL);

			btAssert( m_collisionObjects.findLinearSearch(collisionObject)  == m_collisionObjects.size());
			m_collisionObjects.push_back(collisionObject);
		}
	}
}

void	btFractureDynamicsWorld::removeRigidBody(btRigidBody* body)
{
	if (body->getInternalType() & CUSTOM_FRACTURE_TYPE)
	{
		btFractureBody* fbody = (btFractureBody*)body;
		btAlignedObjectArray<btTypedConstraint*> tmpConstraints;

		for (int i=0;i<fbody->getNumConstraintRefs();i++)
		{
			tmpConstraints.push_back(fbody->getConstraintRef(i));
		}

		//remove all constraints attached to this rigid body too		
		for (int i=0;i<tmpConstraints.size();i++)
			/*btDiscreteDynamicsWorld::*/removeConstraint(tmpConstraints[i]);

		//m_fractureBodies.remove(fbody);

		//m_nonStaticRigidBodies.remove(body);
		//removeCollisionObject(body);
	}
	/*else
	{*/
		btDiscreteDynamicsWorld::removeRigidBody(body);
	//}
}

void	btFractureDynamicsWorld::breakDisconnectedParts( btFractureBody* fracObj)
{

	if (!fracObj->getCollisionShape()->isCompound())
		return;

	btCompoundShape* compound = (btCompoundShape*)fracObj->getCollisionShape();
	int numChildren = compound->getNumChildShapes();

	if (numChildren<=1)
		return;

	//compute connectivity
	btUnionFind unionFind;

	btAlignedObjectArray<int> tags;
	tags.resize(numChildren);
	int i, index = 0;
	for ( i=0;i<numChildren;i++)
	{
#ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION
		tags[i] = index++;
#else
		tags[i] = i;
		index=i+1;
#endif
	}

	unionFind.reset(index);
	//int numElem = unionFind.getNumElements();
	for (i=0;i<fracObj->m_connections.size();i++)
	{
		btConnection connection = fracObj->m_connections[i];
		if (connection.m_childIndex0 >= tags.size() ||
		    connection.m_childIndex1 >= tags.size() )
		{
			//fracObj->m_connections.remove(connection);
			continue;
		}

		if (connection.m_strength > 0.)
		{
			int tag0 = tags[connection.m_childIndex0];
			int tag1 = tags[connection.m_childIndex1];

			//printf("Uniting: %d %d\n",connection.m_childIndex0, connection.m_childIndex1);
			unionFind.unite(tag0, tag1);
		}
	}
	int numElem = unionFind.getNumElements();

	index=0;
	for (int ai=0;ai<numChildren;ai++)
	{
		int tag = unionFind.find(index);
		tags[ai] = tag;
		//Set the correct object offset in Collision Object Array
#if STATIC_SIMULATION_ISLAND_OPTIMIZATION
		unionFind.getElement(index).m_sz = ai;
#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION
		index++;
	}
	unionFind.sortIslands();

	int endIslandIndex=1;
	int startIslandIndex;

	btAlignedObjectArray<btCollisionObject*> removedObjects;

	int numIslands = 0;

	for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
	{
		int islandId = unionFind.getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (unionFind.getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
		{
		}

	//	int fractureObjectIndex = -1;

		int numShapes=0;


		btCompoundShape* newCompound = new btCompoundShape(false);
		btAlignedObjectArray<btScalar> masses;

		int idx;
		for (idx=startIslandIndex;idx<endIslandIndex;idx++)
		{
			int i = unionFind.getElement(idx).m_sz;
	//		btCollisionShape* shape = compound->getChildShape(i);

			newCompound->addChildShape(compound->getChildTransform(i),compound->getChildShape(i));
			masses.push_back(fracObj->m_masses[i]);
			numShapes++;
		}
		if (numShapes)
		{
			int objectIndex, shardIndex;
			btFractureBody* newBody = addNewBody(fracObj->getWorldTransform(),&masses[0],newCompound, fracObj->m_propagationParameter);
			newBody->setLinearVelocity(fracObj->getLinearVelocity());
			newBody->setAngularVelocity(fracObj->getAngularVelocity());

			//pass user pointer from old compound parent to new one
			newBody->setUserPointer(fracObj->getUserPointer());
			m_idCallback(newBody->getUserPointer(), &objectIndex, &shardIndex);
			m_childIndexHash->insert(to_str(objectIndex, shardIndex), -1);

			int numChildren = newCompound->getNumChildShapes();

			for (int i=0; i<numChildren; i++)
			{
				int obIndex, shIndex;
				btCollisionShape *cshape = newCompound->getChildShape(i);
				m_idCallback(cshape->getUserPointer(), &obIndex, &shIndex);
				m_childIndexHash->insert(to_str(obIndex, shardIndex), i);
			}

			numIslands++;
		}
	}





	removeRigidBody(fracObj);//should it also be removed from the array?


}

#include <stdio.h>

#if 0
void btFractureDynamicsWorld::propagateDamage(btFractureBody *body, btScalar *impulse, int connection_index,
                                              bool* needsBreakingCheck, const btVector3& direction, int* depth)
{
	if (*depth == 0)
		return;

	(*depth)--;

	//min break impulse, todo expose
	if (/*body->m_propagationParameter.m_minimum_impulse > 0 &&
	   (*impulse > body->m_propagationParameter.m_minimum_impulse) &&*/
	   (body->m_connections.size() > connection_index))
	{
		btConnection& connection = body->m_connections[connection_index];
		btCollisionShape* shape0 = connection.m_childShape0;
		btCollisionShape* shape1 = connection.m_childShape1;

		btRigidBody *fbody0 = m_shapeBodyCallback(shape0->getUserPointer());
		btRigidBody *fbody1 = m_shapeBodyCallback(shape1->getUserPointer());

		//calculate dot product between body->fbody and direction, if > say 0.75 then roughly same direction ?

		btScalar factor = body->m_propagationParameter.m_directional_factor;
		const btVector3& dir = fbody0->getWorldTransform().getOrigin() - fbody1->getWorldTransform().getOrigin();
		if (dir.dot(direction) > factor)
		{
			//printf("FACTOR: %f\n", dir.dot(direction));

			connection.m_strength -= *impulse;
			//printf("strengthp=%f %f\n",connection->m_strength, *impulse);

			if (connection.m_strength<0)
			{
				//printf("brokenp=%d %d\n",connection->m_childIndex0, connection->m_childIndex1);
				//remove or set to zero
				//printf("Breaking: %d %d\n",connection.m_childIndex0, connection.m_childIndex1);
				connection.m_strength=0.f;
				*needsBreakingCheck = true;
				body->m_connections.remove(connection);
			}
		}

		//impulse dampening, todo expose
		*impulse *= (1.0f - body->m_propagationParameter.m_impulse_dampening);

		btAlignedObjectArray<int> *adjacents = body->m_connection_map->find(connection_index);
		if (adjacents && (*impulse > body->m_propagationParameter.m_minimum_impulse))
		{
			int i, size = adjacents->size();
			//clamp size... else too much recursion going on, leading to crashes
			if (size > 2)
			{
				for (i=0;i<size;i++)
				{
					if(body->m_connections[adjacents->at(i)].m_strength > 0.f)
						propagateDamage(body, impulse, adjacents->at(i), needsBreakingCheck, direction, depth);
				}
			}
		}
	}
}

void btFractureDynamicsWorld::breakNeighborhood(btFractureBody *body, int connection_index)
{
	if (body->m_connections.size() > connection_index)
	{
		btAlignedObjectArray<int> *adjacents = body->m_connection_map->find(connection_index);
		if (adjacents)
		{
			int i, size = adjacents->size();
			float thresh = (1.0f - body->m_propagationParameter.m_stability_factor);
			//clamp size... else too much recursion going on, leading to crashes
			//if (size > 2)
			{
				for (i=0;i<size;i++)
				{
					float rnd = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
					if (rnd < thresh)
					{
						//printf("Breaking neighbor %d\n", i);
						btConnection& con = body->m_connections[adjacents->at(i)];
						con.m_strength = 0;
						//*needsBreakingCheck = true;
						body->m_connections.remove(con);
						breakNeighborhood(body, adjacents->at(i));
					}
				}
			}
		}
	}
}
#endif

void btFractureDynamicsWorld::fractureCallback( )
{

	btAlignedObjectArray<btFracturePair> sFracturePairs;

	if (!m_fracturingMode)
	{
		glueCallback();
		m_fracturingMode = true;
		//return;
	}

	int numManifolds = getDispatcher()->getNumManifolds();

	sFracturePairs.clear();

	// seed for random from time
	srand (static_cast <unsigned> (time(0)));


	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* manifold = getDispatcher()->getManifoldByIndexInternal(i);
		if (!manifold->getNumContacts())
			continue;

		btScalar totalImpact = 0.f;
		for (int p=0;p<manifold->getNumContacts();p++)
		{
			totalImpact += manifold->getContactPoint(p).m_appliedImpulse;
		}

		
		//printf("totalImpact=%f\n",totalImpact);

		static float maxImpact = 0;
		if (totalImpact>maxImpact)
			maxImpact = totalImpact;

		//some threshold otherwise resting contact would break objects after a while
		//if (totalImpact < 10.0f) //40.f
		//	continue;

		//		printf("strong impact\n");


		//@todo: add better logic to decide what parts to fracture
		//For example use the idea from the SIGGRAPH talk about the fracture in the movie 2012:
		//
		//Breaking thresholds can be stored as connectivity information between child shapes in the fracture object
		//
		//You can calculate some "impact value" by simulating all the individual child shapes 
		//as rigid bodies, without constraints, running it in a separate simulation world 
		//(or by running the constraint solver without actually modifying the dynamics world)
		//Then measure some "impact value" using the offset and applied impulse for each child shape
		//weaken the connections based on this "impact value" and only break 
		//if this impact value exceeds the breaking threshold.
		//you can propagate the weakening and breaking of connections using the connectivity information

		int f0 = m_fractureBodies.findLinearSearch((btFractureBody*)manifold->getBody0());
		int f1 = m_fractureBodies.findLinearSearch((btFractureBody*)manifold->getBody1());

		if (f0 == f1 == m_fractureBodies.size())
			continue;


		if (f0<m_fractureBodies.size())
		{
			int j=f0;

			btCollisionObject* colOb = (btCollisionObject*)manifold->getBody1();
			btRigidBody* otherOb = btRigidBody::upcast(colOb);
				if (!otherOb->getInvMass())
					continue;

			int pi=-1;

			for (int p=0;p<sFracturePairs.size();p++)
			{
				if (sFracturePairs[p].m_fracObj == m_fractureBodies[j])
				{
					pi = p; break;
				}
			}

			if (pi<0)
			{
				btFracturePair p;
				p.m_fracObj = m_fractureBodies[j];
				p.m_contactManifolds.push_back(manifold->m_index1a);
				sFracturePairs.push_back(p);
			} else
			{
				//btAssert(sFracturePairs[pi].m_contactManifolds.findLinearSearch(manifold)==sFracturePairs[pi].m_contactManifolds.size());
				sFracturePairs[pi].m_contactManifolds.push_back(manifold->m_index1a);
			}
		}


		if (f1 < m_fractureBodies.size())
		{
			int j=f1;
			{
				btCollisionObject* colOb = (btCollisionObject*)manifold->getBody0();
				btRigidBody* otherOb = btRigidBody::upcast(colOb);
					if (!otherOb->getInvMass())
						continue;


				int pi=-1;

				for (int p=0;p<sFracturePairs.size();p++)
				{
					if (sFracturePairs[p].m_fracObj == m_fractureBodies[j])
					{
						pi = p; break;
					}
				}
				if (pi<0)
				{
					btFracturePair p;
					p.m_fracObj = m_fractureBodies[j];
					p.m_contactManifolds.push_back( manifold->m_index1a);
					sFracturePairs.push_back(p);
				} else
				{
					//btAssert(sFracturePairs[pi].m_contactManifolds.findLinearSearch(manifold)==sFracturePairs[pi].m_contactManifolds.size());
					sFracturePairs[pi].m_contactManifolds.push_back(manifold->m_index1a);
				}
			}
		}

		//
	}

	//printf("m_fractureBodies size=%d\n",m_fractureBodies.size());
	//printf("sFracturePairs size=%d\n",sFracturePairs.size());
	if (!sFracturePairs.size())
		return;


	{
		//		printf("fracturing\n");

		for (int i=0;i<sFracturePairs.size();i++)
		{
			//check impulse/displacement at impact

			//weaken/break connections (and propagate breaking)

			//compute connectivity of connected child shapes


			if (sFracturePairs[i].m_fracObj->getCollisionShape()->isCompound())
			{
				btTransform tr;
				tr.setIdentity();
				btCompoundShape* oldCompound = (btCompoundShape*)sFracturePairs[i].m_fracObj->getCollisionShape();
				if (oldCompound->getNumChildShapes()>1)
				{
					bool needsBreakingCheck = false;
					btScalar totalObjImpact = 0.0f;

					//weaken/break the connections

					//@todo: propagate along the connection graph

					for (int j=0;j<sFracturePairs[i].m_contactManifolds.size();j++)
					{
						if (getDispatcher()->getNumManifolds() > sFracturePairs[i].m_contactManifolds[j])
						{
							btPersistentManifold* manifold = getDispatcher()->getManifoldByIndexInternal(sFracturePairs[i].m_contactManifolds[j]);
							if (!manifold)
								continue;

							for (int k=0;k<manifold->getNumContacts();k++)
							{
								btManifoldPoint& pt = manifold->getContactPoint(k);
								totalObjImpact += pt.m_appliedImpulse;
							}
						}
					}

					if ((totalObjImpact < sFracturePairs[i].m_fracObj->m_propagationParameter.m_minimum_impulse) ||
						(sFracturePairs[i].m_fracObj->m_propagationParameter.m_minimum_impulse == 0.0f))
						continue;

					for (int j=0;j<sFracturePairs[i].m_contactManifolds.size();j++)
					{
						if (getDispatcher()->getNumManifolds() > sFracturePairs[i].m_contactManifolds[j])
						{
							btPersistentManifold* manifold = getDispatcher()->getManifoldByIndexInternal(sFracturePairs[i].m_contactManifolds[j]);
							if (!manifold)
								continue;

							for (int k=0;k<manifold->getNumContacts();k++)
							{
								btManifoldPoint& pt = manifold->getContactPoint(k);
								btScalar impulse = pt.m_appliedImpulse * (1.0f - sFracturePairs[i].m_fracObj->m_propagationParameter.m_stability_factor);
								if (manifold->getBody0()==sFracturePairs[i].m_fracObj)
								{
									for (int f=0;f<sFracturePairs[i].m_fracObj->m_connections.size();f++)
									{
										//direct damage
										btConnection& connection = sFracturePairs[i].m_fracObj->m_connections[f];
										if ((connection.m_childIndex0 == pt.m_index0) ||
											(connection.m_childIndex1 == pt.m_index0))
										{
											connection.m_strength -= impulse;
											//printf("strength0=%f\n",connection.m_strength);

											if (connection.m_strength<0)
											{
												//remove or set to zero
												connection.m_strength=0.f;
												needsBreakingCheck = true;
												sFracturePairs[i].m_fracObj->m_connections.remove(connection);

											//	breakNeighborhood(sFracturePairs[i].m_fracObj, pt.m_index0);
											}
										}
									}
								} else
								{
									//direct damage
									for (int f=0;f<sFracturePairs[i].m_fracObj->m_connections.size();f++)
									{
										btConnection& connection = sFracturePairs[i].m_fracObj->m_connections[f];
										if ((connection.m_childIndex0 == pt.m_index1) ||
											   (connection.m_childIndex1 == pt.m_index1))
										{
											//printf("strength1=%f\n",connection.m_strength);
											connection.m_strength -= impulse;
											if (connection.m_strength<0)
											{
												//remove or set to zero
												connection.m_strength=0.f;
												needsBreakingCheck = true;
												sFracturePairs[i].m_fracObj->m_connections.remove(connection);

											//	breakNeighborhood(sFracturePairs[i].m_fracObj, pt.m_index1);
											}
										}
									}
								}
							}
						}
					}

					if (needsBreakingCheck)
					{
						breakDisconnectedParts(sFracturePairs[i].m_fracObj);
						//glueCallback();
					}
				}

			}

		}
	}

	sFracturePairs.clear();

}
