
#include "btFractureBody.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

#include "BulletDynamics/Dynamics/btFractureDynamicsWorld.h"


class btConnectionSortPredicate
{
	public:

		bool operator() ( const btConnection& lhs, const btConnection& rhs ) const
		{
			btVector3 locLhsA = lhs.m_parent->getWorldTransform().inverse() * lhs.m_obA->getWorldTransform().getOrigin();
			btVector3 locLhsB = lhs.m_parent->getWorldTransform().inverse() * lhs.m_obB->getWorldTransform().getOrigin();

			btVector3 locRhsA = rhs.m_parent->getWorldTransform().inverse() * rhs.m_obA->getWorldTransform().getOrigin();
			btVector3 locRhsB = rhs.m_parent->getWorldTransform().inverse() * rhs.m_obB->getWorldTransform().getOrigin();

			btVector3 locLhs = (locLhsA + locLhsB) * 0.5f;
			btVector3 locRhs = (locRhsA + locRhsB) * 0.5f;

			//lhs.parent should match rhs.parent... same object
			btAssert(lhs.m_parent == rhs.m_parent);

			btScalar dLhs = lhs.m_parent->getWorldTransform().getOrigin().distance(locLhs);
			btScalar dRhs = rhs.m_parent->getWorldTransform().getOrigin().distance(locRhs);
			//btTransform id = btTransform::getIdentity();
			//btScalar dLhs = id.getOrigin().distance(locLhs);
			//btScalar dRhs = id.getOrigin().distance(locRhs);

			return dLhs < dRhs;
		}
};


void	btFractureBody::recomputeConnectivity(btCollisionWorld* world)
{
	m_connections.clear();
	//@todo use the AABB tree to avoid N^2 checks

	if (getCollisionShape()->isCompound())
	{
		btCompoundShape* compound = (btCompoundShape*)getCollisionShape();
		for (int i=0;i<compound->getNumChildShapes();i++)
		{
			for (int j=i+1;j<compound->getNumChildShapes();j++)
			{

				struct   MyContactResultCallback : public btCollisionWorld::ContactResultCallback
				{
					bool m_connected;
					btScalar m_margin;
					MyContactResultCallback() :m_connected(false),m_margin(0.05)
					{
					}
					virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
					{
						if (cp.getDistance()<=m_margin)
							m_connected = true;
						return 1.f;
					}
			   };

				MyContactResultCallback result;

				btCollisionObject obA;
				obA.setWorldTransform(compound->getChildTransform(i));
				obA.setCollisionShape(compound->getChildShape(i));
				btCollisionObject obB;
				obB.setWorldTransform(compound->getChildTransform(j));
				obB.setCollisionShape(compound->getChildShape(j));
				world->contactPairTest(&obA,&obB,result);
				if (result.m_connected)
				{
					btConnection tmp;
					tmp.m_childIndex0 = i;
					tmp.m_childIndex1 = j;
					tmp.m_childShape0 = compound->getChildShape(i);
					tmp.m_childShape1 = compound->getChildShape(j);
					tmp.m_strength = 1.f;//??
					m_connections.push_back(tmp);
				}
			}
		}
	}
}

void btFractureBody::recomputeConnectivityByConstraints(btCollisionWorld *world1)
{
	btFractureDynamicsWorld *world = (btFractureDynamicsWorld*)world1;

	if (world->m_idCallback != NULL)
	{
		int i, size = world->m_compoundConstraints.size();
		m_connections.clear();

		for (i=0; i<size;i++)
		{
			btCompoundConstraint *con = world->m_compoundConstraints[i];

			if (con->isEnabled())
			{
				int obIdA, shardIdA, obIdB, shardIdB;
				btFractureBody *obA = (btFractureBody*)&con->getRigidBodyA();
				btFractureBody *obB = (btFractureBody*)&con->getRigidBodyB();

				//if (this == obA || this == obB)
				{
					int *index0 = NULL, *index1 = NULL;
					world->m_idCallback(obA->getUserPointer(), &obIdA, &shardIdA);
					world->m_idCallback(obB->getUserPointer(), &obIdB, &shardIdB);

					index0 = world->m_childIndexHash->find(to_str(obIdA, shardIdA));
					index1 = world->m_childIndexHash->find(to_str(obIdB, shardIdB));

					if ((obIdA == obIdB) && (shardIdA != shardIdB) &&
					    index0 && index1 && *index0 > -1 && *index1 > -1)
					{
						btConnection tmp;
						tmp.m_childIndex0 = *index0;
						tmp.m_childIndex1 = *index1;
						tmp.m_childShape0 = obA->getCollisionShape();
						tmp.m_childShape1 = obB->getCollisionShape();
						tmp.m_strength = con->getBreakingImpulseThreshold();
						tmp.m_obA = obA;
						tmp.m_obB = obB;
						tmp.m_parent = this;
						tmp.m_id = i;
						m_connections.push_back(tmp);
					}

					//break;
				}
			}
		}

		m_connections.quickSort(btConnectionSortPredicate());
		//build a connection map
		m_connection_map->clear();

		size = m_connections.size();
		for (i=0; i < size; i++)
		{
			btConnection& con = m_connections[i];
			btAlignedObjectArray<int> *adjacents = m_connection_map->find(con.m_childIndex0);
			if (!adjacents) {
				btAlignedObjectArray<int> adj;
				adj.push_back(con.m_childIndex1);
				m_connection_map->insert(con.m_childIndex0, adj);
			}
			else
			{
				if (adjacents->size() != adjacents->findLinearSearch(con.m_childIndex1))
					adjacents->push_back(con.m_childIndex1);
			}
		}
	}
}

btCompoundShape* btFractureBody::shiftTransformDistributeMass(btCompoundShape* boxCompound,btScalar mass,btTransform& shift)
{

	btVector3 principalInertia;

	btScalar* masses = new btScalar[boxCompound->getNumChildShapes()];
	for (int j=0;j<boxCompound->getNumChildShapes();j++)
	{
		//evenly distribute mass
		masses[j]=mass/boxCompound->getNumChildShapes();
	}

	return shiftTransform(boxCompound,masses,shift,principalInertia);

}


btCompoundShape* btFractureBody::shiftTransform(btCompoundShape* boxCompound,btScalar* masses,btTransform& shift, btVector3& principalInertia)
{
	btTransform principal;

	boxCompound->calculatePrincipalAxisTransform(masses,principal,principalInertia);


	///create a new compound with world transform/center of mass properly aligned with the principal axis

	///non-recursive compound shapes perform better
	
#ifdef USE_RECURSIVE_COMPOUND

	btCompoundShape* newCompound = new btCompoundShape();
	newCompound->addChildShape(principal.inverse(),boxCompound);
	newBoxCompound = newCompound;
	//m_collisionShapes.push_back(newCompound);

	//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,newCompound,principalInertia);

#else
#ifdef CHANGE_COMPOUND_INPLACE
	newBoxCompound = boxCompound;
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		boxCompound->updateChildTransform(i,newChildTransform);
	}
	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);
	if (isDynamic)
		boxCompound->calculateLocalInertia(mass,localInertia);
	
#else
	///creation is faster using a new compound to store the shifted children
	btCompoundShape* newBoxCompound = new btCompoundShape();
	for (int i=0;i<boxCompound->getNumChildShapes();i++)
	{
		btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
		///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
		newBoxCompound->addChildShape(newChildTransform,boxCompound->getChildShape(i));
	}



#endif

#endif//USE_RECURSIVE_COMPOUND

	shift = principal;
	return newBoxCompound;
}






