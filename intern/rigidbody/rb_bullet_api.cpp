/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2013 Blender Foundation
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Joshua Leung, Sergej Reich, Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file rb_bullet_api.cpp
 *  \ingroup RigidBody
 *  \brief Rigid Body API implementation for Bullet
 */

/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
 
/* This file defines the "RigidBody interface" for the 
 * Bullet Physics Engine. This API is designed to be used
 * from C-code in Blender as part of the Rigid Body simulation
 * system.
 *
 * It is based on the Bullet C-API, but is heavily modified to 
 * give access to more data types and to offer a nicer interface.
 *
 * -- Joshua Leung, June 2010
 */

#include <stdio.h>
#include <errno.h>
#include <iomanip>

#include "RBI_api.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"	
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btConvexHullComputer.h"

#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"

#include "BulletDynamics/Dynamics/btFractureBody.h"
#include "BulletDynamics/Dynamics/btFractureDynamicsWorld.h"

#include "../../extern/glew/include/GL/glew.h"

typedef struct rbConstraint
{
	btTypedConstraint *con;
	btTransform pivot;
	char id[64];
} rbConstraint;


struct	ViewportDebugDraw : public btIDebugDraw
{
	ViewportDebugDraw () :
		m_debugMode(0)
	{
	}

	enum DebugDrawModes2 {
		DBG_DrawImpulses = 1 << 15,
	};

	int m_debugMode;

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
	{
		if (m_debugMode >0)
		{
			//draw lines
			glBegin(GL_LINES);
				glColor4f(color[0], color[1], color[2], 1.0f);
				glVertex3fv(from);
				glVertex3fv(to);
			glEnd();
		}
	}

	virtual void	reportErrorWarning(const char* warningString)
	{

	}

	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,float distance,int lifeTime,const btVector3& color)
	{
		drawLine(PointOnB, PointOnB + normalOnB, color);
		drawSphere(PointOnB, 0.1, color);
	}

	virtual void	setDebugMode(int debugMode)
	{
		m_debugMode = debugMode;
	}
	virtual int		getDebugMode() const
	{
		return m_debugMode;
	}
	///todo: find out if Blender can do this
	virtual void	draw3dText(const btVector3& location,const char* textString)
	{

	}

};


struct rbRigidBody {
	btRigidBody *body;
	int col_groups;
	int linear_index;
	void *meshIsland;
	void *blenderOb;
	btVector3 *bbox;
	rbDynamicsWorld *world;
};

typedef void (*IdOutCallback)(void *world, void *meshisland, int *objectId, int *shardId);

static btRigidBody* getBodyFromShape(void *shapePtr)
{
	rbRigidBody *body = (rbRigidBody*)shapePtr;
/*	if (body->body->getInternalType() & CUSTOM_FRACTURE_TYPE)
	{
		btFractureBody* fbody = (btFractureBody*)body->body;
		return fbody;
	}*/
	return body->body;
}

static inline void copy_v3_btvec3(float vec[3], const btVector3 &btvec)
{
	vec[0] = (float)btvec[0];
	vec[1] = (float)btvec[1];
	vec[2] = (float)btvec[2];
}

typedef void (*rbContactCallback)(rbContactPoint * cp, void *bworld);
typedef void (*rbTickCallback)(btScalar timeStep, void *bworld);

class TickDiscreteDynamicsWorld : public btFractureDynamicsWorld
{
	public:
		TickDiscreteDynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache,
		                          btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration,
		                          rbContactCallback cont_callback, void *bworld, void *bScene, IdCallback id_callback, rbTickCallback tick_callback);
		rbContactPoint* make_contact_point(btManifoldPoint& point, const btCollisionObject *body0, const btCollisionObject *body1);
		rbContactCallback m_contactCallback;
		rbTickCallback m_tickCallback;
		void* m_bworld;
		void* m_bscene;
		virtual void debugDrawConstraints(rbConstraint *con, draw_string str_callback, float loc[]);
		virtual void debugDrawWorld(draw_string str_callback);
};

btScalar connection_dist(btFractureBody *fbody, int index, btVector3 impact)
{
	btConnection& con = fbody->m_connections[index];
	btVector3 con_posA = con.m_parent->getWorldTransform().inverse() * con.m_obA->getWorldTransform().getOrigin();
	btVector3 con_posB = con.m_parent->getWorldTransform().inverse() * con.m_obB->getWorldTransform().getOrigin();
	btVector3 con_pos = (con_posA + con_posB) * 0.5f;

	return impact.distance(con_pos);
}

//KDTree needed here, we need a range search of which points are closer than distance x to the impact point
int connection_binary_search(btFractureBody *fbody, btVector3 impact, btScalar range)
{
	int mid, low = 0, high = fbody->m_connections.size();

	while (low <= high) {
		mid = (low + high)/2;

		if (mid == high-1)
			return mid;

		btScalar dist = connection_dist(fbody, mid, impact);
		btScalar dist1 = connection_dist(fbody, mid+1, impact);

		if (dist < range && range <= dist1)
		{
			return mid;
		}

		if (dist >= range)
			high= mid - 1;
		else if (dist < range)
			low= mid + 1;
		else
			return mid;
	}

	return low;
}

bool weakenCompound(const btCollisionObject *body, btScalar force, btVector3 impact, btFractureDynamicsWorld *world)
{
	//just weaken strengths of this obA and obB according to force !
	if (body->getInternalType() & CUSTOM_FRACTURE_TYPE && force > 0.0f)
	{
		btFractureBody *fbody = (btFractureBody*)body;
		int size = fbody->m_connections.size();
		bool needs_break = false;

		if (size == 0)
			return false;

		btVector3 *bbox = ((rbRigidBody*)fbody->getUserPointer())->bbox;
		btScalar dim = (*bbox)[bbox->maxAxis()] * 2;

		//stop at this limit, distances are sorted... so no closer object will come later
		//int mid = connection_binary_search(fbody, impact, dim * (1.0f - fbody->m_propagationParameter.m_stability_factor));
		for (int i = 0; i < size; i++)
		{
			btVector3 imp = fbody->getWorldTransform().inverse() * impact;
			btScalar dist = connection_dist(fbody, i, imp);
			btConnection& con = fbody->m_connections[i];
			btScalar lforce = force;
			if (dist > 0.0f)
			{
				lforce = lforce / dist; // the closer, the higher the force
				//printf("lforce %f\n", lforce);
				lforce *= (1.0f - fbody->m_propagationParameter.m_stability_factor);
				//printf("lforce2 %f\n", lforce);
			}

			if (lforce > fbody->m_propagationParameter.m_minimum_impulse)
			{
				if (con.m_strength > 0.0f)
				{
					con.m_strength -= lforce;
					//printf("Damaged connection %d %d with %f\n", fbody->m_connections[i].m_childIndex0,
					//   fbody->m_connections[i].m_childIndex1, force);

					if (con.m_strength <= 0)
					{
						con.m_strength = 0;
						needs_break = true;
					}
				}
			}

			btScalar pdist = connection_dist(fbody, i, con.m_parent->getWorldTransform().getOrigin());
			if (pdist > (dim * (1.0f - fbody->m_propagationParameter.m_stability_factor)))
			{
				break;
			}
		}

		if (needs_break)
			world->breakDisconnectedParts(fbody);

		return needs_break;
	}

	return false;
}

void tickCallback(btDynamicsWorld *world, btScalar timeStep)
{
	btFractureDynamicsWorld *fworld = (btFractureDynamicsWorld*)world;
	fworld->updateBodies();

	TickDiscreteDynamicsWorld* tworld = (TickDiscreteDynamicsWorld*)world;
	bool broken = false;

	int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		int numContacts = contactManifold->getNumContacts();
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance()<0.f)
			{
				/*const btVector3& ptA = pt.getPositionWorldOnA();
				const btVector3& ptB = pt.getPositionWorldOnB();
				const btVector3& normalOnB = pt.m_normalWorldOnB;*/

				//TickDiscreteDynamicsWorld* tworld = (TickDiscreteDynamicsWorld*)world;
				if (tworld->m_contactCallback)
				{

					rbContactPoint* cp = tworld->make_contact_point(pt, obA, obB);
					broken = weakenCompound(obA, cp->contact_force, pt.getPositionWorldOnA(), fworld);
					broken = broken || weakenCompound(obB, cp->contact_force, pt.getPositionWorldOnB(), fworld);
					tworld->m_contactCallback(cp, tworld->m_bworld);
					delete cp;
				}

				//if (broken)
				//	break;
			}
		}

		if (broken)
			break;
	}

	if (tworld->m_tickCallback)
	{
		tworld->m_tickCallback(timeStep, tworld->m_bscene);
	}
}

TickDiscreteDynamicsWorld::TickDiscreteDynamicsWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache,
                                                     btConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration,
                                                     rbContactCallback cont_callback, void *bworld, void *bScene, IdCallback id_callback,
                                                     rbTickCallback tick_callback) :

                                                     btFractureDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration,
                                                                             id_callback, getBodyFromShape)
{
	m_internalTickCallback = tickCallback;
	m_contactCallback = cont_callback;
	m_bworld = bworld;
	m_bscene = bScene;
	m_tickCallback = tick_callback;
}

rbContactPoint* TickDiscreteDynamicsWorld::make_contact_point(btManifoldPoint& point, const btCollisionObject* body0, const btCollisionObject* body1)
{
	rbContactPoint *cp = new rbContactPoint;
	btFractureBody* bodyA = (btFractureBody*)(body0);
	btFractureBody* bodyB = (btFractureBody*)(body1);
	rbRigidBody* rbA = (rbRigidBody*)(bodyA->getUserPointer());
	rbRigidBody* rbB = (rbRigidBody*)(bodyB->getUserPointer());
	if (rbA)
		cp->contact_body_indexA = rbA->linear_index;

	if (rbB)
		cp->contact_body_indexB = rbB->linear_index;

	cp->contact_force = point.getAppliedImpulse();
	copy_v3_btvec3(cp->contact_pos_world_onA, point.getPositionWorldOnA());
	copy_v3_btvec3(cp->contact_pos_world_onB, point.getPositionWorldOnB());

	return cp;
}

void TickDiscreteDynamicsWorld::debugDrawWorld(draw_string str_callback)
{
	BT_PROFILE("debugDrawWorld");

	btCollisionWorld::debugDrawWorld();

	bool drawConstraints = false;
	if (getDebugDrawer())
	{
		int mode = getDebugDrawer()->getDebugMode();
		if(mode  & (btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits))
		{
			drawConstraints = true;
		}
	}
	if(drawConstraints)
	{
		btVector3 lastVec(0,0,0);
		btVector3 drawVec(0,0,0);
		btVector3 offset(0, 0, 0.1);
		for(int i = getNumConstraints()-1; i>=0 ;i--)
		{
			float loc[3];
			btTypedConstraint* constraint = getConstraint(i);
			rbConstraint *con = (rbConstraint*)constraint->getUserConstraintPtr();

			if (lastVec == con->pivot.getOrigin())
			{
				drawVec += offset;
			}
			else
			{
				drawVec = con->pivot.getOrigin();
			}
			copy_v3_btvec3(loc, drawVec);
			debugDrawConstraints(con, str_callback, loc);
			lastVec = con->pivot.getOrigin();

		}
	}



    if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb | btIDebugDraw::DBG_DrawNormals)))
	{
		int i;

		if (getDebugDrawer() && getDebugDrawer()->getDebugMode())
		{
			for (i=0;i<m_actions.size();i++)
			{
				m_actions[i]->debugDraw(m_debugDrawer);
			}
		}
	}
}

const char* val_to_str(rbConstraint* con, int precision, int *length)
{
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(precision) << con->id << ":" << con->con->getAppliedImpulse() << ":" << con->con->getBreakingImpulseThreshold();
	*length = oss.str().length();
	return oss.str().c_str();
}

void TickDiscreteDynamicsWorld::debugDrawConstraints(rbConstraint* con , draw_string str_callback, float loc[3])
{
	btTypedConstraint *constraint = con->con;
	bool drawFrames = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & btIDebugDraw::DBG_DrawConstraintLimits) != 0;
	bool drawImpulses = str_callback != NULL; // && (getDebugDrawer()->getDebugMode() & ViewportDebugDraw::DBG_DrawImpulses) != 0;
	btScalar dbgDrawSize = constraint->getDbgDrawSize();
	int p = 0;

	//color code the load on the constraint
	float color[3] = {1.0f, 1.0f, 1.0f};
	btScalar imp = constraint->getAppliedImpulse();
	btScalar thr = constraint->getBreakingImpulseThreshold();
	float ratio = fabs(imp) / thr;
	int len = 0; // strlen(con->id);
	const char *str = val_to_str(con, p, &len);
	//const char *str = con->id;
	//float loc[3];
	//copy_v3_btvec3(loc, con->pivot.getOrigin());

	if (ratio <= 0.5f)
	{
		//green -> yellow
		color[0] = 2 * ratio;
		color[1] = 1.0f;
		color[2] = 0.0f;
	}
	else if (ratio > 0.5f && ratio <= 1.0f)
	{
		//yellow -> red
		color[0] = 1.0f;
		color[1] = 1.0f - 2 * (1.0f - ratio);
		color[2] = 0.0f;
	}

	if (!constraint->isEnabled())
	{
		color[0] = 0.0f;
		color[1] = 0.0f;
		color[2] = 0.0f;
	}


	if(dbgDrawSize <= btScalar(0.f))
	{
		return;
	}

	if(drawImpulses) str_callback(loc, str, len, color);

	switch(constraint->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			{
				btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)constraint;
				btTransform tr;
				tr.setIdentity();
				btVector3 pivot = p2pC->getPivotInA();
				pivot = p2pC->getRigidBodyA().getCenterOfMassTransform() * pivot;
				tr.setOrigin(pivot);
				getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				// that ideally should draw the same frame
				pivot = p2pC->getPivotInB();
				pivot = p2pC->getRigidBodyB().getCenterOfMassTransform() * pivot;
				tr.setOrigin(pivot);
			}
			break;
		case HINGE_CONSTRAINT_TYPE:
			{
				btHingeConstraint* pHinge = (btHingeConstraint*)constraint;
				btTransform tr = pHinge->getRigidBodyA().getCenterOfMassTransform() * pHinge->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pHinge->getRigidBodyB().getCenterOfMassTransform() * pHinge->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				btScalar minAng = pHinge->getLowerLimit();
				btScalar maxAng = pHinge->getUpperLimit();
				if(minAng == maxAng)
				{
					break;
				}
				bool drawSect = true;
				if(minAng > maxAng)
				{
					minAng = btScalar(0.f);
					maxAng = SIMD_2_PI;
					drawSect = false;
				}
				if(drawLimits)
				{
					btVector3& center = tr.getOrigin();
					btVector3 normal = tr.getBasis().getColumn(2);
					btVector3 axis = tr.getBasis().getColumn(0);
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, btVector3(0,0,0), drawSect);
				}
			}
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			{
				btConeTwistConstraint* pCT = (btConeTwistConstraint*)constraint;
				btTransform tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					//const btScalar length = btScalar(5);
					const btScalar length = dbgDrawSize;
					static int nSegments = 8*4;
					btScalar fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)(nSegments-1)/btScalar(nSegments);
					btVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
					pPrev = tr * pPrev;
					for (int i=0; i<nSegments; i++)
					{
						fAngleInRadians = btScalar(2.*3.1415926) * (btScalar)i/btScalar(nSegments);
						btVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
						pCur = tr * pCur;
						getDebugDrawer()->drawLine(pPrev, pCur, btVector3(0,0,0));

						if (i%(nSegments/8) == 0)
							getDebugDrawer()->drawLine(tr.getOrigin(), pCur, btVector3(0,0,0));

						pPrev = pCur;
					}
					btScalar tws = pCT->getTwistSpan();
					btScalar twa = pCT->getTwistAngle();
					bool useFrameB = (pCT->getRigidBodyB().getInvMass() > btScalar(0.f));
					if(useFrameB)
					{
						tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
					}
					else
					{
						tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
					}
					btVector3 pivot = tr.getOrigin();
					btVector3 normal = tr.getBasis().getColumn(0);
					btVector3 axis1 = tr.getBasis().getColumn(1);
					getDebugDrawer()->drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, btVector3(0,0,0), true);

				}
			}
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		case D6_CONSTRAINT_TYPE:
			{
				btGeneric6DofConstraint* p6DOF = (btGeneric6DofConstraint*)constraint;
				btTransform tr = p6DOF->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					tr = p6DOF->getCalculatedTransformA();
					const btVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					btVector3 up = tr.getBasis().getColumn(2);
					btVector3 axis = tr.getBasis().getColumn(0);
					btScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					btScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					btScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
					btScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
					getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btScalar(.9f), minTh, maxTh, minPs, maxPs, btVector3(0,0,0));
					axis = tr.getBasis().getColumn(1);
					btScalar ay = p6DOF->getAngle(1);
					btScalar az = p6DOF->getAngle(2);
					btScalar cy = btCos(ay);
					btScalar sy = btSin(ay);
					btScalar cz = btCos(az);
					btScalar sz = btSin(az);
					btVector3 ref;
					ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
					ref[1] = -sz*axis[0] + cz*axis[1];
					ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
					tr = p6DOF->getCalculatedTransformB();
					btVector3 normal = -tr.getBasis().getColumn(0);
					btScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					btScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if(minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btVector3(0,0,0), false);
					}
					else if(minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btVector3(0,0,0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					btVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					btVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, btVector3(0,0,0));
				}
			}
			break;
		case SLIDER_CONSTRAINT_TYPE:
			{
				btSliderConstraint* pSlider = (btSliderConstraint*)constraint;
				btTransform tr = pSlider->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pSlider->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					btTransform tr = pSlider->getUseLinearReferenceFrameA() ? pSlider->getCalculatedTransformA() : pSlider->getCalculatedTransformB();
					btVector3 li_min = tr * btVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
					btVector3 li_max = tr * btVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
					getDebugDrawer()->drawLine(li_min, li_max, btVector3(0, 0, 0));
					btVector3 normal = tr.getBasis().getColumn(0);
					btVector3 axis = tr.getBasis().getColumn(1);
					btScalar a_min = pSlider->getLowerAngLimit();
					btScalar a_max = pSlider->getUpperAngLimit();
					const btVector3& center = pSlider->getCalculatedTransformB().getOrigin();
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btVector3(0,0,0), true);
				}

			}
			break;
		default :
			break;
	}
	return;
}

struct rbDynamicsWorld {
	TickDiscreteDynamicsWorld *dynamicsWorld;
	btDefaultCollisionConfiguration *collisionConfiguration;
	btDispatcher *dispatcher;
	btBroadphaseInterface *pairCache;
	btConstraintSolver *constraintSolver;
	btOverlapFilterCallback *filterCallback;
	void *blenderWorld;
	//struct rbContactCallback *contactCallback;
	IdOutCallback idOutCallback;
	void *blenderScene; // ouch, very very clumsy approach, this is just a borrowed pointer
};

struct rbVert {
	float x, y, z;
};
struct rbTri {
	int v0, v1, v2;
};

struct rbMeshData {
	btTriangleIndexVertexArray *index_array;
	rbVert *vertices;
	rbTri *triangles;
	int num_vertices;
	int num_triangles;
};

struct rbCollisionShape {
	btCollisionShape *cshape;
	rbMeshData *mesh;
	rbRigidBody *body;
};

struct myResultCallback : public btCollisionWorld::ClosestRayResultCallback
{
	public:

	bool needsCollision(btBroadphaseProxy *proxy0) const
	{
		return true;
	}

	myResultCallback(const btVector3 &v1, const btVector3 &v2)
	    : btCollisionWorld::ClosestRayResultCallback(v1, v2)
	{
	}
};

struct rbFilterCallback : public btOverlapFilterCallback
{
	int (*callback)(void* world, void* island1, void* island2, void* blenderOb1, void* blenderOb2);

	rbFilterCallback(int (*callback)(void* world, void* island1, void* island2, void* blenderOb1, void* blenderOb2)) {
		this->callback = callback;
	}

	virtual bool needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const
	{
		rbRigidBody *rb0 = (rbRigidBody *)((btFractureBody *)proxy0->m_clientObject)->getUserPointer();
		rbRigidBody *rb1 = (rbRigidBody *)((btFractureBody *)proxy1->m_clientObject)->getUserPointer();
		
		bool collides;
		collides = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
		collides = collides && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);
		if (!rb0 || !rb1)
			return collides;

		collides = collides && (rb0->col_groups & rb1->col_groups);
		if (this->callback != NULL) {
			int result = 0;
			//cast ray from centroid of 1 rigidbody to another, do this only for mesh shapes (all other can use standard bbox)
			int stype0 = rb0->body->getCollisionShape()->getShapeType();
			int stype1 = rb1->body->getCollisionShape()->getShapeType();
			bool nonMeshShape0 = (stype0 != GIMPACT_SHAPE_PROXYTYPE) && (stype0 != TRIANGLE_MESH_SHAPE_PROXYTYPE);
			bool nonMeshShape1 = (stype1 != GIMPACT_SHAPE_PROXYTYPE) && (stype1 != TRIANGLE_MESH_SHAPE_PROXYTYPE);

			if ((rb0->meshIsland != NULL) ^ (rb1->meshIsland != NULL))
			{
				btVector3 v0, v1;
				rbRigidBody* rb2 = NULL;
				bool valid = false;

				if (rb0->meshIsland != NULL)
				{
					v0 = rb0->body->getWorldTransform().getOrigin();
					v1 = rb1->body->getWorldTransform().getOrigin();
				}
				else if (rb1->meshIsland != NULL)
				{
					v0 = rb1->body->getWorldTransform().getOrigin();
					v1 = rb0->body->getWorldTransform().getOrigin();
				}

				myResultCallback cb(v0, v1);
				rb0->world->dynamicsWorld->rayTest(v0, v1, cb);
				if (cb.m_collisionObject != NULL)
				{
					rb2 = (rbRigidBody*)cb.m_collisionObject->getUserPointer();
				}
				else
				{
					valid = false;
				}

				if (rb0->meshIsland != NULL && rb2 != NULL)
				{
					valid = rb2->blenderOb != rb0->blenderOb;
				}
				else if (rb1->meshIsland != NULL && rb2 != NULL)
				{
					valid = rb2->blenderOb != rb1->blenderOb;
				}

				if (rb0->meshIsland != NULL)
				{
					valid = valid || nonMeshShape1;
				}
				else if (rb1->meshIsland != NULL)
				{
					valid = valid || nonMeshShape0;
				}


				if (valid)
				{
					result = this->callback(rb0->world->blenderWorld, rb0->meshIsland, rb1->meshIsland, rb0->blenderOb, rb1->blenderOb);
				}
				else
				{
					//just check for ghost flags and collision groups there
					result = this->callback(NULL, NULL, NULL, rb0->blenderOb, rb1->blenderOb);
				}
			}
			else
			{
				result = this->callback(rb0->world->blenderWorld, rb0->meshIsland, rb1->meshIsland, rb0->blenderOb, rb1->blenderOb);
			}

			collides = collides && (bool)result;
		}
		
		return collides;
	}
};

static inline void copy_quat_btquat(float quat[4], const btQuaternion &btquat)
{
	quat[0] = btquat.getW();
	quat[1] = btquat.getX();
	quat[2] = btquat.getY();
	quat[3] = btquat.getZ();
}

#if 0
/*Contact Handling*/
typedef void (*cont_callback)(rbContactPoint *cp, void* bworld);

struct rbContactCallback
{
	static cont_callback callback;
	static void* bworld;
	rbContactCallback(cont_callback cp, void* bworld);
	static bool handle_contacts(btManifoldPoint& point, btCollisionObject* body0, btCollisionObject* body1);
};

/*rbContactCallback::rbContactCallback(cont_callback callback, void *bworld){
	rbContactCallback::callback = callback;
	rbContactCallback::bworld = bworld;
	gContactProcessedCallback = (ContactProcessedCallback)&rbContactCallback::handle_contacts;
}*/

cont_callback rbContactCallback::callback = 0;
void* rbContactCallback::bworld = NULL;

bool rbContactCallback::handle_contacts(btManifoldPoint& point, btCollisionObject* body0, btCollisionObject* body1)
{
	bool ret = false;
	if (rbContactCallback::callback)
	{
		rbContactPoint *cp = new rbContactPoint;
		btFractureBody* bodyA = (btFractureBody*)(body0);
		btFractureBody* bodyB = (btFractureBody*)(body1);
		rbRigidBody* rbA = (rbRigidBody*)(bodyA->getUserPointer());
		rbRigidBody* rbB = (rbRigidBody*)(bodyB->getUserPointer());
		if (rbA)
			cp->contact_body_indexA = rbA->linear_index;

		if (rbB)
			cp->contact_body_indexB = rbB->linear_index;

		cp->contact_force = point.getAppliedImpulse();
		copy_v3_btvec3(cp->contact_pos_world_onA, point.getPositionWorldOnA());
		copy_v3_btvec3(cp->contact_pos_world_onB, point.getPositionWorldOnB());

		rbContactCallback::callback(cp, rbContactCallback::bworld);

		delete cp;
	}
	return ret;
}
#endif

/* ********************************** */
/* Dynamics World Methods */

/* Setup ---------------------------- */

void RB_dworld_init_compounds(rbDynamicsWorld *world)
{
	//world->dynamicsWorld->glueCallback();
	world->dynamicsWorld->m_fracturingMode = false;
	//trigger the glueing only at beginning ?!
}

static void idCallback(void *userPtr, int* objectId, int* shardId)
{
	rbRigidBody *body = (rbRigidBody*)userPtr;
	if (body)
	{
		rbDynamicsWorld *world = body->world;
		if (world->idOutCallback)
			world->idOutCallback(world->blenderWorld, body->meshIsland, objectId, shardId);
	}
}

//yuck, but need a handle for the world somewhere for collision callback...
rbDynamicsWorld *RB_dworld_new(const float gravity[3], void* blenderWorld, void* blenderScene, int (*callback)(void *, void *, void *, void *, void *),
							   void (*contactCallback)(rbContactPoint* cp, void *bworld), void (*idCallbackOut)(void*, void*, int*, int*),
							   void (*tickCallback)(float timestep, void *bworld))
{
	rbDynamicsWorld *world = new rbDynamicsWorld;
	
	/* collision detection/handling */
	world->collisionConfiguration = new btDefaultCollisionConfiguration();
	
	world->dispatcher = new btCollisionDispatcher(world->collisionConfiguration);
	btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher *)world->dispatcher);
	
	world->pairCache = new btDbvtBroadphase();
	
	world->filterCallback = new rbFilterCallback(callback);
	world->pairCache->getOverlappingPairCache()->setOverlapFilterCallback(world->filterCallback);

	/* constraint solving */
	world->constraintSolver = new btSequentialImpulseConstraintSolver();

	TickDiscreteDynamicsWorld *tworld = new TickDiscreteDynamicsWorld(world->dispatcher,
	                                                                  world->pairCache,
	                                                      world->constraintSolver,
	                                                      world->collisionConfiguration,
	                                                      contactCallback, blenderWorld, blenderScene, idCallback, tickCallback);

	/* world */
	world->dynamicsWorld = tworld;
	world->blenderWorld = blenderWorld;
	world->idOutCallback = idCallbackOut;

	RB_dworld_set_gravity(world, gravity);


	/*debug drawer*/
	world->dynamicsWorld->setDebugDrawer(new ViewportDebugDraw());
	world->dynamicsWorld->getDebugDrawer()->setDebugMode(
	            btIDebugDraw::DBG_DrawWireframe|btIDebugDraw::DBG_DrawAabb|
	            btIDebugDraw::DBG_DrawContactPoints|btIDebugDraw::DBG_DrawText|
	            btIDebugDraw::DBG_DrawConstraints| /*btIDebugDraw::DBG_DrawConstraintLimits*/
	            ViewportDebugDraw::DBG_DrawImpulses);

	/*contact callback */
/*
	if (contactCallback)
	{
		world->contactCallback = new rbContactCallback(contactCallback, world->blenderWorld);
	} */
	
	return world;
}

void RB_dworld_delete(rbDynamicsWorld *world)
{
	/* bullet doesn't like if we free these in a different order */
	delete world->dynamicsWorld;
	delete world->constraintSolver;
	delete world->pairCache;
	delete world->dispatcher;
	delete world->collisionConfiguration;
	delete world->filterCallback;
	delete world;
}

/* Settings ------------------------- */

/* Gravity */
void RB_dworld_get_gravity(rbDynamicsWorld *world, float g_out[3])
{
	copy_v3_btvec3(g_out, world->dynamicsWorld->getGravity());
}

void RB_dworld_set_gravity(rbDynamicsWorld *world, const float g_in[3])
{
	world->dynamicsWorld->setGravity(btVector3(g_in[0], g_in[1], g_in[2]));
}

/* Constraint Solver */
void RB_dworld_set_solver_iterations(rbDynamicsWorld *world, int num_solver_iterations)
{
	btContactSolverInfo& info = world->dynamicsWorld->getSolverInfo();
	
	info.m_numIterations = num_solver_iterations;
}

/* Split Impulse */
void RB_dworld_set_split_impulse(rbDynamicsWorld *world, int split_impulse)
{
	btContactSolverInfo& info = world->dynamicsWorld->getSolverInfo();
	
	info.m_splitImpulse = split_impulse;
}

/* Simulation ----------------------- */

void RB_dworld_step_simulation(rbDynamicsWorld *world, float timeStep, int maxSubSteps, float timeSubStep)
{
	world->dynamicsWorld->stepSimulation(timeStep, maxSubSteps, timeSubStep);
}

void RB_dworld_debug_draw(rbDynamicsWorld *world, draw_string str_callback)
{
	world->dynamicsWorld->debugDrawWorld(str_callback);
}

/* Export -------------------------- */

/**
 * Exports entire dynamics world to Bullet's "*.bullet" binary format
 * which is similar to Blender's SDNA system.
 *
 * \param world Dynamics world to write to file
 * \param filename Assumed to be a valid filename, with .bullet extension
 */
void RB_dworld_export(rbDynamicsWorld *world, const char *filename)
{
	//create a large enough buffer. There is no method to pre-calculate the buffer size yet.
	int maxSerializeBufferSize = 1024 * 1024 * 5;
	
	btDefaultSerializer *serializer = new btDefaultSerializer(maxSerializeBufferSize);
	world->dynamicsWorld->serialize(serializer);
	
	FILE *file = fopen(filename, "wb");
	if (file) {
		fwrite(serializer->getBufferPointer(), serializer->getCurrentBufferSize(), 1, file);
		fclose(file);
	}
	else {
		 fprintf(stderr, "RB_dworld_export: %s\n", strerror(errno));
	}
}

/* ********************************** */
/* Rigid Body Methods */

/* Setup ---------------------------- */

void RB_dworld_add_body(rbDynamicsWorld *world, rbRigidBody *object, int col_groups, void* meshIsland, void* blenderOb, int linear_index)
{
	btRigidBody *body = object->body;

	if (body->getInternalType() == CUSTOM_FRACTURE_TYPE)
	{
		btFractureBody* fbody = (btFractureBody*)body;
		fbody->setWorld(world->dynamicsWorld);
	}

	object->col_groups = col_groups;
	object->meshIsland = meshIsland;
	object->world = world;
	object->blenderOb = blenderOb;
	object->linear_index = linear_index;
	world->dynamicsWorld->addRigidBody(body);
}

void RB_dworld_remove_body(rbDynamicsWorld *world, rbRigidBody *object)
{
	btRigidBody *body = object->body;
	
	world->dynamicsWorld->removeRigidBody(body);
}

/* Collision detection */

void RB_world_convex_sweep_test(
        rbDynamicsWorld *world, rbRigidBody *object,
        const float loc_start[3], const float loc_end[3],
        float v_location[3],  float v_hitpoint[3],  float v_normal[3], int *r_hit)
{
	btRigidBody *body = object->body;
	btCollisionShape *collisionShape = body->getCollisionShape();
	/* only convex shapes are supported, but user can specify a non convex shape */
	if (collisionShape->isConvex()) {
		btCollisionWorld::ClosestConvexResultCallback result(btVector3(loc_start[0], loc_start[1], loc_start[2]), btVector3(loc_end[0], loc_end[1], loc_end[2]));

		btQuaternion obRot = body->getWorldTransform().getRotation();
		
		btTransform rayFromTrans;
		rayFromTrans.setIdentity();
		rayFromTrans.setRotation(obRot);
		rayFromTrans.setOrigin(btVector3(loc_start[0], loc_start[1], loc_start[2]));

		btTransform rayToTrans;
		rayToTrans.setIdentity();
		rayToTrans.setRotation(obRot);
		rayToTrans.setOrigin(btVector3(loc_end[0], loc_end[1], loc_end[2]));
		
		world->dynamicsWorld->convexSweepTest((btConvexShape *)collisionShape, rayFromTrans, rayToTrans, result, 0);
		
		if (result.hasHit()) {
			*r_hit = 1;
			
			v_location[0] = result.m_convexFromWorld[0] + (result.m_convexToWorld[0] - result.m_convexFromWorld[0]) * result.m_closestHitFraction;
			v_location[1] = result.m_convexFromWorld[1] + (result.m_convexToWorld[1] - result.m_convexFromWorld[1]) * result.m_closestHitFraction;
			v_location[2] = result.m_convexFromWorld[2] + (result.m_convexToWorld[2] - result.m_convexFromWorld[2]) * result.m_closestHitFraction;
			
			v_hitpoint[0] = result.m_hitPointWorld[0];
			v_hitpoint[1] = result.m_hitPointWorld[1];
			v_hitpoint[2] = result.m_hitPointWorld[2];
			
			v_normal[0] = result.m_hitNormalWorld[0];
			v_normal[1] = result.m_hitNormalWorld[1];
			v_normal[2] = result.m_hitNormalWorld[2];
			
		}
		else {
			*r_hit = 0;
		}
	}
	else {
		/* we need to return a value if user passes non convex body, to report */
		*r_hit = -2;
	}
}

/* ............ */

rbRigidBody *RB_body_new(rbCollisionShape *shape, const float loc[3], const float rot[4], bool use_compounds, float dampening, float factor,
                         float min_impulse, float stability_factor, const float bbox[3])
{
	rbRigidBody *object = new rbRigidBody;
	/* current transform */
	btTransform trans;
	trans.setOrigin(btVector3(loc[0], loc[1], loc[2]));
	trans.setRotation(btQuaternion(rot[1], rot[2], rot[3], rot[0]));
	
	/* create motionstate, which is necessary for interpolation (includes reverse playback) */
	btDefaultMotionState *motionState = new btDefaultMotionState(trans);
	
	/* make rigidbody */
	btRigidBody::btRigidBodyConstructionInfo rbInfo(1.0f, motionState, shape->cshape);
	
	if (use_compounds)
	{
		btPropagationParameter param;
		param.m_impulse_dampening = dampening;
		param.m_directional_factor = factor;
		param.m_minimum_impulse = min_impulse;
		param.m_stability_factor = stability_factor;
		object->body = new btFractureBody(rbInfo, param);
	}
	else
	{
		object->body = new btRigidBody(rbInfo);
	}
	
	object->bbox = new btVector3(bbox[0], bbox[1], bbox[2]);

	/* user pointers */
	object->body->setUserPointer(object);
	shape->cshape->setUserPointer(object);

	return object;
}

void RB_body_delete(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	
	/* motion state */
	btMotionState *ms = body->getMotionState();
	if (ms)
		delete ms;
	
	/* collision shape is done elsewhere... */
	
	/* body itself */
	
	/* manually remove constraint refs of the rigid body, normally this happens when removing constraints from the world
	 * but since we delete everything when the world is rebult, we need to do it manually here */
	for (int i = body->getNumConstraintRefs() - 1; i >= 0; i--) {
		btTypedConstraint *con = body->getConstraintRef(i);
		body->removeConstraintRef(con);
	}
	
	delete body;

	delete object->bbox;
	delete object;
}

/* Settings ------------------------- */

void RB_body_set_collision_shape(rbRigidBody *object, rbCollisionShape *shape)
{
	btRigidBody *body = object->body;

	/* user pointer */
	shape->cshape->setUserPointer(object);
	
	/* set new collision shape */
	body->setCollisionShape(shape->cshape);
	
	/* recalculate inertia, since that depends on the collision shape... */
	RB_body_set_mass(object, RB_body_get_mass(object));
}

/* ............ */

float RB_body_get_mass(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	
	/* there isn't really a mass setting, but rather 'inverse mass'  
	 * which we convert back to mass by taking the reciprocal again 
	 */
	float value = (float)body->getInvMass();
	
	if (value)
		value = 1.0f / value;
		
	return value;
}

void RB_body_set_mass(rbRigidBody *object, float value)
{
	btRigidBody *body = object->body;
	btVector3 localInertia(0, 0, 0);
	
	/* calculate new inertia if non-zero mass */
	if (value) {
		btCollisionShape *shape = body->getCollisionShape();
		shape->calculateLocalInertia(value, localInertia);
	}
	
	body->setMassProps(value, localInertia);
	body->updateInertiaTensor();
}


float RB_body_get_friction(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	return body->getFriction();
}

void RB_body_set_friction(rbRigidBody *object, float value)
{
	btRigidBody *body = object->body;
	body->setFriction(value);
}


float RB_body_get_restitution(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	return body->getRestitution();
}

void RB_body_set_restitution(rbRigidBody *object, float value)
{
	btRigidBody *body = object->body;
	body->setRestitution(value);
}


float RB_body_get_linear_damping(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	return body->getLinearDamping();
}

void RB_body_set_linear_damping(rbRigidBody *object, float value)
{
	RB_body_set_damping(object, value, RB_body_get_linear_damping(object));
}

float RB_body_get_angular_damping(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	return body->getAngularDamping();
}

void RB_body_set_angular_damping(rbRigidBody *object, float value)
{
	RB_body_set_damping(object, RB_body_get_linear_damping(object), value);
}

void RB_body_set_damping(rbRigidBody *object, float linear, float angular)
{
	btRigidBody *body = object->body;
	body->setDamping(linear, angular);
}


float RB_body_get_linear_sleep_thresh(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	return body->getLinearSleepingThreshold();
}

void RB_body_set_linear_sleep_thresh(rbRigidBody *object, float value)
{
	RB_body_set_sleep_thresh(object, value, RB_body_get_angular_sleep_thresh(object));
}

float RB_body_get_angular_sleep_thresh(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	return body->getAngularSleepingThreshold();
}

void RB_body_set_angular_sleep_thresh(rbRigidBody *object, float value)
{
	RB_body_set_sleep_thresh(object, RB_body_get_linear_sleep_thresh(object), value);
}

void RB_body_set_sleep_thresh(rbRigidBody *object, float linear, float angular)
{
	btRigidBody *body = object->body;
	body->setSleepingThresholds(linear, angular);
}

/* ............ */

void RB_body_get_linear_velocity(rbRigidBody *object, float v_out[3])
{
	btRigidBody *body = object->body;
	
	copy_v3_btvec3(v_out, body->getLinearVelocity());
}

void RB_body_set_linear_velocity(rbRigidBody *object, const float v_in[3])
{
	btRigidBody *body = object->body;
	
	body->setLinearVelocity(btVector3(v_in[0], v_in[1], v_in[2]));
}


void RB_body_get_angular_velocity(rbRigidBody *object, float v_out[3])
{
	btRigidBody *body = object->body;
	
	copy_v3_btvec3(v_out, body->getAngularVelocity());
}

void RB_body_set_angular_velocity(rbRigidBody *object, const float v_in[3])
{
	btRigidBody *body = object->body;
	
	body->setAngularVelocity(btVector3(v_in[0], v_in[1], v_in[2]));
}

void RB_body_set_linear_factor(rbRigidBody *object, float x, float y, float z)
{
	btRigidBody *body = object->body;
	body->setLinearFactor(btVector3(x, y, z));
}

void RB_body_set_angular_factor(rbRigidBody *object, float x, float y, float z)
{
	btRigidBody *body = object->body;
	body->setAngularFactor(btVector3(x, y, z));
}

/* ............ */

void RB_body_set_kinematic_state(rbRigidBody *object, int kinematic)
{
	btRigidBody *body = object->body;
	if (kinematic)
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	else
		body->setCollisionFlags(body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
}

/* ............ */

void RB_body_set_activation_state(rbRigidBody *object, int use_deactivation)
{
	btRigidBody *body = object->body;
	if (use_deactivation)
		body->forceActivationState(ACTIVE_TAG);
	else
		body->setActivationState(DISABLE_DEACTIVATION);
}
void RB_body_activate(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	body->setActivationState(ACTIVE_TAG);
}
void RB_body_deactivate(rbRigidBody *object)
{
	btRigidBody *body = object->body;
	body->setActivationState(ISLAND_SLEEPING);
}

int RB_body_get_activation_state(rbRigidBody* object)
{
	btRigidBody* body = object->body;
	return body->getActivationState();
}

/* ............ */



/* Simulation ----------------------- */

/* The transform matrices Blender uses are OpenGL-style matrices, 
 * while Bullet uses the Right-Handed coordinate system style instead.
 */

void RB_body_get_transform_matrix(rbRigidBody *object, float m_out[4][4])
{
	btRigidBody *body = object->body;
	btMotionState *ms = body->getMotionState();
	
	btTransform trans;
	ms->getWorldTransform(trans);
	
	trans.getOpenGLMatrix((btScalar *)m_out);
}

void RB_body_set_loc_rot(rbRigidBody *object, const float loc[3], const float rot[4])
{
	btRigidBody *body = object->body;
	btMotionState *ms = body->getMotionState();
	
	/* set transform matrix */
	btTransform trans;
	trans.setOrigin(btVector3(loc[0], loc[1], loc[2]));
	trans.setRotation(btQuaternion(rot[1], rot[2], rot[3], rot[0]));
	
	ms->setWorldTransform(trans);
}

void RB_body_set_scale(rbRigidBody *object, const float scale[3])
{
	btRigidBody *body = object->body;
	
	/* apply scaling factor from matrix above to the collision shape */
	btCollisionShape *cshape = body->getCollisionShape();
	if (cshape) {
		cshape->setLocalScaling(btVector3(scale[0], scale[1], scale[2]));
		
		/* GIimpact shapes have to be updated to take scaling into account */
		if (cshape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
			((btGImpactMeshShape *)cshape)->updateBound();
	}
}

/* ............ */
/* Read-only state info about status of simulation */

void RB_body_get_position(rbRigidBody *object, float v_out[3])
{
	btRigidBody *body = object->body;
	
	copy_v3_btvec3(v_out, body->getWorldTransform().getOrigin());
}

void RB_body_get_orientation(rbRigidBody *object, float v_out[4])
{
	btRigidBody *body = object->body;
	
	copy_quat_btquat(v_out, body->getWorldTransform().getRotation());
}

/* ............ */
/* Overrides for simulation */

void RB_body_apply_central_force(rbRigidBody *object, const float v_in[3])
{
	btRigidBody *body = object->body;
	
	body->applyCentralForce(btVector3(v_in[0], v_in[1], v_in[2]));
}

void RB_body_apply_impulse(rbRigidBody* object, const float impulse[3], const float pos[3])
{
	btRigidBody *body = object->body;

	body->applyImpulse(btVector3(impulse[0], impulse[1], impulse[2]), btVector3(pos[0], pos[1], pos[2]));
}

void RB_body_apply_force(rbRigidBody* object, const float force[3], const float pos[3])
{
	btRigidBody *body = object->body;

	body->applyForce(btVector3(force[0], force[1], force[2]), btVector3(pos[0], pos[1], pos[2]));
}

/* ********************************** */
/* Collision Shape Methods */

/* Setup (Standard Shapes) ----------- */

rbCollisionShape *RB_shape_new_box(float x, float y, float z)
{
	rbCollisionShape *shape = new rbCollisionShape;
	shape->cshape = new btBoxShape(btVector3(x, y, z));
	shape->mesh = NULL;
	return shape;
}

rbCollisionShape *RB_shape_new_sphere(float radius)
{
	rbCollisionShape *shape = new rbCollisionShape;
	shape->cshape = new btSphereShape(radius);
	shape->mesh = NULL;
	return shape;
}

rbCollisionShape *RB_shape_new_capsule(float radius, float height)
{
	rbCollisionShape *shape = new rbCollisionShape;
	shape->cshape = new btCapsuleShapeZ(radius, height);
	shape->mesh = NULL;
	return shape;
}

rbCollisionShape *RB_shape_new_cone(float radius, float height)
{
	rbCollisionShape *shape = new rbCollisionShape;
	shape->cshape = new btConeShapeZ(radius, height);
	shape->mesh = NULL;
	return shape;
}

rbCollisionShape *RB_shape_new_cylinder(float radius, float height)
{
	rbCollisionShape *shape = new rbCollisionShape;
	shape->cshape = new btCylinderShapeZ(btVector3(radius, radius, height));
	shape->mesh = NULL;
	return shape;
}

/* Setup (Convex Hull) ------------ */

rbCollisionShape *RB_shape_new_convex_hull(float *verts, int stride, int count, float margin, bool *can_embed)
{
	btConvexHullComputer hull_computer = btConvexHullComputer();
	
	// try to embed the margin, if that fails don't shrink the hull
	if (hull_computer.compute(verts, stride, count, margin, 0.0f) < 0.0f) {
		hull_computer.compute(verts, stride, count, 0.0f, 0.0f);
		*can_embed = false;
	}
	
	rbCollisionShape *shape = new rbCollisionShape;
	btConvexHullShape *hull_shape = new btConvexHullShape(&(hull_computer.vertices[0].getX()), hull_computer.vertices.size());
	
	shape->cshape = hull_shape;
	shape->mesh = NULL;
	return shape;
}

/* Setup (Triangle Mesh) ---------- */

/* Need to call RB_trimesh_finish() after creating triangle mesh and adding vertices and triangles */

rbMeshData *RB_trimesh_data_new(int num_tris, int num_verts)
{
	rbMeshData *mesh = new rbMeshData;
	mesh->vertices = new rbVert[num_verts];
	mesh->triangles = new rbTri[num_tris];
	mesh->num_vertices = num_verts;
	mesh->num_triangles = num_tris;
	
	return mesh;
}

static void RB_trimesh_data_delete(rbMeshData *mesh)
{
	delete mesh->index_array;
	delete[] mesh->vertices;
	delete[] mesh->triangles;
	delete mesh;
}
 
void RB_trimesh_add_vertices(rbMeshData *mesh, float *vertices, int num_verts, int vert_stride)
{
	for (int i = 0; i < num_verts; i++) {
		float *vert = (float*)(((char*)vertices + i * vert_stride));
		mesh->vertices[i].x = vert[0];
		mesh->vertices[i].y = vert[1];
		mesh->vertices[i].z = vert[2];
	}
}
void RB_trimesh_add_triangle_indices(rbMeshData *mesh, int num, int index0, int index1, int index2)
{
	mesh->triangles[num].v0 = index0;
	mesh->triangles[num].v1 = index1;
	mesh->triangles[num].v2 = index2;
}

void RB_trimesh_finish(rbMeshData *mesh)
{
	mesh->index_array = new btTriangleIndexVertexArray(mesh->num_triangles, (int*)mesh->triangles, sizeof(rbTri),
	                                                   mesh->num_vertices, (float*)mesh->vertices, sizeof(rbVert));
}
 
rbCollisionShape *RB_shape_new_trimesh(rbMeshData *mesh)
{
	rbCollisionShape *shape = new rbCollisionShape;
	
	/* triangle-mesh we create is a BVH wrapper for triangle mesh data (for faster lookups) */
	// RB_TODO perhaps we need to allow saving out this for performance when rebuilding?
	btBvhTriangleMeshShape *unscaledShape = new btBvhTriangleMeshShape(mesh->index_array, true, true);
	
	shape->cshape = new btScaledBvhTriangleMeshShape(unscaledShape, btVector3(1.0f, 1.0f, 1.0f));
	shape->mesh = mesh;
	return shape;
}

void RB_shape_trimesh_update(rbCollisionShape *shape, float *vertices, int num_verts, int vert_stride, float min[3], float max[3])
{
	if (shape->mesh == NULL || num_verts != shape->mesh->num_vertices)
		return;
	
	for (int i = 0; i < num_verts; i++) {
		float *vert = (float*)(((char*)vertices + i * vert_stride));
		shape->mesh->vertices[i].x = vert[0];
		shape->mesh->vertices[i].y = vert[1];
		shape->mesh->vertices[i].z = vert[2];
	}
	
	if (shape->cshape->getShapeType() == SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE) {
		btScaledBvhTriangleMeshShape *scaled_shape = (btScaledBvhTriangleMeshShape *)shape->cshape;
		btBvhTriangleMeshShape *mesh_shape = scaled_shape->getChildShape();
		mesh_shape->refitTree(btVector3(min[0], min[1], min[2]), btVector3(max[0], max[1], max[2]));
	}
	else if (shape->cshape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE) {
		btGImpactMeshShape *mesh_shape = (btGImpactMeshShape*)shape->cshape;
		mesh_shape->updateBound();
	}
}

rbCollisionShape *RB_shape_new_gimpact_mesh(rbMeshData *mesh)
{
	rbCollisionShape *shape = new rbCollisionShape;
	
	btGImpactMeshShape *gimpactShape = new btGImpactMeshShape(mesh->index_array);
	gimpactShape->updateBound(); // TODO: add this to the update collision margin call?
	
	shape->cshape = gimpactShape;
	shape->mesh = mesh;
	return shape;
}

/* needed to rebuild mesh on demand */
int RB_shape_get_num_verts(rbCollisionShape *shape)
{
	if (shape->mesh)
	{
		return shape->mesh->num_vertices;
	}

	return 0;
}

//compound shapes
rbCollisionShape *RB_shape_new_compound()
{
	rbCollisionShape *shape = new rbCollisionShape;
	shape->cshape = new btCompoundShape();
	shape->mesh = NULL;
	return shape;
}

void RB_shape_add_compound_child(rbCollisionShape** compound, rbCollisionShape* child, float loc[3], float rot[4])
{
	btCompoundShape *comp = reinterpret_cast<btCompoundShape*>((*compound)->cshape);
	btCollisionShape* ch = child->cshape;
	btTransform trans;
	trans.setOrigin(btVector3(loc[0], loc[1], loc[2]));
	trans.setRotation(btQuaternion(rot[1], rot[2], rot[3], rot[0]));

	comp->addChildShape(trans, ch);
}

/* Cleanup --------------------------- */

void RB_shape_delete(rbCollisionShape *shape)
{
	if (shape->cshape->getShapeType() == SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE) {
		btBvhTriangleMeshShape *child_shape = ((btScaledBvhTriangleMeshShape *)shape->cshape)->getChildShape();
		if (child_shape)
			delete child_shape;
	}
	if (shape->mesh)
		RB_trimesh_data_delete(shape->mesh);
	delete shape->cshape;
	delete shape;
}

/* Settings --------------------------- */

float RB_shape_get_margin(rbCollisionShape *shape)
{
	return shape->cshape->getMargin();
}

void RB_shape_set_margin(rbCollisionShape *shape, float value)
{
	shape->cshape->setMargin(value);

	/* GIimpact shapes have to be updated to take new margin into account */
	if (shape->cshape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
		((btGImpactMeshShape *)(shape->cshape))->updateBound();
}

/* ********************************** */
/* Constraints */

/* Setup ----------------------------- */

void RB_dworld_add_constraint(rbDynamicsWorld *world, rbConstraint *con, int disable_collisions)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);
	
	world->dynamicsWorld->addConstraint(constraint, disable_collisions);
}

void RB_dworld_remove_constraint(rbDynamicsWorld *world, rbConstraint *con)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);
	
	world->dynamicsWorld->removeConstraint(constraint);
}

/* ............ */
static void make_constraint_transforms(btTransform &transform1, btTransform &transform2, btRigidBody *body1, btRigidBody *body2, float pivot[3], float orn[4])
{
	btTransform pivot_transform = btTransform();
	pivot_transform.setOrigin(btVector3(pivot[0], pivot[1], pivot[2]));
	pivot_transform.setRotation(btQuaternion(orn[1], orn[2], orn[3], orn[0]));
	
	transform1 = body1->getWorldTransform().inverse() * pivot_transform;
	transform2 = body2->getWorldTransform().inverse() * pivot_transform;
}

rbConstraint *RB_constraint_new_point(float pivot[3], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	
	btVector3 pivot1 = body1->getWorldTransform().inverse() * btVector3(pivot[0], pivot[1], pivot[2]);
	btVector3 pivot2 = body2->getWorldTransform().inverse() * btVector3(pivot[0], pivot[1], pivot[2]);
	
	btTypedConstraint *con = new btPoint2PointConstraint(*body1, *body2, pivot1, pivot2);

	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);
	
	return rbc;
}

rbConstraint *RB_constraint_new_fixed(float pivot[3], float orn[4], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	btTransform transform1;
	btTransform transform2;
	
	make_constraint_transforms(transform1, transform2, body1, body2, pivot, orn);
	
	btFixedConstraint *con = new btFixedConstraint(*body1, *body2, transform1, transform2);
	
	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);

	return rbc;
}

rbConstraint *RB_constraint_new_hinge(float pivot[3], float orn[4], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	btTransform transform1;
	btTransform transform2;
	
	make_constraint_transforms(transform1, transform2, body1, body2, pivot, orn);
	
	btHingeConstraint *con = new btHingeConstraint(*body1, *body2, transform1, transform2);
	
	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);

	return rbc;
}

rbConstraint *RB_constraint_new_slider(float pivot[3], float orn[4], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	btTransform transform1;
	btTransform transform2;
	
	make_constraint_transforms(transform1, transform2, body1, body2, pivot, orn);
	
	btSliderConstraint *con = new btSliderConstraint(*body1, *body2, transform1, transform2, true);

	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);

	return rbc;
}

rbConstraint *RB_constraint_new_piston(float pivot[3], float orn[4], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	btTransform transform1;
	btTransform transform2;
	
	make_constraint_transforms(transform1, transform2, body1, body2, pivot, orn);
	
	btSliderConstraint *con = new btSliderConstraint(*body1, *body2, transform1, transform2, true);
	con->setUpperAngLimit(-1.0f); // unlock rotation axis
	
	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);

	return rbc;
}

rbConstraint *RB_constraint_new_6dof(float pivot[3], float orn[4], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	btTransform transform1;
	btTransform transform2;
	
	make_constraint_transforms(transform1, transform2, body1, body2, pivot, orn);
	
	btTypedConstraint *con = new btGeneric6DofConstraint(*body1, *body2, transform1, transform2, true);
	
	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);

	return rbc;
}

rbConstraint *RB_constraint_new_6dof_spring(float pivot[3], float orn[4], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	btTransform transform1;
	btTransform transform2;
	
	make_constraint_transforms(transform1, transform2, body1, body2, pivot, orn);
	
	btTypedConstraint *con = new btGeneric6DofSpringConstraint(*body1, *body2, transform1, transform2, true);

	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);

	return rbc;
}

rbConstraint *RB_constraint_new_motor(float pivot[3], float orn[4], rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;
	btTransform transform1;
	btTransform transform2;
	
	make_constraint_transforms(transform1, transform2, body1, body2, pivot, orn);
	
	btGeneric6DofConstraint *con = new btGeneric6DofConstraint(*body1, *body2, transform1, transform2, true);
	
	/* unlock constraint axes */
	for (int i = 0; i < 6; i++) {
		con->setLimit(i, 0.0f, -1.0f);
	}
	/* unlock motor axes */
	con->getTranslationalLimitMotor()->m_upperLimit.setValue(-1.0f, -1.0f, -1.0f);
	
	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	con->setUserConstraintPtr(rbc);

	btVector3 vec(pivot[0], pivot[1], pivot[2]);
	rbc->pivot.setOrigin(vec);

	return rbc;
}

rbConstraint *RB_constraint_new_compound(rbRigidBody *rb1, rbRigidBody *rb2)
{
	btRigidBody *body1 = rb1->body;
	btRigidBody *body2 = rb2->body;

	btCompoundConstraint *con = new btCompoundConstraint(*body1, *body2);

	rbConstraint *rbc = new rbConstraint();
	rbc->con = con;
	rbc->pivot.setOrigin(btVector3(0, 0, 0));

	return rbc;
}

/* Cleanup ----------------------------- */

void RB_constraint_delete(rbConstraint *con)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);
	delete constraint;
	delete con;
}

/* Settings ------------------------- */

void RB_constraint_set_enabled(rbConstraint *con, int enabled)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);
	
	constraint->setEnabled(enabled);
	constraint->enableFeedback(enabled);
}

int RB_constraint_is_enabled(rbConstraint *con)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);

	return constraint->isEnabled();
}

float RB_constraint_get_applied_impulse(rbConstraint *con)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);
	return (float)constraint->getAppliedImpulse();
}

void RB_constraint_set_limits_hinge(rbConstraint *con, float lower, float upper)
{
	btHingeConstraint *constraint = reinterpret_cast<btHingeConstraint*>(con->con);
	
	// RB_TODO expose these
	float softness = 0.9f;
	float bias_factor = 0.3f;
	float relaxation_factor = 1.0f;
	
	constraint->setLimit(lower, upper, softness, bias_factor, relaxation_factor);
}

void RB_constraint_set_limits_slider(rbConstraint *con, float lower, float upper)
{
	btSliderConstraint *constraint = reinterpret_cast<btSliderConstraint*>(con->con);
	
	constraint->setLowerLinLimit(lower);
	constraint->setUpperLinLimit(upper);
}

void RB_constraint_set_limits_piston(rbConstraint *con, float lin_lower, float lin_upper, float ang_lower, float ang_upper)
{
	btSliderConstraint *constraint = reinterpret_cast<btSliderConstraint*>(con->con);
	
	constraint->setLowerLinLimit(lin_lower);
	constraint->setUpperLinLimit(lin_upper);
	constraint->setLowerAngLimit(ang_lower);
	constraint->setUpperAngLimit(ang_upper);
}

void RB_constraint_set_limits_6dof(rbConstraint *con, int axis, float lower, float upper)
{
	btGeneric6DofConstraint *constraint = reinterpret_cast<btGeneric6DofConstraint*>(con->con);
	
	constraint->setLimit(axis, lower, upper);
}

void RB_constraint_set_stiffness_6dof_spring(rbConstraint *con, int axis, float stiffness)
{
	btGeneric6DofSpringConstraint *constraint = reinterpret_cast<btGeneric6DofSpringConstraint*>(con->con);
	
	constraint->setStiffness(axis, stiffness);
}

void RB_constraint_set_damping_6dof_spring(rbConstraint *con, int axis, float damping)
{
	btGeneric6DofSpringConstraint *constraint = reinterpret_cast<btGeneric6DofSpringConstraint*>(con->con);
	
	// invert damping range so that 0 = no damping
	constraint->setDamping(axis, 1.0f - damping);
}

void RB_constraint_set_spring_6dof_spring(rbConstraint *con, int axis, int enable)
{
	btGeneric6DofSpringConstraint *constraint = reinterpret_cast<btGeneric6DofSpringConstraint*>(con->con);
	
	constraint->enableSpring(axis, enable);
}

void RB_constraint_set_equilibrium_6dof_spring(rbConstraint *con)
{
	btGeneric6DofSpringConstraint *constraint = reinterpret_cast<btGeneric6DofSpringConstraint*>(con->con);
	
	constraint->setEquilibriumPoint();
}

void RB_constraint_set_solver_iterations(rbConstraint *con, int num_solver_iterations)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);
	
	constraint->setOverrideNumSolverIterations(num_solver_iterations);
}

void RB_constraint_set_breaking_threshold(rbConstraint *con, float threshold)
{
	btTypedConstraint *constraint = reinterpret_cast<btTypedConstraint*>(con->con);
	
	constraint->setBreakingImpulseThreshold(threshold);
}

void RB_constraint_set_enable_motor(rbConstraint *con, int enable_lin, int enable_ang)
{
	btGeneric6DofConstraint *constraint = reinterpret_cast<btGeneric6DofConstraint*>(con->con);
	
	constraint->getTranslationalLimitMotor()->m_enableMotor[0] = enable_lin;
	constraint->getRotationalLimitMotor(0)->m_enableMotor = enable_ang;
}

void RB_constraint_set_max_impulse_motor(rbConstraint *con, float max_impulse_lin, float max_impulse_ang)
{
	btGeneric6DofConstraint *constraint = reinterpret_cast<btGeneric6DofConstraint*>(con->con);
	
	constraint->getTranslationalLimitMotor()->m_maxMotorForce.setX(max_impulse_lin);
	constraint->getRotationalLimitMotor(0)->m_maxMotorForce = max_impulse_ang;
}

void RB_constraint_set_target_velocity_motor(rbConstraint *con, float velocity_lin, float velocity_ang)
{
	btGeneric6DofConstraint *constraint = reinterpret_cast<btGeneric6DofConstraint*>(con->con);
	
	constraint->getTranslationalLimitMotor()->m_targetVelocity.setX(velocity_lin);
	constraint->getRotationalLimitMotor(0)->m_targetVelocity = velocity_ang;
}

void RB_constraint_set_id(rbConstraint *con, char id[64])
{
	strncpy(con->id, id, strlen(id));
}

/* ********************************** */
