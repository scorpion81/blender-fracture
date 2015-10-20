#ifndef BTCOMPOUNDCONSTRAINT_H
#define BTCOMPOUNDCONSTRAINT_H

#include "btTypedConstraint.h"

#define COMPOUND_CONSTRAINT_TYPE (MAX_CONSTRAINT_TYPE+1)

ATTRIBUTE_ALIGNED16(class) btCompoundConstraint : public btTypedConstraint
{
	public:
		btCompoundConstraint(btRigidBody& rbA,btRigidBody& rbB);
		virtual void getInfo1(btConstraintInfo1* info) {}
		virtual void getInfo2(btConstraintInfo2* info) {}
		virtual btScalar getParam(int num, int axis) const { return 0;}
		virtual void setParam(int num, btScalar value, int axis) {}

		int m_objectId;
};

#endif // BTCOMPOUNDCONSTRAINT_H
