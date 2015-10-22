
#include "btCompoundConstraint.h"

btCompoundConstraint::btCompoundConstraint(btRigidBody& rbA, btRigidBody& rbB)
    :btTypedConstraint(static_cast<btTypedConstraintType>(COMPOUND_CONSTRAINT_TYPE), rbA, rbB)
{
}

