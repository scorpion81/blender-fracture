
#include "btCompoundConstraint.h"

btCompoundConstraint::btCompoundConstraint(btRigidBody& rbA, btRigidBody& rbB)
    :btTypedConstraint::btTypedConstraint(COMPOUND_CONSTRAINT_TYPE, rbA, rbB)
{
}

