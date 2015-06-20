/*
 * Copyright 2013, Blender Foundation.
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
 */

#ifndef PTC_ABC_SCHEMA_H
#define PTC_ABC_SCHEMA_H

#include <Alembic/AbcGeom/SchemaInfoDeclarations.h>
#include <Alembic/AbcGeom/IGeomBase.h>
#include <Alembic/AbcGeom/OGeomBase.h>

namespace PTC {

#if 0
#define PTC_SCHEMA_INFO ALEMBIC_ABCGEOM_DECLARE_SCHEMA_INFO

using namespace Alembic::AbcGeom;
#endif


#if 0
/* XXX We define an extended schema class to implement the wrapper constructor.
 * This was removed in Alembic 1.1 for some reason ...
 */
template <class SCHEMA>
class OSchemaObject : public Abc::OSchemaObject<SCHEMA>
{
public:
	//! The default constructor creates an empty OSchemaObject function set.
	//! ...
	OSchemaObject() : Abc::OSchemaObject<SCHEMA>() {}

	//! The primary constructor creates an OSchemaObject as a child of the
	//! first argument, which is any Abc or AbcCoreAbstract (or other)
	//! object which can be intrusively cast to an ObjectWriterPtr.
	template <class OBJECT_PTR>
	OSchemaObject(OBJECT_PTR iParentObject,
	              const std::string &iName,

	              const Argument &iArg0 = Argument(),
	              const Argument &iArg1 = Argument(),
	              const Argument &iArg2 = Argument())
	    : Abc::OSchemaObject<SCHEMA>(iParentObject, iName, iArg0, iArg1, iArg2)
	{}

	//! Wrap an existing schema object.
	//! ...
	template <class OBJECT_PTR>
	OSchemaObject(OBJECT_PTR iThisObject,
	              WrapExistingFlag iFlag,
	              const Argument &iArg0 = Argument(),
	              const Argument &iArg1 = Argument(),
	              const Argument &iArg2 = Argument() );
};

//-*****************************************************************************		
template<class SCHEMA>
template<class OBJECT_PTR>
inline OSchemaObject<SCHEMA>::OSchemaObject(
        OBJECT_PTR iObject,
        WrapExistingFlag iFlag,
        const Argument &iArg0,
        const Argument &iArg1,
        const Argument &iArg2 )
    : OObject(iObject,
              iFlag,
              GetErrorHandlerPolicy(iObject,
                                    iArg0, iArg1, iArg2))
{
	ALEMBIC_ABC_SAFE_CALL_BEGIN("OSchemaObject::OSchemaObject( wrap )");
	
	const AbcA::ObjectHeader &oheader = this->getHeader();
	
	Abc::OSchemaObject<SCHEMA>::m_schema = SCHEMA(
	               this->getProperties().getProperty(SCHEMA::getDefaultSchemaName()).getPtr()->asCompoundPtr(),
	               iFlag,
	               this->getErrorHandlerPolicy(),
	               GetSchemaInterpMatching(iArg0, iArg1, iArg2));
	
	/* XXX TODO gives compiler error atm */
//	ABCA_ASSERT(matches(oheader, GetSchemaInterpMatching(iArg0, iArg1, iArg2)),
//	             "Incorrect match of schema: "
//	             << oheader.getMetaData().get( "schemaObjTitle" )
//	             << " to expected: "
//	             << getSchemaObjTitle());
	
	ALEMBIC_ABC_SAFE_CALL_END_RESET();
}
#endif

} /* namespace PTC */

#endif  /* PTC_SCHEMA_H */
