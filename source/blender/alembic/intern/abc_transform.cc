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
 * Contributor(s): Esteban Tovagliari, Cedric Paille, Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "abc_transform.h"

#include <OpenEXR/ImathBoxAlgo.h>

#include "abc_util.h"

extern "C" {
#include "DNA_object_types.h"

#include "BLI_math.h"
}

using namespace Alembic;

AbcTransformWriter::AbcTransformWriter(Object *obj, Abc::OObject abcParent, AbcTransformWriter *writerParent,
												unsigned int timeSampling, AbcExportOptions &opts) : AbcObjectWriter(obj, opts)
{
	m_is_animated = hasAnimation(m_object);
	m_parent = NULL;
	m_no_parent_invert = false;

	if (!m_is_animated)
		timeSampling = 0;

    m_xform = AbcGeom::OXform(abcParent, getObjectName(m_object), timeSampling);
	m_schema = m_xform.getSchema();

	if (writerParent)
		writerParent->addChild(this);
}

// recompute transform matrix of object in new coordinate system (from Z-Up to Y-Up)
static void createTransformMatrix(float transform_mat[4][4], Object *obj)
{
    float rot_mat[3][3], rot[3][3], scale_mat[4][4], invmat[4][4], mat[4][4];
    float rot_x_mat[3][3], rot_y_mat[3][3], rot_z_mat[3][3];
    float loc[3], scale[3], euler[3];

    zero_v3(loc);
    zero_v3(scale);
    zero_v3(euler);
    unit_m3(rot);
    unit_m3(rot_mat);
    unit_m4(scale_mat);
    unit_m4(transform_mat);
    unit_m4(invmat);
    unit_m4(mat);

    // get local matrix
    if (obj->parent) {
        invert_m4_m4(invmat, obj->parent->obmat);
        mul_m4_m4m4(mat, invmat, obj->obmat);
    }
    else {
        copy_m4_m4(mat, obj->obmat);
    }

    // compute rotation matrix
    switch(obj->rotmode) {
        case ROT_MODE_AXISANGLE:
        {
            // get euler angles from axis angle rotation
            axis_angle_to_eulO(euler, ROT_MODE_XYZ, obj->rotAxis, obj->rotAngle);

            // create X, Y, Z rotation matrices from euler angles
            rotate_m3_zup_yup(rot_x_mat, rot_y_mat, rot_z_mat, euler);

            // concatenate rotation matrices
            mul_m3_m3m3(rot_mat, rot_mat, rot_y_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_z_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_x_mat);

            // extract location and scale from matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            break;
        }

        case ROT_MODE_QUAT:
        {
            float q[4];
            copy_v4_v4(q, obj->quat);

            // swap axis
            q[2] = obj->quat[3];
            q[3] = -obj->quat[2];

            // compute rotation matrix from quaternion
            quat_to_mat3(rot_mat, q);

            // extract location and scale from matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            break;
        }

        case ROT_MODE_XYZ:
        {
            // extract location, rotation, and scale form matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            // get euler angles from rotation matrix
            mat3_to_eulO(euler, ROT_MODE_XYZ, rot);

            // create X, Y, Z rotation matrices from euler angles
            rotate_m3_zup_yup(rot_x_mat, rot_y_mat, rot_z_mat, euler);

            // concatenate rotation matrices
            mul_m3_m3m3(rot_mat, rot_mat, rot_y_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_z_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_x_mat);

            break;
        }

        case ROT_MODE_XZY:
        {
            // extract location, rotation, and scale form matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            // get euler angles from rotation matrix
            mat3_to_eulO(euler, ROT_MODE_XZY, rot);

            // create X, Y, Z rotation matrices from euler angles
            rotate_m3_zup_yup(rot_x_mat, rot_y_mat, rot_z_mat, euler);

            // concatenate rotation matrices
            mul_m3_m3m3(rot_mat, rot_mat, rot_z_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_y_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_x_mat);

            break;
        }

        case ROT_MODE_YXZ:
        {
            // extract location, rotation, and scale form matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            // get euler angles from rotation matrix
            mat3_to_eulO(euler, ROT_MODE_YXZ, rot);

            // create X, Y, Z rotation matrices from euler angles
            rotate_m3_zup_yup(rot_x_mat, rot_y_mat, rot_z_mat, euler);

            // concatenate rotation matrices
            mul_m3_m3m3(rot_mat, rot_mat, rot_y_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_x_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_z_mat);

            break;
        }

        case ROT_MODE_YZX:
        {
            // extract location, rotation, and scale form matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            // get euler angles from rotation matrix
            mat3_to_eulO(euler, ROT_MODE_YZX, rot);

            // create X, Y, Z rotation matrices from euler angles
            rotate_m3_zup_yup(rot_x_mat, rot_y_mat, rot_z_mat, euler);

            // concatenate rotation matrices
            mul_m3_m3m3(rot_mat, rot_mat, rot_x_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_y_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_z_mat);

            break;
        }

        case ROT_MODE_ZXY:
        {
            // extract location, rotation, and scale form matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            // get euler angles from rotation matrix
            mat3_to_eulO(euler, ROT_MODE_ZXY, rot);

            // create X, Y, Z rotation matrices from euler angles
            rotate_m3_zup_yup(rot_x_mat, rot_y_mat, rot_z_mat, euler);

            // concatenate rotation matrices
            mul_m3_m3m3(rot_mat, rot_mat, rot_z_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_x_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_y_mat);

            break;
        }

        case ROT_MODE_ZYX:
        {
            // extract location, rotation, and scale form matrix
            mat4_to_loc_rot_size(loc, rot, scale, mat);

            // get euler angles from rotation matrix
            mat3_to_eulO(euler, ROT_MODE_ZYX, rot);

            // create X, Y, Z rotation matrices from euler angles
            rotate_m3_zup_yup(rot_x_mat, rot_y_mat, rot_z_mat, euler);

            // concatenate rotation matrices
            mul_m3_m3m3(rot_mat, rot_mat, rot_x_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_z_mat);
            mul_m3_m3m3(rot_mat, rot_mat, rot_y_mat);

            break;
        }
    }

    // add rotation matrix to transformation matrix
    copy_m4_m3(transform_mat, rot_mat);

    // add translation to transformation matrix
    transform_mat[3][0] = loc[0];
    transform_mat[3][1] = loc[2];
    transform_mat[3][2] = -loc[1];

    // create scale matrix
    scale_mat[0][0] = scale[0];
    scale_mat[1][1] = scale[2];
    scale_mat[2][2] = scale[1];

    // add scale to transformation matrix
    mul_m4_m4m4(transform_mat, transform_mat, scale_mat);
}

void AbcTransformWriter::do_write()
{
	if (m_first_frame) {
		if (m_world_space) {
			Abc::OCompoundProperty userProps = m_schema.getUserProperties();
			m_rot90xprop = Abc::OBoolProperty(userProps, "bl_was_z_up");
			m_rot90xprop.set(true);
		}

		if (hasProperties(reinterpret_cast<ID *>(m_object))) {
			if (m_options.export_props_as_geo_params)
				writeProperties(reinterpret_cast<ID *>(m_object), m_schema.getArbGeomParams());
			else
				writeProperties(reinterpret_cast<ID *>(m_object), m_schema.getUserProperties());
		}

		m_visibility =  Alembic::AbcGeom::CreateVisibilityProperty(m_xform, m_xform.getSchema().getTimeSampling());
	}

	bool visibility = ! (m_object->restrictflag & OB_RESTRICT_VIEW);
	m_visibility.set(visibility);

	if (!m_first_frame && !m_is_animated)
		return;

    float mat[4][4];

    createTransformMatrix(mat, m_object);

    float rot_mat[4][4];

    if (m_object->type == OB_CAMERA) {
        unit_m4(rot_mat);
        rotate_m4(rot_mat, 'X', -M_PI_2);
        mul_m4_m4m4(mat, mat, rot_mat);
    }

    m_matrix = convertMatrix(mat);

	m_sample.setMatrix(m_matrix);
	m_schema.set(m_sample);
}

Abc::Box3d AbcTransformWriter::bounds() const
{
	AbcGeom::Box3d bounds;

	for (int i = 0; i < m_children.size(); ++i) {
		Abc::Box3d box(m_children[i]->bounds());
		bounds.extendBy(box);
	}

	return Imath::transform(bounds, m_matrix);
}

bool AbcTransformWriter::hasAnimation(Object *obj) const
{
	// TODO: implement this
	return true;
}
