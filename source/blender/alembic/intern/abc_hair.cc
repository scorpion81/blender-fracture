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

#include "abc_hair.h"

#include <cstdio>

#include "abc_transform.h"
#include "abc_util.h"

extern "C" {
#include "DNA_modifier_types.h"

#include "BLI_math_geom.h"

#include "BKE_DerivedMesh.h"
#include "BKE_particle.h"
}

using Alembic::AbcGeom::OCurves;
using Alembic::AbcGeom::OCurvesSchema;
using Alembic::AbcGeom::ON3fGeomParam;
using Alembic::AbcGeom::OV2fGeomParam;

static const float nscale = 1.0f / 32767.0f;

AbcHairWriter::AbcHairWriter(Scene *scene,
                             Object *ob,
                             AbcTransformWriter *parent,
                             uint32_t timeSampling,
                             AbcExportOptions &opts,
                             ParticleSystem *psys)
    : AbcShapeWriter(scene, ob, parent, timeSampling, opts)
{
	std::string name = get_object_name(m_object);
	name.append("Hair");
	m_psys = psys;

	m_is_animated = isAnimated();

	OCurves curves(parent->alembicXform(), name, m_time_sampling);
	m_curves_schema = curves.getSchema();
}

bool AbcHairWriter::isAnimated() const
{
	return true;
}

void AbcHairWriter::do_write()
{
	if (!m_psys) {
		return;
	}

	ParticleSystemModifierData *psmd = psys_get_modifier(m_object, m_psys);

	if (!psmd->dm_final) {
		return;
	}

	DerivedMesh *dm = mesh_create_derived_view(m_scene, m_object, CD_MASK_MESH);
	DM_ensure_tessface(dm);
	DM_update_tessface_data(dm);

	std::vector<Imath::V3f> verts;
	std::vector<int32_t> hvertices;
	std::vector<Imath::V2f> uv_values;
	std::vector<Imath::V3f> norm_values;

	ParticleSettings *part = m_psys->part;

	if (part->type == PART_HAIR && m_psys->pathcache) {
		write_hair_sample(dm, part, verts, norm_values, uv_values, hvertices);

		if (m_options.export_child_hairs && m_psys->childcache) {
			write_hair_child_sample(dm, part, verts, norm_values, uv_values, hvertices);
		}
	}

	dm->release(dm);

	Alembic::Abc::P3fArraySample iPos(verts);
	m_curves_schema_sample = OCurvesSchema::Sample(iPos, hvertices);

	if (!uv_values.empty()) {
		OV2fGeomParam::Sample uv_smp;
		uv_smp.setVals(uv_values);
		m_curves_schema_sample.setUVs(uv_smp);
	}

	if (!norm_values.empty()) {
		ON3fGeomParam::Sample norm_smp;
		norm_smp.setVals(norm_values);
		m_curves_schema_sample.setNormals(norm_smp);
	}

	m_curves_schema_sample.setSelfBounds(bounds());
	m_curves_schema.set(m_curves_schema_sample);
}

void AbcHairWriter::write_hair_sample(DerivedMesh *dm,
                                      ParticleSettings *part,
                                      std::vector<Imath::V3f> &verts,
                                      std::vector<Imath::V3f> &norm_values,
                                      std::vector<Imath::V2f> &uv_values,
                                      std::vector<int32_t> &hvertices)
{
	/* Get untransformed vertices, there's a xform under the hair. */
	float inv_mat[4][4];
	invert_m4_m4_safe(inv_mat, m_object->obmat);

	MTFace *mtface = static_cast<MTFace *>(CustomData_get_layer(&dm->faceData, CD_MTFACE));
	MFace *mface = dm->getTessFaceArray(dm);
	MVert *mverts = dm->getVertArray(dm);

	if (!mtface || !mface) {
		std::fprintf(stderr, "Warning, no UV set found for underlying geometry.\n");
	}

	ParticleData * pa = m_psys->particles;
	int k;

	ParticleCacheKey **cache = m_psys->pathcache;
	ParticleCacheKey *path;

	for (int p = 0; p < m_psys->totpart; ++p, ++pa) {
		/* underlying info for faces-only emission */
		path = cache[p];

		if (part->from == PART_FROM_FACE && mtface) {
			float r_uv[2];
			float tmpnor[3], mapfw[4], vec[3];
			int num = pa->num_dmcache >= 0 ? pa->num_dmcache : pa->num;

			if (num < dm->getNumTessFaces(dm)) {
				MFace *face = (MFace*)dm->getTessFaceData(dm, num, CD_MFACE);
				MTFace *tface = mtface + num;

				if (mface) {
					psys_interpolate_uvs(tface, face->v4, pa->fuv, r_uv);
					uv_values.push_back(Imath::V2f(r_uv[0], r_uv[1]));

					psys_interpolate_face(mverts, face, tface, NULL, mapfw, vec, tmpnor, NULL, NULL, NULL, NULL);

					if (m_options.do_convert_axis) {
						mul_m3_v3(m_options.convert_matrix, tmpnor);
					}

					norm_values.push_back(Imath::V3f(tmpnor[0], tmpnor[1], tmpnor[2]));
				}
			}
			else {
				std::fprintf(stderr, "Particle to faces overflow (%d/%d)\n", num, dm->getNumTessFaces(dm));
			}
		}
		else if (part->from == PART_FROM_VERT && mtface) {
			/* vertex id */
			const int num = (pa->num_dmcache >= 0) ? pa->num_dmcache : pa->num;

			/* iterate over all faces to find a corresponding underlying UV */
			for (int n = 0; n < dm->getNumTessFaces(dm); ++n) {
				MFace *face  = (MFace*)dm->getTessFaceData(dm, n, CD_MFACE);
				MTFace *tface = mtface + n;
				unsigned int vtx[4];
				vtx[0] = face->v1;
				vtx[1] = face->v2;
				vtx[2] = face->v3;
				vtx[3] = face->v4;
				bool found = false;

				for (int o = 0; o < 4; ++o) {
					if (o > 2 && vtx[o] == 0) {
						break;
					}

					if (vtx[o] == num) {
						uv_values.push_back(Imath::V2f(tface->uv[o][0], tface->uv[o][1]));
						MVert *mv = mverts + vtx[o];
						norm_values.push_back(Imath::V3f(mv->no[0] * nscale,
						                      mv->no[1] * nscale, mv->no[2] * nscale));
						found = true;
						break;
					}
				}

				if (found) {
					break;
				}
			}
		}

		int steps = path->segments + 1;
		hvertices.push_back(steps);

		for (k = 0; k < steps; ++k) {
			float vert[3];
			copy_v3_v3(vert, path->co);
			mul_m4_v3(inv_mat, vert);

			if (m_options.do_convert_axis) {
				mul_m3_v3(m_options.convert_matrix, vert);
			}

			verts.push_back(Imath::V3f(vert[0], vert[1], vert[2]));

			++path;
		}
	}
}

void AbcHairWriter::write_hair_child_sample(DerivedMesh *dm,
                                            ParticleSettings *part,
                                            std::vector<Imath::V3f> &verts,
                                            std::vector<Imath::V3f> &norm_values,
                                            std::vector<Imath::V2f> &uv_values,
                                            std::vector<int32_t> &hvertices)
{
	/* Get untransformed vertices, there's a xform under the hair. */
	float inv_mat[4][4];
	invert_m4_m4_safe(inv_mat, m_object->obmat);

	MTFace *mtface = static_cast<MTFace *>(CustomData_get_layer(&dm->faceData, CD_MTFACE));
	MFace *mface = dm->getTessFaceArray(dm);
	MVert *mverts = dm->getVertArray(dm);

	if (!mtface || !mface) {
		std::fprintf(stderr, "Warning, no UV set found for underlying geometry.\n");
	}

	ParticleCacheKey **cache = m_psys->childcache;
	ParticleCacheKey *path;

	ChildParticle *pc = m_psys->child;

	for (int p = 0; p < m_psys->totchild; ++p, ++pc) {
		path = cache[p];

		if (part->from == PART_FROM_FACE) {
			float r_uv[2];
			float tmpnor[3], mapfw[4], vec[3];
			int num = pc->num;

			MFace *face = (MFace *)dm->getTessFaceData(dm, num, CD_MFACE);
			MTFace *tface = mtface + num;

			if (mface && mtface) {
				psys_interpolate_uvs(tface, face->v4, pc->fuv, r_uv);
				uv_values.push_back(Imath::V2f(r_uv[0], r_uv[1]));

				psys_interpolate_face(mverts, face, tface, NULL, mapfw, vec, tmpnor, NULL, NULL, NULL, NULL);

				if (m_options.do_convert_axis) {
					mul_m3_v3(m_options.convert_matrix, tmpnor);
				}

				norm_values.push_back(Imath::V3f(tmpnor[0], tmpnor[1], tmpnor[2]));
			}
		}

		int steps = path->segments + 1;
		hvertices.push_back(steps);

		for (int k = 0; k < steps; ++k) {
			float vert[3];
			copy_v3_v3(vert, path->co);
			mul_m4_v3(inv_mat, vert);
			verts.push_back(Imath::V3f(vert[0], vert[1], vert[2]));
			++path;
		}
	}
}
