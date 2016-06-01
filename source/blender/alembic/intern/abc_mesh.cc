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

#include "abc_mesh.h"

#include <algorithm>

#include "abc_transform.h"
#include "abc_util.h"

extern "C" {
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_fluidsim.h"
#include "DNA_object_types.h"

#include "BLI_math_geom.h"
#include "BLI_string.h"

#include "BKE_DerivedMesh.h"
#include "BKE_depsgraph.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"

#include "ED_object.h"

#include "WM_api.h"
#include "WM_types.h"
}

using Alembic::Abc::FloatArraySample;
using Alembic::Abc::Int32ArraySample;
using Alembic::Abc::Int32ArraySamplePtr;
using Alembic::Abc::P3fArraySamplePtr;
using Alembic::Abc::V2fArraySample;
using Alembic::Abc::V3fArraySample;

using Alembic::AbcGeom::IFaceSet;
using Alembic::AbcGeom::IFaceSetSchema;
using Alembic::AbcGeom::IObject;
using Alembic::AbcGeom::IPolyMesh;
using Alembic::AbcGeom::IPolyMeshSchema;
using Alembic::AbcGeom::ISampleSelector;
using Alembic::AbcGeom::ISubD;
using Alembic::AbcGeom::ISubDSchema;
using Alembic::AbcGeom::IV2fGeomParam;

using Alembic::AbcGeom::OArrayProperty;
using Alembic::AbcGeom::OBoolProperty;
using Alembic::AbcGeom::OC3fArrayProperty;
using Alembic::AbcGeom::OC3fGeomParam;
using Alembic::AbcGeom::OCompoundProperty;
using Alembic::AbcGeom::OFaceSet;
using Alembic::AbcGeom::OFaceSetSchema;
using Alembic::AbcGeom::OFloatGeomParam;
using Alembic::AbcGeom::OInt32GeomParam;
using Alembic::AbcGeom::ON3fArrayProperty;
using Alembic::AbcGeom::ON3fGeomParam;
using Alembic::AbcGeom::OPolyMesh;
using Alembic::AbcGeom::OPolyMeshSchema;
using Alembic::AbcGeom::OSubD;
using Alembic::AbcGeom::OSubDSchema;
using Alembic::AbcGeom::OV2fGeomParam;
using Alembic::AbcGeom::OV3fGeomParam;

using Alembic::AbcGeom::kFacevaryingScope;
using Alembic::AbcGeom::kVaryingScope;
using Alembic::AbcGeom::kVertexScope;
using Alembic::AbcGeom::kWrapExisting;
using Alembic::AbcGeom::UInt32ArraySample;

AbcMeshWriter::AbcMeshWriter(Scene *scene,
                             Object *ob,
                             AbcTransformWriter *parent,
                             uint32_t timeSampling,
                             ExportSettings &settings)
    : AbcShapeWriter(scene, ob, parent, timeSampling, settings)
{
	m_is_animated = isAnimated();
	m_subsurf_mod = NULL;
	m_has_per_face_materials = false;
	m_has_vertex_weights = false;
	bool isSubd = false;

	/* if the object is static, use the default static time sampling */
	if (!m_is_animated) {
		timeSampling = 0;
	}

	if (!m_settings.export_subsurfs_as_meshes) {
		/* check if the mesh is a subsurf, ignoring disabled modifiers and
		 * displace if it's after subsurf. */
		ModifierData *md = static_cast<ModifierData *>(m_object->modifiers.last);

		while (md) {
			if (modifier_isEnabled(m_scene, md, eModifierMode_Render)) {
				if (md->type == eModifierType_Subsurf) {
					SubsurfModifierData *smd = reinterpret_cast<SubsurfModifierData*>(md);

					if (smd->subdivType == ME_CC_SUBSURF) {
						m_subsurf_mod = md;
						isSubd = true;
						break;
					}
				}

				/* mesh is not a subsurf. break */
				if ((md->type != eModifierType_Displace) && (md->type != eModifierType_ParticleSystem)) {
					break;
				}
			}

			md = md->prev;
		}
	}

	m_is_fluid = (getFluidSimModifier() != NULL);

	while (parent->alembicXform().getChildHeader(m_name)) {
		m_name.append("_");
	}

	if (m_settings.use_subdiv_schema && isSubd) {
		OSubD subd(parent->alembicXform(), m_name, m_time_sampling);
		m_subdiv_schema = subd.getSchema();
	}
	else {
		OPolyMesh mesh(parent->alembicXform(), m_name, m_time_sampling);
		m_mesh_schema = mesh.getSchema();

		OCompoundProperty typeContainer = m_mesh_schema.getUserProperties();
		OBoolProperty type(typeContainer, "meshtype");
		m_subd_type = isSubd;
		type.set(isSubd);
	}
}

AbcMeshWriter::~AbcMeshWriter()
{
	if (m_subsurf_mod) {
		m_subsurf_mod->mode &= ~eModifierMode_DisableTemporary;
	}
}

bool AbcMeshWriter::isSubD() const
{
	return m_subd_type;
}

bool AbcMeshWriter::isAnimated() const
{
	/* check if object has shape keys */
	Mesh *me = static_cast<Mesh *>(m_object->data);

	if (me->key) {
		return true;
	}

	/* test modifiers */
	ModifierData *md = static_cast<ModifierData *>(m_object->modifiers.first);

	while (md) {
		if (md->type != eModifierType_Subsurf) {
			return true;
		}

		md = md->next;
	}

	return false;
}

void AbcMeshWriter::do_write()
{
	/* we have already stored a sample for this object. */
	if (!m_first_frame && !m_is_animated)
		return;

	if (m_settings.use_subdiv_schema && m_subdiv_schema.valid()) {
		writeSubD();
	}
	else {
		writeMesh();
	}
}

void AbcMeshWriter::writeMesh()
{
	DerivedMesh *dm = getFinalMesh();

	try {
		std::vector<std::vector<float> > uvs;
		std::vector<float> points, normals, creaseSharpness;
		std::vector<int32_t> facePoints, faceCounts;
		std::vector<int32_t> creaseIndices, creaseLengths;

		std::vector<uint32_t> uvIdx;
		std::vector<Imath::V2f> uvValArray;

		int active_uvlayer = CustomData_get_active_layer(&dm->loopData, CD_MLOOPUV);

		if (active_uvlayer >= 0) {
			getUVs(dm, uvValArray, uvIdx, active_uvlayer);
		}

		getMeshInfo(dm, points, facePoints, faceCounts, uvs, creaseIndices,
		            creaseLengths, creaseSharpness);

		if (m_first_frame) {
			/* create materials' facesets */
			std::map< std::string, std::vector<int32_t>  > geoGroups;
			getGeoGroups(dm, geoGroups);

			for (std::map< std::string, std::vector<int32_t>  >::iterator it = geoGroups.begin(); it != geoGroups.end(); ++it) {
				OFaceSet faceSet;

				faceSet = m_mesh_schema.createFaceSet(it->first);

				OFaceSetSchema::Sample samp;
				samp.setFaces(Int32ArraySample(it->second));
				faceSet.getSchema().set(samp);
			}

			if (hasProperties(reinterpret_cast<ID *>(m_object->data))) {
				if (m_settings.export_props_as_geo_params)
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_mesh_schema.getArbGeomParams(), false);
				else
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_mesh_schema.getUserProperties(), true);
			}
			createArbGeoParams(dm);
		}

		/* Export UVs */
		OV2fGeomParam::Sample uvSamp;
		if (!uvIdx.empty() && !uvValArray.empty()) {
			uvSamp.setScope(kFacevaryingScope);

			uvSamp.setVals(V2fArraySample(
			                   &uvValArray.front(),
			                   uvValArray.size()));

			UInt32ArraySample idxSamp(
			            (const uint32_t *) &uvIdx.front(),
			            uvIdx.size());

			uvSamp.setIndices(idxSamp);
		}

		/* Normals export */
		ON3fGeomParam::Sample normalsSamp;
		if (!normals.empty()) {
			normalsSamp.setScope(kFacevaryingScope);
			normalsSamp.setVals(
			            V3fArraySample(
			                (const Imath::V3f *) &normals.front(),
			                normals.size() / 3));
		}

		m_mesh_sample = OPolyMeshSchema::Sample(
		                    V3fArraySample(
		                        (const Imath::V3f *) &points.front(),
		                        points.size() / 3),
		                    Int32ArraySample(facePoints),
		                    Int32ArraySample(faceCounts), uvSamp,
		                    normalsSamp);

		/* TODO : export all uvmaps */

		m_mesh_sample.setSelfBounds(bounds());
		m_mesh_schema.set(m_mesh_sample);
		writeArbGeoParams(dm);
		freeMesh(dm);
	}
	catch (...) {
		freeMesh(dm);
		throw;
	}
}

void AbcMeshWriter::writeSubD()
{
	DerivedMesh *dm = getFinalMesh();

	try {
		std::vector<float> points, creaseSharpness;
		std::vector<std::vector<float> > uvs;
		std::vector<int32_t> facePoints, faceCounts;
		std::vector<int32_t> creaseIndices, creaseLengths;
		std::vector<uint32_t> uvIdx;
		std::vector<Imath::V2f> uvValArray;

		int active_uvlayer  = CustomData_get_active_layer(&dm->loopData, CD_MLOOPUV);

		if (active_uvlayer >= 0)
			getUVs(dm, uvValArray, uvIdx, active_uvlayer);

		getMeshInfo(dm, points, facePoints, faceCounts, uvs, creaseIndices,
		            creaseLengths, creaseSharpness);

		if (m_first_frame) {
			std::map< std::string, std::vector<int32_t>  > geoGroups;
			getGeoGroups(dm, geoGroups);

			for (std::map< std::string, std::vector<int32_t>  >::iterator it = geoGroups.begin(); it != geoGroups.end(); ++it) {
				OFaceSet faceSet;

				faceSet = m_subdiv_schema.createFaceSet(it->first);

				OFaceSetSchema::Sample samp;
				samp.setFaces(Int32ArraySample(it->second));
				faceSet.getSchema().set(samp);
			}

			if (hasProperties(reinterpret_cast<ID *>(m_object->data))) {
				if (m_settings.export_props_as_geo_params)
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_subdiv_schema.getArbGeomParams(), false);
				else
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_subdiv_schema.getUserProperties(), true);
			}

			createArbGeoParams(dm);
		}

		OV2fGeomParam::Sample uvSamp;

		m_subdiv_sample = OSubDSchema::Sample(
		                      V3fArraySample(
		                          (const Imath::V3f *) &points.front(),
		                          points.size() / 3),
		                      Int32ArraySample(facePoints),
		                      Int32ArraySample(faceCounts));

		if (!uvIdx.empty() && !uvValArray.empty()) {
			uvSamp.setScope(kFacevaryingScope);
			uvSamp.setVals(V2fArraySample(
			                   (const Imath::V2f *) &uvValArray.front(),
			                   uvValArray.size()));
			UInt32ArraySample idxSamp(
			            (const uint32_t *) &uvIdx.front(),
			            uvIdx.size());
			uvSamp.setIndices(idxSamp);
			m_subdiv_sample.setUVs(uvSamp);
		}

		if (!creaseIndices.empty()) {
			m_subdiv_sample.setCreaseIndices(
			            Int32ArraySample(creaseIndices));
			m_subdiv_sample.setCreaseLengths(
			            Int32ArraySample(creaseLengths));
			m_subdiv_sample.setCreaseSharpnesses(
			            FloatArraySample(creaseSharpness));
		}

		m_subdiv_sample.setSelfBounds(bounds());
		m_subdiv_schema.set(m_subdiv_sample);
		writeArbGeoParams(dm);
		freeMesh(dm);
	}
	catch (...) {
		freeMesh(dm);
		throw;
	}
}

void AbcMeshWriter::getMeshInfo(DerivedMesh *dm,
                                std::vector<float> &points,
                                std::vector<int32_t> &facePoints,
                                std::vector<int32_t> &faceCounts,
                                std::vector<std::vector<float> > &uvs,
                                std::vector<int32_t> &creaseIndices,
                                std::vector<int32_t> &creaseLengths,
                                std::vector<float> &creaseSharpness)
{
	getPoints(dm, points);
	getTopology(dm, facePoints, faceCounts);

	const float creaseFactor = 1.0f / 255.0f;

	creaseIndices.clear();
	creaseSharpness.clear();
	creaseLengths.clear();

	MEdge *edge = dm->getEdgeArray(dm);

	for (int i = 0, e = dm->getNumEdges(dm); i < e; ++i) {
		float sharpness = (float) edge[i].crease * creaseFactor;

		if (sharpness != 0.0f) {
			creaseIndices.push_back(edge[i].v1);
			creaseIndices.push_back(edge[i].v2);
			creaseSharpness.push_back(sharpness);
		}
	}

	creaseLengths.resize(creaseSharpness.size(), 2);
}

DerivedMesh *AbcMeshWriter::getFinalMesh()
{
	/* We don't want subdivided mesh data */
	if (m_subsurf_mod) {
		m_subsurf_mod->mode |= eModifierMode_DisableTemporary;
	}

	DerivedMesh *dm = mesh_create_derived_render(m_scene, m_object, CD_MASK_MESH);

	if (m_subsurf_mod) {
		m_subsurf_mod->mode &= ~eModifierMode_DisableTemporary;
	}

	return dm;
}

void AbcMeshWriter::freeMesh(DerivedMesh *dm)
{
	dm->release(dm);
}

void AbcMeshWriter::getPoints(DerivedMesh *dm, std::vector<float> &points)
{
	points.clear();
	points.reserve(dm->getNumVerts(dm) * 3);

	MVert *verts = dm->getVertArray(dm);

	for (int i = 0, e = dm->getNumVerts(dm); i < e; ++i) {
		/* Convert Z-up to Y-up. */
		points.push_back(verts[i].co[0]);
		points.push_back(verts[i].co[2]);
		points.push_back(-verts[i].co[1]);
	}

	calcBounds(points);
}

void AbcMeshWriter::getTopology(DerivedMesh *dm,
                                std::vector<int32_t> &facePoints,
                                std::vector<int32_t> &pointCounts)
{
	facePoints.clear();
	pointCounts.clear();

	int num_poly = dm->getNumPolys(dm);
	MLoop *loop_array = dm->getLoopArray(dm);
	MPoly *polygons = dm->getPolyArray(dm);

	pointCounts.reserve(num_poly);

	for (int i = 0; i < num_poly; ++i) {
		MPoly &current_poly = polygons[i];
		MLoop *loop = loop_array + current_poly.loopstart + current_poly.totloop;
		pointCounts.push_back(current_poly.totloop);

		for (int j = 0; j < current_poly.totloop; j++) {
			loop--;
			facePoints.push_back(loop->v);
		}
	}
}

void AbcMeshWriter::getNormals(DerivedMesh *dm, std::vector<float> &norms)
{
	/* TODO: check if we need to reverse the normals. */

	const float nscale = 1.0f / 32767.0f;
	norms.clear();

	if (m_settings.export_normals) {
		norms.reserve(m_num_face_verts);

		MVert *verts = dm->getVertArray(dm);
		MFace *faces = dm->getTessFaceArray(dm);

		for (int i = 0, e = dm->getNumTessFaces(dm); i < e; ++i) {
			MFace *face = &faces[i];

			if (face->flag & ME_SMOOTH) {
				int index = face->v4;

				if (face->v4) {
					norms.push_back(verts[index].no[0] * nscale);
					norms.push_back(verts[index].no[1] * nscale);
					norms.push_back(verts[index].no[2] * nscale);
				}

				index = face->v3;
				norms.push_back(verts[index].no[0] * nscale);
				norms.push_back(verts[index].no[1] * nscale);
				norms.push_back(verts[index].no[2] * nscale);

				index = face->v2;
				norms.push_back(verts[index].no[0] * nscale);
				norms.push_back(verts[index].no[1] * nscale);
				norms.push_back(verts[index].no[2] * nscale);

				index = face->v1;
				norms.push_back(verts[index].no[0] * nscale);
				norms.push_back(verts[index].no[1] * nscale);
				norms.push_back(verts[index].no[2] * nscale);
			}
			else {
				float no[3];

				if (face->v4) {
					normal_quad_v3(no, verts[face->v1].co, verts[face->v2].co,
					        verts[face->v3].co, verts[face->v4].co);

					norms.push_back(no[0]);
					norms.push_back(no[1]);
					norms.push_back(no[2]);
				}
				else
					normal_tri_v3(no, verts[face->v1].co, verts[face->v2].co,
					        verts[face->v3].co);

				norms.push_back(no[0]);
				norms.push_back(no[1]);
				norms.push_back(no[2]);

				norms.push_back(no[0]);
				norms.push_back(no[1]);
				norms.push_back(no[2]);

				norms.push_back(no[0]);
				norms.push_back(no[1]);
				norms.push_back(no[2]);
			}
		}
	}
}

void AbcMeshWriter::getUVs(DerivedMesh *dm,
                           std::vector<Imath::V2f> &uvs,
                           std::vector<uint32_t> &uvidx, int layer_idx)
{
	uvs.clear();
	uvidx.clear();

	MLoopUV *mloopuv_array = static_cast<MLoopUV *>(CustomData_get_layer_n(&dm->loopData, CD_MLOOPUV, layer_idx));

	if (!(m_settings.export_uvs && mloopuv_array)) {
		return;
	}

	int num_poly = dm->getNumPolys(dm);
	MPoly *polygons = dm->getPolyArray(dm);

	if (!m_settings.pack_uv) {
		int cnt = 0;
		for (int i = 0; i < num_poly; ++i) {
			MPoly &current_poly = polygons[i];
			MLoopUV *loopuvpoly = mloopuv_array + current_poly.loopstart + current_poly.totloop;

			for (int j = 0; j < current_poly.totloop; ++j) {
				loopuvpoly--;
				uvidx.push_back(cnt++);
				Imath::V2f uv(loopuvpoly->uv[0], loopuvpoly->uv[1]);
				uvs.push_back(uv);
			}
		}
	}
	else {
		for (int i = 0; i < num_poly; ++i) {
			MPoly &current_poly = polygons[i];
			MLoopUV *loopuvpoly = mloopuv_array + current_poly.loopstart + current_poly.totloop;

			for (int j = 0; j < current_poly.totloop; ++j) {
				loopuvpoly--;
				Imath::V2f uv(loopuvpoly->uv[0], loopuvpoly->uv[1]);

				std::vector<Imath::V2f>::iterator it = std::find(uvs.begin(), uvs.end(), uv);

				if (it == uvs.end()) {
					uvidx.push_back(uvs.size());
					uvs.push_back(uv);
				}
				else {
					uvidx.push_back(std::distance(uvs.begin(), it));
				}
			}
		}
	}
}

void AbcMeshWriter::getMaterialIndices(DerivedMesh *dm,
                                       std::vector<int32_t> &indices)
{
	indices.clear();
	indices.reserve(dm->getNumTessFaces(dm));

	MFace *faces = dm->getTessFaceArray(dm);

	for (int i = 1, e = dm->getNumTessFaces(dm); i < e; ++i) {
		MFace *face = &faces[i];
		indices.push_back(face->mat_nr);
	}
}

void AbcMeshWriter::createArbGeoParams(DerivedMesh *dm)
{
	if (m_is_fluid) {
		/* TODO: replace this, when velocities are added by default to schemas. */
		OV3fGeomParam param;

		if (m_subdiv_schema.valid()) {
			param = OV3fGeomParam(
			            m_subdiv_schema.getArbGeomParams(), "velocity", false,
			            kVertexScope, 1, m_time_sampling);
		}
		else {
			param = OV3fGeomParam(
			            m_mesh_schema.getArbGeomParams(), "velocity", false,
			            kVertexScope, 1, m_time_sampling);
		}

		m_velocity = param.getValueProperty();

		/* we don't need anything more for fluid meshes */
		return;
	}

	std::string layer_name;

	for (int i = 0; i < dm->vertData.totlayer; ++i) {
		layer_name = dm->vertData.layers[i].name;

		/* skip unnamed layers */
		if (layer_name == "") {
			continue;
		}

		if (m_subdiv_schema.valid())
			createVertexLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		else
			createVertexLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
	}

	for (int i = 0; i < dm->polyData.totlayer; ++i) {
		CustomDataLayer *layer = &dm->polyData.layers[i];
		layer_name = dm->polyData.layers[i].name;

		/* skip unnamed layers */
		if (layer_name == "") {
			continue;
		}

		if (layer->type == CD_MCOL && !m_settings.export_vcols)
			continue;

		if (m_subdiv_schema.valid()) {
			createFaceLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		}
		else {
			createFaceLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
		}
	}
}

void AbcMeshWriter::createVertexLayerParam(DerivedMesh *dm, int index,
                                           const OCompoundProperty &arbGeoParams)
{
	CustomDataLayer *layer = &dm->vertData.layers[index];
	const std::string layer_name = layer->name;

	/* we have already a layer named layerName. skip */
	if (m_layers_written.count(layer_name) != 0) {
		return;
	}

	switch (layer->type) {
		case CD_PROP_FLT:
		{
			OFloatGeomParam param(arbGeoParams, layer_name, false,
			                      kVertexScope, 1, m_time_sampling);
			m_layers_written.insert(layer_name);
			m_vert_layers.push_back(
			            std::pair<int, OArrayProperty>(index,
			                                           param.getValueProperty()));

			break;
		}
		case CD_PROP_INT:
		{
			OInt32GeomParam param(arbGeoParams, layer_name, false,
			                      kVertexScope, 1, m_time_sampling);
			m_layers_written.insert(layer_name);
			m_vert_layers.push_back(
			            std::pair<int, OArrayProperty>(index,
			                                           param.getValueProperty()));

			break;
		}
	}
}

void AbcMeshWriter::createFaceLayerParam(DerivedMesh *dm, int index,
                                         const OCompoundProperty &arbGeoParams)
{
	CustomDataLayer *layer = &dm->polyData.layers[index];
	const std::string layer_name = layer->name;

	/* we have already a layer named layer_name, skip */
	if (m_layers_written.count(layer_name) != 0) {
		return;
	}

	switch (layer->type) {
		case CD_MCOL:
		{
			OC3fGeomParam param(arbGeoParams, layer_name, false,
			                    kFacevaryingScope, 1, m_time_sampling);

			m_layers_written.insert(layer_name);
			m_face_layers.push_back(
			            std::pair<int, OArrayProperty>(index,
			                                           param.getValueProperty()));

			break;
		}
		default:
			break;
	};
}

void AbcMeshWriter::writeArbGeoParams(DerivedMesh *dm)
{
	if (m_is_fluid) {
		std::vector<float> velocities;
		getVelocities(dm, velocities);

		Alembic::AbcCoreAbstract::ArraySample samp(&(velocities.front()),
		                                           m_velocity.getDataType(),
		                                           Alembic::Util::Dimensions(dm->getNumVerts(dm)));
		m_velocity.set(samp);

		/* we have all we need */
		return;
	}

	/* vertex data */
	for (int i = 0; i < m_vert_layers.size(); ++i) {
		if (m_subdiv_schema.valid()) {
			writeVertexLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		}
		else {
			writeVertexLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
		}
	}

	/* face varying data */
	for (int i = 0; i < m_face_layers.size(); ++i) {
		if (m_subdiv_schema.valid()) {
			writeFaceLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		}
		else {
			writeFaceLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
		}
	}

	if (m_first_frame && m_has_per_face_materials) {
		std::vector<int32_t> faceVals;

		if (m_settings.export_face_sets || m_settings.export_mat_indices) {
			getMaterialIndices(dm, faceVals);
		}

		if (m_settings.export_face_sets) {
			OFaceSetSchema::Sample samp;
			samp.setFaces(Int32ArraySample(faceVals));
			m_face_set.getSchema().set(samp);
		}

		if (m_settings.export_mat_indices) {
			Alembic::AbcCoreAbstract::ArraySample samp(&(faceVals.front()),
			                                           m_mat_indices.getDataType(),
			                                           Alembic::Util::Dimensions(dm->getNumTessFaces(dm)));
			m_mat_indices.set(samp);
		}
	}
}

void AbcMeshWriter::writeVertexLayerParam(DerivedMesh *dm, int index,
                                          const OCompoundProperty &/*arbGeoParams*/)
{
	CustomDataLayer *layer = &dm->vertData.layers[m_vert_layers[index].first];
	int totvert = dm->getNumVerts(dm);

	switch (layer->type) {
		case CD_PROP_FLT:
		case CD_PROP_INT:
		{
			Alembic::AbcCoreAbstract::ArraySample samp(layer->data,
			                                           m_vert_layers[index].second.getDataType(),
			                                           Alembic::Util::Dimensions(totvert));

			m_vert_layers[index].second.set(samp);
			break;
		}
	};
}

void AbcMeshWriter::writeFaceLayerParam(DerivedMesh *dm, int index,
                                        const OCompoundProperty &/*arbGeoParams*/)
{
	CustomDataLayer *layer = &dm->polyData.layers[m_face_layers[index].first];
	const int totpolys = dm->getNumPolys(dm);

	std::vector<float> buffer;

	switch (layer->type) {
		case CD_MCOL:
		{
			const float cscale = 1.0f / 255.0f;

			buffer.clear();
			MPoly *polys = dm->getPolyArray(dm);
			MCol *cfaces = static_cast<MCol *>(layer->data);

			for (int i = 0; i < totpolys; ++i) {
				MPoly *p = &polys[i];
				MCol *cface = &cfaces[p->loopstart + p->totloop];

				for (int j = 0; j < p->totloop; ++j) {
					cface--;
					buffer.push_back(cface->b * cscale);
					buffer.push_back(cface->g * cscale);
					buffer.push_back(cface->r * cscale);
				}
			}

			Alembic::AbcCoreAbstract::ArraySample samp(&buffer.front(),
			                                           m_face_layers[index].second.getDataType(),
			                                           Alembic::Util::Dimensions(m_num_face_verts));
			break;
		}
		default:
			break;
	};
}

ModifierData *AbcMeshWriter::getFluidSimModifier()
{
	ModifierData *md = modifiers_findByType(m_object, eModifierType_Fluidsim);

	if (md && (modifier_isEnabled(m_scene, md, eModifierMode_Render))) {
		FluidsimModifierData *fsmd = reinterpret_cast<FluidsimModifierData *>(md);

		if (fsmd->fss && fsmd->fss->type == OB_FLUIDSIM_DOMAIN) {
			return md;
		}
	}

	return NULL;
}

void AbcMeshWriter::getVelocities(DerivedMesh *dm, std::vector<float> &vels)
{
	const int totverts = dm->getNumVerts(dm);

	vels.clear();
	vels.reserve(totverts);

	ModifierData *md = getFluidSimModifier();
	FluidsimModifierData *fmd = reinterpret_cast<FluidsimModifierData *>(md);
	FluidsimSettings *fss = fmd->fss;

	if (fss->meshVelocities) {
		float *meshVels = reinterpret_cast<float *>(fss->meshVelocities);
		float vel[3];

		for (int i = 0; i < totverts; ++i) {
			copy_v3_v3(vel, meshVels);

			/* Convert Z-up to Y-up. */
			vels.push_back(vels[0]);
			vels.push_back(vels[2]);
			vels.push_back(-vels[1]);
			meshVels += 3;
		}
	}
	else {
		for (int i = 0; i < totverts; ++i) {
			vels.push_back(0);
			vels.push_back(0);
			vels.push_back(0);
		}
	}
}

void AbcMeshWriter::getGeoGroups(
        DerivedMesh *dm,
        std::map<std::string, std::vector<int32_t> > &geo_groups)
{
	const int num_poly = dm->getNumPolys(dm);
	MPoly *polygons = dm->getPolyArray(dm);

	for (int i = 0; i < num_poly; ++i) {
		MPoly &current_poly = polygons[i];
		short mnr = current_poly.mat_nr;

		Material *mat = give_current_material(m_object, mnr + 1);

		if (!mat) {
			continue;
		}

		std::string name = get_id_name(&mat->id);

		if (geo_groups.find(name) == geo_groups.end()) {
			std::vector<int32_t> faceArray;
			geo_groups[name] = faceArray;
		}

		geo_groups[name].push_back(i);
	}

	if (geo_groups.size() == 0) {
		Material *mat = give_current_material(m_object, 1);

		std::string name = (mat) ? get_id_name(&mat->id) : "default";

		std::vector<int32_t> faceArray;

		for (int i = 0, e = dm->getNumTessFaces(dm); i < e; ++i) {
			faceArray.push_back(i);
		}

		geo_groups[name] = faceArray;
	}
}

/* ******************************* mesh reader ****************************** */

/* Some helpers for mesh generation */
namespace utils {

static void mesh_add_verts(Mesh *mesh, size_t len)
{
	if (len == 0) {
		return;
	}

	int totvert = mesh->totvert + len;
	CustomData vdata;
	CustomData_copy(&mesh->vdata, &vdata, CD_MASK_MESH, CD_DEFAULT, totvert);
	CustomData_copy_data(&mesh->vdata, &vdata, 0, 0, mesh->totvert);

	if (!CustomData_has_layer(&vdata, CD_MVERT)) {
		CustomData_add_layer(&vdata, CD_MVERT, CD_CALLOC, NULL, totvert);
	}

	CustomData_free(&mesh->vdata, mesh->totvert);
	mesh->vdata = vdata;
	BKE_mesh_update_customdata_pointers(mesh, false);

	/* set final vertex list size */
	mesh->totvert = totvert;
}

static void mesh_add_mloops(Mesh *mesh, size_t len)
{
	if (len == 0) {
		return;
	}

	/* new face count */
	const int totloops = mesh->totloop + len;

	CustomData ldata;
	CustomData_copy(&mesh->ldata, &ldata, CD_MASK_MESH, CD_DEFAULT, totloops);
	CustomData_copy_data(&mesh->ldata, &ldata, 0, 0, mesh->totloop);

	if (!CustomData_has_layer(&ldata, CD_MLOOP)) {
		CustomData_add_layer(&ldata, CD_MLOOP, CD_CALLOC, NULL, totloops);
	}

	if (!CustomData_has_layer(&ldata, CD_MLOOPUV)) {
		CustomData_add_layer(&ldata, CD_MLOOPUV, CD_CALLOC, NULL, totloops);
	}

	CustomData_free(&mesh->ldata, mesh->totloop);
	mesh->ldata = ldata;
	BKE_mesh_update_customdata_pointers(mesh, false);

	mesh->totloop = totloops;
}

static void mesh_add_mpolygons(Mesh *mesh, size_t len)
{
	if (len == 0) {
		return;
	}

	/* new face count */
	const int totpolys = mesh->totpoly + len;

	CustomData pdata;
	CustomData_copy(&mesh->pdata, &pdata, CD_MASK_MESH, CD_DEFAULT, totpolys);
	CustomData_copy_data(&mesh->pdata, &pdata, 0, 0, mesh->totpoly);

	if (!CustomData_has_layer(&pdata, CD_MPOLY)) {
		CustomData_add_layer(&pdata, CD_MPOLY, CD_CALLOC, NULL, totpolys);
	}

	if (!CustomData_has_layer(&pdata, CD_MTEXPOLY)) {
		CustomData_add_layer(&pdata, CD_MTEXPOLY, CD_CALLOC, NULL, totpolys);
	}

	CustomData_free(&mesh->pdata, mesh->totpoly);
	mesh->pdata = pdata;
	BKE_mesh_update_customdata_pointers(mesh, false);

	mesh->totpoly = totpolys;
}

static Material *find_material(Main *bmain, const char *name)
{
	Material *material, *found_material = NULL;

	for (material = (Material*)bmain->mat.first; material; material = (Material*)material->id.next) {

		if (BLI_strcaseeq(material->id.name+2, name) == true) {
			found_material = material;
			break;
		}
	}

	return found_material;
}

static void assign_materials(Main *bmain, Object *ob, const std::map<std::string, int> &mat_map)
{
	/* Clean up slots */
	while (object_remove_material_slot(ob));

	bool can_assign = true;
	std::map<std::string, int>::const_iterator it = mat_map.begin();

	int matcount = 0;
	for (; it != mat_map.end(); ++it, ++matcount) {
		Material *curmat = give_current_material(ob, matcount);

		if (curmat != NULL) {
			continue;
		}

		if (!object_add_material_slot(ob)) {
			can_assign = false;
			break;
		}
	}

	if (can_assign) {
		it = mat_map.begin();

		for (; it != mat_map.end(); ++it) {
			std::string mat_name = it->first;
			Material *assigned_name = find_material(bmain, mat_name.c_str());

			if (assigned_name == NULL) {
				assigned_name = BKE_material_add(bmain, mat_name.c_str());
			}

			assign_material(ob, assigned_name, it->second, BKE_MAT_ASSIGN_OBJECT);
		}
	}
}

}  /* namespace utils */

/* ****************************** AbcMeshReader ***************************** */

AbcMeshReader::AbcMeshReader(const IObject &object, ImportSettings &settings, bool is_subd)
    : AbcObjectReader(object, settings)
{
	if (is_subd) {
		ISubD isubd_mesh(m_iobject, kWrapExisting);
		m_subd_schema = isubd_mesh.getSchema();
	}
	else {
		IPolyMesh ipoly_mesh(m_iobject, kWrapExisting);
		m_schema = ipoly_mesh.getSchema();
	}
}

bool AbcMeshReader::valid() const
{
	return m_schema.valid() || m_subd_schema.valid();
}

void AbcMeshReader::readObjectData(Main *bmain, Scene *scene, float time)
{
	Mesh *mesh = BKE_mesh_add(bmain, m_data_name.c_str());

	const ISampleSelector sample_sel(time);
	const size_t poly_start = mesh->totpoly;

	bool is_constant = true;

	if (m_subd_schema.valid()) {
		is_constant = m_subd_schema.isConstant();

		const ISubDSchema::Sample sample = m_subd_schema.getValue(sample_sel);

		readVertexDataSample(mesh, sample.getPositions());
		readPolyDataSample(mesh, sample.getFaceIndices(), sample.getFaceCounts());
	}
	else {
		is_constant = m_schema.isConstant();

		const IPolyMeshSchema::Sample sample = m_schema.getValue(sample_sel);

		readVertexDataSample(mesh, sample.getPositions());
		readPolyDataSample(mesh, sample.getFaceIndices(), sample.getFaceCounts());
	}

	BKE_mesh_validate(mesh, false, false);

	m_object = BKE_object_add(bmain, scene, OB_MESH, m_object_name.c_str());
	m_object->data = mesh;

	/* TODO: expose this as a setting to the user? */
	const bool assign_mat = true;

	if (assign_mat) {
		readFaceSetsSample(bmain, mesh, poly_start, sample_sel);
	}

	if (m_settings->is_sequence || !is_constant) {
		addDefaultModifier();
	}
}

void AbcMeshReader::readVertexDataSample(Mesh *mesh, const P3fArraySamplePtr &positions)
{
	utils::mesh_add_verts(mesh, positions->size());
	read_mverts(mesh->mvert, positions);
}

void AbcMeshReader::readPolyDataSample(Mesh *mesh,
                                       const Int32ArraySamplePtr &face_indices,
                                       const Int32ArraySamplePtr &face_counts)
{
	const size_t num_poly = face_counts->size();
	const size_t num_loops = face_indices->size();

	utils::mesh_add_mpolygons(mesh, num_poly);
	utils::mesh_add_mloops(mesh, num_loops);

	IV2fGeomParam::Sample::samp_ptr_type uvsamp_vals;
	const IV2fGeomParam uv = (m_subd_schema.valid() ? m_subd_schema.getUVsParam()
	                                                : m_schema.getUVsParam());

	if (uv.valid()) {
		IV2fGeomParam::Sample uvsamp = uv.getExpandedValue();
		uvsamp_vals = uvsamp.getVals();
	}

	read_mpolys(mesh->mpoly, mesh->mloop, mesh->mloopuv,
	            face_indices, face_counts, uvsamp_vals);
}

void AbcMeshReader::readFaceSetsSample(Main *bmain, Mesh *mesh, size_t poly_start,
                                       const ISampleSelector &sample_sel)
{
	std::vector<std::string> face_sets;

	if (m_subd_schema.valid()) {
		m_subd_schema.getFaceSetNames(face_sets);
	}
	else {
		m_schema.getFaceSetNames(face_sets);
	}

	if (face_sets.empty()) {
		return;
	}

	std::map<std::string, int> mat_map;
	int current_mat = 0;

	for (int i = 0; i < face_sets.size(); ++i) {
		const std::string &grp_name = face_sets[i];

		if (mat_map.find(grp_name) == mat_map.end()) {
			mat_map[grp_name] = 1 + current_mat++;
		}

		const int assigned_mat = mat_map[grp_name];

		const IFaceSet faceset = (m_subd_schema.valid() ? m_subd_schema.getFaceSet(grp_name)
		                                                : m_schema.getFaceSet(grp_name));

		if (!faceset.valid()) {
			continue;
		}

		const IFaceSetSchema face_schem = faceset.getSchema();
		const IFaceSetSchema::Sample face_sample = face_schem.getValue(sample_sel);
		const Int32ArraySamplePtr group_faces = face_sample.getFaces();
		const size_t num_group_faces = group_faces->size();

		for (size_t l = 0; l < num_group_faces; l++) {
			size_t pos = (*group_faces)[l] + poly_start;

			if (pos >= mesh->totpoly) {
				std::cerr << "Faceset overflow on " << faceset.getName() << '\n';
				break;
			}

			MPoly &poly = mesh->mpoly[pos];
			poly.mat_nr = assigned_mat - 1;
		}
	}

	utils::assign_materials(bmain, m_object, mat_map);
}

/* ********************************************************** */

void read_mverts(MVert *mverts, const Alembic::AbcGeom::P3fArraySamplePtr &positions)
{
	for (int i = 0; i < positions->size(); ++i) {
		MVert &mvert = mverts[i];
		Imath::V3f pos_in = (*positions)[i];

		/* Convert Y-up to Z-up. */
		mvert.co[0] = pos_in[0];
		mvert.co[1] = -pos_in[2];
		mvert.co[2] = pos_in[1];
		mvert.bweight = 0;
	}
}

void read_mpolys(MPoly *mpolys, MLoop *mloops, MLoopUV *mloopuvs,
                 const Alembic::AbcGeom::Int32ArraySamplePtr &face_indices,
                 const Alembic::AbcGeom::Int32ArraySamplePtr &face_counts,
                 const Alembic::AbcGeom::V2fArraySamplePtr &uvs)
{
	int loopcount = 0;
	for (int i = 0; i < face_counts->size(); ++i) {
		int face_size = (*face_counts)[i];
		MPoly &poly = mpolys[i];

		poly.loopstart = loopcount;
		poly.totloop = face_size;

		/* TODO: reverse */
		int rev_loop = loopcount;
		for (int f = face_size; f-- ;) {
			MLoop &loop 	= mloops[rev_loop + f];

			if (mloopuvs && uvs) {
				MLoopUV &loopuv = mloopuvs[rev_loop + f];
				loopuv.uv[0] = (*uvs)[loopcount][0];
				loopuv.uv[1] = (*uvs)[loopcount][1];
			}

			loop.v = (*face_indices)[loopcount++];
		}
	}
}

/* ***************************** AbcEmptyReader ***************************** */

AbcEmptyReader::AbcEmptyReader(const Alembic::Abc::IObject &object, ImportSettings &settings)
    : AbcObjectReader(object, settings)
{}

bool AbcEmptyReader::valid() const
{
	return true; // TODO? m_schema.valid();
}

void AbcEmptyReader::readObjectData(Main *bmain, Scene *scene, float /*time*/)
{
	m_object = BKE_object_add(bmain, scene, OB_EMPTY, m_object_name.c_str());
}
