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

#include "BKE_DerivedMesh.h"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
}

AbcMeshWriter::AbcMeshWriter(Scene *sce, Object *obj,
                             AbcTransformWriter *parent, Alembic::Util::uint32_t timeSampling,
                             AbcExportOptions &opts)
    : AbcShapeWriter(sce, obj, parent, timeSampling, opts)
{
	std::string name = getObjectName(m_object);
	name.append("Shape");

	m_is_animated 		= isAnimated();
	m_subsurf_mod 	= NULL;
	m_has_per_face_materials = false;
	m_has_vertex_weights 	 = false;
	bool isSubd 		 = false;

	// if the object is static, use the default static time sampling
	if (!m_is_animated)
		timeSampling = 0;

	if (!m_options.export_subsurfs_as_meshes) {
		// check if the mesh is a subsurf, ignoring disabled modifiers and
		// displace if it's after subsurf.
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

				if ((md->type != eModifierType_Displace) && (md->type != eModifierType_ParticleSystem))
					break; // mesh is not a subsurf. break
			}

			md = md->prev;
		}
	}

	m_is_fluid = (getFluidSimModifier() != NULL);

	while (parent->alembicXform().getChildHeader(name)) {
		name.append("_");
	}

	if (m_options.use_subdiv_schema && isSubd) {
		Alembic::AbcGeom::OSubD subd(parent->alembicXform(), name, m_time_sampling);
		m_subdiv_schema = subd.getSchema();
	}
	else {
		Alembic::AbcGeom::OPolyMesh mesh(parent->alembicXform(), name, m_time_sampling);
		m_mesh_schema = mesh.getSchema();

		Alembic::AbcGeom::OCompoundProperty typeContainer = m_mesh_schema.getUserProperties();
		Alembic::AbcGeom::OBoolProperty type(typeContainer, "meshtype");
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
	return m_subd_type;//mSubDSchema.valid();
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

	if (m_options.use_subdiv_schema && m_subdiv_schema.valid())
		writeSubD();
	else
		writeMesh();
}

void AbcMeshWriter::writeMesh()
{
	DerivedMesh *dm = getFinalMesh();

	try {
		std::vector<std::vector<float> > uvs;
		std::vector<float> points, normals, creaseSharpness;
		std::vector<Alembic::Util::int32_t> facePoints, faceCounts;

		std::vector<Alembic::Util::int32_t> creaseIndices, creaseLengths;

		std::vector<uint32_t> uvIdx;
		std::vector<Imath::V2f> uvValArray;

		int active_uvlayer = CustomData_get_active_layer(&dm->loopData, CD_MLOOPUV);

		if (active_uvlayer >= 0)
			getUVs(dm, uvValArray, uvIdx, active_uvlayer);
		getMeshInfo(dm, points, facePoints, faceCounts, uvs, creaseIndices,
		            creaseLengths, creaseSharpness);

		if (m_first_frame) {
			// Here I'm creating materials' facesets
			std::map< std::string, std::vector<Alembic::Util::int32_t>  > geoGroups;
			getGeoGroups(dm, geoGroups);

			for (std::map< std::string, std::vector<Alembic::Util::int32_t>  >::iterator it = geoGroups.begin(); it != geoGroups.end(); ++it) {
				Alembic::AbcGeom::OFaceSet faceSet;

				faceSet = m_mesh_schema.createFaceSet(it->first);

				Alembic::AbcGeom::OFaceSetSchema::Sample samp;
				samp.setFaces(Alembic::Abc::Int32ArraySample(it->second));
				faceSet.getSchema().set(samp);
			}

			if (hasProperties(reinterpret_cast<ID *>(m_object->data))) {
				if (m_options.export_props_as_geo_params)
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_mesh_schema.getArbGeomParams(), false);
				else
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_mesh_schema.getUserProperties(), true);
			}
			createArbGeoParams(dm);
		}

		// Export UVs
		Alembic::AbcGeom::OV2fGeomParam::Sample uvSamp;
		if (!uvIdx.empty() && !uvValArray.empty()) {
			uvSamp.setScope(Alembic::AbcGeom::kFacevaryingScope);
			uvSamp.setVals(Alembic::AbcGeom::V2fArraySample(
			                   &uvValArray.front(),
			                   uvValArray.size()));
			Alembic::AbcGeom::UInt32ArraySample idxSamp(
			            (const uint32_t *) &uvIdx.front(),
			            uvIdx.size());
			uvSamp.setIndices(idxSamp);
		}

		// Normals export
		Alembic::AbcGeom::ON3fGeomParam::Sample normalsSamp;
		if (!normals.empty()) {
			normalsSamp.setScope(Alembic::AbcGeom::kFacevaryingScope);
			normalsSamp.setVals(
			            Alembic::AbcGeom::N3fArraySample(
			                (const Imath::V3f *) &normals.front(),
			                normals.size() / 3));
		}

		m_mesh_sample = Alembic::AbcGeom::OPolyMeshSchema::Sample(
		                  Alembic::Abc::V3fArraySample(
		                      (const Imath::V3f *) &points.front(),
		                      points.size() / 3),
		                  Alembic::Abc::Int32ArraySample(facePoints),
		                  Alembic::Abc::Int32ArraySample(faceCounts), uvSamp,
		                  normalsSamp);

		// TODO : export all uvmaps

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
		std::vector<Alembic::Util::int32_t> facePoints, faceCounts;
		std::vector<Alembic::Util::int32_t> creaseIndices, creaseLengths;
		std::vector<uint32_t> uvIdx;
		std::vector<Imath::V2f> uvValArray;

		int active_uvlayer  = CustomData_get_active_layer(&dm->loopData, CD_MLOOPUV);
		//int num_uv_layers = dm->loopData.totlayer;

		if (active_uvlayer >= 0)
			getUVs(dm, uvValArray, uvIdx, active_uvlayer);

		getMeshInfo(dm, points, facePoints, faceCounts, uvs, creaseIndices,
		            creaseLengths, creaseSharpness);

		if (m_first_frame) {
			std::map< std::string, std::vector<Alembic::Util::int32_t>  > geoGroups;
			getGeoGroups(dm, geoGroups);

			for (std::map< std::string, std::vector<Alembic::Util::int32_t>  >::iterator it = geoGroups.begin(); it != geoGroups.end(); ++it) {
				Alembic::AbcGeom::OFaceSet faceSet;

				faceSet = m_subdiv_schema.createFaceSet(it->first);

				Alembic::AbcGeom::OFaceSetSchema::Sample samp;
				samp.setFaces(Alembic::Abc::Int32ArraySample(it->second));
				faceSet.getSchema().set(samp);
			}

			if (hasProperties(reinterpret_cast<ID *>(m_object->data))) {
				if (m_options.export_props_as_geo_params)
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_subdiv_schema.getArbGeomParams(), false);
				else
					writeProperties(reinterpret_cast<ID *>(m_object->data),
					                m_subdiv_schema.getUserProperties(), true);
			}

			createArbGeoParams(dm);
		}

		Alembic::AbcGeom::OV2fGeomParam::Sample uvSamp;

		m_subdiv_sample = Alembic::AbcGeom::OSubDSchema::Sample(
		                  Alembic::Abc::V3fArraySample(
		                      (const Imath::V3f *) &points.front(),
		                      points.size() / 3),
		                  Alembic::Abc::Int32ArraySample(facePoints),
		                  Alembic::Abc::Int32ArraySample(faceCounts));

		if (!uvIdx.empty() && !uvValArray.empty()) {
			uvSamp.setScope(Alembic::AbcGeom::kFacevaryingScope);
			uvSamp.setVals(Alembic::AbcGeom::V2fArraySample(
			                   (const Imath::V2f *) &uvValArray.front(),
			                   uvValArray.size()));
			Alembic::AbcGeom::UInt32ArraySample idxSamp(
			            (const uint32_t *) &uvIdx.front(),
			            uvIdx.size());
			uvSamp.setIndices(idxSamp);
			m_subdiv_sample.setUVs(uvSamp);
		}

		if (!creaseIndices.empty()) {
			m_subdiv_sample.setCreaseIndices(
			            Alembic::Abc::Int32ArraySample(creaseIndices));
			m_subdiv_sample.setCreaseLengths(
			            Alembic::Abc::Int32ArraySample(creaseLengths));
			m_subdiv_sample.setCreaseSharpnesses(
			            Alembic::Abc::FloatArraySample(creaseSharpness));
		}

		//mSubDSample.setChildBounds(bounds());
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
                                std::vector<Alembic::Util::int32_t> &facePoints,
                                std::vector<Alembic::Util::int32_t> &faceCounts,
                                std::vector<std::vector<float> > &uvs,
                                std::vector<Alembic::Util::int32_t> &creaseIndices,
                                std::vector<Alembic::Util::int32_t> &creaseLengths,
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

	if (m_rotate_matrix) {
		for (int i = 0, e = dm->getNumVerts(dm); i < e; ++i) {
			points.push_back(verts[i].co[0]);
			points.push_back(verts[i].co[2]);
			points.push_back(-verts[i].co[1]);
		}
	}
	else {
		for (int i = 0, e = dm->getNumVerts(dm); i < e; ++i) {
			points.push_back(verts[i].co[0]);
			points.push_back(verts[i].co[1]);
			points.push_back(verts[i].co[2]);
		}
	}

	calcBounds(points);
}

void AbcMeshWriter::getTopology(DerivedMesh *dm,
                                std::vector<Alembic::Util::int32_t> &facePoints,
                                std::vector<Alembic::Util::int32_t> &pointCounts)
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
	// TODO: check if we need to reverse the normals.

	float nscale = 1.0f / 32767.0f;
	norms.clear();

	if (m_options.export_normals) {
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

	if (!(m_options.export_uvs && mloopuv_array)) {
		return;
	}

	int num_poly = dm->getNumPolys(dm);
	MPoly *polygons = dm->getPolyArray(dm);

	if (!m_options.pack_uv) {
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
                                       std::vector<Alembic::Util::int32_t> &indices)
{
	indices.clear();
	indices.reserve(dm->getNumTessFaces(dm));

	MFace *faces = dm->getTessFaceArray(dm);

	for (int i = 1, e = dm->getNumTessFaces(dm); i < e; ++i) {
		MFace *face = &faces[i];
		//assert(face->mat_nr >= 0);
		indices.push_back(face->mat_nr);
	}
}

void AbcMeshWriter::createArbGeoParams(DerivedMesh *dm)
{
	if (m_is_fluid) {
		// TODO: replace this, when velocities are added by default to schemas.
		Alembic::AbcGeom::OV3fGeomParam param;

		if (m_subdiv_schema.valid())
			param = Alembic::AbcGeom::OV3fGeomParam(
			            m_subdiv_schema.getArbGeomParams(), "velocity", false,
			            Alembic::AbcGeom::kVertexScope, 1, m_time_sampling);
		else
			param = Alembic::AbcGeom::OV3fGeomParam(
			            m_mesh_schema.getArbGeomParams(), "velocity", false,
			            Alembic::AbcGeom::kVertexScope, 1, m_time_sampling);

		m_velocity = param.getValueProperty();

		return; // we don't need anything more for fluid meshes
	}

	std::string layerName;

	for (int i = 0; i < dm->vertData.totlayer; ++i) {
		layerName = dm->vertData.layers[i].name;

		// skip unnamed layers
		if (layerName == "")
			continue;

		if (m_subdiv_schema.valid())
			createVertexLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		else
			createVertexLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
	}

	for (int i = 0; i < dm->polyData.totlayer; ++i) {
		CustomDataLayer *layer = &dm->polyData.layers[i];
		layerName = dm->polyData.layers[i].name;

		// skip unnamed layers
		if (layerName == "")
			continue;

		if (layer->type == CD_MCOL && !m_options.export_vcols)
			continue;

		if (m_subdiv_schema.valid())
			createFaceLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		else
			createFaceLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
	}

	if (m_options.export_vweigths) {
	}
}

void AbcMeshWriter::createVertexLayerParam(DerivedMesh *dm, int index,
                                               Alembic::Abc::OCompoundProperty arbGeoParams)
{
	CustomDataLayer *layer = &dm->vertData.layers[index];
	std::string layerName = layer->name;

	// we have already a layer named layerName. skip
	if (m_layers_written.count(layerName) != 0)
		return;

	switch (layer->type) {
		case CD_PROP_FLT: {
			Alembic::AbcGeom::OFloatGeomParam param(arbGeoParams, layerName, false,
			                                        Alembic::AbcGeom::kVertexScope, 1, m_time_sampling);
			m_layers_written.insert(layerName);
			m_vert_layers.push_back(
			            std::pair<int, Alembic::Abc::OArrayProperty>(index,
			                                                         param.getValueProperty()));
		}
			break;

		case CD_PROP_INT: {
			Alembic::AbcGeom::OInt32GeomParam param(arbGeoParams, layerName, false,
			                                        Alembic::AbcGeom::kVertexScope, 1, m_time_sampling);
			m_layers_written.insert(layerName);
			m_vert_layers.push_back(
			            std::pair<int, Alembic::Abc::OArrayProperty>(index,
			                                                         param.getValueProperty()));
		}
			break;
	}
}

void AbcMeshWriter::createFaceLayerParam(DerivedMesh *dm, int index,
                                             Alembic::Abc::OCompoundProperty arbGeoParams)
{
	CustomDataLayer *layer = &dm->polyData.layers[index];
	std::string layerName = layer->name;

	// we have already a layer named layerName. skip
	if (m_layers_written.count(layerName) != 0)
		return;

	switch (layer->type) {
		case CD_MCOL: {
			Alembic::AbcGeom::OC3fGeomParam param(arbGeoParams, layerName, false,
			                                      Alembic::AbcGeom::kFacevaryingScope, 1, m_time_sampling);
			m_layers_written.insert(layerName);
			m_face_layers.push_back(
			            std::pair<int, Alembic::Abc::OArrayProperty>(index,
			                                                         param.getValueProperty()));
		}
			break;

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
		return; // we have all we need.
	}

	// vertex data
	for (int i = 0; i < m_vert_layers.size(); ++i) {
		if (m_subdiv_schema.valid())
			writeVertexLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		else
			writeVertexLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
	}

	// face varying data
	for (int i = 0; i < m_face_layers.size(); ++i) {
		if (m_subdiv_schema.valid())
			writeFaceLayerParam(dm, i, m_subdiv_schema.getArbGeomParams());
		else
			writeFaceLayerParam(dm, i, m_mesh_schema.getArbGeomParams());
	}

	if (m_first_frame && m_has_per_face_materials) {
		std::vector<Alembic::Util::int32_t> faceVals;

		if (m_options.export_face_sets || m_options.export_mat_indices)
			getMaterialIndices(dm, faceVals);

		if (m_options.export_face_sets) {
			Alembic::AbcGeom::OFaceSetSchema::Sample samp;
			samp.setFaces(Alembic::Abc::Int32ArraySample(faceVals));
			m_face_set.getSchema().set(samp);
		}

		if (m_options.export_mat_indices) {
			Alembic::AbcCoreAbstract::ArraySample samp(&(faceVals.front()),
			                                           m_mat_indices.getDataType(),
			                                           Alembic::Util::Dimensions(dm->getNumTessFaces(dm)));
			m_mat_indices.set(samp);
		}
	}
}

void AbcMeshWriter::writeVertexLayerParam(DerivedMesh *dm, int index,
                                          Alembic::Abc::OCompoundProperty arbGeoParams)
{
	CustomDataLayer *layer = &dm->vertData.layers[m_vert_layers[index].first];
	std::string layerName = layer->name;
	int totvert = dm->getNumVerts(dm);

	switch (layer->type) {
		case CD_PROP_FLT:
		case CD_PROP_INT: {
			Alembic::AbcCoreAbstract::ArraySample samp(layer->data,
			                                           m_vert_layers[index].second.getDataType(),
			                                           Alembic::Util::Dimensions(totvert));

			m_vert_layers[index].second.set(samp);
		}
			break;
	};
}

void AbcMeshWriter::writeFaceLayerParam(DerivedMesh *dm, int index,
                                            Alembic::Abc::OCompoundProperty arbGeoParams)
{
	CustomDataLayer *layer = &dm->polyData.layers[m_face_layers[index].first];

	int totpolys = dm->getNumPolys(dm);
	std::vector<float> buffer;

	switch (layer->type) {
		case CD_MCOL: {
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

void AbcMeshWriter::getVelocities(DerivedMesh *dm,
                                      std::vector<float> &vels)
{
	int totverts = dm->getNumVerts(dm);

	vels.clear();
	vels.reserve(dm->getNumVerts(dm));

	ModifierData *md = getFluidSimModifier();
	FluidsimModifierData *fmd = reinterpret_cast<FluidsimModifierData *>(md);
	FluidsimSettings *fss = fmd->fss;

	if (fss->meshVelocities) {
		float *meshVels = reinterpret_cast<float *>(fss->meshVelocities);

		if (m_rotate_matrix) {
			for (int i = 0; i < totverts; ++i) {
				vels.push_back(meshVels[0]);
				vels.push_back(meshVels[2]);
				vels.push_back(-meshVels[1]);
				meshVels += 3;
			}
		}
		else {
			{
				vels.push_back(meshVels[0]);
				vels.push_back(meshVels[1]);
				vels.push_back(meshVels[2]);
				meshVels += 3;
			}
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

void AbcMeshWriter::getGeoGroups(DerivedMesh *dm,
                                 std::map<std::string,
                                 std::vector<Alembic::Util::int32_t> > &geoGroups)
{
	int num_poly = dm->getNumPolys(dm);
	MPoly *polygons = dm->getPolyArray(dm);

	for (int i = 0; i < num_poly; ++i) {
		MPoly &current_poly = polygons[i];
		short mnr = current_poly.mat_nr;

		Material *mat = give_current_material(m_object, mnr + 1);
		if (!mat)
			continue;

		std::string name = (mat->id.name + 2);

		std::replace(name.begin(), name.end(), ' ', '_');
		std::replace(name.begin(), name.end(), '.', '_');
		std::replace(name.begin(), name.end(), ':', '_');

		if (geoGroups.find(name) == geoGroups.end()) {
			std::vector<Alembic::Util::int32_t> faceArray;
			geoGroups[name] = faceArray;
		}

		geoGroups[name].push_back(i);
	}

	if (geoGroups.size() == 0) {
		Material *mat = give_current_material(m_object, 1);
		std::string name;

		if (!mat)
			name = "default";
		else {
			std::string matname = mat->id.name + 2;
			name = matname;
		}

		std::replace(name.begin(), name.end(), ' ', '_');
		std::replace(name.begin(), name.end(), '.', '_');
		std::replace(name.begin(), name.end(), ':', '_');

		std::vector<Alembic::Util::int32_t> faceArray;

		for (int i = 0, e = dm->getNumTessFaces(dm); i < e; ++i) {
			faceArray.push_back(i);
		}

		geoGroups[name] = faceArray;
	}
}

/* ******************************* mesh reader ****************************** */

// Some helpers for mesh generation
namespace mesh_utils {

static void mesh_add_verts(Mesh *mesh, size_t len)
{
	if (len == 0) {
		return;
	}

	int totvert = mesh->totvert + len;
	CustomData vdata;
	CustomData_copy(&mesh->vdata, &vdata, CD_MASK_MESH, CD_DEFAULT, totvert);
	CustomData_copy_data(&mesh->vdata, &vdata, 0, 0, mesh->totvert);

	if (!CustomData_has_layer(&vdata, CD_MVERT))
		CustomData_add_layer(&vdata, CD_MVERT, CD_CALLOC, NULL, totvert);

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

	const int totpolys = mesh->totpoly + len;   /* new face count */

	CustomData pdata;
	CustomData_copy(&mesh->pdata, &pdata, CD_MASK_MESH, CD_DEFAULT, totpolys);
	CustomData_copy_data(&mesh->pdata, &pdata, 0, 0, mesh->totpoly);

	if (!CustomData_has_layer(&pdata, CD_MPOLY))
		CustomData_add_layer(&pdata, CD_MPOLY, CD_CALLOC, NULL, totpolys);

	if (!CustomData_has_layer(&pdata, CD_MTEXPOLY))
		CustomData_add_layer(&pdata, CD_MTEXPOLY, CD_CALLOC, NULL, totpolys);

	CustomData_free(&mesh->pdata, mesh->totpoly);
	mesh->pdata = pdata;
	BKE_mesh_update_customdata_pointers(mesh, false);

	mesh->totpoly = totpolys;
}

} /* mesh_utils */

AbcMeshReader::AbcMeshReader(const Alembic::Abc::IObject &object, int from_forward, int from_up)
    : AbcObjectReader(object, from_forward, from_up)
{
	Alembic::AbcGeom::IPolyMesh abc_mesh(m_iobject, Alembic::AbcGeom::kWrapExisting);
	m_schema = abc_mesh.getSchema();
}

bool AbcMeshReader::valid() const
{
	return m_schema.valid();
}

void AbcMeshReader::readObject(Main *bmain, Scene *scene, float time)
{
	Mesh *blender_mesh = BKE_mesh_add(bmain, m_data_name.c_str());

	size_t idx_pos  = blender_mesh->totpoly;
	size_t vtx_pos  = blender_mesh->totvert;
	size_t loop_pos = blender_mesh->totloop;

	Alembic::AbcGeom::IV2fGeomParam uv = m_schema.getUVsParam();
	Alembic::Abc::ISampleSelector sample_sel(time);

	Alembic::AbcGeom::IPolyMeshSchema::Sample smp = m_schema.getValue(sample_sel);
	Alembic::Abc::P3fArraySamplePtr positions = smp.getPositions();
	Alembic::Abc::Int32ArraySamplePtr face_indices = smp.getFaceIndices();
	Alembic::Abc::Int32ArraySamplePtr face_counts  = smp.getFaceCounts();

	const size_t vertex_count = positions->size();
	const size_t num_poly = face_counts->size();
	const size_t num_loops = face_indices->size();

	std::vector<std::string> face_sets;
	m_schema.getFaceSetNames(face_sets);

	mesh_utils::mesh_add_verts(blender_mesh, vertex_count);
	mesh_utils::mesh_add_mpolygons(blender_mesh, num_poly);
	mesh_utils::mesh_add_mloops(blender_mesh, num_loops);

	Alembic::AbcGeom::IV2fGeomParam::Sample::samp_ptr_type uvsamp_vals;

	if (uv.valid()) {
		Alembic::AbcGeom::IV2fGeomParam::Sample uvsamp = uv.getExpandedValue();
		uvsamp_vals = uvsamp.getVals();
	}

	int j = vtx_pos;
	for (int i = 0; i < vertex_count; ++i, ++j) {
		MVert &mvert = blender_mesh->mvert[j];
		Alembic::Abc::V3f pos_in = (*positions)[i];

		mvert.co[0] = pos_in[0];
		mvert.co[1] = pos_in[1];
		mvert.co[2] = pos_in[2];

		mvert.bweight = 0;
	}

	if (m_do_convert_mat) {
		j = vtx_pos;
		for (int i = 0; i < vertex_count; ++i, ++j) {
			MVert &mvert = blender_mesh->mvert[j];
			mul_m3_v3(m_conversion_mat, mvert.co);
		}
	}

	j = idx_pos;
	int loopcount = loop_pos;
	for (int i = 0; i < num_poly; ++i, ++j) {
		int face_size = (*face_counts)[i];
		MPoly &poly = blender_mesh->mpoly[j];

		poly.loopstart = loopcount;
		poly.totloop   = face_size;

		// TODO : reverse
		int rev_loop = loopcount;
		for (int f = face_size; f-- ;) {
			MLoop &loop 	= blender_mesh->mloop[rev_loop+f];
			MLoopUV &loopuv = blender_mesh->mloopuv[rev_loop+f];

			if (uvsamp_vals) {
				loopuv.uv[0] = (*uvsamp_vals)[loopcount][0];
				loopuv.uv[1] = (*uvsamp_vals)[loopcount][1];
			}

			loop.v = (*face_indices)[loopcount++];
		}
	}

	// TODO
#if 0
	if (assign_mat) {
		std::map<std::string, int> &mat_map = abc_manager->mesh_map[key].mat_map;
		int &current_mat = abc_manager->mesh_map[key].current_mat;

		for (int i = 0; i < face_sets.size(); ++i) {
			std::string grp_name = face_sets[i];

			if (mat_map.find(grp_name) == mat_map.end()) {
				mat_map[grp_name] = 1 + current_mat++;
			}

			int assigned_mat = mat_map[grp_name];

			Alembic::AbcGeom::IFaceSet faceset 					= m_schema.getFaceSet(face_sets[i]);
			if (!faceset.valid())
				continue;
			Alembic::AbcGeom::IFaceSetSchema face_schem 			= faceset.getSchema();
			Alembic::AbcGeom::IFaceSetSchema::Sample face_sample 	= face_schem.getValue(sample_sel);
			Alembic::Abc::Int32ArraySamplePtr group_faces 	= face_sample.getFaces();
			size_t num_group_faces 				= group_faces->size();

			for (size_t l = 0; l < num_group_faces; l++) {
				size_t pos = (*group_faces)[l]+idx_pos;

				if (pos >= blender_mesh->totpoly) {
					std::cerr << "Faceset overflow on " << faceset.getName() << std::endl;
					break;
				}

				MPoly  &poly = blender_mesh->mpoly[pos];
				poly.mat_nr = assigned_mat - 1;
			}
		}
	}
#endif

	// Compute edge array is done here
	BKE_mesh_validate(blender_mesh, false, false);

	m_object = BKE_object_add(bmain, scene, OB_MESH, m_object_name.c_str());
	m_object->data = blender_mesh;
}
