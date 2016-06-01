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

#ifndef __ABC_MESH_WRITER_H__
#define __ABC_MESH_WRITER_H__

#include "abc_shape.h"

struct DerivedMesh;
struct Mesh;
struct ModifierData;

class AbcMeshWriter : public AbcShapeWriter {
	Alembic::AbcGeom::OPolyMeshSchema m_mesh_schema;
	Alembic::AbcGeom::OPolyMeshSchema::Sample m_mesh_sample;

	Alembic::AbcGeom::OSubDSchema m_subdiv_schema;
	Alembic::AbcGeom::OSubDSchema::Sample m_subdiv_sample;

	bool m_has_per_face_materials;
	Alembic::AbcGeom::OFaceSet m_face_set;
	Alembic::Abc::OArrayProperty m_mat_indices;

	bool m_is_animated;
	ModifierData *m_subsurf_mod;

	std::set<std::string> m_layers_written;
	int m_num_face_verts;

	std::vector<std::pair<int, Alembic::Abc::OArrayProperty> > m_vert_layers;
	std::vector<std::pair<int, Alembic::Abc::OArrayProperty> > m_face_layers;

	bool m_has_vertex_weights;

	bool m_is_fluid;
	bool m_subd_type;
	Alembic::Abc::OArrayProperty m_velocity;

public:
	AbcMeshWriter(Scene *scene,
	              Object *ob,
                  AbcTransformWriter *parent,
                  uint32_t timeSampling,
                  ExportSettings &settings);

	~AbcMeshWriter();

private:
	virtual void do_write();

    bool isAnimated() const;
	bool isSubD() const;

	void writeMesh();
	void writeSubD();

	void getMeshInfo(DerivedMesh *dm, std::vector<float> &points,
	                 std::vector<int32_t> &facePoints,
	                 std::vector<int32_t> &faceCounts,
	                 std::vector< std::vector<float> > &uvs,
	                 std::vector<int32_t> &creaseIndices,
	                 std::vector<int32_t> &creaseLengths,
	                 std::vector<float> &creaseSharpness);

	DerivedMesh *getFinalMesh();
	void freeMesh(DerivedMesh *dm);

	void getPoints(DerivedMesh *dm, std::vector<float> &points);
	void getTopology(DerivedMesh *dm, std::vector<int32_t> &facePoints, std::vector<int32_t> &pointCounts);
	void getNormals(DerivedMesh *dm, std::vector<float> &norms);
    void getUVs(DerivedMesh *dm, std::vector< Imath::V2f > &uvs, std::vector<uint32_t> &, int layer_idx);
    void getMaterialIndices(DerivedMesh *dm, std::vector<int32_t> &indices);

	void createArbGeoParams(DerivedMesh *dm);
	void createVertexLayerParam(DerivedMesh *dm, int index, const Alembic::Abc::OCompoundProperty &arbGeoParams);
	void createFaceLayerParam(DerivedMesh *dm, int index, const Alembic::Abc::OCompoundProperty &arbGeoParams);

	void writeArbGeoParams(DerivedMesh *dm);
	void writeVertexLayerParam(DerivedMesh *dm, int index, const Alembic::Abc::OCompoundProperty &arbGeoParams);
	void writeFaceLayerParam(DerivedMesh *dm, int index, const Alembic::Abc::OCompoundProperty &arbGeoParams);
	void getGeoGroups(DerivedMesh *dm, std::map<std::string, std::vector<int32_t> > &geoGroups);
	
	/* fluid surfaces support */
	ModifierData *getFluidSimModifier();
    void getVelocities(DerivedMesh *dm, std::vector<float> &vels);
};

class AbcMeshReader : public AbcObjectReader {
	Alembic::AbcGeom::IPolyMeshSchema m_schema;
	Alembic::AbcGeom::ISubDSchema m_subd_schema;

public:
	AbcMeshReader(const Alembic::Abc::IObject &object, ImportSettings &settings, bool is_subd);

	bool valid() const;

	void readObjectData(Main *bmain, Scene *scene, float time);

private:
	void readFaceSetsSample(Main *bmain, Mesh *mesh, size_t poly_start,
	                        const Alembic::AbcGeom::ISampleSelector &sample_sel);

	void readPolyDataSample(Mesh *mesh,
	                        const Alembic::AbcGeom::Int32ArraySamplePtr &face_indices,
                            const Alembic::AbcGeom::Int32ArraySamplePtr &face_counts);

	void readVertexDataSample(Mesh *mesh,
	                          const Alembic::AbcGeom::P3fArraySamplePtr &positions);
};

/* *********************************** */

struct MLoop;
struct MLoopUV;
struct MPoly;
struct MVert;

void read_mverts(MVert *mverts, const Alembic::AbcGeom::P3fArraySamplePtr &positions);
void read_mpolys(MPoly *mpolys, MLoop *mloops, MLoopUV *mloopuvs,
                 const Alembic::AbcGeom::Int32ArraySamplePtr &face_indices,
                 const Alembic::AbcGeom::Int32ArraySamplePtr &face_counts,
                 const Alembic::AbcGeom::V2fArraySamplePtr &uvs = Alembic::AbcGeom::V2fArraySamplePtr());

namespace utils {

void mesh_add_verts(struct Mesh *mesh, size_t len);

}

/* *********************************** */

class AbcEmptyReader : public AbcObjectReader {
	Alembic::AbcGeom::IXformSchema m_schema;

public:
	AbcEmptyReader(const Alembic::Abc::IObject &object, ImportSettings &settings);

	bool valid() const;

	void readObjectData(Main *bmain, Scene *scene, float time);
};

#endif  /* __ABC_MESH_WRITER_H__ */
