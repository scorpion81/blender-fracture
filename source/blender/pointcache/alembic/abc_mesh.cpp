/*
 * Copyright 2014, Blender Foundation.
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

#include "abc_interpolate.h"
#include "abc_mesh.h"

extern "C" {
#include "BLI_math.h"

#include "DNA_object_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"

#include "BKE_DerivedMesh.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_mesh.h"

#include "PIL_time.h"
}

#include "PTC_api.h"

//#define USE_TIMING

namespace PTC {

using namespace Abc;
using namespace AbcGeom;

/* CD layers that are stored in generic customdata arrays created with CD_ALLOC */
/* XXX CD_MASK_MTFACE and CD_MASK_MTEXPOLY are currently still needed as dummies for syncing
 * particle UV and MCol layers to the mesh shader attributes ...
 */
static CustomDataMask CD_MASK_CACHE_EXCLUDE =
        CD_MASK_MVERT | CD_MASK_MEDGE | CD_MASK_MFACE | CD_MASK_MPOLY | CD_MASK_MLOOP |
        /*CD_MASK_MTFACE | CD_MASK_MTEXPOLY |*/
        CD_MASK_PROP_STR |
        CD_MASK_SHAPEKEY | CD_MASK_SHAPE_KEYINDEX |
        CD_MASK_MDISPS | CD_MASK_CREASE | CD_MASK_BWEIGHT | CD_MASK_RECAST | CD_MASK_PAINT_MASK |
        CD_MASK_GRID_PAINT_MASK | CD_MASK_MVERT_SKIN | CD_MASK_FREESTYLE_EDGE | CD_MASK_FREESTYLE_FACE;

static CustomDataMask CD_MASK_CACHE_VERT = ~(CD_MASK_CACHE_EXCLUDE | CD_MASK_NORMAL);
static CustomDataMask CD_MASK_CACHE_EDGE = ~(CD_MASK_CACHE_EXCLUDE);
static CustomDataMask CD_MASK_CACHE_FACE = ~(CD_MASK_CACHE_EXCLUDE);
static CustomDataMask CD_MASK_CACHE_POLY = ~(CD_MASK_CACHE_EXCLUDE);
static CustomDataMask CD_MASK_CACHE_LOOP = ~(CD_MASK_CACHE_EXCLUDE);

struct MVertSample {
	std::vector<V3f> co;
	std::vector<N3f> no;
	std::vector<int8_t> flag;
	std::vector<int8_t> bweight;
};

struct MEdgeSample {
	std::vector<uint32_t> verts;
	std::vector<int16_t> flag;
	std::vector<int8_t> crease;
	std::vector<int8_t> bweight;
};

struct MPolySample {
	/*std::vector<int32_t> loopstart;*/ /* loopstart is not stored explicitly */
	std::vector<int32_t> totloop;
	std::vector<int16_t> mat_nr;
	std::vector<int8_t> flag;
};

struct MLoopSample {
	/* XXX these are unsigned int in DNA, but Alembic expects signed int */
	std::vector<int32_t> verts;
	std::vector<int32_t> edges;
};

AbcDerivedMeshWriter::AbcDerivedMeshWriter(const std::string &name, Object *ob, DerivedMesh **dm_ptr) :
    DerivedMeshWriter(ob, dm_ptr, name),
    m_vert_data_writer("vertex_data", CD_MASK_CACHE_VERT),
    m_edge_data_writer("edge_data", CD_MASK_CACHE_EDGE),
    m_face_data_writer("face_data", CD_MASK_CACHE_FACE),
    m_poly_data_writer("poly_data", CD_MASK_CACHE_POLY),
    m_loop_data_writer("loop_data", CD_MASK_CACHE_LOOP)
{
}

AbcDerivedMeshWriter::~AbcDerivedMeshWriter()
{
}

void AbcDerivedMeshWriter::init_abc(OObject parent)
{
	if (m_mesh)
		return;
	
	m_mesh = OPolyMesh(parent, m_name, abc_archive()->frame_sampling_index());
	
	OPolyMeshSchema &schema = m_mesh.getSchema();
//	OCompoundProperty geom_props = schema.getArbGeomParams();
	OCompoundProperty user_props = schema.getUserProperties();
	
	m_prop_vert_normals = ON3fArrayProperty(user_props, "vertex_normals", frame_sampling());
	m_prop_vert_flag = OCharArrayProperty(user_props, "vertex_flag", frame_sampling());
	m_prop_vert_bweight = OCharArrayProperty(user_props, "vertex_bweight", frame_sampling());
	
	m_prop_edge_verts = OUInt32ArrayProperty(user_props, "edge_verts", frame_sampling());
	m_prop_edge_flag = OInt16ArrayProperty(user_props, "edge_flag", frame_sampling());
	m_prop_edge_crease = OCharArrayProperty(user_props, "edge_crease", frame_sampling());
	m_prop_edge_bweight = OCharArrayProperty(user_props, "edge_bweight", frame_sampling());
	
	m_prop_poly_mat_nr = OInt16ArrayProperty(user_props, "poly_mat_nr", frame_sampling());
	m_prop_poly_flag = OCharArrayProperty(user_props, "poly_flag", frame_sampling());
	
	m_prop_loop_verts = OInt32ArrayProperty(user_props, "loop_verts", frame_sampling());
	m_prop_loop_edges = OInt32ArrayProperty(user_props, "loop_edges", frame_sampling());
	
	m_vert_data_writer.init(frame_sampling());
	m_edge_data_writer.init(frame_sampling());
	m_face_data_writer.init(frame_sampling());
	m_poly_data_writer.init(frame_sampling());
	m_loop_data_writer.init(frame_sampling());
}

/* XXX modifiers are not allowed to generate poly normals on their own!
 * see assert in DerivedMesh.c : dm_ensure_display_normals
 */
#if 0
static void ensure_normal_data(DerivedMesh *dm)
{
	MVert *mverts = dm->getVertArray(dm);
	MLoop *mloops = dm->getLoopArray(dm);
	MPoly *mpolys = dm->getPolyArray(dm);
	CustomData *cdata = dm->getPolyDataLayout(dm);
	float (*polynors)[3];
	int totvert = dm->getNumVerts(dm);
	int totloop = dm->getNumLoops(dm);
	int totpoly = dm->getNumPolys(dm);
	
	if (CustomData_has_layer(cdata, CD_NORMAL))
		polynors = (float (*)[3])CustomData_get_layer(cdata, CD_NORMAL);
	else
		polynors = (float (*)[3])CustomData_add_layer(cdata, CD_NORMAL, CD_CALLOC, NULL, totpoly);
	
	BKE_mesh_calc_normals_poly(mverts, totvert, mloops, mpolys, totloop, totpoly, polynors, false);
}
#endif

static void create_sample_verts(DerivedMesh *dm, MVertSample &sample)
{
	MVert *mv, *mverts = dm->getVertArray(dm);
	int i, totvert = dm->getNumVerts(dm);
	
	sample.co.reserve(totvert);
	sample.no.reserve(totvert);
	sample.flag.reserve(totvert);
	sample.bweight.reserve(totvert);
	for (i = 0, mv = mverts; i < totvert; ++i, ++mv) {
		float nor[3];
		
		sample.co.push_back(V3f(mv->co[0], mv->co[1], mv->co[2]));
		
		normal_short_to_float_v3(nor, mv->no);
		sample.no.push_back(N3f(nor[0], nor[1], nor[2]));
		
		sample.flag.push_back(mv->flag);
		sample.bweight.push_back(mv->bweight);
	}
}

static void create_sample_edges(DerivedMesh *dm, MEdgeSample &sample)
{
	MEdge *me, *medges = dm->getEdgeArray(dm);
	int i, totedge = dm->getNumEdges(dm);
	
	sample.verts.reserve(totedge * 2);
	sample.flag.reserve(totedge);
	sample.crease.reserve(totedge);
	sample.bweight.reserve(totedge);
	
	for (i = 0, me = medges; i < totedge; ++i, ++me) {
		sample.verts.push_back(me->v1);
		sample.verts.push_back(me->v2);
		sample.flag.push_back(me->flag);
		sample.crease.push_back(me->crease);
		sample.bweight.push_back(me->bweight);
	}
}

static void create_sample_polys(DerivedMesh *dm, MPolySample &sample)
{
	MPoly *mp, *mpolys = dm->getPolyArray(dm);
	int i, totpoly = dm->getNumPolys(dm);
	
	sample.totloop.reserve(totpoly);
	sample.mat_nr.reserve(totpoly);
	sample.flag.reserve(totpoly);
	
	for (i = 0, mp = mpolys; i < totpoly; ++i, ++mp) {
		sample.totloop.push_back(mp->totloop);
		sample.mat_nr.push_back(mp->mat_nr);
		sample.flag.push_back(mp->flag);
	}
}

static void create_sample_loops(DerivedMesh *dm, MLoopSample &sample)
{
	MLoop *ml, *mloops = dm->getLoopArray(dm);
	int i, totloop = dm->getNumLoops(dm);
	
	sample.verts.reserve(totloop);
	sample.edges.reserve(totloop);
	
	for (i = 0, ml = mloops; i < totloop; ++i, ++ml) {
		sample.verts.push_back(ml->v);
		sample.edges.push_back(ml->e);
	}
}

static N3fArraySample create_sample_loop_normals(DerivedMesh *dm, std::vector<N3f> &data)
{
	CustomData *cdata = dm->getLoopDataLayout(dm);
	float (*nor)[3], (*loopnors)[3];
	int i, totloop = dm->getNumLoops(dm);
	
	if (!CustomData_has_layer(cdata, CD_NORMAL))
		return N3fArraySample();
	
	loopnors = (float (*)[3])CustomData_get_layer(cdata, CD_NORMAL);
	
	data.reserve(totloop);
	for (i = 0, nor = loopnors; i < totloop; ++i, ++nor) {
		float *vec = *nor;
		data.push_back(N3f(vec[0], vec[1], vec[2]));
	}
	
	return N3fArraySample(data);
}

void AbcDerivedMeshWriter::write_sample()
{
	if (!m_mesh)
		return;
	
	DerivedMesh *output_dm = *m_dm_ptr;
	if (!output_dm)
		return;
	
	/* TODO make this optional by a flag? */
	/* XXX does not work atm, see comment above */
	/*ensure_normal_data(output_dm);*/
	
	OPolyMeshSchema &schema = m_mesh.getSchema();
	OCompoundProperty user_props = schema.getUserProperties();
	
	MVertSample vert_sample;
	MEdgeSample edge_sample;
	MPolySample poly_sample;
	MLoopSample loop_sample;
	
	std::vector<N3f> loop_normals_buffer;
	
	// TODO decide how to handle vertex/face normals, in caching vs. export ...
	DM_ensure_normals(output_dm);
	
	create_sample_verts(output_dm, vert_sample);
	create_sample_edges(output_dm, edge_sample);
	create_sample_polys(output_dm, poly_sample);
	create_sample_loops(output_dm, loop_sample);
	
	N3fArraySample lnormals = create_sample_loop_normals(output_dm, loop_normals_buffer);
	
	OPolyMeshSchema::Sample sample = OPolyMeshSchema::Sample(
	            P3fArraySample(vert_sample.co),
	            Int32ArraySample(loop_sample.verts),
	            Int32ArraySample(poly_sample.totloop),
	            OV2fGeomParam::Sample(), /* XXX define how/which UV map should be considered primary for the alembic schema */
	            ON3fGeomParam::Sample(lnormals, kFacevaryingScope)
	            );
	schema.set(sample);
	
	m_prop_vert_normals.set(N3fArraySample(vert_sample.no));
	m_prop_vert_flag.set(CharArraySample(vert_sample.flag));
	m_prop_vert_bweight.set(CharArraySample(vert_sample.bweight));
	
	m_prop_edge_verts.set(UInt32ArraySample(edge_sample.verts));
	m_prop_edge_flag.set(Int16ArraySample(edge_sample.flag));
	m_prop_edge_crease.set(CharArraySample(edge_sample.crease));
	m_prop_edge_bweight.set(CharArraySample(edge_sample.bweight));
	
	m_prop_poly_mat_nr.set(Int16ArraySample(poly_sample.mat_nr));
	m_prop_poly_flag.set(CharArraySample(poly_sample.flag));
	
	m_prop_loop_verts.set(Int32ArraySample(loop_sample.verts));
	m_prop_loop_edges.set(Int32ArraySample(loop_sample.edges));
	
	CustomData *vdata = output_dm->getVertDataLayout(output_dm);
	int num_vdata = output_dm->getNumVerts(output_dm);
	m_vert_data_writer.write_sample(vdata, num_vdata, user_props);
	
	CustomData *edata = output_dm->getEdgeDataLayout(output_dm);
	int num_edata = output_dm->getNumEdges(output_dm);
	m_edge_data_writer.write_sample(edata, num_edata, user_props);
	
	CustomData *pdata = output_dm->getPolyDataLayout(output_dm);
	int num_pdata = output_dm->getNumPolys(output_dm);
	m_poly_data_writer.write_sample(pdata, num_pdata, user_props);
	
	CustomData *ldata = output_dm->getLoopDataLayout(output_dm);
	int num_ldata = output_dm->getNumLoops(output_dm);
	m_loop_data_writer.write_sample(ldata, num_ldata, user_props);
	
	DM_ensure_tessface(output_dm);
	CustomData *fdata = output_dm->getTessFaceDataLayout(output_dm);
	int num_fdata = output_dm->getNumTessFaces(output_dm);
	m_face_data_writer.write_sample(fdata, num_fdata, user_props);
}

/* ========================================================================= */

AbcDerivedMeshReader::AbcDerivedMeshReader(const std::string &name, Object *ob) :
    DerivedMeshReader(ob, name),
    m_vert_data_reader("vertex_data", CD_MASK_CACHE_VERT),
    m_edge_data_reader("edge_data", CD_MASK_CACHE_EDGE),
    m_face_data_reader("face_data", CD_MASK_CACHE_FACE),
    m_poly_data_reader("poly_data", CD_MASK_CACHE_POLY),
    m_loop_data_reader("loop_data", CD_MASK_CACHE_LOOP)
{
}

AbcDerivedMeshReader::~AbcDerivedMeshReader()
{
}

void AbcDerivedMeshReader::init_abc(IObject object)
{
	if (m_mesh)
		return;
	m_mesh = IPolyMesh(object, kWrapExisting);
	
	IPolyMeshSchema &schema = m_mesh.getSchema();
	ICompoundProperty geom_props = schema.getArbGeomParams();
	ICompoundProperty user_props = schema.getUserProperties();
	
	m_prop_vert_normals = IN3fArrayProperty(user_props, "vertex_normals", 0);
	m_prop_vert_flag = ICharArrayProperty(user_props, "vertex_flag", 0);
	m_prop_vert_bweight = ICharArrayProperty(user_props, "vertex_bweight", 0);
	
	m_prop_edge_verts = IUInt32ArrayProperty(user_props, "edge_verts", 0);
	m_prop_edge_flag = IInt16ArrayProperty(user_props, "edge_flag", 0);
	m_prop_edge_crease = ICharArrayProperty(user_props, "edge_crease", 0);
	m_prop_edge_bweight = ICharArrayProperty(user_props, "edge_bweight", 0);
	
	m_prop_poly_mat_nr = IInt16ArrayProperty(user_props, "poly_mat_nr", 0);
	m_prop_poly_flag = ICharArrayProperty(user_props, "poly_flag", 0);
	
	m_prop_loop_verts = IInt32ArrayProperty(user_props, "loop_verts", 0);
	m_prop_loop_edges = IInt32ArrayProperty(user_props, "loop_edges", 0);
}

static PTCReadSampleResult apply_sample_verts(DerivedMesh *dm, P3fArraySamplePtr sample_co, N3fArraySamplePtr sample_no,
                                              CharArraySamplePtr sample_flag, CharArraySamplePtr sample_bweight)
{
	int totvert = dm->getNumVerts(dm);
	
	if (sample_co->size() != totvert ||
	    sample_no->size() != totvert ||
	    sample_flag->size() != totvert ||
	    sample_bweight->size() != totvert)
	{
		return PTC_READ_SAMPLE_INVALID;
	}
	
	MVert *mv = dm->getVertArray(dm);
	for (int i = 0; i < totvert; ++i) {
		copy_v3_v3(mv->co, (*sample_co)[i].getValue());
		normal_float_to_short_v3(mv->no, (*sample_no)[i].getValue());
		mv->flag = (*sample_flag)[i];
		mv->bweight = (*sample_bweight)[i];
		
		++mv;
	}
	
	return PTC_READ_SAMPLE_EXACT;
}

static PTCReadSampleResult apply_sample_edges(DerivedMesh *dm, UInt32ArraySamplePtr sample_verts, Int16ArraySamplePtr sample_flag,
                                              CharArraySamplePtr sample_crease, CharArraySamplePtr sample_bweight, bool &has_edges)
{
	int totedge = dm->getNumEdges(dm);
	if (sample_verts->size() != totedge * 2 ||
	    sample_flag->size() != totedge ||
	    sample_crease->size() != totedge ||
	    sample_bweight->size() != totedge) 
	{
		has_edges = false;
		return PTC_READ_SAMPLE_INVALID;
	}
	
	const uint32_t *data_verts = sample_verts->get();
	const int16_t *data_flag = sample_flag->get();
	const int8_t *data_crease = sample_crease->get();
	const int8_t *data_bweight = sample_bweight->get();
	
	MEdge *me = dm->getEdgeArray(dm);
	for (int i = 0; i < totedge; ++i) {
		me->v1 = data_verts[0];
		me->v2 = data_verts[1];
		me->flag = *data_flag;
		me->crease = *data_crease;
		me->bweight = *data_bweight;
		
		++me;
		data_verts += 2;
		++data_flag;
		++data_crease;
		++data_bweight;
	}
	
	return PTC_READ_SAMPLE_EXACT;
}

static PTCReadSampleResult apply_sample_polys(DerivedMesh *dm, Int32ArraySamplePtr sample_totloop, Int16ArraySamplePtr sample_mat_nr, CharArraySamplePtr sample_flag)
{
	int totpoly = dm->getNumPolys(dm);
	if (sample_totloop->size() != totpoly ||
	    sample_mat_nr->size() != totpoly ||
	    sample_flag->size() != totpoly)
	{
		return PTC_READ_SAMPLE_INVALID;
	}
	
	const int32_t *data_totloop = sample_totloop->get();
	const int16_t *data_mat_nr = sample_mat_nr->get();
	const int8_t *data_flag = sample_flag->get();
	
	int loopstart = 0;
	MPoly *mp = dm->getPolyArray(dm);
	for (int i = 0; i < totpoly; ++i) {
		mp->totloop = *data_totloop;
		mp->loopstart = loopstart;
		mp->mat_nr = *data_mat_nr;
		mp->flag = *data_flag;
		
		loopstart += mp->totloop;
		
		++mp;
		++data_totloop;
		++data_mat_nr;
		++data_flag;
	}
	
	return PTC_READ_SAMPLE_EXACT;
}

static PTCReadSampleResult apply_sample_loops(DerivedMesh *dm, Int32ArraySamplePtr sample_verts, Int32ArraySamplePtr sample_edges, bool &has_edges)
{
	int totloop = dm->getNumLoops(dm);
	if (sample_verts->size() != totloop)
		return PTC_READ_SAMPLE_INVALID;
	
	const int32_t *data_verts = sample_verts->get();
	
	MLoop *ml = dm->getLoopArray(dm);
	for (int i = 0; i < totloop; ++i) {
		ml->v = *data_verts;
		
		++ml;
		++data_verts;
	}
	
	/* edge data is optional, if not available the edges must be recalculated */
	if (sample_edges->size() == totloop) {
		const int32_t *data_edges = sample_edges->get();
		
		MLoop *ml = dm->getLoopArray(dm);
		for (int i = 0; i < totloop; ++i) {
			ml->e = *data_edges;
			
			++ml;
			++data_edges;
		}
	}
	else {
		has_edges = false;
	}
	
	return PTC_READ_SAMPLE_EXACT;
}

PTCReadSampleResult AbcDerivedMeshReader::read_sample_abc(chrono_t time)
{
#ifdef USE_TIMING
	double start_time;
	double time_get_sample, time_build_mesh, time_calc_edges, time_calc_normals;
	
#define PROFILE_START \
	start_time = PIL_check_seconds_timer();
#define PROFILE_END(var) \
	var = PIL_check_seconds_timer() - start_time;
#else
#define PROFILE_START ;
#define PROFILE_END(var) ;
#endif
	
	/* discard existing result data */
	discard_result();
	
	if (!m_mesh)
		return PTC_READ_SAMPLE_INVALID;
	
	IPolyMeshSchema &schema = m_mesh.getSchema();
	if (schema.getNumSamples() == 0)
		return PTC_READ_SAMPLE_INVALID;
	ICompoundProperty user_props = schema.getUserProperties();
	
	ISampleSelector ss = get_frame_sample_selector(time);
	
	PROFILE_START;
	IPolyMeshSchema::Sample sample;
	schema.get(sample, ss);
	
	P3fArraySamplePtr vert_co = abc_interpolate_sample_linear(schema.getPositionsProperty(), time);
	Int32ArraySamplePtr loop_verts = sample.getFaceIndices();
	Int32ArraySamplePtr poly_totloop = sample.getFaceCounts();
	
	N3fArraySamplePtr vnormals;
	bool has_normals = false;
	if (m_prop_vert_normals && m_prop_vert_normals.getNumSamples() > 0) {
		vnormals = abc_interpolate_sample_linear(m_prop_vert_normals, time, InterpolateSemanticVector_Slerp);
		has_normals = vnormals->valid();
	}
	
	UInt32ArraySamplePtr edge_verts = m_prop_edge_verts.getValue(ss);
	Int32ArraySamplePtr loop_edges = m_prop_loop_edges.getValue(ss);
	PROFILE_END(time_get_sample);
	
	PROFILE_START;
	bool has_edges = true; // XXX do we have to check for existing sample in advance?
	int totverts = vert_co->size();
	int totloops = loop_verts->size();
	int totpolys = poly_totloop->size();
	int totedges = has_edges ? edge_verts->size() >> 1 : 0;
	m_result = CDDM_new(totverts, totedges, 0, totloops, totpolys);
	
	apply_sample_verts(m_result, vert_co, vnormals, m_prop_vert_flag.getValue(ss), m_prop_vert_bweight.getValue(ss));
	apply_sample_edges(m_result, edge_verts, m_prop_edge_flag.getValue(ss), m_prop_edge_crease.getValue(ss), m_prop_edge_bweight.getValue(ss), has_edges);
	apply_sample_polys(m_result, poly_totloop, m_prop_poly_mat_nr.getValue(ss), m_prop_poly_flag.getValue(ss));
	apply_sample_loops(m_result, loop_verts, loop_edges, has_edges);
	PROFILE_END(time_build_mesh);
	
	CustomData *vdata = m_result->getVertDataLayout(m_result);
	int num_vdata = totverts;
	m_vert_data_reader.read_sample(ss, vdata, num_vdata, user_props);
	
	CustomData *edata = m_result->getEdgeDataLayout(m_result);
	int num_edata = totedges;
	m_edge_data_reader.read_sample(ss, edata, num_edata, user_props);
	
	CustomData *pdata = m_result->getPolyDataLayout(m_result);
	int num_pdata = totpolys;
	m_poly_data_reader.read_sample(ss, pdata, num_pdata, user_props);
	
	CustomData *ldata = m_result->getLoopDataLayout(m_result);
	int num_ldata = totloops;
	m_loop_data_reader.read_sample(ss, ldata, num_ldata, user_props);
	
	DM_ensure_tessface(m_result);
	CustomData *fdata = m_result->getTessFaceDataLayout(m_result);
	int num_fdata = m_result->getNumTessFaces(m_result);
	m_face_data_reader.read_sample(ss, fdata, num_fdata, user_props);
	
	PROFILE_START;
	if (!has_edges)
		CDDM_calc_edges(m_result);
	PROFILE_END(time_calc_edges);
	
	PROFILE_START;
	/* we need all normal properties defined, otherwise have to recalculate */
	has_normals &= CustomData_has_layer(pdata, CD_NORMAL);
	if (!has_normals) {
		/* make sure normals are recalculated if there is no sample data */
		m_result->dirty = (DMDirtyFlag)((int)m_result->dirty | DM_DIRTY_NORMALS);
	}
	DM_ensure_normals(m_result); /* only recalculates normals if no valid samples were found (has_normals == false) */
	PROFILE_END(time_calc_normals);
	
//	BLI_assert(DM_is_valid(m_result));
	
#ifdef USE_TIMING
	printf("-------- Point Cache Timing --------\n");
	printf("read sample: %f seconds\n", time_get_sample);
	printf("build mesh: %f seconds\n", time_build_mesh);
	printf("calculate edges: %f seconds\n", time_calc_edges);
	printf("calculate normals: %f seconds\n", time_calc_normals);
	printf("------------------------------------\n");
#endif
	
	return PTC_READ_SAMPLE_EXACT;
}

/* ========================================================================= */

AbcDerivedFinalRealtimeWriter::AbcDerivedFinalRealtimeWriter(const std::string &name, Object *ob) :
    AbcDerivedMeshWriter(name, ob, &ob->derivedFinal)
{
}


AbcDerivedFinalRenderWriter::AbcDerivedFinalRenderWriter(const std::string &name, Scene *scene, Object *ob, DerivedMesh **render_dm_ptr) :
    AbcDerivedMeshWriter(name, ob, render_dm_ptr),
    m_scene(scene)
{
}


} /* namespace PTC */
