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

#include "abc_cloth.h"
#include "abc_interpolate.h"
#include "abc_mesh.h"
#include "abc_particles.h"

extern "C" {
#include "BLI_listbase.h"
#include "BLI_math_color.h"
#include "BLI_math.h"

#include "DNA_listBase.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_particle_types.h"

#include "BKE_anim.h"
#include "BKE_mesh_sample.h"
#include "BKE_particle.h"
#include "BKE_strands.h"
}

namespace PTC {

using namespace Abc;
using namespace AbcGeom;


struct StrandsChildrenSample {
	std::vector<int32_t> numverts;
	std::vector<Quatf> root_rotations;
	std::vector<V3f> root_positions;
	std::vector<float32_t> cutoff;
	
	std::vector<V3f> positions;
	std::vector<float32_t> times;
	std::vector<int32_t> parents;
	std::vector<float32_t> parent_weights;
	std::vector<V2f> curve_uvs;
	std::vector<C3f> curve_vcols;
};

struct StrandsSample {
	std::vector<int32_t> numverts;
	std::vector<Quatf> root_rotations;
	std::vector<uint32_t> root_orig_verts;
	std::vector<float32_t> root_orig_weights;
	std::vector<int32_t> root_orig_poly;
	std::vector<uint32_t> root_orig_loops;
	
	std::vector<V3f> positions;
	std::vector<float32_t> times;
	std::vector<float32_t> weights;
	
	std::vector<V3f> motion_co;
	std::vector<V3f> motion_vel;
};

AbcHairChildrenWriter::AbcHairChildrenWriter(const std::string &name, Object *ob, ParticleSystem *psys) :
    ParticlesWriter(ob, psys, name)
{
	m_psmd = psys_get_modifier(ob, psys);
}

AbcHairChildrenWriter::~AbcHairChildrenWriter()
{
}

void AbcHairChildrenWriter::init_abc(OObject parent)
{
	if (m_curves)
		return;
	
	/* XXX non-escaped string construction here ... */
	m_curves = OCurves(parent, m_name, abc_archive()->frame_sampling_index());
	
	OCurvesSchema &schema = m_curves.getSchema();
	OCompoundProperty geom_props = schema.getArbGeomParams();
	OCompoundProperty user_props = schema.getUserProperties();
	
	m_prop_root_rot = OQuatfArrayProperty(user_props, "root_rotations", abc_archive()->frame_sampling());
	m_prop_root_positions = OV3fArrayProperty(user_props, "root_positions", abc_archive()->frame_sampling());
	m_param_cutoff = OFloatGeomParam(geom_props, "cutoff", false, kUniformScope, 1, 0);
	m_param_times = OFloatGeomParam(geom_props, "times", false, kVertexScope, 1, 0);
	m_prop_parents = OInt32ArrayProperty(user_props, "parents", abc_archive()->frame_sampling());
	m_prop_parent_weights = OFloatArrayProperty(user_props, "parent_weights", abc_archive()->frame_sampling());
	m_prop_curve_uvs = AbcGeom::OV2fArrayProperty(user_props, "curve_uvs", abc_archive()->frame_sampling());
	m_prop_curve_vcols = AbcGeom::OC3fArrayProperty(user_props, "curve_vcols", abc_archive()->frame_sampling());
}

static int hair_children_count_totkeys(ParticleCacheKey **pathcache, int totpart)
{
	int p;
	int totkeys = 0;
	
	if (pathcache) {
		for (p = 0; p < totpart; ++p) {
			ParticleCacheKey *keys = pathcache[p];
			totkeys += keys->segments + 1;
		}
	}
	
	return totkeys;
}

#if 0
static int hair_children_parent_advance(HairKey *keys, int totkeys, float time, int k)
{
	for (; k + 1 < totkeys; ++k) {
		if (keys[k+1].time > time)
			break;
	}
	return k;
}

static void hair_children_calc_strand(Object *ob, ParticleSystem *psys, ParticleSystemModifierData *psmd, ChildParticle *cpa, ParticleCacheKey *keys, int maxkeys, StrandsChildrenSample &sample)
{
	const bool between = (psys->part->childtype == PART_CHILD_FACES);
	ParticleData *parent[4];
	float weight[4];
	float hairmat[4][4][4];
	int parent_key[4] = {0,0,0,0};
	
	int i, k;
	
	if (between) {
		for (i = 0; i < 4; ++i) {
			parent[i] = &psys->particles[cpa->pa[i]];
			weight[i] = cpa->w[i];
			if (parent[i])
				psys_mat_hair_to_global(ob, psmd->dm, psys->part->from, parent[i], hairmat[i]);
		}
	}
	else {
		parent[0] = &psys->particles[cpa->parent];
		parent[1] = parent[2] = parent[3] = NULL;
		weight[0] = 1.0f;
		weight[1] = weight[2] = weight[3] = 0.0f;
		if (parent[0])
			psys_mat_hair_to_global(ob, psmd->dm, psys->part->from, parent[0], hairmat[0]);
	}
	
	int numkeys = keys->segments + 1;
	for (k = 0; k < numkeys; ++k) {
		ParticleCacheKey *key = &keys[k];
		/* XXX particle time values are too messy and confusing, recalculate */
		float time = maxkeys > 1 ? (float)k / (float)(maxkeys-1) : 0.0f;
		
		float parent_co[3];
		zero_v3(parent_co);
		for (i = 0; i < 4; ++i) {
			if (!parent[i] || weight[i] <= 0.0f)
				continue;
			parent_key[i] = hair_children_parent_advance(parent[i]->hair, parent[i]->totkey, time, parent_key[i]);
			
			float key_co[3];
			if (parent_key[i] + 1 < parent[i]->totkey) {
				HairKey *key0 = &parent[i]->hair[parent_key[i]];
				HairKey *key1 = &parent[i]->hair[parent_key[i] + 1];
				float x = (key1->time > key0->time) ? (time - key0->time) / (key1->time - key0->time) : 0.0f;
				interp_v3_v3v3(key_co, key0->co, key1->co, x);
			}
			else {
				HairKey *key0 = &parent[i]->hair[parent_key[i]];
				copy_v3_v3(key_co, key0->co);
			}
			
			madd_v3_v3fl(parent_co, key_co, weight[i]);
			
			/* Hair keys are in hair root space, pathcache keys are in world space,
			 * transform both to world space to calculate the offset
			 */
			mul_m4_v3(hairmat[i], parent_co);
		}
		
		/* child position is an offset from the parent */
		float co[3];
		sub_v3_v3v3(co, key->co, parent_co);
		
		sample.positions.push_back(V3f(parent_co[0], parent_co[1], parent_co[2]));
		sample.times.push_back(time);
	}
}
#endif

BLI_INLINE bool particle_get_face(ParticleSystem *psys, int num_tessface, ChildParticle *cpa, int *r_num, float **r_fuv)
{
	ParticleSettings *part = psys->part;
	const bool between = (part->childtype == PART_CHILD_FACES);
	
	int num = DMCACHE_NOTFOUND;
	float *fuv;
	if (between) {
		num = cpa->num;
		fuv = cpa->fuv;
	}
	else if (part->from == PART_FROM_FACE) {
		ParticleData *pa = psys->particles + cpa->pa[0];
		num = pa->num_dmcache;
		if (num == DMCACHE_NOTFOUND)
			num = pa->num;
		if (num >= num_tessface) {
			/* happens when simplify is enabled gives invalid coords but would crash otherwise */
			num = DMCACHE_NOTFOUND;
		}
		fuv = pa->fuv;
	}
	
	if (ELEM(num, DMCACHE_NOTFOUND, DMCACHE_ISCHILD)) {
		return false;
	}
	else {
		*r_num = num;
		*r_fuv = fuv;
		return true;
	}
}

static void hair_children_get_uvs(ParticleSystem *psys, ParticleSystemModifierData *psmd, int totpart, StrandsChildrenSample &sample)
{
	const int num_tessface = psmd->dm->getNumTessFaces(psmd->dm);
	
	MFace *mface = (MFace *)psmd->dm->getTessFaceArray(psmd->dm);
	const CustomData *facedata = psmd->dm->getTessFaceDataLayout(psmd->dm);
	int tot_layers = CustomData_number_of_layers(facedata, CD_MTFACE);
	
	sample.curve_uvs.reserve(tot_layers * totpart);
	
	for (int num_uv = 0; num_uv < tot_layers; ++num_uv) {
		MTFace *mtface = (MTFace *)CustomData_get_layer_n(facedata, CD_MTFACE, num_uv);
		if (!mtface)
			continue;
		
		for (int p = 0; p < totpart; ++p) {
			ChildParticle *cpa = &psys->child[p];
			
			int num;
			float *fuv;
			if (particle_get_face(psys, num_tessface, cpa, &num, &fuv)) {
				MFace *mf = mface + num;
				MTFace *mtf = mtface + num;
				
				float uv[2];
				psys_interpolate_uvs(mtf, mf->v4, fuv, uv);
				sample.curve_uvs.push_back(V2f(uv[0], uv[1]));
			}
			else {
				sample.curve_uvs.push_back(V2f(0.0f, 0.0f));
			}
		}
	}
}

static void hair_children_get_vcols(ParticleSystem *psys, ParticleSystemModifierData *psmd, int totpart, StrandsChildrenSample &sample)
{
	const int num_tessface = psmd->dm->getNumTessFaces(psmd->dm);
	
	MFace *mface = (MFace *)psmd->dm->getTessFaceArray(psmd->dm);
	const CustomData *facedata = psmd->dm->getTessFaceDataLayout(psmd->dm);
	int tot_layers = CustomData_number_of_layers(facedata, CD_MCOL);
	
	sample.curve_vcols.reserve(tot_layers * totpart);
	
	for (int num_vcol = 0; num_vcol < tot_layers; ++num_vcol) {
		MCol *mcol = (MCol *)CustomData_get_layer_n(facedata, CD_MCOL, num_vcol);
		if (!mcol)
			continue;
		
		for (int p = 0; p < totpart; ++p) {
			ChildParticle *cpa = &psys->child[p];
			
			int num;
			float *fuv;
			if (particle_get_face(psys, num_tessface, cpa, &num, &fuv)) {
				MFace *mf = mface + num;
				/* XXX another legacy thing: MCol are tessface data, but 4 values per face ... */
				MCol *mc = mcol + 4 * num;
				
				MCol col;
				psys_interpolate_mcol(mc, mf->v4, fuv, &col);
				/* XXX stupid legacy code: MCol stores values are BGR */
				unsigned char icol[3] = {col.b, col.g, col.r};
				C3f fcol;
				rgb_uchar_to_float(fcol.getValue(), icol);
				sample.curve_vcols.push_back(fcol);
			}
			else {
				sample.curve_vcols.push_back(C3f(0.0f, 0.0f, 0.0f));
			}
		}
	}
}

static void hair_children_create_sample(Object *ob, ParticleSystem *psys, ParticleSystemModifierData *psmd, ParticleCacheKey **pathcache, int totpart, int totkeys, int maxkeys,
                                        StrandsChildrenSample &sample, bool write_constants)
{
	ParticleSettings *part = psys->part;
	const bool between = (part->childtype == PART_CHILD_FACES);
	
	int p, k;
	
	if (write_constants) {
		sample.numverts.reserve(totpart);
		sample.parents.reserve(4*totpart);
		sample.parent_weights.reserve(4*totpart);
		
		sample.positions.reserve(totkeys);
		sample.times.reserve(totkeys);
	}
	
	sample.root_rotations.reserve(totpart);
	sample.root_positions.reserve(totpart);
	
	for (p = 0; p < totpart; ++p) {
		ChildParticle *cpa = &psys->child[p];
		
		float hairmat[4][4];
		psys_child_mat_to_object(ob, psys, psmd, cpa, hairmat);
		
		if (pathcache) {
			ParticleCacheKey *keys = pathcache[p];
			int numkeys = keys->segments + 1;
			
			if (write_constants) {
				sample.numverts.push_back(numkeys);
				if (between) {
					sample.parents.push_back(cpa->pa[0]);
					sample.parents.push_back(cpa->pa[1]);
					sample.parents.push_back(cpa->pa[2]);
					sample.parents.push_back(cpa->pa[3]);
					sample.parent_weights.push_back(cpa->w[0]);
					sample.parent_weights.push_back(cpa->w[1]);
					sample.parent_weights.push_back(cpa->w[2]);
					sample.parent_weights.push_back(cpa->w[3]);
				}
				else {
					sample.parents.push_back(cpa->parent);
					sample.parents.push_back(-1);
					sample.parents.push_back(-1);
					sample.parents.push_back(-1);
					sample.parent_weights.push_back(1.0f);
					sample.parent_weights.push_back(0.0f);
					sample.parent_weights.push_back(0.0f);
					sample.parent_weights.push_back(0.0f);
				}
				
				float imat[4][4];
				mul_m4_m4m4(imat, ob->obmat, hairmat);
				invert_m4(imat);
				
				for (k = 0; k < numkeys; ++k) {
					ParticleCacheKey *key = &keys[k];
					
					/* pathcache keys are in world space, transform to hair root space */
					float co[3];
					mul_v3_m4v3(co, imat, key->co);
					
					sample.positions.push_back(V3f(co[0], co[1], co[2]));
					/* XXX particle time values are too messy and confusing, recalculate */
					sample.times.push_back(maxkeys > 1 ? (float)k / (float)(maxkeys-1) : 0.0f);
				}
			}
		}
		
		float qt[4];
		mat4_to_quat(qt, hairmat);
		sample.root_rotations.push_back(Quatf(qt[0], qt[1], qt[2], qt[3]));
		float *co = hairmat[3];
		sample.root_positions.push_back(V3f(co[0], co[1], co[2]));
		sample.cutoff.push_back(-1.0f);
	}
	
	if (write_constants) {
		DM_ensure_tessface(psmd->dm);
		hair_children_get_uvs(psys, psmd, totpart, sample);
		hair_children_get_vcols(psys, psmd, totpart, sample);
	}
}

void AbcHairChildrenWriter::write_sample()
{
	if (!m_curves)
		return;
	
	int totkeys = hair_children_count_totkeys(m_psys->childcache, m_psys->totchild);
	
	int keysteps = abc_archive()->use_render() ? m_psys->part->ren_step : m_psys->part->draw_step;
	int maxkeys = (1 << keysteps) + 1 + (m_psys->part->kink);
	if (ELEM(m_psys->part->kink, PART_KINK_SPIRAL))
		maxkeys += m_psys->part->kink_extra_steps;
	
	OCurvesSchema &schema = m_curves.getSchema();
	
	StrandsChildrenSample child_sample;
	OCurvesSchema::Sample sample;
	if (schema.getNumSamples() == 0) {
		/* write curve sizes only first time, assuming they are constant! */
		hair_children_create_sample(m_ob, m_psys, m_psmd, m_psys->childcache, m_psys->totchild, totkeys, maxkeys, child_sample, true);
		sample = OCurvesSchema::Sample(child_sample.positions, child_sample.numverts);
		
		m_prop_parents.set(Int32ArraySample(child_sample.parents));
		m_prop_parent_weights.set(FloatArraySample(child_sample.parent_weights));
		m_prop_curve_uvs.set(V2fArraySample(child_sample.curve_uvs));
		m_prop_curve_vcols.set(C3fArraySample(child_sample.curve_vcols));
		
		m_param_times.set(OFloatGeomParam::Sample(FloatArraySample(child_sample.times), kVertexScope));
		
		schema.set(sample);
	}
	else {
		hair_children_create_sample(m_ob, m_psys, m_psmd, m_psys->childcache, m_psys->totchild, totkeys, maxkeys, child_sample, false);
	}
	
	m_prop_root_rot.set(QuatfArraySample(child_sample.root_rotations));
	m_prop_root_positions.set(V3fArraySample(child_sample.root_positions));
	m_param_cutoff.set(OFloatGeomParam::Sample(FloatArraySample(child_sample.cutoff), kUniformScope));
}


AbcHairWriter::AbcHairWriter(const std::string &name, Object *ob, ParticleSystem *psys) :
    ParticlesWriter(ob, psys, name),
    m_child_writer("children", ob, psys)
{
	m_psmd = psys_get_modifier(ob, psys);
}

AbcHairWriter::~AbcHairWriter()
{
}

void AbcHairWriter::init(WriterArchive *archive)
{
	AbcWriter::init(archive);
	m_child_writer.init(archive);
}

void AbcHairWriter::init_abc(OObject parent)
{
	if (m_curves)
		return;
	m_curves = OCurves(parent, m_name, abc_archive()->frame_sampling_index());
	
	OCurvesSchema &schema = m_curves.getSchema();
	OCompoundProperty geom_props = schema.getArbGeomParams();
	
	m_param_root_rot = OQuatfGeomParam(geom_props, "root_rotations", false, kUniformScope, 1, 0);
	m_param_root_orig_verts = OUInt32GeomParam(geom_props, "root_orig_verts", false, kUniformScope, 1, 0);
	m_param_root_orig_weights = OFloatGeomParam(geom_props, "root_orig_weights", false, kUniformScope, 1, 0);
	m_param_root_orig_poly = OInt32GeomParam(geom_props, "root_orig_poly", false, kUniformScope, 1, 0);
	m_param_root_orig_loops = OUInt32GeomParam(geom_props, "root_orig_loops", false, kUniformScope, 1, 0);
	
	m_param_times = OFloatGeomParam(geom_props, "times", false, kVertexScope, 1, 0);
	m_param_weights = OFloatGeomParam(geom_props, "weights", false, kVertexScope, 1, 0);
	
	m_child_writer.init_abc(m_curves);
}

static int hair_count_totverts(ParticleSystem *psys)
{
	int p;
	int totverts = 0;
	
	for (p = 0; p < psys->totpart; ++p) {
		ParticleData *pa = &psys->particles[p];
		totverts += pa->totkey;
	}
	
	return totverts;
}

static void hair_create_sample(Object *ob, DerivedMesh *dm, ParticleSystem *psys, StrandsSample &sample, bool do_numverts)
{
	int totpart = psys->totpart;
	int totverts = hair_count_totverts(psys);
	int p, k;
	
	if (totverts == 0)
		return;
	
	if (do_numverts)
		sample.numverts.reserve(totpart);
	sample.root_rotations.reserve(totpart);
	sample.root_orig_verts.reserve(totpart * 3);
	sample.root_orig_weights.reserve(totpart * 3);
	sample.root_orig_poly.reserve(totpart);
	sample.root_orig_loops.reserve(totpart * 3);
	sample.positions.reserve(totverts);
	sample.times.reserve(totverts);
	sample.weights.reserve(totverts);
	
	for (p = 0; p < totpart; ++p) {
		ParticleData *pa = &psys->particles[p];
		int numverts = pa->totkey;
		float hairmat[4][4];
		
		if (do_numverts)
			sample.numverts.push_back(numverts);
		
		psys_mat_hair_to_object(ob, dm, psys->part->from, pa, hairmat);
		float root_qt[4];
		mat4_to_quat(root_qt, hairmat);
		sample.root_rotations.push_back(Quatf(root_qt[0], root_qt[1], root_qt[2], root_qt[3]));
		
		MSurfaceSample surf;
		BKE_mesh_sample_from_particle(&surf, psys, dm, pa);
		sample.root_orig_verts.push_back(surf.orig_verts[0]);
		sample.root_orig_verts.push_back(surf.orig_verts[1]);
		sample.root_orig_verts.push_back(surf.orig_verts[2]);
		sample.root_orig_weights.push_back(surf.orig_weights[0]);
		sample.root_orig_weights.push_back(surf.orig_weights[1]);
		sample.root_orig_weights.push_back(surf.orig_weights[2]);
		sample.root_orig_poly.push_back(surf.orig_poly);
		sample.root_orig_loops.push_back(surf.orig_loops[0]);
		sample.root_orig_loops.push_back(surf.orig_loops[1]);
		sample.root_orig_loops.push_back(surf.orig_loops[2]);
		
		for (k = 0; k < numverts; ++k) {
			HairKey *key = &pa->hair[k];
			
			/* hair keys are in "hair space" relative to the mesh,
			 * store them in object space for compatibility and to avoid
			 * complexities of how particles work.
			 */
			float co[3];
			mul_v3_m4v3(co, hairmat, key->co);
			
			sample.positions.push_back(V3f(co[0], co[1], co[2]));
			/* XXX particle time values are too messy and confusing, recalculate */
			sample.times.push_back(numverts > 1 ? (float)k / (float)(numverts-1) : 0.0f);
			sample.weights.push_back(key->weight);
		}
	}
}

void AbcHairWriter::write_sample()
{
	if (!m_curves)
		return;
	if (!m_psmd || !m_psmd->dm)
		return;
	
	OCurvesSchema &schema = m_curves.getSchema();
	
	StrandsSample hair_sample;
	OCurvesSchema::Sample sample;
	if (schema.getNumSamples() == 0) {
		/* write curve sizes only first time, assuming they are constant! */
		hair_create_sample(m_ob, m_psmd->dm, m_psys, hair_sample, true);
		sample = OCurvesSchema::Sample(hair_sample.positions, hair_sample.numverts);
	}
	else {
		hair_create_sample(m_ob, m_psmd->dm, m_psys, hair_sample, false);
		sample = OCurvesSchema::Sample(hair_sample.positions);
	}
	schema.set(sample);
	
	m_param_root_rot.set(OQuatfGeomParam::Sample(QuatfArraySample(hair_sample.root_rotations), kUniformScope));
	m_param_root_orig_verts.set(OUInt32GeomParam::Sample(UInt32ArraySample(hair_sample.root_orig_verts), kUniformScope));
	m_param_root_orig_weights.set(OFloatGeomParam::Sample(FloatArraySample(hair_sample.root_orig_weights), kUniformScope));
	m_param_root_orig_poly.set(OInt32GeomParam::Sample(Int32ArraySample(hair_sample.root_orig_poly), kUniformScope));
	m_param_root_orig_loops.set(OUInt32GeomParam::Sample(UInt32ArraySample(hair_sample.root_orig_loops), kUniformScope));
	
	m_param_times.set(OFloatGeomParam::Sample(FloatArraySample(hair_sample.times), kVertexScope));
	m_param_weights.set(OFloatGeomParam::Sample(FloatArraySample(hair_sample.weights), kVertexScope));
	
	m_child_writer.write_sample();
}


AbcStrandsChildrenWriter::AbcStrandsChildrenWriter(const std::string &name, const std::string &abc_name, DupliObjectData *dobdata) :
    m_name(name),
    m_abc_name(abc_name),
    m_dobdata(dobdata)
{
}

StrandsChildren *AbcStrandsChildrenWriter::get_strands() const
{
	StrandsChildren *children;
	BKE_dupli_object_data_find_strands(m_dobdata, m_name.c_str(), NULL, &children);
	return children;
}

void AbcStrandsChildrenWriter::init_abc(OObject parent)
{
	if (m_curves)
		return;
	m_curves = OCurves(parent, m_abc_name, abc_archive()->frame_sampling_index());
	
	OCurvesSchema &schema = m_curves.getSchema();
	OCompoundProperty geom_props = schema.getArbGeomParams();
	OCompoundProperty user_props = schema.getUserProperties();
	
	m_prop_root_rot = OQuatfArrayProperty(user_props, "root_rotations", abc_archive()->frame_sampling());
	m_prop_root_positions = OV3fArrayProperty(user_props, "root_positions", abc_archive()->frame_sampling());
	m_param_cutoff = OFloatGeomParam(geom_props, "cutoff", false, kUniformScope, 1, abc_archive()->frame_sampling());
	m_param_times = OFloatGeomParam(geom_props, "times", false, kVertexScope, 1, abc_archive()->frame_sampling());
	m_prop_parents = OInt32ArrayProperty(user_props, "parents", abc_archive()->frame_sampling());
	m_prop_parent_weights = OFloatArrayProperty(user_props, "parent_weights", abc_archive()->frame_sampling());
	m_prop_curve_uvs = AbcGeom::OV2fArrayProperty(user_props, "curve_uvs", abc_archive()->frame_sampling());
	m_prop_curve_vcols = AbcGeom::OC3fArrayProperty(user_props, "curve_vcols", abc_archive()->frame_sampling());
}

static void strands_children_get_uvs(StrandsChildren *strands, StrandsChildrenSample &sample)
{
	int totuv = strands->numuv * strands->totcurves;
	
	sample.curve_uvs.reserve(totuv);
	
	for (int i = 0; i < totuv; ++i) {
		StrandsChildCurveUV *uv = &strands->curve_uvs[i];
		sample.curve_uvs.push_back(V2f(uv->uv[0], uv->uv[1]));
	}
}

static void strands_children_get_vcols(StrandsChildren *strands, StrandsChildrenSample &sample)
{
	int totvcol = strands->numvcol * strands->totcurves;
	
	sample.curve_vcols.reserve(totvcol);
	
	for (int i = 0; i < totvcol; ++i) {
		StrandsChildCurveVCol *vcol = &strands->curve_vcols[i];
		sample.curve_vcols.push_back(C3f(vcol->vcol[0], vcol->vcol[1], vcol->vcol[2]));
	}
}

static void strands_children_create_sample(StrandsChildren *strands, StrandsChildrenSample &sample, bool write_constants)
{
	int totcurves = strands->totcurves;
	int totverts = strands->totverts;
	
	if (write_constants) {
		sample.numverts.reserve(totcurves);
		sample.parents.reserve(4*totcurves);
		sample.parent_weights.reserve(4*totcurves);
		
		sample.positions.reserve(totverts);
		sample.times.reserve(totverts);
	}
	
	sample.root_rotations.reserve(totcurves);
	sample.root_positions.reserve(totcurves);
	
	StrandChildIterator it_strand;
	for (BKE_strand_child_iter_init(&it_strand, strands); BKE_strand_child_iter_valid(&it_strand); BKE_strand_child_iter_next(&it_strand)) {
		int numverts = it_strand.curve->numverts;
		
		if (write_constants) {
			sample.numverts.push_back(numverts);
			
			sample.parents.push_back(it_strand.curve->parents[0]);
			sample.parents.push_back(it_strand.curve->parents[1]);
			sample.parents.push_back(it_strand.curve->parents[2]);
			sample.parents.push_back(it_strand.curve->parents[3]);
			sample.parent_weights.push_back(it_strand.curve->parent_weights[0]);
			sample.parent_weights.push_back(it_strand.curve->parent_weights[1]);
			sample.parent_weights.push_back(it_strand.curve->parent_weights[2]);
			sample.parent_weights.push_back(it_strand.curve->parent_weights[3]);
			
			StrandChildVertexIterator it_vert;
			for (BKE_strand_child_vertex_iter_init(&it_vert, &it_strand); BKE_strand_child_vertex_iter_valid(&it_vert); BKE_strand_child_vertex_iter_next(&it_vert)) {
				const float *co = it_vert.vertex->base;
				sample.positions.push_back(V3f(co[0], co[1], co[2]));
				sample.times.push_back(it_vert.vertex->time);
			}
		}
		
		float qt[4];
		mat4_to_quat(qt, it_strand.curve->root_matrix);
		sample.root_rotations.push_back(Quatf(qt[0], qt[1], qt[2], qt[3]));
		float *co = it_strand.curve->root_matrix[3];
		sample.root_positions.push_back(V3f(co[0], co[1], co[2]));
		sample.cutoff.push_back(it_strand.curve->cutoff);
	}
	
	if (write_constants) {
		strands_children_get_uvs(strands, sample);
		strands_children_get_vcols(strands, sample);
	}
}

void AbcStrandsChildrenWriter::write_sample()
{
	if (!m_curves)
		return;
	StrandsChildren *strands = get_strands();
	if (!strands)
		return;
	
	OCurvesSchema &schema = m_curves.getSchema();
	
	StrandsChildrenSample strands_sample;
	OCurvesSchema::Sample sample;
	if (schema.getNumSamples() == 0) {
		/* write curve sizes only first time, assuming they are constant! */
		strands_children_create_sample(strands, strands_sample, true);
		sample = OCurvesSchema::Sample(strands_sample.positions, strands_sample.numverts);
		
		m_prop_parents.set(Int32ArraySample(strands_sample.parents));
		m_prop_parent_weights.set(FloatArraySample(strands_sample.parent_weights));
		m_prop_curve_uvs.set(V2fArraySample(strands_sample.curve_uvs));
		m_prop_curve_vcols.set(C3fArraySample(strands_sample.curve_vcols));
		
		m_param_times.set(OFloatGeomParam::Sample(FloatArraySample(strands_sample.times), kVertexScope));
		
		schema.set(sample);
	}
	else {
		strands_children_create_sample(strands, strands_sample, false);
	}
	
	m_prop_root_rot.set(QuatfArraySample(strands_sample.root_rotations));
	m_prop_root_positions.set(V3fArraySample(strands_sample.root_positions));
	m_param_cutoff.set(OFloatGeomParam::Sample(FloatArraySample(strands_sample.cutoff), kUniformScope));
}


AbcStrandsWriter::AbcStrandsWriter(const std::string &name, DupliObjectData *dobdata) :
    m_name(name),
    m_dobdata(dobdata),
    m_child_writer(name, "children", dobdata)
{
}

Strands *AbcStrandsWriter::get_strands() const
{
	Strands *strands;
	BKE_dupli_object_data_find_strands(m_dobdata, m_name.c_str(), &strands, NULL);
	return strands;
}

void AbcStrandsWriter::init(WriterArchive *archive)
{
	AbcWriter::init(archive);
	m_child_writer.init(archive);
}

void AbcStrandsWriter::init_abc(OObject parent)
{
	if (m_curves)
		return;
	m_curves = OCurves(parent, m_name, abc_archive()->frame_sampling_index());
	
	OCurvesSchema &schema = m_curves.getSchema();
	OCompoundProperty geom_props = schema.getArbGeomParams();
	
	m_param_root_rot = OQuatfGeomParam(geom_props, "root_rotations", false, kUniformScope, 1, abc_archive()->frame_sampling());
	m_param_root_orig_verts = OUInt32GeomParam(geom_props, "root_orig_verts", false, kUniformScope, 1, 0);
	m_param_root_orig_weights = OFloatGeomParam(geom_props, "root_orig_weights", false, kUniformScope, 1, 0);
	m_param_root_orig_poly = OInt32GeomParam(geom_props, "root_orig_poly", false, kUniformScope, 1, 0);
	m_param_root_orig_loops = OUInt32GeomParam(geom_props, "root_orig_loops", false, kUniformScope, 1, 0);
	
	m_param_times = OFloatGeomParam(geom_props, "times", false, kVertexScope, 1, abc_archive()->frame_sampling());
	m_param_weights = OFloatGeomParam(geom_props, "weights", false, kVertexScope, 1, abc_archive()->frame_sampling());
	
	m_param_motion_state = OCompoundProperty(geom_props, "motion_state", abc_archive()->frame_sampling());
	m_param_motion_co = OP3fGeomParam(m_param_motion_state, "position", false, kVertexScope, 1, abc_archive()->frame_sampling());
	m_param_motion_vel = OV3fGeomParam(m_param_motion_state, "velocity", false, kVertexScope, 1, abc_archive()->frame_sampling());
	
	m_child_writer.init_abc(m_curves);
}

static void strands_create_sample(Strands *strands, StrandsSample &sample, bool do_numverts)
{
	const bool do_state = strands->state;
	
	int totcurves = strands->totcurves;
	int totverts = strands->totverts;
	
	if (totverts == 0)
		return;
	
	if (do_numverts)
		sample.numverts.reserve(totcurves);
	sample.root_rotations.reserve(totcurves);
	sample.root_orig_verts.reserve(totcurves * 3);
	sample.root_orig_weights.reserve(totcurves * 3);
	sample.root_orig_poly.reserve(totcurves);
	sample.root_orig_loops.reserve(totcurves * 3);
	
	sample.positions.reserve(totverts);
	sample.times.reserve(totverts);
	sample.weights.reserve(totverts);
	if (do_state) {
		sample.motion_co.reserve(totverts);
		sample.motion_vel.reserve(totverts);
	}
	
	StrandIterator it_strand;
	for (BKE_strand_iter_init(&it_strand, strands); BKE_strand_iter_valid(&it_strand); BKE_strand_iter_next(&it_strand)) {
		int numverts = it_strand.curve->numverts;
		
		if (do_numverts)
			sample.numverts.push_back(numverts);
		float qt[4];
		mat3_to_quat(qt, it_strand.curve->root_matrix);
		sample.root_rotations.push_back(Quatf(qt[0], qt[1], qt[2], qt[3]));
		
		sample.root_orig_verts.push_back(it_strand.curve->msurf.orig_verts[0]);
		sample.root_orig_verts.push_back(it_strand.curve->msurf.orig_verts[1]);
		sample.root_orig_verts.push_back(it_strand.curve->msurf.orig_verts[2]);
		sample.root_orig_weights.push_back(it_strand.curve->msurf.orig_weights[0]);
		sample.root_orig_weights.push_back(it_strand.curve->msurf.orig_weights[1]);
		sample.root_orig_weights.push_back(it_strand.curve->msurf.orig_weights[2]);
		sample.root_orig_poly.push_back(it_strand.curve->msurf.orig_poly);
		sample.root_orig_loops.push_back(it_strand.curve->msurf.orig_loops[0]);
		sample.root_orig_loops.push_back(it_strand.curve->msurf.orig_loops[1]);
		sample.root_orig_loops.push_back(it_strand.curve->msurf.orig_loops[2]);
		
		StrandVertexIterator it_vert;
		for (BKE_strand_vertex_iter_init(&it_vert, &it_strand); BKE_strand_vertex_iter_valid(&it_vert); BKE_strand_vertex_iter_next(&it_vert)) {
			const float *co = it_vert.vertex->co;
			sample.positions.push_back(V3f(co[0], co[1], co[2]));
			sample.times.push_back(it_vert.vertex->time);
			sample.weights.push_back(it_vert.vertex->weight);
			
			if (do_state) {
				float *co = it_vert.state->co;
				float *vel = it_vert.state->vel;
				sample.motion_co.push_back(V3f(co[0], co[1], co[2]));
				sample.motion_vel.push_back(V3f(vel[0], vel[1], vel[2]));
			}
		}
	}
}

void AbcStrandsWriter::write_sample()
{
	if (!m_curves)
		return;
	Strands *strands = get_strands();
	if (!strands)
		return;
	
	OCurvesSchema &schema = m_curves.getSchema();
	
	StrandsSample strands_sample;
	OCurvesSchema::Sample sample;
	if (schema.getNumSamples() == 0) {
		/* write curve sizes only first time, assuming they are constant! */
		strands_create_sample(strands, strands_sample, true);
		sample = OCurvesSchema::Sample(strands_sample.positions, strands_sample.numverts);
	}
	else {
		strands_create_sample(strands, strands_sample, false);
		sample = OCurvesSchema::Sample(strands_sample.positions);
	}
	schema.set(sample);
	
	m_param_root_rot.set(OQuatfGeomParam::Sample(QuatfArraySample(strands_sample.root_rotations), kUniformScope));
	m_param_root_orig_verts.set(OUInt32GeomParam::Sample(UInt32ArraySample(strands_sample.root_orig_verts), kUniformScope));
	m_param_root_orig_weights.set(OFloatGeomParam::Sample(FloatArraySample(strands_sample.root_orig_weights), kUniformScope));
	m_param_root_orig_poly.set(OInt32GeomParam::Sample(Int32ArraySample(strands_sample.root_orig_poly), kUniformScope));
	m_param_root_orig_loops.set(OUInt32GeomParam::Sample(UInt32ArraySample(strands_sample.root_orig_loops), kUniformScope));
	
	m_param_times.set(OFloatGeomParam::Sample(FloatArraySample(strands_sample.times), kVertexScope));
	m_param_weights.set(OFloatGeomParam::Sample(FloatArraySample(strands_sample.weights), kVertexScope));
	
	if (strands->state) {
		m_param_motion_co.set(OP3fGeomParam::Sample(P3fArraySample(strands_sample.motion_co), kVertexScope));
		m_param_motion_vel.set(OV3fGeomParam::Sample(V3fArraySample(strands_sample.motion_vel), kVertexScope));
	}
	
	m_child_writer.write_sample();
}

#if 0
#define PRINT_M3_FORMAT "((%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f))"
#define PRINT_M3_ARGS(m) (double)m[0][0], (double)m[0][1], (double)m[0][2], (double)m[1][0], (double)m[1][1], (double)m[1][2], (double)m[2][0], (double)m[2][1], (double)m[2][2]
#define PRINT_M4_FORMAT "((%.3f, %.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f))"
#define PRINT_M4_ARGS(m) (double)m[0][0], (double)m[0][1], (double)m[0][2], (double)m[0][3], (double)m[1][0], (double)m[1][1], (double)m[1][2], (double)m[1][3], \
                         (double)m[2][0], (double)m[2][1], (double)m[2][2], (double)m[2][3], (double)m[3][0], (double)m[3][1], (double)m[3][2], (double)m[3][3]
#endif

AbcStrandsChildrenReader::AbcStrandsChildrenReader(StrandsChildren *strands) :
    m_strands(strands)
{
}

AbcStrandsChildrenReader::~AbcStrandsChildrenReader()
{
	discard_result();
}

void AbcStrandsChildrenReader::init_abc(IObject object)
{
	if (m_curves)
		return;
	m_curves = ICurves(object, kWrapExisting);
	
	ICurvesSchema &schema = m_curves.getSchema();
	ICompoundProperty geom_props = schema.getArbGeomParams();
	ICompoundProperty user_props = schema.getUserProperties();
	
	m_prop_root_rot = IQuatfArrayProperty(user_props, "root_rotations");
	m_prop_root_positions = IV3fArrayProperty(user_props, "root_positions");
	if (geom_props.getPropertyHeader("cutoff"))
		m_param_cutoff = IFloatGeomParam(geom_props, "cutoff");
	m_param_times = IFloatGeomParam(geom_props, "times");
	m_prop_parents = IInt32ArrayProperty(user_props, "parents", 0);
	m_prop_parent_weights = IFloatArrayProperty(user_props, "parent_weights", 0);
	m_prop_curve_uvs = IV2fArrayProperty(user_props, "curve_uvs", 0);
	m_prop_curve_vcols = IC3fArrayProperty(user_props, "curve_vcols", 0);
}

PTCReadSampleResult AbcStrandsChildrenReader::read_sample_abc(chrono_t time)
{
	ISampleSelector ss = get_frame_sample_selector(time);
	
	if (!m_curves.valid()) {
		return PTC_READ_SAMPLE_INVALID;
	}
	
	ICurvesSchema &schema = m_curves.getSchema();
	if (schema.getNumSamples() == 0) {
		return PTC_READ_SAMPLE_INVALID;
	}
	
	ICurvesSchema::Sample sample;
	schema.get(sample, ss);
	
	P3fArraySamplePtr sample_co = sample.getPositions();
	Int32ArraySamplePtr sample_numvert = sample.getCurvesNumVertices();
	QuatfArraySamplePtr sample_root_rotations = abc_interpolate_sample_linear(m_prop_root_rot, time);
	V3fArraySamplePtr sample_root_positions = abc_interpolate_sample_linear(m_prop_root_positions, time);
	IFloatGeomParam::Sample sample_cutoff;
	if (m_param_cutoff)
		sample_cutoff = m_param_cutoff.getExpandedValue(ss);
	IFloatGeomParam::Sample sample_time = m_param_times.getExpandedValue(ss);
	Int32ArraySamplePtr sample_parents = m_prop_parents.getValue(ss);
	FloatArraySamplePtr sample_parent_weights = m_prop_parent_weights.getValue(ss);
	V2fArraySamplePtr sample_curve_uvs = m_prop_curve_uvs.getValue(ss);
	C3fArraySamplePtr sample_curve_vcols = m_prop_curve_vcols.getValue(ss);
	
	if (!sample_co || !sample_numvert) {
		return PTC_READ_SAMPLE_INVALID;
	}
	
	int totcurves = sample_numvert->size();
	int totverts = sample_co->size();

	if (!totcurves) {
		return PTC_READ_SAMPLE_INVALID;
	}

	if (sample_root_rotations->size() != totcurves ||
	    sample_root_positions->size() != totcurves ||
	    sample_parents->size() != 4 * totcurves ||
	    sample_parent_weights->size() != 4 * totcurves)
		return PTC_READ_SAMPLE_INVALID;
	
	if (m_strands && (m_strands->totcurves != totcurves || m_strands->totverts != totverts))
		m_strands = NULL;
	if (!m_strands)
		m_strands = BKE_strands_children_new(totcurves, totverts);
	
	const int32_t *numvert = sample_numvert->get();
	const Quatf *root_rot = sample_root_rotations->get();
	const V3f *root_positions = sample_root_positions->get();
	const float32_t *cutoff = sample_cutoff ? sample_cutoff.getVals()->get() : NULL;
	const int32_t *parents = sample_parents->get();
	const float32_t *parent_weights = sample_parent_weights->get();
	for (int i = 0; i < sample_numvert->size(); ++i) {
		StrandsChildCurve *scurve = &m_strands->curves[i];
		scurve->numverts = *numvert;
		scurve->cutoff = -1.0f;
		
		float qt[4] = {root_rot->r, root_rot->v.x, root_rot->v.y, root_rot->v.z};
		quat_to_mat4(scurve->root_matrix, qt);
		copy_v3_v3(scurve->root_matrix[3], root_positions->getValue());
		
		scurve->cutoff = cutoff ? *cutoff : -1.0f;
		
		scurve->parents[0] = parents[0];
		scurve->parents[1] = parents[1];
		scurve->parents[2] = parents[2];
		scurve->parents[3] = parents[3];
		scurve->parent_weights[0] = parent_weights[0];
		scurve->parent_weights[1] = parent_weights[1];
		scurve->parent_weights[2] = parent_weights[2];
		scurve->parent_weights[3] = parent_weights[3];
		
		++numvert;
		++root_rot;
		++root_positions;
		parents += 4;
		parent_weights += 4;
		if (cutoff) ++cutoff;
	}
	
	if (sample_curve_uvs->size() > 0 && sample_curve_uvs->size() % totcurves == 0) {
		int num_layers = sample_curve_uvs->size() / totcurves;
		
		const V2f *uvs = sample_curve_uvs->get();
		
		BKE_strands_children_add_uvs(m_strands, num_layers);
		
		StrandsChildCurveUV *scurve_uv = m_strands->curve_uvs;
		for (int j = 0; j < num_layers; ++j) {
			for (int i = 0; i < totcurves; ++i) {
				copy_v2_v2(scurve_uv->uv, uvs->getValue());
				
				++uvs;
				++scurve_uv;
			}
		}
	}
	
	if (sample_curve_vcols->size() > 0 && sample_curve_vcols->size() % totcurves == 0) {
		int num_layers = sample_curve_vcols->size() / totcurves;
		
		const C3f *vcols = sample_curve_vcols->get();
		
		BKE_strands_children_add_vcols(m_strands, num_layers);
		
		StrandsChildCurveVCol *scurve_vcol = m_strands->curve_vcols;
		for (int j = 0; j < num_layers; ++j) {
			for (int i = 0; i < totcurves; ++i) {
				copy_v3_v3(scurve_vcol->vcol, vcols->getValue());
				
				++vcols;
				++scurve_vcol;
			}
		}
	}
	
	const V3f *co = sample_co->get();
	const float32_t *curve_time = sample_time.getVals()->get();
	for (int i = 0; i < sample_co->size(); ++i) {
		StrandsChildVertex *svert = &m_strands->verts[i];
		copy_v3_v3(svert->co, co->getValue());
		copy_v3_v3(svert->base, svert->co);
		svert->time = *curve_time;
		
		++co;
		++curve_time;
	}
	
	BKE_strands_children_ensure_normals(m_strands);
	
	return PTC_READ_SAMPLE_EXACT;
}

StrandsChildren *AbcStrandsChildrenReader::acquire_result()
{
	StrandsChildren *strands = m_strands;
	m_strands = NULL;
	return strands;
}

void AbcStrandsChildrenReader::discard_result()
{
	BKE_strands_children_free(m_strands);
	m_strands = NULL;
}


AbcStrandsReader::AbcStrandsReader(Strands *strands, StrandsChildren *children, bool read_motion, bool read_children) :
    m_read_motion(read_motion),
    m_read_children(read_children),
    m_strands(strands),
    m_child_reader(children)
{
}

AbcStrandsReader::~AbcStrandsReader()
{
	discard_result();
}

void AbcStrandsReader::init(ReaderArchive *archive)
{
	AbcReader::init(archive);
	m_child_reader.init(archive);
}

void AbcStrandsReader::init_abc(IObject object)
{
	if (m_curves)
		return;
	m_curves = ICurves(object, kWrapExisting);
	
	ICurvesSchema &schema = m_curves.getSchema();
	ICompoundProperty geom_props = schema.getArbGeomParams();
	
	m_param_root_rot = IQuatfGeomParam(geom_props, "root_rotations");
	m_param_root_orig_verts = IUInt32GeomParam(geom_props, "root_orig_verts");
	m_param_root_orig_weights = IFloatGeomParam(geom_props, "root_orig_weights");
	m_param_root_orig_poly = IInt32GeomParam(geom_props, "root_orig_poly");
	m_param_root_orig_loops = IUInt32GeomParam(geom_props, "root_orig_loops");
	
	m_param_times = IFloatGeomParam(geom_props, "times");
	m_param_weights = IFloatGeomParam(geom_props, "weights");
	
	if (m_read_motion && geom_props.getPropertyHeader("motion_state")) {
		m_param_motion_state = ICompoundProperty(geom_props, "motion_state");
		m_param_motion_co = IP3fGeomParam(m_param_motion_state, "position");
		m_param_motion_vel = IV3fGeomParam(m_param_motion_state, "velocity");
	}
	
	if (m_read_children && m_curves.getChildHeader("children")) {
		IObject child = m_curves.getChild("children");
		m_child_reader.init_abc(child);
	}
}

PTCReadSampleResult AbcStrandsReader::read_sample_abc(chrono_t time)
{
	ISampleSelector ss = get_frame_sample_selector(time);
	
	if (!m_curves.valid())
		return PTC_READ_SAMPLE_INVALID;
	
	ICurvesSchema &schema = m_curves.getSchema();
	if (schema.getNumSamples() == 0)
		return PTC_READ_SAMPLE_INVALID;
	
	ICurvesSchema::Sample sample, sample_base;
	schema.get(sample, ss);
	schema.get(sample_base, ISampleSelector((index_t)0));
	
	P3fArraySamplePtr sample_co = sample.getPositions();
	P3fArraySamplePtr sample_co_base = sample_base.getPositions();
	Int32ArraySamplePtr sample_numvert = sample.getCurvesNumVertices();
	IQuatfGeomParam::Sample sample_root_rotations = m_param_root_rot.getExpandedValue(ss);
	IUInt32GeomParam::Sample sample_root_orig_verts = m_param_root_orig_verts.getExpandedValue(ss);
	IFloatGeomParam::Sample sample_root_orig_weights = m_param_root_orig_weights.getExpandedValue(ss);
	IInt32GeomParam::Sample sample_root_orig_poly = m_param_root_orig_poly.getExpandedValue(ss);
	IUInt32GeomParam::Sample sample_root_orig_loops = m_param_root_orig_loops.getExpandedValue(ss);
	IQuatfGeomParam::Sample sample_root_rotations_base = m_param_root_rot.getExpandedValue(ISampleSelector((index_t)0));
	IFloatGeomParam::Sample sample_time = m_param_times.getExpandedValue(ss);
	IFloatGeomParam::Sample sample_weight = m_param_weights.getExpandedValue(ss);
	
	if (!sample_co || !sample_numvert || !sample_co_base || sample_co_base->size() != sample_co->size())
		return PTC_READ_SAMPLE_INVALID;
	
	if (m_strands && (m_strands->totcurves != sample_numvert->size() || m_strands->totverts != sample_co->size()))
		m_strands = NULL;
	if (!m_strands)
		m_strands = BKE_strands_new(sample_numvert->size(), sample_co->size());
	
	const int32_t *numvert = sample_numvert->get();
	const Quatf *root_rot = sample_root_rotations.getVals()->get();
	const uint32_t *orig_verts = sample_root_orig_verts.getVals()->get();
	const float32_t *orig_weights = sample_root_orig_weights.getVals()->get();
	const int32_t *orig_poly = sample_root_orig_poly.getVals()->get();
	const uint32_t *orig_loops = sample_root_orig_loops.getVals()->get();
	for (int i = 0; i < sample_numvert->size(); ++i) {
		StrandsCurve *scurve = &m_strands->curves[i];
		scurve->numverts = *numvert;
		float qt[4] = {root_rot->r, root_rot->v.x, root_rot->v.y, root_rot->v.z};
		quat_to_mat3(scurve->root_matrix, qt);
		scurve->msurf.orig_verts[0] = orig_verts[0];
		scurve->msurf.orig_verts[1] = orig_verts[1];
		scurve->msurf.orig_verts[2] = orig_verts[2];
		scurve->msurf.orig_weights[0] = orig_weights[0];
		scurve->msurf.orig_weights[1] = orig_weights[1];
		scurve->msurf.orig_weights[2] = orig_weights[2];
		scurve->msurf.orig_poly = *orig_poly;
		scurve->msurf.orig_loops[0] = orig_loops[0];
		scurve->msurf.orig_loops[1] = orig_loops[1];
		scurve->msurf.orig_loops[2] = orig_loops[2];
		
		++numvert;
		++root_rot;
		orig_verts += 3;
		orig_weights += 3;
		orig_poly += 1;
		orig_loops += 3;
	}
	
	const V3f *co = sample_co->get();
	const float32_t *curve_time = sample_time.getVals()->get();
	const float32_t *weight = sample_weight.getVals()->get();
	for (int i = 0; i < sample_co->size(); ++i) {
		StrandsVertex *svert = &m_strands->verts[i];
		copy_v3_v3(svert->co, co->getValue());
		svert->time = *curve_time;
		svert->weight = *weight;
		
		++co;
		++curve_time;
		++weight;
	}
	
	/* Correction for base coordinates: these are in object space of frame 1,
	 * but we want the relative shape. Offset them to the current root location.
	 */
	const Quatf *root_rot_base = sample_root_rotations_base.getVals()->get();
	const V3f *co_base = sample_co_base->get();
	StrandIterator it_strand;
	for (BKE_strand_iter_init(&it_strand, m_strands); BKE_strand_iter_valid(&it_strand); BKE_strand_iter_next(&it_strand)) {
		if (it_strand.curve->numverts <= 0)
			continue;
		
		float hairmat_base[4][4];
		float qt[4] = {root_rot_base->r, root_rot_base->v.x, root_rot_base->v.y, root_rot_base->v.z};
		quat_to_mat4(hairmat_base, qt);
		copy_v3_v3(hairmat_base[3], co_base[0].getValue());
		
		float hairmat[4][4];
		copy_m4_m3(hairmat, it_strand.curve->root_matrix);
		copy_v3_v3(hairmat[3], it_strand.verts[0].co);
		
		float mat[4][4];
		invert_m4_m4(mat, hairmat_base);
		mul_m4_m4m4(mat, hairmat, mat);
		
		StrandVertexIterator it_vert;
		for (BKE_strand_vertex_iter_init(&it_vert, &it_strand); BKE_strand_vertex_iter_valid(&it_vert); BKE_strand_vertex_iter_next(&it_vert)) {
//			mul_v3_m4v3(it_vert.vertex->base, mat, co_base->getValue());
			copy_v3_v3(it_vert.vertex->base, it_vert.vertex->co);
			
			++co_base;
		}
		
		++root_rot_base;
	}
	
	if (m_read_motion &&
	    m_param_motion_co && m_param_motion_co.getNumSamples() > 0 &&
	    m_param_motion_vel && m_param_motion_vel.getNumSamples() > 0)
	{
		IP3fGeomParam::Sample sample_motion_co = m_param_motion_co.getExpandedValue(ss);
		IV3fGeomParam::Sample sample_motion_vel = m_param_motion_vel.getExpandedValue(ss);
		
		const V3f *co = sample_motion_co.getVals()->get();
		const V3f *vel = sample_motion_vel.getVals()->get();
		if (co && vel) {
			BKE_strands_add_motion_state(m_strands);
			
			for (int i = 0; i < m_strands->totverts; ++i) {
				StrandsMotionState *ms = &m_strands->state[i];
				copy_v3_v3(ms->co, co->getValue());
				copy_v3_v3(ms->vel, vel->getValue());
				
				++co;
				++vel;
			}
		}
	}
	
	BKE_strands_ensure_normals(m_strands);
	
	if (m_read_children) {
		m_child_reader.read_sample_abc(time);
	}
	
	return PTC_READ_SAMPLE_EXACT;
}

Strands *AbcStrandsReader::acquire_result()
{
	Strands *strands = m_strands;
	m_strands = NULL;
	return strands;
}

void AbcStrandsReader::discard_result()
{
	BKE_strands_free(m_strands);
	m_strands = NULL;
}


} /* namespace PTC */
