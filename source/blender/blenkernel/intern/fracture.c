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
 * Copyright (C) 2014 by Martin Felke.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/blenkernel/intern/fracture.c
 *  \ingroup blenkernel
 */

#include <stdio.h>
#include <stdlib.h>

#include "MEM_guardedalloc.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_customdata.h"
#include "BKE_deform.h"
#include "BKE_DerivedMesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_global.h"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_pointcache.h"
#include "BKE_rigidbody.h"

#include "BLI_edgehash.h"
#include "BLI_kdtree.h"
#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_mempool.h"
#include "BLI_path_util.h"
#include "BLI_rand.h"
#include "BLI_string.h"
#include "BLI_sort.h"
#include "BLI_task.h"
#include "BLI_utildefines.h"

#include "DNA_scene_types.h"
#include "DNA_fracture_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_group_types.h"
#include "DNA_material_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_rigidbody_types.h"

#include "bmesh.h"

#include "RBI_api.h"
//#include "GPU_glew.h" /* uaahh, direct access to modelview matrix */

/* debug timing */
#define USE_DEBUG_TIMER

#ifdef USE_DEBUG_TIMER
#include "PIL_time.h"
#endif

#ifdef WITH_VORO
#include "../../../../extern/voro++/src/c_interface.hh"
#endif

/* prototypes */
static void add_shard(FracMesh *fm, Shard *s, float mat[4][4]);
static Shard *parse_cell(cell c);
static void parse_cell_verts(cell c, MVert *mvert, int totvert);
static void parse_cell_polys(cell c, MPoly *mpoly, int totpoly, int *r_totloop);
static void parse_cell_loops(cell c, MLoop *mloop, int totloop, MPoly *mpoly, int totpoly);
static void parse_cell_neighbors(cell c, int *neighbors, int totpoly);
static void fracture_collect_layers(Shard *shard, DerivedMesh *result, int vertstart, int polystart, int loopstart, int edgestart);
static void remove_participants(RigidBodyShardCon *con, MeshIsland *mi);

static void add_shard(FracMesh *fm, Shard *s, float mat[4][4])
{
	MVert *mv;
	int i = 0;

	for (i = 0, mv = s->mvert; i < s->totvert; i++, mv++ )
	{
		mul_m4_v3(mat, mv->co);
	}

	mul_m4_v3(mat, s->centroid);

	BLI_addtail(&fm->shard_map, s);
	s->shard_id = fm->shard_count;
	fm->shard_count++;
}

static BMesh *shard_to_bmesh(Shard *s)
{
	DerivedMesh *dm_parent;
	BMesh *bm_parent;
	BMIter iter;
	BMFace *f;

	bm_parent = BM_mesh_create(&bm_mesh_allocsize_default,  &((struct BMeshCreateParams){.use_toolflags = true,}));
	dm_parent = BKE_shard_create_dm(s, true);
	DM_to_bmesh_ex(dm_parent, bm_parent, true);
	BM_mesh_elem_table_ensure(bm_parent, BM_VERT | BM_FACE);

	BM_ITER_MESH (f, &iter, bm_parent, BM_FACES_OF_MESH)
	{
		BM_elem_flag_disable(f, BM_ELEM_SELECT);
	}

	dm_parent->needsFree = 1;
	dm_parent->release(dm_parent);
	dm_parent = NULL;

	return bm_parent;
}

static void shard_boundbox(Shard *s, float r_loc[3], float r_size[3])
{
	float min[3], max[3];
	float mloc[3], msize[3];

	if (!r_loc) r_loc = mloc;
	if (!r_size) r_size = msize;

	if (!BKE_shard_calc_minmax(s)) {
		min[0] = min[1] = min[2] = -1.0f;
		max[0] = max[1] = max[2] = 1.0f;
	}

	copy_v3_v3(max, s->max);
	copy_v3_v3(min, s->min);

	mid_v3_v3v3(r_loc, min, max);

	r_size[0] = (max[0] - min[0]) / 2.0f;
	r_size[1] = (max[1] - min[1]) / 2.0f;
	r_size[2] = (max[2] - min[2]) / 2.0f;
}

static int shard_sortdist(const void *s1, const void *s2, void* context)
{
	Shard **sh1 = (Shard **)s1;
	Shard **sh2 = (Shard **)s2;
	cell *sh = (cell*)context;

	float val_a,  val_b;

	if ((*sh1 == NULL) || (*sh2 == NULL)) {
		return -1;
	}

	val_a = len_squared_v3v3(sh->centroid, (*sh1)->centroid);
	val_b = len_squared_v3v3(sh->centroid, (*sh2)->centroid);

	/* sort descending */
	if      (val_a < val_b) return -1;
	else if (val_a > val_b) return 1;
	return 0;
}

static int shard_sortsize(const void *s1, const void *s2, void* UNUSED(context))
{
	Shard **sh1 = (Shard **)s1;
	Shard **sh2 = (Shard **)s2;

	float size1[3], size2[3], loc[3];
	float val_a,  val_b;

	if ((*sh1 == NULL) || (*sh2 == NULL)) {
		return -1;
	}

	shard_boundbox(*sh1, loc, size1);
	shard_boundbox(*sh2, loc, size2);

	//squared diameter
	val_a = size1[0]*size1[0] + size1[1]*size1[1] + size1[2]*size1[2];
	val_b = size2[0]*size2[0] + size2[1]*size2[1] + size2[2]*size2[2];

	/* sort descending */
	if      (val_a < val_b) return 1;
	else if (val_a > val_b) return -1;
	return 0;
}

void* check_add_layer(CustomData *src, CustomData *dst, int type, int totelem, const char* name)
{
	void *layer =  CustomData_get_layer_named(dst, type, name);

	if (!layer) {
		void* orig = NULL;

		if (src) {
			orig = CustomData_get_layer_named(src, type, name);
		}

		if (orig) {
			return CustomData_add_layer_named(dst, type, CD_DUPLICATE, orig, totelem, name);
		}
		else{
			return CustomData_add_layer_named(dst, type, CD_CALLOC, NULL, totelem, name);
		}
	}
	else {
		return layer;
	}
}

Shard *BKE_custom_data_to_shard(Shard *s, DerivedMesh *dm)
{
	CustomData_reset(&s->vertData);
	CustomData_reset(&s->loopData);
	CustomData_reset(&s->polyData);
	CustomData_reset(&s->edgeData);

	CustomData_copy(&dm->vertData, &s->vertData, CD_MASK_MDEFORMVERT, CD_DUPLICATE, s->totvert);
	CustomData_copy(&dm->loopData, &s->loopData, CD_MASK_MLOOPUV, CD_DUPLICATE, s->totloop);
	CustomData_copy(&dm->polyData, &s->polyData, CD_MASK_MTEXPOLY, CD_DUPLICATE, s->totpoly);
	CustomData_copy(&dm->edgeData, &s->edgeData, CD_MASK_CREASE | CD_MASK_BWEIGHT | CD_MASK_MEDGE, CD_DUPLICATE, s->totedge);

	//add velocity vertex layers...
	check_add_layer(&dm->vertData, &s->vertData, CD_PROP_FLT, s->totvert, "velX");
	check_add_layer(&dm->vertData, &s->vertData, CD_PROP_FLT, s->totvert, "velY");
	check_add_layer(&dm->vertData, &s->vertData, CD_PROP_FLT, s->totvert, "velZ");

	return s;
}

/* modified from BKE_mesh_center_median */
bool BKE_fracture_shard_center_median(Shard *shard, float cent[3])
{
	int i = shard->totvert;
	MVert *mvert;
	zero_v3(cent);
	for (mvert = shard->mvert; i--; mvert++) {
		add_v3_v3(cent, mvert->co);
	}
	/* otherwise we get NAN for 0 verts */
	if (shard->totvert) {
		mul_v3_fl(cent, 1.0f / (float)shard->totvert);
	}

	return (shard->totvert != 0);
}

/* copied from mesh_evaluate.c */
/**
 * Calculate the volume and volume-weighted centroid of the volume formed by the polygon and the origin.
 * Results will be negative if the origin is "outside" the polygon
 * (+ve normal side), but the polygon may be non-planar with no effect.
 *
 * Method from:
 * - http://forums.cgsociety.org/archive/index.php?t-756235.html
 * - http://www.globalspec.com/reference/52702/203279/4-8-the-centroid-of-a-tetrahedron
 *
 * \note volume is 6x actual volume, and centroid is 4x actual volume-weighted centroid
 * (so division can be done once at the end)
 * \note results will have bias if polygon is non-planar.
 */
static float mesh_calc_poly_volume_and_weighted_centroid(
        const MPoly *mpoly, const MLoop *loopstart, const MVert *mvarray,
        float r_cent[3])
{
	const float *v_pivot, *v_step1;
	float total_volume = 0.0f;

	zero_v3(r_cent);

	v_pivot = mvarray[loopstart[0].v].co;
	v_step1 = mvarray[loopstart[1].v].co;

	for (int i = 2; i < mpoly->totloop; i++) {
		const float *v_step2 = mvarray[loopstart[i].v].co;

		/* Calculate the 6x volume of the tetrahedron formed by the 3 vertices
		 * of the triangle and the origin as the fourth vertex */
		float v_cross[3];
		cross_v3_v3v3(v_cross, v_pivot, v_step1);
		const float tetra_volume = dot_v3v3 (v_cross, v_step2);
		total_volume += tetra_volume;

		/* Calculate the centroid of the tetrahedron formed by the 3 vertices
		 * of the triangle and the origin as the fourth vertex.
		 * The centroid is simply the average of the 4 vertices.
		 *
		 * Note that the vector is 4x the actual centroid so the division can be done once at the end. */
		for (uint j = 0; j < 3; j++) {
			r_cent[j] += tetra_volume * (v_pivot[j] + v_step1[j] + v_step2[j]);
		}

		v_step1 = v_step2;
	}

	return total_volume;
}

/* modified from BKE_mesh_center_centroid */
bool BKE_fracture_shard_center_centroid(Shard *shard, float r_cent[3])
{
	int i = shard->totpoly;
	MPoly *mpoly;
	float poly_volume;
	float total_volume = 0.0f;
	float poly_cent[3];

	zero_v3(r_cent);

	/* calculate a weighted average of polyhedron centroids */
	for (mpoly = shard->mpoly; i--; mpoly++) {
		poly_volume = mesh_calc_poly_volume_and_weighted_centroid(mpoly, shard->mloop + mpoly->loopstart, shard->mvert, poly_cent);

		/* poly_cent is already volume-weighted, so no need to multiply by the volume */
		add_v3_v3(r_cent, poly_cent);
		total_volume += poly_volume;
	}
	/* otherwise we get NAN for 0 polys */
	if (total_volume != 0.0f) {
		/* multipy by 0.25 to get the correct centroid */
		/* no need to divide volume by 6 as the centroid is weighted by 6x the volume, so it all cancels out */
		mul_v3_fl(r_cent, 0.25f / total_volume);
	}

	/* this can happen for non-manifold objects, first fallback to old area based method, then fallback to median there */
	if (!is_finite_v3(r_cent) || total_volume < 0.000001f) {
		return BKE_fracture_shard_center_centroid_area(shard, r_cent);
	}

	copy_v3_v3(shard->centroid, r_cent);
	return (shard->totpoly != 0);
}

/* note, results won't be correct if polygon is non-planar */
/* copied from mesh_evaluate.c */
static float mesh_calc_poly_planar_area_centroid(
        const MPoly *mpoly, const MLoop *loopstart, const MVert *mvarray,
        float r_cent[3])
{
	int i;
	float tri_area;
	float total_area = 0.0f;
	float v1[3], v2[3], v3[3], normal[3], tri_cent[3];

	BKE_mesh_calc_poly_normal(mpoly, loopstart, mvarray, normal);
	copy_v3_v3(v1, mvarray[loopstart[0].v].co);
	copy_v3_v3(v2, mvarray[loopstart[1].v].co);
	zero_v3(r_cent);

	for (i = 2; i < mpoly->totloop; i++) {
		copy_v3_v3(v3, mvarray[loopstart[i].v].co);

		tri_area = area_tri_signed_v3(v1, v2, v3, normal);
		total_area += tri_area;

		mid_v3_v3v3v3(tri_cent, v1, v2, v3);
		madd_v3_v3fl(r_cent, tri_cent, tri_area);

		copy_v3_v3(v2, v3);
	}

	mul_v3_fl(r_cent, 1.0f / total_area);

	return total_area;
}

// old method, keep for now in case new has different results
/* modified from BKE_mesh_center_centroid */
bool BKE_fracture_shard_center_centroid_area(Shard *shard, float cent[3])
{
	int i = shard->totpoly;
	MPoly *mpoly;
	float poly_area;
	float total_area = 0.0f;
	float poly_cent[3];

	zero_v3(cent);

	/* calculate a weighted average of polygon centroids */
	for (mpoly = shard->mpoly; i--; mpoly++) {
		BKE_mesh_calc_poly_center(mpoly, shard->mloop + mpoly->loopstart, shard->mvert, poly_cent);
//		poly_area = BKE_mesh_calc_poly_area(mpoly, shard->mloop + mpoly->loopstart, shard->mvert);
		poly_area = mesh_calc_poly_planar_area_centroid(mpoly, shard->mloop + mpoly->loopstart, shard->mvert,
		                                                poly_cent);
		madd_v3_v3fl(cent, poly_cent, poly_area);
		total_area += poly_area;
	}
	/* otherwise we get NAN for 0 polys */
	if (shard->totpoly) {
		mul_v3_fl(cent, 1.0f / total_area);
	}

	/* zero area faces cause this, fallback to median */
	if (UNLIKELY(!is_finite_v3(cent))) {
		return BKE_fracture_shard_center_median(shard, cent);
	}
	copy_v3_v3(shard->centroid, cent);

	return (shard->totpoly != 0);
}

void BKE_shard_free(Shard *s, bool doCustomData)
{
	if (s->mvert) {
		MEM_freeN(s->mvert);
	}
	if (s->mloop) {
		MEM_freeN(s->mloop);
	}
	if (s->mpoly) {
		MEM_freeN(s->mpoly);
	}
	if (s->medge) {
		MEM_freeN(s->medge);
	}
	if (s->neighbor_ids) {
		MEM_freeN(s->neighbor_ids);
	}
	if (s->cluster_colors) {
		MEM_freeN(s->cluster_colors);
	}

	if (doCustomData) {
		CustomData_free(&s->vertData, s->totvert);
		CustomData_free(&s->loopData, s->totloop);
		CustomData_free(&s->polyData, s->totpoly);
		CustomData_free(&s->edgeData, s->totedge);
	}

	MEM_freeN(s);
}

float BKE_shard_calc_minmax(Shard *shard)
{
	float min[3], max[3], diff[3];
	int i;
	
	INIT_MINMAX(min, max);
	for (i = 0; i < shard->totvert; i++) {
		minmax_v3v3_v3(min, max, shard->mvert[i].co);
	}
	
	copy_v3_v3(shard->min, min);
	copy_v3_v3(shard->max, max);

	sub_v3_v3v3(diff, max, min);
	return len_v3(diff);
}

Shard* BKE_create_initial_shard(DerivedMesh *dm)
{
	/* create temporary shard covering the entire mesh */
	Shard *s = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm), dm->getEdgeArray(dm),
	                                     dm->numVertData, dm->numPolyData, dm->numLoopData, dm->numEdgeData, true);
	s = BKE_custom_data_to_shard(s, dm);
	s->flag = SHARD_INTACT;
	s->shard_id = -2;
	return s;
}

Shard* BKE_fracture_shard_copy(Shard *s)
{
	Shard *t = BKE_create_fracture_shard(s->mvert, s->mpoly, s->mloop, s->medge, s->totvert, s->totpoly, s->totloop, s->totedge, true);
	copy_v3_v3(t->centroid, s->centroid);
	t->neighbor_count = s->neighbor_count;
	t->neighbor_ids = MEM_mallocN(sizeof(int) * t->neighbor_count, __func__);
	memcpy(t->neighbor_ids, s->neighbor_ids, sizeof(int) * t->neighbor_count);
	copy_v3_v3(t->raw_centroid, s->raw_centroid);
	t->raw_volume = s->raw_volume;
	t->shard_id = s->shard_id;
	t->setting_id = s->setting_id;
	t->flag = s->flag;
	t->parent_id = s->parent_id;
	copy_v3_v3(t->max, s->max);
	copy_v3_v3(t->min, s->min);
	copy_v3_v3(t->impact_loc, s->impact_loc);
	copy_v3_v3(t->impact_size, s->impact_size);
	//TODO, maybe cluster colors too ?

	CustomData_reset(&t->vertData);
	CustomData_reset(&t->loopData);
	CustomData_reset(&t->polyData);
	CustomData_reset(&t->edgeData);

	CustomData_copy(&s->vertData, &t->vertData, CD_MASK_MDEFORMVERT, CD_DUPLICATE, s->totvert);
	CustomData_copy(&s->loopData, &t->loopData, CD_MASK_MLOOPUV, CD_DUPLICATE, s->totloop);
	CustomData_copy(&s->polyData, &t->polyData, CD_MASK_MTEXPOLY, CD_DUPLICATE, s->totpoly);
	CustomData_copy(&s->edgeData, &t->edgeData, CD_MASK_CREASE | CD_MASK_BWEIGHT | CD_MASK_MEDGE, CD_DUPLICATE, s->totedge);

	return t;
}


/*access shard directly by index / id*/
Shard *BKE_shard_by_id(FracMesh *mesh, ShardID id, DerivedMesh *dm) {
	if (/*(id < mesh->shard_count) && */(id >= 0)) {
		//return mesh->shard_map[id];
		//return (Shard *)BLI_findlink(&mesh->shard_map, id);
		Shard *s = mesh->shard_map.first;
		while (s)
		{
			if (s->shard_id == id)
			{
				return s;
			}
			s = s->next;
		}

		return NULL;
	}
	else if (id == -1 && dm != NULL)
	{
		/* create temporary shard covering the entire mesh */
		return BKE_create_initial_shard(dm);
	}
	
	return NULL;
}

bool BKE_get_shard_minmax(FracMesh *mesh, ShardID id, float min_r[3], float max_r[3], DerivedMesh *dm)
{
	Shard *shard = BKE_shard_by_id(mesh, id, dm);
	if (shard != NULL) {
		BKE_shard_calc_minmax(shard);
		copy_v3_v3(min_r, shard->min);
		copy_v3_v3(max_r, shard->max);

		if (shard->shard_id == -2)
		{
			BKE_shard_free(shard, true);
		}

		return true;
	}
	return false;
}

Shard *BKE_create_fracture_shard(MVert *mvert, MPoly *mpoly, MLoop *mloop, MEdge* medge,  int totvert, int totpoly,
                                 int totloop, int totedge, bool copy)
{
	Shard *shard = MEM_mallocN(sizeof(Shard), __func__);
	shard->totvert = totvert;
	shard->totpoly = totpoly;
	shard->totloop = totloop;
	shard->totedge = totedge;
	shard->cluster_colors = NULL;
	shard->neighbor_ids = NULL;
	shard->neighbor_count = 0;
	
	if (copy) {
		shard->mvert = MEM_mallocN(sizeof(MVert) * totvert, "shard vertices");
		shard->mpoly = MEM_mallocN(sizeof(MPoly) * totpoly, "shard polys");
		shard->mloop = MEM_mallocN(sizeof(MLoop) * totloop, "shard loops");
		shard->medge = MEM_mallocN(sizeof(MEdge) * totedge, "shard edges");
		memcpy(shard->mvert, mvert, sizeof(MVert) * totvert);
		memcpy(shard->mpoly, mpoly, sizeof(MPoly) * totpoly);
		memcpy(shard->mloop, mloop, sizeof(MLoop) * totloop);
		memcpy(shard->medge, medge, sizeof(MEdge) * totedge);
	}
	else {
		shard->mvert = mvert;
		shard->mpoly = mpoly;
		shard->mloop = mloop;
		shard->medge = medge;
	}

	shard->shard_id = -1;
	shard->setting_id = -1;
	shard->parent_id = -1;

	shard->flag = SHARD_INTACT;
	BKE_shard_calc_minmax(shard);

	BKE_fracture_shard_center_centroid_area(shard, shard->centroid);
	copy_v3_v3(shard->raw_centroid, shard->centroid);
	zero_v3(shard->impact_loc);
	shard->impact_size[0] = 1.0f;
	shard->impact_size[1] = 1.0f;
	shard->impact_size[2] = 1.0f;

	return shard;
}

FracMesh *BKE_create_fracture_container(void)
{
	FracMesh *fmesh;
	
	fmesh = MEM_mallocN(sizeof(FracMesh), __func__);
	fmesh->shard_map.first = NULL;
	fmesh->shard_map.last = NULL;
	fmesh->shard_count = 0;
	fmesh->cancel = 0;
	fmesh->running = 0;
	fmesh->progress_counter = 0;
	fmesh->last_shards = NULL;
	fmesh->last_shard_tree = NULL;
	fmesh->last_expected_shards = 0;
	
	return fmesh;
}

static void handle_fast_bisect(FracMesh *fm, int expected_shards, int algorithm, BMesh** bm_parent, float obmat[4][4],
                               float centroid[3], short inner_material_index, int parent_id, Shard **tempshards, Shard ***tempresults,
                               char uv_layer[64], cell* cells, float fac, Shard* parent)
{
	int i = 0, index = 0;
	float factor = 1 - fac;

	float dim[3];
	sub_v3_v3v3(dim, parent->max, parent->min);

	for (i = 0; i < expected_shards; i++) {
		Shard *s = NULL;
		Shard *s2 = NULL;
		Shard *t;
		float vec[3];
		int max_axis;

		if (fm->cancel == 1) {
			break;
		}

		printf("Processing shard: %d\n", i);
		t = tempshards[i];

		if (t != NULL) {
			t->parent_id = parent_id;
			t->flag = SHARD_INTACT;
		}

		if (t == NULL || t->totvert == 0 || t->totloop == 0 || t->totpoly == 0) {
			/* invalid shard, stop parsing*/
			continue;
		}

		//index = (int)(BLI_frand() * (t->totpoly - 1));

		if (index > (t->totpoly - 1)){
			index = 0;
		}

		//make a random vector (interpret as cutter plane)
		vec[0] = BLI_frand() * 2 - 1;
		vec[1] = BLI_frand() * 2 - 1;
		vec[2] = BLI_frand() * 2 - 1;

		//multiply two minor dimensions with a factor to emphasize the max dimension
		max_axis = axis_dominant_v3_single(dim);
		switch (max_axis) {
			case 0:
				vec[1] *= factor;
				vec[2] *= factor;
				break;
			case 1:
				vec[0] *= factor;
				vec[2] *= factor;
				break;
			case 2:
				vec[0] *= factor;
				vec[1] *= factor;
				break;
		}

		printf("Bisecting cell %d...\n", i);
		printf("Bisecting cell %d...\n", i + 1);

		s = BKE_fracture_shard_bisect(*bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL,
		                              false, true, index, centroid, inner_material_index, uv_layer, NULL, vec);
		s2 = BKE_fracture_shard_bisect(*bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL,
		                               true, false, index, centroid, inner_material_index, uv_layer, NULL, vec);

		index++;

		if (s == NULL || s2 == NULL) {
			printf("Shard missed....\n");
			continue;
		}

		if (s != NULL && s2 != NULL && tempresults != NULL) {
			int j = 0;

			fm->progress_counter++;

			s->parent_id = parent_id;
			s->flag = SHARD_INTACT;

			s2->parent_id = parent_id;
			s2->flag = SHARD_INTACT;

			if (*bm_parent != NULL) {
				BM_mesh_free(*bm_parent);
				*bm_parent = NULL;
			}

			(*tempresults)[i] = s;
			(*tempresults)[i + 1] = s2;

			//BLI_qsort_r(*tempresults, i + 1, sizeof(Shard *), shard_sortdist, &(cells[i]));
			BLI_qsort_r(*tempresults, i + 1, sizeof(Shard *), shard_sortsize, &(cells[i]));

			while ((*tempresults)[j] == NULL && j < (i + 1)) {
				/* ignore invalid shards */
				j++;
			}

			/* continue splitting if not all expected shards exist yet */
			if ((i + 2) < expected_shards) {
				*bm_parent = shard_to_bmesh((*tempresults)[j]);
				copy_v3_v3(centroid, (*tempresults)[j]->centroid);
				sub_v3_v3v3(dim, (*tempresults)[j]->max, (*tempresults)[j]->min);

				BKE_shard_free((*tempresults)[j], true);
				(*tempresults)[j] = NULL;
			}
			i++;
		}
	}
}

static void handle_boolean_fractal(Shard* p, Shard* t, int expected_shards, DerivedMesh* dm_parent, Object *obj, short inner_material_index,
                                   int num_cuts, float fractal, int num_levels, bool smooth,int parent_id, int* i, Shard ***tempresults,
                                   DerivedMesh **dm_p, char uv_layer[64], int solver, int thresh, float fac)
{
	/* physics shard and fractalized shard, so we need to booleanize twice */
	/* and we need both halves, so twice again */
	Shard *s2 = NULL;
	Shard *s = NULL;
	int index = 0;
	int max_retries = 3;
	float factor = 1 - fac;

	/*continue with "halves", randomly*/
	if ((*i) == 0) {
		*dm_p = CDDM_copy(dm_parent);
	}

	while (s == NULL || s2 == NULL) {

		float radius;
		float size[3];
		float quat[4];
		float loc[3], vec[3];
		float min[3], max[3];
		float one[3] = {1.0f, 1.0f, 1.0f};
		float matrix[4][4];
		int max_axis;

		/*make a plane as cutter*/
//		shard_boundbox(p, loc, size);
		INIT_MINMAX(min, max);
		(*dm_p)->getMinMax(*dm_p, min, max);

		mid_v3_v3v3(loc, min, max);
		size[0] = (max[0] - min[0]) / 2.0f;
		size[1] = (max[1] - min[1]) / 2.0f;
		size[2] = (max[2] - min[2]) / 2.0f;

		radius = sqrt(size[0]*size[0] + size[1]*size[1] + size[2]*size[2]);

		vec[0] = BLI_frand() * 2 - 1;
		vec[1] = BLI_frand() * 2 - 1;
		vec[2] = BLI_frand() * 2 - 1;

		//multiply two minor dimensions with a factor to emphasize the max dimension
		max_axis = axis_dominant_v3_single(size);
		switch (max_axis) {
			case 0:
				vec[1] *= factor;
				vec[2] *= factor;
				break;
			case 1:
				vec[0] *= factor;
				vec[2] *= factor;
				break;
			case 2:
				vec[0] *= factor;
				vec[1] *= factor;
				break;
		}

		//printf("(%f %f %f) (%f %f %f) \n", size[0], size[1], size[2], eul[0], eul[1], eul[2]);*/
		//loc_eul_size_to_mat4(matrix, loc, vec, one);
		vec_to_quat(quat, vec, OB_POSZ, OB_POSX);
		loc_quat_size_to_mat4(matrix, loc, quat, one);

		/*visual shards next, fractalized cuts */
		s = BKE_fracture_shard_boolean(obj, *dm_p, t, inner_material_index, num_cuts,fractal, &s2, matrix, radius, smooth, num_levels, uv_layer, solver, thresh);

		if (index < max_retries)
		{
			printf("Retrying...%d\n", index);
			index++;
		}
		else if (s == NULL || s2 == NULL)
		{
			(*i)++;
			break;
		}
	}

	if ((s != NULL) && (s2 != NULL)) {
		int j = 0;

		s->parent_id = parent_id;
		s->flag = SHARD_INTACT;
		(*tempresults)[(*i)+1] = s;

		s2->parent_id = parent_id;
		s2->flag = SHARD_INTACT;
		(*tempresults)[*i] = s2;

		BLI_qsort_r(*tempresults, (*i) + 1, sizeof(Shard *), shard_sortsize, i);
		while ((*tempresults)[j] == NULL && j < ((*i) + 1)) {
			/* ignore invalid shards */
			j++;
		}

		/* continue splitting if not all expected shards exist yet */
		if (((*i) + 2) < expected_shards) {

			Shard *p = (*tempresults)[j];

			if (*dm_p != NULL) {
				(*dm_p)->needsFree = 1;
				(*dm_p)->release(*dm_p);
				*dm_p = NULL;
			}

			if (p != NULL) {
				*dm_p = BKE_shard_create_dm(p, true);
				BKE_shard_free((*tempresults)[j], true);
				(*tempresults)[j] = NULL;
			}
		}
		(*i)++; //XXX remember to "double" the shard amount....
	}
}

static bool handle_boolean_bisect(FracMesh *fm, Object *obj, int expected_shards, int algorithm, int parent_id, Shard **tempshards,
                                  DerivedMesh *dm_parent, BMesh* bm_parent, float obmat[4][4], short inner_material_index, int num_cuts,
                                  int num_levels, float fractal, int *i, bool smooth, Shard*** tempresults, DerivedMesh **dm_p, char uv_layer[64],
                                  KDTree *preselect_tree, int solver, int thresh, Shard* p, float fac)
{
	Shard *s = NULL, *t = NULL;
	if (fm->cancel == 1)
		return true;

	t = tempshards[*i];

	if (t != NULL) {
		t->parent_id = parent_id;
		t->flag = SHARD_INTACT;
	}

	if (t == NULL || t->totvert == 0 || t->totloop == 0 || t->totpoly == 0) {
		/* invalid shard, stop parsing */
		return true;
	}

	printf("Processing shard: %d\n", *i);

	/* XXX TODO, need object for material as well, or atleast a material index... */
	if (algorithm == MOD_FRACTURE_BOOLEAN) {
		s = BKE_fracture_shard_boolean(obj, dm_parent, t, inner_material_index, 0, 0.0f, NULL, NULL, 0.0f, false, 0, uv_layer, solver, thresh);
	}
	else if (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
		handle_boolean_fractal(p, t, expected_shards, dm_parent, obj, inner_material_index, num_cuts, fractal,
		                       num_levels, smooth, parent_id, i, tempresults, dm_p, uv_layer, solver, thresh, fac);
	}
	else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL) {
		float co[3] = {0, 0, 0}, quat[4] =  {1, 0, 0, 0};
		printf("Bisecting cell %d...\n", *i);
		s = BKE_fracture_shard_bisect(bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FILL, false, true, -1, co, inner_material_index, uv_layer,
		                              preselect_tree, quat);
	}
	else {
		/* do not fracture case */
		s = t;
	}

	if ((s != NULL) && (algorithm != MOD_FRACTURE_BOOLEAN_FRACTAL)) {
		s->parent_id = parent_id;
		s->flag = SHARD_INTACT;

		(*tempresults)[*i] = s;
	}

	fm->progress_counter++;
	return false;
}

static void do_prepare_cells(FracMesh *fm, cell *cells, int expected_shards, int algorithm, Shard *p, float (*centroid)[3],
                             DerivedMesh **dm_parent, BMesh** bm_parent, Shard ***tempshards, Shard ***tempresults, int override_count,
                             FractureModifierData *fmd)
{
	int i;
	Shard *s = NULL;
	int *skipmap = MEM_callocN(sizeof(int) * expected_shards, "skipmap");
	int *deletemap = MEM_callocN(sizeof(int) * fm->shard_count, "deletemap");

	if ((algorithm == MOD_FRACTURE_BOOLEAN) || (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL)) {
		MPoly *mpoly, *mp;
		int totpoly, po;

		*dm_parent = BKE_shard_create_dm(p, true);
		mpoly = (*dm_parent)->getPolyArray(*dm_parent);
		totpoly = (*dm_parent)->getNumPolys(*dm_parent);
		for (po = 0, mp = mpoly; po < totpoly; po++, mp++) {
			mp->flag &= ~ME_FACE_SEL;
		}
	}
	else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL ||
	         algorithm == MOD_FRACTURE_BISECT_FAST || algorithm == MOD_FRACTURE_BISECT_FAST_FILL)
	{
		*bm_parent = shard_to_bmesh(p);
		copy_v3_v3(*centroid, p->centroid);
	}

	if (algorithm == MOD_FRACTURE_BISECT_FAST ||
	    algorithm == MOD_FRACTURE_BISECT_FAST_FILL ||
	    algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL)
	{
		copy_vn_i(deletemap, fm->shard_count, 1);
	}

	if (fm->last_shard_tree)
	{
		copy_vn_i(deletemap, fm->shard_count, 1);
		copy_vn_i(skipmap, expected_shards, 1);

		for (i = 0; i < expected_shards; i++)
		{
			KDTreeNearest n;
			int l, j;
			float max = 0;
			for (l = 0; l < cells[i].totpoly; l++)
			{
				int index = cells[i].neighbors[l];
				if (index > -1)
				{
					float dist = len_squared_v3v3(cells[index].centroid, cells[i].centroid);
					if (dist > max)
					{
						max = dist;
					}
				}
			}

			j = BLI_kdtree_find_nearest(fm->last_shard_tree, cells[i].centroid, &n);
			if (j > -1)
			{
				float epsilon = 0.00001;
				Shard *t = fm->last_shards[j];
				if (t != NULL && n.dist < max)
				{
					if (n.dist < epsilon) {
						if ((fabsf(cells[i].volume - t->raw_volume) < epsilon))
						{
							deletemap[j] = false;
						}
						else
						{
							skipmap[i] = false;
						}
					}
					else
					{
						skipmap[i] = false;
					}
				}
				else
				{
					skipmap[i] = false;
				}
			}
			else {
				skipmap[i] = true;
			}
		}
	}

	//BLI_lock_thread(LOCK_CUSTOM1);
	//skipping /deletion pass


	for (i = 0; i < expected_shards; i++)
	{
		if (fm->cancel == 1) {
			break;
		}

		if (skipmap[i])
		{
			printf("Skipping shard: %d\n", i);
			(*tempshards)[i] = NULL;
		}
		else
		{
			printf("Parsing shard: %d\n", i);
			s = parse_cell(cells[i]);
			(*tempshards)[i] = s;
		}

		(*tempresults)[i] = NULL;

		fm->progress_counter++;
	}

	for (i = 0; i < fm->shard_count; i++)
	{
		if (deletemap[i] && fm->last_shards)
		{
			Shard *t = fm->last_shards[i];

			if (!t)
				continue;

			//seems this override count was a totally wrong thought, just passing -1 or 0 here... hmm
			if ((override_count == -1) || ((override_count > 0) && (i < override_count+1)))
			{
				printf("Deleting shard: %d %d %d\n", i, t->shard_id, t->setting_id);
				BLI_remlink_safe(&fm->shard_map, t);
				BKE_shard_free(t, false);
				fm->last_shards[i] = NULL;
			}
			else
			{
				printf("NOT Deleting shard: %d %d %d\n", i, t->shard_id, t->setting_id);
			}
		}
	}

	if (override_count > -1) {
		printf("Deleting island shards!\n");
		while (fmd->islandShards.first) {
			Shard *sh = fmd->islandShards.first;
			if (sh) {
				BLI_remlink_safe(&fmd->islandShards, sh);
				BKE_shard_free(sh, false);
			}
		}
	}
	//BLI_unlock_thread(LOCK_CUSTOM1);

	fm->last_expected_shards = expected_shards;

	MEM_freeN(skipmap);
	MEM_freeN(deletemap);
}


/* parse the voro++ cell data */
//static ThreadMutex prep_lock = BLI_MUTEX_INITIALIZER;
static void parse_cells(cell *cells, int expected_shards, ShardID parent_id, FracMesh *fm, int algorithm, Object *obj, DerivedMesh *dm,
                        short inner_material_index, float mat[4][4], int num_cuts, float fractal, bool smooth, int num_levels, int mode,
                        bool reset, int active_setting, int num_settings, char uv_layer[64], bool threaded, int solver, float thresh, int override_count,
                        float factor)
{
	/*Parse voronoi raw data*/
	int i = 0, j = 0, count = 0;
	Shard *p = BKE_shard_by_id(fm, parent_id, dm); // *t;
	float obmat[4][4]; /* use unit matrix for now */
	float centroid[3], pcentroid[3] = {0,0,0};
	BMesh *bm_parent = NULL;
	DerivedMesh *dm_parent = NULL;
	DerivedMesh *dm_p = NULL;
	Shard **tempshards;
	Shard **tempresults;
	bool do_tree = (algorithm != MOD_FRACTURE_BISECT_FAST &&
					algorithm != MOD_FRACTURE_BISECT_FAST_FILL &&
					algorithm != MOD_FRACTURE_BOOLEAN_FRACTAL);
	FractureModifierData *fmd = (FractureModifierData*) modifiers_findByType(obj, eModifierType_Fracture);

	if (p == NULL || reset)
	{
		if (fm->last_shard_tree)
		{
			BLI_kdtree_free(fm->last_shard_tree);
			fm->last_shard_tree = NULL;
		}

		if (fm->last_shards)
		{
			MEM_freeN(fm->last_shards);
			fm->last_shards = NULL;
		}

		if (!p)
			return;
	}

	if (mode == MOD_FRACTURE_PREFRACTURED && reset && !threaded)
	{
		while (fm->shard_map.first)
		{
			Shard *t = fm->shard_map.first;
			BLI_remlink_safe(&fm->shard_map, t);
			printf("Resetting shard: %d\n", t->shard_id);
			BKE_shard_free(t, true);
		}
	}

	if (mode == MOD_FRACTURE_PREFRACTURED && !reset)
	{
		//rebuild tree
		if (!fm->last_shard_tree && mode == MOD_FRACTURE_PREFRACTURED)
		{
			Shard *t;
			int ti = 0;
			count = BLI_listbase_count(&fm->shard_map);
			fm->shard_count = count;
			if (do_tree)
			{
				fm->last_shard_tree = BLI_kdtree_new(fm->shard_count);
			}

			fm->last_shards = MEM_callocN(sizeof(Shard*) * fm->shard_count, "last_shards");

			//fill tree from current shardmap
			for (t = fm->shard_map.first; t; t = t->next)
			{
				t->flag &=~ (SHARD_SKIP | SHARD_DELETE);

				if (do_tree)
				{
					BLI_kdtree_insert(fm->last_shard_tree, ti, t->raw_centroid);
				}

				fm->last_shards[ti] = t;
				ti++;
			}

			if (do_tree)
			{
				BLI_kdtree_balance(fm->last_shard_tree);
			}

			p->flag |= SHARD_DELETE;
		}
	}
	else
	{
		fm->last_shard_tree = NULL;
		fm->last_shards = NULL;
	}

	tempshards = MEM_callocN(sizeof(Shard *) * expected_shards, "tempshards");
	tempresults = MEM_callocN(sizeof(Shard *) * expected_shards, "tempresults");

	p->flag = 0;
	p->flag |= SHARD_FRACTURED;

	if (mode == MOD_FRACTURE_DYNAMIC)
	{
		copy_v3_v3(pcentroid, p->centroid);
		parent_id = p->shard_id;
		//remove parent shard from map as well
		BLI_remlink(&fm->shard_map, p);
		fm->shard_count--;
		p->shard_id = -2;
	}

	unit_m4(obmat);
	do_prepare_cells(fm, cells, expected_shards, algorithm, p, &centroid, &dm_parent, &bm_parent, &tempshards, &tempresults, override_count, fmd);

	if (fm->last_shard_tree)
	{
		BLI_kdtree_free(fm->last_shard_tree);
		fm->last_shard_tree = NULL;
	}

	if (fm->last_shards)
	{
		MEM_freeN(fm->last_shards);
		fm->last_shards = NULL;
	}

	if (algorithm != MOD_FRACTURE_BISECT_FAST && algorithm != MOD_FRACTURE_BISECT_FAST_FILL) {
		int totvert = p->totvert;
		MVert *mvert = p->mvert;

		KDTree *preselect_tree = BLI_kdtree_new(totvert);
		for (i = 0; i < totvert; i++) {
			BLI_kdtree_insert(preselect_tree, i, mvert[i].co);
		}

		BLI_kdtree_balance(preselect_tree);

		if (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL)
		{
			//attempt to have some variance atleast here too
			BLI_srandom(fmd->point_seed);
		}

		if ((algorithm == MOD_FRACTURE_BOOLEAN) && !threaded)
		{
			#pragma omp parallel for
			for (i = 0; i < expected_shards; i++)	{
				handle_boolean_bisect(fm, obj, expected_shards, algorithm, parent_id, tempshards, dm_parent,
										bm_parent, obmat, inner_material_index, num_cuts, num_levels, fractal,
										&i, smooth, &tempresults, &dm_p, uv_layer, preselect_tree, solver, thresh, p,
										fmd->orthogonality_factor);
			}
		}
		else {
			for (i = 0; i < expected_shards; i++)	{
				handle_boolean_bisect(fm, obj, expected_shards, algorithm, parent_id, tempshards, dm_parent,
										bm_parent, obmat, inner_material_index, num_cuts, num_levels, fractal,
										&i, smooth, &tempresults, &dm_p, uv_layer, preselect_tree, solver, thresh, p,
										fmd->orthogonality_factor);
			}
		}

		BLI_kdtree_free(preselect_tree);
	}
	else {

		if (expected_shards == 1)
		{
			/* do not fracture case */
			tempresults[0] = p;
			p->shard_id = -1;
		}
		else
		{
			handle_fast_bisect(fm, expected_shards, algorithm, &bm_parent, obmat, centroid, inner_material_index, parent_id,
			                   tempshards, &tempresults, uv_layer, cells, factor, p);
		}
	}

	if (bm_parent != NULL) {
		BM_mesh_free(bm_parent);
		bm_parent = NULL;
	}

	if (dm_parent != NULL) {
		dm_parent->needsFree = 1;
		dm_parent->release(dm_parent);
		dm_parent = NULL;
	}

	/*only used with fractal, and is doubly freed in case of 1 shard (doubled) */
	if (dm_p != NULL && expected_shards > 2) {
		dm_p->needsFree = 1;
		dm_p->release(dm_p);
		dm_p = NULL;
	}

	if (p)
	{
		BLI_remlink_safe(&fm->shard_map, p);
		BKE_shard_free(p, true);
		p = NULL;
	}

	fm->shard_count = 0; /* may be not matching with expected shards, so reset... did increment this for
	                      *progressbar only */

	//keep empty ids... need to catch this later
	if (mode == MOD_FRACTURE_DYNAMIC || active_setting > -1)
	{
		j = 1;

		if (fm->shard_map.last)
		{
			j += ((Shard*)(fm->shard_map.last))->shard_id;
		}
	}
	else
	{
		j = 1;
	}

	for (i = 0; i < expected_shards; i++) {
		Shard *s = tempresults[i];
		Shard *t = tempshards[i];

		if (s != NULL) {
			add_shard(fm, s, mat);
			s->shard_id += j;
			s->parent_id = parent_id;
			s->setting_id = active_setting;
			//printf("ADDED: %d %d %d\n", i, j, s->shard_id);
			if (parent_id > -1)
			{
				int si = 0;
				MVert *v;

				sub_v3_v3(s->centroid, pcentroid);
				for (si = 0, v = s->mvert; si < s->totvert; si++, v++)
				{
					sub_v3_v3(v->co, pcentroid);
				}
			}
		}

		if (t != NULL && t != s) {
			BKE_shard_free(t, false);
		}
	}

	if (fm->shard_count == 0)
	{
		//might happen if all has been skipped, but this distracts the halving method (thinks shardmap is empty)
		//so better correct this here
		fm->shard_count = BLI_listbase_count(&fm->shard_map);
	}

	MEM_freeN(tempshards);
	MEM_freeN(tempresults);
}

static Shard *parse_cell(cell c)
{
	Shard *s;
	MVert *mvert = NULL;
	MPoly *mpoly = NULL;
	MLoop *mloop = NULL;
	int *neighbors = NULL;
	int totpoly = 0, totloop = 0, totvert = 0;
	float centr[3];

	totvert = c.totvert;
	if (totvert > 0) {
		mvert = MEM_callocN(sizeof(MVert) * totvert, __func__);
		parse_cell_verts(c, mvert, totvert);
	}

	totpoly = c.totpoly;
	if (totpoly > 0) {
		mpoly = MEM_callocN(sizeof(MPoly) * totpoly, __func__);
		parse_cell_polys(c, mpoly, totpoly, &totloop);
	}
	else
		totloop = 0;

	if (totloop > 0) {
		mloop = MEM_callocN(sizeof(MLoop) * totloop, __func__);
		parse_cell_loops(c, mloop, totloop, mpoly, totpoly);
	}

	if (totpoly > 0) {
		neighbors = MEM_callocN(sizeof(int) * totpoly, __func__);
		parse_cell_neighbors(c, neighbors, totpoly);
	}

	copy_v3_v3(centr, c.centroid);

	s = BKE_create_fracture_shard(mvert, mpoly, mloop, NULL, totvert, totpoly, totloop, 0, false);

	//s->flag &= ~(SHARD_SKIP | SHARD_DELETE);
	s->neighbor_ids = neighbors;
	s->neighbor_count = totpoly;
	copy_v3_v3(s->centroid, centr);
	copy_v3_v3(s->raw_centroid, centr);
	s->raw_volume = c.volume;

	return s;
}

static void parse_cell_verts(cell c, MVert *mvert, int totvert)
{
	int i;

	for (i = 0; i < totvert; i++) {
		float *co = mvert[i].co;
		copy_v3_v3(co, c.verts[i]);
	}
}

static void parse_cell_polys(cell c, MPoly *mpoly, int totpoly, int *r_totloop)
{
	int i;
	int totloop = 0;

	for (i = 0; i < totpoly; ++i) {
		int numloop;

		numloop = c.poly_totvert[i];

		mpoly[i].loopstart = totloop;
		mpoly[i].totloop = numloop;

		totloop += numloop;
	}

	*r_totloop = totloop;
}

static void parse_cell_loops(cell c, MLoop *mloop, int UNUSED(totloop), MPoly *mpoly, int totpoly)
{
	int i, k;

	for (i = 0; i < totpoly; ++i) {
		int loopstart = mpoly[i].loopstart;
		int numloop = mpoly[i].totloop;

		for (k = 0; k < numloop; ++k) {
			int index;

			index = c.poly_indices[i][k];

			/* note: invert vertex order here,
			 * otherwise normals are pointing inward
			 */
			mloop[loopstart + (numloop - 1) - k].v = index;
		}
	}
}

static void parse_cell_neighbors(cell c, int *neighbors, int totpoly)
{
	int i;

	for (i = 0; i < totpoly; i++) {
		int n;
		n = c.neighbors[i];
		neighbors[i] = n;
	}
}

static void stroke_to_faces(FractureModifierData *fmd, BMesh** bm, bGPDstroke *gps, int inner_material_index)
{
	BMVert *lastv1 = NULL;
	BMVert *lastv2 = NULL;
	int p = 0;
	float thresh = (float)fmd->grease_decimate / 100.0f;
	float half[3] = {0, 0, 1};

	for (p = 0; p < gps->totpoints; p++) {

		if ((BLI_frand() < thresh) || (p == 0) || (p == gps->totpoints-1)) {
			BMVert *v1, *v2;
			float point[3] = {0, 0, 0};

			point[0] = gps->points[p].x;
			point[1] = gps->points[p].y;
			point[2] = gps->points[p].z;

			v1 = BM_vert_create(*bm, point, NULL, 0);

			if (lastv1)
			{
				BMFace* f;
				float nvec[3] = {0.0f, 0.0f, 0.0f}, co1[3], co2[3];

				/*also "extrude" this along the normal, no...use global axises instead*/
				if (fmd->cutter_axis == MOD_FRACTURE_CUTTER_X)
				{
					nvec[0] = 1.0f;
					nvec[1] = 0.0f;
					nvec[2] = 0.0f;
				}

				if (fmd->cutter_axis == MOD_FRACTURE_CUTTER_Y)
				{
					nvec[0] = 0.0f;
					nvec[1] = 1.0f;
					nvec[2] = 0.0f;
				}

				if (fmd->cutter_axis == MOD_FRACTURE_CUTTER_Z)
				{
					nvec[0] = 0.0f;
					nvec[1] = 0.0f;
					nvec[2] = 1.0f;
				}

				mul_v3_fl(nvec, fmd->grease_offset);
				mul_v3_v3fl(half, nvec, 0.5f);

				add_v3_v3v3(co1, v1->co, nvec);
				v2 = BM_vert_create(*bm, co1, NULL, 0);

				if (!lastv2)
				{
					add_v3_v3v3(co2, lastv1->co, nvec);
					lastv2 = BM_vert_create(*bm, co2, NULL, 0);
				}

				f = BM_face_create_quad_tri(*bm, lastv1, v1, v2, lastv2, NULL, 0);
				f->mat_nr = inner_material_index;
				lastv2 = v2;
			}

			lastv1 = v1;
		}
	}

	{
		/* move the stroke mesh a bit out, half of offset */
		BMIter iter;
		BMVert *v;

		BM_ITER_MESH(v, &iter, *bm, BM_VERTS_OF_MESH)
		{
			sub_v3_v3(v->co, half);
		}

		BM_mesh_elem_hflag_enable_all(*bm, BM_FACE | BM_EDGE | BM_VERT, BM_ELEM_SELECT, false);
		BMO_op_callf(*bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		             "remove_doubles verts=%av dist=%f", BM_VERTS_OF_MESH, 0.01, false);
	}
}

static void add_vgroup(Shard *s, Object *ob, const char* name)
{
	int index = 0, i = 0;
	MDeformVert *dvert;
	if (!defgroup_find_name(ob, name)) {
		BKE_defgroup_new(ob, name);
	}
	index = defgroup_name_index(ob, name);
	dvert = CustomData_get_layer(&s->vertData, CD_MDEFORMVERT);
	if (dvert == NULL) {
		dvert = CustomData_add_layer(&s->vertData, CD_MDEFORMVERT, CD_CALLOC, NULL, s->totvert);
	}
	for (i = 0; i < s->totvert; i++) {
		MDeformVert* dv = dvert + i;
		defvert_add_index_notest(dv, index, 1.0f);
	}
}

static void add_material(Shard* s, Object *ob, short mat_ofs) {

	/* only use material offsets if we have 3 or more materials; since FM material handling still is a bit odd  */
	/* todo, use index for inner material too, then this hack here isnt necessary any more */

	const short mat_nr_max = ob->totcol > 1 ? ob->totcol - 1 : 0;
	mat_ofs = mat_nr_max ? mat_ofs : 0;

	if (mat_ofs) {
		MPoly *mp;
		int i = 0;

		for (i = 0; i < s->totpoly; i++)
		{
			mp = s->mpoly + i;
			mp->mat_nr = mat_ofs;

			//hrm first material and second (inner) should be untouched...
			CLAMP(mp->mat_nr, 0, mat_nr_max);
		}
	}
}

static void do_intersect(FractureModifierData *fmd, Object* ob, Shard *t, short inner_mat_index,
                         bool is_zero, float mat[4][4], int **shard_counts, int* count,
                         int k, DerivedMesh **dm_parent, bool keep_other_shard, int solver, float thresh)
{
	/*just keep appending items at the end here */
	MPoly *mpoly, *mp;
	int totpoly;
	Shard *parent = NULL;
	Shard *s = NULL, *s2 = NULL;
	int shards = 0, j = 0;

	if (is_zero == false && *dm_parent == NULL) {
		parent = BLI_findlink(&fmd->frac_mesh->shard_map, k);
		*dm_parent = BKE_shard_create_dm(parent, true);
	}

	mpoly = (*dm_parent)->getPolyArray(*dm_parent);
	totpoly = (*dm_parent)->getNumPolys(*dm_parent);

	for (j = 0, mp = mpoly; j < totpoly; j++, mp++) {
		mp->flag &= ~ME_FACE_SEL;
	}

	if (keep_other_shard)
	{
		s = BKE_fracture_shard_boolean(ob, *dm_parent, t, inner_mat_index, 0, 0.0f, &s2, NULL, 0.0f, false, 0, fmd->uvlayer_name, solver, thresh);
	}
	else
	{
		s = BKE_fracture_shard_boolean(ob, *dm_parent, t, inner_mat_index, 0, 0.0f, NULL, NULL, 0.0f, false, 0, fmd->uvlayer_name, solver, thresh);
	}

	//printf("Fractured: %d\n", k);

	if (ELEM(fmd->keep_cutter_shards, MOD_FRACTURE_KEEP_BOTH, MOD_FRACTURE_KEEP_INTERSECT)) {
		if (s != NULL) {
			add_vgroup(s, ob, "Intersect");
			add_material(s, ob, fmd->mat_ofs_intersect);
			add_shard(fmd->frac_mesh, s, mat);
			shards++;
			s = NULL;
		}
	}

	if (ELEM(fmd->keep_cutter_shards, MOD_FRACTURE_KEEP_BOTH, MOD_FRACTURE_KEEP_DIFFERENCE)) {
		if (s2 != NULL) {
			add_vgroup(s2, ob, "Difference");
			add_material(s2, ob, fmd->mat_ofs_difference);
			add_shard(fmd->frac_mesh, s2, mat);
			shards++;
			s2 = NULL;
		}
	}

	//if ((is_zero && ob->derivedFinal == NULL) || !is_zero) {
	{
		if (is_zero) {
			*count = 0;
		}

		(*dm_parent)->needsFree = 1;
		(*dm_parent)->release(*dm_parent);
		*dm_parent = NULL;
	}

	if (is_zero) {
		shards = 0;
	}

	(*shard_counts)[k] = shards;
	//printf("k, shards: %d %d \n", k, shards);
	//shards = 0;
}



static void intersect_shards_by_dm(FractureModifierData *fmd, DerivedMesh *d, Object *ob, Object *ob2, short inner_mat_index, float mat[4][4],
                                   bool keep_other_shard, int solver, float thresh)
{
	Shard *t = NULL;
	int i = 0, count = 0, k = 0;
	float imat[4][4];
	int* shard_counts = NULL;
	bool is_zero = false;
	MVert *mv;
	DerivedMesh *dm_parent = NULL;

	t = BKE_create_fracture_shard(d->getVertArray(d), d->getPolyArray(d), d->getLoopArray(d), d->getEdgeArray(d),
	                              d->getNumVerts(d), d->getNumPolys(d), d->getNumLoops(d), d->getNumEdges(d), true);
	t = BKE_custom_data_to_shard(t, d);


	invert_m4_m4(imat, ob->obmat);
	for (i = 0, mv = t->mvert; i < t->totvert; mv++, i++){
		if (ob2)
			mul_m4_v3(ob2->obmat, mv->co);
		mul_m4_v3(imat, mv->co);
	}

	count = fmd->frac_mesh->shard_count;

	/*TODO, pass modifier mesh here !!! */
	if (count == 0 && keep_other_shard) {

		if (ob->derivedFinal != NULL) {
			dm_parent = CDDM_copy(ob->derivedFinal);
		}

		if (dm_parent == NULL) {
			dm_parent = CDDM_from_mesh(ob->data);
		}

		count = 1;
		is_zero = true;
	}

	shard_counts = MEM_mallocN(sizeof(int) * count, "shard_counts");

	for (k = 0; k < count; k++) {
		do_intersect(fmd, ob, t, inner_mat_index, is_zero, mat, &shard_counts, &count, k, &dm_parent, keep_other_shard, solver, thresh);
	}

	for (k = 0; k < count; k++)
	{
		int cnt = shard_counts[k];

		if (cnt > 0)
		{
			if (keep_other_shard)
			{
				/*clean up old entries here to avoid unnecessary shards*/
				Shard *first = fmd->frac_mesh->shard_map.first;
				BLI_remlink_safe(&fmd->frac_mesh->shard_map,first);
				BKE_shard_free(first, true);
				first = NULL;
			}

			/* keep asynchronous by intent, to keep track of original shard count */
			fmd->frac_mesh->shard_count--;
		}
	}

	MEM_freeN(shard_counts);
	shard_counts = NULL;

	BKE_shard_free(t, true);
}

static void reset_shards(FractureModifierData *fmd)
{
	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED && fmd->reset_shards)
	{
		FracMesh *fm = fmd->frac_mesh;
		while (fm && fm->shard_map.first)
		{
			Shard *t = fm->shard_map.first;
			BLI_remlink_safe(&fm->shard_map, t);
			printf("Resetting shard (Greasepencil/Cutter Plane): %d\n", t->shard_id);
			BKE_shard_free(t, true);
		}
		fm->shard_count = 0;
		/* do not reset again afterwards, in case we have multiple point sources */
		if (!fmd->execute_threaded) {
			fmd->reset_shards = false;
		}
	}
}

void BKE_fracture_shard_by_greasepencil(FractureModifierData *fmd, Object *obj, short inner_material_index, float mat[4][4])
{
	bGPDlayer *gpl;
	bGPDframe *gpf;
	bGPDstroke *gps;

	reset_shards(fmd);

	if ((obj->gpd) && (obj->gpd->layers.first)) {

		float imat[4][4];
		invert_m4_m4(imat, mat);
		for (gpl = obj->gpd->layers.first; gpl; gpl = gpl->next) {
			for (gpf = gpl->frames.first; gpf; gpf = gpf->next) {
				for (gps = gpf->strokes.first; gps; gps = gps->next) {
					BMesh *bm = BM_mesh_create(&bm_mesh_allocsize_default, &((struct BMeshCreateParams){.use_toolflags = true,}));
					DerivedMesh *dm = NULL;


					/*create stroke mesh */
					stroke_to_faces(fmd, &bm, gps, inner_material_index);
					dm = CDDM_from_bmesh(bm, true);
#if 0
					{
						/*create debug mesh*/
						Object* o;
						o = BKE_object_add(G.main, fmd->modifier.scene, OB_MESH, "DUMMY");
						BM_mesh_bm_to_me(bm, o->data, (&(struct BMeshToMeshParams){0}));
					}
#endif

					BM_mesh_free(bm);

					/*do intersection*/
					intersect_shards_by_dm(fmd, dm, obj, NULL, inner_material_index, mat, true, fmd->boolean_solver, fmd->boolean_double_threshold);

					dm->needsFree = 1;
					dm->release(dm);
					dm = NULL;
				}
			}
		}
	}
}

void BKE_fracture_shard_by_planes(FractureModifierData *fmd, Object *obj, short inner_material_index, float mat[4][4])
{
	if (fmd->cutter_group != NULL && obj->type == OB_MESH)
	{
		GroupObject* go;
		float imat[4][4];

		reset_shards(fmd);
		invert_m4_m4(imat, obj->obmat);

		for (go = fmd->cutter_group->gobject.first; go; go = go->next)
		{
			Object* ob = go->ob;

			printf("Cutting with %s ...\n", ob->id.name);
			/*simple case....one cutter object per object*/
			if (ob->type == OB_MESH) {

				FractureModifierData *fmd2 = (FractureModifierData*)modifiers_findByType(ob, eModifierType_Fracture);
				if (fmd2 && BLI_listbase_count(&fmd2->meshIslands) > 0)
				{
					MeshIsland* mi = NULL;
					int j = 0;
					for (mi = fmd2->meshIslands.first; mi; mi = mi->next)
					{
						DerivedMesh *dm = CDDM_copy(mi->physics_mesh);
						MVert *mv = dm->getVertArray(dm), *v = NULL;
						int totvert = dm->getNumVerts(dm);
						int i = 0;

						//printf("Cutting with %s, island %d...\n", ob->id.name, j);
						for (i = 0, v = mv; i < totvert; i++, v++)
						{
							add_v3_v3(v->co, mi->centroid);
						}

						intersect_shards_by_dm(fmd, dm, obj, ob, inner_material_index, mat, false, fmd->boolean_solver, fmd->boolean_double_threshold);

						dm->needsFree = 1;
						dm->release(dm);
						dm = NULL;
						j++;
					}

					/*now delete first shards, those are the old ones */
					while (fmd->frac_mesh->shard_count > 0)
					{
						/*clean up old entries here to avoid unnecessary shards*/
						Shard *first = fmd->frac_mesh->shard_map.first;
						BLI_remlink_safe(&fmd->frac_mesh->shard_map,first);
						BKE_shard_free(first, true);
						first = NULL;
						fmd->frac_mesh->shard_count--;
					}

					/* re-synchronize counts, was possibly different before */
					fmd->frac_mesh->shard_count = BLI_listbase_count(&fmd->frac_mesh->shard_map);
				}
				else
				{

					DerivedMesh *d;
					bool copied = false;
					if (ob->derivedFinal == NULL) {
						d = CDDM_from_mesh(ob->data);
						copied = true;
					}
					else {
						d = ob->derivedFinal;
					}

					intersect_shards_by_dm(fmd, d, obj, ob, inner_material_index, mat, true, fmd->boolean_solver, fmd->boolean_double_threshold);

					if (copied)
					{	/*was copied before */
						d->needsFree = 1;
						d->release(d);
						d = NULL;
					}
				}
			}
		}
	}
}

typedef struct FractureData {
	cell *voro_cells;
	FracMesh *fmesh;
	ShardID id;
	int totpoints;
	int algorithm;
	Object *obj;
	DerivedMesh *dm;
	short inner_material_index;
	float mat[4][4];
	int num_cuts;
	float fractal;
	bool smooth;
	int num_levels;
	int mode;
	bool reset;
	int active_setting;
	int num_settings;
	char uv_layer[64];
	int solver;
	float thresh;
	int override_count;
	float factor;
} FractureData;


static void compute_fracture(TaskPool *UNUSED(pool), void *taskdata, int UNUSED(threadid))
{
    FractureData *fd = (FractureData*)taskdata;

	/*Evaluate result*/
	if (fd->totpoints > 0) {
		parse_cells(fd->voro_cells, fd->totpoints, fd->id, fd->fmesh, fd->algorithm, fd->obj, fd->dm, fd->inner_material_index, fd->mat,
	                fd->num_cuts, fd->fractal, fd->smooth, fd->num_levels,fd->mode, fd->reset, fd->active_setting, fd->num_settings, fd->uv_layer,
		            true, fd->solver, fd->thresh, fd->override_count, fd->factor);
	}
}


static FractureData segment_cells(cell *voro_cells, int startcell, int totcells, FracMesh *fmesh, ShardID id, FracPointCloud *pointcloud,
                    int algorithm, Object *obj, DerivedMesh *dm, short
                    inner_material_index, float mat[4][4], int num_cuts, float fractal, bool smooth, int num_levels, int mode,
                    bool reset, int active_setting, int num_settings, char uv_layer[64], int solver, float thresh, int override_count,
                    float factor)
{
	FractureData fd;
	fd.fmesh = fmesh;
	fd.id = id;
	fd.totpoints = totcells;
	fd.algorithm = algorithm;
	fd.obj = obj;
	fd.dm = dm;
	fd.inner_material_index = inner_material_index;
	copy_m4_m4(fd.mat, mat);
	fd.num_cuts = num_cuts;
	fd.fractal = fractal;
	fd.smooth = smooth;
	fd.num_levels = num_levels;
	fd.mode = mode;
	fd.reset = reset;
	fd.active_setting = active_setting;
	fd.num_settings = num_settings;
	strncpy(fd.uv_layer, uv_layer, 64);
	fd.solver = solver;
	fd.thresh = thresh;
	fd.override_count = override_count;
	fd.factor = factor;

	//cell start pointer, only take fd.totpoints cells out
	fd.voro_cells = voro_cells + startcell;

	return fd;
}

void BKE_fracture_shard_by_points(FracMesh *fmesh, ShardID id, FracPointCloud *pointcloud, int algorithm, Object *obj, DerivedMesh *dm, short
                                  inner_material_index, float mat[4][4], int num_cuts, float fractal, bool smooth, int num_levels, int mode,
                                  bool reset, int active_setting, int num_settings, char uv_layer[64], bool threaded, int solver,
                                  float thresh, bool shards_to_islands, int override_count, float factor, int point_source,
                                  int resolution[3], float spacing[3])
{
	int n_size = 8;
	
	Shard *shard;
	
	float min[3], max[3];
	float theta = 0.001f; /* TODO, container enlargement, because boundbox exact container and boolean might create artifacts */
	int p, i = 0, num = 0, totcell = 0, remainder_start = 0;
	
	container *voro_container;
	particle_order *voro_particle_order;
	cell *voro_cells;

	TaskScheduler *scheduler = BLI_task_scheduler_get();
	TaskPool *pool;
	FractureData *fdata;

#ifdef USE_DEBUG_TIMER
	double time_start;
#endif
	
	shard = BKE_shard_by_id(fmesh, id, dm);
	if (!shard || (shard->flag & SHARD_FRACTURED && (mode == MOD_FRACTURE_DYNAMIC))) {
		int val = shards_to_islands ? -1 : 0;
		if (id == val)
		{
			//fallback to entire mesh
			shard = BKE_shard_by_id(fmesh, -1 , dm);
		}
		else
		{
			return;
		}
	}

	printf("Fracturing with %d points...\n", pointcloud->totpoints);
	/* calculate bounding box with theta margin */
	copy_v3_v3(min, shard->min);
	copy_v3_v3(max, shard->max);

	if (shard->shard_id == -2) {
		BKE_shard_free(shard, true);
	}

	add_v3_fl(min, -theta);
	add_v3_fl(max, theta);

	mul_m4_v3(mat, min);
	mul_m4_v3(mat, max);

	
	if (point_source & MOD_FRACTURE_GRID)
	{
		float off[3] =  {0, 0, 0};
		for (p = 0; p < pointcloud->totpoints; p++)
		{
			//find "max" offset (where atleast 1 axis is > 0)
			if (pointcloud->points[p].offset[0] > 0 ||
			    pointcloud->points[p].offset[1] > 0 ||
			    pointcloud->points[p].offset[2] > 0)
			{
				copy_v3_v3(off, pointcloud->points[p].offset);
				break;
			}
		}

		if (off[0] > 0 || off[1] > 0 || off[2] > 0)
		{
			sub_v3_v3(min, off);

			//special treatment for grid pointsource... with offsets
			voro_container = container_new(min[0], max[0], min[1], max[1], min[2], max[2],
										   resolution[0] * 2, resolution[1] * 2, resolution[2] * 2, false, false, false,
										   pointcloud->totpoints);

		}
		else {
			voro_container = container_new(min[0], max[0], min[1], max[1], min[2], max[2],
										   n_size, n_size, n_size, false, false, false,
										   pointcloud->totpoints);
		}
	}
	else {
		voro_container = container_new(min[0], max[0], min[1], max[1], min[2], max[2],
									   n_size, n_size, n_size, false, false, false,
									   pointcloud->totpoints);
	}
	
	voro_particle_order = particle_order_new();
	for (p = 0; p < pointcloud->totpoints; p++) {
		float *co = pointcloud->points[p].co;
		container_put(voro_container, voro_particle_order, p, co[0], co[1], co[2]);
	}

#ifdef USE_DEBUG_TIMER
	time_start = PIL_check_seconds_timer();
#endif

	/* we expect as many raw cells as we have particles */
	voro_cells = cells_new(pointcloud->totpoints);

	/*Compute directly...*/
	container_compute_cells(voro_container, voro_cells);

	/*Apply offsets (if any, grid only */
	if (point_source & MOD_FRACTURE_GRID)
	{
		int v = 0;
		float fact[3] = {1 - spacing[0], 1 - spacing[1], 1 - spacing[2]};
		for (p = 0; p < pointcloud->totpoints; p++)
		{
			//adjust centroid and...
			float off[3], cent[3];
			copy_v3_v3(off, pointcloud->points[p].offset);
			add_v3_v3(voro_cells[p].centroid, off);
			copy_v3_v3(cent, voro_cells[p].centroid);
			mul_v3_v3(cent, fact);

			//vertex coordinates
			for (v = 0; v < voro_cells[p].totvert; v++)
			{
				add_v3_v3(voro_cells[p].verts[v], off);
				//print_v3("Vert", voro_cells[p].verts[v]);
				sub_v3_v3(voro_cells[p].verts[v], cent);
				add_v3_v3(voro_cells[p].verts[v], voro_cells[p].centroid);
			}
		}
	}

	/*Disable for fast bisect/fill, dynamic and mousebased for now -> errors and crashes */
	if (mode != MOD_FRACTURE_DYNAMIC && reset == true && algorithm != MOD_FRACTURE_BISECT_FAST && algorithm != MOD_FRACTURE_BISECT_FAST_FILL && threaded == true) {
		/*segment cells, give each thread a chunk to work on */
		pool = BLI_task_pool_create(scheduler, NULL);
		num = BLI_task_scheduler_num_threads(scheduler);
		fdata = MEM_callocN(sizeof(FractureData) * (num + 1), "threaded fracture data");
		totcell = pointcloud->totpoints / num;
		for (i = 0; i < num; i++) {
			//give each task a segment of the shards...
			int startcell = i * totcell;
			FractureData fd = segment_cells(voro_cells, startcell, totcell, fmesh, id, pointcloud, algorithm, obj, dm, inner_material_index,
											mat, num_cuts, fractal,smooth, num_levels, mode, reset, active_setting, num_settings, uv_layer,
											solver, thresh, override_count, factor);
			fdata[i] = fd;
		}

		//remainder...
		remainder_start = fdata[0].totpoints * num;
		if (remainder_start < pointcloud->totpoints) {
			int remainder = pointcloud->totpoints - remainder_start;
			int startcell = remainder_start;
			printf("REMAINDER %d %d\n", startcell, remainder);
			fdata[num] = segment_cells(voro_cells, startcell, remainder, fmesh, id, pointcloud, algorithm, obj, dm, inner_material_index,
										mat, num_cuts, fractal,smooth, num_levels, mode, reset, active_setting, num_settings, uv_layer,
										solver, thresh, override_count, factor);
		}

		for (i = 0; i < num+1; i++) {
			BLI_task_pool_push(pool, compute_fracture, &fdata[i], false, TASK_PRIORITY_HIGH);
		}

		BLI_task_pool_work_and_wait(pool);
		BLI_task_pool_free(pool);
		MEM_freeN(fdata);
	}
	else {
		/*Evaluate result*/
		parse_cells(voro_cells, pointcloud->totpoints, id, fmesh, algorithm, obj, dm, inner_material_index, mat,
					num_cuts, fractal, smooth, num_levels, mode, reset, active_setting, num_settings, uv_layer,
					false, solver, thresh, override_count, factor);
	}

	/*Free structs in C++ area of memory */
	cells_free(voro_cells, pointcloud->totpoints);
	particle_order_free(voro_particle_order);
	container_free(voro_container);

#ifdef USE_DEBUG_TIMER
	printf("Fracture done, %g\n", PIL_check_seconds_timer() - time_start);
#endif

}

void BKE_fracmesh_free(FracMesh *fm, bool doCustomData)
{
	if (fm == NULL) {
		return;
	}

	while (fm->shard_map.first) {
		Shard* s = (Shard*)fm->shard_map.first;
		BLI_remlink(&fm->shard_map, s);
		BKE_shard_free(s, doCustomData);
	}

	if (fm->last_shard_tree)
	{
		BLI_kdtree_free(fm->last_shard_tree);
		fm->last_shard_tree = NULL;
	}

	if (fm->last_shards)
	{
		MEM_freeN(fm->last_shards);
		fm->last_shards = NULL;
	}
}


static void do_marking(FractureModifierData *fmd, DerivedMesh *result)
{
	MEdge *medge = result->getEdgeArray(result);
	MPoly *mpoly = result->getPolyArray(result), *mp = NULL;
	MLoop *mloop = result->getLoopArray(result);
	MVert *mvert = result->getVertArray(result);
	int totpoly = result->getNumPolys(result);
	int i = 0;
	for (i = 0, mp = mpoly; i < totpoly; i++, mp++)
	{
		if (mp->flag & ME_FACE_SEL)
		{
			int j = 0;
			for (j = 0; j < mp->totloop; j++)
			{
				MLoop ml;
				ml = mloop[mp->loopstart + j];
				medge[ml.e].flag |= ME_SHARP;
				medge[ml.e].crease = fmd->inner_crease * 255.0f;
				mvert[ml.v].flag |= ME_VERT_TMP_TAG;
			}

			if (fmd->use_smooth)
				mp->flag |= ME_SMOOTH;
		}
		else
		{
			/*remove verts from unselected faces again*/
			int j = 0;
			for (j = 0; j < mp->totloop; j++)
			{
				MLoop ml;
				ml = mloop[mp->loopstart + j];
				mvert[ml.v].flag &= ~ME_VERT_TMP_TAG;
			}
		}
	}
}

static DerivedMesh* do_create(FractureModifierData *fmd, int num_verts, int num_loops, int num_polys, int num_edges,
                              bool doCustomData, bool use_packed)
{
	int shard_count = fmd->shards_to_islands ? BLI_listbase_count(&fmd->islandShards) : fmd->frac_mesh->shard_count;
	ListBase *shardlist;
	Shard *shard;

	int vertstart, polystart, loopstart, edgestart;

	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;
	MEdge *medges;

	DerivedMesh *result = CDDM_new(num_verts, num_edges, 0, num_loops, num_polys);
	mverts = CDDM_get_verts(result);
	mloops = CDDM_get_loops(result);
	mpolys = CDDM_get_polys(result);

	if (num_edges > 0) {
		medges = CDDM_get_edges(result);
	}

#if 0
	if (doCustomData && shard_count > 0) {

		Shard *s;

		if (fmd->shards_to_islands) {
			s = (Shard *)fmd->islandShards.first;
		}
		else {
			s = (Shard *)fmd->frac_mesh->shard_map.first;
		}

		//if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED || fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			/*keep old behavior for now for older modes */
			CustomData_merge(&s->vertData, &result->vertData, CD_MASK_MDEFORMVERT, CD_CALLOC, num_verts);
			CustomData_merge(&s->polyData, &result->polyData, CD_MASK_MTEXPOLY, CD_CALLOC, num_polys);
			CustomData_merge(&s->loopData, &result->loopData, CD_MASK_MLOOPUV, CD_CALLOC, num_loops);
			CustomData_merge(&s->edgeData, &result->edgeData, CD_MASK_CREASE | CD_MASK_BWEIGHT | CD_MASK_MEDGE, CD_CALLOC, num_edges);
		}
		else
		{
			/*just create new empty layers */
			CustomData_add_layer(&result->vertData, CD_MDEFORMVERT, CD_CALLOC, NULL, num_verts);
			CustomData_add_layer(&result->polyData, CD_MTEXPOLY, CD_CALLOC, NULL, num_polys);
			CustomData_add_layer(&result->loopData, CD_MLOOPUV, CD_CALLOC, NULL, num_loops);
			CustomData_add_layer(&result->edgeData, CD_CREASE, CD_CALLOC, NULL, num_edges);
			CustomData_add_layer(&result->edgeData, CD_BWEIGHT, CD_CALLOC, NULL, num_edges);
		}
	}
#endif

	vertstart = polystart = loopstart = edgestart = 0;
	if (use_packed)
	{
		shardlist = &fmd->pack_storage;
	}
	else if (fmd->shards_to_islands) {
		shardlist = &fmd->islandShards;
	}
	else {
		shardlist = &fmd->frac_mesh->shard_map;
	}

	for (shard = shardlist->first; shard; shard = shard->next)
	{
		MPoly *mp;
		MLoop *ml;
		MEdge *me;
		int i;

		memcpy(mverts + vertstart, shard->mvert, shard->totvert * sizeof(MVert));
		memcpy(mpolys + polystart, shard->mpoly, shard->totpoly * sizeof(MPoly));

		for (i = 0, mp = mpolys + polystart; i < shard->totpoly; ++i, ++mp) {
			/* adjust loopstart index */
			mp->loopstart += loopstart;
		}

		memcpy(mloops + loopstart, shard->mloop, shard->totloop * sizeof(MLoop));

		for (i = 0, ml = mloops + loopstart; i < shard->totloop; ++i, ++ml) {
			/* adjust vertex index */
			ml->v += vertstart;
			ml->e += edgestart;
		}

		if (num_edges > 0) {
			memcpy(medges + edgestart, shard->medge, shard->totedge * sizeof(MEdge));

			for (i = 0, me = medges + edgestart; i < shard->totedge; ++i, ++me) {
				/* adjust vertex indices */
				me->v1 += vertstart;
				me->v2 += vertstart;
			}
		}

#if 0
		if (doCustomData) {

			if (shard->totvert > 1) {
				CustomData_copy_data(&shard->vertData, &result->vertData, 0, vertstart, shard->totvert);
			}

			if (shard->totloop > 0) {
				CustomData_copy_data(&shard->loopData, &result->loopData, 0, loopstart, shard->totloop);
			}

			if (shard->totpoly > 0) {
				CustomData_copy_data(&shard->polyData, &result->polyData, 0, polystart, shard->totpoly);
			}
		}
#endif

		if (doCustomData) {
			fracture_collect_layers(shard, result, vertstart, polystart, loopstart, edgestart);
		}

		vertstart += shard->totvert;
		polystart += shard->totpoly;
		loopstart += shard->totloop;
		edgestart += shard->totedge;
	}

	return result;
}

/* DerivedMesh */
static DerivedMesh *create_dm(FractureModifierData *fmd, bool doCustomData, bool use_packed)
{
	Shard *s;
	int num_verts, num_polys, num_loops, num_edges;
	DerivedMesh *result;
	
	num_verts = num_polys = num_loops = num_edges = 0;

	if (use_packed)
	{
		for (s = fmd->pack_storage.first; s; s = s->next) {
			num_verts += s->totvert;
			num_polys += s->totpoly;
			num_loops += s->totloop;
			num_edges += s->totedge;
		}
	}
	else if (fmd->shards_to_islands) {
		for (s = fmd->islandShards.first; s; s = s->next) {
			num_verts += s->totvert;
			num_polys += s->totpoly;
			num_loops += s->totloop;
			num_edges += s->totedge;
		}
	}
	else {

		if (!fmd->frac_mesh)
			return NULL;

		for (s = fmd->frac_mesh->shard_map.first; s; s = s->next) {
			num_verts += s->totvert;
			num_polys += s->totpoly;
			num_loops += s->totloop;
			num_edges += s->totedge;
		}
	}
	
	result = do_create(fmd, num_verts, num_loops, num_polys, num_edges, doCustomData, use_packed);

	if (num_edges == 0) {
		CustomData_free(&result->edgeData, 0);
		CDDM_calc_edges(result);
	}

	do_marking(fmd, result);
	
	result->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(result);
	return result;
}

DerivedMesh* BKE_fracture_create_dm(FractureModifierData *fmd, bool doCustomData, bool use_packed)
{
	DerivedMesh *dm_final = NULL;
	
	if (fmd->dm && !use_packed) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
	
	dm_final = create_dm(fmd, doCustomData, use_packed);

	if (!use_packed) {
		fmd->dm = dm_final;
	}

	return dm_final;
}

void BKE_copy_customdata_layers(CustomData* dest, CustomData *src, int type, int count)
{
	int layer;
	for (layer = 0; layer < src->totlayer; layer++)
	{
		if (src->layers[layer].type == type)
		{
			CustomData_add_layer(dest, type, CD_DUPLICATE, src->layers[layer].data, count);
		}
	}
}

DerivedMesh *BKE_shard_create_dm(Shard *s, bool doCustomData)
{
	DerivedMesh *dm;
	MVert *mverts;
	MLoop *mloops;
	MPoly *mpolys;
	
	dm = CDDM_new(s->totvert, s->totedge, 0, s->totloop, s->totpoly);

	mverts = CDDM_get_verts(dm);
	mloops = CDDM_get_loops(dm);
	mpolys = CDDM_get_polys(dm);

	memcpy(mverts, s->mvert, s->totvert * sizeof(MVert));
	memcpy(mloops, s->mloop, s->totloop * sizeof(MLoop));
	memcpy(mpolys, s->mpoly, s->totpoly * sizeof(MPoly));

	if (s->totedge > 0) {
		MEdge *medges = CDDM_get_edges(dm);
		memcpy(medges, s->medge, s->totedge * sizeof(MEdge));
	}
	else {
		CustomData_free(&dm->edgeData, 0);
		CDDM_calc_edges(dm);
	}

	dm->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(dm);

	if (doCustomData) {
		/*if (s->totvert > 0) {
			BKE_copy_customdata_layers(&dm->vertData, &s->vertData, CD_MDEFORMVERT, s->totvert);
		}
		if (s->totloop > 0) {
			BKE_copy_customdata_layers(&dm->loopData, &s->loopData, CD_MLOOPUV, s->totloop);
		}
		if (s->totpoly > 0) {
			BKE_copy_customdata_layers(&dm->polyData, &s->polyData, CD_MTEXPOLY, s->totpoly);
		}*/

		fracture_collect_layers(s, dm, 0, 0, 0, 0);
	}

	return dm;
}

void BKE_get_next_entries(FractureModifierData *fmd)
{
	/*meshislands and shards SHOULD be synchronized !!!!*/
	if (fmd->current_mi_entry && fmd->current_mi_entry->next)
	{ // && fmd->current_mi_entry->next->is_new == false) {
		fmd->current_mi_entry = fmd->current_mi_entry->next;
		fmd->meshIslands = fmd->current_mi_entry->meshIslands;
		fmd->visible_mesh_cached = fmd->current_mi_entry->visible_dm;
	}

	if (fmd->current_shard_entry && fmd->current_shard_entry->next)
	{
		fmd->current_shard_entry = fmd->current_shard_entry->next;
		fmd->frac_mesh = fmd->current_shard_entry->frac_mesh;
	}
}

void BKE_get_prev_entries(FractureModifierData *fmd)
{
	/*meshislands and shards SHOULD be synchronized !!!!*/
	if (fmd->current_mi_entry && fmd->current_mi_entry->prev)
	{
		fmd->current_mi_entry = fmd->current_mi_entry->prev;
		fmd->meshIslands = fmd->current_mi_entry->meshIslands;
		fmd->visible_mesh_cached = fmd->current_mi_entry->visible_dm;
	}

	if (fmd->current_shard_entry && fmd->current_shard_entry->prev)
	{
		fmd->current_shard_entry = fmd->current_shard_entry->prev;
		fmd->frac_mesh = fmd->current_shard_entry->frac_mesh;
	}
}

bool BKE_lookup_mesh_state(FractureModifierData *fmd, int frame, int do_lookup)
{
	bool changed = false;
	bool forward = false;
	bool backward = false;

	backward = ((fmd->last_frame > frame) && fmd->current_mi_entry && fmd->current_mi_entry->prev);
	forward = ((fmd->last_frame < frame) && (fmd->current_mi_entry) && (fmd->current_mi_entry->next != NULL) &&
	           (fmd->current_mi_entry->is_new == false));

	if (backward)
	{
		if (do_lookup)
		{
			while (fmd->current_mi_entry && fmd->current_mi_entry->prev &&
				   frame <= fmd->current_mi_entry->prev->frame)
			{
				printf("Jumping backward because %d is smaller than %d\n", frame, fmd->current_mi_entry->prev->frame);
				changed = true;
				//BKE_free_constraints(fmd);
				BKE_get_prev_entries(fmd);
			}
		}
	}
	else if (forward)
	{
		if (do_lookup)
		{
			while ((fmd->current_mi_entry) && (fmd->current_mi_entry->next != NULL) &&
				   (fmd->current_mi_entry->is_new == false) &&
				   frame > fmd->current_mi_entry->frame)
			{
				printf("Jumping forward because %d is greater than %d\n", frame, fmd->current_mi_entry->frame);
				changed = true;
				//BKE_free_constraints(fmd);
				BKE_get_next_entries(fmd);
			}
		}
	}

	if (do_lookup)
	{
		return changed;
	}
	else
	{
		if (forward || backward)
		{
			fmd->modifier.scene->rigidbody_world->flag |= RBW_FLAG_REFRESH_MODIFIERS;
			fmd->modifier.scene->rigidbody_world->flag |= RBW_FLAG_OBJECT_CHANGED;
		}

		return forward || backward;
	}
}

void BKE_match_vertex_coords(MeshIsland* mi, MeshIsland *par, Object *ob, int frame, bool is_parent, bool shards_to_islands)
{
	float loc[3] = {0.0f, 0.0f, 0.0f};
	float rot[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	int j = 0;

	float centr[3] = {0.0f, 0.0f, 0.0f};

	float mat[4][4];
	float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float qrot[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float iquat[4] =  {1.0f, 0.0f, 0.0f, 0.0f};

	int val = shards_to_islands ? -1 : 0;

	invert_m4_m4(mat, ob->obmat);

	mi->locs[0] = loc[0] = par->locs[3*frame];
	mi->locs[1] = loc[1] = par->locs[3*frame+1];
	mi->locs[2] = loc[2] = par->locs[3*frame+2];

	mi->rots[0] = rot[0] = par->rots[4*frame];
	mi->rots[1] = rot[1] = par->rots[4*frame+1];
	mi->rots[2] = rot[2] = par->rots[4*frame+2];
	mi->rots[3] = rot[3] = par->rots[4*frame+3];

	mul_m4_v3(mat, loc);
	mat4_to_quat(quat, ob->obmat);
	invert_qt_qt(iquat, quat);

	if (par->id == val)
	{
		invert_qt_qt(qrot, par->rot);
		mul_qt_qtqt(qrot, rot, qrot);
		mul_qt_qtqt(qrot, iquat, qrot);
	}
	else
	{
		mul_qt_qtqt(qrot, rot, par->rot);
		mul_qt_qtqt(qrot, iquat, qrot);
	}

	if (is_parent)
	{
		copy_v3_v3(centr, mi->centroid);
		mul_qt_v3(qrot, centr);
		add_v3_v3(centr, loc);
	}
	else
	{
		copy_v3_v3(centr, loc);
	}

	for (j = 0; j < mi->vertex_count; j++)
	{
		float co[3];

		//first add vert to centroid, then rotate
		copy_v3_v3(co, mi->vertices_cached[j]->co);

		sub_v3_v3(co, mi->centroid);
		mul_qt_v3(qrot, co);
		add_v3_v3(co, centr);

		copy_v3_v3(mi->vertices_cached[j]->co, co);

		mi->vertco[3*j]   = co[0];
		mi->vertco[3*j+1] = co[1];
		mi->vertco[3*j+2] = co[2];
	}

	{
		DerivedMesh *dm = mi->physics_mesh;
		MVert* mv, *mvert = dm->getVertArray(dm);
		int numVert = dm->getNumVerts(dm);

		for (j = 0, mv = mvert; j < numVert; j++, mv++)
		{
			//also rotate physicsmesh (shouldnt be necessary,
			//but lets do it to check whether its correct then)
			mul_qt_v3(qrot, mv->co);
		}
	}

	//init rigidbody properly ?
	copy_v3_v3(mi->centroid, centr);
	copy_qt_qt(mi->rot, qrot);
}

void BKE_free_constraints(FractureModifierData *fmd)
{
	MeshIsland *mi = NULL;
	RigidBodyShardCon *rbsc = NULL;

	//hmm after loading the pointers might be out of sync...
	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC) {
		if (fmd->current_mi_entry) {
			fmd->meshIslands = fmd->current_mi_entry->meshIslands;
		}
		else {
			fmd->meshIslands.first = NULL;
			fmd->meshIslands.last = NULL;
		}
	}



	for (mi = fmd->meshIslands.first; mi; mi = mi->next) {
		if (mi->participating_constraints != NULL && mi->participating_constraint_count > 0) {
			int i;
			for (i = 0; i < mi->participating_constraint_count; i++)
			{
				RigidBodyShardCon *con = mi->participating_constraints[i];
				if (con) {
					con->mi1 = NULL;
					con->mi2 = NULL;
				}
			}

			MEM_freeN(mi->participating_constraints);
			mi->participating_constraints = NULL;
			mi->participating_constraint_count = 0;
		}
	}

	while (fmd->meshConstraints.first) {
		rbsc = fmd->meshConstraints.first;
		BLI_remlink(&fmd->meshConstraints, rbsc);

		if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && fmd->modifier.scene)
		{
			BKE_rigidbody_remove_shard_con(fmd->modifier.scene, rbsc);
		}
		MEM_freeN(rbsc);
		rbsc = NULL;
	}

	fmd->meshConstraints.first = NULL;
	fmd->meshConstraints.last = NULL;
}

void BKE_fracture_load_settings(FractureModifierData *fmd, FractureSetting *fs)
{
	/*copy settings values to FM itself....*/

	/* vgroups  XXX TODO non ascii strings ?*/
	strncpy(fmd->thresh_defgrp_name, fs->thresh_defgrp_name, strlen(fs->thresh_defgrp_name));
	strncpy(fmd->ground_defgrp_name, fs->ground_defgrp_name, strlen(fs->ground_defgrp_name));
	strncpy(fmd->inner_defgrp_name, fs->inner_defgrp_name, strlen(fs->inner_defgrp_name));

	fmd->inner_material = fs->inner_material;
	fmd->extra_group = fs->extra_group;
	fmd->cluster_group = fs->cluster_group;
	fmd->cutter_group = fs->cutter_group;

	fmd->breaking_threshold = fs->breaking_threshold;
	fmd->use_constraints = fs->use_constraints;
	fmd->contact_dist = fs->contact_dist;
	fmd->use_mass_dependent_thresholds = fs->use_mass_dependent_thresholds;

	fmd->constraint_limit = fs->constraint_limit;
	fmd->breaking_angle = fs->breaking_angle;
	fmd->breaking_distance = fs->breaking_distance;
	fmd->breaking_percentage = fs->breaking_percentage;
	fmd->use_experimental = fs->use_experimental;

	fmd->cluster_count = fs->cluster_count;
	fmd->cluster_breaking_threshold = fs->cluster_breaking_threshold;
	fmd->solver_iterations_override = fs->solver_iterations_override;
	fmd->shards_to_islands = fs->shards_to_islands;

	fmd->shard_count = fs->shard_count;
	fmd->frac_algorithm = fs->frac_algorithm;

	fmd->solver_iterations_override = fs->solver_iterations_override;

	fmd->breaking_angle_weighted = fs->breaking_angle_weighted;
	fmd->breaking_distance_weighted = fs->breaking_distance_weighted;
	fmd->breaking_percentage_weighted = fs->breaking_percentage_weighted;

	fmd->point_seed = fs->point_seed;
	fmd->point_source = fs->point_source;

	fmd->use_particle_birth_coordinates = fs->use_particle_birth_coordinates;
	fmd->splinter_axis = fs->splinter_axis;
	fmd->splinter_length = fs->splinter_length;
	fmd->cluster_solver_iterations_override = fs->cluster_solver_iterations_override;

	fmd->cluster_breaking_angle = fs->cluster_breaking_angle;
	fmd->cluster_breaking_distance = fs->cluster_breaking_distance;
	fmd->cluster_breaking_percentage = fs->cluster_breaking_percentage;

	fmd->use_breaking = fs->use_breaking;
	fmd->use_smooth = fs->use_smooth;
	fmd->fractal_cuts = fs->fractal_cuts;
	fmd->fractal_amount = fs->fractal_amount;

	fmd->grease_decimate = fs->grease_decimate;
	fmd->grease_offset = fs->grease_offset;
	fmd->use_greasepencil_edges = fs->use_greasepencil_edges;
	fmd->cutter_axis = fs->cutter_axis;

	// add more constraint types, as in special ones (3x generic and so on)
	fmd->cluster_constraint_type = fs->cluster_constraint_type;
	fmd->constraint_target = fs->constraint_target;

	fmd->impulse_dampening = fs->impulse_dampening;
	fmd->minimum_impulse = fs->minimum_impulse;
	fmd->directional_factor = fs->directional_factor;
	fmd->mass_threshold_factor = fs->mass_threshold_factor;
	fmd->use_compounds = fs->use_compounds;
}

void BKE_fracture_store_settings(FractureModifierData *fs, FractureSetting *fmd)
{
	//just invert fmd and fs here, same variables
	/*copy settings values to FM itself....*/

	/* vgroups  XXX TODO non ascii strings ?*/
	if (!fmd || !fs)
		return;

	strncpy(fmd->thresh_defgrp_name, fs->thresh_defgrp_name, strlen(fs->thresh_defgrp_name));
	strncpy(fmd->ground_defgrp_name, fs->ground_defgrp_name, strlen(fs->ground_defgrp_name));
	strncpy(fmd->inner_defgrp_name, fs->inner_defgrp_name, strlen(fs->inner_defgrp_name));

	fmd->inner_material = fs->inner_material;
	fmd->extra_group = fs->extra_group;
	fmd->cluster_group = fs->cluster_group;
	fmd->cutter_group = fs->cutter_group;

	fmd->breaking_threshold = fs->breaking_threshold;
	fmd->use_constraints = fs->use_constraints;
	fmd->contact_dist = fs->contact_dist;
	fmd->use_mass_dependent_thresholds = fs->use_mass_dependent_thresholds;

	fmd->constraint_limit = fs->constraint_limit;
	fmd->breaking_angle = fs->breaking_angle;
	fmd->breaking_distance = fs->breaking_distance;
	fmd->breaking_percentage = fs->breaking_percentage;
	fmd->use_experimental = fs->use_experimental;

	fmd->cluster_count = fs->cluster_count;
	fmd->cluster_breaking_threshold = fs->cluster_breaking_threshold;
	fmd->solver_iterations_override = fs->solver_iterations_override;
	fmd->shards_to_islands = fs->shards_to_islands;

	fmd->shard_count = fs->shard_count;
	fmd->frac_algorithm = fs->frac_algorithm;

	fmd->solver_iterations_override = fs->solver_iterations_override;

	fmd->breaking_angle_weighted = fs->breaking_angle_weighted;
	fmd->breaking_distance_weighted = fs->breaking_distance_weighted;
	fmd->breaking_percentage_weighted = fs->breaking_percentage_weighted;

	fmd->point_seed = fs->point_seed;
	fmd->point_source = fs->point_source;

	fmd->use_particle_birth_coordinates = fs->use_particle_birth_coordinates;
	fmd->splinter_length = fs->splinter_length;
	fmd->splinter_axis = fs->splinter_axis;
	fmd->cluster_solver_iterations_override = fs->cluster_solver_iterations_override;

	fmd->cluster_breaking_angle = fs->cluster_breaking_angle;
	fmd->cluster_breaking_distance = fs->cluster_breaking_distance;
	fmd->cluster_breaking_percentage = fs->cluster_breaking_percentage;

	fmd->use_breaking = fs->use_breaking;
	fmd->use_smooth = fs->use_smooth;
	fmd->fractal_cuts = fs->fractal_cuts;
	fmd->fractal_amount = fs->fractal_amount;

	fmd->grease_decimate = fs->grease_decimate;
	fmd->grease_offset = fs->grease_offset;
	fmd->use_greasepencil_edges = fs->use_greasepencil_edges;
	fmd->cutter_axis = fs->cutter_axis;

	// add more constraint types, as in special ones (3x generic and so on)
	fmd->cluster_constraint_type = fs->cluster_constraint_type;
	fmd->constraint_target = fs->constraint_target;

	fmd->impulse_dampening = fs->impulse_dampening;
	fmd->minimum_impulse = fs->minimum_impulse;
	fmd->directional_factor = fs->directional_factor;
	fmd->mass_threshold_factor = fs->mass_threshold_factor;
	fmd->use_compounds = fs->use_compounds;
}

static Shard* fracture_object_to_shard( Object *own, Object* target)
{
	DerivedMesh *dm;
	Shard *s = NULL;

	MVert* mvert, *mv;
	MPoly* mpoly;
	MLoop* mloop;
	MEdge* medge;
	SpaceTransform trans;
	float mat[4][4], size[3] = {1.0f, 1.0f, 1.0f};

	int totvert, totpoly, totloop, totedge, v;
	bool do_free = false;

	dm = target->derivedFinal;
	if (!dm)
	{	//fallback if no derivedFinal available
		dm = CDDM_from_mesh((Mesh*)target->data);
		do_free = true;
	}

	unit_m4(mat);
	BLI_space_transform_from_matrices(&trans, target->obmat, mat);
	//BLI_SPACE_TRANSFORM_SETUP(&trans, target, own);
	mat4_to_size(size, target->obmat);

	mvert = dm->getVertArray(dm);
	mpoly = dm->getPolyArray(dm);
	mloop = dm->getLoopArray(dm);
	medge = dm->getEdgeArray(dm);
	totvert = dm->getNumVerts(dm);
	totpoly = dm->getNumPolys(dm);
	totloop = dm->getNumLoops(dm);
	totedge = dm->getNumEdges(dm);

	// create temp shard -> that necessary at all ?
	s = BKE_create_fracture_shard(mvert, mpoly, mloop, medge, totvert, totpoly, totloop, totedge, true);

	//use this as size holder, and rawcentroid is the old ob location
	copy_v3_v3(s->impact_size, size);

	//compare centroid in worldspace with location
	mul_v3_m4v3(s->raw_centroid, target->obmat, s->centroid);

	for (v = 0, mv = s->mvert; v < s->totvert; v++, mv++)
	{
		//mul_v3_v3(mv->co, size);

		//shrink the shard ? (and take centroid diff into account here, too)
		BLI_space_transform_apply(&trans, mv->co);

		//add_v3_v3(mv->co, target->loc);
		//sub_v3_v3(mv->co, s->raw_centroid);
	}

//	BLI_space_transform_apply(&trans, s->raw_centroid);
	BLI_space_transform_apply(&trans, s->centroid);

	s = BKE_custom_data_to_shard(s, dm);
	BKE_shard_calc_minmax(s);

	if (do_free && dm)
	{
		dm->needsFree = 1;
		dm->release(dm);
	}

	return s;
}

static void fracture_update_shards(FractureModifierData *fmd, Shard *s)
{
	FracMesh* fm;

	if (!fmd->frac_mesh)
	{
		fmd->frac_mesh = BKE_create_fracture_container();
		fmd->frac_mesh->progress_counter = 0; //XXXX ABUSE this for vertstart now, threading doesnt work anyway yet
		fmd->matstart = 1; //TODO, is this 1-based ?
	}

	fm = fmd->frac_mesh;
	BLI_addtail(&fm->shard_map, s);
	s->shard_id = fm->shard_count;
	fm->shard_count++;
}

static MeshIsland* fracture_shard_to_island(FractureModifierData *fmd, Shard *s, int vertstart, float quat[4])
{
	MeshIsland *mi;
	int k = 0, j = 0, totvert;
	MVert *mverts = NULL, *verts, *mv;

	//create mesh island and intialize
	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
	BLI_addtail(&fmd->meshIslands, mi);
	mi->participating_constraints = NULL;
	mi->participating_constraint_count = 0;
	mi->thresh_weight = 0.0f;
	mi->ground_weight = 0.0f;
	mi->vertex_count = s->totvert;
	mi->totcol = 0;
	mi->totdef = 0;

	//link up the visual mesh verts
	mi->vertices_cached = MEM_mallocN(sizeof(MVert *) * s->totvert, "vert_cache");
	if (fmd->visible_mesh_cached) /*ensure to be NULL in "pack, unpack" methods */
		mverts = CDDM_get_verts(fmd->visible_mesh_cached);
	mi->vertex_indices = MEM_mallocN(sizeof(int) * mi->vertex_count, "mi->vertex_indices");

	for (k = 0; k < s->totvert; k++) {
		if (mverts)
		{
			mi->vertices_cached[k] = mverts + vertstart + k;
		}
		else
		{
			mi->vertices_cached[k] = NULL;
		}
		mi->vertex_indices[k] = vertstart + k;
	}

	//some dummy setup, necessary here ?
	mi->locs = MEM_mallocN(sizeof(float)*3, "mi->locs");
	mi->rots = MEM_mallocN(sizeof(float)*4, "mi->rots");
	mi->frame_count = 0;
	if (fmd->modifier.scene && fmd->modifier.scene->rigidbody_world)
	{
		/*modifier might have no linked scene yet after creation on an inactive layer */
		/*so just try a fallback here */
		mi->start_frame = fmd->modifier.scene->rigidbody_world->pointcache->startframe;
	}
	else
	{
		mi->start_frame = 1;
	}

	mi->physics_mesh = BKE_shard_create_dm(s, true);
	totvert = mi->physics_mesh->getNumVerts(mi->physics_mesh);
	verts = mi->physics_mesh->getVertArray(mi->physics_mesh);

	mi->vertco = MEM_mallocN(sizeof(float) * 3 * totvert, "vertco");
	mi->vertno = MEM_mallocN(sizeof(short) * 3 * totvert, "vertno");

	for (mv = verts, j = 0; j < totvert; mv++, j++) {
		short no[3];

		mi->vertco[j * 3] = mv->co[0];
		mi->vertco[j * 3 + 1] = mv->co[1];
		mi->vertco[j * 3 + 2] = mv->co[2];

		copy_v3_v3_short(no, mv->no);

		mi->vertno[j * 3] = no[0];
		mi->vertno[j * 3 + 1] = no[1];
		mi->vertno[j * 3 + 2] = no[2];

		/* then eliminate centroid in vertex coords*/
		sub_v3_v3(mv->co, s->centroid);

		mul_qt_v3(quat, mv->co);
	}

	copy_v3_v3(mi->centroid, s->centroid);
	mi->id = s->shard_id;
	mi->bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(mi->bb, s->min, s->max);
	mi->particle_index = -1;

	//this info isnt necessary here... constraints will be provided too !
	mi->neighbor_ids = s->neighbor_ids;
	mi->neighbor_count = s->neighbor_count;

	return mi;
}

int BKE_fracture_update_visual_mesh(FractureModifierData *fmd, Object *ob, bool do_custom_data)
{
	MeshIsland *mi;
	DerivedMesh *dm = fmd->visible_mesh_cached;
	int vertstart = 0, totvert = 0, totpoly = 0, polystart = 0, matstart = 1, defstart = 0, loopstart = 0;
	MVert *mv = NULL;
	MPoly *mp = NULL, *mpoly = NULL, *ppoly = NULL, *pp = NULL, *spoly = NULL, *sp = NULL, *tpoly = NULL, *tp = NULL;
	int i = 0, j = 0;
	MDeformVert *dvert = NULL;
	Mesh *me = (Mesh*)ob->data;
	Shard *s, *t;

	if (dm)
	{
		vertstart = dm->getNumVerts(dm);
		dm->needsFree = 1;
		dm->release(dm);
		dm = fmd->visible_mesh_cached = NULL;
	}

	fmd->visible_mesh_cached = create_dm(fmd, do_custom_data, fmd->pack_storage.first != NULL);

	if (!fmd->visible_mesh_cached)
		return 0;

	//store start mesh in order to be able to change autohide dist based on it later in sim too !
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}

	fmd->dm = CDDM_copy(fmd->visible_mesh_cached);

	dm = fmd->visible_mesh_cached;
	mv = dm->getVertArray(dm);
	totvert = dm->getNumVerts(dm);
	mpoly = dm->getPolyArray(dm);
	dvert = CustomData_get_layer(&dm->vertData, CD_MDEFORMVERT);

	CustomData_merge(&dm->loopData, &me->ldata, CD_MASK_MLOOPUV, CD_CALLOC, dm->getNumLoops(dm));
	CustomData_merge(&dm->polyData, &me->pdata, CD_MASK_MTEXPOLY, CD_CALLOC, dm->getNumPolys(dm));

	s = fmd->frac_mesh->shard_map.first;
	t = fmd->pack_storage.first;

	//update existing island's vert refs, if any...should have used indexes instead :S
	for (mi = fmd->meshIslands.first; mi; mi = mi->next)
	{
		MVert *pvert = mi->physics_mesh->getVertArray(mi->physics_mesh);
		float inv_size[3] = {1.0f, 1.0f, 1.0f};
		//Shard *s = BLI_findlink(&fmd->frac_mesh->shard_map, mi->id);
		//Shard *t = BLI_findlink(&fmd->pack_storage, mi->id);

		//if (!s)
		//	continue;

		//CustomData_copy_data(&dm->loopData, &me->ldata, loopstart, loopstart, s->totloop);
		//CustomData_copy_data(&dm->polyData, &me->pdata, polystart, polystart, s->totpoly);

		inv_size[0] = 1.0f / s->impact_size[0];
		inv_size[1] = 1.0f / s->impact_size[1];
		inv_size[2] = 1.0f / s->impact_size[2];

		for (i = 0; i < mi->vertex_count; i++)
		{
			//just update pointers, dont need to reallocate something
			MVert *v = NULL, *pv = NULL;
			int index;

			//also correct indexes
			if (mi->vertex_indices[i] >= totvert)
			{
				index = mi->vertex_indices[i];
				mi->vertex_indices[i] -= (vertstart - totvert);
				printf("I: %d, O: %d, N: %d\n", i, index, mi->vertex_indices[i]);
			}

			index = mi->vertex_indices[i];
			v = mv + index;
			mi->vertices_cached[i] = v;

			pv = pvert + i;
			mul_v3_v3(pv->co, inv_size);

			//transform vertex properly ? compensate for shrunken shard ?
			//sub_v3_v3v3(loc, mi->centroid, s->raw_centroid);
			//loc_quat_size_to_mat4(mat, loc , rot, s->impact_size);

			//invert_m4_m4(imat, mat);
#if 0
			pv = pvert + i;
			mul_m4_v3(imat, pv->co);
#endif

			//eliminate shrink but take also difference in centroids into account here
			//sub_v3_v3(v->co, s->centroid);
			//mul_m4_v3(imat, v->co);
			//add_v3_v3(v->co, s->centroid);

//			sub_v3_v3(v->co, s->centroid);
//			mul_v3_v3(v->co, inv_size);
//			add_v3_v3(v->co, s->centroid);

			//printf("%d %d\n", index, dm->getNumVerts(dm));

			//hrm perhaps we need to update rest coordinates, too...
			mi->vertco[3*i] = v->co[0];
			mi->vertco[3*i+1] = v->co[1];
			mi->vertco[3*i+2] = v->co[2];

			mi->vertno[3*i] = v->no[0];
			mi->vertno[3*i+1] = v->no[1];
			mi->vertno[3*i+2] = v->no[2];

			if (ob && dvert)
			{
				int l;
				MDeformVert *dv = dvert + mi->vertex_indices[i];
				if (dv && dv->dw)
				{
					for (l = 0; l < dv->totweight; l++)
					{
						MDeformWeight *dw = dv->dw;
						//refill mapping data, to make it accessible for each vert (for dumb mapping function)
						int key = defstart + l;
						int index = GET_INT_FROM_POINTER(BLI_ghash_lookup(fmd->defgrp_index_map, SET_INT_IN_POINTER(key)));
						//printf("Got: %d %d\n", key, index);
						if (dw->def_nr == l)
							dw->def_nr = index;
					}

					//XXX TODO store this on physics mesh too ? to be able to reload it from blend
				}
			}
		}

		defstart += mi->totdef;

		totpoly = mi->physics_mesh->getNumPolys(mi->physics_mesh);
		ppoly = mi->physics_mesh->getPolyArray(mi->physics_mesh);
		spoly = s->mpoly;
		if (t) {
			tpoly = t->mpoly;
		}

		for (j = 0, mp = mpoly + polystart, pp = ppoly, sp = spoly; j < totpoly; j++, mp++, pp++, sp++)
		{
			/* material index lookup and correction, avoid having the same material in different slots */
			int index = GET_INT_FROM_POINTER(BLI_ghash_lookup(fmd->material_index_map,
			                                 SET_INT_IN_POINTER(mp->mat_nr + matstart)));

			if (index > 0)
				index--;

			mp->mat_nr = index;
			//store this on physics mesh as well, and on shard too so for being able to reload it from blend later (without
			// having a materialmap then)
			pp->mat_nr = index;
			sp->mat_nr = index;

			//also dont forget pack storage
			if (tpoly) {
				tp = tpoly + j;
				tp->mat_nr = index;
			}
		}

		/* fortunately we know how many faces "belong" to this meshisland, too */
		polystart += totpoly;
		matstart += mi->totcol;
		loopstart += s->totloop;

		if (s) {
			s = s->next;
		}

		if (t) {
			t = t->next;
		}
	}

	return vertstart;
}

int fracture_collect_defgrp(Object* o, Object* ob, int defstart, GHash** def_index_map)
{
	bDeformGroup *vgroup, *ngroup;
	int k = 0;

	/* create vertexgroups on new object, if they dont exist already there*/
	for (vgroup = o->defbase.first; vgroup; vgroup = vgroup->next) {
		int index = defgroup_name_index(ob, vgroup->name);
		int key = defstart + k;

		if (index == -1) {
			// old group index + defstart to make it somehow linearized
			ngroup = MEM_callocN(sizeof(bDeformGroup), "collect deformGroup");
			memcpy(ngroup, vgroup, sizeof(bDeformGroup));
			BLI_addtail(&ob->defbase, ngroup);
			index = BLI_listbase_count(&ob->defbase)-1;
		}

		if (!BLI_ghash_haskey(*def_index_map, SET_INT_IN_POINTER(key)))
			BLI_ghash_insert(*def_index_map, SET_INT_IN_POINTER(key), SET_INT_IN_POINTER(index));

		k++;
	}

	if (ob->defbase.first && ob->actdef == 0)
		ob->actdef = 1;

	return k;
}

short BKE_fracture_collect_materials(Object* o, Object* ob, int matstart, GHash** mat_index_map)
{
	short *totcolp = NULL;
	Material ***matarar = NULL;
	int j;

	/* append materials to target object, if not existing yet */
	totcolp = give_totcolp(o);
	matarar = give_matarar(o);

	for (j = 0; j < (*totcolp); j++)
	{
		void *key;
		int index = BKE_object_material_slot_find_index(ob, (*matarar)[j]);
		if (index == 0)
		{
			index = ob->totcol+1;
			assign_material(ob, (*matarar)[j], index, BKE_MAT_ASSIGN_USERPREF);
		}

		key = SET_INT_IN_POINTER(matstart+j);
		if (!BLI_ghash_haskey(*mat_index_map, key))
			BLI_ghash_insert(*mat_index_map, key, SET_INT_IN_POINTER(index));
	}

	return (*totcolp);
}

void pack_storage_add(FractureModifierData *fmd, Shard* s)
{
	Shard *t = BKE_fracture_shard_copy(s);
	BLI_addtail(&fmd->pack_storage, t);
}

void fracture_collect_layer(CustomData* src, CustomData *dst, int totelem, int cd_type, int dst_offset, int count)
{
	int layerstart = CustomData_get_layer_index(src, cd_type);
	int totlayer = CustomData_number_of_layers(src, cd_type);
	int j;

	for (j = 0; j < totlayer; j++)
	{
		const char *name = CustomData_get_layer_name(src, cd_type, j);

		//find index of named layer in dst mesh
		int index = CustomData_get_named_layer_index(dst, cd_type, name);
		if (index == -1)
		{
			//add layer if not there
			//void *layer = CustomData_get_layer_named(src, cd_type, name);
			CustomData_add_layer_named(dst, cd_type, CD_CALLOC, NULL, totelem, name);
		}

		index = CustomData_get_named_layer_index(dst, cd_type, name);
		CustomData_copy_data_layer(src, dst, j+layerstart, index, 0, dst_offset, count);
	}
}

void fracture_collect_layers(Shard* s, DerivedMesh *dm, int vertstart, int polystart, int loopstart, int edgestart)
{
	int totloop = dm->getNumLoops(dm);
	int totvert = dm->getNumVerts(dm);
	int totpoly = dm->getNumPolys(dm);
	int totedge = dm->getNumEdges(dm);

	fracture_collect_layer(&s->vertData, &dm->vertData, totvert, CD_MDEFORMVERT, vertstart, s->totvert);
	fracture_collect_layer(&s->loopData, &dm->loopData, totloop, CD_MLOOPUV, loopstart, s->totloop);
	fracture_collect_layer(&s->polyData, &dm->polyData, totpoly, CD_MTEXPOLY, polystart, s->totpoly);
	fracture_collect_layer(&s->edgeData, &dm->edgeData, totedge, CD_CREASE, edgestart, s->totedge);
	fracture_collect_layer(&s->edgeData, &dm->edgeData, totedge, CD_BWEIGHT, edgestart, s->totedge);
	//fracture_collect_layer(&s->edgeData, &dm->edgeData, totedge, CD_MEDGE, edgestart, s->totedge);
	fracture_collect_layer(&s->vertData, &dm->vertData, totvert, CD_PROP_FLT, vertstart, s->totvert);
}

MeshIsland* BKE_fracture_mesh_island_add(FractureModifierData *fmd, Object* own, Object *target)
{
	MeshIsland *mi;
	Shard *s;
	int vertstart = 0;
	short totcol = 0, totdef = 0;
	float loc[3], quat[4], iquat[4];

	if (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL || own->type != OB_MESH || !own->data)
		return NULL;

	if (target->type != OB_MESH || !target->data)
		return NULL;

	//lets see whether we need to add loc here too XXX TODO
	mat4_to_loc_quat(loc, quat, target->obmat);

	s = fracture_object_to_shard(own, target);
	copy_v3_v3(s->centroid, loc);

	fracture_update_shards(fmd, s);

	vertstart = fmd->frac_mesh->progress_counter;
	fmd->frac_mesh->progress_counter += s->totvert;

	//hrm need to rebuild ALL islands since vertex refs are bonkers now after mesh has changed
	invert_qt_qt(iquat, quat);
	mi = fracture_shard_to_island(fmd, s, vertstart, iquat);

	copy_qt_qt(mi->rot, quat);
	copy_v3_v3(mi->centroid, loc);

	mi->rigidbody = BKE_rigidbody_create_shard(fmd->modifier.scene, own, target, mi);
	if (mi->rigidbody)
	{
		mi->rigidbody->meshisland_index = mi->id;
	}

	BLI_strncpy(mi->name, target->id.name + 2, MAX_ID_NAME - 2);

	//handle materials
	if (!fmd->material_index_map)
	{
		fmd->material_index_map = BLI_ghash_int_new("mat_index_map");
		fmd->matstart = 1;
	}

	totcol = BKE_fracture_collect_materials(target, own, fmd->matstart, &fmd->material_index_map);
	if (totcol < 0)
	    totcol = 0;
	fmd->matstart += totcol;
	mi->totcol = totcol;

	/*XXXXX TODO deal with material deletion, and reorder (in material code) */

	//handle vertexgroups, too
	if (!fmd->defgrp_index_map)
	{
		fmd->defgrp_index_map = BLI_ghash_int_new("defgrp_index_map");
		fmd->defstart = 0;
	}

	totdef = fracture_collect_defgrp(target, own, fmd->defstart, &fmd->defgrp_index_map);
	if (totdef < 0)
		totdef = 0;
	fmd->defstart += totdef;
	mi->totdef = totdef;

	//XXX TODO handle UVs, shapekeys and more ?
//	fracture_collect_uv_tex(target, own);

	//add shard to pack storage
	pack_storage_add(fmd, s);

	return mi;
}

void BKE_fracture_free_mesh_island(FractureModifierData *rmd, MeshIsland *mi, bool remove_rigidbody)
{
	if (mi->physics_mesh) {
		mi->physics_mesh->needsFree = 1;
		mi->physics_mesh->release(mi->physics_mesh);
		mi->physics_mesh = NULL;
	}

	if (mi->rigidbody) {
		if (remove_rigidbody && rmd->modifier.scene)
			BKE_rigidbody_remove_shard(rmd->modifier.scene, mi);
		MEM_freeN(mi->rigidbody);
		mi->rigidbody = NULL;
	}

	if (mi->vertco) {
		MEM_freeN(mi->vertco);
		mi->vertco = NULL;
	}

	if (mi->vertno) {
		MEM_freeN(mi->vertno);
		mi->vertno = NULL;
	}

	if (mi->vertices) {
		//MEM_freeN(mi->vertices);
		mi->vertices = NULL; /*borrowed only !!!*/
	}

	if (mi->vertices_cached) {
		MEM_freeN(mi->vertices_cached);
		mi->vertices_cached = NULL;
	}

	if (mi->bb != NULL) {
		MEM_freeN(mi->bb);
		mi->bb = NULL;
	}

	if (mi->participating_constraints != NULL) {
		int i = 0;
		for (i = 0; i < mi->participating_constraint_count; i++)
		{
			RigidBodyShardCon *con = mi->participating_constraints[i];
			if (con) {
				con->mi1 = NULL;
				con->mi2 = NULL;
			}
		}

		MEM_freeN(mi->participating_constraints);
		mi->participating_constraints = NULL;
		mi->participating_constraint_count = 0;
	}

	if (mi->vertex_indices) {
		MEM_freeN(mi->vertex_indices);
		mi->vertex_indices = NULL;
	}

	if (mi->rots) {
		MEM_freeN(mi->rots);
		mi->rots = NULL;
	}

	if (mi->locs) {
		MEM_freeN(mi->locs);
		mi->locs = NULL;
	}

	if (mi->acc_sequence)
	{
		MEM_freeN(mi->acc_sequence);
		mi->acc_sequence = NULL;
	}

	mi->frame_count = 0;

	MEM_freeN(mi);
	mi = NULL;
}

void pack_storage_remove(FractureModifierData *fmd, Shard *s)
{
	Shard *t = fmd->pack_storage.first;
	while(t)
	{
		if (t->shard_id == s->shard_id)
		{
			BLI_remlink(&fmd->pack_storage, t);
			BKE_shard_free(t, true);
			break;
		}

		t = t->next;
	}
}

void BKE_fracture_mesh_island_remove(FractureModifierData *fmd, MeshIsland *mi)
{
	if (BLI_listbase_is_single(&fmd->meshIslands))
	{
		BKE_fracture_mesh_island_remove_all(fmd);
		return;
	}

	if (fmd->frac_mesh && mi)
	{
		int index = BLI_findindex(&fmd->meshIslands, mi);
		Shard *s = BLI_findlink(&fmd->frac_mesh->shard_map, index);
		if (s)
		{
			int i;
			BLI_remlink(&fmd->frac_mesh->shard_map, s);
			pack_storage_remove(fmd, s);
			BKE_shard_free(s, true);
			fmd->frac_mesh->shard_count--;

			BLI_remlink(&fmd->meshIslands, mi);
			for (i = 0; i < mi->participating_constraint_count; i++)
			{
				RigidBodyShardCon *con = mi->participating_constraints[i];
				BLI_remlink(&fmd->meshConstraints, con);
				BKE_rigidbody_remove_shard_con(fmd->modifier.scene, con);
			}

			BKE_fracture_free_mesh_island(fmd, mi, true);
		}
	}
}

void pack_storage_remove_all(FractureModifierData*fmd)
{
	Shard *s;
	while (fmd->pack_storage.first) {
		s = fmd->pack_storage.first;
		BLI_remlink(&fmd->pack_storage, s);
		BKE_shard_free(s, true);
	}
}

void BKE_fracture_mesh_island_remove_all(FractureModifierData *fmd)
{
	MeshIsland *mi;

	//free all shards
	BKE_fracmesh_free(fmd->frac_mesh, true);
	MEM_freeN(fmd->frac_mesh);
	fmd->frac_mesh = NULL;

	//free pack storage
	pack_storage_remove_all(fmd);

	//free all constraints first
	BKE_free_constraints(fmd);

	//free all meshislands
	while (fmd->meshIslands.first) {
		mi = fmd->meshIslands.first;
		BLI_remlink(&fmd->meshIslands, mi);
		BKE_fracture_free_mesh_island(fmd, mi, true);
	}

	fmd->meshIslands.first = NULL;
	fmd->meshIslands.last = NULL;

	//free visual_mesh
	if (fmd->visible_mesh_cached)
	{
		fmd->visible_mesh_cached->needsFree = 1;
		fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
		fmd->visible_mesh_cached = NULL;
	}

	if (fmd->material_index_map)
	{
		BLI_ghash_free(fmd->material_index_map, NULL, NULL);
		fmd->material_index_map = NULL;
		fmd->matstart = 1;
	}

	if (fmd->defgrp_index_map)
	{
		BLI_ghash_free(fmd->defgrp_index_map, NULL, NULL);
		fmd->defgrp_index_map = NULL;
		fmd->defstart = 0;
	}
}

RigidBodyShardCon *BKE_fracture_mesh_islands_connect(FractureModifierData *fmd, MeshIsland *mi1, MeshIsland *mi2, short con_type)
{
	RigidBodyShardCon *rbsc;

	if (mi1 == NULL || mi2 == NULL)
		return NULL;

	rbsc = BKE_rigidbody_create_shard_constraint(fmd->modifier.scene, con_type, false);
	rbsc->mi1 = mi1;
	rbsc->mi2 = mi2;

	if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		/* disable breaking flag here by default, only enable later via python if necessary */
		rbsc->flag &= ~RBC_FLAG_USE_BREAKING;

		/* also delete all other "default" flags here, let them being overriden from python too */
		//rbsc->flag &= ~RBC_FLAG_ENABLED;
		rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;

#if 0
		/* and dont allow to let constrained objects collide per default, as with regular constraints */
		rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;
#endif
	}

	/* moved default meshconstraint pos calculation here to creation, so you can override it later on*/
	/* do this for all constraints */
	/* location for fixed constraints doesnt matter, so keep old setting */
	if (rbsc->type == RBC_TYPE_FIXED) {
		copy_v3_v3(rbsc->pos, rbsc->mi1->rigidbody->pos);
	}
	else {
		/* else set location to center */
		add_v3_v3v3(rbsc->pos, rbsc->mi1->rigidbody->pos, rbsc->mi2->rigidbody->pos);
		mul_v3_fl(rbsc->pos, 0.5f);
	}

	copy_qt_qt(rbsc->orn, rbsc->mi1->rigidbody->orn);

	if (BLI_listbase_is_empty(&fmd->meshConstraints))
	{
		fmd->constraint_count = 0;
	}

	BLI_addtail(&fmd->meshConstraints, rbsc);
	fmd->constraint_count++;

#if 0
	if (index > -1)
	{
		rbsc->id = index;
	}
	else
	{
		rbsc->id = fmd->constraint_count-1;
	}
#endif

	/* store constraints per meshisland too, to allow breaking percentage */
	if (mi1->participating_constraints == NULL) {
		mi1->participating_constraints = MEM_mallocN(sizeof(RigidBodyShardCon *), "part_constraints_mi1");
		mi1->participating_constraints[0] = rbsc;
		mi1->participating_constraint_count = 1;
	}
	else
	{
		mi1->participating_constraints = MEM_reallocN(mi1->participating_constraints, sizeof(RigidBodyShardCon *) * (mi1->participating_constraint_count + 1));
		mi1->participating_constraints[mi1->participating_constraint_count] = rbsc;
		mi1->participating_constraint_count++;
	}

	if (mi2->participating_constraints == NULL) {
		mi2->participating_constraints = MEM_mallocN(sizeof(RigidBodyShardCon *), "part_constraints_mi2");
		mi2->participating_constraints[0] = rbsc;
		mi2->participating_constraint_count = 1;
	}
	else
	{
		mi2->participating_constraints = MEM_reallocN(mi2->participating_constraints, sizeof(RigidBodyShardCon *) * (mi2->participating_constraint_count + 1));
		mi2->participating_constraints[mi2->participating_constraint_count] = rbsc;
		mi2->participating_constraint_count++;
	}

	return rbsc;
}

static void remove_participants(RigidBodyShardCon* con, MeshIsland *mi)
{
	RigidBodyShardCon **cons;
	/* Probably wrong, would need to shrink array size... listbase would have been better here */
	/* info not necessary so omit */
	int count = 0;

	if (!mi)
		return;

	count = mi->participating_constraint_count;

	if (count > 1)
	{
		int i, j = 0;
		mi->participating_constraint_count--;
		cons = MEM_callocN(sizeof(RigidBodyShardCon *) * (count-1) , "temp cons");
		for (i = 0; i < mi->participating_constraint_count; i++)
		{
			if (mi->participating_constraints[i] != con)
			{
				cons[j] = con;
				j++;
			}
		}

		MEM_freeN(mi->participating_constraints);
		mi->participating_constraints = cons;
	}
}

void BKE_fracture_mesh_constraint_remove(FractureModifierData *fmd, RigidBodyShardCon* con)
{
	remove_participants(con, con->mi1);
	remove_participants(con, con->mi2);

	BLI_remlink(&fmd->meshConstraints, con);
	BKE_rigidbody_remove_shard_con(fmd->modifier.scene, con);
	MEM_freeN(con);
	if (fmd->constraint_count > 0)
	{
		fmd->constraint_count--;
	}
}

void BKE_fracture_mesh_constraint_remove_all(FractureModifierData *fmd)
{
	BKE_free_constraints(fmd);
	fmd->constraint_count = 0;
}

/* flush a hflag to from verts to edges/faces */
void BKE_bm_mesh_hflag_flush_vert(BMesh *bm, const char hflag)
{
	BMEdge *e;
	BMLoop *l_iter;
	BMLoop *l_first;
	BMFace *f;

	BMIter eiter;
	BMIter fiter;

	int ok;

	BM_ITER_MESH (e, &eiter, bm, BM_EDGES_OF_MESH) {
		if (BM_elem_flag_test(e->v1, hflag) &&
		    BM_elem_flag_test(e->v2, hflag))
		{
			BM_elem_flag_enable(e, hflag);
		}
		else {
			BM_elem_flag_disable(e, hflag);
		}
	}
	BM_ITER_MESH (f, &fiter, bm, BM_FACES_OF_MESH) {
		ok = true;
		l_iter = l_first = BM_FACE_FIRST_LOOP(f);
		do {
			if (!BM_elem_flag_test(l_iter->v, hflag)) {
				ok = false;
				break;
			}
		} while ((l_iter = l_iter->next) != l_first);

		BM_elem_flag_set(f, hflag, ok);
	}
}

void BKE_meshisland_constraint_create(FractureModifierData* fmd, MeshIsland *mi1, MeshIsland *mi2, int con_type, float thresh)
{
	RigidBodyShardCon *rbsc;
	rbsc = BKE_rigidbody_create_shard_constraint(fmd->modifier.scene, con_type, fmd->fracture_mode != MOD_FRACTURE_DYNAMIC);
	rbsc->mi1 = mi1;
	rbsc->mi2 = mi2;

	BLI_snprintf(rbsc->name, 64, "%s-%s", rbsc->mi1->name, rbsc->mi2->name);

	if (thresh == 0 || fmd->use_breaking == false) {
		rbsc->flag &= ~RBC_FLAG_USE_BREAKING;
	}

	if (!fmd->use_constraint_collision) {
		rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;
	}
	else {
		rbsc->flag &= ~RBC_FLAG_DISABLE_COLLISIONS;
	}

	if ((mi1->particle_index != -1) && (mi2->particle_index != -1) &&
		(mi1->particle_index == mi2->particle_index))
	{
		if (fmd->cluster_count > 1) {
			rbsc->breaking_threshold = fmd->cluster_breaking_threshold;
		}
		else {
			rbsc->breaking_threshold = thresh;
		}
	}
	else
	{
		if ((mi1->particle_index != -1) && (mi2->particle_index != -1) &&
			(mi1->particle_index != mi2->particle_index))
		{
			/* set a different type of constraint between clusters */
			rbsc->type = fmd->cluster_constraint_type;
		}
		rbsc->breaking_threshold = thresh;
	}

	if (fmd->thresh_defgrp_name[0]) {
		/* modify maximum threshold by minimum weight */
		rbsc->breaking_threshold = thresh * MIN2(mi1->thresh_weight, mi2->thresh_weight);
	}

	BLI_addtail(&fmd->meshConstraints, rbsc);

	if ((mi1->object_index == -1) && (mi2->object_index == -1))
	{
		/* store constraints per meshisland too, to allow breaking percentage */
		if (mi1->participating_constraints == NULL) {
			mi1->participating_constraints = MEM_callocN(sizeof(RigidBodyShardCon *), "part_constraints_mi1");
			mi1->participating_constraint_count = 0;
		}
		mi1->participating_constraints = MEM_reallocN(mi1->participating_constraints,
		                                              sizeof(RigidBodyShardCon *) * (mi1->participating_constraint_count + 1));
		mi1->participating_constraints[mi1->participating_constraint_count] = rbsc;
		mi1->participating_constraint_count++;

		if (mi2->participating_constraints == NULL) {
			mi2->participating_constraints = MEM_callocN(sizeof(RigidBodyShardCon *), "part_constraints_mi2");
			mi2->participating_constraint_count = 0;
		}
		mi2->participating_constraints = MEM_reallocN(mi2->participating_constraints,
		                                              sizeof(RigidBodyShardCon *) * (mi2->participating_constraint_count + 1));
		mi2->participating_constraints[mi2->participating_constraint_count] = rbsc;
		mi2->participating_constraint_count++;
	}
}

void BKE_update_acceleration_map(FractureModifierData *fmd, MeshIsland* mi, Object* ob, int ctime, float acc, RigidBodyWorld *rbw)
{
	const int acc_defgrp_index = defgroup_name_index(ob, fmd->acceleration_defgrp_name);
	DerivedMesh *dm = fmd->visible_mesh_cached;
	MDeformVert *dvert = NULL, *dv = NULL;
	MDeformWeight *dw = NULL;
	float weight = 0.0f, denom;
	int i = 0, w = 0;
	int totvert = dm ? dm->getNumVerts(dm) : 0;

	if (!dm)
		return;

	dvert = dm->getVertDataArray(dm, CD_MDEFORMVERT);

	if (dvert == NULL)
	{
		dvert = CustomData_add_layer(&dm->vertData, CD_MDEFORMVERT, CD_CALLOC,
	                             NULL, totvert);
	}

	//calculate weight from force...
	denom = fmd->max_acceleration - fmd->min_acceleration;

	//sanity check
	if (denom == 0.0f)
		denom = 1.0f;

	//if (mi->acc_sequence)
	{
		weight = (acc - fmd->min_acceleration) / denom;

		for (i = 0; i < mi->vertex_count; i++)
		{
			dv = dvert + mi->vertex_indices[i];
			if (dv) {
				if (dv->dw == NULL && acc_defgrp_index >= 0) {
					defvert_add_index_notest(dv, acc_defgrp_index, 0.0f);
				}

				for (dw = dv->dw, w = 0; w < dv->totweight; dw++, w++)
				{
					if (dw->def_nr == acc_defgrp_index) {

						if (weight >= 0.0f && weight <= 1.0f) {
							dw->weight = weight;
						}
						else if (ctime == rbw->pointcache->startframe) {
							dw->weight = 0.0f;
						}

						if (ctime == rbw->ltime + 1)
							dw->weight *= fmd->acceleration_fade;
					}
				}
			}
		}
	}
}

void BKE_update_velocity_layer(FractureModifierData *fmd, MeshIsland *mi)
{
	DerivedMesh *dm = fmd->visible_mesh_cached;
	float *velX=NULL, *velY=NULL, *velZ = NULL;
	RigidBodyOb *rbo = mi->rigidbody;
	Shard *s, *t = NULL;
	void *pX, *pY, *pZ, *spX = NULL, *spY = NULL, *spZ = NULL;
	float *sX=NULL, *sY=NULL, *sZ=NULL;
	int i = 0;
	ListBase *lb;

	if (!dm)
		return;

	//XXX TODO deal with split shards to islands etc, here take only "real" shards for now
	if (fmd->shards_to_islands) {
		lb = &fmd->islandShards;
	}
	else {
		lb = &fmd->frac_mesh->shard_map;
	}

	for (s = lb->first; s; s = s->next)
	{
		if (s->shard_id == mi->id)
		{
			t = s;
			break;
		}
	}

	pX = CustomData_get_layer_named(&dm->vertData, CD_PROP_FLT, "velX");
	pY = CustomData_get_layer_named(&dm->vertData, CD_PROP_FLT, "velY");
	pZ = CustomData_get_layer_named(&dm->vertData, CD_PROP_FLT, "velZ");

	if (!pX ||!pY || !pZ)
		return;

	velX = (float*)pX;
	velY = (float*)pY;
	velZ = (float*)pZ;

	//XXX how to represent this in mblur ?
	//zero_v3(rbo->ang_vel);

	if (t)
	{
		spX = check_add_layer(NULL, &t->vertData, CD_PROP_FLT, t->totvert, "velX");
		spY = check_add_layer(NULL, &t->vertData, CD_PROP_FLT, t->totvert, "velY");
		spZ = check_add_layer(NULL, &t->vertData, CD_PROP_FLT, t->totvert, "velZ");
	}

	for (i = 0; i < mi->vertex_count; i++)
	{
		if (spX && spY && spZ)
		{
			sX = (float*)spX;
			sY = (float*)spY;
			sZ = (float*)spZ;

			sX[i] = rbo->lin_vel[0] + rbo->ang_vel[0];
			sY[i] = rbo->lin_vel[1] + rbo->ang_vel[1];
			sZ[i] = rbo->lin_vel[2] + rbo->ang_vel[2];
		}

		velX[mi->vertex_indices[i]] = rbo->lin_vel[0] + rbo->ang_vel[0];
		velY[mi->vertex_indices[i]] = rbo->lin_vel[1] + rbo->ang_vel[1];
		velZ[mi->vertex_indices[i]] = rbo->lin_vel[2] + rbo->ang_vel[2];
	}
}

/* gah, it could be that simple, if each mod handled its stuff itself */
static DerivedMesh *eval_mod_stack_simple(Object *ob)
{
	DerivedMesh *dm = NULL;
	ModifierData *md;

	if (ob->type != OB_MESH)
		return NULL;

	if (ob->data == NULL)
		return NULL;

	dm = CDDM_from_mesh(ob->data);

	for (md = ob->modifiers.first; md; md = md->next)
	{
		const ModifierTypeInfo *mti = modifierType_getInfo(md->type);
		if (mti->deformVerts && (md->mode & (eModifierMode_Realtime | eModifierMode_Render)))
		{
			float (*vertexCos)[3];
			int totvert = dm->getNumVerts(dm);

			vertexCos = MEM_callocN(sizeof(float) * 3 * totvert, "Vertex Cos");
			dm->getVertCos(dm, vertexCos);
			mti->deformVerts(md, ob, dm, vertexCos, totvert, 0);
			CDDM_apply_vert_coords(dm, vertexCos);
			MEM_freeN(vertexCos);
			CDDM_calc_normals(dm);
		}

		if (mti->applyModifier && (md->mode & (eModifierMode_Realtime | eModifierMode_Render)))
		{
			DerivedMesh *ndm;

			ndm = mti->applyModifier(md, ob, dm, 0);
			if (ndm != dm)
			{
				dm->needsFree = 1;
				dm->release(dm);
			}
			dm = ndm;
			CDDM_calc_normals(dm);
		}
	}

	return dm;
}

void activate(MeshIsland *mi, AnimBind *bind)
{
	bind->v = -1;
	bind->v1 = -1;
	bind->v2 = -1;
	bind->mi = -1;
	zero_v3(bind->no);
	zero_v3(bind->offset);
	unit_qt(bind->quat);

	if (mi->rigidbody->type == RBO_TYPE_ACTIVE)
	{
		RigidBodyOb* rbo = mi->rigidbody;

		rbo->flag &= ~RBO_FLAG_KINEMATIC;
		rbo->flag &= ~RBO_FLAG_KINEMATIC_BOUND;
		rbo->flag |= RBO_FLAG_NEEDS_VALIDATE;

		if (rbo->physics_object)
		{
			RB_body_set_mass(rbo->physics_object, rbo->mass);
			RB_body_set_kinematic_state(rbo->physics_object, false);
			RB_body_activate(rbo->physics_object);
		}
	}
}

void BKE_read_animated_loc_rot(FractureModifierData *fmd, Object *ob, bool do_bind)
{
	//to be called after rigidbodies have been actually created... from MOD_fracture.c
	//rotation is optional, remesher + particlesystem can provide it
	float *quatX, *quatY, *quatZ, *quatW;
	MVert *mvert = NULL;
	MPoly *mpoly = NULL;
	MLoop *mloop = NULL;
	MeshIsland *mi;
	DerivedMesh *dm = NULL;
	int totvert, count = 0, i = 0, *orig_index = NULL, totpoly, items;
	KDTree *tree = NULL;
	float anim_quat[4], anim_imat[4][4], imat[4][4];

	if (!fmd->anim_mesh_ob)
		return;

	if (fmd->anim_mesh_ob == ob)
		return;

	dm = eval_mod_stack_simple(fmd->anim_mesh_ob);

	if (!dm)
		return;

	totvert = dm->getNumVerts(dm);
	totpoly = dm->getNumPolys(dm);

	invert_m4_m4(anim_imat, fmd->anim_mesh_ob->obmat);
	invert_m4_m4(imat, ob->obmat);

	if (totvert == 0) {
		dm->release(dm);
		return;
	}

	if (do_bind) {

		items = totpoly > 0 ? totpoly : totvert;

		count = BLI_listbase_count(&fmd->meshIslands);
		tree = BLI_kdtree_new(items);

		fmd->anim_bind_len = count;
		if (fmd->anim_bind) {
			MEM_freeN(fmd->anim_bind);
		}

		//TODO, possibly a weak solution, but do we really want to store the length too ?
		fmd->anim_bind = MEM_mallocN(sizeof(AnimBind) * fmd->anim_bind_len, "anim_bind");
		for (i = 0; i < fmd->anim_bind_len; i++)
		{
			fmd->anim_bind[i].mi = -1;
			fmd->anim_bind[i].v = -1;
			fmd->anim_bind[i].v1 = -1;
			fmd->anim_bind[i].v2 = -1;
			zero_v3(fmd->anim_bind[i].offset);
			zero_v3(fmd->anim_bind[i].no);
			unit_qt(fmd->anim_bind[i].quat);
		}
	}

	i = 0;
	mvert = dm->getVertArray(dm);
	mpoly = dm->getPolyArray(dm);
	mloop = dm->getLoopArray(dm);
	if (do_bind)
	{
		if (totpoly > 0)
		{
			//poly based bind
			for (i = 0; i < totpoly; i++)
			{
				float co[3];
				copy_v3_v3(co, mvert[mloop[mpoly[i].loopstart].v].co);
				BLI_kdtree_insert(tree, i, co);
			}
		}
		else
		{
			//no faces -> vertex based bind
			for (i = 0; i < totvert; i++)
			{
				float co[3];
				copy_v3_v3(co, mvert[i].co);
				BLI_kdtree_insert(tree, i, co);
			}
		}

		BLI_kdtree_balance(tree);
	}


	quatX = CustomData_get_layer_named(&dm->vertData, CD_PROP_FLT, "quatX");
	quatY = CustomData_get_layer_named(&dm->vertData, CD_PROP_FLT, "quatY");
	quatZ = CustomData_get_layer_named(&dm->vertData, CD_PROP_FLT, "quatZ");
	quatW = CustomData_get_layer_named(&dm->vertData, CD_PROP_FLT, "quatW");
	orig_index = CustomData_get_layer(&dm->vertData, CD_ORIGINDEX);

	//check vertexcount and islandcount, TODO for splitshards... there it might differ, ignore then for now
	//later do interpolation ? propagate to islands somehow then, not yet now...
	//also maybe skip for dynamic for now, since this is totally different

	//bind loop, bind the verts to the shards
	if (do_bind)
	{
		i = 0;
		for (mi = fmd->meshIslands.first; mi; mi = mi->next, i++)
		{
			KDTreeNearest n;
			float co[3], diff[3] = {0, 0, 0}, f_no[3];
			int j = 0;

			copy_v3_v3(co, mi->rigidbody->pos);
			mul_m4_v3(anim_imat, co);
			BLI_kdtree_find_nearest(tree, co, &n);

			if (n.dist <= fmd->anim_bind_limit || fmd->anim_bind_limit == 0)
			{
				if (totpoly > 0)
				{
					int v1, v2, v3;
					float limit = 0.0001f;
					MPoly *mp  = mpoly + n.index;
					MLoop *ml = mloop + mp->loopstart;
					BKE_mesh_calc_poly_normal(mp, ml, mvert, f_no);

					v1 = ml->v;
					v2 = (ml + 1)->v;
					v3 = (ml + 2)->v;

					if (mpoly[n.index].totloop < 3)
					{
						printf("Degenerate face, skipping\n");
						activate(mi, &fmd->anim_bind[i]);
						continue;
					}

					if (compare_v3v3(mvert[v1].co, mvert[v2].co, limit) ||
					    compare_v3v3(mvert[v1].co, mvert[v3].co, limit) ||
					    compare_v3v3(mvert[v2].co, mvert[v3].co, limit))
					{
						printf("Very close coordinates, skipping %d %d %d in %d\n", v1, v2, v3, mi->id);
						activate(mi, &fmd->anim_bind[i]);
						continue;
					}

					fmd->anim_bind[i].v = v1;
					fmd->anim_bind[i].v1 = v2;
					fmd->anim_bind[i].v2 = v3;
					fmd->anim_bind[i].poly = n.index;
					mi->rigidbody->flag |= RBO_FLAG_KINEMATIC_BOUND;
				}
				else {
					fmd->anim_bind[i].v = n.index;
					fmd->anim_bind[i].v1 = -1;
					fmd->anim_bind[i].v2 = -1;
					mi->rigidbody->flag |= RBO_FLAG_KINEMATIC_BOUND;
				}

				if (fmd->anim_bind[i].v != -1)
				{
					fmd->anim_bind[i].mi = i;
					sub_v3_v3v3(diff, n.co, co);

					copy_v3_v3(fmd->anim_bind[i].offset, diff);

					if ((fmd->anim_bind[i].v1 == -1 || fmd->anim_bind[i].v2 == -1)) {
						//fallback if not enough verts around
						normal_short_to_float_v3(fmd->anim_bind[i].no, mvert[n.index].no);
						normalize_v3(fmd->anim_bind[i].no);
					}
					else if (totpoly > 0) {
						tri_to_quat_ex(fmd->anim_bind[i].quat,
								mvert[fmd->anim_bind[i].v].co,
								mvert[fmd->anim_bind[i].v1].co,
								mvert[fmd->anim_bind[i].v2].co, f_no);
						copy_v3_v3(fmd->anim_bind[i].no, f_no);
					}
				}
			}
			else
			{
				activate(mi, &fmd->anim_bind[i]);
			}
		}

		if (tree)
			BLI_kdtree_free(tree);
	}
	else
	{
		mi = fmd->meshIslands.first;
		for (i = 0; i < fmd->anim_bind_len; i++, mi = mi->next)
		{
			float co[3];
			int index = -1;
			int vindex = -1;

			index = fmd->anim_bind[i].mi;
			vindex = fmd->anim_bind[i].v;

			if (index == -1 || vindex == -1)
			{
				activate(mi, &fmd->anim_bind[i]);
				continue;
			}

			//only let kinematic rbs do this, active ones are being taken care of by bullet
			if (mi && mi->rigidbody && (mi->rigidbody->flag & RBO_FLAG_KINEMATIC))
			{
				//the 4 rot layers *should* be aligned, caller needs to ensure !
				bool quats = quatX && quatY && quatZ && quatW;
				float quat[4], vec[3], no[3], off[3];
				int v = fmd->anim_bind[i].v;
				unit_qt(quat);

				if (v >= totvert) {
					activate(mi, &fmd->anim_bind[i]);
					continue;
				}

				if ((orig_index && orig_index[v] != v && (fmd->anim_bind[i].v1 == -1 || fmd->anim_bind[i].v2 == -1)))
				{
					activate(mi, &fmd->anim_bind[i]);
					continue;
				}

				copy_v3_v3(co, mvert[v].co);
				copy_v3_v3(off, fmd->anim_bind[i].offset);

				if (fmd->anim_mesh_rot)
				{
					if (quats)
					{
						quat[0] = quatX[v];
						quat[1] = quatY[v];
						quat[2] = quatZ[v];
						quat[3] = quatW[v];
					}
					else
					{
						copy_v3_v3(vec, fmd->anim_bind[i].no);
						if (fmd->anim_bind[i].v1 == -1 || fmd->anim_bind[i].v2 == -1) {
							//fallback if not enough verts around;
							normal_short_to_float_v3(no, mvert[v].no);
							normalize_v3(no);
							rotation_between_vecs_to_quat(quat, vec, no);
						}
						else {
							float rot[4], iquat[4], fno[3];
							MPoly *mp = mpoly + fmd->anim_bind[i].poly;
							MLoop *ml = mloop + mp->loopstart;
							BKE_mesh_calc_poly_normal(mp, ml, mvert, fno);

							tri_to_quat_ex(rot, mvert[fmd->anim_bind[i].v].co,
											  mvert[fmd->anim_bind[i].v1].co,
											  mvert[fmd->anim_bind[i].v2].co,
											  fno);
							invert_qt_qt(iquat, fmd->anim_bind[i].quat);
							mul_qt_qtqt(quat, rot, iquat);
						}
					}
				}

				mul_qt_v3(quat, off);
				sub_v3_v3(co, off);
				mul_m4_v3(fmd->anim_mesh_ob->obmat, co);

				copy_v3_v3(mi->rigidbody->pos, co);

				if (fmd->anim_mesh_rot)
				{
					float ob_quat[4];
					mat4_to_quat(ob_quat, ob->obmat);
					mul_qt_qtqt(quat, ob_quat, quat);
					copy_qt_qt(mi->rigidbody->orn, quat);
				}

				mi->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
				if (mi->rigidbody->physics_object)
				{
					RB_body_set_loc_rot(mi->rigidbody->physics_object, mi->rigidbody->pos, mi->rigidbody->orn);
				}
			}
		}
	}

	if (dm)
		dm->release(dm);
}
