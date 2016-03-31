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
 * The Original Code is Copyright (C) Blender Foundation
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Martin Felke
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

#include "BLI_kdtree.h"
#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_mempool.h"
#include "BLI_path_util.h"
#include "BLI_rand.h"
#include "BLI_string.h"
#include "BLI_sort.h"
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

	dm_parent = BKE_shard_create_dm(s, true);
	bm_parent = DM_to_bmesh(dm_parent, true);
	BM_mesh_elem_table_ensure(bm_parent, BM_FACE);

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

	val_a = size1[0] * size1[1] * size1[2];
	val_b = size2[0] * size2[1] * size2[2];

	/* sort descending */
	if      (val_a < val_b) return 1;
	else if (val_a > val_b) return -1;
	return 0;
}

Shard *BKE_custom_data_to_shard(Shard *s, DerivedMesh *dm)
{
	CustomData_reset(&s->vertData);
	CustomData_reset(&s->loopData);
	CustomData_reset(&s->polyData);

	CustomData_copy(&dm->vertData, &s->vertData, CD_MASK_MDEFORMVERT, CD_DUPLICATE, s->totvert);
	CustomData_copy(&dm->loopData, &s->loopData, CD_MASK_MLOOPUV, CD_DUPLICATE, s->totloop);
	CustomData_copy(&dm->polyData, &s->polyData, CD_MASK_MTEXPOLY, CD_DUPLICATE, s->totpoly);

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

/* modified from BKE_mesh_center_centroid */
bool BKE_fracture_shard_center_centroid(Shard *shard, float cent[3])
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
		poly_area = BKE_mesh_calc_poly_area(mpoly, shard->mloop + mpoly->loopstart, shard->mvert);
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
	Shard *s = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
	                                     dm->numVertData, dm->numPolyData, dm->numLoopData, true);
	s = BKE_custom_data_to_shard(s, dm);
	s->flag = SHARD_INTACT;
	s->shard_id = -2;
	return s;
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

Shard *BKE_create_fracture_shard(MVert *mvert, MPoly *mpoly, MLoop *mloop, int totvert, int totpoly, int totloop, bool copy)
{
	Shard *shard = MEM_mallocN(sizeof(Shard), __func__);
	shard->totvert = totvert;
	shard->totpoly = totpoly;
	shard->totloop = totloop;
	shard->cluster_colors = NULL;
	shard->neighbor_ids = NULL;
	shard->neighbor_count = 0;
	
	if (copy) {
		shard->mvert = MEM_mallocN(sizeof(MVert) * totvert, "shard vertices");
		shard->mpoly = MEM_mallocN(sizeof(MPoly) * totpoly, "shard polys");
		shard->mloop = MEM_mallocN(sizeof(MLoop) * totloop, "shard loops");
		memcpy(shard->mvert, mvert, sizeof(MVert) * totvert);
		memcpy(shard->mpoly, mpoly, sizeof(MPoly) * totpoly);
		memcpy(shard->mloop, mloop, sizeof(MLoop) * totloop);
	}
	else {
		shard->mvert = mvert;
		shard->mpoly = mpoly;
		shard->mloop = mloop;
	}

	shard->shard_id = -1;
	shard->setting_id = -1;
	shard->parent_id = -1;

	shard->flag = SHARD_INTACT;
	BKE_shard_calc_minmax(shard);

	BKE_fracture_shard_center_centroid(shard, shard->centroid);
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
                               char uv_layer[64])
{
	int i = 0;

	for (i = 0; i < expected_shards; i++) {
		Shard *s = NULL;
		Shard *s2 = NULL;
		Shard *t;
		int index = 0;

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
			break;
		}

		index = (int)(BLI_frand() * (t->totpoly - 1));
		if (index == 0) {
			index = 1;
		}

		printf("Bisecting cell %d...\n", i);
		printf("Bisecting cell %d...\n", i + 1);

		s = BKE_fracture_shard_bisect(*bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL,
		                              false, true, index, centroid, inner_material_index, uv_layer);
		s2 = BKE_fracture_shard_bisect(*bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL,
		                               true, false, index, centroid, inner_material_index, uv_layer);

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

			BLI_qsort_r(*tempresults, i + 1, sizeof(Shard *), shard_sortsize, &i);

			while ((*tempresults)[j] == NULL && j < (i + 1)) {
				/* ignore invalid shards */
				j++;
			}

			/* continue splitting if not all expected shards exist yet */
			if ((i + 2) < expected_shards) {
				*bm_parent = shard_to_bmesh((*tempresults)[j]);
				copy_v3_v3(centroid, (*tempresults)[j]->centroid);

				BKE_shard_free((*tempresults)[j], true);
				(*tempresults)[j] = NULL;
			}
			i++;
		}
	}
}

static void handle_boolean_fractal(Shard* s, Shard* t, int expected_shards, DerivedMesh* dm_parent, Object *obj, short inner_material_index,
                                   int num_cuts, float fractal, int num_levels, bool smooth,int parent_id, int* i, Shard ***tempresults,
                                   DerivedMesh **dm_p, char uv_layer[64])
{
	/* physics shard and fractalized shard, so we need to booleanize twice */
	/* and we need both halves, so twice again */
	Shard *s2 = NULL;
	int index = 0;
	int max_retries = 20;

	/*continue with "halves", randomly*/
	if ((*i) == 0) {
		*dm_p = dm_parent;
	}

	while (s == NULL || s2 == NULL) {

		float radius;
		float size[3];
		float eul[3];
		float loc[3];
		float one[3] = {1.0f, 1.0f, 1.0f};
		float matrix[4][4];

		/*make a plane as cutter*/
		BKE_object_dimensions_get(obj, size);
		radius = MAX3(size[0], size[1], size[2]);

		loc[0] = (BLI_frand() - 0.5f) * size[0];
		loc[1] = (BLI_frand() - 0.5f) * size[1];
		loc[2] = (BLI_frand() - 0.5f) * size[2];

		eul[0] = BLI_frand() * M_PI;
		eul[1] = BLI_frand() * M_PI;
		eul[2] = BLI_frand() * M_PI;

		//printf("(%f %f %f) (%f %f %f) \n", loc[0], loc[1], loc[2], eul[0], eul[1], eul[2]);

		loc_eul_size_to_mat4(matrix, loc, eul, one);

		/*visual shards next, fractalized cuts */
		s = BKE_fracture_shard_boolean(obj, *dm_p, t, inner_material_index, num_cuts,fractal, &s2, matrix, radius, smooth, num_levels, uv_layer);

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

			if (*dm_p != dm_parent && *dm_p != NULL) {
				(*dm_p)->needsFree = 1;
				(*dm_p)->release(*dm_p);
			}

			*dm_p = BKE_shard_create_dm(p, true);

			BKE_shard_free((*tempresults)[j], true);
			(*tempresults)[j] = NULL;
		}
		(*i)++; //XXX remember to "double" the shard amount....
	}
}

static bool handle_boolean_bisect(FracMesh *fm, Object *obj, int expected_shards, int algorithm, int parent_id, Shard **tempshards,
                                  DerivedMesh *dm_parent, BMesh* bm_parent, float obmat[4][4], short inner_material_index, int num_cuts,
                                  int num_levels, float fractal, int *i, bool smooth, Shard*** tempresults, DerivedMesh **dm_p, char uv_layer[64])
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
		s = BKE_fracture_shard_boolean(obj, dm_parent, t, inner_material_index, 0, 0.0f, NULL, NULL, 0.0f, false, 0, uv_layer);
	}
	else if (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
		handle_boolean_fractal(s, t, expected_shards, dm_parent, obj, inner_material_index, num_cuts, fractal,
		                       num_levels, smooth, parent_id, i, tempresults, dm_p, uv_layer);
	}
	else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL) {
		float co[3] = {0, 0, 0};
		printf("Bisecting cell %d...\n", *i);
		s = BKE_fracture_shard_bisect(bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FILL, false, true, 0, co, inner_material_index, uv_layer);
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
                             DerivedMesh **dm_parent, BMesh** bm_parent, Shard ***tempshards, Shard ***tempresults,
                             int active_setting, int num_settings)
{
	int i;
	Shard *s = NULL;
	int *skipmap = MEM_callocN(sizeof(int) * expected_shards, "skipmap");
	int *deletemap = MEM_callocN(sizeof(int) * fm->shard_count, "deletemap");

	if ((algorithm == MOD_FRACTURE_BOOLEAN) || (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL)) {
		MPoly *mpoly, *mp;
		int totpoly, i;

		*dm_parent = BKE_shard_create_dm(p, true);
		mpoly = (*dm_parent)->getPolyArray(*dm_parent);
		totpoly = (*dm_parent)->getNumPolys(*dm_parent);
		for (i = 0, mp = mpoly; i < totpoly; i++, mp++) {
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
		if (expected_shards <= fm->last_expected_shards)
		{
			copy_vn_i(deletemap, fm->shard_count, 1);
		}
		else
		{
			copy_vn_i(skipmap, expected_shards, 1);
		}

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
				Shard *t = fm->last_shards[j];
				float dist = len_squared_v3v3(n.co, cells[i].centroid);
				if (t != NULL && dist < max)
				{
					if (dist < 0.00001) {
						if (fabsf(cells[i].volume - t->raw_volume) < 0.00001) {
							//printf("Tagging skip: %d\n", i);
							skipmap[i] = true;
							deletemap[j] = false;
						}
						else
						{
							deletemap[j] = true;
							skipmap[i] = false;
						}
					}
					else
					{
						skipmap[i] = false;
						deletemap[j] = true;
					}
				}
			}
		}
	}

	//skipping /deletion pass
	for (i = 0; i < expected_shards; i++)
	{
		Shard *t = NULL;
		if (fm->cancel == 1) {
			break;
		}

		if (fm->last_shards && i < fm->shard_count)
			t = fm->last_shards[i];

		if (skipmap[i] /*&& ((t &&
		    t->setting_id == active_setting &&
		    t->shard_id > num_settings) || !t)*/)
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

			if (t->setting_id == active_setting /*|| t->shard_id < num_settings*/)
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

	fm->last_expected_shards = expected_shards;

	MEM_freeN(skipmap);
	MEM_freeN(deletemap);
}


/* parse the voro++ cell data */
static void parse_cells(cell *cells, int expected_shards, ShardID parent_id, FracMesh *fm, int algorithm, Object *obj, DerivedMesh *dm,
                        short inner_material_index, float mat[4][4], int num_cuts, float fractal, bool smooth, int num_levels, int mode,
                        bool reset, int active_setting, int num_settings, char uv_layer[64])
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

	if (mode == MOD_FRACTURE_PREFRACTURED && reset)
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
		if (!fm->last_shard_tree && (fm->shard_count > 0) && mode == MOD_FRACTURE_PREFRACTURED)
		{
			Shard *t;
			int i = 0;
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
					BLI_kdtree_insert(fm->last_shard_tree, i, t->raw_centroid);
				}
				fm->last_shards[i] = t;
				i++;
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

	do_prepare_cells(fm, cells, expected_shards, algorithm, p, &centroid, &dm_parent, &bm_parent, &tempshards, &tempresults, active_setting, num_settings);

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
		for (i = 0; i < expected_shards; i++) {
			bool stop = handle_boolean_bisect(fm, obj, expected_shards, algorithm, parent_id, tempshards, dm_parent,
			                      bm_parent, obmat, inner_material_index, num_cuts, num_levels, fractal,
			                      &i, smooth, &tempresults, &dm_p, uv_layer);
			//if (stop)
			//	break;
		}
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
			                   tempshards, &tempresults, uv_layer);
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

	//if (p->shard_id == -2)
	if (p && (parent_id == -2 /*|| parent_id == -1*/))
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

		//if (mode == MOD_FRACTURE_DYNAMIC)
		//	j = 1;

		if (fm->shard_map.last)
		{
			j += ((Shard*)(fm->shard_map.last))->shard_id;
		}
	}
	else
	{
		j = 0;
	}

	for (i = 0; i < expected_shards; i++) {
		Shard *s = tempresults[i];
		Shard *t = tempshards[i];

		if (s != NULL) {
			add_shard(fm, s, mat);
			s->shard_id += j+1;
			s->parent_id = parent_id;
			s->setting_id = active_setting;
			//printf("ADDED: %d %d %d\n", i, j, s->shard_id);
			if (parent_id > -1)
			{
				int i = 0;
				MVert *v;

				sub_v3_v3(s->centroid, pcentroid);
				for (i = 0, v = s->mvert; i < s->totvert; i++, v++)
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

	s = BKE_create_fracture_shard(mvert, mpoly, mloop, totvert, totpoly, totloop, false);

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
				float nvec[3], co1[3], co2[3];

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

static void do_intersect(FractureModifierData *fmd, Object* ob, Shard *t, short inner_mat_index,
                         bool is_zero, float mat[4][4], int **shard_counts, int* count,
                         int k, DerivedMesh **dm_parent, bool keep_other_shard)
{
	/*just keep appending items at the end here */
	MPoly *mpoly, *mp;
	int totpoly;
	Shard *parent = NULL;
	Shard *s = NULL, *s2 = NULL;
	int shards = 0, j = 0;

	if (is_zero == false) {
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
		s = BKE_fracture_shard_boolean(ob, *dm_parent, t, inner_mat_index, 0, 0.0f, &s2, NULL, 0.0f, false, 0, fmd->uvlayer_name);
	}
	else
	{
		s = BKE_fracture_shard_boolean(ob, *dm_parent, t, inner_mat_index, 0, 0.0f, NULL, NULL, 0.0f, false, 0, fmd->uvlayer_name);
	}

	//printf("Fractured: %d\n", k);

	if (s != NULL) {
		add_shard(fmd->frac_mesh, s, mat);
		shards++;
		s = NULL;
	}

	if (s2 != NULL) {
		add_shard(fmd->frac_mesh, s2, mat);
		shards++;
		s2 = NULL;
	}

	if ((is_zero && ob->derivedFinal == NULL) || !is_zero) {
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
	shards = 0;
}



static void intersect_shards_by_dm(FractureModifierData *fmd, DerivedMesh *d, Object *ob, Object *ob2, short inner_mat_index, float mat[4][4],
                                   bool keep_other_shard)
{
	Shard *t = NULL;
	int i = 0, count = 0, k = 0;
	float imat[4][4];
	int* shard_counts = NULL;
	bool is_zero = false;
	MVert *mv;
	DerivedMesh *dm_parent = NULL;

	t = BKE_create_fracture_shard(d->getVertArray(d), d->getPolyArray(d), d->getLoopArray(d),
	                              d->getNumVerts(d), d->getNumPolys(d), d->getNumLoops(d), true);
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
		do_intersect(fmd, ob, t, inner_mat_index, is_zero, mat, &shard_counts, &count, k, &dm_parent, keep_other_shard);
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
		fmd->reset_shards = false;
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
					BMesh *bm = BM_mesh_create(&bm_mesh_allocsize_default);
					DerivedMesh *dm = NULL;


					/*create stroke mesh */
					stroke_to_faces(fmd, &bm, gps, inner_material_index);
					dm = CDDM_from_bmesh(bm, true);
#if 0
					{
						/*create debug mesh*/
						Object* o;
						o = BKE_object_add(G.main, fmd->modifier.scene, OB_MESH);
						BM_mesh_bm_to_me(bm, o->data, true);
					}
#endif

					BM_mesh_free(bm);

					/*do intersection*/
					intersect_shards_by_dm(fmd, dm, obj, NULL, inner_material_index, mat, true);

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
	if (fmd->frac_algorithm == MOD_FRACTURE_BOOLEAN && fmd->cutter_group != NULL && obj->type == OB_MESH)
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

						intersect_shards_by_dm(fmd, dm, obj, ob, inner_material_index, mat, false);

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
					d = ob->derivedFinal;
					if (d == NULL) {
						d = CDDM_from_mesh(ob->data);
					}

					intersect_shards_by_dm(fmd, d, obj, ob, inner_material_index, mat, true);

					if (ob->derivedFinal == NULL)
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

void BKE_fracture_shard_by_points(FracMesh *fmesh, ShardID id, FracPointCloud *pointcloud, int algorithm, Object *obj, DerivedMesh *dm, short
                                  inner_material_index, float mat[4][4], int num_cuts, float fractal, bool smooth, int num_levels, int mode,
                                  bool reset, int active_setting, int num_settings, char uv_layer[64])
{
	int n_size = 8;
	
	Shard *shard;
	
	float min[3], max[3];
	float theta = 0.1f; /* TODO, container enlargement, because boundbox exact container and boolean might create artifacts */
	int p;
	
	container *voro_container;
	particle_order *voro_particle_order;
	cell *voro_cells;

#ifdef USE_DEBUG_TIMER
	double time_start;
#endif
	
	shard = BKE_shard_by_id(fmesh, id, dm);
	if (!shard || (shard->flag & SHARD_FRACTURED && (mode == MOD_FRACTURE_DYNAMIC))) {
		if (id == 0)
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

	
	voro_container = container_new(min[0], max[0], min[1], max[1], min[2], max[2],
	                               n_size, n_size, n_size, false, false, false,
	                               pointcloud->totpoints);
	
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

	/*Evaluate result*/
	parse_cells(voro_cells, pointcloud->totpoints, id, fmesh, algorithm, obj, dm, inner_material_index, mat,
	            num_cuts, fractal, smooth, num_levels, mode, reset, active_setting, num_settings, uv_layer);

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
		BLI_remlink_safe(&fm->shard_map, s);
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

static DerivedMesh* do_create(FractureModifierData *fmd, int num_verts, int num_loops, int num_polys, bool doCustomData)
{
	int shard_count = fmd->shards_to_islands ? BLI_listbase_count(&fmd->islandShards) : fmd->frac_mesh->shard_count;
	ListBase *shardlist;
	Shard *shard;

	int vertstart, polystart, loopstart;

	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;

	DerivedMesh *result = CDDM_new(num_verts, 0, 0, num_loops, num_polys);
	mverts = CDDM_get_verts(result);
	mloops = CDDM_get_loops(result);
	mpolys = CDDM_get_polys(result);

	if (doCustomData && shard_count > 0) {
		Shard *s;
		if (fmd->shards_to_islands) {
			s = (Shard *)fmd->islandShards.first;
		}
		else {
			s = (Shard *)fmd->frac_mesh->shard_map.first;
		}

		if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
		{
			/*keep old behavior for now for older modes */
			CustomData_merge(&s->vertData, &result->vertData, CD_MASK_MDEFORMVERT, CD_CALLOC, num_verts);
			CustomData_merge(&s->polyData, &result->polyData, CD_MASK_MTEXPOLY, CD_CALLOC, num_polys);
			CustomData_merge(&s->loopData, &result->loopData, CD_MASK_MLOOPUV, CD_CALLOC, num_loops);
		}
		else
		{
			/*just create new empty layers */
			CustomData_add_layer(&result->vertData, CD_MDEFORMVERT, CD_CALLOC, NULL, num_verts);
			CustomData_add_layer(&result->polyData, CD_MTEXPOLY, CD_CALLOC, NULL, num_polys);
			CustomData_add_layer(&result->loopData, CD_MLOOPUV, CD_CALLOC, NULL, num_loops);
		}
	}

	vertstart = polystart = loopstart = 0;
	if (fmd->shards_to_islands) {
		shardlist = &fmd->islandShards;
	}
	else {
		shardlist = &fmd->frac_mesh->shard_map;
	}

	for (shard = shardlist->first; shard; shard = shard->next)
	{
		MPoly *mp;
		MLoop *ml;
		int i;

		memcpy(mverts + vertstart, shard->mvert, shard->totvert * sizeof(MVert));
		memcpy(mpolys + polystart, shard->mpoly, shard->totpoly * sizeof(MPoly));

#if 0
		if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
		{
			for (i = 0, mv = mverts + vertstart; i < shard->totvert; i++, mv++)
			{
				sub_v3_v3(mv->co, shard->centroid);
				mul_v3_v3(mv->co, shard->raw_centroid);
				add_v3_v3(mv->co, shard->centroid);
			}
		}
#endif

		for (i = 0, mp = mpolys + polystart; i < shard->totpoly; ++i, ++mp) {
			/* adjust loopstart index */
			mp->loopstart += loopstart;
		}

		memcpy(mloops + loopstart, shard->mloop, shard->totloop * sizeof(MLoop));

		for (i = 0, ml = mloops + loopstart; i < shard->totloop; ++i, ++ml) {
			/* adjust vertex index */
			ml->v += vertstart;
		}

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

		vertstart += shard->totvert;
		polystart += shard->totpoly;
		loopstart += shard->totloop;
	}

	return result;
}

/* DerivedMesh */
static DerivedMesh *create_dm(FractureModifierData *fmd, bool doCustomData)
{
	Shard *s;
	int num_verts, num_polys, num_loops;
	DerivedMesh *result;
	
	num_verts = num_polys = num_loops = 0;

	if (fmd->shards_to_islands) {
		for (s = fmd->islandShards.first; s; s = s->next) {
			num_verts += s->totvert;
			num_polys += s->totpoly;
			num_loops += s->totloop;
		}
	}
	else {

		if (!fmd->frac_mesh)
			return NULL;

		for (s = fmd->frac_mesh->shard_map.first; s; s = s->next) {
			num_verts += s->totvert;
			num_polys += s->totpoly;
			num_loops += s->totloop;
		}
	}
	
	result = do_create(fmd, num_verts, num_loops, num_polys, doCustomData);

	CustomData_free(&result->edgeData, 0);
	CDDM_calc_edges(result);

	do_marking(fmd, result);
	
	result->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(result);
	return result;
}

void BKE_fracture_create_dm(FractureModifierData *fmd, bool doCustomData)
{
	DerivedMesh *dm_final = NULL;
	
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
	
	dm_final = create_dm(fmd, doCustomData);
	fmd->dm = dm_final;
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
	
	dm  = CDDM_new(s->totvert, 0, 0, s->totloop, s->totpoly);

	mverts = CDDM_get_verts(dm);
	mloops = CDDM_get_loops(dm);
	mpolys = CDDM_get_polys(dm);

	memcpy(mverts, s->mvert, s->totvert * sizeof(MVert));
	memcpy(mloops, s->mloop, s->totloop * sizeof(MLoop));
	memcpy(mpolys, s->mpoly, s->totpoly * sizeof(MPoly));

	CustomData_free(&dm->edgeData, 0);
	CDDM_calc_edges(dm);

	dm->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(dm);

	if (doCustomData) {
		if (s->totvert > 1) {
			BKE_copy_customdata_layers(&dm->vertData, &s->vertData, CD_MDEFORMVERT, s->totvert);
		}
		if (s->totloop > 0) {
			BKE_copy_customdata_layers(&dm->loopData, &s->loopData, CD_MLOOPUV, s->totloop);
		}
		if (s->totpoly > 0) {
			BKE_copy_customdata_layers(&dm->polyData, &s->polyData, CD_MTEXPOLY, s->totpoly);
		}
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
				BKE_free_constraints(fmd);
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
				BKE_free_constraints(fmd);
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

void BKE_match_vertex_coords(MeshIsland* mi, MeshIsland *par, Object *ob, int frame, bool is_parent)
{
	float loc[3] = {0.0f, 0.0f, 0.0f};
	float rot[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	int j = 0;

	float centr[3] = {0.0f, 0.0f, 0.0f};

	float mat[4][4];
	float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
	float qrot[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	invert_m4_m4(mat, ob->obmat);

	mi->locs[0] = loc[0] = par->locs[3*frame];
	mi->locs[1] = loc[1] = par->locs[3*frame+1];
	mi->locs[2] = loc[2] = par->locs[3*frame+2];

	mi->rots[0] = rot[0] = par->rots[4*frame];
	mi->rots[1] = rot[1] = par->rots[4*frame+1];
	mi->rots[2] = rot[2] = par->rots[4*frame+2];
	mi->rots[3] = rot[3] = par->rots[4*frame+3];

	mul_m4_v3(mat, loc);
	mat4_to_quat(quat, mat);


	if (par->id > 0)
	{
		mul_qt_qtqt(qrot, rot, par->rot);
		mul_qt_qtqt(qrot, quat, qrot);
	}
	else
	{
		invert_qt_qt(qrot, par->rot);
		mul_qt_qtqt(qrot, rot, qrot);
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

	for (mi = fmd->meshIslands.first; mi; mi = mi->next) {
		if (mi->participating_constraints != NULL && mi->participating_constraint_count > 0) {
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
	SpaceTransform trans;
	float mat[4][4], size[3];

	int totvert, totpoly, totloop, v;
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
	totvert = dm->getNumVerts(dm);
	totpoly = dm->getNumPolys(dm);
	totloop = dm->getNumLoops(dm);

	// create temp shard -> that necessary at all ?
	s = BKE_create_fracture_shard(mvert, mpoly, mloop, totvert, totpoly, totloop, true);

	//use this as size holder, and rawcentroid is the old ob location
	copy_v3_v3(s->impact_size, size);

	//compare centroid in worldspace with location
	mul_v3_m4v3(s->raw_centroid, target->obmat, s->centroid);

	for (v = 0, mv = s->mvert; v < s->totvert; v++, mv++)
	{
		mul_v3_v3(mv->co, size);

		//shrink the shard ? (and take centroid diff into account here, too)
		BLI_space_transform_apply(&trans, mv->co);

		//add_v3_v3(mv->co, target->loc);
		//sub_v3_v3(mv->co, s->raw_centroid);
	}

	//BLI_space_transform_apply(&trans, s->raw_centroid);
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

static MeshIsland* fracture_shard_to_island(FractureModifierData *fmd, Shard *s, int vertstart)
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
	if (fmd->modifier.scene->rigidbody_world)
	{
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

		mi->vertno[j * 3] = no[0];
		mi->vertno[j * 3 + 1] = no[1];
		mi->vertno[j * 3 + 2] = no[2];

		/* then eliminate centroid in vertex coords*/
		sub_v3_v3(mv->co, s->centroid);
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
	int vertstart = 0, totvert = 0, totpoly = 0, polystart = 0, matstart = 1, defstart = 0;
	MVert *mv = NULL;
	MPoly *mp = NULL, *mpoly = NULL, *ppoly = NULL, *pp = NULL;
	int i = 0, j = 0;
	MDeformVert *dvert = NULL;

	if (dm)
	{
		vertstart = dm->getNumVerts(dm);
		dm->needsFree = 1;
		dm->release(dm);
		dm = fmd->visible_mesh_cached = NULL;
	}

	fmd->visible_mesh_cached = create_dm(fmd, do_custom_data);

	if (!fmd->visible_mesh_cached)
		return 0;

	dm = fmd->visible_mesh_cached;
	mv = dm->getVertArray(dm);
	totvert = dm->getNumVerts(dm);
	mpoly = dm->getPolyArray(dm);
	dvert = CustomData_get_layer(&dm->vertData, CD_MDEFORMVERT);

	//update existing island's vert refs, if any...should have used indexes instead :S
	for (mi = fmd->meshIslands.first; mi; mi = mi->next)
	{
		//MVert *pvert = mi->physics_mesh->getVertArray(mi->physics_mesh);
		float inv_size[3] = {1.0f, 1.0f, 1.0f};
		Shard *s = BLI_findlink(&fmd->frac_mesh->shard_map, mi->id);
		if (!s)
			continue;

		inv_size[0] = 1.0f / s->impact_size[0];
		inv_size[1] = 1.0f / s->impact_size[1];
		inv_size[2] = 1.0f / s->impact_size[2];

		for (i = 0; i < mi->vertex_count; i++)
		{
			//just update pointers, dont need to reallocate something
			MVert *v = NULL;
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
		ppoly = s->mpoly;
		for (j = 0, mp = mpoly + polystart, pp = ppoly; j < totpoly; j++, mp++, pp++)
		{
			/* material index lookup and correction, avoid having the same material in different slots */
			int index = GET_INT_FROM_POINTER(BLI_ghash_lookup(fmd->material_index_map,
			                                 SET_INT_IN_POINTER(mp->mat_nr + matstart)));

			if (index > 0)
				index--;

			mp->mat_nr = index;
			//store this on physics mesh as well, so for being able to reload it from blend later (without
			// having a materialmap then)
			pp->mat_nr = index;
		}

		/* fortunately we know how many faces "belong" to this meshisland, too */
		polystart += totpoly;
		matstart += mi->totcol;
	}

	return vertstart;
}

short fracture_collect_defgrp(Object* o, Object* ob, short defstart, GHash** def_index_map)
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

short BKE_fracture_collect_materials(Object* o, Object* ob, short matstart, GHash** mat_index_map)
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
		int index = find_material_index(ob, (*matarar)[j]);
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

MeshIsland* BKE_fracture_mesh_island_add(FractureModifierData *fmd, Object* own, Object *target)
{
	MeshIsland *mi;
	Shard *s;
	int vertstart = 0;
	short totcol = 0, totdef = 0;
	float loc[3], quat[4];

	if (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL || own->type != OB_MESH || !own->data)
		return NULL;

	if (target->type != OB_MESH || !target->data)
		return NULL;

	s = fracture_object_to_shard(own, target);
	fracture_update_shards(fmd, s);

	vertstart = fmd->frac_mesh->progress_counter;
	fmd->frac_mesh->progress_counter += s->totvert;

	//hrm need to rebuild ALL islands since vertex refs are bonkers now after mesh has changed
	mi = fracture_shard_to_island(fmd, s, vertstart);

	//lets see whether we need to add loc here too XXX TODO
	mat4_to_loc_quat(loc, quat, target->obmat);

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

	totcol = BKE_fracture_collect_materials(target, own, (short)fmd->matstart, &fmd->material_index_map);
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

	mi->frame_count = 0;

	MEM_freeN(mi);
	mi = NULL;
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

void BKE_fracture_mesh_island_remove_all(FractureModifierData *fmd)
{
	MeshIsland *mi;

	//free all shards
	BKE_fracmesh_free(fmd->frac_mesh, true);
	MEM_freeN(fmd->frac_mesh);
	fmd->frac_mesh = NULL;

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
		mi1->participating_constraints = MEM_callocN(sizeof(RigidBodyShardCon *), "part_constraints_mi1");
		mi1->participating_constraint_count = 0;
	}

	mi1->participating_constraints = MEM_reallocN(mi1->participating_constraints, sizeof(RigidBodyShardCon *) * (mi1->participating_constraint_count + 1));
	mi1->participating_constraints[mi1->participating_constraint_count] = rbsc;
	mi1->participating_constraint_count++;

	if (mi2->participating_constraints == NULL) {
		mi2->participating_constraints = MEM_callocN(sizeof(RigidBodyShardCon *), "part_constraints_mi2");
		mi2->participating_constraint_count = 0;
	}

	mi2->participating_constraints = MEM_reallocN(mi2->participating_constraints, sizeof(RigidBodyShardCon *) * (mi2->participating_constraint_count + 1));
	mi2->participating_constraints[mi2->participating_constraint_count] = rbsc;
	mi2->participating_constraint_count++;

	return rbsc;
}

static void remove_participants(RigidBodyShardCon* con, MeshIsland *mi)
{
	RigidBodyShardCon **cons;
	/* Probably wrong, would need to shrink array size... listbase would have been better here */
	/* info not necessary so omit */
	int count = mi->participating_constraint_count;

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
