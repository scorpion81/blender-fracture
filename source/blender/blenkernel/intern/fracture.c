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
#include "BKE_DerivedMesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_global.h"
#include "BKE_mesh.h"
#include "BKE_object.h"

#include "BLI_listbase.h"
#include "BLI_math_vector.h"
#include "BLI_mempool.h"
#include "BLI_path_util.h"
#include "BLI_rand.h"
#include "BLI_sort.h"
#include "BLI_utildefines.h"

#include "DNA_fracture_types.h"
#include "DNA_group_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"

#include "bmesh.h"

#include "RBI_api.h"

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

#if 0
static float shard_size(Shard* s)
{
	float size[3], loc[3];
	shard_boundbox(s, loc, size);

	return size[0] * size[1] * size[2];
}
#endif


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

	CustomData_add_layer(&s->vertData, CD_MDEFORMVERT, CD_DUPLICATE, CustomData_get_layer(&dm->vertData, CD_MDEFORMVERT), s->totvert);
	CustomData_add_layer(&s->loopData, CD_MLOOPUV, CD_DUPLICATE, CustomData_get_layer(&dm->loopData, CD_MLOOPUV), s->totloop);
	CustomData_add_layer(&s->polyData, CD_MTEXPOLY, CD_DUPLICATE, CustomData_get_layer(&dm->polyData, CD_MTEXPOLY), s->totpoly);

	/*XXX TODO how do i use customdata PROPERLY ? */

	/*CustomData_copy(&dm->vertData, &s->vertData, CD_MASK_MDEFORMVERT, CD_CALLOC, s->totvert);
	   CustomData_copy_data(&dm->vertData, &s->vertData,
	                     0, 0, s->totvert);

	   CustomData_copy(&dm->loopData, &s->loopData, CD_MASK_MLOOPUV, CD_CALLOC, s->totloop);
	   CustomData_copy_data(&dm->loopData, &s->loopData,
	                     0, 0, s->totloop);

	   CustomData_copy(&dm->polyData, &s->polyData, CD_MASK_MTEXPOLY, CD_CALLOC, s->totpoly);
	   CustomData_copy_data(&dm->polyData, &s->polyData,
	                     0, 0, s->totpoly);*/

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


/*access shard directly by index / id*/
Shard *BKE_shard_by_id(FracMesh *mesh, ShardID id, DerivedMesh *dm) {
	if ((id < mesh->shard_count) && (id >= 0)) {
		//return mesh->shard_map[id];
		return (Shard *)BLI_findlink(&mesh->shard_map, id);
	}
	else if (id == -1)
	{
		/* create temporary shard covering the entire mesh */
		Shard *s = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
		                                     dm->numVertData, dm->numPolyData, dm->numLoopData, true);
		s = BKE_custom_data_to_shard(s, dm);
		s->flag = SHARD_INTACT;
		s->shard_id = -2;
		return s;
	}
	
	return NULL;
}

void BKE_get_shard_minmax(FracMesh *mesh, ShardID id, float min_r[3], float max_r[3], DerivedMesh *dm)
{
	Shard *shard = BKE_shard_by_id(mesh, id, dm);
	if (shard != NULL) {
		copy_v3_v3(min_r, shard->min);
		copy_v3_v3(max_r, shard->max);
	}

	if (shard->shard_id == -2)
	{
		BKE_shard_free(shard, true);
	}
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
	shard->flag |= SHARD_INTACT;
	BKE_shard_calc_minmax(shard);

	BKE_fracture_shard_center_centroid(shard, shard->centroid);

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
	
	return fmesh;
}


/* parse the voro++ cell data */
static void parse_cells(cell *cells, int expected_shards, ShardID parent_id, FracMesh *fm, int algorithm, Object *obj, DerivedMesh *dm, short inner_material_index, float mat[4][4], int num_cuts, float fractal, bool smooth, int num_levels)
{
	/*Parse voronoi raw data*/
	int i = 0;
	Shard *s = NULL, *p = BKE_shard_by_id(fm, parent_id, dm);
	float obmat[4][4]; /* use unit matrix for now */
	float centroid[3];
	BMesh *bm_parent = NULL;
	DerivedMesh *dm_parent = NULL;
	DerivedMesh *dm_p = NULL;
	Shard **tempshards;
	Shard **tempresults;
	int max_retries = 50;

	tempshards = MEM_mallocN(sizeof(Shard *) * expected_shards, "tempshards");
	tempresults = MEM_mallocN(sizeof(Shard *) * expected_shards, "tempresults");

	p->flag = 0;
	p->flag |= SHARD_FRACTURED;
	unit_m4(obmat);

	if ((algorithm == MOD_FRACTURE_BOOLEAN) || (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL)) {
		MPoly *mpoly, *mp;
		int totpoly, i;

		dm_parent = BKE_shard_create_dm(p, true);
		mpoly = dm_parent->getPolyArray(dm_parent);
		totpoly = dm_parent->getNumPolys(dm_parent);
		for (i = 0, mp = mpoly; i < totpoly; i++, mp++) {
			mp->flag &= ~ME_FACE_SEL;
		}
	}
	else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL ||
	         algorithm == MOD_FRACTURE_BISECT_FAST || algorithm == MOD_FRACTURE_BISECT_FAST_FILL)
	{
		bm_parent = shard_to_bmesh(p);
		copy_v3_v3(centroid, p->centroid);
	}

	for (i = 0; i < expected_shards; i++) {
		if (fm->cancel == 1) {
			break;
		}

		printf("Parsing shard: %d\n", i);
		s = parse_cell(cells[i]);
		tempshards[i] = s;
		tempresults[i] = NULL;
		fm->progress_counter++;
	}

	if (algorithm != MOD_FRACTURE_BISECT_FAST && algorithm != MOD_FRACTURE_BISECT_FAST_FILL) {
		for (i = 0; i < expected_shards; i++) {
			Shard *t;
			if (fm->cancel == 1)
				break;

			printf("Processing shard: %d\n", i);
			t = tempshards[i];

			if (t != NULL) {
				t->parent_id = parent_id;
				t->flag = SHARD_INTACT;
			}

			if (t == NULL || t->totvert == 0 || t->totloop == 0 || t->totpoly == 0) {
				/* invalid shard, stop parsing */
				break;
			}

			/* XXX TODO, need object for material as well, or atleast a material index... */
			if (algorithm == MOD_FRACTURE_BOOLEAN) {
				s = BKE_fracture_shard_boolean(obj, dm_parent, t, inner_material_index, 0, 0.0f, NULL, NULL, 0.0f, false, 0);
			}
			else if (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
				/* physics shard and fractalized shard, so we need to booleanize twice */
				/* and we need both halves, so twice again */
				Shard *s2 = NULL;
				int index = 0;

				/*continue with "halves", randomly*/
				if (i == 0) {
					dm_p = dm_parent;
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
					s = BKE_fracture_shard_boolean(obj, dm_p, t, inner_material_index, num_cuts,fractal, &s2, matrix, radius, smooth, num_levels);

					if (index < max_retries)
					{
						printf("Retrying...%d\n", index);
						index++;
					}
					else if (s == NULL || s2 == NULL)
					{
						i++;
						break;
					}
				}

				if ((s != NULL) && (s2 != NULL)) {
					int j = 0; //, k = 0;
					//float size_max = 0;

					s->parent_id = parent_id;
					s->flag = SHARD_INTACT;
					tempresults[i+1] = s;

					s2->parent_id = parent_id;
					s2->flag = SHARD_INTACT;
					tempresults[i] = s2;

					BLI_qsort_r(tempresults, i + 1, sizeof(Shard *), shard_sortsize, &i);
					while (tempresults[j] == NULL && j < (i + 1)) {
						/* ignore invalid shards */
						j++;
					}
#if 0
					/*search for biggest shard and keep aligned with fractalized one*/
					for (k = 0; k < (i+2); k++)
					{
						float size;
						if (tempresults[k] != NULL)
						{
							size = shard_size(tempresults[k]);
							if (size > size_max)
							{
								j = k;
								size_max = size;
							}
						}
					}
#endif
					/* continue splitting if not all expected shards exist yet */
					if ((i + 2) < expected_shards) {

						Shard *p = tempresults[j];

						if (dm_p != dm_parent && dm_p != NULL) {
							dm_p->needsFree = 1;
							dm_p->release(dm_p);
						}

						dm_p = BKE_shard_create_dm(p, true);

						BKE_shard_free(tempresults[j], true);
						tempresults[j] = NULL;

					}

					i++; //XXX remember to "double" the shard amount....
				}
			}
			else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL) {
				float co[3] = {0, 0, 0};
				printf("Bisecting cell %d...\n", i);
				s = BKE_fracture_shard_bisect(bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FILL, false, true, 0, co, inner_material_index);
			}
			else {
				/* do not fracture case */
				s = t;
			}

			if ((s != NULL) && (algorithm != MOD_FRACTURE_BOOLEAN_FRACTAL)) {
				s->parent_id = parent_id;
				s->flag = SHARD_INTACT;

				tempresults[i] = s;
			}

			fm->progress_counter++;
		}
	}
	else {
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

			s = BKE_fracture_shard_bisect(bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL, false, true, index, centroid, inner_material_index);
			s2 = BKE_fracture_shard_bisect(bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL, true, false, index, centroid, inner_material_index);

			if (s != NULL && s2 != NULL && tempresults != NULL) {
				int j = 0;

				fm->progress_counter++;

				s->parent_id = parent_id;
				s->flag = SHARD_INTACT;

				s2->parent_id = parent_id;
				s2->flag = SHARD_INTACT;

				if (bm_parent != NULL) {
					BM_mesh_free(bm_parent);
					bm_parent = NULL;
				}

				if (dm_parent != NULL) {
					dm_parent->needsFree = 1;
					dm_parent->release(dm_parent);
					dm_parent = NULL;
				}
				tempresults[i] = s;
				tempresults[i + 1] = s2;

				BLI_qsort_r(tempresults, i + 1, sizeof(Shard *), shard_sortsize, &i);

				while (tempresults[j] == NULL && j < (i + 1)) {
					/* ignore invalid shards */
					j++;
				}

				/* continue splitting if not all expected shards exist yet */
				if ((i + 2) < expected_shards) {
					bm_parent = shard_to_bmesh(tempresults[j]);
					copy_v3_v3(centroid, tempresults[j]->centroid);

					BKE_shard_free(tempresults[j], true);
					tempresults[j] = NULL;
				}
				i++;
			}
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

	if (dm_p != NULL) {
		dm_p->needsFree = 1;
		dm_p->release(dm_p);
		dm_p = NULL;
	}


	if (p->shard_id == -2)
	{
		BKE_shard_free(p, true);
	}

	fm->shard_count = 0; /* may be not matching with expected shards, so reset... did increment this for
	                      *progressbar only */

	/*blocks are here because of deleted unnecessary if conditions, kept for convenience with declaring local variables */
	{
		{
			for (i = 0; i < expected_shards; i++) {
				Shard *s = tempresults[i];

				if (s != NULL) {
					add_shard(fm, s, mat);
				}

				{
					Shard *t = tempshards[i];
					if (t != NULL) {
						BKE_shard_free(t, false);
					}
				}
			}
		}
		MEM_freeN(tempshards);
		MEM_freeN(tempresults);
	}
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
//	int shard_id;

//	shard_id = c.index;

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

	s->neighbor_ids = neighbors;
	s->neighbor_count = totpoly;
	copy_v3_v3(s->centroid, centr);

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

void BKE_fracture_shard_by_planes(FractureModifierData *fmd, Object *obj, short inner_material_index, float mat[4][4])
{
	DerivedMesh *dm_parent = NULL;
	int shards = 0;

	if (fmd->frac_algorithm == MOD_FRACTURE_BOOLEAN && fmd->cutter_group != NULL && obj->type == OB_MESH)
	{
		GroupObject* go;
		float imat[4][4];

		invert_m4_m4(imat, obj->obmat);

		for (go = fmd->cutter_group->gobject.first; go; go = go->next)
		{
			Object* ob = go->ob;
			Shard *s2 = NULL;
			Shard *t = NULL;
			Shard *s = NULL;

			/*simple case....one cutter object per object*/
			if (ob->type == OB_MESH)
			{
				int i = 0, j = 0, k = 0, count = 0;
				DerivedMesh *d;
				MVert *mv;

				d = ob->derivedFinal;
				if (d == NULL)
				{
					d = CDDM_from_mesh(ob->data);
				}

				t = BKE_create_fracture_shard(d->getVertArray(d), d->getPolyArray(d), d->getLoopArray(d),
				                              d->getNumVerts(d), d->getNumPolys(d), d->getNumLoops(d), true);
				t = BKE_custom_data_to_shard(t, d);

				/*complicated cases, self intersecting planes which could be separated by loose first */
				/*omit for now */

				for (i = 0, mv = t->mvert; i < t->totvert; mv++, i++){
					mul_m4_v3(ob->obmat, mv->co);
					mul_m4_v3(imat, mv->co);
				}

				count = fmd->frac_mesh->shard_count;

				if (count == 0)
				{
					if (obj->derivedFinal != NULL)
					{
						dm_parent = CDDM_copy(obj->derivedFinal);
					}

					if (dm_parent == NULL) {
						dm_parent = CDDM_from_mesh(obj->data);
					}

					count = 1;
				}

				for (k = 0; k < count; k++)
				{
					/*just keep appending items at the end here */

					MPoly *mpoly, *mp;
					int totpoly, totvert;
					Shard *parent = NULL;

					if (count > 1)
					{
						parent = BLI_findlink(&fmd->frac_mesh->shard_map, k);
						dm_parent = BKE_shard_create_dm(parent, true);
					}

					totvert = dm_parent->getNumVerts(dm_parent);
					mpoly = dm_parent->getPolyArray(dm_parent);
					totpoly = dm_parent->getNumPolys(dm_parent);
					for (i = 0, mp = mpoly; i < totpoly; i++, mp++) {
						mp->flag &= ~ME_FACE_SEL;
					}

					s = BKE_fracture_shard_boolean(obj, dm_parent, t, inner_material_index, 0, 0.0f, &s2, NULL, 0.0f, false, 0);
					if (s != NULL)
					{
						add_shard(fmd->frac_mesh, s, mat);
						shards++;
						s = NULL;
					}

					if (s2 != NULL)
					{
						add_shard(fmd->frac_mesh, s2, mat);
						shards++;
						s2 = NULL;
					}

					if ((count == 1 && ob->derivedFinal == NULL) || (count > 1))
					{
						if (count == 1)
						{
							count = 0;
						}

						dm_parent->needsFree = 1;
						dm_parent->release(dm_parent);
						dm_parent = NULL;
					}

					//shards--;
				}

				count = fmd->frac_mesh->shard_count;

				/*new count - shards = shards to remove*/
				for (k = 0; k < count-shards; k++)
				{
					/*clean up old entries here to avoid unnecessary shards*/
					Shard *first = fmd->frac_mesh->shard_map.first;
					BLI_remlink_safe(&fmd->frac_mesh->shard_map,first);
					BKE_shard_free(first, true);
					first = NULL;
					fmd->frac_mesh->shard_count--;
				}

				shards = 0;

				BKE_shard_free(t, true);
				if (ob->derivedFinal == NULL)
				{	/*was copied before */
					d->needsFree = 1;
					d->release(d);
					d = NULL;
				}

				j++;
			}
		}
	}
}

void BKE_fracture_shard_by_points(FracMesh *fmesh, ShardID id, FracPointCloud *pointcloud, int algorithm, Object *obj, DerivedMesh *dm, short
                                  inner_material_index, float mat[4][4], int num_cuts, float fractal, bool smooth, int num_levels) {
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
	if (!shard || shard->flag & SHARD_FRACTURED)
		return;

	
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
	parse_cells(voro_cells, pointcloud->totpoints, id, fmesh, algorithm, obj, dm, inner_material_index, mat, num_cuts, fractal, smooth, num_levels);

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
}


/* DerivedMesh */
static DerivedMesh *create_dm(FractureModifierData *fmd, bool doCustomData)
{
	int shard_count = fmd->shards_to_islands ? BLI_listbase_count(&fmd->islandShards) : fmd->frac_mesh->shard_count;
	ListBase *shardlist;
	Shard *shard, *s;
	
	int num_verts, num_polys, num_loops;
	int vertstart, polystart, loopstart;
	DerivedMesh *result;
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;
	
	num_verts = num_polys = num_loops = 0;

	if (fmd->shards_to_islands) {
		for (s = fmd->islandShards.first; s; s = s->next) {
			num_verts += s->totvert;
			num_polys += s->totpoly;
			num_loops += s->totloop;
		}
	}
	else {
		for (s = fmd->frac_mesh->shard_map.first; s; s = s->next) {
			num_verts += s->totvert;
			num_polys += s->totpoly;
			num_loops += s->totloop;
		}
	}
	
	result = CDDM_new(num_verts, 0, 0, num_loops, num_polys);
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

		CustomData_merge(&s->vertData, &result->vertData, CD_MASK_MDEFORMVERT, CD_CALLOC, num_verts);
		CustomData_merge(&s->polyData, &result->polyData, CD_MASK_MTEXPOLY, CD_CALLOC, num_polys);
		CustomData_merge(&s->loopData, &result->loopData, CD_MASK_MLOOPUV, CD_CALLOC, num_loops);
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
	
	CustomData_free(&result->edgeData, 0);
	CDDM_calc_edges(result);

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

	CDDM_calc_edges(dm);

	dm->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(dm);

	if (doCustomData) {
		if (s->totvert > 1) {
			CustomData_add_layer(&dm->vertData, CD_MDEFORMVERT, CD_DUPLICATE, CustomData_get_layer(&s->vertData, CD_MDEFORMVERT), s->totvert);
		}
		if (s->totloop > 0) {
			CustomData_add_layer(&dm->loopData, CD_MLOOPUV, CD_DUPLICATE, CustomData_get_layer(&s->loopData, CD_MLOOPUV), s->totloop);
		}
		if (s->totpoly > 0) {
			CustomData_add_layer(&dm->polyData, CD_MTEXPOLY, CD_DUPLICATE, CustomData_get_layer(&s->polyData, CD_MTEXPOLY), s->totpoly);
		}
	}

	return dm;
}
