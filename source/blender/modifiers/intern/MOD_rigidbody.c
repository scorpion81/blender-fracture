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
 * along with this program; if not, write to the Free Software  Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2005 by the Blender Foundation.
 * All rights reserved.
 *
 * Contributor(s): Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

/** \file blender/modifiers/intern/MOD_rigidbody.c
 *  \ingroup modifiers
 *
 * Rigid Body modifier
 *
 * Creates rigidbodies from mesh islands
 */

#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_math.h"
#include "BLI_listbase.h"
#include "BLI_kdtree.h"
#include "BLI_edgehash.h"
#include "BLI_ghash.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"
#include "BKE_rigidbody.h"
#include "BKE_pointcache.h"
#include "BKE_scene.h"
#include "BKE_object.h"
#include "BKE_particle.h"
#include "BKE_group.h"
#include "BKE_depsgraph.h"
#include "BKE_global.h"
#include "BKE_library.h"
#include "BKE_main.h"
#include "BKE_submesh.h"

#include "bmesh.h"

#include "DNA_object_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"
#include "DNA_listBase.h"
#include "DNA_group_types.h"

#include "../../rigidbody/RBI_api.h"
#include "PIL_time.h"

//#ifdef _OPENMP
//#include <omp.h>
//#endif

int vol_check(RigidBodyModifierData *rmd, MeshIsland* mi);
void destroy_compound(RigidBodyModifierData* rmd, Object *ob, MeshIsland* mi, float cfra);
void buildCompounds(RigidBodyModifierData *rmd, Object *ob);

static void initData(ModifierData *md)
{
	RigidBodyModifierData *rmd = (RigidBodyModifierData *) md;
	rmd->visible_mesh = NULL;
	rmd->visible_mesh_cached = NULL;
	rmd->refresh = TRUE;
	zero_m4(rmd->origmat);
	rmd->breaking_threshold = 10.0f;
	rmd->use_constraints = FALSE;
	rmd->constraint_group = NULL;
	rmd->contact_dist = 1.0f;
	rmd->group_breaking_threshold = 1.0f;
	rmd->group_contact_dist = 0.0001f;
	rmd->mass_dependent_thresholds = FALSE;
	rmd->auto_merge = FALSE;
	rmd->sel_indexes = NULL;
	rmd->sel_counter = 0;
	rmd->auto_merge_dist = 0.0001f;
	rmd->inner_constraint_type = RBC_TYPE_FIXED;
	rmd->outer_constraint_type = RBC_TYPE_FIXED;
	rmd->outer_constraint_location = MOD_RIGIDBODY_CENTER;
	rmd->outer_constraint_pattern = MOD_RIGIDBODY_SELECTED_TO_ACTIVE;
	rmd->idmap = NULL;
	rmd->explo_shared = FALSE;
	rmd->constraint_limit = 0;
	rmd->dist_dependent_thresholds = FALSE;
	rmd->contact_dist_meaning = MOD_RIGIDBODY_CENTROIDS;
	rmd->breaking_distance = 0;
	rmd->breaking_angle = 0;
	rmd->breaking_percentage = 0; //disable by default
	rmd->use_both_directions = FALSE;
	rmd->use_proportional_distance = FALSE;
	rmd->use_proportional_limit = FALSE;
	rmd->max_vol = 0;
	rmd->cell_size = 1.0f;
	rmd->refresh_constraints = FALSE;
	rmd->split = destroy_compound;
	rmd->join = buildCompounds;
	rmd->use_cellbased_sim = FALSE;
	rmd->framecount = 0;
	rmd->framemap = NULL;
	rmd->disable_self_collision = TRUE;
	rmd->cluster_breaking_threshold = 1000.0f;
	rmd->use_proportional_solver_iterations = FALSE;
	rmd->solver_iterations_override = 0;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	RigidBodyModifierData *rmd  = (RigidBodyModifierData *)md;
	RigidBodyModifierData *trmd = (RigidBodyModifierData *)target;

	zero_m4(trmd->origmat);
	//trmd->refresh = rmd->refresh;
	trmd->auto_merge = rmd->auto_merge;
	trmd->breaking_threshold = rmd->breaking_threshold;
	trmd->use_constraints = rmd->use_constraints;
	trmd->constraint_group = rmd->constraint_group;
	trmd->contact_dist = rmd->contact_dist;
	trmd->group_breaking_threshold = rmd->group_breaking_threshold;
	trmd->group_contact_dist = rmd->group_contact_dist;
	trmd->mass_dependent_thresholds = rmd->mass_dependent_thresholds;
	trmd->sel_indexes = MEM_dupallocN(rmd->sel_indexes);
	trmd->sel_counter = rmd->sel_counter;
	trmd->explo_shared = rmd->explo_shared;

	trmd->visible_mesh = NULL;
	trmd->visible_mesh_cached = NULL;
	trmd->idmap = NULL; 
	trmd->meshIslands.first = NULL;
	trmd->meshIslands.last = NULL;
	trmd->meshConstraints.first = NULL;
	trmd->meshConstraints.last = NULL;

	trmd->auto_merge_dist = rmd->auto_merge_dist;
	trmd->refresh = FALSE;
	trmd->constraint_limit = rmd->constraint_limit;
	trmd->breaking_angle = rmd->breaking_angle;
	trmd->breaking_distance= rmd->breaking_distance;
	trmd->breaking_percentage = rmd->breaking_percentage;
	trmd->cell_size = rmd->cell_size;
	trmd->contact_dist_meaning = rmd->contact_dist_meaning;
	trmd->use_cellbased_sim = rmd->use_cellbased_sim;
	trmd->use_experimental = rmd->use_experimental;
	trmd->use_both_directions = rmd->use_both_directions;
	trmd->dist_dependent_thresholds = rmd->dist_dependent_thresholds;
	trmd->refresh_constraints = FALSE;
	trmd->outer_constraint_location = rmd->outer_constraint_location;
	trmd->outer_constraint_pattern = rmd->outer_constraint_pattern;
	trmd->outer_constraint_type = rmd->outer_constraint_type;
	trmd->inner_constraint_type = rmd->inner_constraint_type;
	
	trmd->framecount = 0;
	trmd->framemap = NULL;
	trmd->join = buildCompounds;
	trmd->split = destroy_compound;
	trmd->disable_self_collision = rmd->disable_self_collision;
	trmd->cluster_breaking_threshold = rmd->cluster_breaking_threshold;
	trmd->use_proportional_solver_iterations = rmd->use_proportional_solver_iterations;
	trmd->solver_iterations_override = rmd->solver_iterations_override;
}

void freeMeshIsland(RigidBodyModifierData* rmd, MeshIsland* mi)
{
	int i;
	
	if (mi->physics_mesh) {
		DM_release(mi->physics_mesh);
		MEM_freeN(mi->physics_mesh);
		mi->physics_mesh = NULL;
	}
	if (mi->rigidbody) {
		//BKE_rigidbody_remove_shard(rmd->modifier.scene, mi);
		MEM_freeN(mi->rigidbody);
		mi->rigidbody = NULL;
	}
	
	if (!rmd->explo_shared || mi->compound_count > 0) {
		if (mi->vertco /*&& rmd->refresh == FALSE*/) {
			MEM_freeN(mi->vertco);
			mi->vertco = NULL;
		}
		if (mi->vertices /*&& rmd->refresh == FALSE*/) {
			MEM_freeN(mi->vertices);
			mi->vertices = NULL; //borrowed only !!!
		}
	}
	
	if (mi->vertices_cached)
	{
		MEM_freeN(mi->vertices_cached);
		mi->vertices_cached = NULL;
	}
	
	if (mi->compound_count > 0)
	{
		MEM_freeN(mi->compound_children);
		mi->compound_count = 0;
		mi->compound_parent = NULL;
	}

	if (mi->bb != NULL) {
		MEM_freeN(mi->bb);
		mi->bb = NULL;
	}
	
	if (mi->participating_constraints != NULL)
	{
		MEM_freeN(mi->participating_constraints);
		mi->participating_constraints = NULL;
		mi->participating_constraint_count = 0;
	}
	
	MEM_freeN(mi);
	mi = NULL;
}

static void freeData(ModifierData* md)
{
	MeshIsland *mi;
	RigidBodyShardCon *rbsc;
	RigidBodyModifierData *rmd = (RigidBodyModifierData*)md;
	int i;

	if (!rmd->refresh_constraints)
	{
		while (rmd->meshIslands.first) {
			mi = rmd->meshIslands.first;
			BLI_remlink(&rmd->meshIslands, mi);
			freeMeshIsland(rmd, mi);
			mi = NULL;
		}
			
		rmd->meshIslands.first = NULL;
		rmd->meshIslands.last = NULL;
	
		if (!rmd->explo_shared) {
			if (rmd->visible_mesh != NULL)
			{
				BM_mesh_free(rmd->visible_mesh);
				rmd->visible_mesh = NULL;
			}
		}
	
		if (rmd->sel_indexes != NULL && rmd->refresh == FALSE) {
			for (i = 0; i < rmd->sel_counter; i++) {
				MEM_freeN(rmd->sel_indexes[i]);
				rmd->sel_indexes[i] = NULL;
			}
			MEM_freeN(rmd->sel_indexes);
			rmd->sel_indexes = NULL;
			rmd->sel_counter = 0;
		}
	
		if (rmd->idmap != NULL) {
			BLI_ghash_free(rmd->idmap, NULL, NULL);
			rmd->idmap = NULL;
		}
		
		if (!rmd->refresh)
		{
			if (rmd->framemap)
			{
				MEM_freeN(rmd->framemap);
				rmd->framemap = NULL;
				rmd->framecount = 0;
			}
		}
	}
	
	if (rmd->refresh_constraints || !rmd->refresh)
	{
		for (mi = rmd->meshIslands.first; mi; mi = mi->next)
		{
			if (mi->participating_constraints != NULL)
			{
				MEM_freeN(mi->participating_constraints);
				mi->participating_constraints = NULL;
				mi->participating_constraint_count = 0;
			}
		}
	}
	
	if (rmd->visible_mesh_cached)
	{
		DM_release(rmd->visible_mesh_cached);
		MEM_freeN(rmd->visible_mesh_cached);
		rmd->visible_mesh_cached = NULL;
	}
	
	while (rmd->cells.first)
	{
		NeighborhoodCell* c = rmd->cells.first;
		BLI_remlink(&rmd->cells, c);
		MEM_freeN(c);
	}
	rmd->cells.first = NULL;
	rmd->cells.last = NULL;

	while (rmd->meshConstraints.first) {
		rbsc = rmd->meshConstraints.first;
		BLI_remlink(&rmd->meshConstraints, rbsc);
		//BKE_rigidbody_remove_shard_con(md->scene, rbsc);
		MEM_freeN(rbsc);
		rbsc = NULL;
	}
	
	rmd->meshConstraints.first = NULL;
	rmd->meshConstraints.last = NULL;
}

static int dm_minmax(DerivedMesh* dm, float min[3], float max[3])
{
	int verts = dm->getNumVerts(dm);
	MVert *mverts = dm->getVertArray(dm);
	MVert *mvert;
	int i = 0;

	INIT_MINMAX(min, max);
	for (i = 0; i < verts; i++) {
		mvert = &mverts[i];
		minmax_v3v3_v3(min, max, mvert->co);
	}

	return (verts != 0);
}

float bbox_vol(BoundBox* bb)
{
	float x[3], y[3], z[3];
	
	sub_v3_v3v3(x, bb->vec[4], bb->vec[0]);
	sub_v3_v3v3(y, bb->vec[3], bb->vec[0]);
	sub_v3_v3v3(z, bb->vec[1], bb->vec[0]);
	
	return len_v3(x) * len_v3(y) * len_v3(z);
}

int vol_check(RigidBodyModifierData *rmd, MeshIsland* mi)
{
	float vol, cellvol;
	vol = bbox_vol(mi->bb);
	cellvol = rmd->cell_size * rmd->cell_size * rmd->cell_size;
	
	return vol > cellvol;
}

void bbox_dim(BoundBox* bb, float dim[3])
{
	float x[3], y[3], z[3];
	
	sub_v3_v3v3(x, bb->vec[4], bb->vec[0]);
	sub_v3_v3v3(y, bb->vec[3], bb->vec[0]);
	sub_v3_v3v3(z, bb->vec[1], bb->vec[0]);
	
	dim[0] = len_v3(x);
	dim[1] = len_v3(y);
	dim[2] = len_v3(z);
}

int BM_calc_center_centroid(BMesh *bm, float cent[3], int tagged)
{
	BMFace *f;
	BMIter iter;
	float face_area;
	float total_area = 0.0f;
	float face_cent[3];

	zero_v3(cent);

	/* calculate a weighted average of face centroids */
	BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
		if (BM_elem_flag_test(f, BM_ELEM_TAG) || !tagged) {
			BM_face_calc_center_mean (f, face_cent);
			face_area = BM_face_calc_area(f);

			madd_v3_v3fl(cent, face_cent, face_area);
			total_area += face_area;
		}
	}
	/* otherwise we get NAN for 0 polys */
	if (bm->totface) {
		mul_v3_fl(cent, 1.0f / total_area);
	}
	else if (bm->totvert == 1)
	{
		copy_v3_v3(cent, BM_vert_at_index(bm, 0)->co);
	}

	return (bm->totface != 0);
}


/*static void DM_mesh_boundbox(DerivedMesh* bm, float r_loc[3], float r_size[3])
{
	float min[3], max[3];
	float mloc[3], msize[3];

	if (!r_loc) r_loc = mloc;
	if (!r_size) r_size = msize;

	INIT_MINMAX(min, max);
	if (!dm_minmax(bm, min, max)) {
		min[0] = min[1] = min[2] = -1.0f;
		max[0] = max[1] = max[2] = 1.0f;
	}

	mid_v3_v3v3(r_loc, min, max);

	r_size[0] = (max[0] - min[0]) / 2.0f;
	r_size[1] = (max[1] - min[1]) / 2.0f;
	r_size[2] = (max[2] - min[2]) / 2.0f;
}*/

static int BM_mesh_minmax(BMesh *bm, float r_min[3], float r_max[3], int tagged)
{
	BMVert* v;
	BMIter iter;
	INIT_MINMAX(r_min, r_max);
	BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
		if ((tagged && BM_elem_flag_test(v, BM_ELEM_SELECT)) || (!tagged))
		{
			minmax_v3v3_v3(r_min, r_max, v->co);
		}
	}

	//BM_mesh_normals_update(bm, FALSE);
	return (bm->totvert != 0);
}
	
static ExplodeModifierData *findPrecedingExploModifier(Object *ob, RigidBodyModifierData *rmd)
{
	ModifierData *md;
	ExplodeModifierData *emd = NULL;

	for (md = ob->modifiers.first; rmd != md; md = md->next) {
		if (md->type == eModifierType_Explode) {
			emd = (ExplodeModifierData *) md;
			if (emd->mode == eFractureMode_Cells || 1) {
				return emd;
			}
			else
			{
				return NULL;
			}
		}
	}
	return emd;
}

static float mesh_separate_tagged(RigidBodyModifierData* rmd, Object *ob, BMVert** v_tag, int v_count, float** startco, BMesh* bm_work /*, MeshIsland*** mi_array, int* mi_count*/)
{
	BMesh *bm_new;
	BMesh *bm_old = bm_work;
	MeshIsland *mi;
	float centroid[3], dummyloc[3], rot[4], min[3], max[3], vol = 0;
	BMVert* v;
	BMIter iter;
	DerivedMesh *dm = NULL;
	
	//*mi_array = MEM_reallocN(*mi_array, sizeof(MeshIsland*) * (*mi_count+1));
	bm_new = BM_mesh_create(&bm_mesh_allocsize_default);
	BM_mesh_elem_toolflags_ensure(bm_new);  /* needed for 'duplicate' bmo */
	
	CustomData_copy(&bm_old->vdata, &bm_new->vdata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_old->edata, &bm_new->edata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_old->ldata, &bm_new->ldata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_old->pdata, &bm_new->pdata, CD_MASK_BMESH, CD_CALLOC, 0);
	
	CustomData_bmesh_init_pool(&bm_new->vdata, bm_mesh_allocsize_default.totvert, BM_VERT);
	CustomData_bmesh_init_pool(&bm_new->edata, bm_mesh_allocsize_default.totedge, BM_EDGE);
	CustomData_bmesh_init_pool(&bm_new->ldata, bm_mesh_allocsize_default.totloop, BM_LOOP);
	CustomData_bmesh_init_pool(&bm_new->pdata, bm_mesh_allocsize_default.totface, BM_FACE);
	
	BMO_op_callf(bm_old, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				 "duplicate geom=%hvef dest=%p", BM_ELEM_TAG, bm_new);
	
	//can delete now since we are on working copy
	/*BMO_op_callf(bm_old, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				 "delete geom=%hvef context=%i", BM_ELEM_TAG, DEL_FACES);*/
	
	BM_calc_center_centroid(bm_new, centroid, FALSE);
	
	//use unmodified coords for bbox here
	//BM_mesh_minmax(bm_new, min, max);
	
	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
		//then eliminate centroid in vertex coords ?
		sub_v3_v3(v->co, centroid);
	}
	
	
	// add 1 MeshIsland
	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
	BLI_addtail(&rmd->meshIslands, mi);
	
	mi->vertices = v_tag;
	mi->vertco = *startco;
	mi->compound_children = NULL;
	mi->compound_count = 0;
	mi->compound_parent = NULL;
	zero_v3(mi->start_co);
	
	BM_mesh_normals_update(bm_new);
	BM_mesh_minmax(bm_new, min, max, FALSE);
	dm = CDDM_from_bmesh(bm_new, true);
	BM_mesh_free(bm_new);
	bm_new = NULL;
	
	mi->physics_mesh = dm;
	mi->vertex_count = v_count;
	copy_v3_v3(mi->centroid, centroid);
	mat4_to_loc_quat(dummyloc, rot, ob->obmat);
	copy_v3_v3(mi->rot, rot);
	mi->parent_mod = rmd;
	mi->bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(mi->bb, min, max);
	mi->participating_constraints = NULL;
	mi->participating_constraint_count = 0;
	mi->destruction_frame = -1;
	
	vol = bbox_vol(mi->bb);
	if (vol > rmd->max_vol)
	{
		rmd->max_vol = vol;
	}
	
	//(*mi_array)[*mi_count] = mi;
	//(*mi_count)++;
	mi->rigidbody = NULL;
	mi->vertices_cached = NULL;
	
	if (!rmd->use_cellbased_sim || rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED)
	{
		mi->rigidbody = BKE_rigidbody_create_shard(rmd->modifier.scene, ob, mi);
		BKE_rigidbody_calc_shard_mass(ob, mi);
		if (rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED)
			mi->rigidbody->flag |= RBO_FLAG_ACTIVE_COMPOUND;
		//mi->rigidbody->flag |= RBO_FLAG_BAKED_COMPOUND;
	//mi->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
	//mi->rigidbody->flag |= RBO_FLAG_INACTIVE_COMPOUND;
		//BKE_rigidbody_validate_sim_shard(rmd->modifier.scene->rigidbody_world, mi, ob, true);
		//mi->rigidbody->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
	}
	
	/* deselect loose data - this used to get deleted,
	 * we could de-select edges and verts only, but this turns out to be less complicated
	 * since de-selecting all skips selection flushing logic */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, FALSE);
	//BM_mesh_normals_update(bm_new);
	
	return vol;
}

/* flush a hflag to from verts to edges/faces */
static void bm_mesh_hflag_flush_vert(BMesh *bm, const char hflag)
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
		ok = TRUE;
		l_iter = l_first = BM_FACE_FIRST_LOOP(f);
		do {
			if (!BM_elem_flag_test(l_iter->v, hflag)) {
				ok = FALSE;
				break;
			}
		} while ((l_iter = l_iter->next) != l_first);

		BM_elem_flag_set(f, hflag, ok);
	}
}

unsigned int vertHash(BMVert* v) {
	//return (int)(v->co[0] * 100) ^ (int)(v->co[1] * 1000) ^ (int)(v->co[2] * 10000);
	return v->head.index;
}

void BM_mesh_join(BMesh** dest, BMesh* src)
{
	BMIter iter;
	BMVert* v, **verts;
	BMEdge *e, **edges;
	BMFace *f;
	int vcount = 0, ecount = 0;
	
	verts = MEM_mallocN(sizeof(BMVert*), "verts");
	edges = MEM_mallocN(sizeof(BMEdge*), "edges");
	
	CustomData_bmesh_merge(&src->vdata, &(*dest)->vdata, CD_MASK_BMESH, CD_CALLOC, *dest, BM_VERT);
	CustomData_bmesh_merge(&src->edata, &(*dest)->edata, CD_MASK_BMESH, CD_CALLOC, *dest, BM_EDGE);
	CustomData_bmesh_merge(&src->ldata, &(*dest)->ldata, CD_MASK_BMESH, CD_CALLOC, *dest, BM_LOOP);
	CustomData_bmesh_merge(&src->pdata, &(*dest)->pdata, CD_MASK_BMESH, CD_CALLOC, *dest, BM_FACE);
	
	BM_ITER_MESH(v, &iter, src, BM_VERTS_OF_MESH)
	{
		BMVert *vert = BM_vert_create(*dest, v->co, NULL, 0);
		verts = MEM_reallocN(verts, sizeof(BMVert*) *(vcount+1));
		verts[vcount] = vert;
		vcount++;
	}
		
	BM_ITER_MESH(e, &iter, src, BM_EDGES_OF_MESH)
	{
		BMEdge* edge;
		BMVert* v1 = verts[e->v1->head.index];
		BMVert* v2 = verts[e->v2->head.index];
		edge = BM_edge_create(*dest, v1, v2, NULL, 0);
		edges = MEM_reallocN(edges, sizeof(BMEdge*) * (ecount+1));
		edges[ecount] = edge;
		ecount++;
	}

	BM_ITER_MESH(f, &iter, src, BM_FACES_OF_MESH)
	{
		BMIter iter2;
		BMLoop* l;
		BMVert **ve = MEM_mallocN(sizeof(BMVert*), "face_verts");
		BMEdge **ed = MEM_mallocN(sizeof(BMEdge*), "face_edges");
		int lcount = 0;

		BM_ITER_ELEM(l, &iter2, f, BM_LOOPS_OF_FACE)
		{
			BMVert* v = verts[l->v->head.index];
			BMEdge* e = edges[l->e->head.index];
			
			ed = MEM_reallocN(ed, sizeof(BMEdge*) * (lcount+1));
			ed[lcount] = e;

			ve = MEM_reallocN(ve, sizeof(BMVert*) * (lcount+1));
			ve[lcount] = v;
			//lcount++;
		}

		BM_face_create(*dest, ve, ed, lcount, NULL, 0);
		MEM_freeN(ve);
		MEM_freeN(ed);
	}
	
	MEM_freeN(verts);
	MEM_freeN(edges);
}

void mesh_separate_loose_partition(RigidBodyModifierData* rmd, Object* ob, BMesh* bm_work, BMVert** orig_work/*, int doSplit*/)
{
	int i, j, tag_counter = 0;
	BMEdge *e;
	BMVert *v_seed, **v_tag;
	BMWalker walker;
	int tot = 0, seedcounter = 0;
	BMesh* bm_old = bm_work;
	int max_iter = bm_old->totvert;
	BMIter iter;
	BoundBox* bb;
	float* startco, min[3], max[3], vol_old = 0, cell_vol = 0;
	float vol = 0;
	//int mi_count = 0;
	//MeshIsland** mi_array = MEM_callocN(sizeof(MeshIsland*), "mi_array");

	/* Clear all selected vertices */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_INTERNAL_TAG | BM_ELEM_TAG, FALSE);
	
/*	if (rmd->use_cellbased_sim && bm_old->totvert > 0 && !doSplit)
	{
		bb = BKE_boundbox_alloc_unit();
		BM_mesh_minmax(bm_old, min, max, FALSE);
		//printf("MinMax\n");
		BKE_boundbox_init_from_minmax(bb, min, max);
		//printf("BoundBox\n");
		vol_old = bbox_vol(bb);
		MEM_freeN(bb);
	}
	
	cell_vol = rmd->cell_size * rmd->cell_size * rmd->cell_size;*/
	

	/* A "while (true)" loop should work here as each iteration should
	 * select and remove at least one vertex and when all vertices
	 * are selected the loop will break out. But guard against bad
	 * behavior by limiting iterations to the number of vertices in the
	 * original mesh.*/
	for (i = 0; i < max_iter; i++) {
		tag_counter = 0;
		seedcounter = 0;

		BM_ITER_MESH(v_seed, &iter, bm_old, BM_VERTS_OF_MESH)
		{
			//Hrm need to look at earlier verts to for unused ones.
			if (!BM_elem_flag_test(v_seed, BM_ELEM_TAG) && !BM_elem_flag_test(v_seed, BM_ELEM_INTERNAL_TAG)) {
				break;
			}
		}

		/* No vertices available, can't do anything */
		if (v_seed == NULL){
			break;
		}
		/* Select the seed explicitly, in case it has no edges */
		if (!BM_elem_flag_test(v_seed, BM_ELEM_TAG) && !BM_elem_flag_test(v_seed, BM_ELEM_INTERNAL_TAG)) { //!BLI_ghash_haskey(hash, v_seed)) {

			v_tag = MEM_callocN(sizeof(BMVert*), "v_tag");
			startco = MEM_callocN(sizeof(float), "mesh_separate_loose->startco");

			//BLI_ghash_insert(hash, v_seed, v_seed);
			BM_elem_flag_enable(v_seed, BM_ELEM_TAG);
			BM_elem_flag_enable(v_seed, BM_ELEM_INTERNAL_TAG);
			v_tag = MEM_reallocN(v_tag, sizeof(BMVert*) * (tag_counter+1));
			v_tag[tag_counter] = orig_work[v_seed->head.index]; //BM_vert_at_index(rmd->visible_mesh, v_seed->head.index);

			startco = MEM_reallocN(startco, (tag_counter+1) * 3 * sizeof(float));
			startco[3 * tag_counter] = v_seed->co[0];
			startco[3 * tag_counter+1] = v_seed->co[1];
			startco[3 * tag_counter+2] = v_seed->co[2];
			tot++;
			tag_counter++;
		}

		/* Walk from the single vertex, selecting everything connected
		 * to it */
		BMW_init(&walker, bm_old, BMW_SHELL,
					BMW_MASK_NOP, BMW_MASK_NOP, BMW_MASK_NOP,
					BMW_FLAG_NOP,
					BMW_NIL_LAY);

		e = BMW_begin(&walker, v_seed);
		for (; e; e = BMW_step(&walker)) {
			if (!BM_elem_flag_test(e->v1, BM_ELEM_TAG) && !BM_elem_flag_test(e->v1, BM_ELEM_INTERNAL_TAG)) { //(!BLI_ghash_haskey(hash, e->v1))) {
				//BLI_ghash_insert(hash, e->v1, e->v1);
				BM_elem_flag_enable(e->v1, BM_ELEM_TAG);
				BM_elem_flag_enable(e->v1, BM_ELEM_INTERNAL_TAG);
				v_tag = MEM_reallocN(v_tag, sizeof(BMVert*) * (tag_counter+1));
				v_tag[tag_counter] = orig_work[e->v1->head.index];

				startco = MEM_reallocN(startco, (tag_counter+1) * 3 * sizeof(float));
				startco[3 * tag_counter] = e->v1->co[0];
				startco[3 * tag_counter+1] = e->v1->co[1];
				startco[3 * tag_counter+2] = e->v1->co[2];
				tot++;
				tag_counter++;
			}
			if (!BM_elem_flag_test(e->v2, BM_ELEM_TAG) && !BM_elem_flag_test(e->v2, BM_ELEM_INTERNAL_TAG)) { //BLI_ghash_haskey(hash, e->v2))){
				//BLI_ghash_insert(hash, e->v2, e->v2);
				BM_elem_flag_enable(e->v2, BM_ELEM_TAG);
				BM_elem_flag_enable(e->v2, BM_ELEM_INTERNAL_TAG);

				v_tag = MEM_reallocN(v_tag, sizeof(BMVert*) * (tag_counter+1));
				v_tag[tag_counter] = orig_work[e->v2->head.index];

				startco = MEM_reallocN(startco, (tag_counter+1) * 3 * sizeof(float));
				startco[3 * tag_counter] = e->v2->co[0];
				startco[3 * tag_counter+1] = e->v2->co[1];
				startco[3 * tag_counter+2] = e->v2->co[2];
				tot++;
				tag_counter++;
			}
		}
		BMW_end(&walker);

		/* Flush the selection to get edge/face selections matching
		 * the vertex selection */
		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_TAG);

		/* Move selection into a separate object */
		//printf("splitting...\n");
		mesh_separate_tagged(rmd, ob, v_tag, tag_counter, &startco, bm_old /*, &mi_array, &mi_count*/);
		printf("mesh_separate_tagged: %d %d\n", tot, bm_old->totvert);

		if (tot >= bm_old->totvert) {
			break;
		}

		seedcounter = tot;
	}
	
	//MEM_freeN(mi_array);
	//mi_count = 0;
}

static void select_linked(BMesh** bm_in)
{
	BMIter iter;
	BMVert *v;
	BMEdge *e;
	BMWalker walker;
	BMesh *bm_work = *bm_in;


/*	BMFace *efa;

	BM_ITER_MESH (efa, &iter, bm_work, BM_FACES_OF_MESH) {
		BM_elem_flag_set(efa, BM_ELEM_TAG, (BM_elem_flag_test(efa, BM_ELEM_SELECT) &&
											!BM_elem_flag_test(efa, BM_ELEM_HIDDEN)));
	}

	BMW_init(&walker, bm_work, BMW_ISLAND,
			 BMW_MASK_NOP, BMW_MASK_NOP, BMW_MASK_NOP,
			 BMW_FLAG_TEST_HIDDEN,
			 BMW_NIL_LAY);

	BM_ITER_MESH (efa, &iter, bm_work, BM_FACES_OF_MESH) {
		if (BM_elem_flag_test(efa, BM_ELEM_TAG)) {
			for (efa = BMW_begin(&walker, efa); efa; efa = BMW_step(&walker)) {
				BM_face_select_set(bm_work, efa, true);
			}
		}
	}
	BMW_end(&walker);*/


	BM_ITER_MESH (v, &iter, bm_work, BM_VERTS_OF_MESH) {
		if (BM_elem_flag_test(v, BM_ELEM_SELECT)) {
			BM_elem_flag_enable(v, BM_ELEM_TAG);
		}
		else {
			BM_elem_flag_disable(v, BM_ELEM_TAG);
		}
	}

	BMW_init(&walker, bm_work , BMW_SHELL,
			 BMW_MASK_NOP, BMW_MASK_NOP, BMW_MASK_NOP,
			 BMW_FLAG_TEST_HIDDEN,
			 BMW_NIL_LAY);

	BM_ITER_MESH (v, &iter, bm_work, BM_VERTS_OF_MESH) {
		if (BM_elem_flag_test(v, BM_ELEM_TAG)) {
			for (e = BMW_begin(&walker, v); e; e = BMW_step(&walker)) {
				BM_edge_select_set(bm_work, e, true);
			}
		}
	}
	BMW_end(&walker);

	BM_mesh_select_flush(bm_work);
}

void mesh_separate_selected(BMesh** bm_work, BMesh** bm_out, BMVert** orig_work, BMVert*** orig_out1, BMVert*** orig_out2)
{
	BMesh *bm_old = *bm_work;
	BMesh *bm_new = *bm_out;
	BMVert* v, **orig_new = *orig_out1, **orig_mod = *orig_out2;
	BMIter iter;
	int new_index = 0, mod_index = 0;

	BM_mesh_elem_hflag_disable_all(bm_old, BM_FACE | BM_EDGE | BM_VERT, BM_ELEM_TAG, false);
	/* sel -> tag */
	BM_mesh_elem_hflag_enable_test(bm_old, BM_FACE | BM_EDGE | BM_VERT, BM_ELEM_TAG, true, BM_ELEM_SELECT);

	BM_mesh_elem_toolflags_ensure(bm_new);  /* needed for 'duplicate' bmo */

	CustomData_copy(&bm_old->vdata, &bm_new->vdata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_old->edata, &bm_new->edata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_old->ldata, &bm_new->ldata, CD_MASK_BMESH, CD_CALLOC, 0);
	CustomData_copy(&bm_old->pdata, &bm_new->pdata, CD_MASK_BMESH, CD_CALLOC, 0);

	CustomData_bmesh_init_pool(&bm_new->vdata, bm_mesh_allocsize_default.totvert, BM_VERT);
	CustomData_bmesh_init_pool(&bm_new->edata, bm_mesh_allocsize_default.totedge, BM_EDGE);
	CustomData_bmesh_init_pool(&bm_new->ldata, bm_mesh_allocsize_default.totloop, BM_LOOP);
	CustomData_bmesh_init_pool(&bm_new->pdata, bm_mesh_allocsize_default.totface, BM_FACE);

	BMO_op_callf(bm_old, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				 "duplicate geom=%hvef dest=%p", BM_ELEM_TAG, bm_new);

	//lets hope the order of elements in new mesh is the same as it was in old mesh
	BM_ITER_MESH(v, &iter, bm_old, BM_VERTS_OF_MESH)
	{
		if (BM_elem_flag_test(v, BM_ELEM_TAG)) {
			orig_new[new_index] = orig_work[v->head.index];
			new_index++;
		}
		else
		{
			orig_mod[mod_index] = orig_work[v->head.index];
			mod_index++;
		}
	}

	new_index = 0;
	BM_ITER_MESH(v, &iter, bm_new, BM_VERTS_OF_MESH)
	{
		v->head.index = new_index;
		new_index++;
	}

	BMO_op_callf(bm_old, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
				 "delete geom=%hvef context=%i", BM_ELEM_TAG, DEL_FACES);

	/* deselect loose data - this used to get deleted,
	 * we could de-select edges and verts only, but this turns out to be less complicated
	 * since de-selecting all skips selection flushing logic */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);

	BM_mesh_normals_update(bm_new);
}

float calc_bb_vol(BMesh* bm)
{
	float vol, min[3], max[3];
	BoundBox* bb;
	BM_mesh_minmax(bm, min, max, TRUE);
	bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(bb, min, max);
	vol = bbox_vol(bb);
	MEM_freeN(bb);
	return vol;
}

void halve(RigidBodyModifierData* rmd, Object* ob, int minsize, BMesh** bm_work, BMVert*** orig_work, bool separated/*, bool doSplit*/)
{

	int half;
	int i = 0, new_count = 0;
	float vol_old = 0, vol_new = 0, min[3], max[3], cellvol = 0;
	BMIter iter;
	BMVert **orig_old = *orig_work, **orig_new, **orig_mod;
	BMVert *v;
	BMesh* bm_old = *bm_work;
	BMesh* bm_new = BM_mesh_create(&bm_mesh_allocsize_default);
	separated = false;

	{
		BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, FALSE);
		
		/*if (rmd->use_cellbased_sim && 0)
		{
			float half_vol = vol_old * 0.5f;
			BM_ITER_MESH(v, &iter, bm_old, BM_VERTS_OF_MESH)
			{
				float vol;
				BM_elem_select_set(bm_old, v, true);
				vol = calc_bb_vol(bm_old);
				
				if (vol >= half_vol)
				{
					break;
				}
			}
		}
		else*/
		{
			half = bm_old->totvert / 2;
			BM_ITER_MESH(v, &iter, bm_old, BM_VERTS_OF_MESH)
			{
				if (i >= half) {
					break;
				}
				BM_elem_select_set(bm_old, v, true);
				i++;
			}
		}

		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_SELECT);
		select_linked(&bm_old);

		new_count = bm_old->totvertsel;
		printf("Halving...%d => %d %d\n", bm_old->totvert, new_count, bm_old->totvert - new_count);

		orig_new = MEM_callocN(sizeof(BMVert*) * new_count, "orig_new");
		orig_mod = MEM_callocN(sizeof(BMVert*) * bm_old->totvert - new_count, "orig_mod");
		mesh_separate_selected(&bm_old, &bm_new, orig_old, &orig_new, &orig_mod);
		//BM_mesh_elem_hflag_disable_all(bm_new, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, FALSE);
	}

	printf("Old New: %d %d\n", bm_old->totvert, bm_new->totvert);
	if ((bm_old->totvert <= minsize && bm_old->totvert > 0) || (bm_new->totvert == 0)) {
		mesh_separate_loose_partition(rmd, ob, bm_old, orig_mod);
		separated = true;
	}

	if ((bm_new->totvert <= minsize && bm_new->totvert > 0) || (bm_old->totvert == 0)) {
		mesh_separate_loose_partition(rmd, ob, bm_new, orig_new);
		separated = true;
	}

	if ((bm_old->totvert > minsize && bm_new->totvert > 0) || (bm_new->totvert == 0 && !separated)) {
		halve(rmd, ob, minsize, &bm_old, &orig_mod, separated);
	}

	if ((bm_new->totvert > minsize && bm_old->totvert > 0) || (bm_old->totvert == 0 && !separated)) {
		halve(rmd, ob, minsize, &bm_new, &orig_new, separated);
	}

	MEM_freeN(orig_mod);
	MEM_freeN(orig_new);
	BM_mesh_free(bm_new);
	bm_new = NULL;
}

typedef struct VertParticle
{
	BMVert** verts;
	int vertcount;
	float *vertco;
} VertParticle;

/*int facebyparticle(const void *e1, const void *e2)
{
	const VertParticle *fp1 = *(void **)e1, *fp2 = *(void **)e2;
	int x1 = fp1->particle;
	int x2 = fp2->particle;

	if      (x1 > x2) return  1;
	else if (x1 < x2) return -1;
	else return 0;
}*/

void mesh_separate_loose(RigidBodyModifierData* rmd, ExplodeModifierData *emd, Object* ob)
{
	int minsize = 1000;
	//GHash* vhash = BLI_ghash_ptr_new("VertHash");
	BMesh* bm_work;
	BMVert* vert, **orig_start;
	BMIter iter;
	//int lastparticle = -1;
	//VertParticle **fps = MEM_callocN(sizeof(VertParticle*) * rmd->visible_mesh->totface, "faceparticles");
	//IF HAVE CLASSIC EXPLO, DO NOT SPLIT, TAKE ITS ISLANDS AS IS...
	if (emd && emd->mode == eFractureMode_Faces)
	{
		GHash *verthash = BLI_ghash_ptr_new("verthash");
		if (emd->vertpahash)
		{
			MeshIsland *mi = NULL;
			EdgeHashIterator *ehi = BLI_edgehashIterator_new(emd->vertpahash);
			GHashIterator ghi;
			
			//faster than BM_vert_at_index...
			BMVert** vertarray = MEM_callocN(sizeof(BMVert*) * rmd->visible_mesh->totvert, "vertarray");
			BM_ITER_MESH(vert, &iter, rmd->visible_mesh, BM_VERTS_OF_MESH)
			{
				vertarray[vert->head.index] = vert;
			}	
			
			//contains vertex index and particle index at ed_v1 and at ed_v2;
			for (; !BLI_edgehashIterator_isDone(ehi); BLI_edgehashIterator_step(ehi)) {
				unsigned int ed_v1, ed_v2;
				void* lookup;
				VertParticle *vertpa;
				int v;
				
				BLI_edgehashIterator_getKey(ehi, &ed_v1, &ed_v2);
				ed_v2 -= rmd->visible_mesh->totvert; //is shifted to help distiguish vert indexes from particle indexes
				v = GET_INT_FROM_POINTER(BLI_edgehashIterator_getValue(ehi));
		
				//need v and ed_v2, group v by ed_v2
				lookup = BLI_ghash_lookup(verthash, SET_INT_IN_POINTER(ed_v2));
				if (!lookup)
				{
					vertpa = MEM_callocN(sizeof(VertParticle), "vertpa");
					vertpa->verts = MEM_callocN(sizeof(BMVert*), "vertlist");
					vertpa->vertco = MEM_callocN(sizeof(float)*3, "vertco");
					vertpa->verts[0] = vertarray[v];
					vertpa->vertco[0] = vertarray[v]->co[0];
					vertpa->vertco[1] = vertarray[v]->co[1];
					vertpa->vertco[2] = vertarray[v]->co[2];
					vertpa->vertcount = 1;
					BLI_ghash_insert(verthash, SET_INT_IN_POINTER(ed_v2), vertpa);
				}
				else
				{
					vertpa = (VertParticle*)lookup;
					vertpa->verts = MEM_reallocN(vertpa->verts, sizeof(BMVert*) * (vertpa->vertcount+1));
					vertpa->vertco = MEM_reallocN(vertpa->vertco, sizeof(float)*3*(vertpa->vertcount+1));
					vertpa->verts[vertpa->vertcount] = vertarray[v];
					vertpa->vertco[vertpa->vertcount * 3] = vertarray[v]->co[0];
					vertpa->vertco[vertpa->vertcount * 3+1] = vertarray[v]->co[1];
					vertpa->vertco[vertpa->vertcount * 3+2] = vertarray[v]->co[2];
					vertpa->vertcount++;
				}
			}
			
			BLI_edgehashIterator_free(ehi);
			
			//BMO_op_callf(rmd->visible_mesh, BMO_FLAG_DEFAULTS, "remove_doubles verts=%av dist=%f", BM_VERTS_OF_MESH, 0.0001f);
			BM_mesh_elem_hflag_disable_all(rmd->visible_mesh, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, FALSE);
			GHASH_ITER(ghi, verthash)
			{
				float centroid[3], min[3], max[3], rot[4], vol, dummyloc[3];
				int j;
				VertParticle* vpa = (VertParticle*)(BLI_ghashIterator_getValue(&ghi));
				DerivedMesh* dm = NULL;
				BMesh* bm_old = rmd->visible_mesh;
				BMesh *bm_new = BM_mesh_create(&bm_mesh_allocsize_default);
				BM_mesh_elem_toolflags_ensure(bm_new);
				
				mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
				BLI_addtail(&rmd->meshIslands, mi);
				
				mi->particle_index = -1;
				mi->vertices = vpa->verts;
				mi->vertex_count = vpa->vertcount;
				mi->vertco = vpa->vertco;
				
				for (j = 0; j < mi->vertex_count; j++)
				{
					BM_elem_flag_enable(mi->vertices[j], BM_ELEM_TAG);
				}
				
				bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_TAG);
				
				mi->compound_children = NULL;
				mi->compound_count = 0;
				mi->compound_parent = NULL;
				zero_v3(mi->start_co);
				
				mi->participating_constraints = NULL;
				mi->participating_constraint_count = 0;
				mi->destruction_frame = -1;
				
				CustomData_copy(&bm_old->vdata, &bm_new->vdata, CD_MASK_BMESH, CD_CALLOC, 0);
				CustomData_copy(&bm_old->edata, &bm_new->edata, CD_MASK_BMESH, CD_CALLOC, 0);
				CustomData_copy(&bm_old->ldata, &bm_new->ldata, CD_MASK_BMESH, CD_CALLOC, 0);
				CustomData_copy(&bm_old->pdata, &bm_new->pdata, CD_MASK_BMESH, CD_CALLOC, 0);
				
				CustomData_bmesh_init_pool(&bm_new->vdata, bm_mesh_allocsize_default.totvert, BM_VERT);
				CustomData_bmesh_init_pool(&bm_new->edata, bm_mesh_allocsize_default.totedge, BM_EDGE);
				CustomData_bmesh_init_pool(&bm_new->ldata, bm_mesh_allocsize_default.totloop, BM_LOOP);
				CustomData_bmesh_init_pool(&bm_new->pdata, bm_mesh_allocsize_default.totface, BM_FACE);
				
				
				BMO_op_callf(bm_old, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
							 "duplicate geom=%hvef dest=%p", BM_ELEM_TAG, bm_new);
				
				BM_calc_center_centroid(bm_new, centroid, FALSE);
				BM_ITER_MESH (vert, &iter, bm_new, BM_VERTS_OF_MESH) {
					sub_v3_v3(vert->co, centroid);
				}
				
				BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, FALSE);
				
				BM_mesh_normals_update(bm_new);
				BM_mesh_minmax(bm_new, min, max, FALSE);
				dm = CDDM_from_bmesh(bm_new, true);
				BM_mesh_free(bm_new);
				bm_new = NULL;
				mi->physics_mesh = dm;
				
				copy_v3_v3(mi->centroid, centroid);
				mat4_to_loc_quat(dummyloc, rot, ob->obmat);
				copy_v3_v3(mi->rot, rot);
				mi->parent_mod = rmd;
				mi->bb = BKE_boundbox_alloc_unit();
				BKE_boundbox_init_from_minmax(mi->bb, min, max);
				
				vol = bbox_vol(mi->bb);
				if (vol > rmd->max_vol)
				{
					rmd->max_vol = vol;
				}
				
				if (!rmd->use_cellbased_sim || rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED)
				{
					mi->rigidbody = BKE_rigidbody_create_shard(rmd->modifier.scene, ob, mi);
					BKE_rigidbody_calc_shard_mass(ob, mi);
					if (rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED)
						mi->rigidbody->flag |= RBO_FLAG_ACTIVE_COMPOUND;
				}
				
				MEM_freeN(vpa);
			}
			
			MEM_freeN(vertarray);
		}
		BLI_ghash_free(verthash, NULL, NULL);
	}
	else
	{
		BM_mesh_elem_hflag_disable_all(rmd->visible_mesh, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, FALSE);
		bm_work = BM_mesh_copy(rmd->visible_mesh);
	
		orig_start = MEM_callocN(sizeof(BMVert*) * rmd->visible_mesh->totvert, "orig_start");
		//associate new verts with old verts, here indexes should match still
		BM_ITER_MESH(vert, &iter, rmd->visible_mesh, BM_VERTS_OF_MESH)
		{
			orig_start[vert->head.index] = vert;
		}	
	
		halve(rmd, ob, minsize, &bm_work, &orig_start, false);
	
	//	BLI_ghash_free(vhash, NULL, NULL);
		MEM_freeN(orig_start);
		orig_start = NULL;
		BM_mesh_free(bm_work);
		bm_work = NULL;
	}
}

/*void mesh_separate_island(RigidBodyModifierData* rmd, Object* ob, MeshIsland* mi)
{
	int minsize = 1000;
	BMesh* bm_work;
	BMVert** orig_start;

	bm_work = DM_to_bmesh(mi->physics_mesh);
	BM_mesh_elem_hflag_disable_all(bm_work, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG | BM_ELEM_INTERNAL_TAG, FALSE);

	orig_start = mi->vertices;
	halve(rmd, ob, minsize, &bm_work, &orig_start, false, true);
	
	BLI_remlink(&rmd->meshIslands, mi);
	MEM_freeN(mi->physics_mesh);
	MEM_freeN(mi->vertices);
	MEM_freeN(mi->vertco);
	MEM_freeN(mi->bb);
	BM_mesh_free(bm_work);
	bm_work = NULL;
}*/

void destroy_compound(RigidBodyModifierData* rmd, Object* ob, MeshIsland *mi, float cfra)
{
	//add all children and remove ourself
	int i = 0;
	float centr[3], size[3];
	invert_m4_m4(ob->imat, ob->obmat);
	mat4_to_size(size, ob->obmat);
	
	for (i = 0; i < mi->compound_count; i++)
	{
		MeshIsland* mi2 = mi->compound_children[i];
		if (mi2->rigidbody == NULL)
		{
			mi2->rigidbody = BKE_rigidbody_create_shard(rmd->modifier.scene, ob, mi2);
			BKE_rigidbody_calc_shard_mass(ob, mi2);
			mi2->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
			mi2->rigidbody->flag |= RBO_FLAG_ACTIVE_COMPOUND; // do not build constraints between those
			//mi2->rigidbody->flag |= RBO_FLAG_BAKED_COMPOUND;
			//linear, angular velocities too ?!
		//	BKE_rigidbody_validate_sim_shard(rmd->modifier.scene->rigidbody_world, mi2, ob, true);
		//	mi2->rigidbody->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
		}
		mi2->rigidbody->flag |= RBO_FLAG_BAKED_COMPOUND;
		
		if (mi->rigidbody != NULL)
		{
			copy_v3_v3(mi2->rigidbody->pos, mi->rigidbody->pos);
			copy_qt_qt(mi2->rigidbody->orn, mi->rigidbody->orn);
			copy_v3_v3(centr, mi2->centroid);
			mul_v3_v3(centr, size);
			mul_qt_v3(mi2->rigidbody->orn, centr);
			add_v3_v3(mi2->rigidbody->pos, centr);
		}
	}
	
	
	if (mi->compound_count > 0)
	{
		bool baked = rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED;
		if (!baked) {
			mi->rigidbody->flag &= ~RBO_FLAG_ACTIVE_COMPOUND;
			mi->rigidbody->flag &= ~RBO_FLAG_BAKED_COMPOUND;
		}
		
		if (mi->destruction_frame < 0)
		{
			mi->destruction_frame = cfra;
			//dont update framemap in baked mode
			if (rmd->framemap != NULL && !baked)
			{
				int index = BLI_findindex(&rmd->meshIslands, mi);
				if (index < rmd->framecount)
				{
					rmd->framemap[index] = cfra;
				}
			}
		}
	}
}

void select_inner_faces(RigidBodyModifierData* rmd, KDTree* tree, MeshIsland* mi) {

	BMFace *face;
	BMIter iter;

	//if (vert == NULL) //can happen with constraint groups, investigate why (TODO) but prevent crash for now
	//	return;

	BM_ITER_MESH(face, &iter, mi->physics_mesh, BM_FACES_OF_MESH) {
		int n, i = 0;
		float co[3];
		BMFace* f;
		//KDTreeNearest *face_near = MEM_mallocN(sizeof(KDTreeNearest) * 2, "kdtreenearest_face");
		BM_face_calc_center_bounds(face, co);
		n = BLI_kdtree_find_nearest(tree, co, face->no, NULL);
		//BM_elem_flag_enable(face, BM_ELEM_SELECT);

		//for (i = 0; i < n; i++) {
			f = BM_face_at_index(rmd->visible_mesh, n);//face_near[i].index);
			//if (BM_elem_flag_test(face, BM_ELEM_TAG) && BM_elem_flag_test(f, BM_ELEM_TAG))
			//	break;

			//dist = face_near[i].dist;
			if (f->head.index != face->head.index) {
				float res[3], zero[3];
				zero_v3(zero);
				add_v3_v3v3(res, face->no, f->no);
				//printf("Res (%f %f %f) %d %d %f\n", res[0], res[1], res[2], face->head.index, f->head.index, face_near[i].dist);
				if (compare_v3v3(res, zero, 0.01f))
				{
					BM_elem_flag_enable(f, BM_ELEM_SELECT);
					BM_elem_flag_enable(face, BM_ELEM_SELECT);

					if ((!(BM_elem_flag_test(f, BM_ELEM_TAG)) && (!BM_elem_flag_test(face, BM_ELEM_TAG))) || 1) {
						rmd->sel_indexes = MEM_reallocN(rmd->sel_indexes, sizeof(int*) * (rmd->sel_counter+1));
						rmd->sel_indexes[rmd->sel_counter] = MEM_callocN(sizeof(int)*2, "sel_index_pair");
						rmd->sel_indexes[rmd->sel_counter][0] = f->head.index;
						rmd->sel_indexes[rmd->sel_counter][1] = face->head.index;
						rmd->sel_counter++;
					}

					BM_elem_flag_enable(f, BM_ELEM_TAG);
				}
			}
			else
			{
				//BM_elem_flag_disable(f, BM_ELEM_SELECT);
				BM_elem_flag_enable(f, BM_ELEM_TAG);
			}

		//}
		//MEM_freeN(face_near);
		//face_near = NULL;
	}
}

void select_inner_faces_of_vert(RigidBodyModifierData* rmd, KDTree* tree, BMVert* vert) {

	BMFace *face;
	BMIter iter;

	if (vert == NULL) //can happen with constraint groups, investigate why (TODO) but prevent crash for now
		return;

	BM_ITER_ELEM(face, &iter, vert, BM_FACES_OF_VERT) {
		int n, i = 0;
		float co[3];
		BMFace* f;
		KDTreeNearest *face_near = MEM_mallocN(sizeof(KDTreeNearest) * 2, "kdtreenearest_face");
		BM_face_calc_center_bounds(face, co);
		n = BLI_kdtree_find_nearest_n(tree, co, face->no, face_near, 2);
		//BM_elem_flag_enable(face, BM_ELEM_SELECT);

		for (i = 0; i < n; i++) {
			f = BM_face_at_index(rmd->visible_mesh, face_near[i].index);
			if (BM_elem_flag_test(face, BM_ELEM_TAG) && BM_elem_flag_test(f, BM_ELEM_TAG))
				break;

			//dist = face_near[i].dist;
			if (f->head.index != face->head.index) {
				float res[3], zero[3];
				zero_v3(zero);
				add_v3_v3v3(res, face->no, f->no);
				//printf("Res (%f %f %f) %d %d %f\n", res[0], res[1], res[2], face->head.index, f->head.index, face_near[i].dist);
				if (compare_v3v3(res, zero, 0.000001f))
				{
					BM_elem_flag_enable(f, BM_ELEM_SELECT);
					BM_elem_flag_enable(face, BM_ELEM_SELECT);

					if ((!(BM_elem_flag_test(f, BM_ELEM_TAG)) && (!BM_elem_flag_test(face, BM_ELEM_TAG))) || 1) {
						rmd->sel_indexes = MEM_reallocN(rmd->sel_indexes, sizeof(int*) * (rmd->sel_counter+1));
						rmd->sel_indexes[rmd->sel_counter] = MEM_callocN(sizeof(int)*2, "sel_index_pair");
						rmd->sel_indexes[rmd->sel_counter][0] = f->head.index;
						rmd->sel_indexes[rmd->sel_counter][1] = face->head.index;
						rmd->sel_counter++;
					}

					BM_elem_flag_enable(f, BM_ELEM_TAG);
				}
			}
			else
			{
				//BM_elem_flag_disable(f, BM_ELEM_SELECT);
				BM_elem_flag_enable(f, BM_ELEM_TAG);
			}

		}
		MEM_freeN(face_near);
		face_near = NULL;
	}
}

static void connect_meshislands(RigidBodyModifierData* rmd, Object* ob, MeshIsland* mi1, MeshIsland* mi2, int con_type, float thresh)
{
	int con_found = FALSE;
	RigidBodyShardCon *con, *rbsc;
	bool ok = mi1 && mi1->rigidbody && !(mi1->rigidbody->flag & RBO_FLAG_ACTIVE_COMPOUND);
	ok = ok && mi2 && mi2->rigidbody && !(mi2->rigidbody->flag & RBO_FLAG_ACTIVE_COMPOUND);
	
	if (((!rmd->use_both_directions) || (mi1->parent_mod != mi2->parent_mod)) && ok)
	{
		//search local constraint list instead of global one !!! saves lots of time
		int i;
		for (i = 0; i < mi1->participating_constraint_count; i++)
		{
			con = mi1->participating_constraints[i];
			if ((con->mi1 == mi2) || (con->mi2 == mi2)) {
				con_found = TRUE;
				break;
			}
		}
	
		if (!con_found)
		{
			for (i = 0; i < mi2->participating_constraint_count; i++)
			{
				con = mi2->participating_constraints[i];
				if ((con->mi1 == mi1) || (con->mi2 == mi1)){
					con_found = TRUE;
					break;
				}
			}
		}
		
		/*for (con = rmd->meshConstraints.first; con; con = con->next) {
			if (((con->mi1 == mi1) && (con->mi2 == mi2)) ||
				((con->mi1 == mi2 && (con->mi2 == mi1)))) {
				con_found = TRUE;
				break;
			}
		}*/
	}

	if (!con_found && ok)
	{
		if (rmd->use_constraints) {
			if (((rmd->constraint_group != NULL) &&
				(!BKE_group_object_exists(rmd->constraint_group, ob))) ||
				(rmd->constraint_group == NULL)) {
				
				rbsc = BKE_rigidbody_create_shard_constraint(rmd->modifier.scene, con_type);
				rbsc->mi1 = mi1;
				rbsc->mi2 = mi2;
				if (thresh == 0)
				{
					rbsc->flag &= ~RBC_FLAG_USE_BREAKING;
				}
				
				if (rmd->disable_self_collision)
				{
					rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;
				}
				
				if ((mi1->particle_index != -1) && (mi2->particle_index != -1) && (mi1->particle_index == mi2->particle_index))
				{
					ExplodeModifierData *emd = findPrecedingExploModifier(ob, rmd);
					if ((emd != NULL) && (emd->cluster_size > 1))
					{
						rbsc->breaking_threshold = rmd->cluster_breaking_threshold;
					}
					else
					{
						rbsc->breaking_threshold = thresh;
					}
				}
				else
				{
					rbsc->breaking_threshold = thresh;
				}
						
				//BKE_rigidbody_start_dist_angle(rbsc);
				BLI_addtail(&rmd->meshConstraints, rbsc);
				
				//store constraints per meshisland too, to allow breaking percentage
				if (mi1->participating_constraints == NULL)
				{
					mi1->participating_constraints = MEM_callocN(sizeof(RigidBodyShardCon*), "part_constraints_mi1");
					mi1->participating_constraint_count = 0;
				}
				mi1->participating_constraints = MEM_reallocN(mi1->participating_constraints, sizeof(RigidBodyShardCon*) * (mi1->participating_constraint_count+1));
				mi1->participating_constraints[mi1->participating_constraint_count] = rbsc;
				mi1->participating_constraint_count++;
				
				if (mi2->participating_constraints == NULL)
				{
					mi2->participating_constraints = MEM_callocN(sizeof(RigidBodyShardCon*), "part_constraints_mi2");
					mi2->participating_constraint_count = 0;
				}
				mi2->participating_constraints = MEM_reallocN(mi2->participating_constraints, sizeof(RigidBodyShardCon*) * (mi2->participating_constraint_count+1));
				mi2->participating_constraints[mi2->participating_constraint_count] = rbsc;
				mi2->participating_constraint_count++;
			}
		}
	}
}

static int check_meshislands_adjacency(RigidBodyModifierData* rmd, MeshIsland* mi, MeshIsland* mi2, BMesh **combined_mesh, KDTree* face_tree, Object* ob)
{
	BMOperator op;
	BMOpSlot *slot;
	int same = TRUE;
	int shared = 0, island_vert_key_index = 0, island_vert_map_index = 0;
	int* island_verts_key = MEM_mallocN(sizeof(int), "island_verts_key");
	int* island_verts_map = MEM_mallocN(sizeof(int), "island_verts_map");
	int v;

	//check whether we are in the same object or not
	float thresh, dist;
	int con_type, equal = mi->parent_mod == mi2->parent_mod;
	//equal = equal && (mi->parent_mod == rmd);

	thresh = equal ? rmd->breaking_threshold : rmd->group_breaking_threshold;
	con_type = equal ? rmd->inner_constraint_type : rmd->outer_constraint_type;
	dist = equal ? rmd->contact_dist : rmd->outer_constraint_type == RBC_TYPE_FIXED ? rmd->group_contact_dist : 0;
	//connect here only in "fixed" case, otherwise its done separately

	//select "our" vertices
	for (v = 0; v < mi2->vertex_count; v++) {
		BM_elem_flag_enable(BM_vert_at_index(*combined_mesh, mi2->combined_index_map[v]), BM_ELEM_TAG);
	}

	//do we share atleast 3 verts in selection

	BMO_op_initf(*combined_mesh, &op, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "find_doubles verts=%hv dist=%f", BM_ELEM_TAG, dist);
	BMO_op_exec(*combined_mesh, &op);
	slot = BMO_slot_get(op.slots_out, "targetmap.out");

	if (slot->data.ghash && BLI_ghash_size(slot->data.ghash) > 2) {
		GHashIterator it;
		int ind1 = 0, ind2 = 0;
		GHASH_ITER(it, slot->data.ghash) {
			BMVert *vert_key, *vert_map;
			//BMOElemMapping * mapping = BLI_ghashIterator_getValue(&it);
			vert_key = BLI_ghashIterator_getKey(&it);
			//vert_map = mapping[1].element;
			vert_map = BMO_slot_map_elem_get(slot, vert_key);

			if (vert_key == vert_map)
			{
				printf("EQUAL! D'OH\n");
			}

			for (v = 0; v < mi->vertex_count; v++) {
				if ((vert_key == BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]))) {
					island_verts_key = MEM_reallocN(island_verts_key, sizeof(int) * (island_vert_key_index+1));
					island_verts_key[island_vert_key_index] = mi->combined_index_map[v];
					island_vert_key_index++;
					ind1++;
					break;
				}

				if ((vert_map == BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]))) {
					island_verts_map = MEM_reallocN(island_verts_map, sizeof(int) * (island_vert_map_index+1));
					island_verts_map[island_vert_map_index] = mi->combined_index_map[v];
					island_vert_map_index++;
					break;
				}
			}

			for (v = 0; v < mi2->vertex_count; v++) {
				if ((vert_key == BM_vert_at_index(*combined_mesh, mi2->combined_index_map[v]))) {
					island_verts_key = MEM_reallocN(island_verts_key, sizeof(int) * (island_vert_key_index+1));
					island_verts_key[island_vert_key_index] = mi2->combined_index_map[v];
					island_vert_key_index++;
					ind2++;
					break;
				}

				if ((vert_map == BM_vert_at_index(*combined_mesh, mi2->combined_index_map[v]))) {
					island_verts_map = MEM_reallocN(island_verts_map, sizeof(int) * (island_vert_map_index+1));
					island_verts_map[island_vert_map_index] = mi2->combined_index_map[v];
					island_vert_map_index++;
					break;
				}
			}
		}

		// verts are in different objects, ok, only if we have different modifiers as parent
		same = ((equal && ((ind1 == 0) || (ind2 == 0))) || (!equal) || (rmd->inner_constraint_type != RBC_TYPE_FIXED));
		//"same" does not work anymore for other constraint types than FIXED, so use it for this as distinction only
	}

	//if ((slot->data.ghash) && (slot->data.ghash->nentries > 2))
	//	printf("%d %d\n", island_vert_key_index, island_vert_map_index);

	if (rmd->auto_merge && ((island_vert_key_index > 0) || (island_vert_map_index > 0))) {
		BMVert **vert_arr, **vert_arr2;
		int s;

		vert_arr = MEM_mallocN(sizeof(BMVert*) * island_vert_key_index, "vert_arr");
		vert_arr2 = MEM_mallocN(sizeof(BMVert*) * island_vert_map_index, "vert_arr2");

		for (s = 0; s < island_vert_key_index; s++) {
			vert_arr[s] = BM_vert_at_index(rmd->visible_mesh, island_verts_key[s]);
		}

		for (s = 0; s < island_vert_map_index; s++) {
			vert_arr2[s] = BM_vert_at_index(rmd->visible_mesh, island_verts_map[s]);
		}

		//select_inner_faces_of_vert(rmd, face_tree, vert_arr[0]);
		//select_inner_faces_of_vert(rmd, face_tree, vert_arr2[0]);
		if (island_vert_key_index > 0) {
			for (s = 0; s < island_vert_key_index; s++)
			{
				select_inner_faces_of_vert(rmd, face_tree, vert_arr[s]);
			}
		}

		if (island_vert_map_index > 0) {
			for (s = 0; s < island_vert_map_index; s++)
			{
				select_inner_faces_of_vert(rmd, face_tree, vert_arr2[s]);
			}
		}

		MEM_freeN(vert_arr);
		MEM_freeN(vert_arr2);
		vert_arr = NULL;
		vert_arr2 = NULL;
	}

	MEM_freeN(island_verts_map);
	MEM_freeN(island_verts_key);
	island_verts_map = NULL;
	island_verts_key = NULL;
	island_vert_key_index = 0;
	island_vert_map_index = 0;

	if (slot->data.ghash) {
		shared = BLI_ghash_size(slot->data.ghash);
	}
	else
	{
		shared = 0;
	}

/*	if ((rmd->auto_merge) && (shared > 0)) {
		float co[3], co2[3];
		copy_v3_v3(co, mi->centroid);
		copy_v3_v3(co2, mi2->centroid);
		printf("MeshIsland: %d %d (%f, %f, %f) | (%f, %f, %f) - %d \n",
			   j, (n+i)->index, co[0], co[1], co[2], co2[0], co2[1], co2[2], shared);
	}*/

	BMO_op_finish(*combined_mesh, &op);
	slot = NULL;

	//deselect vertices
	for (v = 0; v < mi2->vertex_count; v++) {
		BM_elem_flag_disable(BM_vert_at_index(*combined_mesh, mi2->combined_index_map[v]), BM_ELEM_TAG);
	}

	if (shared > 0) {
		// shared vertices (atleast one face ?), so connect...
		// if all verts either in same object or not !
		if (same)
			connect_meshislands(rmd, ob, mi, mi2, con_type, thresh);
	}

	return shared;
}

static int bbox_intersect(RigidBodyModifierData *rmd, MeshIsland *mi, MeshIsland *mi2)
{
	float cent_vec[3], test_x1[3], test_y1[3], test_z1[3], test_x2[3], test_y2[3], test_z2[3];

	//pre test with bounding boxes
	//vec between centroids -> v, if v[0] > bb1[4] - bb1[0] / 2 + bb2[4] - bb[0] / 2 in x range ?
	// analogous y and z, if one overlaps, bboxes touch, make expensive test then
	int equal = mi->parent_mod == mi2->parent_mod;
	//float dist = equal ? rmd->contact_dist : rmd->group_contact_dist;
	float dist = equal ? rmd->contact_dist : rmd->outer_constraint_type == RBC_TYPE_FIXED ? rmd->group_contact_dist : 0;

	if ((mi->bb == NULL) || (mi2->bb == NULL)) {
		//compat with older files, where bb test missed
		return TRUE;
	}

	sub_v3_v3v3(cent_vec, mi->centroid, mi2->centroid);
	sub_v3_v3v3(test_x1, mi->bb->vec[4], mi->bb->vec[0]);
	sub_v3_v3v3(test_x2, mi2->bb->vec[4], mi2->bb->vec[0]);
	mul_v3_fl(test_x1, 0.5f);
	mul_v3_fl(test_x2, 0.5f);

	sub_v3_v3v3(test_y1, mi->bb->vec[3], mi->bb->vec[0]);
	sub_v3_v3v3(test_y2, mi2->bb->vec[3], mi2->bb->vec[0]);
	mul_v3_fl(test_y1, 0.5f);
	mul_v3_fl(test_y2, 0.5f);

	sub_v3_v3v3(test_z1, mi->bb->vec[1], mi->bb->vec[0]);
	sub_v3_v3v3(test_z2, mi2->bb->vec[1], mi2->bb->vec[0]);
	mul_v3_fl(test_z1, 0.5f);
	mul_v3_fl(test_z2, 0.5f);

	/*printf("X: %f %f\n", fabs(test_x1[0] + test_x2[0]) + dist, fabs(cent_vec[0]));
	printf("Y: %f %f\n", fabs(test_y1[1] + test_y2[1]) + dist, fabs(cent_vec[1]));
	printf("Z: %f %f\n", fabs(test_z1[2] + test_z2[2]) + dist, fabs(cent_vec[2]));*/

	if (fabs(test_x1[0] + test_x2[0]) + dist >= fabs(cent_vec[0])) {
		//printf("X ok\n");
		if (fabs(test_y1[1] + test_y2[1]) + dist >= fabs(cent_vec[1])) {
			//printf("Y ok\n");
			if (fabs(test_z1[2] + test_z2[2]) + dist >= fabs(cent_vec[2])) {
				//printf("Z ok\n");
				return TRUE;
			}
		}
	}
	return FALSE;
}

static void search_centroid_based(RigidBodyModifierData *rmd, Object* ob, MeshIsland* mi, MeshIsland** meshIslands, KDTree**combined_tree, float centr[3])
{
	int r = 0, limit = 0, i = 0;
	KDTreeNearest* n3 = NULL;
	float dist, obj_centr[3], ratio = 1;

	//mi = meshIslands[j/*(n2+j)->index*/];
	//if (j == 0)
	//	first = mi;
	limit = rmd->constraint_limit;
	dist = mi->parent_mod == rmd ? rmd->contact_dist : rmd->group_contact_dist;
	
	if ((rmd->use_proportional_distance || rmd->use_proportional_limit))
	{
		if (rmd->max_vol > 0)
		{
			ratio = bbox_vol(mi->bb) / rmd->max_vol;
		}
		
		if (rmd->use_proportional_limit && (limit > 0))
		{
			limit = (int)(ratio * limit)+1;
		}
		
		if (rmd->use_proportional_distance)
		{
			dist = ratio * dist;
		}
	}
	
	if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELL_CENTROIDS)
	{
		copy_v3_v3(obj_centr, centr);
	}
	else
	{
		mul_v3_m4v3(obj_centr, mi->parent_mod->origmat, mi->centroid );
	}
	
	r = BLI_kdtree_range_search(*combined_tree, obj_centr, NULL, &n3, dist);
	
	//use centroid dist based approach here, together with limit ?
	for (i = 0; i < r; i++)
	{
		MeshIsland *mi2 = meshIslands[(n3+i)->index];
		if ((mi != mi2) && (mi2 != NULL))
		{
			float thresh;
			int con_type, equal, ok;
			equal = mi->parent_mod == mi2->parent_mod;
			ok = equal || (!equal && rmd->outer_constraint_type == RBC_TYPE_FIXED);
			thresh = equal ? rmd->breaking_threshold : rmd->group_breaking_threshold;
			con_type = equal ? rmd->inner_constraint_type : rmd->outer_constraint_type;

			if (((i >= limit) && (limit > 0)) || !ok)
			{
				break;
			}
			
			//#pragma omp critical
			connect_meshislands(rmd, ob, mi, mi2, con_type, thresh);
		}
	}

	if (n3 != NULL)
	{
		MEM_freeN(n3);
		n3 = NULL;
	}
}

KDTree* make_cell_tree(RigidBodyModifierData* rmd, Object* ob)
{
	float min[3], max[3], start[3], dim[3], co[3], csize, size[3];
	int cells[3], index = 0, i, j, k;
	KDTree* tree;
	BoundBox* bb = BKE_boundbox_alloc_unit();
	DerivedMesh* dm = CDDM_from_mesh(ob->data, ob); 
	dm_minmax(dm, min, max);
	BKE_boundbox_init_from_minmax(bb, min, max);
	copy_v3_v3(start, bb->vec[0]);
	bbox_dim(bb, dim);
	
	//invert_m4_m4(ob->imat, ob->obmat);
	//mat4_to_size(size, ob->obmat);
	//mul_v3_v3(dim, size);
	//mul_v3_v3(start, size);
	csize = rmd->cell_size;
	
	/*cells[0] = (int)(dim[0] / csize)+1;
	cells[1] = (int)(dim[1] / csize)+1;
	cells[2] = (int)(dim[2] / csize)+1;*/
	
	cells[0] = (int)(ceil(dim[0] / csize));
	cells[1] = (int)(ceil(dim[1] / csize));
	cells[2] = (int)(ceil(dim[2] / csize));
	
	tree = BLI_kdtree_new(cells[0]*cells[1]*cells[2]);
	
	while (rmd->cells.first)
	{
		NeighborhoodCell *cell = rmd->cells.first;
		if (cell != NULL)
		{
			BLI_remlink(&rmd->cells, cell);
			MEM_freeN(cell);
		}
	}
	rmd->cells.first = NULL;
	rmd->cells.last = NULL;
	
	for (i = 0; i < cells[0]; i++)
	{
		for (j = 0; j < cells[1]; j++)
		{
			for (k = 0; k < cells[2]; k++)
			{
				NeighborhoodCell* cell = MEM_callocN(sizeof(NeighborhoodCell), "cell");
				//cell->islands = MEM_callocN(sizeof(MeshIsland*), "islands");
				
				co[0] = start[0] + i * csize + csize * 0.5f;
				co[1] = start[1] + j * csize + csize * 0.5f;
				co[2] = start[2] + k * csize + csize * 0.5f;
				mul_m4_v3(ob->obmat, co);
				BLI_kdtree_insert(tree, index, co, NULL);
				index++;
				
				copy_v3_v3(cell->co, co);
				//printf("CELL: (%f %f %f)")
				BLI_addtail(&rmd->cells, cell);
				//cell->size = csize; //hmm not necessary to store -> equal
			}
		}
	}
	
	printf("Cells %d\n", BLI_countlist(&rmd->cells));
	BLI_kdtree_balance(tree);
	MEM_freeN(bb);
	dm->release(dm);
	dm = NULL;
	
	return tree;
}

void search_cell_based(RigidBodyModifierData *rmd, Object* ob,  MeshIsland *mi, KDTree** cells)
{
	KDTreeNearest *n = NULL;
	//use search distance based on cell volume ?
	int i, r = 0, j;
	float obj_centr[3], dim[3], dist;
	bbox_dim(mi->bb, dim);
	
	dist = MAX3(dim[0], dim[1], dim[2]) + rmd->cell_size + rmd->contact_dist;
	mul_v3_m4v3(obj_centr, ob->obmat, mi->centroid);
	r = BLI_kdtree_range_search(*cells, obj_centr, NULL, &n, dist);
	
	for (i = 0; i < r; i++)
	{
		int index = (n+i)->index;
		NeighborhoodCell* c = BLI_findlink(&rmd->cells, index);
		c->islands = MEM_reallocN(c->islands, sizeof(MeshIsland*) * (c->island_count+1));
		c->islands[c->island_count] = mi;
		c->island_count++;
	}

	if (n != NULL) {
		MEM_freeN(n);
		n = NULL;
	}
}

void search_cell_centroid_based(RigidBodyModifierData *rmd, Object* ob,  MeshIsland *mi, MeshIsland** meshIslands, KDTree** combined_tree, KDTree** cells)
{
	KDTreeNearest *n = NULL;
	//use search distance based on cell volume ?
	int i, r = 0, j;
	float obj_centr[3], dim[3], dist, co[3];
	bbox_dim(mi->bb, dim);
	
	dist = MAX2(MAX3(dim[0], dim[1], dim[2]), rmd->cell_size);
	mul_v3_m4v3(obj_centr, ob->obmat, mi->centroid);
	//copy_v3_v3(obj_centr, mi->centroid);
	r = BLI_kdtree_range_search(*cells, obj_centr, NULL, &n, dist);
	for (i = 0; i < r; i++)
	{
		copy_v3_v3(co, (n+i)->co);
		search_centroid_based(rmd, ob, mi, meshIslands, combined_tree, co);
	}
	
	if (n != NULL) {
		MEM_freeN(n);
		n = NULL;
	}
}

void connect_constraints(RigidBodyModifierData* rmd,  Object* ob, MeshIsland **meshIslands, int count, BMesh **combined_mesh, KDTree **combined_tree) {

	MeshIsland *mi, *first, *last;
	int i, j, v; //, sel_counter;
	KDTreeNearest *n = MEM_mallocN(sizeof(KDTreeNearest)*count, "kdtreenearest");
	KDTreeNearest *n2 = MEM_mallocN(sizeof(KDTreeNearest)*count, "kdtreenearest2");
	ExplodeModifierData *emd = NULL;

	KDTree* face_tree;
	BMFace *fa;
	BMIter bmi;

	face_tree = BLI_kdtree_new(rmd->visible_mesh->totface);
	BM_ITER_MESH(fa, &bmi, rmd->visible_mesh, BM_FACES_OF_MESH)
	{
		float co[3];
		BM_face_calc_center_bounds(fa, co);
		BLI_kdtree_insert(face_tree, fa->head.index, co, fa->no);
	}

	BLI_kdtree_balance(face_tree);

	//handle outer constraints here, connect the closest pairs of meshislands (1 per object) only
	if (rmd->use_constraints && rmd->constraint_group != NULL && rmd->outer_constraint_type != RBC_TYPE_FIXED) {
		GroupObject *go, *go2;
		ModifierData *md, *md2;
		RigidBodyModifierData* rbmd;
		MeshIsland* mil;
		GHash* trees = BLI_ghash_ptr_new("trees");
		GHash* closest_all = BLI_ghash_ptr_new("closest_all");
		GHashIterator it, it2, it3, it4;


		KDTree *tree, *obtree;
		int count, x = 0, y = 0, obcount;

		count = BLI_countlist(&rmd->meshIslands);
		obcount = BLI_countlist(&rmd->constraint_group->gobject)+1;
		tree = BLI_kdtree_new(count);
		obtree = BLI_kdtree_new(obcount);
		for (mil = rmd->meshIslands.first; mil; mil = mil->next) {
			BLI_kdtree_insert(tree, x, mil->centroid, NULL);
			x++;
		}

		BLI_kdtree_balance(tree);
		BLI_ghash_insert(trees, ob, tree);

		x = 0;
		for (go = rmd->constraint_group->gobject.first; go; go = go->next) {

			for (md = go->ob->modifiers.first; md; md = md->next) {
				if (md->type == eModifierType_RigidBody) {
					rbmd = (RigidBodyModifierData*)md;
					count = BLI_countlist(&rbmd->meshIslands);
					tree = BLI_kdtree_new(count);
					for (mil = rbmd->meshIslands.first; mil; mil = mil->next) {
						BLI_kdtree_insert(tree, x, mil->centroid, NULL);
						x++;
					}

					BLI_kdtree_balance(tree);
					BLI_ghash_insert(trees, go->ob, tree);
					BLI_kdtree_insert(obtree, y, go->ob->loc, NULL);
					y++;
				}
			}
		}

		BLI_kdtree_insert(obtree, y, ob->loc, NULL);
		BLI_kdtree_balance(obtree);

		for (go = rmd->constraint_group->gobject.first; go; go = go->next) {
			GHash* closest_ob = BLI_ghash_ptr_new("closest_ob");
			for (md = go->ob->modifiers.first; md; md = md->next)
			{
				if (md->type == eModifierType_RigidBody) {
					int index;

					rbmd = (RigidBodyModifierData*)md;
					tree = BLI_ghash_lookup(trees, ob);
					index = BLI_kdtree_find_nearest(tree, go->ob->loc, NULL, NULL);
					mil = BLI_findlink(&rmd->meshIslands, index);
					BLI_ghash_insert(closest_ob, go->ob, mil);
				}
			}
			BLI_ghash_insert(closest_all, ob, closest_ob);
		}

		for (go = rmd->constraint_group->gobject.first; go; go = go->next) {
			for (md = go->ob->modifiers.first; md; md = md->next)
			{
				if (md->type == eModifierType_RigidBody) {
					GHash* closest_ob = BLI_ghash_ptr_new("closest_ob");
					int index;
					rbmd = (RigidBodyModifierData*)md;
					tree = BLI_ghash_lookup(trees, go->ob);
					//handle ob here too
					index = BLI_kdtree_find_nearest(tree, ob->loc, NULL, NULL);
					mil = BLI_findlink(&rbmd->meshIslands, index);
					BLI_ghash_insert(closest_ob, ob, mil);

					for (go2 = rmd->constraint_group->gobject.first; go2; go2 = go2->next) {
						if (go2->ob != go->ob) {
							for (md2 = go2->ob->modifiers.first; md2; md2 = md2->next) {
								if (md2->type == eModifierType_RigidBody) {
									index = BLI_kdtree_find_nearest(tree, go2->ob->loc, NULL, NULL);
									mil = BLI_findlink(&rbmd->meshIslands, index);
									BLI_ghash_insert(closest_ob, go2->ob, mil);
								}
							}
						}
					}

					BLI_ghash_insert(closest_all, go->ob, closest_ob);
				}
			}
		}

		//connect the closestobjects
		GHASH_ITER(it2, closest_all) {
			MeshIsland* mil1, *mil2;
			Object *ob1 = BLI_ghashIterator_getKey(&it2);
			Object *ob2;

			GHash* ob1_closest = BLI_ghashIterator_getValue(&it2);
			GHash* ob2_closest;

			//RigidBodyShardCon* rbsc, *con;
			int index;

			//find 2 nearest because the first is the object itself !!
			KDTreeNearest* near = MEM_mallocN(sizeof(KDTreeNearest)*2, "near");
			BLI_kdtree_find_nearest_n(obtree, ob1->loc, NULL, near, 2);
			index = near[1].index;

			if ((index < obcount-1) && (index >= 0)) {
				GroupObject* go = BLI_findlink(&rmd->constraint_group->gobject, index);
				ob2 = go->ob;
			}
			else if (index == obcount-1) {
				ob2 = ob;
			}

			ob2_closest = BLI_ghash_lookup(closest_all, ob2);

			if (ob2_closest) {
				//find closest pair of mesh islands for object pair
				mil1 = BLI_ghash_lookup(ob1_closest, ob2);
				mil2 = BLI_ghash_lookup(ob2_closest, ob1);
			}
			else
			{
				mil1 = NULL;
				mil2 = NULL;
			}

			connect_meshislands(rmd, ob, mil1, mil2, rmd->outer_constraint_type, rmd->group_breaking_threshold);
			MEM_freeN(near);
		}

		GHASH_ITER(it3, closest_all)
		{
			GHash* h = BLI_ghashIterator_getValue(&it3);
			BLI_ghash_free(h, NULL, NULL);
		}

		GHASH_ITER(it4, trees)
		{
			KDTree* t = BLI_ghashIterator_getValue(&it4);
			BLI_kdtree_free(t);
		}
		BLI_kdtree_free(obtree);
		BLI_ghash_free(trees, NULL, NULL);
		BLI_ghash_free(closest_all, NULL, NULL);
	}


	//Do we have a explo modifier, if yes, use its neighborhood info before calculating (inner) neighborhoods here

	emd = findPrecedingExploModifier(ob, rmd);
	if (emd != NULL && emd->cells != NULL && ((!emd->use_clipping && !rmd->use_cellbased_sim  && !emd->use_boolean) || 
		((rmd->contact_dist_meaning == MOD_RIGIDBODY_VERTICES) && (rmd->contact_dist == 0.0f)))) {
		int i = 0, j;
		GHash* visited_ids = BLI_ghash_pair_new("visited_ids");
		for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
			MeshIsland* mi2;
			int shared = 0;

			if (rmd->auto_merge && mi->is_at_boundary) {
				for (v = 0; v < mi->vertex_count; v++) {
					//BMVert* ve = BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]);
					BMVert* ve = mi->vertices[v];
					select_inner_faces_of_vert(rmd, face_tree, ve);
				}
			}


			for (i = 0; i < mi->neighbor_count; i++) {
				int id = mi->neighbor_ids[i];
				int index;
				if (id >= 0) {
					index = GET_INT_FROM_POINTER(BLI_ghash_lookup(rmd->idmap, SET_INT_IN_POINTER(id)));
					mi2 = BLI_findlink(&rmd->meshIslands, index);
					if ((mi != mi2) && (mi2 != NULL)) {
						GHashPair* id_pair = BLI_ghashutil_pairalloc(id, mi->id);

						if (rmd->auto_merge && mi2->is_at_boundary) {
							for (v = 0; v < mi2->vertex_count; v++) {
								BMVert* ve2 = mi2->vertices[v];
								select_inner_faces_of_vert(rmd, face_tree, ve2);
							}
						}

						if (!BLI_ghash_haskey(visited_ids, id_pair)) {
							//shared = check_meshislands_adjacency(rmd, mi, mi2, combined_mesh, face_tree, ob);
							//RigidBodyShardCon *con;
							//int con_found = FALSE;
							BLI_ghash_insert(visited_ids, id_pair, SET_INT_IN_POINTER(i));
							
							connect_meshislands(rmd, ob, mi, mi2, rmd->inner_constraint_type, rmd->breaking_threshold);
						}
						else
						{
							if (!mi->is_at_boundary && !mi2->is_at_boundary)
							{
								//use fast path for interior shards

								int glob, glob2;
								//now, if cell not altered by boolean, can select inner faces
								// i is index of face in mi, need to find face index of visitor id
								int secondface = GET_INT_FROM_POINTER(BLI_ghash_lookup(visited_ids, id_pair));
								//get global face index and select them
								glob = mi->global_face_map[i];
								glob2 = mi2->global_face_map[secondface]; //was "pair" necessary here ???

								rmd->sel_indexes = MEM_reallocN(rmd->sel_indexes, sizeof(int*) * (rmd->sel_counter+1));
								rmd->sel_indexes[rmd->sel_counter] = MEM_callocN(sizeof(int)*2, "sel_index_pair");
								rmd->sel_indexes[rmd->sel_counter][0] = glob;
								rmd->sel_indexes[rmd->sel_counter][1] = glob2;
								rmd->sel_counter++;
							}
							BLI_ghashutil_pairfree(id_pair);
						}
					}
				}
			}

			//GO through OUTER shards HERE only // determine this during fracture if atleast one index is negative
			if (rmd->constraint_group != NULL) {
				for (v = 0; v < mi->vertex_count; v++) {
					BMVert* ve = BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]);
					BM_elem_flag_enable(ve, BM_ELEM_TAG);
				}

				//prepare for outer constraints, neighborhood is negative then (if we reach a boundary)
				BLI_kdtree_find_nearest_n(*combined_tree, mi->centroid, NULL, n, count);
				for (j = 0; j < count; j++) {
					mi2 = meshIslands[(n+j)->index];
					if ((mi != mi2) && (mi2 != NULL) && (mi2->parent_mod != rmd)) {
						int bbox_int = bbox_intersect(rmd, mi, mi2);
						//printf("Overlap %d %d %d\n", bbox_int, (n2+j)->index, (n+i)->index);
						if (bbox_int == FALSE)
							continue;

						shared = check_meshislands_adjacency(rmd, mi, mi2, combined_mesh, face_tree, ob);
						if (shared == 0) break;
					}
				}
			}

			for (v = 0; v < mi->vertex_count; v++) {
				BM_elem_flag_disable(BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]), BM_ELEM_TAG);
			}
		}

		BLI_ghash_free(visited_ids, BLI_ghashutil_pairfree, NULL);
		visited_ids = NULL;
	}
	else
	{
		KDTree* cells = NULL;
		
		if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELLS || rmd->contact_dist_meaning == MOD_RIGIDBODY_CELL_CENTROIDS)
		{
			cells = make_cell_tree(rmd, ob);
		}
		//without explo modifier, automerge is useless in most cases (have non-adjacent stuff mostly

		//BLI_kdtree_find_n_nearest(*combined_tree, count, meshIslands[0]->centroid, NULL, n2);
		//KDTreeNearest* n3;
		//MeshIsland* mi2;
		//#pragma omp parallel for private(n3) schedule(static)
		for (j = 0; j < count; j++) {

			if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CENTROIDS)
			{
				search_centroid_based(rmd, ob, meshIslands[j], meshIslands, combined_tree, NULL);
			}
			else if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELLS) 
			{
				search_cell_based(rmd, ob, meshIslands[j], &cells);
			}
			else if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELL_CENTROIDS)
			{
				search_cell_centroid_based(rmd, ob, meshIslands[j], meshIslands, combined_tree, &cells);
			}
			else if (rmd->contact_dist_meaning == MOD_RIGIDBODY_VERTICES)//use vertex distance as FALLBACK
			{
				int shared = 0;
				BLI_kdtree_find_nearest_n(*combined_tree, meshIslands[0]->centroid, NULL, n2, count);
				mi = meshIslands[(n2+j)->index];

				for (v = 0; v < mi->vertex_count; v++) {
					BM_elem_flag_enable(BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]), BM_ELEM_TAG);
				}

				BLI_kdtree_find_nearest_n(*combined_tree, mi->centroid, NULL, n, count);
				for (i = 0; i < count; i++) {
					MeshIsland *mi2 = meshIslands[(n+i)->index];
					//printf("Nearest: %d %d %f %f %f\n",m, (n+i)->index, (n+i)->co[0], (n+i)->co[2], (n+i)->co[2]);
					if ((mi != mi2) && (mi2 != NULL)) {
						//pre test with bounding boxes
						//vec between centroids -> v, if v[0] > bb1[4] - bb1[0] / 2 + bb2[4] - bb[0] / 2 in x range ?
						// analogous y and z, if one overlaps, bboxes touch, make expensive test then
						int bbox_int = bbox_intersect(rmd, mi, mi2);
						//printf("Overlap %d %d %d\n", bbox_int, (n2+j)->index, (n+i)->index);
						if ((bbox_int == FALSE) && (mi->parent_mod == mi2->parent_mod))
							continue;

						shared = check_meshislands_adjacency(rmd, mi, mi2, combined_mesh, face_tree, ob);
//						if ((shared == 0) && (rmd->inner_constraint_type == RBC_TYPE_FIXED) && (rmd->outer_constraint_type == RBC_TYPE_FIXED))
//							break; //load faster when both constraint types are FIXED, otherwise its too slow or incorrect

						/*if ((j == (count-1)) && (i == (count-2))) {
							last = mi2;
						}*/

						if ((rmd->constraint_limit > 0) && (i >= rmd->constraint_limit))
						{
							break;
						}
					}
				}
				//if (shared == 0) break; // should be too far away already, farther than dist

				for (v = 0; v < mi->vertex_count; v++) {
					BM_elem_flag_disable(BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]), BM_ELEM_TAG);
				}
			}
		}
		//compare last with first
		//check_meshislands_adjacency(rmd, last, first, combined_mesh, face_tree, ob);
		if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELLS)
		{
			NeighborhoodCell* cell;
			for (cell = rmd->cells.first; cell; cell = cell->next)
			{
				for (i = 0; i < cell->island_count; i++)
				{
					MeshIsland* mi = cell->islands[i];
					for (j = 0; j < cell->island_count; j++)
					{
						float thresh;
						int con_type, equal, ok, limit;
						MeshIsland* mi2 = cell->islands[j];
						if (mi != mi2)
						{
							equal = mi->parent_mod == mi2->parent_mod;
							ok = equal || (!equal && rmd->outer_constraint_type == RBC_TYPE_FIXED);
							thresh = equal ? rmd->breaking_threshold : rmd->group_breaking_threshold;
							con_type = equal ? rmd->inner_constraint_type : rmd->outer_constraint_type;
							limit = rmd->constraint_limit;
				
							if (((i >= limit) && (limit > 0)) || !ok)
							{
								break;
							}
							
							connect_meshislands(rmd, ob, mi, mi2, con_type, thresh);
						}
					}
				}
			}
		}
		
		if(rmd->contact_dist_meaning == MOD_RIGIDBODY_CELLS ||  rmd->contact_dist_meaning == MOD_RIGIDBODY_CELL_CENTROIDS)
		{
			if (cells != NULL)
			{
				//maybe create this once, and not per refresh (its the same always...)
				BLI_kdtree_free(cells);
				cells = NULL;
			}
		}
	}

	MEM_freeN(n);
	MEM_freeN(n2);
	BLI_kdtree_free(face_tree);
	face_tree = NULL;
}

static int create_combined_neighborhood(RigidBodyModifierData *rmd, MeshIsland ***mesh_islands, BMesh **combined_mesh, KDTree **combined_tree)
{
	ModifierData* md;
	RigidBodyModifierData* rmd2;
	GroupObject* go;
	MeshIsland* mi;
	int v, i = 0, islands = 0, vert_counter = 0;

	//create a combined mesh over all part bmeshes, and a combined kdtree to find "outer" constraints as well
	//handle single object here
	*combined_mesh = BM_mesh_create(&bm_mesh_allocsize_default);
	BM_mesh_elem_toolflags_ensure(*combined_mesh);

	islands = BLI_countlist(&rmd->meshIslands);
	*mesh_islands = MEM_reallocN(*mesh_islands, islands*sizeof(MeshIsland*));
	for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
		mi->combined_index_map = MEM_mallocN(mi->vertex_count*sizeof(int), "combined_index_map");
		for (v = 0; v < mi->vertex_count; v++) {
			float co[3];
			copy_v3_v3(co, mi->vertices[v]->co);
			BM_vert_create(*combined_mesh, co, NULL, 0);
			mi->combined_index_map[v] = vert_counter;
			vert_counter++;
		}
		(*mesh_islands)[i] = mi;
		i++;
	}

	//handle a group of objects to be taken account into as well
	if (rmd->constraint_group != NULL) {
		for (go = rmd->constraint_group->gobject.first; go; go = go->next) {
			for (md = go->ob->modifiers.first; md; md = md->next) {
				if (md->type == eModifierType_RigidBody) {
					rmd2 = (RigidBodyModifierData*)md;
					rmd2->constraint_group = rmd->constraint_group;
					islands += BLI_countlist(&rmd2->meshIslands);
					*mesh_islands = MEM_reallocN(*mesh_islands, islands*sizeof(MeshIsland*));
					for (mi = rmd2->meshIslands.first; mi; mi = mi->next) {
						mi->combined_index_map = MEM_mallocN(mi->vertex_count*sizeof(int), "combined_index_map");
						for (v = 0; v < mi->vertex_count; v++) {
							float co[3];
							copy_v3_v3(co, mi->vertices[v]->co);
							BM_vert_create(*combined_mesh, co, NULL, 0);
							mi->combined_index_map[v] = vert_counter;
							vert_counter++;
						}
						(*mesh_islands)[i] = mi;
						i++;
					}
					//rmd2->refresh = TRUE;
				}
				//break;
			}
		}
	}

	*combined_tree = BLI_kdtree_new(islands);
	for (i = 0; i < islands; i++) {
		float obj_centr[3];
		mul_v3_m4v3(obj_centr, (*mesh_islands)[i]->parent_mod->origmat, (*mesh_islands)[i]->centroid );
		BLI_kdtree_insert(*combined_tree, i, obj_centr, NULL);
	}

	BLI_kdtree_balance(*combined_tree);

	return islands;
}

static void create_constraints(RigidBodyModifierData *rmd, Object* ob)
{
	KDTree* combined_tree = NULL;
	BMesh* combined_mesh = NULL;
	MeshIsland** mesh_islands = MEM_mallocN(sizeof(MeshIsland*), "mesh_islands");
	int count, i = 0;

	count = create_combined_neighborhood(rmd, &mesh_islands, &combined_mesh, &combined_tree);

	if ((combined_mesh != NULL) && (combined_tree != NULL))
		connect_constraints(rmd, ob, mesh_islands, count, &combined_mesh, &combined_tree);

	if (combined_tree != NULL) {
		BLI_kdtree_free(combined_tree);
		combined_tree = NULL;
	}

	if (combined_mesh != NULL) {
		BM_mesh_free(combined_mesh);
		combined_mesh = NULL;
	}

	for (i = 0; i < count; i++) {
		MEM_freeN(mesh_islands[i]->combined_index_map);
	}

	MEM_freeN(mesh_islands);
}

BMFace* findClosestFace(KDTree* tree, BMesh* bm, BMFace* f)
{
	int index;
	float co[3];
	BMFace *f1, *f2;
	KDTreeNearest *n = MEM_mallocN(sizeof(KDTreeNearest)*2, "nearest");

	BM_face_calc_center_bounds(f, co);
	BLI_kdtree_find_nearest_n(tree, co, NULL, n, 2);

	index = n[0].index;
	f1 = BM_face_at_index(bm, index);

	index = n[1].index;
	f2 = BM_face_at_index(bm, index);

	MEM_freeN(n);

	if (f == f1){
		return f2;
	}
	return f1;
}

BMFace* closest_available_face(RigidBodyModifierData* rmd, KDTree* tree, BMesh* bm,  BMFace* f, BMFace* f2)
{
	float co[3], co2[3];

	if (!f) return findClosestFace(tree, bm, f2);

	BM_face_calc_center_bounds(f, co);
	BM_face_calc_center_bounds(f2, co2);

	if (!compare_v3v3(co, co2, rmd->auto_merge_dist)) {
		return findClosestFace(tree, bm, f2);
	}

	return NULL;
}

KDTree* make_face_tree(RigidBodyModifierData* rmd, BMesh* merge_copy)
{
	KDTree *tree;
	BMIter iter;
	BMFace* face;

	tree = BLI_kdtree_new(rmd->visible_mesh->totface);

	//build tree of left selected faces
	BM_ITER_MESH(face, &iter, merge_copy, BM_FACES_OF_MESH)
	{
		if (BM_elem_flag_test(face, BM_ELEM_TAG))
		{
			float co[3];
			BM_face_calc_center_bounds(face, co);
			BLI_kdtree_insert(tree, face->head.index, co, NULL);
		}
	}

	BLI_kdtree_balance(tree);
	return tree;
}


void check_face_by_adjacency(RigidBodyModifierData *rmd, BMesh *merge_copy, int i, KDTree *tree)
{
	BMFace *f, *f2, *f3, *f4;
	int index = rmd->sel_indexes[i][0];
	int index2 = rmd->sel_indexes[i][1];

	f = BM_face_at_index(merge_copy, index);
	f2 = BM_face_at_index(rmd->visible_mesh, index);
	f3 = closest_available_face(rmd, tree, merge_copy, f, f2);

	if (f3 != NULL) {
		BM_elem_flag_enable(f3, BM_ELEM_SELECT);
	}

	f = BM_face_at_index(merge_copy, index2);
	f2 = BM_face_at_index(rmd->visible_mesh, index2);
	f4 = closest_available_face(rmd, tree, merge_copy, f, f2);


	if (f4 != NULL) {
		BM_elem_flag_enable(f4, BM_ELEM_SELECT);
	}
}

void check_face_draw_by_constraint(RigidBodyModifierData* rmd, BMesh* merge_copy) {

	int sel = rmd->sel_counter;
	int i = 0;
	BMFace *face, *face2;
	KDTree* tree;

	tree = make_face_tree(rmd, merge_copy);

	for (i = 0; i < sel; i++)
	{
		face = BM_face_at_index(merge_copy, rmd->sel_indexes[i][0]);
		face2 = BM_face_at_index(merge_copy, rmd->sel_indexes[i][1]);

		if ((face == NULL) || (face2 == NULL)) continue;
		BM_elem_flag_enable(face, BM_ELEM_TAG);
		BM_elem_flag_enable(face2, BM_ELEM_TAG);
		if (BLI_countlist(&rmd->meshConstraints) > 0) {
			RigidBodyShardCon* con = BLI_findlink(&rmd->meshConstraints,i);
			if (con && con->physics_constraint) {
				if (RB_constraint_is_enabled(con->physics_constraint)) {
					BM_elem_flag_enable(face, BM_ELEM_SELECT);
					BM_elem_flag_enable(face2, BM_ELEM_SELECT);
				}
				else
				{
					BM_elem_flag_disable(face, BM_ELEM_SELECT);
					BM_elem_flag_disable(face2, BM_ELEM_SELECT);
				}
			}
			else
			{
				//BM_elem_flag_disable(face, BM_ELEM_SELECT);
				//BM_elem_flag_disable(face2, BM_ELEM_SELECT);
				BMFace* f;
				f = findClosestFace(tree, merge_copy, face);
				if (f == face2)
				{
					BM_elem_flag_enable(face, BM_ELEM_SELECT);
					BM_elem_flag_enable(face2, BM_ELEM_SELECT);
				}
				else
				{
					BM_elem_flag_disable(face, BM_ELEM_SELECT);
					BM_elem_flag_disable(face2, BM_ELEM_SELECT);
				}
			}
		}
		else
		{
			BM_elem_flag_disable(face, BM_ELEM_SELECT);
			BM_elem_flag_disable(face2, BM_ELEM_SELECT);
		}
	}

	BLI_kdtree_free(tree);
	tree = NULL;
}

void check_face_draw_by_proximity(RigidBodyModifierData* rmd, BMesh* merge_copy) {

	int sel = rmd->sel_counter;
	int i = 0;
	KDTree* tree;

	//merge vertices
	BMO_op_callf(merge_copy, BMO_FLAG_DEFAULTS, "automerge verts=%av dist=%f", BM_VERTS_OF_MESH, 0.000001f);

	tree = make_face_tree(rmd, merge_copy);

	//delete invisible inner faces, check which faces have been merged away, delete those who still there ?
	for (i = 0; i < sel; i++) {
		check_face_by_adjacency(rmd, merge_copy, i, tree);
	}

	BLI_kdtree_free(tree);
	tree = NULL;
}

void buildCompounds(RigidBodyModifierData* rmd, Object *ob)
{
	//join new meshislands into 1 new, remove other ones
	float centroid[3], dummyloc[3], rot[4], min[3], max[3], size[3];
	int count = BLI_countlist(&rmd->meshIslands);
	int index = 0, i = 0, j = 0, rbcount = 0;
	KDTree *centroidtree = BLI_kdtree_new(count);
	NeighborhoodCell* cell;
	KDTreeNearest* n = NULL;
	MeshIsland *mi;
	RigidBodyWorld *rbw = rmd->modifier.scene->rigidbody_world;
	
	//create cell array... / tree (ugh, how inefficient..)
	KDTree* celltree = make_cell_tree(rmd, ob);
	BLI_kdtree_free(celltree); //silly but only temporary...
	
	
	if (rbw && !(rbw->pointcache->flag & PTCACHE_BAKED))
	{
		for (mi = rmd->meshIslands.first; mi; mi = mi->next)
		{
			if (mi->compound_count > 0)	//clean up old compounds first
			{
				for (i = 0; i < mi->compound_count; i++)
				{
					MeshIsland* mi2 = mi->compound_children[i];
					if (mi2->rigidbody != NULL)
					{
						BKE_rigidbody_remove_shard(rmd->modifier.scene, mi2);
						MEM_freeN(mi2->rigidbody);
						mi2->rigidbody = NULL;
					}
					mi2->destruction_frame = -1;
					mi2->compound_parent = NULL;
				}
				BKE_rigidbody_remove_shard(rmd->modifier.scene, mi);
				BLI_remlink(&rmd->meshIslands, mi);
				freeMeshIsland(rmd, mi);
			}
		}
	}
	
	for (mi = rmd->meshIslands.first; mi; mi = mi->next)
	{
		//do this in object space maybe... ? TODO
		float obj_centr[3];
		//mul_v3_m4v3(obj_centr, ob->obmat, mi->centroid);
		copy_v3_v3(obj_centr, mi->centroid);
		BLI_kdtree_insert(centroidtree, index, obj_centr, NULL);
		index++;
	}
	
	BLI_kdtree_balance(centroidtree);
	
	invert_m4_m4(ob->imat, ob->obmat);
	mat4_to_size(size, ob->imat);
	for (cell = rmd->cells.first; cell; cell = cell->next)
	{
		BMesh *bm_compound;
		BMVert* v;
		BMIter iter;
		MeshIsland *mi_compound;
		float co[3];
		int r;
		
		mul_v3_m4v3(co, ob->imat, cell->co);
		r = BLI_kdtree_range_search(centroidtree, co, NULL, &n, rmd->cell_size);
		if (r == 0)
			continue;
		
		bm_compound = BM_mesh_create(&bm_mesh_allocsize_default);
		mi_compound = MEM_callocN(sizeof(MeshIsland), "mi_compound");
		mi_compound->vertices = MEM_callocN(sizeof(BMVert*), "compoundverts");
		mi_compound->vertco = MEM_callocN(sizeof(float), "compoundvertco");
		mi_compound->vertex_count = 0;
		mi_compound->compound_children = MEM_callocN(sizeof(MeshIsland*), "compoundchilds");
		mi_compound->compound_count = 0;
		mi_compound->compound_parent = NULL;
		mi_compound->participating_constraint_count = 0;
		mi_compound->destruction_frame = -1;
		mi_compound->vertices_cached = NULL;
		
		printf("Joining %d islands to compound\n", r);
		for (i = 0; i < r; i++)
		{
			MeshIsland *mi = BLI_findlink(&rmd->meshIslands, n[i].index);
			BMesh *bm;
			if (mi->compound_parent != NULL || mi->compound_count > 0)
			{	//dont compound shards with a parent or with children (no recursion)
				continue;
			}
			
			bm = DM_to_bmesh(mi->physics_mesh, true);
			
			BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
				add_v3_v3(v->co, mi->centroid);
			}
			
			mi_compound->vertices = MEM_reallocN(mi_compound->vertices, sizeof(BMVert*) * (mi_compound->vertex_count + mi->vertex_count));
			mi_compound->vertco = MEM_reallocN(mi_compound->vertco, sizeof(float)*3 *(mi_compound->vertex_count + mi->vertex_count));
			//copy verts and vertco
			for (j = 0; j < mi->vertex_count; j++)
			{
				mi_compound->vertices[mi_compound->vertex_count+j] = mi->vertices[j];
				mi_compound->vertco[(mi_compound->vertex_count + j)*3] = mi->vertco[j*3];
				mi_compound->vertco[(mi_compound->vertex_count + j)*3+1] = mi->vertco[j*3+1];
				mi_compound->vertco[(mi_compound->vertex_count + j)*3+2] = mi->vertco[j*3+2];
			}
			mi_compound->vertex_count += mi->vertex_count;
			
			BM_mesh_join(&bm_compound, bm); // write this...
			
			//BLI_remlink(&rmd->meshIslands, mi);
			/*to_remove = MEM_reallocN(to_remove, sizeof(MeshIsland*) * (to_remove_count+1));
			to_remove[to_remove_count] = mi;
			to_remove_count++;*/
			
			mi_compound->compound_children = MEM_reallocN(mi_compound->compound_children, sizeof(MeshIsland*) * (mi_compound->compound_count+1));
			mi_compound->compound_children[mi_compound->compound_count] = mi; //memorize them and re add them simply later
			mi->compound_parent = mi_compound;
			mi_compound->compound_count++;
			
			BM_mesh_free(bm);
			count++;
		}
		
		if (n != NULL)
		{
			MEM_freeN(n);
			n = NULL;
		}
		
		if (bm_compound->totvert == 0)
		{
			MEM_freeN(mi_compound->vertices);
			MEM_freeN(mi_compound->vertco);
			MEM_freeN(mi_compound->compound_children);
			MEM_freeN(mi_compound);
			BM_mesh_free(bm_compound);
			continue;
		}
		
		//join derived mesh, hmm, maybe via bmesh
		BM_mesh_minmax(bm_compound, min, max, FALSE);
		if (ob->rigidbody_object->shape == RB_SHAPE_COMPOUND || rmd->use_constraints)
		{
			//compounds need center of mass
			BM_calc_center_centroid(bm_compound, centroid, FALSE);
		}
		else
		{	//other shapes like convexhull look better this way...
			mid_v3_v3v3(centroid, min, max);
		}
		
		//mul_v3_v3(centroid, size);
		
		for (i = 0; i < mi_compound->compound_count; i++)
		{
			//set proper relative starting centroid
			//sub_v3_v3v3(mi_compound->compound_children[i]->start_co, mi_compound->compound_children[i]->centroid, centroid);
			copy_v3_v3(mi_compound->compound_children[i]->start_co, mi_compound->compound_children[i]->centroid);
		}
		
		//invert_m4_m4(ob->imat, ob->obmat);
		BM_ITER_MESH (v, &iter, bm_compound, BM_VERTS_OF_MESH) {
			//then eliminate centroid in vertex coords ?
			sub_v3_v3(v->co, centroid);
		}
		
		mi_compound->physics_mesh = CDDM_from_bmesh(bm_compound, TRUE);
		BM_mesh_free(bm_compound);
		copy_v3_v3(mi_compound->centroid, centroid);
		mat4_to_loc_quat(dummyloc, rot, ob->obmat);
		copy_v3_v3(mi_compound->rot, rot);
		mi_compound->parent_mod = rmd;
		mi_compound->bb = BKE_boundbox_alloc_unit();
		BKE_boundbox_init_from_minmax(mi_compound->bb, min, max);
		mi_compound->participating_constraints = NULL;
		mi_compound->participating_constraint_count = 0;
		
		mi_compound->rigidbody = BKE_rigidbody_create_shard(rmd->modifier.scene, ob, mi_compound);
		BKE_rigidbody_calc_shard_mass(ob, mi_compound);
		mi_compound->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
		mi_compound->rigidbody->flag &= ~RBO_FLAG_ACTIVE_COMPOUND;
		mi_compound->rigidbody->flag |= RBO_FLAG_BAKED_COMPOUND;
		//BKE_rigidbody_validate_sim_shard(rmd->modifier.scene->rigidbody_world, mi_compound, ob, true);
		//mi_compound->rigidbody->flag &= ~RBO_FLAG_NEEDS_VALIDATE;
		
		BLI_addtail(&rmd->meshIslands, mi_compound);
		
		if (rmd->framemap != NULL && (rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED))
		{
			if (rmd->framecount >= rbcount)
			{
				int index = BLI_findindex(&rmd->meshIslands, mi_compound);
				mi_compound->destruction_frame = rmd->framemap[index];
			}
		}
	}
	
	//clean up compound children
	/*for (i = 0; i < to_remove_count; i++)
	{
		BLI_remlink(&rmd->meshIslands, to_remove[i]);
	}*/
	
	//if (to_remove != NULL)
	//	MEM_freeN(to_remove);
	BLI_kdtree_free(centroidtree);
}

static DerivedMesh* createCache(RigidBodyModifierData *rmd)
{
	MeshIsland *mi;
	BMVert* v;
	DerivedMesh *dm;
	MVert *verts;
	
	dm = CDDM_from_bmesh(rmd->visible_mesh, TRUE);
	DM_ensure_tessface(dm);
	DM_ensure_normals(dm);
	
	verts = dm->getVertArray(dm);
	for (mi = rmd->meshIslands.first; mi; mi = mi->next)
	{
		int i = 0;
		if (mi->vertices_cached)
		{
			MEM_freeN(mi->vertices_cached);
			mi->vertices_cached = NULL;
		}
		
		mi->vertices_cached = MEM_mallocN(sizeof(MVert*) * mi->vertex_count, "mi->vertices_cached");
		for (i = 0; i < mi->vertex_count; i++)
		{	
			int index = mi->vertices[i]->head.index;
			if (index >= 0 && index <= rmd->visible_mesh->totvert)
			{
				mi->vertices_cached[i] = verts + index;
			}
			else
			{
				mi->vertices_cached[i] = NULL;
			}
		}
	}
	
	return dm;
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
								  DerivedMesh *dm,
								  ModifierApplyFlag UNUSED(flag))
{

	RigidBodyModifierData *rmd = (RigidBodyModifierData *) md;
	ExplodeModifierData *emd = NULL;
	BMesh* temp = NULL;
	int exploOK = FALSE;
	double start;

	if (rmd->refresh || rmd->refresh_constraints)
	{
		//int len = 0;
		
		freeData(rmd);
		rmd->visible_mesh_cached = NULL;
		rmd->split = destroy_compound;
		rmd->join = buildCompounds;
		//rmd->vol_check = vol_check;
		if (rmd->contact_dist_meaning != MOD_RIGIDBODY_CELL_CENTROIDS)
		{
			rmd->use_cellbased_sim = FALSE;
		}
		
		if (rmd->refresh)
		{
			rmd->idmap = BLI_ghash_int_new("rmd->idmap");
			copy_m4_m4(rmd->origmat, ob->obmat);
	
			//grab neighborhood info (and whole fracture info -> cells) if available, if explo before rmd
			emd = findPrecedingExploModifier(ob, rmd);
			if (emd != NULL && emd->cells != NULL && ((!emd->use_clipping && !rmd->use_cellbased_sim /* && !emd->use_boolean*/) || rmd->contact_dist_meaning == MOD_RIGIDBODY_VERTICES)) 
			{
				MeshIsland* mi;
				VoronoiCell *vc;
				BMIter iter;
				int i, j;
				BMVert *v;
				float dummyloc[3], rot[4], min[3], max[3];
	
				//good idea to simply reference this ? Hmm, what about removing the explo modifier later, crash ?)
				rmd->explo_shared = TRUE;
				rmd->visible_mesh = emd->fracMesh;
				for (i = 0; i < emd->cells->count; i++) {
					vc = &emd->cells->data[i];
				//if (compare_v3v3(vc->centroid, centroid, 0.0001f))
				//{
					// add 1 MeshIsland
					mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
					BLI_addtail(&rmd->meshIslands, mi);
					
					mi->participating_constraints = NULL;
					mi->participating_constraint_count = 0;
					
					mi->is_at_boundary = vc->is_at_boundary;
					mi->vertices = vc->vertices;
					mi->vertco = vc->vertco;
					temp = DM_to_bmesh(vc->cell_mesh, true);
	
					BM_ITER_MESH (v, &iter, temp, BM_VERTS_OF_MESH) {
						//then eliminate centroid in vertex coords ?
						sub_v3_v3(v->co, vc->centroid); //or better calc this again
					}
					
					BM_mesh_minmax(temp, min, max, FALSE);
					mi->physics_mesh = CDDM_from_bmesh(temp, TRUE);
					BM_mesh_free(temp);
					temp = NULL;
	
					mi->vertex_count = vc->vertex_count;
					copy_v3_v3(mi->centroid, vc->centroid);
					mat4_to_loc_quat(dummyloc, rot, ob->obmat);
					copy_v3_v3(mi->rot, rot);
					mi->parent_mod = rmd;
	
					mi->bb = BKE_boundbox_alloc_unit();
					BKE_boundbox_init_from_minmax(mi->bb, min, max);
	
					mi->id = vc->pid;
					mi->particle_index = vc->particle_index;
					BLI_ghash_insert(rmd->idmap, SET_INT_IN_POINTER(mi->id), SET_INT_IN_POINTER(i));
					mi->neighbor_ids = vc->neighbor_ids;
					mi->neighbor_count = vc->neighbor_count;
					mi->global_face_map = vc->global_face_map;
					
					mi->rigidbody = NULL;
					mi->vertices_cached = NULL;
					if (!rmd->use_cellbased_sim)
					{
						mi->destruction_frame = -1;
						mi->rigidbody = BKE_rigidbody_create_shard(rmd->modifier.scene, ob, mi);
						mi->rigidbody->flag &= ~RBO_FLAG_ACTIVE_COMPOUND;
						BKE_rigidbody_calc_shard_mass(ob, mi);
					}
				//}
				}
			}
			else
			{
				//split to meshislands now
				rmd->visible_mesh = DM_to_bmesh(dm, true); //ensures indexes automatically
				rmd->explo_shared = FALSE;
				
				start = PIL_check_seconds_timer();
				mesh_separate_loose(rmd, emd, ob);
				printf("Splitting to islands done, %g\n", PIL_check_seconds_timer() - start);
			}
			
			printf("Islands: %d\n", BLI_countlist(&rmd->meshIslands));
		}
		
		if (rmd->use_cellbased_sim)
		{
			MeshIsland* mi;
			int i, count;
			//create Compounds HERE....go through all cells, find meshislands around cell centroids (make separate island tree)
			//and compound them together (storing in first=closest compound, removing from meshisland list, adding the compound...
			start = PIL_check_seconds_timer();
			buildCompounds(rmd, ob);
			printf("Building compounds done, %g\n", PIL_check_seconds_timer() - start);
			
			count = BLI_countlist(&rmd->meshIslands);
			printf("Compound Islands: %d\n", count);
			
			if (rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED && rmd->framemap != NULL)
			{
				int index = 0;
				for (mi = rmd->meshIslands.first; mi; mi = mi->next)
				{
					destroy_compound(rmd, ob, mi, rmd->framemap[index]);
					index++;
					//if (mi->rigidbody)
						//mi->rigidbody->flag &= ~RBO_FLAG_BAKED_COMPOUND;
				}
			}
			
			if (!rmd->refresh_constraints)
			{
				//rebuild framemap after using it when refreshing all data
				if (!(rmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED))
				{
					if (rmd->framemap != NULL)
					{
						MEM_freeN(rmd->framemap);
						rmd->framemap = NULL;
						rmd->framecount = 0;
					}
				}
				
				if (rmd->framemap == NULL)
				{
					rmd->framecount = count;
					rmd->framemap = MEM_callocN(sizeof(float) * rmd->framecount, "framemap");
					for (i = 0; i < rmd->framecount; i++)
					{
						rmd->framemap[i] = -1;
					}
				}
			}
		}
	
		start = PIL_check_seconds_timer();
		if ((rmd->visible_mesh != NULL) && ((rmd->use_constraints) || (rmd->auto_merge))) {
			if (((rmd->constraint_group != NULL) && (!BKE_group_object_exists(rmd->constraint_group, ob))) ||
					(rmd->constraint_group == NULL)) { // { || (rmd->auto_merge)) {
				create_constraints(rmd, ob); //check for actually creating the constraints inside
			}
		}
		
		printf("Building constraints done, %g\n", PIL_check_seconds_timer() - start); 
		printf("Constraints: %d\n", BLI_countlist(&rmd->meshConstraints));
	
		if (rmd->visible_mesh != NULL)
		{
			start = PIL_check_seconds_timer();
			//post process ... convert to DerivedMesh only at refresh times, saves permanent conversion during execution
			rmd->visible_mesh_cached = createCache(rmd);
			printf("Building cached DerivedMesh done, %g\n", PIL_check_seconds_timer() - start);
		}
		
		rmd->refresh = FALSE;
		rmd->refresh_constraints = FALSE;
	}

	emd = findPrecedingExploModifier(ob, rmd);
	exploOK = !rmd->explo_shared || (rmd->explo_shared && emd && emd->cells);

	if (!exploOK || rmd->visible_mesh == NULL)
	{
		MeshIsland* mi;
		//nullify invalid data
		for (mi = rmd->meshIslands.first; mi; mi = mi->next)
		{
			mi->vertco = NULL;
			mi->vertex_count = 0;
			mi->vertices = NULL;
			if (mi->vertices_cached)
			{
				MEM_freeN(mi->vertices_cached);
				mi->vertices_cached = NULL;
			}
		}
		
		if (rmd->visible_mesh_cached)
		{
			DM_release(rmd->visible_mesh_cached);
			MEM_freeN(rmd->visible_mesh_cached);
			rmd->visible_mesh_cached = NULL;
		}
	}

	if ((rmd->visible_mesh != NULL) && exploOK) {
		DerivedMesh *dm_final;
		if (rmd->auto_merge) {
			BMesh* merge_copy = BM_mesh_copy(rmd->visible_mesh);

			check_face_draw_by_constraint(rmd, merge_copy);
			if (rmd->auto_merge_dist > 0)
			{
				check_face_draw_by_proximity(rmd, merge_copy);
			}

			BMO_op_callf(merge_copy, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
						"delete geom=%hvef context=%i", BM_ELEM_SELECT, DEL_FACES);

			//final merge to close gaps
			BMO_op_callf(merge_copy, BMO_FLAG_DEFAULTS, "automerge verts=%hv dist=%f", BM_ELEM_SELECT, rmd->auto_merge_dist);

			dm_final = CDDM_from_bmesh(merge_copy, TRUE);
			BM_mesh_free(merge_copy);
		}
		else
		{
			//dm_final = rmd->visible_mesh_cached;
			dm_final = CDDM_copy(rmd->visible_mesh_cached);
			//dm_final = CDDM_from_bmesh(rmd->visible_mesh, TRUE);
		}
		
		return dm_final;
	}
	else {
		if (rmd->visible_mesh == NULL)
		{
			//oops, something went definitely wrong...
			if (emd) 
			{
				rmd->refresh = TRUE;
			}
			freeData(rmd);
			rmd->visible_mesh_cached = NULL;
			rmd->refresh = FALSE;
		}
		
		return dm;
	}
}

static int dependsOnTime(ModifierData *UNUSED(md))
{
	return 1;
}

static bool dependsOnNormals(ModifierData *UNUSED(md))
{
	return true;
}

static void foreachIDLink(ModifierData *md, Object *ob,
						  IDWalkFunc walk, void *userData)
{
	RigidBodyModifierData *rmd = (RigidBodyModifierData *) md;

	walk(userData, ob, (ID **)&rmd->constraint_group);
}

ModifierTypeInfo modifierType_RigidBody = {
	/* name */              "RigidBody",
	/* structName */        "RigidBodyModifierData",
	/* structSize */        sizeof(RigidBodyModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
							eModifierTypeFlag_AcceptsCVs |
							/*eModifierTypeFlag_UsesPointCache |*/
							eModifierTypeFlag_Single,
	
	/* copyData */          copyData,
	/* deformVerts */       NULL,
	/* deformMatrices */    NULL,
	/* deformVertsEM */     NULL,
	/* deformMatricesEM */  NULL,
	/* applyModifier */     applyModifier,
	/* applyModifierEM */   NULL,
	/* initData */          initData,
	/* requiredDataMask */  NULL,
	/* freeData */          freeData,
	/* isDisabled */        NULL,
	/* updateDepgraph */    NULL,
	/* dependsOnTime */     dependsOnTime,
	/* dependsOnNormals */	dependsOnNormals,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     foreachIDLink,
	/* foreachTexLink */    NULL
};
