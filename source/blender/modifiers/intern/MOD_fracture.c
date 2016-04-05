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
 * The Original Code is Copyright (C) Blender Foundation
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/modifiers/intern/MOD_fracture.c
 *  \ingroup modifiers
 */

//#include "BLI_string_utf8.h"
#include "MEM_guardedalloc.h"

#include "BLI_edgehash.h"
#include "BLI_ghash.h"
#include "BLI_kdtree.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_rand.h"
#include "BLI_utildefines.h"
#include "BLI_string.h"
#include "BLI_threads.h"


#include "BKE_cdderivedmesh.h"
#include "BKE_deform.h"
#include "BKE_depsgraph.h"
#include "BKE_fracture.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_library.h"
#include "BKE_library_query.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_particle.h"
#include "BKE_pointcache.h"
#include "BKE_rigidbody.h"
#include "BKE_scene.h"
#include "BKE_mesh.h"
#include "BKE_curve.h"
#include "BKE_multires.h"

#include "bmesh.h"

#include "DNA_fracture_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_group_types.h"
#include "DNA_listBase.h"
#include "DNA_material_types.h"
#include "DNA_object_types.h"
#include "DNA_particle_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"
#include "DNA_curve_types.h"
#include "MOD_util.h"

#include "../../rigidbody/RBI_api.h"
#include "PIL_time.h"
#include "../../bmesh/tools/bmesh_decimate.h" /* decimate_dissolve function */
#include "depsgraph_private.h" /* for depgraph updates */
#include "limits.h"

static DerivedMesh* do_prefractured(FractureModifierData *fmd, Object *ob, DerivedMesh *derivedData);
static void do_prehalving(FractureModifierData *fmd, Object* ob, DerivedMesh* derivedData);
static Shard* copy_shard(Shard *s);
static void arrange_shard(FractureModifierData *fmd, ShardID id, bool do_verts, float cent[]);
static Shard* find_shard(ListBase *shards, ShardID id);

//TODO XXX Make BKE
static FracMesh* copy_fracmesh(FracMesh* fm)
{
	FracMesh *fmesh;
	Shard* s, *t;
	int i = 0;

	fmesh = MEM_mallocN(sizeof(FracMesh), __func__);
	fmesh->shard_map.first = NULL;
	fmesh->shard_map.last = NULL;

	for (s = fm->shard_map.first; s; s = s->next)
	{
		t = copy_shard(s);
		BLI_addtail(&fmesh->shard_map, t);
		i++;
	}

	fmesh->shard_count = fm->shard_count;
	fmesh->cancel = 0;
	fmesh->running = 0;
	fmesh->progress_counter = 0;
	fmesh->last_shard_tree = NULL;
	fmesh->last_shards = NULL;

	return fmesh;
}

static void initData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData *) md;

	fmd->fracture_all = false;
	fmd->cluster_count = 0;
	fmd->extra_group = NULL;
	fmd->frac_algorithm = MOD_FRACTURE_BOOLEAN;
	fmd->point_source = MOD_FRACTURE_UNIFORM;
	fmd->shard_id = -1;
	fmd->shard_count = 10;
	fmd->percentage = 100;

	fmd->visible_mesh = NULL;
	fmd->visible_mesh_cached = NULL;
	fmd->refresh = false;
	zero_m4(fmd->origmat);
	fmd->breaking_threshold = 10.0f;
	fmd->use_constraints = false;
	fmd->contact_dist = 1.0f;
	fmd->use_mass_dependent_thresholds = false;
	fmd->explo_shared = false;
	fmd->constraint_limit = 50;
	fmd->breaking_distance = 0;
	fmd->breaking_angle = 0;
	fmd->breaking_percentage = 0;     /* disable by default*/
	fmd->max_vol = 0;
	fmd->refresh_constraints = false;

	fmd->cluster_breaking_threshold = 1000.0f;
	fmd->solver_iterations_override = 0;
	fmd->cluster_solver_iterations_override = 0;
	fmd->shards_to_islands = false;
	fmd->execute_threaded = false;
	fmd->nor_tree = NULL;
	fmd->fix_normals = false;
	fmd->auto_execute = false;
	fmd->face_pairs = NULL;
	fmd->autohide_dist = 0.0f;
	fmd->automerge_dist = 0.0f;

	fmd->breaking_percentage_weighted = false;
	fmd->breaking_angle_weighted = false;
	fmd->breaking_distance_weighted = false;

	/* XXX needed because of messy particle cache, shows incorrect positions when start/end on frame 1
	 * default use case is with this flag being enabled, disable at own risk */
	fmd->use_particle_birth_coordinates = true;
	fmd->splinter_length = 1.0f;
	fmd->nor_range = 1.0f;

	fmd->cluster_breaking_angle = 0;
	fmd->cluster_breaking_distance = 0;
	fmd->cluster_breaking_percentage = 0;

	/* used for advanced fracture settings now, XXX needs rename perhaps*/
	fmd->use_experimental = 0;
	fmd->use_breaking = true;
	fmd->use_smooth = false;

	fmd->fractal_cuts = 1;
	fmd->fractal_amount = 1.0f;
	fmd->physics_mesh_scale = 0.75f;
	fmd->fractal_iterations = 5;

	fmd->cluster_group = NULL;
	fmd->cutter_group = NULL;

	fmd->grease_decimate = 100.0f;
	fmd->grease_offset = 0.5f;
	fmd->use_greasepencil_edges = true;

	fmd->cutter_axis = MOD_FRACTURE_CUTTER_Z;
	fmd->cluster_constraint_type = RBC_TYPE_FIXED;
	fmd->vert_index_map = NULL;
	fmd->constraint_target = MOD_FRACTURE_CENTROID;
	fmd->vertex_island_map = NULL;

	fmd->meshIslands.first = NULL;
	fmd->meshIslands.last = NULL;
	fmd->meshConstraints.first = NULL;
	fmd->meshConstraints.last = NULL;

	fmd->fracture_mode = MOD_FRACTURE_PREFRACTURED;
	fmd->last_frame = FLT_MIN;
	fmd->dynamic_force = 10.0f;
	fmd->update_dynamic = false;
	fmd->limit_impact = false;
	fmd->reset_shards = false;
	fmd->active_setting = -1;

	fmd->use_compounds = false;
	fmd->impulse_dampening = 0.05f;
	fmd->directional_factor = 0.0f;
	fmd->minimum_impulse = 0.1f;
	fmd->mass_threshold_factor = 0.0f;

	fmd->autohide_filter_group = NULL;
	fmd->constraint_count = 0;
}

//XXX TODO, freeing functionality should be in BKE too
static void free_meshislands(FractureModifierData* fmd, ListBase* meshIslands, bool do_free_rigidbody)
{
	MeshIsland *mi;

	while (meshIslands->first) {
		mi = meshIslands->first;
		BLI_remlink_safe(meshIslands, mi);
		BKE_fracture_free_mesh_island(fmd, mi, do_free_rigidbody);
		mi = NULL;
	}

	meshIslands->first = NULL;
	meshIslands->last = NULL;

	//if (fmd->modifier.scene && fmd->modifier.scene->rigidbody_world)
	//	BKE_rigidbody_update_ob_array(fmd->modifier.scene->rigidbody_world);
}

static void free_simulation(FractureModifierData *fmd, bool do_free_seq, bool do_free_rigidbody)
{
	/* what happens with this in dynamic fracture ? worst case, we need a sequence for this too*/
	if (fmd->shards_to_islands) {
		while (fmd->islandShards.first) {
			Shard *s = fmd->islandShards.first;
			BLI_remlink(&fmd->islandShards, s);
			BKE_shard_free(s, true);
			s = NULL;
		}

		fmd->islandShards.first = NULL;
		fmd->islandShards.last = NULL;
	}

	/* when freeing meshislands, we MUST get rid of constraints before too !!!! */
	BKE_free_constraints(fmd);

	if (!do_free_seq) {

		if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			//delete all except first sequence elem (really ?)
			MeshIslandSequence *msq = fmd->meshIsland_sequence.first;
			ShardSequence *ssq = fmd->shard_sequence.first;

			if (msq)
				msq = msq->next;

			while (msq && msq != fmd->meshIsland_sequence.first)
			{
				BLI_remlink(&fmd->meshIsland_sequence, msq);
				free_meshislands(fmd, &msq->meshIslands, do_free_rigidbody);
				MEM_freeN(msq);
				msq = fmd->meshIsland_sequence.first;
				if (msq)
					msq = msq->next;
			}

			msq = fmd->meshIsland_sequence.first;
			if (msq)
			{
				msq->is_new = true;
				fmd->current_mi_entry = msq;
				fmd->meshIslands = msq->meshIslands;
				fmd->visible_mesh_cached = msq->visible_dm;

			}
			else
			{
				fmd->meshIsland_sequence.first = NULL;
				fmd->meshIsland_sequence.last = NULL;

				fmd->meshIslands.first = NULL;
				fmd->meshIslands.last = NULL;

				fmd->current_mi_entry = NULL;
			}

			if (ssq)
				ssq = ssq->next;

			while (ssq && ssq != fmd->shard_sequence.first)
			{
				BLI_remlink(&fmd->shard_sequence, ssq);
				BKE_fracmesh_free(ssq->frac_mesh, true);
				MEM_freeN(ssq->frac_mesh);
				MEM_freeN(ssq);

				ssq = fmd->shard_sequence.first;
				if (ssq)
					ssq = ssq->next;
			}

			if (ssq)
			{
				fmd->current_shard_entry = ssq;
				fmd->frac_mesh = ssq->frac_mesh;
			}
			else
			{
				fmd->shard_sequence.first = NULL;
				fmd->shard_sequence.last = NULL;
				fmd->current_shard_entry = NULL;
				fmd->frac_mesh = NULL;
			}
		}
		else
		{
			free_meshislands(fmd, &fmd->meshIslands, do_free_rigidbody);
			fmd->meshIslands.first = NULL;
			fmd->meshIslands.last = NULL;
		}
	}
	else
	{	
		if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			/* in dynamic mode we have to get rid of the entire Meshisland sequence */
			MeshIslandSequence *msq;
			ShardSequence *ssq;

			while (fmd->meshIsland_sequence.first) {
				msq = fmd->meshIsland_sequence.first;
				BLI_remlink(&fmd->meshIsland_sequence, msq);
				free_meshislands(fmd, &msq->meshIslands, do_free_rigidbody);
				MEM_freeN(msq);
				msq = NULL;
			}

			fmd->meshIsland_sequence.first = NULL;
			fmd->meshIsland_sequence.last = NULL;

			fmd->meshIslands.first = NULL;
			fmd->meshIslands.last = NULL;

			fmd->current_mi_entry = NULL;

			while (fmd->shard_sequence.first)
			{
				ssq = fmd->shard_sequence.first;
				BLI_remlink(&fmd->shard_sequence, ssq);
				BKE_fracmesh_free(ssq->frac_mesh, true);
				MEM_freeN(ssq->frac_mesh);
				MEM_freeN(ssq);
			}

			fmd->shard_sequence.first = NULL;
			fmd->shard_sequence.last = NULL;
			fmd->current_shard_entry = NULL;
			fmd->frac_mesh = NULL;
		}
	}

	if (!fmd->explo_shared && fmd->visible_mesh != NULL) {
		BM_mesh_free(fmd->visible_mesh);
		fmd->visible_mesh = NULL;
	}
}

static void free_shards(FractureModifierData *fmd)
{
	if (fmd->frac_mesh) {

		if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED ||
		    fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
		{
			BKE_fracmesh_free(fmd->frac_mesh, true);
			MEM_freeN(fmd->frac_mesh);
			fmd->frac_mesh = NULL;
		}
		else
		{
			/* free entire shard sequence here */
			while(fmd->shard_sequence.first)
			{
				ShardSequence* ssq = (ShardSequence*)fmd->shard_sequence.first;
				BLI_remlink(&fmd->shard_sequence, ssq);
				BKE_fracmesh_free(ssq->frac_mesh, true);
				MEM_freeN(ssq->frac_mesh);
				MEM_freeN(ssq);
			}
			fmd->frac_mesh = NULL;
			fmd->shard_sequence.first = NULL;
			fmd->shard_sequence.last = NULL;

			fmd->current_shard_entry = NULL;
		}
	}
}

static void free_modifier(FractureModifierData *fmd, bool do_free_seq, bool do_free_rigidbody)
{
	free_simulation(fmd, do_free_seq, (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC) && do_free_rigidbody);

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

	if (fmd->vertex_island_map) {
		BLI_ghash_free(fmd->vertex_island_map, NULL, NULL);
		fmd->vertex_island_map = NULL;
	}

	if (fmd->nor_tree != NULL) {
		BLI_kdtree_free(fmd->nor_tree);
		fmd->nor_tree = NULL;
	}

	if (fmd->face_pairs != NULL) {
		BLI_ghash_free(fmd->face_pairs, NULL, NULL);
		fmd->face_pairs = NULL;
	}

	//called on deleting modifier, object or quitting blender...
	//why was this necessary again ?!
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}

	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED || fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		if (fmd->visible_mesh_cached) {
			fmd->visible_mesh_cached->needsFree = 1;
			fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
			fmd->visible_mesh_cached = NULL;
		}

		if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
			free_shards(fmd);
	}

	if (fmd->vert_index_map != NULL) {
		BLI_ghash_free(fmd->vert_index_map, NULL, NULL);
		fmd->vert_index_map = NULL;
	}

	/*needs to be freed in any case here ?*/
	if (fmd->visible_mesh != NULL) {
		BM_mesh_free(fmd->visible_mesh);
		fmd->visible_mesh = NULL;
	}

	while (fmd->fracture_settings.first) {
		FractureSetting *fs = fmd->fracture_settings.first;
		BLI_remlink(&fmd->fracture_settings, fs);
		MEM_freeN(fs);
	}
}

static void freeData_internal(FractureModifierData *fmd, bool do_free_seq, bool do_free_rigidbody)
{
	if ((!fmd->refresh && !fmd->refresh_constraints) || (fmd->frac_mesh && fmd->frac_mesh->cancel == 1)) {
		/* free entire modifier or when job has been cancelled */
		free_modifier(fmd, do_free_seq, do_free_rigidbody);

		if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED || fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
		{
			if (fmd->visible_mesh_cached && !fmd->shards_to_islands)
			{
				/* free visible_mesh_cached in any case ?!*/
				fmd->visible_mesh_cached->needsFree = 1;
				fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
				fmd->visible_mesh_cached = NULL;
			}
		}
	}
	else if (!fmd->refresh_constraints) {
		/* refreshing all simulation data only, no refracture */
		if (fmd->fracture_mode != MOD_FRACTURE_DYNAMIC)
			free_simulation(fmd, false, do_free_rigidbody); // in this case keep the meshisland sequence!
	}
	else if (fmd->refresh_constraints) {
		/* refresh constraints only */
		BKE_free_constraints(fmd);
	}
}

static void freeData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData *) md;

	freeData_internal(fmd, true, false);

	/*force deletion of meshshards here, it slips through improper state detection*/
	/*here we know the modifier is about to be deleted completely*/
	free_shards(fmd);
}

//XXX TODO move cluster handling to BKE too
static void do_cluster_count(FractureModifierData *fmd)
{
	int k = 0;
	KDTree *tree;
	MeshIsland *mi, **seeds;
	int seed_count;

	int mi_count;
	/* zero clusters or one mean no clusters, all shards keep free */
	if (fmd->cluster_count < 2) {
		return;
	}

	/*initialize cluster "colors" -> membership of meshislands to clusters, initally all shards are "free" */
	for (mi = fmd->meshIslands.first; mi; mi = mi->next ) {
		mi->particle_index = -1;
	}

	mi_count = BLI_listbase_count(&fmd->meshIslands);
	seed_count = (fmd->cluster_count > mi_count ? mi_count : fmd->cluster_count);
	seeds = MEM_mallocN(sizeof(MeshIsland *) * seed_count, "seeds");
	tree = BLI_kdtree_new(seed_count);

	/* pick n seed locations, randomly scattered over the object */
	for (k = 0; k < seed_count; k++) {
		int which_index = k * (int)(mi_count / seed_count);
		MeshIsland *which = (MeshIsland *)BLI_findlink(&fmd->meshIslands, which_index);
		which->particle_index = k;
		BLI_kdtree_insert(tree, k, which->centroid);
		seeds[k] = which;
	}

	BLI_kdtree_balance(tree);


	/* assign each shard to its closest center */
	for (mi = fmd->meshIslands.first; mi; mi = mi->next ) {
		KDTreeNearest n;
		int index;

		index = BLI_kdtree_find_nearest(tree, mi->centroid, &n);
		mi->particle_index = seeds[index]->particle_index;
	}

	BLI_kdtree_free(tree);
	MEM_freeN(seeds);
}

static void do_cluster_group(FractureModifierData *fmd, Object* obj)
{
	KDTree *tree;
	MeshIsland *mi;
	int seed_count;

	/*initialize cluster "colors" -> membership of meshislands to clusters, initally all shards are "free" */
	for (mi = fmd->meshIslands.first; mi; mi = mi->next ) {
		mi->particle_index = -1;
	}

	seed_count = BLI_listbase_count(&fmd->cluster_group->gobject);
	if (seed_count > 0)
	{
		GroupObject* go;
		int i = 0;
		tree = BLI_kdtree_new(seed_count);
		for (i = 0, go = fmd->cluster_group->gobject.first; go; i++, go = go->next)
		{
			BLI_kdtree_insert(tree, i, go->ob->loc);
		}

		BLI_kdtree_balance(tree);

		/* assign each shard to its closest center */
		for (mi = fmd->meshIslands.first; mi; mi = mi->next ) {
			KDTreeNearest n;
			int index;
			float co[3];

			mul_v3_m4v3(co, obj->obmat, mi->centroid);

			index = BLI_kdtree_find_nearest(tree, co, &n);
			mi->particle_index = index;
		}

		BLI_kdtree_free(tree);
	}
}

static void do_clusters(FractureModifierData *fmd, Object* obj)
{
	/*grow clusters from all meshIslands */
	if (fmd->cluster_group)
	{
		do_cluster_group(fmd, obj);
	}
	else
	{
		do_cluster_count(fmd);
	}
}

//XXXX TODO same applies for autohide prep and normals fixing, latter could be a separate operator or so, called from refresh op
static KDTree *build_nor_tree(DerivedMesh *dm)
{
	int i = 0, totvert = dm->getNumVerts(dm);
	KDTree *tree = BLI_kdtree_new(totvert);
	MVert *mv, *mvert = dm->getVertArray(dm);

	for (i = 0, mv = mvert; i < totvert; i++, mv++) {
		BLI_kdtree_insert(tree, i, mv->co);
	}

	BLI_kdtree_balance(tree);

	return tree;
}

static void find_normal(DerivedMesh *dm, KDTree *tree, float co[3], short no[3], short rno[3], float range)
{
	KDTreeNearest *n = NULL, n2;
	int index = 0, i = 0, count = 0;
	MVert mvert;
	float fno[3], vno[3];

	normal_short_to_float_v3(fno, no);

	count = BLI_kdtree_range_search(tree, co, &n, range);
	for (i = 0; i < count; i++)
	{
		index = n[i].index;
		dm->getVert(dm, index, &mvert);
		normal_short_to_float_v3(vno, mvert.no);
		if ((dot_v3v3(fno, vno) > 0.0f)){
			copy_v3_v3_short(rno, mvert.no);
			if (n != NULL) {
				MEM_freeN(n);
				n = NULL;
			}
			return;
		}
	}

	if (n != NULL) {
		MEM_freeN(n);
		n = NULL;
	}

	/*fallback if no valid normal in searchrange....*/
	BLI_kdtree_find_nearest(tree, co, &n2);
	index = n2.index;
	dm->getVert(dm, index, &mvert);
	copy_v3_v3_short(rno, mvert.no);
}

//Move to BKE too, TODO XXXX
static DerivedMesh *get_clean_dm(Object *ob, DerivedMesh *dm)
{
	/* may have messed up meshes from conversion */
	if (ob->type == OB_FONT || ob->type == OB_CURVE || ob->type == OB_SURF) {
		DerivedMesh *result = NULL;

		/* convert to BMesh, remove doubles, limited dissolve and convert back */
		BMesh *bm = DM_to_bmesh(dm, true);

		BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		             "remove_doubles verts=%av dist=%f", BM_VERTS_OF_MESH, 0.0001, false);

		BM_mesh_decimate_dissolve(bm, 0.087f, false, 0);
		result = CDDM_from_bmesh(bm, true);
		BM_mesh_free(bm);

		return result;
	}

	return dm;
}

//XXX TODO plan is to get rid of this, since we have a packing mechanism now, but wrap that functionality to useful op (via C, python api is optional)
static int getGroupObjects(Group *gr, Object ***obs, int g_exist)
{
	int ctr = g_exist;
	GroupObject *go;
	if (gr == NULL) return ctr;

	for (go = gr->gobject.first; go; go = go->next) {

		*obs = MEM_reallocN(*obs, sizeof(Object *) * (ctr + 1));
		(*obs)[ctr] = go->ob;
		ctr++;
	}

	return ctr;
}

static DerivedMesh* get_object_dm(Object* o)
{
	DerivedMesh *dm_ob = NULL;

	/*ensure o->derivedFinal*/
	FractureModifierData* fmd2 = (FractureModifierData*) modifiers_findByType(o, eModifierType_Fracture);
	if (fmd2)
	{
		dm_ob = fmd2->visible_mesh_cached;
	}
	else
	{
		dm_ob = o->derivedFinal;
	}

	return dm_ob;
}

static void adjustPolys(MPoly **mpoly, DerivedMesh *dm_ob, GHash *mat_index_map, short matstart, int loopstart, int polystart, DerivedMesh* result)
{
	MPoly *mp;
	int j;

	for (j = 0, mp = *mpoly; j < dm_ob->getNumPolys(dm_ob); ++j, ++mp) {
		short index = 0;
		/* adjust loopstart index */
		if (CustomData_has_layer(&dm_ob->polyData, CD_MTEXPOLY))
		{
			MTexPoly *mtp = CustomData_get(&dm_ob->polyData, j, CD_MTEXPOLY);
			if (mtp)
				CustomData_set(&result->polyData, polystart + j, CD_MTEXPOLY, mtp);
		}
		mp->loopstart += loopstart;

		/* material index lookup and correction, avoid having the same material in different slots */
		index = GET_INT_FROM_POINTER(BLI_ghash_lookup(mat_index_map, SET_INT_IN_POINTER(mp->mat_nr + matstart)));
		mp->mat_nr = index-1;
	}
}

static void adjustLoops(MLoop **mloop, DerivedMesh *dm_ob, int vertstart, int loopstart, DerivedMesh *result)
{
	MLoop *ml;
	int j;

	for (j = 0, ml = *mloop; j < dm_ob->getNumLoops(dm_ob); ++j, ++ml) {
		/* adjust vertex index */
		if (CustomData_has_layer(&dm_ob->loopData, CD_MLOOPUV))
		{
			MLoopUV *mluv = CustomData_get(&dm_ob->loopData, j, CD_MLOOPUV);
			if (mluv)
				CustomData_set(&result->loopData, loopstart + j, CD_MLOOPUV, mluv);
		}
		ml->v += vertstart;
	}
}

static void adjustVerts(MVert **mvert, FractureModifierData *fmd, Object *o, DerivedMesh* dm_ob, int vertstart, int i, DerivedMesh* result)
{
	MVert *mv;
	int v;

	for (v = 0, mv = *mvert; v < dm_ob->getNumVerts(dm_ob); v++, mv++)
	{
		if (CustomData_has_layer(&dm_ob->vertData, CD_MDEFORMVERT))
		{
			MDeformVert *mdv = CustomData_get(&dm_ob->vertData, v, CD_MDEFORMVERT);
			if (mdv)
				CustomData_set(&result->vertData, vertstart + v, CD_MDEFORMVERT, mdv);
		}
		mul_m4_v3(o->obmat, mv->co);
		BLI_ghash_insert(fmd->vert_index_map, SET_INT_IN_POINTER(vertstart + v), SET_INT_IN_POINTER(i));
	}
}

static void collect_derivedmeshes(FractureModifierData* fmd, Object *ob, MVert** mvert, MLoop** mloop, MPoly **mpoly, DerivedMesh* result, GHash** mat_index_map)
{
	int vertstart = 0, polystart = 0, loopstart = 0;
	short matstart = 1;
	MVert *mverts = *mvert;
	MLoop *mloops = *mloop;
	MPoly *mpolys = *mpoly;

	MVert *mv;
	MLoop *ml;
	MPoly *mp;

	GroupObject* go;
	int totcol;
	int i = 0;

	for (go = fmd->dm_group->gobject.first; go; go = go->next)
	{
		DerivedMesh* dm_ob = NULL;
		Object *o = go->ob;

		dm_ob = get_object_dm(o);
		if (dm_ob == NULL)
		{   /* avoid crash atleast...*/
			return;
		}

		totcol = BKE_fracture_collect_materials(o, ob, matstart, mat_index_map);

		mv = mverts + vertstart;
		memcpy(mv, dm_ob->getVertArray(dm_ob), dm_ob->getNumVerts(dm_ob) * sizeof(MVert));
		adjustVerts(&mv, fmd, o, dm_ob, vertstart, i, result);

		mp = mpolys + polystart;
		memcpy(mp, dm_ob->getPolyArray(dm_ob), dm_ob->getNumPolys(dm_ob) * sizeof(MPoly));
		adjustPolys(&mp, dm_ob, *mat_index_map, matstart, loopstart, polystart, result);

		ml = mloops + loopstart;
		memcpy(ml, dm_ob->getLoopArray(dm_ob), dm_ob->getNumLoops(dm_ob) * sizeof(MLoop));
		adjustLoops(&ml, dm_ob, vertstart, loopstart, result);

		vertstart += dm_ob->getNumVerts(dm_ob);
		polystart += dm_ob->getNumPolys(dm_ob);
		loopstart += dm_ob->getNumLoops(dm_ob);
		matstart += totcol;
		i++;
	}
}

static void count_dm_contents(FractureModifierData *fmd, int *num_verts, int *num_loops, int *num_polys)
{
	GroupObject* go;

	for (go = fmd->dm_group->gobject.first; go; go = go->next)
	{
		DerivedMesh* dm_ob = NULL;
		Object *o = go->ob;

		/*ensure o->derivedFinal*/
		FractureModifierData* fmd2 = (FractureModifierData*) modifiers_findByType(o, eModifierType_Fracture);
		if (fmd2)
		{
			dm_ob = fmd2->visible_mesh_cached;
		}
		else
		{
			dm_ob = o->derivedFinal;
		}

		if (dm_ob == NULL) continue;

		(*num_verts) += dm_ob->getNumVerts(dm_ob);
		(*num_polys) += dm_ob->getNumPolys(dm_ob);
		(*num_loops) += dm_ob->getNumLoops(dm_ob);
	}
}


static DerivedMesh *get_group_dm(FractureModifierData *fmd, DerivedMesh *dm, Object* ob)
{
	/* combine derived meshes from group objects into 1, trigger submodifiers if ob->derivedFinal is empty */
	int num_verts = 0, num_polys = 0, num_loops = 0;
	DerivedMesh *result;
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;

	GHash *mat_index_map = NULL;

	if (fmd->dm_group && (fmd->refresh == true || fmd->auto_execute == true))
	{
		mat_index_map = BLI_ghash_int_new("mat_index_map");
		if (fmd->vert_index_map != NULL) {
			BLI_ghash_free(fmd->vert_index_map, NULL, NULL);
			fmd->vert_index_map = NULL;
		}

		fmd->vert_index_map = BLI_ghash_int_new("vert_index_map");

		count_dm_contents(fmd, &num_verts, &num_loops, &num_polys);
		if (num_verts == 0)
		{
			return dm;
		}

		result = CDDM_new(num_verts, 0, 0, num_loops, num_polys);
		mverts = CDDM_get_verts(result);
		mloops = CDDM_get_loops(result);
		mpolys = CDDM_get_polys(result);

		CustomData_add_layer(&result->vertData, CD_MDEFORMVERT, CD_CALLOC, NULL, num_verts);
		CustomData_add_layer(&result->loopData, CD_MLOOPUV, CD_CALLOC, NULL, num_loops);
		CustomData_add_layer(&result->polyData, CD_MTEXPOLY, CD_CALLOC, NULL, num_polys);

		collect_derivedmeshes(fmd, ob, &mverts, &mloops, &mpolys, result, &mat_index_map);
		CDDM_calc_edges(result);

		result->dirty |= DM_DIRTY_NORMALS;
		CDDM_calc_normals_mapping(result);

		BLI_ghash_free(mat_index_map, NULL, NULL);
		mat_index_map = NULL;
		return result;
	}

	return dm;
}

static bool in_bbox(float p[3], float min[3], float max[3])
{
	return (p[0] > min[0]) && (p[0] < max[0]) && (p[1] > min[1]) && (p[1] < max[1]) && (p[2] > min[2]) && (p[2] < max[2]);
}

//XXX MOve to BKE_Fracture.h / fracture.c, prefracture stuff should be a function called from op, or in dynamic case from Rigidbody system callback
static void points_from_verts(Object **ob, int totobj, FracPointCloud *points, float mat[4][4], float thresh, FractureModifierData *emd, DerivedMesh *dm, Object *obj,
                              ShardID id)
{
	int v, o, pt = points->totpoints;
	float co[3];

	for (o = 0; o < totobj; o++) {
		if (ob[o]->type == OB_MESH) {
			/* works for mesh objects only, curves, surfaces, texts have no verts */
			float imat[4][4];
			DerivedMesh *d;
			MVert *vert;

			if (ob[o] == obj) {
				/* same object, use given derivedmesh */
				d = dm;
			}
			else {
				d = mesh_get_derived_final(emd->modifier.scene, ob[o], 0);
			}

			invert_m4_m4(imat, mat);
			vert = d->getVertArray(d);

			for (v = 0; v < d->getNumVerts(d); v++) {
				if (BLI_frand() < thresh) {

					copy_v3_v3(co, vert[v].co);

					//XXXX TODO, own verts transform seems to have a bug here as well
					if (emd->point_source & MOD_FRACTURE_EXTRA_VERTS) {
						mul_m4_v3(ob[o]->obmat, co);
						mul_m4_v3(imat, co);
					}

					if (id > 0)
					{
						Shard *sh;
						float min[3], max[3], cent[3];
						arrange_shard(emd, id, false, cent);
						sh = find_shard(&emd->frac_mesh->shard_map, id);
						if (sh)
						{
							add_v3_v3v3(min, sh->min, cent);
							add_v3_v3v3(max, sh->max, cent);
							if (in_bbox(co, min, max))
							{
								points->points = MEM_reallocN(points->points, (pt + 1) * sizeof(FracPoint));
								copy_v3_v3(points->points[pt].co, co);
								pt++;
							}
						}
					}
					else
					{
						points->points = MEM_reallocN(points->points, (pt + 1) * sizeof(FracPoint));
						copy_v3_v3(points->points[pt].co, co);
						pt++;
					}
				}
			}
		}
	}

	points->totpoints = pt;
}

static void points_from_particles(Object **ob, int totobj, Scene *scene, FracPointCloud *points, float mat[4][4],
                                  float thresh, FractureModifierData *fmd, ShardID id)
{
	int o, p, pt = points->totpoints;
	ParticleSystemModifierData *psmd;
	ParticleData *pa;
	ParticleSimulationData sim = {NULL};
	ParticleKey birth;
	ModifierData *mod;

	for (o = 0; o < totobj; o++) {
		for (mod = ob[o]->modifiers.first; mod; mod = mod->next) {
			if (mod->type == eModifierType_ParticleSystem) {
				float imat[4][4];
				psmd = (ParticleSystemModifierData *)mod;
				sim.scene = scene;
				sim.ob = ob[o];
				sim.psys = psmd->psys;
				sim.psmd = psmd;
				invert_m4_m4(imat, mat);

				for (p = 0, pa = psmd->psys->particles; p < psmd->psys->totpart; p++, pa++) {
					/* XXX was previously there to choose a particle with a certain state */
					bool particle_unborn = pa->alive == PARS_UNBORN;
					bool particle_alive = pa->alive == PARS_ALIVE;
					bool particle_dead = pa->alive == PARS_DEAD;
					bool particle_mask = particle_unborn || particle_alive || particle_dead;

					if ((BLI_frand() < thresh) && particle_mask) {
						float co[3];

						/* birth coordinates are not sufficient in case we did pre-simulate the particles, so they are not
						 * aligned with the emitter any more BUT as the particle cache is messy and shows initially wrong
						 * positions "sabotaging" fracture, default use case is using birth coordinates, let user decide... */
						if (fmd->use_particle_birth_coordinates && fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
						{
							psys_get_birth_coords(&sim, pa, &birth, 0, 0);
						}
						else {
							psys_get_particle_state(&sim, p, &birth, 1);
						}

						copy_v3_v3(co, birth.co);
						mul_m4_v3(imat, co);

						if (id > 0)
						{
							Shard *sh;
							float min[3], max[3], cent[3];
							arrange_shard(fmd, id, false, cent);
							sh = find_shard(&fmd->frac_mesh->shard_map, id);
							if (sh)
							{
								add_v3_v3v3(min, sh->min, cent);
								add_v3_v3v3(max, sh->max, cent);
								if (in_bbox(co, min, max))
								{
									points->points = MEM_reallocN(points->points, (pt + 1) * sizeof(FracPoint));
									copy_v3_v3(points->points[pt].co, co);
									pt++;
								}
							}
						}
						else
						{
							points->points = MEM_reallocN(points->points, (pt + 1) * sizeof(FracPoint));
							copy_v3_v3(points->points[pt].co, co);
							pt++;
						}
					}
				}
			}
		}
	}

	points->totpoints = pt;
}

//XXX TODO maybe remove, hardly used
static void points_from_greasepencil(Object **ob, int totobj, FracPointCloud *points, float mat[4][4], float thresh)
{
	bGPDlayer *gpl;
	bGPDframe *gpf;
	bGPDstroke *gps;
	int pt = points->totpoints, p, o;

	for (o = 0; o < totobj; o++) {
		if ((ob[o]->gpd) && (ob[o]->gpd->layers.first)) {
			float imat[4][4];
			invert_m4_m4(imat, mat);
			for (gpl = ob[o]->gpd->layers.first; gpl; gpl = gpl->next) {
				for (gpf = gpl->frames.first; gpf; gpf = gpf->next) {
					for (gps = gpf->strokes.first; gps; gps = gps->next) {
						for (p = 0; p < gps->totpoints; p++) {
							if (BLI_frand() < thresh) {
								float point[3] = {0, 0, 0};
								points->points = MEM_reallocN(points->points, (pt + 1) * sizeof(FracPoint));

								point[0] = gps->points[p].x;
								point[1] = gps->points[p].y;
								point[2] = gps->points[p].z;

								mul_m4_v3(imat, point);

								copy_v3_v3(points->points[pt].co, point);
								pt++;
							}
						}
					}
				}
			}
		}
	}

	points->totpoints = pt;
}

static FracPointCloud get_points_global(FractureModifierData *emd, Object *ob, DerivedMesh *fracmesh, ShardID id)
{
	Scene *scene = emd->modifier.scene;
	FracPointCloud points;

	/* global settings, for first fracture only, or global secondary and so on fracture, apply to entire fracmesh */
	int totgroup = 0;
	Object **go = MEM_mallocN(sizeof(Object *), "groupobjects");
	float thresh = (float)emd->percentage / 100.0f;
	float min[3], max[3];
	int i;

	points.points = MEM_mallocN(sizeof(FracPoint), "points");
	points.totpoints = 0;

	if (emd->point_source & (MOD_FRACTURE_EXTRA_PARTICLES | MOD_FRACTURE_EXTRA_VERTS)) {
		if (((emd->point_source & MOD_FRACTURE_OWN_PARTICLES) && (emd->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
		    ((emd->point_source & MOD_FRACTURE_OWN_VERTS) && (emd->point_source & MOD_FRACTURE_EXTRA_VERTS)) ||
		    ((emd->point_source & MOD_FRACTURE_GREASEPENCIL) && (emd->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
		    ((emd->point_source & MOD_FRACTURE_GREASEPENCIL) && (emd->point_source & MOD_FRACTURE_EXTRA_VERTS)))
		{
			go = MEM_reallocN(go, sizeof(Object *) * (totgroup + 1));
			go[totgroup] = ob;
			totgroup++;
		}

		totgroup = getGroupObjects(emd->extra_group, &go, totgroup);
	}
	else {
		totgroup = 1;
		go[0] = ob;
	}

	if (emd->point_source & (MOD_FRACTURE_OWN_PARTICLES | MOD_FRACTURE_EXTRA_PARTICLES)) {
		points_from_particles(go, totgroup, scene, &points, ob->obmat, thresh, emd, id);
	}

	if (emd->point_source & (MOD_FRACTURE_OWN_VERTS | MOD_FRACTURE_EXTRA_VERTS)) {
		points_from_verts(go, totgroup, &points, ob->obmat, thresh, emd, fracmesh, ob, id);
	}

	if (emd->point_source & MOD_FRACTURE_GREASEPENCIL && !emd->use_greasepencil_edges) {
		points_from_greasepencil(go, totgroup, &points, ob->obmat, thresh);
	}


	/* local settings, apply per shard!!! Or globally too first. */
	if (emd->point_source & MOD_FRACTURE_UNIFORM)
	{
		float cent[3], bmin[3], bmax[3];
		int count = emd->shard_count;
		INIT_MINMAX(min, max);
		if (!BKE_get_shard_minmax(emd->frac_mesh, id, min, max, fracmesh))
			return points; //id 0 should be entire mesh

		arrange_shard(emd, id, false, cent);
		add_v3_v3v3(bmax, max, cent);
		add_v3_v3v3(bmin, min, cent);

		printf("min, max: (%f %f %f), (%f %f %f)\n", min[0], min[1], min[2], max[0], max[1], max[2]);

		if (emd->frac_algorithm == MOD_FRACTURE_BISECT_FAST || emd->frac_algorithm == MOD_FRACTURE_BISECT_FAST_FILL ||
		    emd->frac_algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
			/* XXX need double amount of shards, because we create 2 islands at each cut... so this matches the input count */
			if ((count > 1) || emd->frac_algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
				count--;
				count *= 2;
			}
		}

		BLI_srandom(emd->point_seed);
		for (i = 0; i < count; ++i) {
			if (BLI_frand() < thresh) {
				float co[3];
				co[0] = min[0] + (max[0] - min[0]) * BLI_frand();
				co[1] = min[1] + (max[1] - min[1]) * BLI_frand();
				co[2] = min[2] + (max[2] - min[2]) * BLI_frand();

				if (id > 0)
				{
					if (in_bbox(co, bmin, bmax))
					{
						points.points = MEM_reallocN(points.points, sizeof(FracPoint) * (points.totpoints + 1));
						copy_v3_v3(points.points[points.totpoints].co, co);
						points.totpoints++;
					}
				}
				else
				{
					points.points = MEM_reallocN(points.points, sizeof(FracPoint) * (points.totpoints + 1));
					copy_v3_v3(points.points[points.totpoints].co, co);
					points.totpoints++;
				}
			}
		}
	}

	MEM_freeN(go);
	return points;
}

static Material* find_material(const char* name)
{
	ID* mat;

	for (mat = G.main->mat.first; mat; mat = mat->next)
	{
		char *cmp = BLI_strdupcat("MA", name);
		if (strcmp(cmp, mat->name) == 0)
		{
			MEM_freeN(cmp);
			cmp = NULL;
			return (Material*)mat;
		}
		else
		{
			MEM_freeN(cmp);
			cmp = NULL;
		}
	}

	return BKE_material_add(G.main, name);
}

//splinter handling is a case for BKE too
static Shard* do_splinters(FractureModifierData *fmd, FracPointCloud points, float(*mat)[4][4], ShardID id, DerivedMesh *dm)
{
	float imat[4][4];

	/*need to add island / shard centroid...*/
	Shard *s = BKE_shard_by_id(fmd->frac_mesh, id, NULL);

	unit_m4(*mat);

	/*splinters... just global axises and a length, for rotation rotate the object */
	if (fmd->splinter_axis & MOD_FRACTURE_SPLINTER_X)
	{
		(*mat)[0][0] *= fmd->splinter_length;
	}
	if (fmd->splinter_axis & MOD_FRACTURE_SPLINTER_Y)
	{
		(*mat)[1][1] *= fmd->splinter_length;
	}
	if (fmd->splinter_axis & MOD_FRACTURE_SPLINTER_Z)
	{
		(*mat)[2][2] *= fmd->splinter_length;
	}

	if ((fmd->splinter_axis & MOD_FRACTURE_SPLINTER_X) ||
		(fmd->splinter_axis & MOD_FRACTURE_SPLINTER_Y) ||
		(fmd->splinter_axis & MOD_FRACTURE_SPLINTER_Z))
	{
		int i = 0, num_verts = 0;
		MVert* mvert = NULL, *mv;


		if (s) {
			mvert = s->mvert;
			num_verts = s->totvert;
			/* copy location to matrix */
			/*(*mat)[3][0] = s->centroid[0];
			(*mat)[3][1] = s->centroid[1];
			(*mat)[3][2] = s->centroid[2];*/
		}
		else
		{
			mvert = dm->getVertArray(dm);
			num_verts = dm->getNumVerts(dm);
		}

		invert_m4_m4(imat, *mat);

		for (i = 0; i < points.totpoints; i++)
		{
			mul_m4_v3(imat, points.points[i].co);
		}

		for (i = 0, mv = mvert; i < num_verts; i++, mv++)
		{
			mul_m4_v3(imat, mv->co);
		}
	}

	return s;
}

//so is material handling too, XXX TODO move to BKE
static short do_materials(FractureModifierData *fmd, Object* obj)
{
	short mat_index = 0;

	if (fmd->inner_material) {
		/* assign inner material as secondary mat to ob if not there already */
		mat_index = find_material_index(obj, fmd->inner_material);
		if (mat_index == 0) {
			object_add_material_slot(obj);
			assign_material(obj, fmd->inner_material, obj->totcol, BKE_MAT_ASSIGN_OBDATA);
		}

		/* get index again */
		mat_index = find_material_index(obj, fmd->inner_material);
	}
	else
	{
		/* autogenerate materials */
		char name[MAX_ID_NAME];

		short* totmat = give_totcolp(obj);

		BLI_strncpy(name, obj->id.name + 2, strlen(obj->id.name));
		if (*totmat == 0)
		{
			/*create both materials*/
			Material* mat_inner;
			char *matname = BLI_strdupcat(name, "_Outer");
			Material* mat_outer = find_material(matname);
			object_add_material_slot(obj);
			assign_material(obj, mat_outer, obj->totcol, BKE_MAT_ASSIGN_OBDATA);

			MEM_freeN(matname);
			matname = NULL;
			matname = BLI_strdupcat(name, "_Inner");
			mat_inner = find_material(matname);
			object_add_material_slot(obj);
			assign_material(obj, mat_inner, obj->totcol, BKE_MAT_ASSIGN_OBDATA);

			MEM_freeN(matname);
			matname = NULL;

			fmd->inner_material = mat_inner;
		}
		else if (*totmat == 1)
		{
			char* matname = BLI_strdupcat(name, "_Inner");
			Material* mat_inner = find_material(matname);
			object_add_material_slot(obj);
			assign_material(obj, mat_inner, obj->totcol, BKE_MAT_ASSIGN_OBDATA);
			MEM_freeN(matname);
			matname = NULL;

			fmd->inner_material = mat_inner;
		}
		else /*use 2nd material slot*/
		{
			Material* mat_inner = give_current_material(obj, 2);

			fmd->inner_material = mat_inner;
		}

		mat_index = 2;
	}

	return mat_index;
}

static void cleanup_splinters(FractureModifierData *fmd, float mat[4][4], Shard *s, DerivedMesh *dm)
{
	if ((fmd->splinter_axis & MOD_FRACTURE_SPLINTER_X) ||
		(fmd->splinter_axis & MOD_FRACTURE_SPLINTER_Y) ||
		(fmd->splinter_axis & MOD_FRACTURE_SPLINTER_Z))
	{
		int i = 0, num_verts = 0;
		MVert* mvert = NULL, *mv;

		if (s) {
			mvert = s->mvert;
			num_verts = s->totvert;
		}
		else
		{
			mvert = dm->getVertArray(dm);
			num_verts = dm->getNumVerts(dm);
		}

		for (i = 0, mv = mvert; i < num_verts; i++, mv++)
		{
			mul_m4_v3(mat, mv->co);
		}
	}
}

static Shard* find_shard(ListBase *shards, ShardID id)
{
	Shard *s = shards->first;
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

static void arrange_shard(FractureModifierData *fmd, ShardID id, bool do_verts, float cent[3])
{
	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		zero_v3(cent);
		bool found = false;
		Shard *sh = find_shard(&fmd->frac_mesh->shard_map, id);
		ShardSequence *seq = fmd->current_shard_entry;
		while (seq->prev && sh)
		{
			Shard *par = find_shard(&seq->prev->frac_mesh->shard_map, sh->parent_id);
			if (par)
			{
				add_v3_v3(cent, par->centroid);
				found = true;
			}

			seq = seq->prev;
		}

		if (found && do_verts)
		{
			add_v3_v3(sh->centroid, cent);
			add_v3_v3(sh->min, cent);
			add_v3_v3(sh->max, cent);

			int i = 0;
			for (i = 0; i < sh->totvert; i++)
			{
				add_v3_v3(sh->mvert[i].co, cent);
			}
		}
	}
}

static void cleanup_arrange_shard(FractureModifierData *fmd, Shard* sh, float cent[3])
{
	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && sh)
	{
		sub_v3_v3(sh->centroid, cent);
		sub_v3_v3(sh->min, cent);
		sub_v3_v3(sh->max, cent);

		int i = 0;
		for (i = 0; i < sh->totvert; i++)
		{
			sub_v3_v3(sh->mvert[i].co, cent);
		}
	}
}

//this is the main fracture function, outsource to BKE, so op or rb system can call it
static void do_fracture(FractureModifierData *fmd, ShardID id, Object *obj, DerivedMesh *dm)
{
	/* dummy point cloud, random */
	FracPointCloud points;
	int num_settings = BLI_listbase_count(&fmd->fracture_settings);

	points = get_points_global(fmd, obj, dm, id);

	if (points.totpoints > 0 || fmd->use_greasepencil_edges) {
		bool temp = fmd->shards_to_islands;
		short mat_index = 0;
		float mat[4][4];
		Shard *s = NULL;
		float cent[3];

		/*splinters... just global axises and a length, for rotation rotate the object */
		s = do_splinters(fmd, points, &mat, id, dm);

		arrange_shard(fmd, id, true, cent);

		mat_index = do_materials(fmd, obj);
		mat_index = mat_index > 0 ? mat_index - 1 : mat_index;

#if 0
		if (fmd->uvlayer_name[0] && obj->type == OB_MESH)
		{
			//const char *name = "InnerUV";
			//create default uv layer (both on mesh and DM ?) and use it
			//ED_mesh_uv_texture_ensure((Mesh*)ob->data, name);
			int index = CustomData_get_named_layer_index(&dm->loopData, CD_MLOOPUV, fmd->uvlayer_name);
			if (index > -1)
				CustomData_free_layer(&dm->loopData, CD_MLOOPUV, dm->numLoopData,index);

			CustomData_add_layer_named(&dm->loopData, CD_MLOOPUV, CD_CALLOC, NULL, dm->numLoopData, fmd->uvlayer_name);
			//BLI_strncpy(fmd->uvlayer_name, name, sizeof(name));
		}
#endif

		if (points.totpoints > 0) {
			BKE_fracture_shard_by_points(fmd->frac_mesh, id, &points, fmd->frac_algorithm, obj, dm, mat_index, mat,
			                             fmd->fractal_cuts, fmd->fractal_amount, fmd->use_smooth, fmd->fractal_iterations,
			                             fmd->fracture_mode, fmd->reset_shards, fmd->active_setting, num_settings, fmd->uvlayer_name);
		}

		/*TODO, limit this to settings shards !*/
		if (fmd->point_source & MOD_FRACTURE_GREASEPENCIL && fmd->use_greasepencil_edges) {
			BKE_fracture_shard_by_greasepencil(fmd, obj, mat_index, mat);
		}

		if (fmd->frac_algorithm == MOD_FRACTURE_BOOLEAN && fmd->cutter_group != NULL) {
			BKE_fracture_shard_by_planes(fmd, obj, mat_index, mat);
		}

		cleanup_arrange_shard(fmd, s, cent);


		/* job has been cancelled, throw away all data */
		if (fmd->frac_mesh->cancel == 1)
		{
			fmd->frac_mesh->running = 0;
			fmd->refresh = true;
			freeData_internal(fmd, fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED, true);
			fmd->frac_mesh = NULL;
			fmd->refresh = false;
			MEM_freeN(points.points);
			return;
		}

		/* here we REALLY need to fracture so deactivate the shards to islands flag and activate afterwards */
		fmd->shards_to_islands = false;
		BKE_fracture_create_dm(fmd, true);
		fmd->shards_to_islands = temp;

		if (!s)
		{
			cleanup_splinters(fmd, mat, s, dm);
		}
		fmd->reset_shards = false;
	}
	MEM_freeN(points.points);
}

//XXX todo, simplify to copy generic stuff, maybe take shards over even, but re-init the meshisland verts as in packing system
static void copyData(ModifierData *md, ModifierData *target)
{
	FractureModifierData *rmd  = (FractureModifierData *)md;
	FractureModifierData *trmd = (FractureModifierData *)target;

	/*todo -> copy fracture stuff as well, and dont forget readfile / writefile...*/
	zero_m4(trmd->origmat);

	/* vgroups  XXX TODO non ascii strings ?*/
	strncpy(trmd->thresh_defgrp_name, rmd->thresh_defgrp_name, strlen(rmd->thresh_defgrp_name));
	strncpy(trmd->ground_defgrp_name, rmd->ground_defgrp_name, strlen(rmd->ground_defgrp_name));
	strncpy(trmd->inner_defgrp_name, rmd->inner_defgrp_name, strlen(rmd->inner_defgrp_name));

	trmd->visible_mesh = NULL;
	trmd->visible_mesh_cached = NULL;
	trmd->meshIslands.first = NULL;
	trmd->meshIslands.last = NULL;
	trmd->meshConstraints.first = NULL;
	trmd->meshConstraints.last = NULL;
	trmd->face_pairs = NULL;
	trmd->vert_index_map = NULL;
	trmd->vertex_island_map = NULL;

	trmd->breaking_threshold = rmd->breaking_threshold;
	trmd->use_constraints = rmd->use_constraints;
	trmd->contact_dist = rmd->contact_dist;
	trmd->use_mass_dependent_thresholds = rmd->use_mass_dependent_thresholds;
	trmd->explo_shared = rmd->explo_shared;

	trmd->refresh = true; //ensure valid data ?
	trmd->constraint_limit = rmd->constraint_limit;
	trmd->breaking_angle = rmd->breaking_angle;
	trmd->breaking_distance = rmd->breaking_distance;
	trmd->breaking_percentage = rmd->breaking_percentage;
	trmd->use_experimental = rmd->use_experimental;
	trmd->refresh_constraints = false;

	trmd->cluster_count = rmd->cluster_count;
	trmd->cluster_breaking_threshold = rmd->cluster_breaking_threshold;
	trmd->solver_iterations_override = rmd->solver_iterations_override;
	trmd->shards_to_islands = rmd->shards_to_islands;

	trmd->shard_count = rmd->shard_count;
	trmd->frac_algorithm = rmd->frac_algorithm;

	trmd->auto_execute = rmd->auto_execute;
	trmd->autohide_dist = rmd->autohide_dist;
	trmd->automerge_dist = rmd->automerge_dist;

	trmd->solver_iterations_override = rmd->solver_iterations_override;

	trmd->breaking_angle_weighted = rmd->breaking_angle_weighted;
	trmd->breaking_distance_weighted = rmd->breaking_distance_weighted;
	trmd->breaking_percentage_weighted = rmd->breaking_percentage_weighted;

	trmd->execute_threaded = rmd->execute_threaded;
	trmd->point_seed = rmd->point_seed;
	trmd->point_source = rmd->point_source;

	/*id refs ?*/
	trmd->inner_material = rmd->inner_material;
	trmd->extra_group = rmd->extra_group;

	/* sub object group  XXX Do we keep this ?*/
	trmd->dm_group = rmd->dm_group;

	trmd->cluster_group = rmd->cluster_group;
	trmd->cutter_group = rmd->cutter_group;

	trmd->use_particle_birth_coordinates = rmd->use_particle_birth_coordinates;
	trmd->splinter_length = rmd->splinter_length;
	trmd->cluster_solver_iterations_override = rmd->cluster_solver_iterations_override;

	trmd->cluster_breaking_angle = rmd->cluster_breaking_angle;
	trmd->cluster_breaking_distance = rmd->cluster_breaking_distance;
	trmd->cluster_breaking_percentage = rmd->cluster_breaking_percentage;

	trmd->use_breaking = rmd->use_breaking;
	trmd->use_smooth = rmd->use_smooth;
	trmd->fractal_cuts = rmd->fractal_cuts;
	trmd->fractal_amount = rmd->fractal_amount;

	trmd->grease_decimate = rmd->grease_decimate;
	trmd->grease_offset = rmd->grease_offset;
	trmd->use_greasepencil_edges = rmd->use_greasepencil_edges;
	trmd->cutter_axis = rmd->cutter_axis;

	trmd->cluster_constraint_type = rmd->cluster_constraint_type;
	trmd->constraint_target = rmd->constraint_target;

	trmd->fracture_mode = rmd->fracture_mode;
	trmd->last_frame = rmd->last_frame;
	trmd->dynamic_force = rmd->dynamic_force;
	trmd->update_dynamic = false;
	trmd->reset_shards = false;

	trmd->impulse_dampening = rmd->impulse_dampening;
	trmd->directional_factor = rmd->directional_factor;
	trmd->minimum_impulse = rmd->minimum_impulse;
	trmd->mass_threshold_factor = rmd->mass_threshold_factor;
	trmd->use_compounds = rmd->use_compounds;

	trmd->autohide_filter_group = rmd->autohide_filter_group;
}

//XXXX TODO, is BB really useds still ? aint there exact volume calc now ?
/* mi->bb, its for volume fraction calculation.... */
static float bbox_vol(BoundBox *bb)
{
	float x[3], y[3], z[3];

	sub_v3_v3v3(x, bb->vec[4], bb->vec[0]);
	sub_v3_v3v3(y, bb->vec[3], bb->vec[0]);
	sub_v3_v3v3(z, bb->vec[1], bb->vec[0]);

	return len_v3(x) * len_v3(y) * len_v3(z);
}

static void bbox_dim(BoundBox *bb, float dim[3])
{
	float x[3], y[3], z[3];

	sub_v3_v3v3(x, bb->vec[4], bb->vec[0]);
	sub_v3_v3v3(y, bb->vec[3], bb->vec[0]);
	sub_v3_v3v3(z, bb->vec[1], bb->vec[0]);

	dim[0] = len_v3(x);
	dim[1] = len_v3(y);
	dim[2] = len_v3(z);
}

//This still necessary ? if yes move to fracture.c for now
static int BM_calc_center_centroid(BMesh *bm, float cent[3], int tagged)
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
			BM_face_calc_center_mean(f, face_cent);
			face_area = BM_face_calc_area(f);

			madd_v3_v3fl(cent, face_cent, face_area);
			total_area += face_area;
		}
	}
	/* otherwise we get NAN for 0 polys */
	if (bm->totface) {
		mul_v3_fl(cent, 1.0f / total_area);
	}
	else if (bm->totvert == 1) {
		copy_v3_v3(cent, BM_vert_at_index_find(bm, 0)->co);
	}

	return (bm->totface != 0);
}


//XXX THose are double functionality.... one should be enough
static int DM_mesh_minmax(DerivedMesh *dm, float r_min[3], float r_max[3])
{
	MVert *v;
	int i = 0;
	for (i = 0; i < dm->numVertData; i++) {
		v = CDDM_get_vert(dm, i);
		minmax_v3v3_v3(r_min, r_max, v->co);
	}

	return (dm->numVertData != 0);
}


static int BM_mesh_minmax(BMesh *bm, float r_min[3], float r_max[3], int tagged)
{
	BMVert *v;
	BMIter iter;
	INIT_MINMAX(r_min, r_max);
	BM_ITER_MESH (v, &iter, bm, BM_VERTS_OF_MESH) {
		if ((tagged && BM_elem_flag_test(v, BM_ELEM_SELECT)) || (!tagged)) {
			minmax_v3v3_v3(r_min, r_max, v->co);
		}
	}

	return (bm->totvert != 0);
}

//XXX BKE
static int do_shard_to_island(FractureModifierData *fmd, BMesh* bm_new)
{
	DerivedMesh *dmtemp;
	Shard *s;

	if ((fmd->shards_to_islands || (fmd->frac_mesh && fmd->frac_mesh->shard_count < 2)) && (!fmd->dm_group)) {
		/* store temporary shards for each island */
		int id = 0;

		dmtemp = CDDM_from_bmesh(bm_new, true);
		s = BKE_create_fracture_shard(dmtemp->getVertArray(dmtemp), dmtemp->getPolyArray(dmtemp), dmtemp->getLoopArray(dmtemp),
		                              dmtemp->getNumVerts(dmtemp), dmtemp->getNumPolys(dmtemp), dmtemp->getNumLoops(dmtemp), true);
		s = BKE_custom_data_to_shard(s, dmtemp);
		id = BLI_listbase_count(&fmd->islandShards);
		s->shard_id = id;
		s->parent_id = -1;
		BLI_addtail(&fmd->islandShards, s);

		dmtemp->needsFree = 1;
		dmtemp->release(dmtemp);
		dmtemp = NULL;

		return id;
	}

	return -1;
}

//bke
static void do_rigidbody(FractureModifierData *fmd, MeshIsland* mi, Object* ob, DerivedMesh *orig_dm, short rb_type, int i)
{
	mi->rigidbody = NULL;
	mi->rigidbody = BKE_rigidbody_create_shard(fmd->modifier.scene, ob, NULL, mi);
	mi->rigidbody->type = rb_type;
	mi->rigidbody->meshisland_index = i;
	BKE_rigidbody_calc_shard_mass(ob, mi, orig_dm);
}

static short do_vert_index_map(FractureModifierData *fmd, MeshIsland *mi)
{
	short rb_type = mi->ground_weight > 0.01f ? RBO_TYPE_PASSIVE : RBO_TYPE_ACTIVE;

	if (fmd->vert_index_map && fmd->dm_group && fmd->cluster_count == 0 && mi->vertex_indices)
	{
		GroupObject* go = NULL;
		/* autocreate clusters out of former objects, if we dont override */
		mi->particle_index = GET_INT_FROM_POINTER(BLI_ghash_lookup(fmd->vert_index_map, SET_INT_IN_POINTER(mi->vertex_indices[0])));

		/*look up whether original object is active or passive */
		go = BLI_findlink(&fmd->dm_group->gobject, mi->particle_index);
		if (go && go->ob && go->ob->rigidbody_object) {
			rb_type = go->ob->rigidbody_object->type;
		}
	}

	return rb_type;
}

static void do_fix_normals(FractureModifierData *fmd, MeshIsland *mi)
{
	/* copy fixed normals to physicsmesh too, for convert to objects */
	if (fmd->fix_normals) {
		MVert *verts, *mv;
		int j = 0, totvert = 0;
		totvert = mi->vertex_count;
		verts = mi->physics_mesh->getVertArray(mi->physics_mesh);
		for (mv = verts, j = 0; j < totvert; mv++, j++) {
			short no[3];
			no[0] = mi->vertno[j * 3];
			no[1] = mi->vertno[j * 3 + 1];
			no[2] = mi->vertno[j * 3 + 2];

			copy_v3_v3_short(mv->no, no);
		}
	}
}

static float do_setup_meshisland(FractureModifierData *fmd, Object *ob, int totvert, float centroid[3],
                                 BMVert **verts, float *vertco, short *vertno, BMesh **bm_new, DerivedMesh *orig_dm, int id)
{
	MeshIsland *mi;
	DerivedMesh *dm;
	float dummyloc[3], rot[4], min[3], max[3], vol = 0;
	int i = 0;
	short rb_type = RBO_TYPE_ACTIVE;

	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");

	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
		mi->locs = MEM_mallocN(sizeof(float)*3, "mi->locs");
		mi->rots = MEM_mallocN(sizeof(float)*4, "mi->rots");
		mi->frame_count = 0;
	}
	else
	{
		/* in dynamic case preallocate cache here */
		int start = fmd->modifier.scene->rigidbody_world->pointcache->startframe;
		int end = fmd->modifier.scene->rigidbody_world->pointcache->endframe;

		if (fmd->current_mi_entry) {
			MeshIslandSequence *prev = fmd->current_mi_entry->prev;
			if (prev)
			{
				start = prev->frame;
			}
		}

		mi->frame_count = end - start + 1;
		mi->start_frame = start;
		mi->locs = MEM_mallocN(sizeof(float)*3* mi->frame_count, "mi->locs");
		mi->rots = MEM_mallocN(sizeof(float)*4* mi->frame_count, "mi->rots");
	}

	mi->particle_index = -1;
	mi->thresh_weight = 0.0f;
	mi->ground_weight = 0.0f;
	mi->id = id;
	mi->vertices = verts; /*those are temporary only !!! */
	mi->vertco = MEM_mallocN(sizeof(float) * 3 * totvert, "mi->vertco");
	memcpy(mi->vertco, vertco, 3 * totvert * sizeof(float));

	mi->vertno = MEM_mallocN(sizeof(short) * 3 * totvert, "mi->vertco");
	memcpy(mi->vertno, vertno, 3 * totvert * sizeof(short));
	zero_v3(mi->start_co);

	BM_mesh_normals_update(*bm_new);
	BM_mesh_minmax(*bm_new, min, max, false);
	dm = CDDM_from_bmesh(*bm_new, true);
	BM_mesh_free(*bm_new);
	*bm_new = NULL;

	mi->physics_mesh = dm;
	mi->vertex_count = totvert;

	mi->vertex_indices = MEM_mallocN(sizeof(int) * mi->vertex_count, "mi->vertex_indices");
	for (i = 0; i < mi->vertex_count; i++) {
		mi->vertex_indices[i] = mi->vertices[i]->head.index;
	}

	do_fix_normals(fmd, mi);

	copy_v3_v3(mi->centroid, centroid);
	mat4_to_loc_quat(dummyloc, rot, ob->obmat);
	copy_qt_qt(mi->rot, rot);
	mi->bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(mi->bb, min, max);
	mi->participating_constraints = NULL;
	mi->participating_constraint_count = 0;

	vol = bbox_vol(mi->bb);
	if (vol > fmd->max_vol) {
		fmd->max_vol = vol;
	}

	mi->vertices_cached = NULL;

	rb_type = do_vert_index_map(fmd, mi);
	i = BLI_listbase_count(&fmd->meshIslands);
	do_rigidbody(fmd, mi, ob, orig_dm, rb_type, i);

	mi->start_frame = fmd->modifier.scene->rigidbody_world->pointcache->startframe;

	BLI_addtail(&fmd->meshIslands, mi);

	return vol;
}

static float mesh_separate_tagged(FractureModifierData *fmd, Object *ob, BMVert **v_tag, int v_count,
                                  float *startco, BMesh *bm_work, short *startno, DerivedMesh *orig_dm)
{
	BMesh *bm_new;
	BMesh *bm_old = bm_work;
	float centroid[3];
	float vol;
	int id;

	BMVert *v;
	BMIter iter;

	if (fmd->frac_mesh && fmd->frac_mesh->cancel == 1)
		return 0.0f;

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

	BM_calc_center_centroid(bm_new, centroid, false);
	BM_mesh_elem_index_ensure(bm_new, BM_VERT | BM_EDGE | BM_FACE);

	id = do_shard_to_island(fmd, bm_new);

	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
		/* eliminate centroid in vertex coords */
		sub_v3_v3(v->co, centroid);
	}

	vol = do_setup_meshisland(fmd, ob, v_count, centroid, v_tag, startco, startno, &bm_new, orig_dm, id);

	/* deselect loose data - this used to get deleted,
	 * we could de-select edges and verts only, but this turns out to be less complicated
	 * since de-selecting all skips selection flushing logic */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);

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

static void handle_vert(FractureModifierData *fmd, DerivedMesh *dm, BMVert* vert, BMVert** orig_work,
                        float **startco, short **startno, BMVert*** v_tag, int *tot, int *tag_counter)
{
	/* treat the specified vert and put it into the tagged array, also store its coordinates and normals
	 * for usage in meshislands later on */

	short no[3];
	short vno[3];

	if (*v_tag == NULL)
		*v_tag = MEM_callocN(sizeof(BMVert *), "v_tag");

	if (*startco == NULL)
		*startco = MEM_callocN(sizeof(float), "mesh_separate_loose->startco");

	if (*startno == NULL)
		*startno = MEM_callocN(sizeof(short), "mesh_separate_loose->startno");

	BM_elem_flag_enable(vert, BM_ELEM_TAG);
	BM_elem_flag_enable(vert, BM_ELEM_INTERNAL_TAG);
	*v_tag = MEM_reallocN(*v_tag, sizeof(BMVert *) * ((*tag_counter) + 1));
	(*v_tag)[(*tag_counter)] = orig_work[vert->head.index];

	*startco = MEM_reallocN(*startco, ((*tag_counter) + 1) * 3 * sizeof(float));
	(*startco)[3 * (*tag_counter)] = vert->co[0];
	(*startco)[3 * (*tag_counter) + 1] = vert->co[1];
	(*startco)[3 * (*tag_counter) + 2] = vert->co[2];

	*startno = MEM_reallocN(*startno, ((*tag_counter) + 1) * 3 * sizeof(short));

	normal_float_to_short_v3(vno, vert->no);
	normal_float_to_short_v3(no, vert->no);
	if (fmd->fix_normals)
		find_normal(dm, fmd->nor_tree, vert->co, vno, no, fmd->nor_range);
	(*startno)[3 * (*tag_counter)] = no[0];
	(*startno)[3 * (*tag_counter) + 1] = no[1];
	(*startno)[3 * (*tag_counter) + 2] = no[2];

	(*tot)++;
	(*tag_counter)++;
}

static void mesh_separate_loose_partition(FractureModifierData *fmd, Object *ob, BMesh *bm_work, BMVert **orig_work, DerivedMesh *dm)
{
	int i, tag_counter = 0;
	BMEdge *e;
	BMVert *v_seed = NULL, **v_tag = NULL;
	BMWalker walker;
	int tot = 0;
	BMesh *bm_old = bm_work;
	int max_iter = bm_old->totvert;
	BMIter iter;
	float *startco = NULL;
	short *startno = NULL;

	if (max_iter > 0 && fmd->frac_mesh) {
		fmd->frac_mesh->progress_counter++;
	}

	/* Clear all selected vertices */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_INTERNAL_TAG | BM_ELEM_TAG, false);


	/* A "while (true)" loop should work here as each iteration should
	 * select and remove at least one vertex and when all vertices
	 * are selected the loop will break out. But guard against bad
	 * behavior by limiting iterations to the number of vertices in the
	 * original mesh.*/
	for (i = 0; i < max_iter; i++) {
		tag_counter = 0;

		BM_ITER_MESH (v_seed, &iter, bm_old, BM_VERTS_OF_MESH) {
			/* Hrm need to look at earlier verts to for unused ones.*/
			if (!BM_elem_flag_test(v_seed, BM_ELEM_TAG) && !BM_elem_flag_test(v_seed, BM_ELEM_INTERNAL_TAG)) {
				break;
			}
		}

		/* No vertices available, can't do anything */
		if (v_seed == NULL) {
			break;
		}
		/* Select the seed explicitly, in case it has no edges */
		if (!BM_elem_flag_test(v_seed, BM_ELEM_TAG) && !BM_elem_flag_test(v_seed, BM_ELEM_INTERNAL_TAG)) {
			handle_vert(fmd, dm, v_seed, orig_work, &startco, &startno, &v_tag, &tot, &tag_counter);
		}

		/* Walk from the single vertex, selecting everything connected
		 * to it */
		BMW_init(&walker, bm_old, BMW_VERT_SHELL,
		         BMW_MASK_NOP, BMW_MASK_NOP, BMW_MASK_NOP,
		         BMW_FLAG_NOP,
		         BMW_NIL_LAY);

		e = BMW_begin(&walker, v_seed);
		for (; e; e = BMW_step(&walker)) {
			if (!BM_elem_flag_test(e->v1, BM_ELEM_TAG) && !BM_elem_flag_test(e->v1, BM_ELEM_INTERNAL_TAG)) {
				handle_vert(fmd, dm, e->v1, orig_work, &startco, &startno, &v_tag, &tot, &tag_counter);
			}
			if (!BM_elem_flag_test(e->v2, BM_ELEM_TAG) && !BM_elem_flag_test(e->v2, BM_ELEM_INTERNAL_TAG)) {
				handle_vert(fmd, dm, e->v2, orig_work, &startco, &startno, &v_tag, &tot, &tag_counter);
			}
		}
		BMW_end(&walker);

		/* Flush the selection to get edge/face selections matching
		 * the vertex selection */
		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_TAG);

		/* Move selection into a separate object */
		mesh_separate_tagged(fmd, ob, v_tag, tag_counter, startco, bm_old, startno, dm);

		MEM_freeN(v_tag);
		v_tag = NULL;

		MEM_freeN(startco);
		startco = NULL;

		MEM_freeN(startno);
		startno = NULL;

		if (tot >= bm_old->totvert) {
			break;
		}
	}
}

/* inlined select_linked functionality here, because not easy to reach without modifications */
static void select_linked(BMesh **bm_in)
{
	BMIter iter;
	BMVert *v;
	BMEdge *e;
	BMWalker walker;
	BMesh *bm_work = *bm_in;


	BM_ITER_MESH (v, &iter, bm_work, BM_VERTS_OF_MESH) {
		if (BM_elem_flag_test(v, BM_ELEM_SELECT)) {
			BM_elem_flag_enable(v, BM_ELEM_TAG);
		}
		else {
			BM_elem_flag_disable(v, BM_ELEM_TAG);
		}
	}

	BMW_init(&walker, bm_work, BMW_VERT_SHELL,
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

static void mesh_separate_selected(BMesh **bm_work, BMesh **bm_out, BMVert **orig_work, BMVert ***orig_out1, BMVert ***orig_out2)
{
	BMesh *bm_old = *bm_work;
	BMesh *bm_new = *bm_out;
	BMVert *v, **orig_new = *orig_out1, **orig_mod = *orig_out2;
	BMIter iter;
	int new_index = 0, mod_index = 0;

	BM_mesh_elem_hflag_disable_all(bm_old, BM_FACE | BM_EDGE | BM_VERT, BM_ELEM_TAG, false);
	/* sel -> tag */
	BM_mesh_elem_hflag_enable_test(bm_old, BM_FACE | BM_EDGE | BM_VERT, BM_ELEM_TAG, true, false, BM_ELEM_SELECT);

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

	/* lets hope the order of elements in new mesh is the same as it was in old mesh */
	BM_ITER_MESH (v, &iter, bm_old, BM_VERTS_OF_MESH) {
		if (BM_elem_flag_test(v, BM_ELEM_TAG)) {
			orig_new[new_index] = orig_work[v->head.index];
			new_index++;
		}
		else {
			orig_mod[mod_index] = orig_work[v->head.index];
			mod_index++;
		}
	}

	new_index = 0;
	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
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

static void halve(FractureModifierData *rmd, Object *ob, int minsize, BMesh **bm_work, BMVert ***orig_work, bool separated, DerivedMesh *dm)
{

	int half;
	int i = 0, new_count = 0;
	BMIter iter;
	BMVert **orig_old = *orig_work, **orig_new, **orig_mod;
	BMVert *v;
	BMesh *bm_old = *bm_work;
	BMesh *bm_new = NULL;
	separated = false;

	if (rmd->frac_mesh && rmd->frac_mesh->cancel == 1) {
		return;
	}

	bm_new = BM_mesh_create(&bm_mesh_allocsize_default);

	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);

	half = bm_old->totvert / 2;
	BM_ITER_MESH (v, &iter, bm_old, BM_VERTS_OF_MESH) {
		if (i >= half) {
			break;
		}
		BM_elem_select_set(bm_old, (BMElem *)v, true);
		i++;
	}

	bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_SELECT);
	select_linked(&bm_old);

	new_count = bm_old->totvertsel;
	printf("Halving...%d => %d %d\n", bm_old->totvert, new_count, bm_old->totvert - new_count);

	orig_new = MEM_callocN(sizeof(BMVert *) * new_count, "orig_new");
	orig_mod = MEM_callocN(sizeof(BMVert *) * bm_old->totvert - new_count, "orig_mod");
	mesh_separate_selected(&bm_old, &bm_new, orig_old, &orig_new, &orig_mod);

	printf("Old New: %d %d\n", bm_old->totvert, bm_new->totvert);
	if ((bm_old->totvert <= minsize && bm_old->totvert > 0) || (bm_new->totvert == 0)) {
		mesh_separate_loose_partition(rmd, ob, bm_old, orig_mod, dm);
		separated = true;
	}

	if ((bm_new->totvert <= minsize && bm_new->totvert > 0) || (bm_old->totvert == 0)) {
		mesh_separate_loose_partition(rmd, ob, bm_new, orig_new, dm);
		separated = true;
	}

	if ((bm_old->totvert > minsize && bm_new->totvert > 0) || (bm_new->totvert == 0 && !separated)) {
		halve(rmd, ob, minsize, &bm_old, &orig_mod, separated, dm);
	}

	if ((bm_new->totvert > minsize && bm_old->totvert > 0) || (bm_old->totvert == 0 && !separated)) {
		halve(rmd, ob, minsize, &bm_new, &orig_new, separated, dm);
	}


	MEM_freeN(orig_mod);
	MEM_freeN(orig_new);
	BM_mesh_free(bm_new);
	bm_new = NULL;
}

static void mesh_separate_loose(FractureModifierData *rmd, Object *ob, DerivedMesh *dm)
{
	int minsize = 1000;
	BMesh *bm_work;
	BMVert *vert, **orig_start;
	BMIter iter;

	BM_mesh_elem_hflag_disable_all(rmd->visible_mesh, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);
	bm_work = BM_mesh_copy(rmd->visible_mesh);

	orig_start = MEM_callocN(sizeof(BMVert *) * rmd->visible_mesh->totvert, "orig_start");
	/* associate new verts with old verts, here indexes should match still */
	BM_ITER_MESH (vert, &iter, rmd->visible_mesh, BM_VERTS_OF_MESH)
	{
		orig_start[vert->head.index] = vert;
	}

	BM_mesh_elem_index_ensure(bm_work, BM_VERT);
	BM_mesh_elem_table_ensure(bm_work, BM_VERT);

	/* free old islandshards first, if any */
	while (rmd->islandShards.first) {
		Shard *s = rmd->islandShards.first;
		BLI_remlink(&rmd->islandShards, s);
		BKE_shard_free(s, true);
		s = NULL;
	}

	rmd->islandShards.first = NULL;
	rmd->islandShards.last = NULL;

	halve(rmd, ob, minsize, &bm_work, &orig_start, false, dm);

	MEM_freeN(orig_start);
	orig_start = NULL;
	BM_mesh_free(bm_work);
	bm_work = NULL;

}

static void do_constraint(FractureModifierData* fmd, MeshIsland *mi1, MeshIsland *mi2, int con_type, float thresh)
{
	RigidBodyShardCon *rbsc;
	rbsc = BKE_rigidbody_create_shard_constraint(fmd->modifier.scene, con_type, true);
	rbsc->mi1 = mi1;
	rbsc->mi2 = mi2;
	if (thresh == 0 || fmd->use_breaking == false) {
		rbsc->flag &= ~RBC_FLAG_USE_BREAKING;
	}

	rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;

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
}

static void connect_meshislands(FractureModifierData *fmd, MeshIsland *mi1, MeshIsland *mi2, int con_type, float thresh)
{
	int con_found = false;
	RigidBodyShardCon *con;
	bool ok = mi1 && mi1->rigidbody;
	ok = ok && mi2 && mi2->rigidbody;
	ok = ok && fmd->use_constraints;

	if (ok) {
		/* search local constraint list instead of global one !!! saves lots of time */
		int i;
		for (i = 0; i < mi1->participating_constraint_count; i++) {
			con = mi1->participating_constraints[i];
			if ((con->mi1 == mi2) || (con->mi2 == mi2)) {
				con_found = true;
				break;
			}
		}

		if (!con_found) {
			for (i = 0; i < mi2->participating_constraint_count; i++) {
				con = mi2->participating_constraints[i];
				if ((con->mi1 == mi1) || (con->mi2 == mi1)) {
					con_found = true;
					break;
				}
			}
		}
	}

	if (!con_found && ok) {
		do_constraint(fmd, mi1, mi2, con_type, thresh);
	}
}

static void search_tree_based(FractureModifierData *rmd, MeshIsland *mi, MeshIsland **meshIslands, KDTree **combined_tree, float co[3], float max_mass)
{
	int r = 0, limit = 0, i = 0;
	KDTreeNearest *n3 = NULL;
	float dist, obj_centr[3], factor;

	limit = rmd->constraint_limit;
	dist = rmd->contact_dist;
	factor = rmd->mass_threshold_factor;

#if 0
	if (factor > 0.0f && rmd->use_compounds) {
		if (mi->rigidbody->mass > 0.0f && max_mass > 0.0f) {
			float ratio = mi->rigidbody->mass / max_mass;
			dist *= (factor * ratio);
			limit *= (factor * ratio);
		}
	}
#endif

	if (rmd->constraint_target == MOD_FRACTURE_CENTROID) {
		mul_v3_m4v3(obj_centr, rmd->origmat, mi->centroid);
	}
	else if (rmd->constraint_target == MOD_FRACTURE_VERTEX){
		mul_v3_m4v3(obj_centr, rmd->origmat, co);
	}

	r = BLI_kdtree_range_search(*combined_tree, obj_centr, &n3, dist);

	/* use centroid dist based approach here, together with limit */
	for (i = 0; i < r; i++) {
		MeshIsland *mi2 = NULL;

		if (rmd->constraint_target == MOD_FRACTURE_CENTROID) {
			mi2 = meshIslands[(n3 + i)->index];
		}
		else if(rmd->constraint_target == MOD_FRACTURE_VERTEX) {
			int index = (n3 + i)->index;
			mi2 = BLI_ghash_lookup(rmd->vertex_island_map, SET_INT_IN_POINTER(index));
		}
		if ((mi != mi2) && (mi2 != NULL)) {
			float thresh = rmd->breaking_threshold;
			int con_type = rmd->use_compounds ? RBC_TYPE_COMPOUND : RBC_TYPE_FIXED;

			if ((i >= limit) && (limit > 0)) {
				break;
			}

			connect_meshislands(rmd, mi, mi2, con_type, thresh);
		}
	}

	if (n3 != NULL) {
		MEM_freeN(n3);
		n3 = NULL;
	}
}
static int prepareConstraintSearch(FractureModifierData *rmd, MeshIsland ***mesh_islands, KDTree **combined_tree)
{
	MeshIsland *mi;
	int i = 0, ret = 0;

	int islands = BLI_listbase_count(&rmd->meshIslands);
	*mesh_islands = MEM_reallocN(*mesh_islands, islands * sizeof(MeshIsland *));
	for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
		(*mesh_islands)[i] = mi;
		i++;
	}

	if (rmd->constraint_target == MOD_FRACTURE_CENTROID)
	{
		*combined_tree = BLI_kdtree_new(islands);
		for (i = 0; i < islands; i++) {
			float obj_centr[3];
			mul_v3_m4v3(obj_centr, rmd->origmat, (*mesh_islands)[i]->centroid);
			BLI_kdtree_insert(*combined_tree, i, obj_centr);
		}

		BLI_kdtree_balance(*combined_tree);
		ret = islands;
	}
	else if (rmd->constraint_target == MOD_FRACTURE_VERTEX)
	{
		int totvert = rmd->visible_mesh_cached->getNumVerts(rmd->visible_mesh_cached);
		MVert *mvert = rmd->visible_mesh_cached->getVertArray(rmd->visible_mesh_cached);
		MVert *mv;

		*combined_tree = BLI_kdtree_new(totvert);
		for (i = 0, mv = mvert; i < totvert; i++, mv++) {
			float co[3];
			mul_v3_m4v3(co, rmd->origmat, mv->co);
			BLI_kdtree_insert(*combined_tree, i, co);
		}

		BLI_kdtree_balance(*combined_tree);
		ret = totvert;
	}

	return ret;
}

static void create_constraints(FractureModifierData *rmd)
{
	KDTree *coord_tree = NULL;
	MeshIsland **mesh_islands = MEM_mallocN(sizeof(MeshIsland *), "mesh_islands");
	int count, i = 0;
	MeshIsland *mi;
	float max_mass = 0.0f;

	if (rmd->visible_mesh_cached && rmd->contact_dist == 0.0f) {
		/* extend contact dist to bbox max dimension here, in case we enter 0 */
		float min[3], max[3], dim[3];
		BoundBox *bb = BKE_boundbox_alloc_unit();
		DM_mesh_minmax(rmd->visible_mesh_cached, min, max);
		BKE_boundbox_init_from_minmax(bb, min, max);
		bbox_dim(bb, dim);
		rmd->contact_dist = MAX3(dim[0], dim[1], dim[2]);
		MEM_freeN(bb);
	}

	for (mi = rmd->meshIslands.first; mi; mi = mi->next)
	{
		if (mi->rigidbody->mass > max_mass)
			max_mass = mi->rigidbody->mass;
	}


	count = prepareConstraintSearch(rmd, &mesh_islands, &coord_tree);

	for (i = 0; i < count; i++) {
		if (rmd->constraint_target == MOD_FRACTURE_CENTROID) {
			search_tree_based(rmd, mesh_islands[i], mesh_islands, &coord_tree, NULL, max_mass);
		}
		else if (rmd->constraint_target == MOD_FRACTURE_VERTEX) {
			MVert mv;
			MeshIsland *mii = NULL;
			rmd->visible_mesh_cached->getVert(rmd->visible_mesh_cached, i, &mv);
			mii = BLI_ghash_lookup(rmd->vertex_island_map, SET_INT_IN_POINTER(i));
			search_tree_based(rmd, mii, mesh_islands, &coord_tree, mv.co, max_mass);
		}
	}

	if (coord_tree != NULL) {
		BLI_kdtree_free(coord_tree);
		coord_tree = NULL;
	}

	MEM_freeN(mesh_islands);
}

static void fill_vgroup(FractureModifierData *rmd, DerivedMesh *dm, MDeformVert *dvert, Object *ob)
{
	/* use fallback over inner material (no more, now directly via tagged verts) */
	if (rmd->inner_defgrp_name[0]) {
		int ind = 0;
		MPoly *mp = dm->getPolyArray(dm);
		MLoop *ml = dm->getLoopArray(dm);
		MVert *mv = dm->getVertArray(dm);
		int count = dm->getNumPolys(dm);
		int totvert = dm->getNumVerts(dm);
		const int inner_defgrp_index = defgroup_name_index(ob, rmd->inner_defgrp_name);

		if (dvert != NULL) {
			CustomData_free_layers(&dm->vertData, CD_MDEFORMVERT, totvert);
			dvert = NULL;
		}

		dvert = CustomData_add_layer(&dm->vertData, CD_MDEFORMVERT, CD_CALLOC,
		                             NULL, totvert);

		for (ind = 0; ind < count; ind++) {
			int j = 0;
			for (j = 0; j < (mp + ind)->totloop; j++) {
				MLoop *l;
				MVert *v;
				int l_index = (mp + ind)->loopstart + j;
				l = ml + l_index;
				v = mv + l->v;
				if (v->flag & ME_VERT_TMP_TAG) {
					defvert_add_index_notest(dvert + l->v, inner_defgrp_index, 1.0f);
					//v->flag &= ~ME_VERT_TMP_TAG;
				}
			}
		}
	}
}

static void do_cache_regular(FractureModifierData* fmd, MeshIsland *mi, int thresh_defgrp_index,
                             int ground_defgrp_index, MVert** verts, MDeformVert** dvert, int *vertstart)
{
	int i;

	for (i = 0; i < mi->vertex_count; i++) {
		mi->vertices_cached[i] = (*verts) + (*vertstart) + i;

		/* sum up vertexweights and divide by vertcount to get islandweight*/
		if (*dvert && ((*dvert) + (*vertstart) + i)->dw && fmd->thresh_defgrp_name[0]) {
			float vweight = defvert_find_weight((*dvert) + (*vertstart) + i, thresh_defgrp_index);
			mi->thresh_weight += vweight;
		}

		if (*dvert && ((*dvert) + (*vertstart) + i)->dw && fmd->ground_defgrp_name[0]) {
			float gweight = defvert_find_weight((*dvert) + (*vertstart) + i, ground_defgrp_index);
			mi->ground_weight += gweight;
		}

		if (mi->vertno != NULL && fmd->fix_normals) {
			short sno[3];
			sno[0] = mi->vertno[i * 3];
			sno[1] = mi->vertno[i * 3 + 1];
			sno[2] = mi->vertno[i * 3 + 2];
			copy_v3_v3_short(mi->vertices_cached[i]->no, sno);
		}
	}

	(*vertstart) += mi->vertex_count;
}

static void do_cache_split_islands(FractureModifierData* fmd, MeshIsland *mi, int thresh_defgrp_index,
                                   int ground_defgrp_index, MVert** verts, MDeformVert** dvert)
{
	int i;

	for (i = 0; i < mi->vertex_count; i++) {

		int index = mi->vertex_indices[i];
		if (index >= 0 && index <= fmd->visible_mesh->totvert) {
			mi->vertices_cached[i] = (*verts) + index;
		}
		else {
			mi->vertices_cached[i] = NULL;
		}

		if (*dvert && ((*dvert) + index)->dw && fmd->thresh_defgrp_name[0]) {
			float vweight = defvert_find_weight((*dvert) + index, thresh_defgrp_index);
			mi->thresh_weight += vweight;
		}

		if (*dvert && ((*dvert) + index)->dw && fmd->ground_defgrp_name[0]) {
			float gweight = defvert_find_weight((*dvert) + index, ground_defgrp_index);
			mi->ground_weight += gweight;
		}

		if (mi->vertno != NULL && fmd->fix_normals) {
			short sno[3];
			sno[0] = mi->vertno[i * 3];
			sno[1] = mi->vertno[i * 3 + 1];
			sno[2] = mi->vertno[i * 3 + 2];
			copy_v3_v3_short(mi->vertices_cached[i]->no, sno);
		}
	}
}

static DerivedMesh *createCache(FractureModifierData *fmd, Object *ob, DerivedMesh *origdm)
{
	MeshIsland *mi;
	DerivedMesh *dm;
	MVert *verts;
	MDeformVert *dvert = NULL;
	int vertstart = 0;
	const int thresh_defgrp_index = defgroup_name_index(ob, fmd->thresh_defgrp_name);
	const int ground_defgrp_index = defgroup_name_index(ob, fmd->ground_defgrp_name);
	bool orig_chosen = false;

	/*regular fracture case */
	if (fmd->dm && !fmd->shards_to_islands && (fmd->dm->getNumPolys(fmd->dm) > 0)) {
		dm = CDDM_copy(fmd->dm);
	}
	/* split to islands or halving case (fast bisect e.g.) */
	else if (fmd->visible_mesh && (fmd->visible_mesh->totface > 0) && BLI_listbase_count(&fmd->meshIslands) > 1) {
		dm = CDDM_from_bmesh(fmd->visible_mesh, true);
	}
	else if (origdm != NULL) {
		dm = CDDM_copy(origdm);
		orig_chosen = true;
	}
	else {
		return NULL;
	}

	DM_ensure_tessface(dm);
	DM_ensure_normals(dm);
	DM_update_tessface_data(dm);

	verts = dm->getVertArray(dm);

	if (dvert == NULL)
		dvert = dm->getVertDataArray(dm, CD_MDEFORMVERT);

	/* we reach this code when we fracture without "split shards to islands", but NOT when we load such a file...
	 * readfile.c has separate code for dealing with this XXX WHY ? there were problems with the mesh...*/
	for (mi = fmd->meshIslands.first; mi; mi = mi->next) {
		if (mi->vertices_cached) {
			MEM_freeN(mi->vertices_cached);
			mi->vertices_cached = NULL;
		}

		if (fmd->thresh_defgrp_name[0]) {
			mi->thresh_weight = 0;
		}

		mi->vertices_cached = MEM_mallocN(sizeof(MVert *) * mi->vertex_count, "mi->vertices_cached");
		if (fmd->dm != NULL && !fmd->shards_to_islands && !orig_chosen && fmd->visible_mesh == NULL) {
			do_cache_regular(fmd, mi, thresh_defgrp_index, ground_defgrp_index, &verts, &dvert, &vertstart);
		}
		else {  /* halving case... */
			do_cache_split_islands(fmd, mi, thresh_defgrp_index, ground_defgrp_index, &verts, &dvert);
		}

		if (mi->vertex_count > 0) {
			mi->thresh_weight /= mi->vertex_count;
			mi->ground_weight /= mi->vertex_count;
		}

		/*disable for dm_group, cannot paint onto this mesh at all */
		if (mi->rigidbody != NULL && fmd->dm_group == NULL) {
			mi->rigidbody->type = mi->ground_weight > 0.01f ? RBO_TYPE_PASSIVE : RBO_TYPE_ACTIVE;
		}

		/* use fallback over inner material*/
		fill_vgroup(fmd, dm, dvert, ob);
	}

	return dm;
}

static void refresh_customdata_image(Mesh *me, CustomData *pdata, int totface)
{
	int i;

	for (i = 0; i < pdata->totlayer; i++) {
		CustomDataLayer *layer = &pdata->layers[i];

		if (layer->type == CD_MTEXPOLY && me->mtpoly) {
			MTexPoly *tf = layer->data;
			int j;

			for (j = 0; j < totface; j++, tf++) {
				//simply use first image here...
				tf->tpage = me->mtpoly->tpage;
				tf->mode = me->mtpoly->mode;
				tf->flag = me->mtpoly->flag;
				tf->tile = me->mtpoly->tile;
				tf->transp = me->mtpoly->transp;

				/*if (tf->tpage && tf->tpage->id.us == 0) {
					tf->tpage->id.us = 1;
				}*/
			}
		}
	}
}

/* inline face center calc here */
static void DM_face_calc_center_mean(DerivedMesh *dm, MPoly *mp, float r_cent[3])
{
	MLoop *ml = NULL;
	MLoop *mloop = dm->getLoopArray(dm);
	MVert *mvert = dm->getVertArray(dm);
	int i = 0;

	zero_v3(r_cent);

	for (i = mp->loopstart; i < mp->loopstart + mp->totloop; i++) {
		MVert *mv = NULL;
		ml = mloop + i;
		mv = mvert + ml->v;

		add_v3_v3(r_cent, mv->co);

	}

	mul_v3_fl(r_cent, 1.0f / (float) mp->totloop);
}

static void do_match_normals(MPoly *mp, MPoly *other_mp, MVert *mvert, MLoop *mloop)
{
	MLoop ml, ml2;
	MVert *v, *v2;
	short sno[3];
	float fno[3], fno2[3];
	int j;

	if (mp->totloop == other_mp->totloop) //mpoly+index
	{
		for (j = 0; j < mp->totloop; j++)
		{
			ml = mloop[mp->loopstart + j];
			ml2 = mloop[other_mp->loopstart + j];
			v = mvert + ml.v;
			v2 = mvert + ml2.v;

			normal_short_to_float_v3(fno, v->no);
			normal_short_to_float_v3(fno2, v2->no);
			add_v3_v3(fno, fno2);
			mul_v3_fl(fno, 0.5f);
			normal_float_to_short_v3(sno, fno);
			copy_v3_v3_short(v->no, sno);
			copy_v3_v3_short(v2->no, sno);
		}
	}
}

static void make_face_pairs(FractureModifierData *fmd, DerivedMesh *dm, Object *ob)
{
	/* make kdtree of all faces of dm, then find closest face for each face*/
	MPoly *mp = NULL;
	MPoly *mpoly = dm->getPolyArray(dm);
	MLoop* mloop = dm->getLoopArray(dm);
	MVert* mvert = dm->getVertArray(dm);
	int totpoly = dm->getNumPolys(dm);
	KDTree *tree = BLI_kdtree_new(totpoly);
	int i = 0;
	int inner_index = find_material_index(ob, fmd->inner_material) - 1;

	//printf("Make Face Pairs\n");
	int faces = 0, pairs = 0;

	for (i = 0, mp = mpoly; i < totpoly; mp++, i++) {
		float co[3];
		DM_face_calc_center_mean(dm, mp, co);
		if (mp->mat_nr == inner_index)
		{
			BLI_kdtree_insert(tree, i, co);
			faces++;
		}
	}

	BLI_kdtree_balance(tree);

	/*now find pairs of close faces*/

	for (i = 0, mp = mpoly; i < totpoly; mp++, i++) {
		if (mp->mat_nr == inner_index) { /* treat only inner faces ( with inner material) */
			int index = -1, j = 0, r = 0;
			KDTreeNearest *n;
			float co[3];

			DM_face_calc_center_mean(dm, mp, co);
			r = BLI_kdtree_range_search(tree, co, &n, fmd->autohide_dist);
			//r = BLI_kdtree_find_nearest_n(tree, co, n, 2);
			/*2nd nearest means not ourselves...*/
			if (r == 0)
				continue;

			index = n[0].index;
			while ((j < r) && i == index) {
				index = n[j].index;
				//printf("I, INDEX %d %d %f\n", i, index, n[j].dist);
				j++;
			}

			if (!BLI_ghash_haskey(fmd->face_pairs, SET_INT_IN_POINTER(index))) {
				BLI_ghash_insert(fmd->face_pairs, SET_INT_IN_POINTER(i), SET_INT_IN_POINTER(index));
				pairs++;
				/*match normals...*/
				if (fmd->fix_normals) {
					do_match_normals(mp, mpoly+index, mvert, mloop);
				}
			}

			if (n != NULL) {
				MEM_freeN(n);
			}
		}
	}

	printf("faces, pairs: %d %d\n", faces, pairs);
	BLI_kdtree_free(tree);
}

static void find_other_face(FractureModifierData *fmd, int i, BMesh* bm, Object* ob, BMFace ***faces, int *del_faces)
{
	float f_centr[3], f_centr_other[3];
	BMFace *f1, *f2;
	int other = GET_INT_FROM_POINTER(BLI_ghash_lookup(fmd->face_pairs, SET_INT_IN_POINTER(i)));
	int inner_index = find_material_index(ob, fmd->inner_material) - 1;

	if (other == i)
	{
		//printf("other == i %d \n", i);
		f1 = BM_face_at_index(bm, i);

		if (f1->mat_nr == inner_index)
		{
			/*is this a remainder face ? */
			*faces = MEM_reallocN(*faces, sizeof(BMFace *) * ((*del_faces) + 1));
			(*faces)[*del_faces] = f1;
			(*del_faces) += 1;
		}

		return;
	}

	f1 = BM_face_at_index(bm, i);
	f2 = BM_face_at_index(bm, other);

	if ((f1 == NULL) || (f2 == NULL)) {
		return;
	}

	BM_face_calc_center_mean(f1, f_centr);
	BM_face_calc_center_mean(f2, f_centr_other);


	if ((len_squared_v3v3(f_centr, f_centr_other) < (fmd->autohide_dist)) && (f1 != f2) &&
	    (f1->mat_nr == inner_index) && (f2->mat_nr == inner_index))
	{
		bool in_filter = false;

		/*filter out face pairs, if we have an autohide filter group */
		if (fmd->autohide_filter_group){
			GroupObject *go;
			for (go = fmd->autohide_filter_group->gobject.first; go; go = go->next) {
				/*check location and scale (maximum size if nonuniform) for now */
				/*if not in any filter range, delete... else keep */
				Object* obj = go->ob;
				float f1_loc[3], f2_loc[3];
				float radius = MAX3(obj->size[0], obj->size[1], obj->size[2]);

				/* TODO XXX watch out if go->ob is parented to ob (Transformation error ?) */
				mul_v3_m4v3(f1_loc, ob->obmat, f_centr);
				mul_v3_m4v3(f2_loc, ob->obmat, f_centr_other);
				radius = radius * radius;

				if ((len_squared_v3v3(f1_loc, obj->loc) < radius) &&
					(len_squared_v3v3(f2_loc, obj->loc) < radius))
				{
					in_filter = true;
					break;
				}
				else
				{
					in_filter = false;
				}
			}
		}

		if (!fmd->autohide_filter_group || !in_filter)
		{
			/*intact face pairs */
			*faces = MEM_reallocN(*faces, sizeof(BMFace *) * ((*del_faces) + 2));
			(*faces)[*del_faces] = f1;
			(*faces)[(*del_faces) + 1] = f2;
			(*del_faces) += 2;
		}
	}
}

static DerivedMesh *do_autoHide(FractureModifierData *fmd, DerivedMesh *dm, Object *ob)
{
	int totpoly = dm->getNumPolys(dm);
	int i = 0;
	BMesh *bm = DM_to_bmesh(dm, true);
	DerivedMesh *result;
	BMFace **faces = MEM_mallocN(sizeof(BMFace *), "faces");
	int del_faces = 0;

	BM_mesh_elem_index_ensure(bm, BM_FACE);
	BM_mesh_elem_table_ensure(bm, BM_FACE);
	BM_mesh_elem_toolflags_ensure(bm);

	BM_mesh_elem_hflag_disable_all(bm, BM_FACE | BM_EDGE | BM_VERT , BM_ELEM_SELECT, false);

	for (i = 0; i < totpoly; i++) {
		find_other_face(fmd, i, bm, ob,  &faces, &del_faces);
	}

	for (i = 0; i < del_faces; i++) {
		BMFace *f = faces[i];
		if (f->l_first->e != NULL) { /* a lame check.... */
			BMIter iter;
			BMVert *v;
			BM_ITER_ELEM(v, &iter, f, BM_VERTS_OF_FACE)
			{
				BM_elem_flag_enable(v, BM_ELEM_SELECT);
			}

			BM_elem_flag_enable(f, BM_ELEM_SELECT);
		}
	}

	BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "delete_keep_normals geom=%hf context=%i", BM_ELEM_SELECT, DEL_FACES);

	if (fmd->automerge_dist > 0) {
		//separate this, because it costs performance and might not work so well with thin objects, but its useful for smooth objects
		BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	             "automerge_keep_normals verts=%hv dist=%f", BM_ELEM_SELECT,
	             fmd->automerge_dist); /*need to merge larger cracks*/

		if (fmd->fix_normals) {
			/* dissolve sharp edges with limit dissolve
			 * this causes massive flicker with displacements and possibly with glass too when autohide is enabled
			 * so use this only when fix normals has been requested and automerge is enabled
			 * for glass in most cases autohide is enough, for displacements too, fix normals and automerge are for special cases where you
			 * want to clear off nearly all cracks (with smooth objects for example), in those cases you still might experience flickering
			 * when using glass or displacements */
			BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "dissolve_limit_keep_normals "
						 "angle_limit=%f use_dissolve_boundaries=%b verts=%av edges=%ae delimit=%i",
						 DEG2RADF(1.0f), false, 0);
		}
	}

	result = CDDM_from_bmesh(bm, true);
	BM_mesh_free(bm);
	MEM_freeN(faces);

	return result;
}

static void do_fix_normals_physics_mesh(FractureModifierData *fmd, Shard* s, MeshIsland* mi, int i, DerivedMesh* orig_dm)
{
	MVert *mv, *verts;
	int totvert;
	int j;

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

		/* either take orignormals or take ones from fractured mesh */
		if (fmd->fix_normals) {
			find_normal(orig_dm, fmd->nor_tree, mv->co, mv->no, no, fmd->nor_range);
		}

		mi->vertno[j * 3] = no[0];
		mi->vertno[j * 3 + 1] = no[1];
		mi->vertno[j * 3 + 2] = no[2];

		if (fmd->fix_normals) {
			copy_v3_v3_short(mi->vertices_cached[j]->no, no);
			copy_v3_v3_short(mv->no, no);
		}

		/* then eliminate centroid in vertex coords*/
		sub_v3_v3(mv->co, s->centroid);
	}

	if (fmd->fix_normals)
	{
		printf("Fixing Normals: %d\n", i);
	}
}

static void do_verts_weights(FractureModifierData *fmd, Shard *s, MeshIsland *mi, int vertstart,
                             int thresh_defgrp_index, int ground_defgrp_index)
{
	MVert *mverts;
	int k;
	MDeformVert *dvert = fmd->dm->getVertDataArray(fmd->dm, CD_MDEFORMVERT);

	mi->vertices_cached = MEM_mallocN(sizeof(MVert *) * s->totvert, "vert_cache");
	mverts = CDDM_get_verts(fmd->visible_mesh_cached);

	mi->vertex_indices = MEM_mallocN(sizeof(int) * mi->vertex_count, "mi->vertex_indices");

	for (k = 0; k < s->totvert; k++) {
		mi->vertices_cached[k] = mverts + vertstart + k;
		mi->vertex_indices[k] = vertstart + k;
		/* sum up vertexweights and divide by vertcount to get islandweight*/
		if (dvert && fmd->thresh_defgrp_name[0]) {
			float vweight = defvert_find_weight(dvert + vertstart + k, thresh_defgrp_index);
			mi->thresh_weight += vweight;
		}

		if (dvert && fmd->ground_defgrp_name[0]) {
			float gweight = defvert_find_weight(dvert + vertstart + k, ground_defgrp_index);
			mi->ground_weight += gweight;
		}
	}

	if (mi->vertex_count > 0) {
		mi->thresh_weight /= mi->vertex_count;
		mi->ground_weight /= mi->vertex_count;
	}
}

#define OUT(name, id, co) printf("%s : %d -> (%.2f, %.2f, %.2f) \n", (name), (id), (co)[0], (co)[1], (co)[2]);
#define OUT4(name,id, co) printf("%s : %d -> (%.2f, %.2f, %.2f, %.2f) \n", (name), (id), (co)[0], (co)[1], (co)[2], (co)[3]);



static void do_handle_parent_mi(FractureModifierData *fmd, MeshIsland *mi, MeshIsland *par, Object* ob, int frame, bool is_parent)
{
	frame -= par->start_frame;
	BKE_match_vertex_coords(mi, par, ob, frame, is_parent);

	BKE_rigidbody_remove_shard(fmd->modifier.scene, par);
	fmd->modifier.scene->rigidbody_world->flag |= RBW_FLAG_OBJECT_CHANGED;
	par->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
}

static MeshIsland* find_meshisland(ListBase* meshIslands, int id)
{
	MeshIsland* mi = meshIslands->first;
	while (mi)
	{
		if (mi->id == id)
		{
			return mi;
		}

		mi = mi->next;
	}

	return NULL;
}

static bool contains(float loc[3], float size[3], float point[3])
{
	if ((fabsf(loc[0] - point[0]) < size[0]) &&
	    (fabsf(loc[1] - point[1]) < size[1]) &&
	    (fabsf(loc[2] - point[2]) < size[2]))
	{
		return true;
	}

	return false;
}

void set_rigidbody_type(FractureModifierData *fmd, Shard *s, MeshIsland *mi)
{
	//how far is impact location away from this shard, if beyond a bbox, keep passive
	if (fmd->current_shard_entry)
	{
		ShardSequence *prev_shards = fmd->current_shard_entry->prev;

		if (prev_shards && (prev_shards->prev == NULL)) //only affect primary fracture
		{
			Shard *par_shard = BKE_shard_by_id(prev_shards->frac_mesh, s->parent_id, NULL);
			if (par_shard)
			{
				float impact_loc[3], impact_size[3];
				copy_v3_v3(impact_loc, par_shard->impact_loc);
				copy_v3_v3(impact_size, par_shard->impact_size);

				if (contains(impact_loc, impact_size, s->centroid))
				{
					mi->rigidbody->flag &= ~RBO_FLAG_KINEMATIC;
				}
				else
				{
					mi->rigidbody->flag |= RBO_FLAG_KINEMATIC;
				}

				mi->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
			}
		}
	}
}

static void do_island_from_shard(FractureModifierData *fmd, Object *ob, Shard* s, DerivedMesh *orig_dm,
                                 int i, int thresh_defgrp_index, int ground_defgrp_index, int vertstart)
{
	MeshIsland *mi;
	MeshIsland *par = NULL;
	bool is_parent = false;
	short rb_type = RBO_TYPE_ACTIVE;
	float dummyloc[3], rot[4];

	if (s->totvert == 0) {
		return;
	}

	fmd->frac_mesh->progress_counter++;

	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
	BLI_addtail(&fmd->meshIslands, mi);

	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
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
	}
	else
	{
		/* in dynamic case preallocate cache here */
		int start = 1;
		int end = 250;

		if (fmd->modifier.scene->rigidbody_world)
		{
			start = fmd->modifier.scene->rigidbody_world->pointcache->startframe;
			end = fmd->modifier.scene->rigidbody_world->pointcache->endframe;
		}

		if (fmd->current_mi_entry) {
			MeshIslandSequence *prev = fmd->current_mi_entry->prev;
			if (prev)
			{
				start = prev->frame + 1;
			}
		}

		mi->frame_count = end - start + 1;
		mi->start_frame = start;
		mi->locs = MEM_mallocN(sizeof(float)*3* mi->frame_count, "mi->locs");
		mi->rots = MEM_mallocN(sizeof(float)*4* mi->frame_count, "mi->rots");
	}

	mi->participating_constraints = NULL;
	mi->participating_constraint_count = 0;
	mi->thresh_weight = 0.0f;
	mi->ground_weight = 0.0f;
	mi->vertex_count = s->totvert;

	do_verts_weights(fmd, s, mi, vertstart, thresh_defgrp_index, ground_defgrp_index);

	/*copy fixed normals to physics mesh too (needed for convert to objects)*/

	do_fix_normals_physics_mesh(fmd, s, mi, i, orig_dm);

	BKE_shard_calc_minmax(s);
	copy_v3_v3(mi->centroid, s->centroid);

	mat4_to_loc_quat(dummyloc, rot, ob->obmat);
	copy_qt_qt(mi->rot, rot);
	mi->id = s->shard_id;

	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		/*take care of previous transformation, if any*/
		MeshIslandSequence *prev = NULL;

		if (fmd->current_mi_entry) {
			prev = fmd->current_mi_entry->prev;
		}

		/*also take over the UNFRACTURED last shards transformation !!! */
		if (s->parent_id == 0)
		{
			//float quat[4];

			//TODO, scale ?
			//mat4_to_quat(quat, ob->obmat);

			mi->locs[0] = mi->centroid[0];
			mi->locs[1] = mi->centroid[1];
			mi->locs[2] = mi->centroid[2];

			mi->rots[0] = mi->rot[0];
			mi->rots[1] = mi->rot[1];
			mi->rots[2] = mi->rot[2];
			mi->rots[3] = mi->rot[3];
		}

		if (prev)
		{
			int frame = prev->frame;

			par = find_meshisland(&prev->meshIslands, s->parent_id);
			if (par)
			{
				is_parent = true;
				do_handle_parent_mi(fmd, mi, par, ob, frame, is_parent);
			}
			else
			{
				par = find_meshisland(&prev->meshIslands, s->shard_id);
				if (par)
				{
					is_parent = false;
					do_handle_parent_mi(fmd, mi, par, ob, frame, is_parent);
				}
			}
		}
	}

	mi->bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(mi->bb, s->min, s->max);

	mi->particle_index = -1;
	mi->neighbor_ids = s->neighbor_ids;
	mi->neighbor_count = s->neighbor_count;

	rb_type = do_vert_index_map(fmd, mi);
	do_rigidbody(fmd, mi, ob, orig_dm, rb_type, i);

	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		if (par != NULL)
		{
			copy_v3_v3(mi->rigidbody->lin_vel, par->rigidbody->lin_vel);
			copy_v3_v3(mi->rigidbody->ang_vel, par->rigidbody->ang_vel);
			mi->rigidbody->flag = par->rigidbody->flag;
		}

		if (fmd->limit_impact)
		{
			set_rigidbody_type(fmd, s, mi);
		}
	}
}

static MDeformVert* do_islands_from_shards(FractureModifierData* fmd, Object* ob, DerivedMesh *orig_dm)
{
	/* can be created without shards even, when using fracturemethod = NONE (re-using islands)*/
	Shard *s;
	int i = 0, vertstart = 0;

	MDeformVert *ivert = NULL;
	ListBase shardlist;
	const int thresh_defgrp_index = defgroup_name_index(ob, fmd->thresh_defgrp_name);
	const int ground_defgrp_index = defgroup_name_index(ob, fmd->ground_defgrp_name);

	/*XXX should rename this... this marks the fracture case, to distinguish from halving case */
	fmd->explo_shared = true;

	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
		/* exchange cached mesh after fracture, XXX looks like double code */
		if (fmd->visible_mesh_cached) {
			fmd->visible_mesh_cached->needsFree = 1;
			fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
			fmd->visible_mesh_cached = NULL;
		}

		fmd->visible_mesh_cached = CDDM_copy(fmd->dm);

		/* to write to a vgroup (inner vgroup) use the copied cached mesh */
		ivert = fmd->visible_mesh_cached->getVertDataArray(fmd->visible_mesh_cached, CD_MDEFORMVERT);

		if (ivert == NULL) {    /* add, if not there */
			int totvert = fmd->visible_mesh_cached->getNumVerts(fmd->visible_mesh_cached);
			ivert = CustomData_add_layer(&fmd->visible_mesh_cached->vertData, CD_MDEFORMVERT, CD_CALLOC,
										 NULL, totvert);
		}
	}
	else
	{
		fmd->visible_mesh_cached = CDDM_copy(fmd->dm);
	}

	shardlist = fmd->frac_mesh->shard_map;

	for (s = shardlist.first; s; s = s->next) {
		do_island_from_shard(fmd, ob, s, orig_dm, i, thresh_defgrp_index, ground_defgrp_index, vertstart);
		vertstart += s->totvert;
		i++;
	}

	return ivert;
}

static DerivedMesh *output_dm(FractureModifierData* fmd, DerivedMesh *dm, Object* ob, bool exploOK)
{
	if ((fmd->visible_mesh_cached != NULL) && exploOK) {
		DerivedMesh *dm_final;

		if (fmd->autohide_dist > 0 && fmd->face_pairs) {
			//printf("Autohide \n");
			dm_final = do_autoHide(fmd, fmd->visible_mesh_cached, ob);
		}
		else {
			dm_final = CDDM_copy(fmd->visible_mesh_cached);
		}
		return dm_final;
	}
	else {
		if (fmd->visible_mesh == NULL && fmd->visible_mesh_cached == NULL) {
			/* oops, something went definitely wrong... */
			fmd->refresh = true;
			freeData_internal(fmd, fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED, true);
			fmd->visible_mesh_cached = NULL;
			fmd->refresh = false;
		}
	}

	return dm;
}

static void do_post_island_creation(FractureModifierData *fmd, Object *ob, DerivedMesh *dm)
{
	double start;

	if (((fmd->visible_mesh != NULL && fmd->refresh && (!fmd->explo_shared)) || (fmd->visible_mesh_cached == NULL))
		&& (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED))
	{
		start = PIL_check_seconds_timer();
		/*post process ... convert to DerivedMesh only at refresh times, saves permanent conversion during execution */
		if (fmd->visible_mesh_cached != NULL) {
			fmd->visible_mesh_cached->needsFree = 1;
			fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
			fmd->visible_mesh_cached = NULL;
		}

		if (fmd->refresh_images && fmd->dm) {
			/*need to ensure images are correct after loading... */
			refresh_customdata_image(ob->data, &fmd->dm->polyData,
			                         fmd->dm->getNumPolys(fmd->dm));
			fmd->refresh_images = false;
		}

		fmd->visible_mesh_cached = createCache(fmd, ob, dm);
		printf("Building cached DerivedMesh done, %g\n", PIL_check_seconds_timer() - start);
	}
	else
	{
		/* fallback, this branch is executed when the modifier data has been loaded via readfile.c,
		 * although this might not be directly visible due to complex logic */

		MDeformVert* dvert = NULL;
		if (fmd->visible_mesh_cached) {
			dvert = fmd->visible_mesh_cached->getVertDataArray(fmd->visible_mesh_cached, CD_MDEFORMVERT);
			fill_vgroup(fmd, fmd->visible_mesh_cached, dvert, ob);
		}
	}

	if (fmd->refresh_images && fmd->visible_mesh_cached) {
		/* need to ensure images are correct after loading... */
		refresh_customdata_image(ob->data, &fmd->visible_mesh_cached->polyData,
		                         fmd->visible_mesh_cached->getNumPolys(fmd->visible_mesh_cached));
		fmd->refresh_images = false;
		DM_update_tessface_data(fmd->visible_mesh_cached);
	}

	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC && fmd->refresh == true)
	{
		//if (fmd->modifier.scene->rigidbody_world->object_changed == false)
		{
			fmd->current_mi_entry->is_new = false;
			//fmd->current_shard_entry->is_new = false;
		}
	}

	fmd->refresh = false;
	fmd->refresh_constraints = true;
	fmd->refresh_autohide = true;

	if (fmd->execute_threaded) {
		/* job done */
		fmd->frac_mesh->running = 0;
	}
}

static void do_refresh_constraints(FractureModifierData *fmd, Object *ob)
{
	double start = PIL_check_seconds_timer();
	do_clusters(fmd, ob);
	printf("Clustering done, %g\n", PIL_check_seconds_timer() - start);

	start = PIL_check_seconds_timer();

	if (fmd->use_constraints) {
		create_constraints(fmd); /* check for actually creating the constraints inside*/
	}
	fmd->refresh_constraints = false;

	printf("Building constraints done, %g\n", PIL_check_seconds_timer() - start);
	printf("Constraints: %d\n", BLI_listbase_count(&fmd->meshConstraints));
}

static void do_refresh_autohide(FractureModifierData *fmd, Object *ob)
{
	fmd->refresh_autohide = false;
	/*HERE make a kdtree of the fractured derivedmesh,
	 * store pairs of faces (MPoly) here (will be most likely the inner faces) */
	if (fmd->face_pairs != NULL) {
		BLI_ghash_free(fmd->face_pairs, NULL, NULL);
		fmd->face_pairs = NULL;
	}

	fmd->face_pairs = BLI_ghash_int_new("face_pairs");

	if (fmd->dm)
	{
		make_face_pairs(fmd, fmd->dm, ob);
	}
	else if (fmd->visible_mesh)
	{
		DerivedMesh *fdm = CDDM_from_bmesh(fmd->visible_mesh, true);
		make_face_pairs(fmd, fdm, ob);

		fdm->needsFree = 1;
		fdm->release(fdm);
		fdm = NULL;
	}
}

/*XXX should never happen */
static void do_clear(FractureModifierData* fmd)
{
	MeshIsland *mi;
	/* nullify invalid data */
	for (mi = fmd->meshIslands.first; mi; mi = mi->next) {
		mi->vertco = NULL;
		mi->vertex_count = 0;
		mi->vertices = NULL;
		if (mi->vertices_cached)
		{
			MEM_freeN(mi->vertices_cached);
			mi->vertices_cached = NULL;
		}
	}

	if (fmd->visible_mesh_cached) {
		fmd->visible_mesh_cached->needsFree = 1;
		fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
		fmd->visible_mesh_cached = NULL;
	}
}

static void do_halving(FractureModifierData *fmd, Object* ob, DerivedMesh *dm, DerivedMesh *orig_dm, bool is_prehalving)
{
	double start;

	if (fmd->dm && fmd->shards_to_islands && !is_prehalving) {
		fmd->visible_mesh = DM_to_bmesh(fmd->dm, true);
	}
	else {
		/* split to meshislands now */
		fmd->visible_mesh = DM_to_bmesh(dm, true); /* ensures indexes automatically*/
	}

	start = PIL_check_seconds_timer();
	//printf("Steps: %d \n", fmd->frac_mesh->progress_counter);
	mesh_separate_loose(fmd, ob, orig_dm);
	printf("Splitting to islands done, %g \n"/*  Steps: %d \n"*/, PIL_check_seconds_timer() - start);//, fmd->frac_mesh->progress_counter);
}

static void do_refresh(FractureModifierData *fmd, Object *ob, DerivedMesh* dm, DerivedMesh *orig_dm)
{
	double start = 0.0;
	MDeformVert *ivert = NULL;

	copy_m4_m4(fmd->origmat, ob->obmat);

	//if (fmd->fracture_mode != MOD_FRACTURE_DYNAMIC)
	{
		/* refracture, convert the fracture shards to new meshislands here *
		 * shards = fracture datastructure
		 * meshisland = simulation datastructure */
		if (fmd->frac_mesh && fmd->frac_mesh->shard_count > 0 && fmd->dm && fmd->dm->numVertData > 0 &&
			!fmd->shards_to_islands /*&& !fmd->dm_group*/)
		{
			if (fmd->fix_normals)
			{
				start = PIL_check_seconds_timer();
			}

			ivert = do_islands_from_shards(fmd, ob, orig_dm);

			if (fmd->fix_normals) {
				printf("Fixing normals done, %g\n", PIL_check_seconds_timer() - start);
			}

			fill_vgroup(fmd, fmd->visible_mesh_cached, ivert, ob);
		}
		else if (fmd->fracture_mode != MOD_FRACTURE_DYNAMIC){
			if (fmd->visible_mesh != NULL) {
				BM_mesh_free(fmd->visible_mesh);
				fmd->visible_mesh = NULL;
			}
			do_halving(fmd, ob, dm, orig_dm, false);
			fmd->explo_shared = false;
		}
	}

	printf("Islands: %d\n", BLI_listbase_count(&fmd->meshIslands));

	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		/* Grrr, due to stupid design of mine (listbase as value in struct instead of pointer)
		 * we have to synchronize the lists here again */

		/* need to ensure(!) old pointers keep valid, else the whole meshisland concept is broken */
		fmd->current_mi_entry->visible_dm = fmd->visible_mesh_cached;
		fmd->current_mi_entry->meshIslands = fmd->meshIslands;
	}
}

static void do_island_index_map(FractureModifierData *fmd)
{
	MeshIsland *mi;

	if (fmd->vertex_island_map) {
		BLI_ghash_free(fmd->vertex_island_map, NULL, NULL);
	}
	fmd->vertex_island_map = BLI_ghash_ptr_new("island_index_map");

	for (mi = fmd->meshIslands.first; mi; mi = mi->next){
		int i = 0;
		if (mi->vertex_indices != NULL)
		{	/* might not existing yet for older files ! */
			for (i = 0; i < mi->vertex_count; i++)
			{
				BLI_ghash_insert(fmd->vertex_island_map, SET_INT_IN_POINTER(mi->vertex_indices[i]), mi);
			}
		}
	}
}


static DerivedMesh *doSimulate(FractureModifierData *fmd, Object *ob, DerivedMesh *dm, DerivedMesh *orig_dm)
{
	bool exploOK = false; /* doFracture */

	if ((fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED) ||
		((fmd->fracture_mode == MOD_FRACTURE_DYNAMIC) &&
		(fmd->current_mi_entry && fmd->current_mi_entry->is_new)))
	{
		if ((fmd->refresh) || (fmd->refresh_constraints && !fmd->execute_threaded) ||
			(fmd->refresh_constraints && fmd->execute_threaded && fmd->frac_mesh && fmd->frac_mesh->running == 0))
		{
			/* if we changed the fracture parameters */
			freeData_internal(fmd, fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED, true);

			/* 2 cases, we can have a visible mesh or a cached visible mesh, the latter primarily when loading blend from file or using halving */
			/* free cached mesh in case of "normal refracture here if we have a visible mesh, does that mean REfracture ?*/
			if (fmd->visible_mesh != NULL && !fmd->shards_to_islands && fmd->frac_mesh->shard_count > 0 && fmd->refresh) {
				if (fmd->visible_mesh_cached) {
					fmd->visible_mesh_cached->needsFree = 1;
					fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
				}
				fmd->visible_mesh_cached = NULL;
			}

			if (fmd->refresh) {
				do_refresh(fmd, ob, dm, orig_dm);
			}

			do_post_island_creation(fmd, ob, dm);
		}
	}

	if (fmd->refresh_autohide) {
		do_refresh_autohide(fmd, ob);
	}

	if (fmd->refresh_constraints) {
		do_island_index_map(fmd);
		do_refresh_constraints(fmd, ob);
	}

	/*XXX better rename this, it checks whether we have a valid fractured mesh */
	exploOK = !fmd->explo_shared || (fmd->explo_shared && fmd->dm && fmd->frac_mesh);

	if ((!exploOK) || (fmd->visible_mesh == NULL && fmd->visible_mesh_cached == NULL)) {
		do_clear(fmd);
	}

	return output_dm(fmd, dm, ob, exploOK);
}

static bool dependsOnTime(ModifierData *UNUSED(md))
{
	return true;
}

static bool dependsOnNormals(ModifierData *UNUSED(md))
{
	return true;
}

static void foreachIDLink(ModifierData *md, Object *ob,
                          IDWalkFunc walk, void *userData)
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	FractureSetting *fs;

	walk(userData, ob, (ID **)&fmd->inner_material, IDWALK_NOP);
	walk(userData, ob, (ID **)&fmd->extra_group, IDWALK_NOP);
	walk(userData, ob, (ID **)&fmd->dm_group, IDWALK_NOP);
	walk(userData, ob, (ID **)&fmd->cluster_group, IDWALK_NOP);
	walk(userData, ob, (ID **)&fmd->cutter_group, IDWALK_NOP);
	walk(userData, ob, (ID **)&fmd->autohide_filter_group, IDWALK_NOP);

	for (fs = fmd->fracture_settings.first; fs; fs = fs->next)
	{
		walk(userData, ob, (ID **)&fs->inner_material, IDWALK_NOP);
		walk(userData, ob, (ID **)&fs->extra_group, IDWALK_NOP);
		walk(userData, ob, (ID **)&fs->cluster_group, IDWALK_NOP);
		walk(userData, ob, (ID **)&fs->cutter_group, IDWALK_NOP);
	}
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md))
{
	CustomDataMask dataMask = 0;
	dataMask |= CD_MASK_MDEFORMVERT;
	return dataMask;
}

static void updateDepgraph(ModifierData *md, DagForest *forest,
                           struct Main *UNUSED(bmain),
                           struct Scene *UNUSED(scene),
                           Object *UNUSED(ob),
                           DagNode *obNode)
{
	FractureModifierData *fmd = (FractureModifierData *) md;

	if (fmd->extra_group) {
		GroupObject *go;
		for (go = fmd->extra_group->gobject.first; go; go = go->next) {
			if (go->ob)
			{
				DagNode *curNode = dag_get_node(forest, go->ob);
				dag_add_relation(forest, curNode, obNode,
				                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Fracture Modifier Extra Group");
			}
		}
	}

	if (fmd->autohide_filter_group) {
		GroupObject *go;
		for (go = fmd->autohide_filter_group->gobject.first; go; go = go->next) {
			if (go->ob)
			{
				DagNode *curNode = dag_get_node(forest, go->ob);
				dag_add_relation(forest, curNode, obNode,
				                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Fracture Modifier Autohide Filter Group");
			}
		}
	}
}

static void foreachObjectLink(
    ModifierData *md, Object *ob,
    ObjectWalkFunc walk, void *userData)
{
	FractureModifierData *fmd = (FractureModifierData *) md;

	if (fmd->extra_group) {
		GroupObject *go;
		for (go = fmd->extra_group->gobject.first; go; go = go->next) {
			if (go->ob) {
				walk(userData, ob, &go->ob, IDWALK_NOP);
			}
		}
	}

	if (fmd->cutter_group) {
		GroupObject *go;
		for (go = fmd->cutter_group->gobject.first; go; go = go->next) {
			if (go->ob) {
				walk(userData, ob, &go->ob, IDWALK_NOP);
			}
		}
	}

	if (fmd->autohide_filter_group) {
		GroupObject *go;
		for (go = fmd->autohide_filter_group->gobject.first; go; go = go->next) {
			if (go->ob) {
				walk(userData, ob, &go->ob, IDWALK_NOP);
			}
		}
	}
}

static ShardSequence* shard_sequence_add(FractureModifierData* fmd, float frame, DerivedMesh* dm)
{
	ShardSequence *ssq = MEM_mallocN(sizeof(ShardSequence), "shard_sequence_add");
	/*copy last state, to be modified now */
	if (fmd->frac_mesh == NULL) {
		Shard *s = NULL;
		bool temp = fmd->shards_to_islands;

		fmd->frac_mesh = BKE_create_fracture_container();
		/* create first shard covering the entire mesh */
		s = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
		                                     dm->numVertData, dm->numPolyData, dm->numLoopData, true);
		s = BKE_custom_data_to_shard(s, dm);
		s->flag = SHARD_INTACT;
		s->shard_id = 0;
		BLI_addtail(&fmd->frac_mesh->shard_map, s);
		fmd->frac_mesh->shard_count = 1;

		//build fmd->dm here !
		fmd->shards_to_islands = false;
		BKE_fracture_create_dm(fmd, true);
		fmd->shards_to_islands = temp;

		ssq->frac_mesh = fmd->frac_mesh;
	}
	else {
		ssq->frac_mesh = copy_fracmesh(fmd->frac_mesh);
	}

	ssq->is_new = true;
	ssq->frame = frame;
	BLI_addtail(&fmd->shard_sequence, ssq);

	return ssq;
}

static MeshIslandSequence* meshisland_sequence_add(FractureModifierData* fmd, float frame, Object *ob, DerivedMesh *dm)
{
	MeshIslandSequence *msq = MEM_mallocN(sizeof(MeshIslandSequence), "meshisland_sequence_add");
	msq->frame = frame;

	if (BLI_listbase_is_empty(&fmd->meshIslands)) {
		msq->meshIslands.first = NULL;
		msq->meshIslands.last = NULL;
		fmd->visible_mesh_cached = CDDM_copy(fmd->dm);
		do_islands_from_shards(fmd, ob, dm);
		msq->meshIslands = fmd->meshIslands;
		msq->visible_dm = fmd->visible_mesh_cached;
		fmd->refresh = false;
		msq->is_new = false;
	}
	else {
		msq->meshIslands.first = NULL;
		msq->meshIslands.last = NULL;
		msq->visible_dm = NULL;
		msq->is_new = true;
	}

	BLI_addtail(&fmd->meshIsland_sequence, msq);

	return msq;
}

static void add_new_entries(FractureModifierData* fmd, DerivedMesh *dm, Object* ob)
{
	int frame = (int)BKE_scene_frame_get(fmd->modifier.scene);
	int end = 250; //TODO might be problematic with longer sequences, take proper end value ?

	if (fmd->modifier.scene->rigidbody_world)
	{
		end = fmd->modifier.scene->rigidbody_world->pointcache->endframe;
	}

	if (fmd->current_shard_entry)
	{
		fmd->current_shard_entry->is_new = false;
		fmd->current_shard_entry->frame = frame;
	}
	fmd->current_shard_entry = shard_sequence_add(fmd, end, dm);
	fmd->frac_mesh = fmd->current_shard_entry->frac_mesh;

	if (fmd->current_mi_entry) {
		fmd->current_mi_entry->frame = frame;
	}

	fmd->current_mi_entry = meshisland_sequence_add(fmd, end, ob, dm);
	fmd->meshIslands = fmd->current_mi_entry->meshIslands;
}

static void do_modifier(FractureModifierData *fmd, Object *ob, DerivedMesh *dm)
{
	/*TODO_1 refresh, move to BKE and just call from operator for prefractured case*/

	if (fmd->refresh)
	{
		printf("ADD NEW 1: %s \n", ob->id.name);
		if (fmd->last_frame == INT_MAX && fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
		{
			if (fmd->reset_shards)
			{
				free_simulation(fmd, true, true);
				free_modifier(fmd, true, true);
				fmd->last_frame = 1;
			}
			else
			{
				free_simulation(fmd, false, true);
				fmd->last_frame = 1;
			}
		}

		if (fmd->fracture_mode != MOD_FRACTURE_DYNAMIC)
		{
			if (fmd->dm != NULL) {
				fmd->dm->needsFree = 1;
				fmd->dm->release(fmd->dm);
				fmd->dm = NULL;
			}
		}

		if (fmd->frac_mesh != NULL) {

			if (fmd->fracture_mode != MOD_FRACTURE_DYNAMIC)
			{
				BKE_fracmesh_free(fmd->frac_mesh, true);
				fmd->frac_mesh = NULL;
			}
		}

		/* here we just create the fracmesh, in dynamic case we add the first sequence entry as well */
		if (fmd->frac_mesh == NULL) {
			if (fmd->fracture_mode != MOD_FRACTURE_DYNAMIC)
			{
				fmd->frac_mesh = BKE_create_fracture_container();
			}
			else
			{
				add_new_entries(fmd, dm, ob);
			}

			/*only in prefracture case... and not even working there... :S*/
			if (fmd->execute_threaded && fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
			{
				fmd->frac_mesh->running = 1;
			}
		}

		if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
		{
			/*normal trees and autohide should work in dynamic too, in theory, but disable for now */
			/* build normaltree from origdm */
			if (fmd->nor_tree != NULL) {
				BLI_kdtree_free(fmd->nor_tree);
				fmd->nor_tree = NULL;
			}

			fmd->nor_tree = build_nor_tree(dm);
			if (fmd->face_pairs != NULL) {
				BLI_ghash_free(fmd->face_pairs, NULL, NULL);
				fmd->face_pairs = NULL;
			}

			fmd->face_pairs = BLI_ghash_int_new("face_pairs");
		}
	}

	/*HERE we must know which shard(s) to fracture... hmm shards... we should "merge" states which happen in the same frame automatically !*/
	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
		do_fracture(fmd, -1, ob, dm);
	}
	else
	{
		/* TODO_2 dynamic, this is called from rigidbody system only !!! so move out of the md loop as well, to BKE */
		int frame = (int)BKE_scene_frame_get(fmd->modifier.scene);

		if (!(BKE_lookup_mesh_state(fmd, frame, false)))
		{
			/*simulation mode*/
			/* bullet callbacks may happen multiple times per frame, in next frame we can evaluate them all,
			 * so we need some array of shardIDs or shards to fracture each *
			 * we need to loop over those shard IDs here, but lookup of shard ids might be slow, but fracturing of many shards is slower...
			 * should not have a visible effect in general */

			int count = 0; //BLI_listbase_count(&fmd->fracture_ids);
			int totpoint = 0;
			FractureID *fid = fmd->fracture_ids.first;

			while(fid) {
				FracPointCloud points;
				points = get_points_global(fmd, ob, dm, fid->shardID);
				totpoint += points.totpoints;
				MEM_freeN(points.points);
				points.totpoints = 0;
				fid = fid->next;
			}

			if (totpoint == 0)
			{
				fmd->update_dynamic = false;
				//return;
			}
			else
			{
				if (fmd->update_dynamic)
				{
					BKE_free_constraints(fmd);
					printf("ADD NEW 2: %s \n", ob->id.name);
					fmd->update_dynamic = false;
					add_new_entries(fmd, dm, ob);
				}

				while(fmd->fracture_ids.first){
					fid = (FractureID*)fmd->fracture_ids.first;
					do_fracture(fmd, fid->shardID, ob, dm);
					BLI_remlink(&fmd->fracture_ids, fid);
					MEM_freeN(fid);
					count++;
				}

				if (count > 0)
				{
					//BKE_free_constraints(fmd);
					printf("REFRESH: %s \n", ob->id.name);
					fmd->modifier.scene->rigidbody_world->flag |= RBW_FLAG_OBJECT_CHANGED;
					fmd->modifier.scene->rigidbody_world->flag &= ~RBW_FLAG_REFRESH_MODIFIERS;
					fmd->refresh = true;
				}
			}
		}

		fmd->last_frame = frame;
	}
}

static DerivedMesh *do_prefractured(FractureModifierData *fmd, Object *ob, DerivedMesh *derivedData)
{
	DerivedMesh *final_dm = derivedData;
	DerivedMesh *group_dm = get_group_dm(fmd, derivedData, ob);
	DerivedMesh *clean_dm = get_clean_dm(ob, group_dm);

	/* TODO_3, this must not be needed, for interactive preview make sure this gets called from transform ops, or keep here as exception for now,
	 * this might be simpler */
	/* disable that automatically if sim is started, but must be re-enabled manually */
/*	if (BKE_rigidbody_check_sim_running(fmd->modifier.scene->rigidbody_world, BKE_scene_frame_get(fmd->modifier.scene))) {
		fmd->auto_execute = false;
	}*/

	if (fmd->auto_execute) {
		fmd->refresh = true;
	}

	/* TODO_4, for proper threading / job support make sure to use locks, spinlocks, and if possible remove those running / cancel flags */
	/* make this local data in job struct maybe, fracturing could perhaps run parallel, but assembling the shards to a derivedmesh not, since
	* addition order matters */
	if (fmd->frac_mesh != NULL && fmd->frac_mesh->running == 1 && fmd->execute_threaded) {
		/* skip modifier execution when fracture job is running */
		return final_dm;
	}

	if (fmd->refresh)
	{
		do_modifier(fmd, ob, clean_dm);

		if (!fmd->refresh) { /* might have been changed from outside, job cancel*/
			return derivedData;
		}
	}

	/* TODO_5, get rid of fmd->dm and perhaps of fmd->visible_mesh (BMESH!) too, the latter should be runtime data for creating islands ONLY */
	/* we should ideally only have one cached derivedmesh */
	if (fmd->dm && fmd->frac_mesh && (fmd->dm->getNumPolys(fmd->dm) > 0)) {
		final_dm = doSimulate(fmd, ob, fmd->dm, clean_dm);
	}
	else {
		final_dm = doSimulate(fmd, ob, clean_dm, clean_dm);
	}

	/* free newly created derivedmeshes only, but keep derivedData and final_dm*/
	if ((clean_dm != group_dm) && (clean_dm != derivedData) && (clean_dm != final_dm))
	{
		clean_dm->needsFree = 1;
		clean_dm->release(clean_dm);
	}

	/* TODO_6 get rid of combine FM Meshes, either replace by external constraints (hard) or rely on packing mechanism and convert to objects before (via C) */
	if ((group_dm != derivedData) && (group_dm != final_dm))
	{
		group_dm->needsFree = 1;
		group_dm->release(group_dm);
	}

	return final_dm;
}

static DerivedMesh *do_dynamic(FractureModifierData *fmd, Object *ob, DerivedMesh *derivedData)
{
	DerivedMesh *final_dm = derivedData;

	/* group_dm, clean_dm not necessary here as we dont support non-mesh objects and subobject_groups here */
	//if (fmd->refresh)
	{
		/*in there we have to decide WHICH shards we fracture*/
		do_modifier(fmd, ob, derivedData);
	}

	/* here we should deal as usual with the current set of shards and meshislands */
	if (fmd->dm && fmd->frac_mesh && (fmd->dm->getNumPolys(fmd->dm) > 0)) {
		final_dm = doSimulate(fmd, ob, fmd->dm, derivedData);
	}
	else {
		final_dm = doSimulate(fmd, ob, derivedData, derivedData);
	}

	//fmd->last_frame = (int)BKE_scene_frame_get(fmd->modifier.scene);

	return final_dm;
}

static Shard* copy_shard(Shard *s)
{
	Shard *t = BKE_create_fracture_shard(s->mvert, s->mpoly, s->mloop, s->totvert, s->totpoly, s->totloop, true);

	CustomData_reset(&t->vertData);
	CustomData_reset(&t->loopData);
	CustomData_reset(&t->polyData);

	CustomData_copy(&s->vertData, &t->vertData, CD_MASK_MDEFORMVERT, CD_DUPLICATE, s->totvert);
	CustomData_copy(&s->loopData, &t->loopData, CD_MASK_MLOOPUV, CD_DUPLICATE, s->totloop);
	CustomData_copy(&s->polyData, &t->polyData, CD_MASK_MTEXPOLY, CD_DUPLICATE, s->totpoly);

	t->neighbor_count = t->neighbor_count;
	t->neighbor_ids = MEM_mallocN(sizeof(int) * s->neighbor_count, __func__);
	memcpy(t->neighbor_ids, s->neighbor_ids, sizeof(int) * s->neighbor_count);
	copy_v3_v3(t->centroid, s->centroid);
	copy_v3_v3(t->raw_centroid, s->raw_centroid);
	t->raw_volume = s->raw_volume;
	t->shard_id = s->shard_id;
	t->parent_id = s->parent_id;

	return t;
}

static ThreadMutex dynamic_lock = BLI_MUTEX_INITIALIZER;
static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{

	FractureModifierData *fmd = (FractureModifierData *) md;
	DerivedMesh *final_dm = derivedData;

	if (fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
		bool init = false;
		//deactivate multiple settings for now, not working properly XXX TODO (also deactivated in RNA and python)
		final_dm = do_prefractured(fmd, ob, derivedData);

		if (init)
			fmd->shard_count = 10;
	}
	else if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		if (ob->rigidbody_object == NULL) {
			//initialize FM here once
			fmd->refresh = true;
		}

		BLI_mutex_lock(&dynamic_lock);
		final_dm = do_dynamic(fmd, ob, derivedData);
		BLI_mutex_unlock(&dynamic_lock);
	}
	else if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		fmd->refresh = false;
		fmd->shards_to_islands = false;

		if (!fmd->visible_mesh_cached)
		{
			BKE_fracture_update_visual_mesh(fmd, ob, true);
		}

		if (fmd->visible_mesh_cached)
			return CDDM_copy(fmd->visible_mesh_cached);
	}

	return final_dm;
}

static DerivedMesh *applyModifierEM(ModifierData *md, Object *ob,
                                    struct BMEditMesh *UNUSED(editData),
                                    DerivedMesh *derivedData,
                                    ModifierApplyFlag flag)
{
	return applyModifier(md, ob, derivedData, flag);
}

ModifierTypeInfo modifierType_Fracture = {
	/* name */ "Fracture",
	/* structName */ "FractureModifierData",
	/* structSize */ sizeof(FractureModifierData),
	/* type */  eModifierTypeType_Constructive,
	/* flags */ eModifierTypeFlag_AcceptsMesh |
	eModifierTypeFlag_AcceptsCVs |
	eModifierTypeFlag_Single |
	eModifierTypeFlag_SupportsEditmode |
	eModifierTypeFlag_SupportsMapping |
	eModifierTypeFlag_UsesPreview,
	/* copyData */ copyData,
	/* deformVerts */ NULL,
	/* deformMatrices */ NULL,
	/* deformVertsEM */ NULL,
	/* deformMatricesEM */ NULL,
	/* applyModifier */ applyModifier,
	/* applyModifierEM */ applyModifierEM,
	/* initData */ initData,
	/* requiredDataMask */ requiredDataMask,
	/* freeData */ freeData,
	/* isDisabled */ NULL,
	/* updateDepgraph */ updateDepgraph,
	/* updateDepsgraph */ NULL,
	/* dependsOnTime */ dependsOnTime,
	/* dependsOnNormals */ dependsOnNormals,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */ foreachIDLink,
	/* foreachTexLink */ NULL,
};
