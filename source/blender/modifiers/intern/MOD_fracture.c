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


#include "BKE_cdderivedmesh.h"
#include "BKE_deform.h"
#include "BKE_depsgraph.h"
#include "BKE_fracture.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_library.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_particle.h"
#include "BKE_pointcache.h"
#include "BKE_rigidbody.h"
#include "BKE_scene.h"

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

#include "MOD_util.h"

#include "../../rigidbody/RBI_api.h"
#include "PIL_time.h"
#include "../../bmesh/tools/bmesh_decimate.h" /* decimate_dissolve function */
#include "depsgraph_private.h" /* for depgraph updates */

static void fill_vgroup(FractureModifierData *rmd, DerivedMesh *dm, MDeformVert *dvert, Object *ob);
static int getGroupObjects(Group *gr, Object ***obs, int g_exist);
static void do_fracture(FractureModifierData *fracmd, ShardID id, Object *obj, DerivedMesh *dm);
void freeMeshIsland(FractureModifierData *rmd, MeshIsland *mi, bool remove_rigidbody);
DerivedMesh *doSimulate(FractureModifierData *fmd, Object *ob, DerivedMesh *dm, DerivedMesh *orig_dm);
void refresh_customdata_image(Mesh *me, CustomData *pdata, int totface);

static void initData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData *) md;

	fmd->cluster_count = 0;
	fmd->extra_group = NULL;
	fmd->frac_algorithm = MOD_FRACTURE_BOOLEAN;
	fmd->point_source = MOD_FRACTURE_UNIFORM;
	fmd->shard_id = -1;
	fmd->shard_count = 10;
	fmd->percentage = 100;;

	fmd->visible_mesh = NULL;
	fmd->visible_mesh_cached = NULL;
	fmd->refresh = false;
	zero_m4(fmd->origmat);
	fmd->breaking_threshold = 10.0f;
	fmd->use_constraints = false;
	fmd->contact_dist = 1.0f;
	fmd->use_mass_dependent_thresholds = false;
	fmd->explo_shared = false;
	fmd->constraint_limit = 0;
	fmd->breaking_distance = 0;
	fmd->breaking_angle = 0;
	fmd->breaking_percentage = 0;     /* disable by default*/
	fmd->max_vol = 0;
	fmd->refresh_constraints = false;

	fmd->cluster_breaking_threshold = 1000.0f;
	fmd->solver_iterations_override = 0;
	fmd->shards_to_islands = false;
	fmd->execute_threaded = false;
	fmd->nor_tree = NULL;
	fmd->fix_normals = false;
	fmd->auto_execute = false;
	fmd->face_pairs = NULL;
	fmd->autohide_dist = 0.0f;

	fmd->breaking_percentage_weighted = false;
	fmd->breaking_angle_weighted = false;
	fmd->breaking_distance_weighted = false;

	/* XXX needed because of messy particle cache, shows incorrect positions when start/end on frame 1
	 * default use case is with this flag being enabled, disable at own risk */
	fmd->use_particle_birth_coordinates = true;
}

static void freeData(ModifierData *md)
{
	FractureModifierData *rmd = (FractureModifierData *) md;
	MeshIsland *mi;
	RigidBodyShardCon *rbsc;
	
	if ((!rmd->refresh && !rmd->refresh_constraints) || (rmd->frac_mesh && rmd->frac_mesh->cancel == 1)) {
		if (rmd->nor_tree != NULL) {
			BLI_kdtree_free(rmd->nor_tree);
			rmd->nor_tree = NULL;
		}

		if (rmd->face_pairs != NULL) {
			BLI_ghash_free(rmd->face_pairs, NULL, NULL);
			rmd->face_pairs = NULL;
		}

		//called on deleting modifier, object or quitting blender...
		if (rmd->dm) {
			rmd->dm->needsFree = 1;
			rmd->dm->release(rmd->dm);
			rmd->dm = NULL;
		}

		if (rmd->visible_mesh_cached) {
			rmd->visible_mesh_cached->needsFree = 1;
			rmd->visible_mesh_cached->release(rmd->visible_mesh_cached);
			rmd->visible_mesh_cached = NULL;
		}

		if (rmd->frac_mesh) {
			BKE_fracmesh_free(rmd->frac_mesh, true);
			MEM_freeN(rmd->frac_mesh);
			rmd->frac_mesh = NULL;
		}

		if (rmd->visible_mesh != NULL) {
			BM_mesh_free(rmd->visible_mesh);
			rmd->visible_mesh = NULL;
		}

		while (rmd->meshIslands.first) {
			mi = rmd->meshIslands.first;
			BLI_remlink(&rmd->meshIslands, mi);
			freeMeshIsland(rmd, mi, false);
			mi = NULL;
		}

		rmd->meshIslands.first = NULL;
		rmd->meshIslands.last = NULL;

		{
			while (rmd->islandShards.first) {
				Shard *s = rmd->islandShards.first;
				BLI_remlink(&rmd->islandShards, s);
				BKE_shard_free(s, true);
				s = NULL;
			}

			rmd->islandShards.first = NULL;
			rmd->islandShards.last = NULL;
		}
	}

	if (rmd->visible_mesh_cached && !rmd->shards_to_islands &&
	    ((!rmd->refresh && !rmd->refresh_constraints)))
	{
		rmd->visible_mesh_cached->needsFree = 1;
		rmd->visible_mesh_cached->release(rmd->visible_mesh_cached);
		rmd->visible_mesh_cached = NULL;
	}

	/* refreshing all simulation data, no refracture */
	if (!rmd->refresh_constraints) {
		if (rmd->shards_to_islands)
		{
			while (rmd->islandShards.first) {
				Shard *s = rmd->islandShards.first;
				BLI_remlink(&rmd->islandShards, s);
				BKE_shard_free(s, true);
				s = NULL;
			}

			rmd->islandShards.first = NULL;
			rmd->islandShards.last = NULL;
		}

		/* when freeing meshislands, we MUST get rid of constraints before too !!!! */
		for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
			if (mi->participating_constraints != NULL) {
				MEM_freeN(mi->participating_constraints);
				mi->participating_constraints = NULL;
				mi->participating_constraint_count = 0;
			}
		}

		while (rmd->meshConstraints.first) {
			rbsc = rmd->meshConstraints.first;
			BLI_remlink(&rmd->meshConstraints, rbsc);
			MEM_freeN(rbsc);
			rbsc = NULL;
		}

		rmd->meshConstraints.first = NULL;
		rmd->meshConstraints.last = NULL;

		while (rmd->meshIslands.first) {
			mi = rmd->meshIslands.first;
			BLI_remlink(&rmd->meshIslands, mi);
			freeMeshIsland(rmd, mi, true);
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
	}

	/* refresh constraints case */
	if ((!rmd->refresh && !rmd->refresh_constraints) || (rmd->frac_mesh && rmd->frac_mesh->cancel == 1) ||
	    rmd->refresh_constraints)
	{
		for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
			if (mi->participating_constraints != NULL) {
				MEM_freeN(mi->participating_constraints);
				mi->participating_constraints = NULL;
				mi->participating_constraint_count = 0;
			}
		}

		while (rmd->meshConstraints.first) {
			rbsc = rmd->meshConstraints.first;
			BLI_remlink(&rmd->meshConstraints, rbsc);
			MEM_freeN(rbsc);
			rbsc = NULL;
		}

		rmd->meshConstraints.first = NULL;
		rmd->meshConstraints.last = NULL;
	}
}

static void growCluster(FractureModifierData *fmd, Shard *seed, int sindex, ListBase *lbVisit, KDTree *tree, int depth)
{
	int i = 0, count = 0;
	float size = 0.1f;
	KDTreeNearest *nearest;

	count = BLI_kdtree_range_search(tree, seed->centroid, &nearest, size * depth);
	for (i = 0; i < count; i++) {
		Shard *neighbor;
		int index = nearest[i].index;
		neighbor = fmd->frac_mesh->shard_map[index];
		if (neighbor->cluster_colors[0] == -1) {
			neighbor->cluster_colors[0] = sindex;
		}

		if (BLI_findindex(lbVisit, seed) == -1) {
			BLI_addtail(lbVisit, seed);
		}
	}
	MEM_freeN(nearest);
}

static void doClusters(FractureModifierData *fmd, int levels)
{
	/*grow clusters from all shards */
	ListBase lbVisit;
	int i = 0, j = 0, k = 0, sindex = 0, counter = 0, depth = 1;
	KDTree *tree = BLI_kdtree_new(fmd->frac_mesh->shard_count);


	lbVisit.first = NULL;
	lbVisit.last = NULL;

	for (j = 0; j < levels; j++) {
		Shard **seeds;
		int seed_count;
		/*prepare shard as list*/
		for (i = 0; i < fmd->frac_mesh->shard_count; i++) {
			Shard *s = fmd->frac_mesh->shard_map[i];
			s->cluster_colors = MEM_mallocN(sizeof(int) * levels, "cluster_colors");
			for (k = 0; k < levels; k++)
			{
				s->cluster_colors[k] = -1;
			}

			BLI_kdtree_insert(tree, i, s->centroid);
		}

		if (fmd->cluster_count < 2) {
			BLI_kdtree_free(tree);
			return;
		}

		BLI_kdtree_balance(tree);

		seed_count = (fmd->cluster_count > fmd->frac_mesh->shard_count ? fmd->frac_mesh->shard_count : fmd->cluster_count);
		seeds = MEM_mallocN(sizeof(Shard *) * seed_count, "seeds");

		for (k = 0; k < seed_count; k++) {
			int color = k;
			int which_index = k * (int)(fmd->frac_mesh->shard_count / seed_count);
			Shard *which = fmd->frac_mesh->shard_map[which_index];
			which->cluster_colors[j] = color;
			BLI_addtail(&lbVisit, which);
			seeds[k] = which;
		}

		while (BLI_countlist(&lbVisit) < fmd->frac_mesh->shard_count) {
			Shard *seed;

			if (sindex == seed_count)
			{
				sindex = 0;
				depth++;
			}

			seed = seeds[sindex];
			growCluster(fmd, seed, sindex, &lbVisit, tree, depth);
			sindex++;

			if (counter == 10000) {
				/*XXX emergency stop... otherwise loop might run eternally */
				break;
			}

			counter++;
		}

		BLI_kdtree_free(tree);
		MEM_freeN(seeds);
	}
}

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

static void find_normal(DerivedMesh *dm, KDTree *tree, float co[3], short no[3])
{
	KDTreeNearest n;
	int index = 0;
	MVert mvert;

	index = BLI_kdtree_find_nearest(tree, co, &n);

	dm->getVert(dm, index, &mvert);

	copy_v3_v3_short(no, mvert.no);
}

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

static DerivedMesh *get_group_dm(FractureModifierData *fmd, DerivedMesh *dm)
{
	/* combine derived meshes from group objects into 1, trigger submodifiers if ob->derivedFinal is empty */
	int totgroup = 0, i = 0;
	int num_verts = 0, num_polys = 0, num_loops = 0;
	int vertstart = 0, polystart = 0, loopstart = 0;
	DerivedMesh *result;
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;

	Object **go = MEM_mallocN(sizeof(Object *), "groupdmobjects");
	totgroup = getGroupObjects(fmd->dm_group, &go, totgroup);

	if (totgroup > 0 && (fmd->refresh == true || fmd->auto_execute == true))
	{
		DerivedMesh *dm_ob = NULL;
		for (i = 0; i < totgroup; i++)
		{
			int v = 0;
			MVert *mv = NULL;
			Object *o = go[i];
			dm_ob = o->derivedFinal;
			if (dm_ob == NULL) continue;

			for (v = 0, mv = dm_ob->getVertArray(dm_ob); v < dm_ob->getNumVerts(dm_ob); v++, mv++)
			{
				mul_m4_v3(o->obmat, mv->co);
			}

			num_verts += dm_ob->getNumVerts(dm_ob);
			num_polys += dm_ob->getNumPolys(dm_ob);
			num_loops += dm_ob->getNumLoops(dm_ob);
		}

		if (num_verts == 0)
		{
			return dm;
		}

		result = CDDM_new(num_verts, 0, 0, num_loops, num_polys);
		mverts = CDDM_get_verts(result);
		mloops = CDDM_get_loops(result);
		mpolys = CDDM_get_polys(result);

		/* XXX doesnt work for some reason.... but is needed for textures and weightpaints */
		/*CustomData_add_layer(&result->vertData, CD_MDEFORMVERT, CD_CALLOC, NULL, num_verts);
		   CustomData_add_layer(&result->loopData, CD_MLOOPUV, CD_CALLOC, NULL, num_loops);
		   CustomData_add_layer(&result->polyData, CD_MTEXPOLY, CD_CALLOC, NULL, num_polys);*/

		vertstart = polystart = loopstart = 0;
		for (i = 0; i < totgroup; i++)
		{
			MPoly *mp;
			MLoop *ml;
			int j;

			Object *o = go[i];
			dm_ob = o->derivedFinal; /* not very reliable... hmm */

			if (dm_ob == NULL)
			{   /* avoid crash atleast...*/
				return dm;
			}

			memcpy(mverts + vertstart, dm_ob->getVertArray(dm_ob), dm_ob->getNumVerts(dm_ob) * sizeof(MVert));
			memcpy(mpolys + polystart, dm_ob->getPolyArray(dm_ob), dm_ob->getNumPolys(dm_ob) * sizeof(MPoly));


			for (j = 0, mp = mpolys + polystart; j < dm_ob->getNumPolys(dm_ob); ++j, ++mp) {
				/* adjust loopstart index */
				mp->loopstart += loopstart;
			}

			memcpy(mloops + loopstart, dm_ob->getLoopArray(dm_ob), dm_ob->getNumLoops(dm_ob) * sizeof(MLoop));

			for (j = 0, ml = mloops + loopstart; j < dm_ob->getNumLoops(dm_ob); ++j, ++ml) {
				/* adjust vertex index */
				ml->v += vertstart;
			}

			/* XXX doesnt work for some reason.... but is needed for textures and weightpaints */
			/*CustomData_copy_data(&dm_ob->vertData, &result->vertData, 0, vertstart, dm_ob->getNumVerts(dm_ob));
			   CustomData_copy_data(&dm_ob->loopData, &result->loopData, 0, loopstart, dm_ob->getNumLoops(dm_ob));
			   CustomData_copy_data(&dm_ob->polyData, &result->polyData, 0, polystart, dm_ob->getNumPolys(dm_ob));*/


			vertstart += dm_ob->getNumVerts(dm_ob);
			polystart += dm_ob->getNumPolys(dm_ob);
			loopstart += dm_ob->getNumLoops(dm_ob);
		}

		CDDM_calc_edges(result);

		result->dirty |= DM_DIRTY_NORMALS;
		CDDM_calc_normals_mapping(result);
		MEM_freeN(go);
		return result;
	}

	MEM_freeN(go);
	return dm;
}


static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{

	FractureModifierData *fmd = (FractureModifierData *) md;
	DerivedMesh *final_dm = derivedData;

	DerivedMesh *group_dm = get_group_dm(fmd, derivedData);
	DerivedMesh *clean_dm = get_clean_dm(ob, group_dm);

	/* disable that automatically if sim is started, but must be re-enabled manually */
	if (BKE_rigidbody_check_sim_running(md->scene->rigidbody_world, BKE_scene_frame_get(md->scene))) {
		fmd->auto_execute = false;
	}

	if (fmd->auto_execute) {
		fmd->refresh = true;
	}

	if (fmd->frac_mesh != NULL && fmd->frac_mesh->running == 1 && fmd->execute_threaded) {
		/* skip modifier execution when fracture job is running */
		return final_dm;
	}

	if (fmd->refresh)
	{
		if (fmd->dm != NULL) {
			fmd->dm->needsFree = 1;
			fmd->dm->release(fmd->dm);
			fmd->dm = NULL;
		}

		if (fmd->frac_mesh != NULL) {
			BKE_fracmesh_free(fmd->frac_mesh, true);
			MEM_freeN(fmd->frac_mesh);
			fmd->frac_mesh = NULL;
		}

		if (fmd->frac_mesh == NULL) {
			fmd->frac_mesh = BKE_create_fracture_container();
			if (fmd->execute_threaded)
			{
				fmd->frac_mesh->running = 1;
			}
		}
	}

	{
		if (fmd->refresh) {
			/* build normaltree from origdm */
			fmd->nor_tree = build_nor_tree(clean_dm);
			if (fmd->face_pairs != NULL) {
				BLI_ghash_free(fmd->face_pairs, NULL, NULL);
				fmd->face_pairs = NULL;
			}

			fmd->face_pairs = BLI_ghash_int_new("face_pairs");

			do_fracture(fmd, -1, ob, clean_dm);

			if (!fmd->refresh) { /* might have been changed from outside, job cancel*/
				return derivedData;
			}
		}
		if (fmd->dm && fmd->frac_mesh && (fmd->dm->getNumPolys(fmd->dm) > 0) && (fmd->dm_group == NULL)) {
			final_dm = doSimulate(fmd, ob, fmd->dm, clean_dm);
		}
		else {
			final_dm = doSimulate(fmd, ob, clean_dm, clean_dm);
		}
	}

	return final_dm;
}

static DerivedMesh *applyModifierEM(ModifierData *md, Object *ob,
                                    struct BMEditMesh *UNUSED(editData),
                                    DerivedMesh *derivedData,
                                    ModifierApplyFlag UNUSED(flag))
{
	FractureModifierData *fmd = (FractureModifierData *) md;
	DerivedMesh *final_dm = derivedData;

	DerivedMesh *group_dm = get_group_dm(fmd, derivedData);
	DerivedMesh *clean_dm = get_clean_dm(ob, group_dm);

	if (BKE_rigidbody_check_sim_running(md->scene->rigidbody_world, BKE_scene_frame_get(md->scene))) {
		fmd->auto_execute = false;
	}

	if (fmd->auto_execute) {
		fmd->refresh = true;
	}

	if (fmd->frac_mesh != NULL && fmd->frac_mesh->running == 1 && fmd->execute_threaded)
	{
		/* skip modifier execution when fracture job is running */
		return final_dm;
	}

	if (fmd->refresh) {
		if (fmd->dm != NULL) {
			fmd->dm->needsFree = 1;
			fmd->dm->release(fmd->dm);
			fmd->dm = NULL;
		}

		if (fmd->frac_mesh != NULL) {
			BKE_fracmesh_free(fmd->frac_mesh, true /*fmd->frac_algorithm != MOD_FRACTURE_VORONOI*/);
			MEM_freeN(fmd->frac_mesh);
			fmd->frac_mesh = NULL;
		}

		if (fmd->frac_mesh == NULL) {
			fmd->frac_mesh = BKE_create_fracture_container();
			if (fmd->execute_threaded) {
				fmd->frac_mesh->running = 1;
			}
		}
	}

	{
		if (fmd->refresh) {
			/* build normaltree from origdm */
			fmd->nor_tree = build_nor_tree(clean_dm);
			if (fmd->face_pairs != NULL) {
				BLI_ghash_free(fmd->face_pairs, NULL, NULL);
				fmd->face_pairs = NULL;
			}

			fmd->face_pairs = BLI_ghash_int_new("face_pairs");

			do_fracture(fmd, -1, ob, clean_dm);

			if (!fmd->refresh) { /*might have been changed from outside, job cancel*/
				return derivedData;
			}
		}

		if (fmd->dm && fmd->frac_mesh && (fmd->dm->getNumPolys(fmd->dm) > 0) && (fmd->dm_group == NULL)) {
			final_dm = doSimulate(fmd, ob, fmd->dm, clean_dm);
		}
		else {
			final_dm = doSimulate(fmd, ob, clean_dm, clean_dm);
		}
	}

	return final_dm;
}

static void points_from_verts(Object **ob, int totobj, FracPointCloud *points, float mat[4][4], float thresh, FractureModifierData *emd, DerivedMesh *dm, Object *obj)
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
					points->points = MEM_reallocN((*points).points, (pt + 1) * sizeof(FracPoint));

					copy_v3_v3(co, vert[v].co);


					if (emd->point_source & MOD_FRACTURE_EXTRA_VERTS) {
						mul_m4_v3(ob[o]->obmat, co);
					}

					mul_m4_v3(imat, co);

					copy_v3_v3(points->points[pt].co, co);
					pt++;
				}
			}
		}
	}

	points->totpoints = pt;
}

static void points_from_particles(Object **ob, int totobj, Scene *scene, FracPointCloud *points, float mat[4][4],
                                  float thresh, FractureModifierData *fmd)
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
						if (fmd->use_particle_birth_coordinates)
						{
							psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);
						}
						else {
							psys_get_particle_state(&sim, p, &birth, 1);
						}

						points->points = MEM_reallocN(points->points, (pt + 1) * sizeof(FracPoint));
						copy_v3_v3(co, birth.co);


						mul_m4_v3(imat, co);

						copy_v3_v3(points->points[pt].co, co);
						pt++;
					}
				}
			}
		}
	}

	points->totpoints = pt;
}

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

static FracPointCloud get_points_global(FractureModifierData *emd, Object *ob, DerivedMesh *fracmesh)
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
		points_from_particles(go, totgroup, scene, &points, ob->obmat, thresh, emd);
	}

	if (emd->point_source & (MOD_FRACTURE_OWN_VERTS | MOD_FRACTURE_EXTRA_VERTS)) {
		points_from_verts(go, totgroup, &points, ob->obmat, thresh, emd, fracmesh, ob);
	}

	if (emd->point_source & MOD_FRACTURE_GREASEPENCIL) {
		points_from_greasepencil(go, totgroup, &points, ob->obmat, thresh);
	}


	/* local settings, apply per shard!!! Or globally too first. */
	if (emd->point_source & MOD_FRACTURE_UNIFORM)
	{
		int count = emd->shard_count;
		INIT_MINMAX(min, max);
		BKE_get_shard_minmax(emd->frac_mesh, -1, min, max, fracmesh); //id 0 should be entire mesh

		if (emd->frac_algorithm == MOD_FRACTURE_BISECT_FAST || emd->frac_algorithm == MOD_FRACTURE_BISECT_FAST_FILL) {
			/* XXX need double amount of shards, because we create 2 islands at each cut... so this matches the input count */
			count *= 2;
		}

		BLI_srandom(emd->point_seed);
		for (i = 0; i < count; ++i) {
			if (BLI_frand() < thresh) {
				float *co;
				points.points = MEM_reallocN(points.points, sizeof(FracPoint) * (points.totpoints + 1));
				co = points.points[points.totpoints].co;
				co[0] = min[0] + (max[0] - min[0]) * BLI_frand();
				co[1] = min[1] + (max[1] - min[1]) * BLI_frand();
				co[2] = min[2] + (max[2] - min[2]) * BLI_frand();
				points.totpoints++;
			}
		}
	}

	MEM_freeN(go);
	return points;
}

static void do_fracture(FractureModifierData *fracmd, ShardID id, Object *obj, DerivedMesh *dm)
{
	/* dummy point cloud, random */
	FracPointCloud points;

	points = get_points_global(fracmd, obj, dm);

	if (points.totpoints > 0) {
		bool temp = fracmd->shards_to_islands;
		short mat_index = 0;

		if (fracmd->inner_material) {
			/* assign inner material as secondary mat to ob if not there already */
			mat_index = find_material_index(obj, fracmd->inner_material);
			if (mat_index == 0) {
				object_add_material_slot(obj);
				assign_material(obj, fracmd->inner_material, obj->totcol, BKE_MAT_ASSIGN_OBDATA);
			}

			/* get index again */
			mat_index = find_material_index(obj, fracmd->inner_material);
		}

		mat_index = mat_index > 0 ? mat_index - 1 : mat_index;
		BKE_fracture_shard_by_points(fracmd->frac_mesh, id, &points, fracmd->frac_algorithm, obj, dm, mat_index);

		/* job has been cancelled, throw away all data */
		if (fracmd->frac_mesh->cancel == 1)
		{
			fracmd->frac_mesh->running = 0;
			fracmd->refresh = true;
			freeData((ModifierData *)fracmd);
			fracmd->frac_mesh = NULL;
			fracmd->refresh = false;
			MEM_freeN(points.points);
			return;
		}

		if (fracmd->frac_mesh->shard_count > 0)
		{
			doClusters(fracmd, 1);
		}

		/* here we REALLY need to fracture so deactivate the shards to islands flag and activate afterwards */
		fracmd->shards_to_islands = false;
		BKE_fracture_create_dm(fracmd, true);
		fracmd->shards_to_islands = temp;
	}
	MEM_freeN(points.points);
}


static void copyData(ModifierData *md, ModifierData *target)
{
	FractureModifierData *rmd  = (FractureModifierData *)md;
	FractureModifierData *trmd = (FractureModifierData *)target;

	/*todo -> copy fracture stuff as well, and dont forget readfile / writefile...*/

	zero_m4(trmd->origmat);
	trmd->breaking_threshold = rmd->breaking_threshold;
	trmd->use_constraints = rmd->use_constraints;
	trmd->contact_dist = rmd->contact_dist;
	trmd->use_mass_dependent_thresholds = rmd->use_mass_dependent_thresholds;
	trmd->explo_shared = rmd->explo_shared;

	trmd->visible_mesh = NULL;
	trmd->visible_mesh_cached = NULL;
	trmd->meshIslands.first = NULL;
	trmd->meshIslands.last = NULL;
	trmd->meshConstraints.first = NULL;
	trmd->meshConstraints.last = NULL;

	trmd->refresh = false;
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

	trmd->solver_iterations_override = rmd->solver_iterations_override;

	trmd->breaking_angle_weighted = rmd->breaking_angle_weighted;
	trmd->breaking_distance_weighted = rmd->breaking_distance_weighted;
	trmd->breaking_percentage_weighted = rmd->breaking_percentage_weighted;

	trmd->execute_threaded = rmd->execute_threaded;
	trmd->point_seed = rmd->point_seed;
	trmd->point_source = rmd->point_source;

	/* vgroups  XXX TODO non ascii strings ?*/
	strncpy(trmd->thresh_defgrp_name, rmd->thresh_defgrp_name, strlen(rmd->thresh_defgrp_name));
	strncpy(trmd->ground_defgrp_name, rmd->ground_defgrp_name, strlen(rmd->ground_defgrp_name));
	strncpy(trmd->inner_defgrp_name, rmd->inner_defgrp_name, strlen(rmd->inner_defgrp_name));

	/*id refs ?*/
	trmd->inner_material = rmd->inner_material;
	trmd->extra_group = rmd->extra_group;

	/* sub object group  XXX Do we keep this ?*/
	/* trmd->dm_group = rmd->dm_group;*/

	trmd->use_particle_birth_coordinates = rmd->use_particle_birth_coordinates;
}

void freeMeshIsland(FractureModifierData *rmd, MeshIsland *mi, bool remove_rigidbody)
{

	if (mi->physics_mesh) {
		mi->physics_mesh->needsFree = 1;
		mi->physics_mesh->release(mi->physics_mesh);
		mi->physics_mesh = NULL;
	}
	if (mi->rigidbody) {
		if (remove_rigidbody)
			BKE_rigidbody_remove_shard(rmd->modifier.scene, mi);
		MEM_freeN(mi->rigidbody);
		mi->rigidbody = NULL;
	}

	{
		if (mi->vertco) {
			MEM_freeN(mi->vertco);
			mi->vertco = NULL;
		}

		if (mi->vertno) {
			MEM_freeN(mi->vertno);
			mi->vertno = NULL;
		}

		if (mi->vertices) {
			MEM_freeN(mi->vertices);
			mi->vertices = NULL; /*borrowed only !!!*/
		}
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

	MEM_freeN(mi);
	mi = NULL;
}

/* mi->bb, its for volume fraction calculation.... */
float bbox_vol(BoundBox *bb);
float bbox_vol(BoundBox *bb)
{
	float x[3], y[3], z[3];

	sub_v3_v3v3(x, bb->vec[4], bb->vec[0]);
	sub_v3_v3v3(y, bb->vec[3], bb->vec[0]);
	sub_v3_v3v3(z, bb->vec[1], bb->vec[0]);

	return len_v3(x) * len_v3(y) * len_v3(z);
}
void bbox_dim(BoundBox *bb, float dim[]);
void bbox_dim(BoundBox *bb, float dim[3])
{
	float x[3], y[3], z[3];

	sub_v3_v3v3(x, bb->vec[4], bb->vec[0]);
	sub_v3_v3v3(y, bb->vec[3], bb->vec[0]);
	sub_v3_v3v3(z, bb->vec[1], bb->vec[0]);

	dim[0] = len_v3(x);
	dim[1] = len_v3(y);
	dim[2] = len_v3(z);
}

int BM_calc_center_centroid(BMesh *bm, float cent[3], int tagged);
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

static float mesh_separate_tagged(FractureModifierData *rmd, Object *ob, BMVert **v_tag, int v_count, float **startco, BMesh *bm_work, short **startno, DerivedMesh *orig_dm)
{
	BMesh *bm_new;
	BMesh *bm_old = bm_work;
	MeshIsland *mi;
	float centroid[3], dummyloc[3], rot[4], min[3], max[3], vol = 0;
	BMVert *v;
	BMIter iter;
	DerivedMesh *dm = NULL, *dmtemp = NULL;
	Shard *s;
	int i = 0;

	if (rmd->frac_mesh->cancel == 1)
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

	if (rmd->shards_to_islands || rmd->frac_mesh->shard_count < 2) {
		/* store temporary shards for each island */
		dmtemp = CDDM_from_bmesh(bm_new, true);
		s = BKE_create_fracture_shard(dmtemp->getVertArray(dmtemp), dmtemp->getPolyArray(dmtemp), dmtemp->getLoopArray(dmtemp),
		                              dmtemp->getNumVerts(dmtemp), dmtemp->getNumPolys(dmtemp), dmtemp->getNumLoops(dmtemp), true);
		s = BKE_custom_data_to_shard(s, dmtemp);
		BLI_addtail(&rmd->islandShards, s);

		dmtemp->needsFree = 1;
		dmtemp->release(dmtemp);
		dmtemp = NULL;
	}

	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
		/* eliminate centroid in vertex coords */
		sub_v3_v3(v->co, centroid);
	}

	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
	BLI_addtail(&rmd->meshIslands, mi);

	mi->thresh_weight = 0;
	mi->vertices = v_tag;
	mi->vertco = *startco;
	mi->vertno = *startno;
	zero_v3(mi->start_co);

	BM_mesh_normals_update(bm_new);
	BM_mesh_minmax(bm_new, min, max, false);
	dm = CDDM_from_bmesh(bm_new, true);
	BM_mesh_free(bm_new);
	bm_new = NULL;

	mi->physics_mesh = dm;
	mi->vertex_count = v_count;

	mi->vertex_indices = MEM_mallocN(sizeof(int) * mi->vertex_count, "mi->vertex_indices");
	for (i = 0; i < mi->vertex_count; i++) {
		mi->vertex_indices[i] = mi->vertices[i]->head.index;
	}

	/* copy fixed normals to physicsmesh too, for convert to objects */
	if (rmd->fix_normals) {
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

	copy_v3_v3(mi->centroid, centroid);
	mat4_to_loc_quat(dummyloc, rot, ob->obmat);
	copy_v3_v3(mi->rot, rot);
	mi->bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(mi->bb, min, max);
	mi->participating_constraints = NULL;
	mi->participating_constraint_count = 0;

	vol = bbox_vol(mi->bb);
	if (vol > rmd->max_vol) {
		rmd->max_vol = vol;
	}

	mi->rigidbody = NULL;
	mi->vertices_cached = NULL;

	mi->rigidbody = BKE_rigidbody_create_shard(rmd->modifier.scene, ob, mi);
	BKE_rigidbody_calc_shard_mass(ob, mi, orig_dm);


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

void mesh_separate_loose_partition(FractureModifierData *rmd, Object *ob, BMesh *bm_work, BMVert **orig_work, DerivedMesh *dm);
void mesh_separate_loose_partition(FractureModifierData *rmd, Object *ob, BMesh *bm_work, BMVert **orig_work, DerivedMesh *dm)
{
	int i, tag_counter = 0;
	BMEdge *e;
	BMVert *v_seed, **v_tag;
	BMWalker walker;
	int tot = 0;
	BMesh *bm_old = bm_work;
	int max_iter = bm_old->totvert;
	BMIter iter;
	float *startco;
	short *startno;


	if (max_iter > 0) {
		rmd->frac_mesh->progress_counter++;
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

			short no[3];

			v_tag = MEM_callocN(sizeof(BMVert *), "v_tag");
			startco = MEM_callocN(sizeof(float), "mesh_separate_loose->startco");
			startno = MEM_callocN(sizeof(short), "mesh_separate_loose->startno");

			BM_elem_flag_enable(v_seed, BM_ELEM_TAG);
			BM_elem_flag_enable(v_seed, BM_ELEM_INTERNAL_TAG);
			v_tag = MEM_reallocN(v_tag, sizeof(BMVert *) * (tag_counter + 1));
			v_tag[tag_counter] = orig_work[v_seed->head.index];

			startco = MEM_reallocN(startco, (tag_counter + 1) * 3 * sizeof(float));
			startco[3 * tag_counter] = v_seed->co[0];
			startco[3 * tag_counter + 1] = v_seed->co[1];
			startco[3 * tag_counter + 2] = v_seed->co[2];

			startno = MEM_reallocN(startno, (tag_counter + 1) * 3 * sizeof(short));

			find_normal(dm, rmd->nor_tree, v_seed->co, no);
			startno[3 * tag_counter] = no[0];
			startno[3 * tag_counter + 1] = no[1];
			startno[3 * tag_counter + 2] = no[2];


			tot++;
			tag_counter++;
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
				short no[3];

				BM_elem_flag_enable(e->v1, BM_ELEM_TAG);
				BM_elem_flag_enable(e->v1, BM_ELEM_INTERNAL_TAG);
				v_tag = MEM_reallocN(v_tag, sizeof(BMVert *) * (tag_counter + 1));
				v_tag[tag_counter] = orig_work[e->v1->head.index];

				startco = MEM_reallocN(startco, (tag_counter + 1) * 3 * sizeof(float));
				startco[3 * tag_counter] = e->v1->co[0];
				startco[3 * tag_counter + 1] = e->v1->co[1];
				startco[3 * tag_counter + 2] = e->v1->co[2];


				startno = MEM_reallocN(startno, (tag_counter + 1) * 3 * sizeof(short));

				find_normal(dm, rmd->nor_tree, e->v1->co, no);
				startno[3 * tag_counter] = no[0];
				startno[3 * tag_counter + 1] = no[1];
				startno[3 * tag_counter + 2] = no[2];

				tot++;
				tag_counter++;
			}
			if (!BM_elem_flag_test(e->v2, BM_ELEM_TAG) && !BM_elem_flag_test(e->v2, BM_ELEM_INTERNAL_TAG)) {
				short no[3];

				BM_elem_flag_enable(e->v2, BM_ELEM_TAG);
				BM_elem_flag_enable(e->v2, BM_ELEM_INTERNAL_TAG);

				v_tag = MEM_reallocN(v_tag, sizeof(BMVert *) * (tag_counter + 1));
				v_tag[tag_counter] = orig_work[e->v2->head.index];

				startco = MEM_reallocN(startco, (tag_counter + 1) * 3 * sizeof(float));
				startco[3 * tag_counter] = e->v2->co[0];
				startco[3 * tag_counter + 1] = e->v2->co[1];
				startco[3 * tag_counter + 2] = e->v2->co[2];

				startno = MEM_reallocN(startno, (tag_counter + 1) * 3 * sizeof(short));

				find_normal(dm, rmd->nor_tree, e->v2->co, no);
				startno[3 * tag_counter] = no[0];
				startno[3 * tag_counter + 1] = no[1];
				startno[3 * tag_counter + 2] = no[2];

				tot++;
				tag_counter++;
			}
		}
		BMW_end(&walker);

		/* Flush the selection to get edge/face selections matching
		 * the vertex selection */
		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_TAG);

		/* Move selection into a separate object */
		mesh_separate_tagged(rmd, ob, v_tag, tag_counter, &startco, bm_old, &startno, dm);
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

void mesh_separate_selected(BMesh **bm_work, BMesh **bm_out, BMVert **orig_work, BMVert ***orig_out1, BMVert ***orig_out2);
void mesh_separate_selected(BMesh **bm_work, BMesh **bm_out, BMVert **orig_work, BMVert ***orig_out1, BMVert ***orig_out2)
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

void halve(FractureModifierData *rmd, Object *ob, int minsize, BMesh **bm_work, BMVert ***orig_work, bool separated, DerivedMesh *dm);

void halve(FractureModifierData *rmd, Object *ob, int minsize, BMesh **bm_work, BMVert ***orig_work, bool separated, DerivedMesh *dm)
{

	int half;
	int i = 0, new_count = 0;
	BMIter iter;
	BMVert **orig_old = *orig_work, **orig_new, **orig_mod;
	BMVert *v;
	BMesh *bm_old = *bm_work;
	BMesh *bm_new = NULL;
	separated = false;

	if (rmd->frac_mesh->cancel == 1) {
		return;
	}

	bm_new = BM_mesh_create(&bm_mesh_allocsize_default);
	{
		{
			BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);
		}

		{
			half = bm_old->totvert / 2;
			BM_ITER_MESH (v, &iter, bm_old, BM_VERTS_OF_MESH) {
				if (i >= half) {
					break;
				}
				BM_elem_select_set(bm_old, (BMElem *)v, true);
				i++;
			}
		}

		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_SELECT);
		select_linked(&bm_old);

		new_count = bm_old->totvertsel;
		printf("Halving...%d => %d %d\n", bm_old->totvert, new_count, bm_old->totvert - new_count);

		orig_new = MEM_callocN(sizeof(BMVert *) * new_count, "orig_new");
		orig_mod = MEM_callocN(sizeof(BMVert *) * bm_old->totvert - new_count, "orig_mod");
		mesh_separate_selected(&bm_old, &bm_new, orig_old, &orig_new, &orig_mod);
	}

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

void mesh_separate_loose(FractureModifierData *rmd, Object *ob, DerivedMesh *dm);
void mesh_separate_loose(FractureModifierData *rmd, Object *ob, DerivedMesh *dm)
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

static void connect_meshislands(FractureModifierData *rmd, MeshIsland *mi1, MeshIsland *mi2, int con_type, float thresh)
{
	int con_found = false;
	RigidBodyShardCon *con, *rbsc;
	bool ok = mi1 && mi1->rigidbody;
	ok = ok && mi2 && mi2->rigidbody;

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
		if (rmd->use_constraints) {
			rbsc = BKE_rigidbody_create_shard_constraint(rmd->modifier.scene, con_type);
			rbsc->mi1 = mi1;
			rbsc->mi2 = mi2;
			if (thresh == 0) {
				rbsc->flag &= ~RBC_FLAG_USE_BREAKING;
			}

			rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;

			if ((mi1->particle_index != -1) && (mi2->particle_index != -1) &&
			    (mi1->particle_index == mi2->particle_index))
			{
				if (rmd->cluster_count > 1) {
					rbsc->breaking_threshold = rmd->cluster_breaking_threshold;
				}
				else {
					rbsc->breaking_threshold = thresh;
				}
			}
			else {
				rbsc->breaking_threshold = thresh;
			}

			if (rmd->thresh_defgrp_name[0]) {
				/* modify maximum threshold by minimum weight */
				rbsc->breaking_threshold = thresh * MIN2(mi1->thresh_weight, mi2->thresh_weight);
			}

			BLI_addtail(&rmd->meshConstraints, rbsc);

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
	}
}

static void search_centroid_based(FractureModifierData *rmd, MeshIsland *mi, MeshIsland **meshIslands, KDTree **combined_tree)
{
	int r = 0, limit = 0, i = 0;
	KDTreeNearest *n3 = NULL;
	float dist, obj_centr[3];

	limit = rmd->constraint_limit;
	dist = rmd->contact_dist;

	mul_v3_m4v3(obj_centr, rmd->origmat, mi->centroid);

	r = BLI_kdtree_range_search(*combined_tree, obj_centr, &n3, dist);

	/* use centroid dist based approach here, together with limit */
	for (i = 0; i < r; i++) {
		MeshIsland *mi2 = meshIslands[(n3 + i)->index];
		if ((mi != mi2) && (mi2 != NULL)) {
			float thresh = rmd->breaking_threshold;
			int con_type = RBC_TYPE_FIXED;

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
	int i = 0, islands = 0;

	islands = BLI_countlist(&rmd->meshIslands);
	*mesh_islands = MEM_reallocN(*mesh_islands, islands * sizeof(MeshIsland *));
	for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
		(*mesh_islands)[i] = mi;
		i++;
	}

	*combined_tree = BLI_kdtree_new(islands);
	for (i = 0; i < islands; i++) {
		float obj_centr[3];
		mul_v3_m4v3(obj_centr, rmd->origmat, (*mesh_islands)[i]->centroid);
		BLI_kdtree_insert(*combined_tree, i, obj_centr);
	}

	BLI_kdtree_balance(*combined_tree);

	return islands;
}

static void create_constraints(FractureModifierData *rmd)
{
	KDTree *centroid_tree = NULL;
	MeshIsland **mesh_islands = MEM_mallocN(sizeof(MeshIsland *), "mesh_islands");
	int count, i = 0;

	if (rmd->visible_mesh && rmd->contact_dist == 0.0f) {
		/* extend contact dist to bbox max dimension here, in case we enter 0 */
		float min[3], max[3], dim[3];
		BoundBox *bb = BKE_boundbox_alloc_unit();
		BM_mesh_minmax(rmd->visible_mesh, min, max, 0);
		BKE_boundbox_init_from_minmax(bb, min, max);
		bbox_dim(bb, dim);
		rmd->contact_dist = MAX3(dim[0], dim[1], dim[2]);
		MEM_freeN(bb);
	}

	count = prepareConstraintSearch(rmd, &mesh_islands, &centroid_tree);

	for (i = 0; i < count; i++) {
		search_centroid_based(rmd, mesh_islands[i], mesh_islands, &centroid_tree);
	}

	if (centroid_tree != NULL) {
		BLI_kdtree_free(centroid_tree);
		centroid_tree = NULL;
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
					v->flag &= ~ME_VERT_TMP_TAG;
				}
			}
		}
	}
}

static DerivedMesh *createCache(FractureModifierData *rmd, Object *ob, DerivedMesh *origdm)
{
	MeshIsland *mi;
	DerivedMesh *dm;
	MVert *verts;
	MDeformVert *dvert = NULL;
	int vertstart = 0;
	const int thresh_defgrp_index = defgroup_name_index(ob, rmd->thresh_defgrp_name);
	const int ground_defgrp_index = defgroup_name_index(ob, rmd->ground_defgrp_name);

	if (rmd->dm && !rmd->shards_to_islands && (rmd->dm->getNumPolys(rmd->dm) > 0)) {
		dm = CDDM_copy(rmd->dm);
	}
	else if (rmd->visible_mesh && (rmd->visible_mesh->totface > 0) && BLI_countlist(&rmd->meshIslands) > 1) {
		dm = CDDM_from_bmesh(rmd->visible_mesh, true);
	}

	else if (origdm != NULL) {
		dm = CDDM_copy(origdm);
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
	for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
		int i = 0;
		if (mi->vertices_cached) {
			MEM_freeN(mi->vertices_cached);
			mi->vertices_cached = NULL;
		}

		if (rmd->thresh_defgrp_name[0]) {
			mi->thresh_weight = 0;
		}

		mi->vertices_cached = MEM_mallocN(sizeof(MVert *) * mi->vertex_count, "mi->vertices_cached");
		if (rmd->dm != NULL && !rmd->shards_to_islands) {
			for (i = 0; i < mi->vertex_count; i++) {
				mi->vertices_cached[i] = verts + vertstart + i;

				/* sum up vertexweights and divide by vertcount to get islandweight*/
				if (dvert && (dvert + vertstart + i)->dw && rmd->thresh_defgrp_name[0]) {
					float vweight = defvert_find_weight(dvert + vertstart + i, thresh_defgrp_index);
					mi->thresh_weight += vweight;
				}

				if (dvert && (dvert + vertstart + i)->dw && rmd->ground_defgrp_name[0]) {
					float gweight = defvert_find_weight(dvert + vertstart + i, ground_defgrp_index);
					mi->ground_weight += gweight;
				}

				if (mi->vertno != NULL && rmd->fix_normals) {
					float no[3];
					short sno[3];
					no[0] = mi->vertno[i * 3];
					no[1] = mi->vertno[i * 3 + 1];
					no[2] = mi->vertno[i * 3 + 2];
					normal_float_to_short_v3(sno, no);
					copy_v3_v3_short(mi->vertices_cached[i]->no, sno);
				}
			}

			vertstart += mi->vertex_count;
		}
		else {  /* halving case... */
			for (i = 0; i < mi->vertex_count; i++) {
				float no[3];
				int index = mi->vertex_indices[i];
				if (index >= 0 && index <= rmd->visible_mesh->totvert) {
					mi->vertices_cached[i] = verts + index;
				}
				else {
					mi->vertices_cached[i] = NULL;
				}

				if (dvert && (dvert + index)->dw && rmd->thresh_defgrp_name[0]) {
					float vweight = defvert_find_weight(dvert + index, thresh_defgrp_index);
					mi->thresh_weight += vweight;
				}

				if (dvert && (dvert + index)->dw && rmd->ground_defgrp_name[0]) {
					float gweight = defvert_find_weight(dvert + index, ground_defgrp_index);
					mi->ground_weight += gweight;
				}


				if (mi->vertno != NULL && rmd->fix_normals) {
					short sno[3];
					no[0] = mi->vertno[i * 3];
					no[1] = mi->vertno[i * 3 + 1];
					no[2] = mi->vertno[i * 3 + 2];
					normal_float_to_short_v3(sno, no);
					copy_v3_v3_short(mi->vertices_cached[i]->no, sno);
				}
			}
		}

		if (mi->vertex_count > 0) {
			mi->thresh_weight /= mi->vertex_count;
			mi->ground_weight /= mi->vertex_count;
		}

		if (mi->rigidbody != NULL) {
			mi->rigidbody->type = mi->ground_weight > 0.5f ? RBO_TYPE_PASSIVE : RBO_TYPE_ACTIVE;
		}

		/* use fallback over inner material*/
		fill_vgroup(rmd, dm, dvert, ob);
	}

	return dm;
}

void refresh_customdata_image(Mesh *me, CustomData *pdata, int totface)
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

				if (tf->tpage && tf->tpage->id.us == 0) {
					tf->tpage->id.us = 1;
				}
			}
		}
	}
}

/* inline face center calc here */
void DM_face_calc_center_mean(DerivedMesh *dm, MPoly *mp, float r_cent[3]);
void DM_face_calc_center_mean(DerivedMesh *dm, MPoly *mp, float r_cent[3])
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

void make_face_pairs(FractureModifierData *fmd, DerivedMesh *dm);
void make_face_pairs(FractureModifierData *fmd, DerivedMesh *dm)
{
	/* make kdtree of all faces of dm, then find closest face for each face*/
	MPoly *mp = NULL;
	MPoly *mpoly = dm->getPolyArray(dm);
	int totpoly = dm->getNumPolys(dm);
	KDTree *tree = BLI_kdtree_new(totpoly);
	int i = 0;

	for (i = 0, mp = mpoly; i < totpoly; mp++, i++) {
		float co[3];
		DM_face_calc_center_mean(dm, mp, co);
		BLI_kdtree_insert(tree, i, co);
	}

	BLI_kdtree_balance(tree);

	/*now find pairs of close faces*/

	for (i = 0, mp = mpoly; i < totpoly; mp++, i++)
	{
		int index = -1, j = 0, r = 0;
		KDTreeNearest *n;
		float co[3];

		DM_face_calc_center_mean(dm, mp, co);
		r = BLI_kdtree_range_search(tree, co, &n, fmd->autohide_dist * 4);
		/*2nd nearest means not ourselves...*/
		if (r == 0)
			continue;

		index = n[0].index;
		while ((j < r) && i == index) {
			index = n[j].index;
			j++;
		}

		if (!BLI_ghash_haskey(fmd->face_pairs, SET_INT_IN_POINTER(index))) {
			BLI_ghash_insert(fmd->face_pairs, SET_INT_IN_POINTER(i), SET_INT_IN_POINTER(index));
		}

		if (n != NULL) {
			MEM_freeN(n);
		}
	}

	BLI_kdtree_free(tree);
}

DerivedMesh *do_autoHide(FractureModifierData *fmd, DerivedMesh *dm);
DerivedMesh *do_autoHide(FractureModifierData *fmd, DerivedMesh *dm)
{
	float f_centr[3], f_centr_other[3];

	int totpoly = dm->getNumPolys(dm);
	int i = 0, other = 0;
	BMesh *bm = DM_to_bmesh(dm, true);
	DerivedMesh *result;
	BMFace **faces = MEM_mallocN(sizeof(BMFace *), "faces");
	int del_faces = 0;

	BM_mesh_elem_index_ensure(bm, BM_FACE);
	BM_mesh_elem_table_ensure(bm, BM_FACE);
	BM_mesh_elem_toolflags_ensure(bm);

	for (i = 0; i < totpoly; i++) {
		BMFace *f1, *f2;
		other = GET_INT_FROM_POINTER(BLI_ghash_lookup(fmd->face_pairs, SET_INT_IN_POINTER(i)));

		if (other == i)
		{
			continue;
		}

		f1 = BM_face_at_index_find(bm, i);
		f2 = BM_face_at_index_find(bm, other);

		if ((f1 == NULL) || (f2 == NULL)) {
			continue;
		}

		BM_face_calc_center_mean(f1, f_centr);
		BM_face_calc_center_mean(f2, f_centr_other);


		if (len_squared_v3v3(f_centr, f_centr_other) < fmd->autohide_dist && f1 != f2) {
			faces = MEM_reallocN(faces, sizeof(BMFace *) * (del_faces + 2));
			faces[del_faces] = f1;
			faces[del_faces + 1] = f2;
			del_faces += 2;
		}
	}

	for (i = 0; i < del_faces; i++) {
		BMFace *f = faces[i];
		if (f->l_first->e != NULL) { /* a lame check.... */
			BM_face_kill(bm, f);
		}
	}

	result = CDDM_from_bmesh(bm, true);
	BM_mesh_free(bm);
	MEM_freeN(faces);

	return result;
}



DerivedMesh *doSimulate(FractureModifierData *fmd, Object *ob, DerivedMesh *dm, DerivedMesh *orig_dm)
{
	bool exploOK = false; /* doFracture */
	double start;

	if ((fmd->refresh) || (fmd->refresh_constraints && !fmd->execute_threaded) ||
	    (fmd->refresh_constraints && fmd->execute_threaded && fmd->frac_mesh && fmd->frac_mesh->running == 0))
	{
		/* if we changed the fracture parameters */

		freeData((ModifierData *)fmd);

		/* 2 cases, we can have a visible mesh or a cached visible mesh, the latter primarily when loading blend from file or using halving */

		/* free cached mesh in case of "normal refracture here if we have a visible mesh, does that mean REfracture ?*/
		if (fmd->visible_mesh != NULL && !fmd->shards_to_islands && fmd->frac_mesh->shard_count > 0 && fmd->refresh) {
			if (fmd->visible_mesh_cached) {
				fmd->visible_mesh_cached->needsFree = 1;
				fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
			}
			fmd->visible_mesh_cached = NULL;
		}

		if (fmd->refresh)
		{
			copy_m4_m4(fmd->origmat, ob->obmat);

			/* refracture, convert the fracture shards to new meshislands here *
			 * shards = fracture datastructure
			 * meshisland = simulation datastructure */
			if (fmd->frac_mesh && fmd->frac_mesh->shard_count > 0 && fmd->dm && fmd->dm->numVertData > 0 &&
			    !fmd->shards_to_islands && !fmd->dm_group)
			{
				Shard *s;
				MeshIsland *mi; /* can be created without shards even, when using fracturemethod = NONE (re-using islands)*/

				int i, j, vertstart = 0, polystart = 0;

				float dummyloc[3], rot[4];
				MDeformVert *dvert = fmd->dm->getVertDataArray(fmd->dm, CD_MDEFORMVERT);
				MDeformVert *ivert;
				const int thresh_defgrp_index = defgroup_name_index(ob, fmd->thresh_defgrp_name);
				const int ground_defgrp_index = defgroup_name_index(ob, fmd->ground_defgrp_name);

				/*XXX should rename this... this marks the fracture case, to distinguish from halving case */
				fmd->explo_shared = true;

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


				for (i = 0; i < fmd->frac_mesh->shard_count; i++) {
					MVert *mv, *verts, *mverts;
					int totvert, k;

					s = fmd->frac_mesh->shard_map[i];
					if (s->totvert == 0) {
						continue;
					}

					fmd->frac_mesh->progress_counter++;

					mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
					BLI_addtail(&fmd->meshIslands, mi);

					mi->participating_constraints = NULL;
					mi->participating_constraint_count = 0;

					mi->thresh_weight = 0;
					mi->vertices_cached = MEM_mallocN(sizeof(MVert *) * s->totvert, "vert_cache");
					mverts = CDDM_get_verts(fmd->visible_mesh_cached);
					for (k = 0; k < s->totvert; k++) {
						mi->vertices_cached[k] = mverts + vertstart + k;
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
					vertstart += s->totvert;
					mi->physics_mesh = BKE_shard_create_dm(s, true);
					totvert = mi->physics_mesh->numVertData;
					verts = mi->physics_mesh->getVertArray(mi->physics_mesh);
					mi->vertco = MEM_mallocN(sizeof(float) * 3 * totvert, "vertco");
					mi->vertno = MEM_mallocN(sizeof(short) * 3 * totvert, "vertno");
					for (mv = verts, j = 0; j < totvert; mv++, j++) {
						short no[3];

						mi->vertco[j * 3] = mv->co[0];
						mi->vertco[j * 3 + 1] = mv->co[1];
						mi->vertco[j * 3 + 2] = mv->co[2];

						/* either take orignormals or take ones from fractured mesh */
						find_normal(orig_dm, fmd->nor_tree, mv->co, no);

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

					/*copy fixed normals to physics mesh too (needed for convert to objects)*/

					BKE_shard_calc_minmax(s);
					mi->vertex_count = s->totvert;
					copy_v3_v3(mi->centroid, s->centroid);
					mat4_to_loc_quat(dummyloc, rot, ob->obmat);
					copy_v3_v3(mi->rot, rot);

					mi->bb = BKE_boundbox_alloc_unit();
					BKE_boundbox_init_from_minmax(mi->bb, s->min, s->max);

					mi->id = s->shard_id;
					mi->particle_index = s->cluster_colors[0];
					mi->neighbor_ids = s->neighbor_ids;
					mi->neighbor_count = s->neighbor_count;

					if (mi->vertex_count > 0) {
						mi->thresh_weight /= mi->vertex_count;
						mi->ground_weight /= mi->vertex_count;
					}

					mi->rigidbody = BKE_rigidbody_create_shard(fmd->modifier.scene, ob, mi);
					BKE_rigidbody_calc_shard_mass(ob, mi, orig_dm);
					mi->vertex_indices = NULL;

					polystart += s->totpoly;

				}

				fill_vgroup(fmd, fmd->visible_mesh_cached, ivert, ob);
			}
			else {
				if (fmd->visible_mesh == NULL) {
					if (fmd->dm && fmd->shards_to_islands) {
						fmd->visible_mesh = DM_to_bmesh(fmd->dm, true);
					}
					else {
						/* split to meshislands now */
						fmd->visible_mesh = DM_to_bmesh(dm, true); /* ensures indexes automatically*/
					}

					start = PIL_check_seconds_timer();
					printf("Steps: %d \n", fmd->frac_mesh->progress_counter);
					mesh_separate_loose(fmd, ob, orig_dm);
					printf("Splitting to islands done, %g  Steps: %d \n", PIL_check_seconds_timer() - start, fmd->frac_mesh->progress_counter);
				}

				fmd->explo_shared = false;
			}

			printf("Islands: %d\n", BLI_countlist(&fmd->meshIslands));
		}

		start = PIL_check_seconds_timer();

		if ((fmd->visible_mesh != NULL && fmd->refresh && (!fmd->explo_shared)) || (fmd->visible_mesh_cached == NULL)) {
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

		if (fmd->refresh_images && fmd->visible_mesh_cached) {
			/* need to ensure images are correct after loading... */
			refresh_customdata_image(ob->data, &fmd->visible_mesh_cached->polyData,
			                         fmd->visible_mesh_cached->getNumPolys(fmd->visible_mesh_cached));
			fmd->refresh_images = false;
			DM_update_tessface_data(fmd->visible_mesh_cached);
		}

		fmd->refresh = false;
		fmd->refresh_constraints = true;

		/*HERE make a kdtree of the fractured derivedmesh,
		 * store pairs of faces (MPoly) here (will be most likely the inner faces) */
		if (fmd->face_pairs != NULL) {
			BLI_ghash_free(fmd->face_pairs, NULL, NULL);
			fmd->face_pairs = NULL;
		}

		fmd->face_pairs = BLI_ghash_int_new("face_pairs");
		make_face_pairs(fmd, fmd->visible_mesh_cached);

		if (fmd->execute_threaded) {
			/* job done */
			fmd->frac_mesh->running = 0;
		}
	}

	if (fmd->refresh_constraints) {
		start = PIL_check_seconds_timer();

		if ((fmd->visible_mesh != NULL || fmd->visible_mesh_cached != NULL)  && (fmd->use_constraints)) {
			if (fmd->visible_mesh == NULL) {    /* ugh, needed to build constraints... */
				fmd->visible_mesh = DM_to_bmesh(fmd->visible_mesh_cached, true);
				BM_mesh_elem_index_ensure(fmd->visible_mesh, BM_VERT | BM_EDGE | BM_FACE);
				BM_mesh_elem_table_ensure(fmd->visible_mesh, BM_VERT | BM_EDGE | BM_FACE);
				BM_mesh_elem_toolflags_ensure(fmd->visible_mesh);
			}
			create_constraints(fmd); /* check for actually creating the constraints inside*/

			if (fmd->visible_mesh_cached != NULL) {
				/* if we had a cached visible mesh, throw away this temp visible mesh again */
				BM_mesh_free(fmd->visible_mesh);
				fmd->visible_mesh = NULL;
			}
		}

		fmd->refresh_constraints = false;

		printf("Building constraints done, %g\n", PIL_check_seconds_timer() - start);
		printf("Constraints: %d\n", BLI_countlist(&fmd->meshConstraints));
	}

	/*XXX better rename this, it checks whether we have a valid fractured mesh */
	exploOK = !fmd->explo_shared || (fmd->explo_shared && fmd->dm && fmd->frac_mesh);

	if ((!exploOK) || (fmd->visible_mesh == NULL && fmd->visible_mesh_cached == NULL))
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

	if ((fmd->visible_mesh != NULL) && exploOK) {
		DerivedMesh *dm_final;
		/* HERE Hide facepairs closer than dist X*/

		if (fmd->autohide_dist > 0) {
			dm_final = do_autoHide(fmd, fmd->visible_mesh_cached);
		}
		else {
			dm_final = CDDM_copy(fmd->visible_mesh_cached);
		}
		return dm_final;
	}
	else if ((fmd->visible_mesh_cached != NULL) && exploOK) {
		DerivedMesh *dm_final;

		if (fmd->autohide_dist > 0) {
			dm_final = do_autoHide(fmd, fmd->visible_mesh_cached);
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
			freeData((ModifierData *)fmd);
			fmd->visible_mesh_cached = NULL;
			fmd->refresh = false;
		}

		return dm;
	}

	return dm;
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

	walk(userData, ob, (ID **)&fmd->inner_material);
	walk(userData, ob, (ID **)&fmd->extra_group);
	walk(userData, ob, (ID **)&fmd->dm_group);
}

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md))
{
	CustomDataMask dataMask = 0;
	dataMask |= CD_MASK_MDEFORMVERT;
	return dataMask;
}

static void updateDepgraph(ModifierData *md, DagForest *forest,
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
				                 DAG_RL_DATA_DATA | DAG_RL_OB_DATA, "Fracture Modifier");
			}
		}
	}
}

static void foreachObjectLink(
    ModifierData *md, Object *ob,
    void (*walk)(void *userData, Object *ob, Object **obpoin),
    void *userData)
{
	FractureModifierData *fmd = (FractureModifierData *) md;

	if (fmd->extra_group) {
		GroupObject *go;
		for (go = fmd->extra_group->gobject.first; go; go = go->next) {
			if (go->ob) {
				walk(userData, ob, &go->ob);
			}
		}
	}
}


ModifierTypeInfo modifierType_Fracture = {
	/* name */ "Fracture",
	/* structName */ "FractureModifierData",
	/* structSize */ sizeof(FractureModifierData),
	/* type */ eModifierTypeType_Constructive,                 //eModifierTypeType_OnlyDeform,
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
	/* dependsOnTime */ dependsOnTime,
	/* dependsOnNormals */ dependsOnNormals,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */ foreachIDLink,
};
