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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

#include "MEM_guardedalloc.h"

#include "BKE_cdderivedmesh.h"
#include "BKE_curve.h"
#include "BKE_customdata.h"
#include "BKE_deform.h"
#include "BKE_depsgraph.h"
#include "BKE_DerivedMesh.h"
#include "BKE_effect.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_library.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_modifier.h"
#include "BKE_multires.h"
#include "BKE_object.h"
#include "BKE_particle.h"
#include "BKE_pointcache.h"
#include "BKE_rigidbody.h"
#include "BKE_scene.h"

#include "BLI_edgehash.h"
#include "BLI_ghash.h"
#include "BLI_kdtree.h"
#include "BLI_listbase.h"
#include "BLI_math.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.h"
#include "BLI_mempool.h"
#include "BLI_path_util.h"
#include "BLI_rand.h"
#include "BLI_sort.h"
#include "BLI_string.h"
#include "BLI_threads.h"
#include "BLI_utildefines.h"

#include "DNA_curve_types.h"
#include "DNA_fracture_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_group_types.h"
#include "DNA_listBase.h"
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_particle_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"

#include "bmesh.h"
#include "../../bmesh/tools/bmesh_decimate.h" /* decimate_dissolve function */

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
static void do_island_from_shard(Object *ob, Shard* s, int i, int thresh_defgrp_index, int ground_defgrp_index, int vertstart);
static void do_island_vertex_index_map(Object *ob, GHash **vertex_index_map, int partner_index);
static void initialize_shard(Object *ob);
static void update_islands(Object *ob);

static bool thread_sentinel(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState* fs = NULL;

	if (fc == NULL)
		return true;

	fs = fc->current;

	if (fs == NULL)
		return true;

	if ((fs->frac_mesh) && (fs->frac_mesh->running == 1 || fs->frac_mesh->cancel == 1) &&
	    (fc->flag & FM_FLAG_EXECUTE_THREADED))
	{
		/* skip fracture execution when fracture job is running or cancel has been requested */
		return true;
	}

	return false;
}

static void copy_shard(Shard *t, Shard *s)
{
	t->totvert = s->totvert;
	t->totpoly = s->totpoly;
	t->totloop = s->totloop;

	t->mvert = MEM_mallocN(sizeof(MVert) * t->totvert, "shard vertices");
	t->mpoly = MEM_mallocN(sizeof(MPoly) * t->totpoly, "shard polys");
	t->mloop = MEM_mallocN(sizeof(MLoop) * t->totloop, "shard loops");
	memcpy(t->mvert,s->mvert, sizeof(MVert) * t->totvert);
	memcpy(t->mpoly,s->mpoly, sizeof(MPoly) * t->totpoly);
	memcpy(t->mloop,s->mloop, sizeof(MLoop) * t->totloop);

	CustomData_reset(&t->vertData);
	CustomData_reset(&t->loopData);
	CustomData_reset(&t->polyData);

	CustomData_add_layer(&t->vertData, CD_MDEFORMVERT, CD_DUPLICATE,
	                     CustomData_get_layer(&s->vertData, CD_MDEFORMVERT), s->totvert);
	CustomData_add_layer(&t->loopData, CD_MLOOPUV, CD_DUPLICATE,
	                     CustomData_get_layer(&s->loopData, CD_MLOOPUV), s->totloop);
	CustomData_add_layer(&t->polyData, CD_MTEXPOLY, CD_DUPLICATE,
	                     CustomData_get_layer(&s->polyData, CD_MTEXPOLY), s->totpoly);
}

FracMesh* BKE_copy_fracmesh(FracMesh* fm)
{
	FracMesh *fmesh;
	Shard* s, *t;

	fmesh = MEM_mallocN(sizeof(FracMesh), __func__);
	fmesh->shard_map.first = NULL;
	fmesh->shard_map.last = NULL;

	for (s = fm->shard_map.first; s; s = s->next)
	{
		t = MEM_callocN(sizeof(Shard), "copy_fracmesh");
		copy_shard(t, s);
		BLI_addtail(&fmesh->shard_map, t);
	}

	fmesh->shard_count = fm->shard_count;
	fmesh->cancel = 0;
	fmesh->running = 0;
	fmesh->progress_counter = 0;
	fmesh->last_shard_tree = NULL;
	fmesh->last_shards = NULL;

	return fmesh;
}

static void freeMeshIsland(Scene *scene, MeshIsland *mi)
{
	if (mi->physics_mesh) {
		mi->physics_mesh->needsFree = 1;
		mi->physics_mesh->release(mi->physics_mesh);
		mi->physics_mesh = NULL;
	}

	if (mi->rigidbody) {
		if (scene != NULL)
		{
			RigidBodyWorld *rbw = scene->rigidbody_world;
			RigidBodyShardOb *rbo = mi->rigidbody;
			if (rbo->physics_object && rbw->physics_world)
				RB_dworld_remove_body(rbw->physics_world, rbo->physics_object);
		}

		MEM_freeN(mi->rigidbody);
		mi->rigidbody = NULL;
	}

	if (mi->vertcos) {
		MEM_freeN(mi->vertcos);
		mi->vertcos = NULL;
	}

	if (mi->vertnos) {
		MEM_freeN(mi->vertnos);
		mi->vertnos = NULL;
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
		int count = mi->participating_constraint_count;
		int i = 0;
		for (i = 0; i < count; i++)
		{
			RigidBodyShardCon *con = mi->participating_constraints[i];
			con->mi1 = NULL;
			con->mi2 = NULL; //purge those as well
			if (scene)
				BKE_rigidbody_remove_shard_con(scene->rigidbody_world, con);
		}

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

static void free_meshislands(Scene *scene, ListBase* meshIslands)
{
	MeshIsland *mi;

	while (meshIslands->first) {
		mi = meshIslands->first;
		BLI_remlink(meshIslands, mi);
		freeMeshIsland(scene, mi);
		mi = NULL;
	}

	meshIslands->first = NULL;
	meshIslands->last = NULL;
}

static void free_shards(FracMesh *fm)
{
	BKE_fracmesh_free(fm, true);
	MEM_freeN(fm);
	fm = NULL;
}

//static ThreadMutex free_fracture_state_lock = BLI_MUTEX_INITIALIZER;
static void free_fracture_state(Scene *scene, FractureState *fs, bool delete_all)
{
	//BLI_mutex_lock(&free_fracture_state_lock);

	free_meshislands(scene, &fs->island_map);

	if (fs->islands) {
		MEM_freeN(fs->islands);
		fs->islands = NULL;
	}

	if (delete_all)
	{
		free_shards(fs->frac_mesh);
		if (fs->visual_mesh)
		{
			fs->visual_mesh->needsFree = 1;
			DM_release(fs->visual_mesh);
			fs->visual_mesh = NULL;
		}

		//do elsewhere FM_TODO
		//MEM_freeN(fs);
		//fs = NULL;
	}

	//BLI_mutex_unlock(&free_fracture_state_lock);
}

static void free_fracture_container(Scene *scene, FractureContainer *fc)
{
	FractureState *fs;
	while (fc->states.first)
	{
		fs = fc->states.first;
		BLI_remlink_safe(&fc->states, fs);
		free_fracture_state(scene, fs, fc->fracture_mode == MOD_FRACTURE_PREFRACTURED);
		MEM_freeN(fs);
		fs = NULL;
	}

	fc->states.first = NULL;
	fc->states.last = NULL;

	BKE_ptcache_free_list(&(fc->ptcaches));
	fc->pointcache = NULL;

	/* free effector weights */
	if (fc->effector_weights) {
		MEM_freeN(fc->effector_weights);
		fc->effector_weights = NULL;
	}

	//maybe make those temporary.... and dont store ?
	if (fc->vertex_island_map) {
		BLI_ghash_free(fc->vertex_island_map, NULL, NULL);
		fc->vertex_island_map = NULL;
	}

	if (fc->nor_tree != NULL) {
		BLI_kdtree_free(fc->nor_tree);
		fc->nor_tree = NULL;
	}

	if (fc->face_pairs != NULL) {
		BLI_ghash_free(fc->face_pairs, NULL, NULL);
		fc->face_pairs = NULL;
	}
}

static void do_cluster_count(FractureContainer *fc)
{
	FractureState *fs = fc->current;
	int k = 0;
	KDTree *tree;
	MeshIsland *mi, **seeds;
	int seed_count;

	int mi_count;
	/* zero clusters or one mean no clusters, all shards keep free */
	if (fc->cluster_count < 2) {
		return;
	}

	/*initialize cluster "colors" -> membership of meshislands to clusters, initally all shards are "free" */
	for (mi = fc->current->island_map.first; mi; mi = mi->next ) {
		mi->particle_index = -1;
	}

	mi_count = BLI_listbase_count(&fs->island_map);
	seed_count = (fc->cluster_count > mi_count ? mi_count : fc->cluster_count);
	seeds = MEM_mallocN(sizeof(MeshIsland *) * seed_count, "seeds");
	tree = BLI_kdtree_new(seed_count);

	/* pick n seed locations, randomly scattered over the object */
	for (k = 0; k < seed_count; k++) {
		int which_index = k * (int)(mi_count / seed_count);
		MeshIsland *which = (MeshIsland *)BLI_findlink(&fs->island_map, which_index);
		which->particle_index = k;
		BLI_kdtree_insert(tree, k, which->centroid);
		seeds[k] = which;
	}

	BLI_kdtree_balance(tree);


	/* assign each shard to its closest center */
	for (mi = fs->island_map.first; mi; mi = mi->next ) {
		KDTreeNearest n;
		int index;

		index = BLI_kdtree_find_nearest(tree, mi->centroid, &n);
		mi->particle_index = seeds[index]->particle_index;
	}

	BLI_kdtree_free(tree);
	MEM_freeN(seeds);
}

static void do_cluster_group(Object* obj)
{
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	KDTree *tree;
	MeshIsland *mi;
	int seed_count;

	/*initialize cluster "colors" -> membership of meshislands to clusters, initally all shards are "free" */
	for (mi = fs->island_map.first; mi; mi = mi->next ) {
		mi->particle_index = -1;
	}

	seed_count = BLI_listbase_count(&fc->cluster_group->gobject);
	if (seed_count > 0)
	{
		GroupObject* go;
		int i = 0;
		tree = BLI_kdtree_new(seed_count);
		for (i = 0, go = fc->cluster_group->gobject.first; go; i++, go = go->next)
		{
			BLI_kdtree_insert(tree, i, go->ob->loc);
		}

		BLI_kdtree_balance(tree);

		/* assign each shard to its closest center */
		for (mi = fs->island_map.first; mi; mi = mi->next ) {
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

static void do_clusters(Object* obj)
{
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;

	/*grow clusters from all meshIslands */
	if (fc->cluster_group)
	{
		do_cluster_group(obj);
	}
	else
	{
		do_cluster_count(fc);
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

static void points_from_verts(Object **ob, int totobj, Scene *scene,Object *obj, float thresh, FracPointCloud *points)
{
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;
	int v, o, pt = points->totpoints;
	float co[3];

	for (o = 0; o < totobj; o++) {
		if (ob[o]->type == OB_MESH) {
			/* works for mesh objects only, curves, surfaces, texts have no verts */
			float imat[4][4];
			DerivedMesh *d;
			MVert *vert;

			d = mesh_get_derived_final(scene, ob[o], 0);
			invert_m4_m4(imat, obj->obmat);
			vert = d->getVertArray(d);

			for (v = 0; v < d->getNumVerts(d); v++) {
				if (BLI_frand() < thresh) {
					points->points = MEM_reallocN((*points).points, (pt + 1) * sizeof(FracPoint));

					copy_v3_v3(co, vert[v].co);


					if (fc->point_source & MOD_FRACTURE_EXTRA_VERTS) {
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

static void points_from_particles(Object **ob, int totobj, Scene *scene, Object *obj,
                                  float thresh, FracPointCloud *points)
{
	int o, p, pt = points->totpoints;
	ParticleSystemModifierData *psmd;
	ParticleData *pa;
	ParticleSimulationData sim = {NULL};
	ParticleKey birth;
	ModifierData *mod;
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;

	for (o = 0; o < totobj; o++) {
		for (mod = ob[o]->modifiers.first; mod; mod = mod->next) {
			if (mod->type == eModifierType_ParticleSystem) {
				float imat[4][4];
				psmd = (ParticleSystemModifierData *)mod;
				sim.scene = scene;
				sim.ob = ob[o];
				sim.psys = psmd->psys;
				sim.psmd = psmd;
				invert_m4_m4(imat, obj->obmat);

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
						if ((fc->flag & FM_FLAG_USE_PARTICLE_BIRTH_COORDS) && fc->fracture_mode == MOD_FRACTURE_PREFRACTURED)
						{
							psys_get_birth_coords(&sim, pa, &birth, 0, 0);
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

static void points_from_greasepencil(Object **ob, int totobj, Object* obj, float thresh, FracPointCloud *points)
{
	bGPDlayer *gpl;
	bGPDframe *gpf;
	bGPDstroke *gps;
	int pt = points->totpoints, p, o;

	for (o = 0; o < totobj; o++) {
		if ((ob[o]->gpd) && (ob[o]->gpd->layers.first)) {
			float imat[4][4];
			invert_m4_m4(imat, obj->obmat);
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

static FracPointCloud get_points_global(Scene* scene, Object *ob, ShardID id)
{
	FracPointCloud points;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;

	/* global settings, for first fracture only, or global secondary and so on fracture, apply to entire fracmesh */
	int totgroup = 0;
	Object **go = MEM_mallocN(sizeof(Object *), "groupobjects");
	float thresh = (float)fc->percentage / 100.0f;
	float min[3], max[3];
	int i;

	points.points = MEM_mallocN(sizeof(FracPoint), "points");
	points.totpoints = 0;

	if (fc->point_source & (MOD_FRACTURE_EXTRA_PARTICLES | MOD_FRACTURE_EXTRA_VERTS)) {
		if (((fc->point_source & MOD_FRACTURE_OWN_PARTICLES) && (fc->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
		    ((fc->point_source & MOD_FRACTURE_OWN_VERTS) && (fc->point_source & MOD_FRACTURE_EXTRA_VERTS)) ||
		    ((fc->point_source & MOD_FRACTURE_GREASEPENCIL) && (fc->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
		    ((fc->point_source & MOD_FRACTURE_GREASEPENCIL) && (fc->point_source & MOD_FRACTURE_EXTRA_VERTS)))
		{
			go = MEM_reallocN(go, sizeof(Object *) * (totgroup + 1));
			go[totgroup] = ob;
			totgroup++;
		}

		totgroup = getGroupObjects(fc->extra_group, &go, totgroup);
	}
	else {
		totgroup = 1;
		go[0] = ob;
	}

	if (fc->point_source & (MOD_FRACTURE_OWN_PARTICLES | MOD_FRACTURE_EXTRA_PARTICLES)) {
		points_from_particles(go, totgroup, scene, ob, thresh, &points);
	}

	if (fc->point_source & (MOD_FRACTURE_OWN_VERTS | MOD_FRACTURE_EXTRA_VERTS)) {
		points_from_verts(go, totgroup, scene, ob, thresh, &points);
	}

	if (fc->point_source & MOD_FRACTURE_GREASEPENCIL && !(fs->flag & FM_FLAG_USE_GREASEPENCIL_EDGES)) {
		points_from_greasepencil(go, totgroup, ob, thresh, &points);
	}


	/* local settings, apply per shard!!! Or globally too first. */
	if (fc->point_source & MOD_FRACTURE_UNIFORM)
	{
		int count = fc->shard_count;
		INIT_MINMAX(min, max);
		//ensure ob->derivedFinal ?
		BKE_get_shard_minmax(fs->frac_mesh, id, min, max); //id 0 should be entire mesh
		printf("min, max: (%f %f %f), (%f %f %f)\n", min[0], min[1], min[2], max[0], max[1], max[2]);

		if (fc->frac_algorithm == MOD_FRACTURE_BISECT_FAST || fc->frac_algorithm == MOD_FRACTURE_BISECT_FAST_FILL ||
		    fc->frac_algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
			/* XXX need double amount of shards, because we create 2 islands at each cut... so this matches the input count */
			if ((count > 1) || fc->frac_algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
				count--;
				count *= 2;
			}
		}

		BLI_srandom(fc->point_seed);
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

static void do_splinters(Object* ob, FracPointCloud points, float(*mat)[4][4])
{
	float imat[4][4];
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->raw_mesh;

	unit_m4(*mat);

	/*splinters... just global axises and a length, for rotation rotate the object */
	if (fc->splinter_axis & MOD_FRACTURE_SPLINTER_X)
	{
		(*mat)[0][0] *= fc->splinter_length;
	}
	if (fc->splinter_axis & MOD_FRACTURE_SPLINTER_Y)
	{
		(*mat)[1][1] *= fc->splinter_length;
	}
	if (fc->splinter_axis & MOD_FRACTURE_SPLINTER_Z)
	{
		(*mat)[2][2] *= fc->splinter_length;
	}

	if ((fc->splinter_axis & MOD_FRACTURE_SPLINTER_X) ||
		(fc->splinter_axis & MOD_FRACTURE_SPLINTER_Y) ||
		(fc->splinter_axis & MOD_FRACTURE_SPLINTER_Z))
	{
		int i = 0;
		MVert* mvert = dm->getVertArray(dm), *mv;
		invert_m4_m4(imat, *mat);

		for (i = 0; i < points.totpoints; i++)
		{
			mul_m4_v3(imat, points.points[i].co);
		}

		for (i = 0, mv = mvert; i < dm->getNumVerts(dm); i++, mv++)
		{
			mul_m4_v3(imat, mv->co);
		}
	}
}

static short do_materials(Object* obj)
{
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;
	short mat_index = 0;

	if (fc->inner_material) {
		/* assign inner material as secondary mat to ob if not there already */
		mat_index = find_material_index(obj, fc->inner_material);
		if (mat_index == 0) {
			object_add_material_slot(obj);
			assign_material(obj, fc->inner_material, obj->totcol, BKE_MAT_ASSIGN_OBDATA);
		}

		/* get index again */
		mat_index = find_material_index(obj, fc->inner_material);
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

			fc->inner_material = mat_inner;
		}
		else if (*totmat == 1)
		{
			char* matname = BLI_strdupcat(name, "_Inner");
			Material* mat_inner = find_material(matname);
			object_add_material_slot(obj);
			assign_material(obj, mat_inner, obj->totcol, BKE_MAT_ASSIGN_OBDATA);
			MEM_freeN(matname);
			matname = NULL;

			fc->inner_material = mat_inner;
		}
		else /*use 2nd material slot*/
		{
			Material* mat_inner = give_current_material(obj, 2);

			fc->inner_material = mat_inner;
		}

		mat_index = 2;
	}

	return mat_index;
}

static void cleanup_splinters(Object* ob, float mat[4][4])
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->current->visual_mesh;

	if ((fc->splinter_axis & MOD_FRACTURE_SPLINTER_X) ||
		(fc->splinter_axis & MOD_FRACTURE_SPLINTER_Y) ||
		(fc->splinter_axis & MOD_FRACTURE_SPLINTER_Z))
	{
		int i = 0;
		MVert* mvert = dm->getVertArray(dm), *mv;
		for (i = 0, mv = mvert; i < dm->getNumVerts(dm); i++, mv++)
		{
			mul_m4_v3(mat, mv->co);
		}
	}
}

static void do_fracture(Scene *scene, Object *obj, ShardID id)
{
	/* dummy point cloud, random */
	FracPointCloud points;
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;

	points = get_points_global(scene, obj, id);

	if (points.totpoints > 0 || (fc->flag & FM_FLAG_USE_GREASEPENCIL_EDGES)) {
		short mat_index = 0;
		float mat[4][4];

		/*splinters... just global axises and a length, for rotation rotate the object */
		do_splinters(obj, points, &mat);

		mat_index = do_materials(obj);
		mat_index = mat_index > 0 ? mat_index - 1 : mat_index;

		if (points.totpoints > 0) {
			BKE_fracture_shard_by_points(obj, id, &points, mat_index, mat);
		}

		if (fc->point_source & MOD_FRACTURE_GREASEPENCIL && (fc->flag & FM_FLAG_USE_GREASEPENCIL_EDGES)) {
			BKE_fracture_shard_by_greasepencil(obj, mat_index, mat);
		}

		if (fc->frac_algorithm == MOD_FRACTURE_BOOLEAN && fc->cutter_group != NULL) {
			BKE_fracture_shard_by_planes(obj, mat_index, mat);
		}

		/* job has been cancelled, throw away all data FM_TODO, really ?*/
		if (fs->frac_mesh->cancel == 1)
		{
			fs->frac_mesh->running = 0;
			//free_fracture_state(scene, fs, true);
			MEM_freeN(points.points);
			return;
		}

		//watch it when overwriting this... free it better before
		if (fs->visual_mesh)
		{
			fs->visual_mesh->needsFree = 1;
			DM_release(fs->visual_mesh);
			fs->visual_mesh = NULL;
		}

		fs->visual_mesh = BKE_fracture_create_dm(obj, fs->frac_mesh, true);

		cleanup_splinters(obj, mat);
		fc->flag &= ~FM_FLAG_RESET_SHARDS;
	}
	MEM_freeN(points.points);
}

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

static Shard* do_shard_to_island(Object* ob, BMesh* bm_new)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	FracMesh *fm = fs->frac_mesh;

	float mat[4][4];
	DerivedMesh *dmtemp;
	Shard *s;

	//mat is splintermatrix ! here we dont use splinters, so take unit_m4
	unit_m4(mat);

	/* store temporary shards for each island */
	dmtemp = CDDM_from_bmesh(bm_new, true);
	s = BKE_create_fracture_shard(dmtemp->getVertArray(dmtemp), dmtemp->getPolyArray(dmtemp), dmtemp->getLoopArray(dmtemp),
								  dmtemp->getNumVerts(dmtemp), dmtemp->getNumPolys(dmtemp), dmtemp->getNumLoops(dmtemp), true);
	s = BKE_custom_data_to_shard(s, dmtemp);

	add_shard(fm, s, mat);

	dmtemp->needsFree = 1;
	dmtemp->release(dmtemp);
	dmtemp = NULL;

	return s;
}

static void do_rigidbody(Object *ob, MeshIsland* mi, short rb_type)
{
	mi->rigidbody = NULL;
	mi->rigidbody = BKE_rigidbody_create_shard(ob, mi);
	mi->rigidbody->type = rb_type;
	mi->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
	//BKE_rigidbody_calc_shard_mass(ob, mi);
}

static void do_fix_normals(Object* ob, MeshIsland *mi)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

	/* copy fixed normals to physicsmesh too, for convert to objects */
	if (fc->flag & FM_FLAG_FIX_NORMALS) {
		MVert *verts, *mv;
		int j = 0, totvert = 0;
		totvert = mi->vertex_count;
		verts = mi->physics_mesh->getVertArray(mi->physics_mesh);
		for (mv = verts, j = 0; j < totvert; mv++, j++) {
			copy_v3_v3_short(mv->no, mi->vertnos[j]);
		}
	}
}

static void do_calc_volume(FractureContainer *fc, MeshIsland *mi)
{
	float vol;

	if (mi->bb)
	{
		vol = bbox_vol(mi->bb);
		if (vol > fc->max_vol) {
			fc->max_vol = vol;
		}
	}
}

#if 0
static float do_setup_meshisland(FractureModifierData *fmd, Object *ob, int totvert, float centroid[3],
                                 BMVert **verts, float *vertco, short *vertno, BMesh **bm_new, DerivedMesh *orig_dm)
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

		if (fmd->fracture->current_mi_entry) {
			MeshIslandSequence *prev = fmd->fracture->current_mi_entry->prev;
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

	mi->thresh_weight = 0;
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
	copy_v3_v3(mi->rot, rot);


	mi->bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(mi->bb, min, max);
	mi->participating_constraints = NULL;
	mi->participating_constraint_count = 0;

	vol = bbox_vol(mi->bb);
	if (vol > fmd->fracture->max_vol) {
		fmd->fracture->max_vol = vol;
	}

	mi->vertices_cached = NULL;

	rb_type = do_vert_index_map(fmd, mi);
	i = BLI_listbase_count(&fmd->fracture->meshIslands);
	do_rigidbody(fmd, mi, ob, orig_dm, rb_type, i);

	mi->start_frame = fmd->modifier.scene->rigidbody_world->pointcache->startframe;

	BLI_addtail(&fmd->fracture->meshIslands, mi);

	return vol;
}
#endif

static void mesh_separate_tagged(Object *ob, BMesh *bm_work)
{
	BMesh *bm_new;
	BMesh *bm_old = bm_work;
	float centroid[3];

	BMVert *v;
	BMIter iter;
	Shard *s;
	FractureContainer* fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;

	const int thresh_defgrp_index = defgroup_name_index(ob, fc->thresh_defgrp_name);
	const int ground_defgrp_index = defgroup_name_index(ob, fc->ground_defgrp_name);

	if (fs->frac_mesh->cancel == 1)
		return;

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

	s = do_shard_to_island(ob, bm_new);

	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
		/* eliminate centroid in vertex coords */
		sub_v3_v3(v->co, centroid);
	}

	//calculate volume here optionally too
	//need to distinguish between prehalving and posthalving, omit meshislands in prehalving, or better...
	//create only after post halving....
	do_island_from_shard(ob, s, 0, thresh_defgrp_index, ground_defgrp_index, 0);

	/* deselect loose data - this used to get deleted,
	 * we could de-select edges and verts only, but this turns out to be less complicated
	 * since de-selecting all skips selection flushing logic */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
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

static void handle_vert(Object* ob, BMVert* vert, BMVert** orig_work,
                        float **startco, short **startno, BMVert*** v_tag, int *tot, int *tag_counter)
{
	/* treat the specified vert and put it into the tagged array, also store its coordinates and normals
	 * for usage in meshislands later on */

	short no[3];
	short vno[3];
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->raw_mesh;

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
	if (fc->flag & FM_FLAG_FIX_NORMALS)
		find_normal(dm, fc->nor_tree, vert->co, vno, no, fc->nor_range);
	(*startno)[3 * (*tag_counter)] = no[0];
	(*startno)[3 * (*tag_counter) + 1] = no[1];
	(*startno)[3 * (*tag_counter) + 2] = no[2];

	(*tot)++;
	(*tag_counter)++;
}

static void mesh_separate_loose_partition(Object *ob, BMesh *bm_work, BMVert **orig_work)
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
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;

	if (max_iter > 0) {
		fs->frac_mesh->progress_counter++;
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
			handle_vert(ob, v_seed, orig_work, &startco, &startno, &v_tag, &tot, &tag_counter);
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
				handle_vert(ob, e->v1, orig_work, &startco, &startno, &v_tag, &tot, &tag_counter);
			}
			if (!BM_elem_flag_test(e->v2, BM_ELEM_TAG) && !BM_elem_flag_test(e->v2, BM_ELEM_INTERNAL_TAG)) {
				handle_vert(ob, e->v2, orig_work, &startco, &startno, &v_tag, &tot, &tag_counter);
			}
		}
		BMW_end(&walker);

		/* Flush the selection to get edge/face selections matching
		 * the vertex selection */
		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_TAG);

		/* Move selection into a separate object */
		mesh_separate_tagged(ob, bm_old);

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

static void halve(Object *ob, int minsize, BMesh **bm_work, BMVert ***orig_work, bool separated)
{

	int half;
	int i = 0, new_count = 0;
	BMIter iter;
	BMVert **orig_old = *orig_work, **orig_new, **orig_mod;
	BMVert *v;
	BMesh *bm_old = *bm_work;
	BMesh *bm_new = NULL;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	separated = false;

	if (fc->current->frac_mesh->cancel == 1) {
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
	BKE_select_linked(&bm_old);

	new_count = bm_old->totvertsel;
	printf("Halving...%d => %d %d\n", bm_old->totvert, new_count, bm_old->totvert - new_count);

	orig_new = MEM_callocN(sizeof(BMVert *) * new_count, "orig_new");
	orig_mod = MEM_callocN(sizeof(BMVert *) * bm_old->totvert - new_count, "orig_mod");
	BKE_mesh_separate_selected(&bm_old, &bm_new, orig_old, &orig_new, &orig_mod);

	printf("Old New: %d %d\n", bm_old->totvert, bm_new->totvert);
	if ((bm_old->totvert <= minsize && bm_old->totvert > 0) || (bm_new->totvert == 0)) {
		mesh_separate_loose_partition(ob, bm_old, orig_mod);
		separated = true;
	}

	if ((bm_new->totvert <= minsize && bm_new->totvert > 0) || (bm_old->totvert == 0)) {
		mesh_separate_loose_partition(ob, bm_new, orig_new);
		separated = true;
	}

	if ((bm_old->totvert > minsize && bm_new->totvert > 0) || (bm_new->totvert == 0 && !separated)) {
		halve(ob, minsize, &bm_old, &orig_mod, separated);
	}

	if ((bm_new->totvert > minsize && bm_old->totvert > 0) || (bm_old->totvert == 0 && !separated)) {
		halve(ob, minsize, &bm_new, &orig_new, separated);
	}


	MEM_freeN(orig_mod);
	MEM_freeN(orig_new);
	BM_mesh_free(bm_new);
	bm_new = NULL;
}

static void mesh_separate_loose(Object *ob)
{
	int minsize = 1000;
	BMesh *bm_work;
	BMVert *vert, **orig_start;
	BMIter iter;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->raw_mesh;

	bm_work = DM_to_bmesh(dm, true);
	BM_mesh_elem_hflag_disable_all(bm_work, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);

	orig_start = MEM_callocN(sizeof(BMVert *) * bm_work->totvert, "orig_start");
	/* associate new verts with old verts, here indexes should match still */
	BM_ITER_MESH (vert, &iter, bm_work, BM_VERTS_OF_MESH)
	{
		orig_start[vert->head.index] = vert;
	}

	BM_mesh_elem_index_ensure(bm_work, BM_VERT);
	BM_mesh_elem_table_ensure(bm_work, BM_VERT);

#if 0
	/* free old islandshards first, if any */
	while (rmd->fracture->islandShards.first) {
		Shard *s = rmd->fracture->islandShards.first;
		BLI_remlink(&rmd->fracture->islandShards, s);
		BKE_shard_free(s, true);
		s = NULL;
	}

	rmd->fracture->islandShards.first = NULL;
	rmd->fracture->islandShards.last = NULL;
#endif

	halve(ob, minsize, &bm_work, &orig_start, false);

	MEM_freeN(orig_start);
	orig_start = NULL;
	BM_mesh_free(bm_work);
	bm_work = NULL;

}

static void do_constraint(Object* ob, MeshIsland *mi1, MeshIsland *mi2, int con_type, float thresh)
{
	RigidBodyShardCon *rbsc;
	ConstraintContainer *cc = ob->rigidbody_constraint->fracture_constraints;
	FractureContainer *fc1 = ob->rigidbody_constraint->ob1->rigidbody_object->fracture_objects;
	FractureContainer *fc2 = ob->rigidbody_constraint->ob2->rigidbody_object->fracture_objects;

	rbsc = BKE_rigidbody_create_shard_constraint(con_type);
	rbsc->mi1 = mi1;
	rbsc->mi2 = mi2;
	if (thresh == 0 || !(cc->flag & FMC_FLAG_USE_BREAKING)){
		rbsc->flag &= ~RBC_FLAG_USE_BREAKING;
	}

	rbsc->flag |= RBC_FLAG_DISABLE_COLLISIONS;

	//inner constraints, check for clusters
	if ((mi1->particle_index != -1) && (mi2->particle_index != -1) &&
		(mi1->particle_index == mi2->particle_index) &&
	    (mi1->partner_index == mi2->partner_index))
	{
		if ((fc1->cluster_count > 1) && (fc2->cluster_count > 1)) {
			rbsc->breaking_threshold = cc->cluster_breaking_threshold;
		}
		else {
			rbsc->breaking_threshold = thresh;
		}
	}
	else
	{
		if ((mi1->particle_index != -1) && (mi2->particle_index != -1) &&
			(mi1->particle_index != mi2->particle_index) &&
		    (mi1->partner_index == mi2->partner_index))
		{
			/* set a different type of constraint between clusters */
			rbsc->type = cc->cluster_constraint_type;
		}
		rbsc->breaking_threshold = thresh;
	}

	if ((fc1->thresh_defgrp_name[0]) && (fc2->thresh_defgrp_name)) {
		/* modify maximum threshold by minimum weight */
		rbsc->breaking_threshold = thresh * MIN2(mi1->thresh_weight, mi2->thresh_weight);
	}

	BLI_addtail(&cc->constraint_map, rbsc);

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

static void connect_meshislands(Object* ob, MeshIsland *mi1, MeshIsland *mi2, int con_type, float thresh)
{
	ConstraintContainer *cc = ob->rigidbody_constraint->fracture_constraints;
	int con_found = false;
	RigidBodyShardCon *con;
	bool ok = mi1 && mi1->rigidbody;
	ok = ok && mi2 && mi2->rigidbody;
	ok = ok && (cc->flag & FMC_FLAG_USE_CONSTRAINTS);

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
		do_constraint(ob, mi1, mi2, con_type, thresh);
	}
}

static void search_tree_based(Object* ob, MeshIsland *mi, MeshIsland **meshIslands,
                              KDTree **combined_tree, GHash* vertex_island_map, float co[3])
{
	int r = 0, limit = 0, i = 0;
	KDTreeNearest *n3 = NULL;
	float dist, obj_centr[3];
	ConstraintContainer* cc = ob->rigidbody_constraint->fracture_constraints;
	//bool outer = ob->rigidbody_constraint->ob1 != ob->rigidbody_constraint->ob2;
	//need to multiply with constraint obmat, not the rigidbody obs so its found in tree properly

	limit = cc->constraint_limit;
	dist = cc->contact_dist;

	if (cc->constraint_target == MOD_FRACTURE_CENTROID) {
		mul_v3_m4v3(obj_centr, ob->obmat, mi->centroid);
#if 0
		if (outer) {
			/* for outer constraints use constraint objects center point */
			/* FM_TODO unsure what to do with vertexbased method */
			copy_v3_v3(obj_centr, ob->loc);
		}
#endif
	}
	else if (cc->constraint_target == MOD_FRACTURE_VERTEX){
		//mul_v3_m4v3(obj_centr, ob->obmat, co);
		copy_v3_v3(obj_centr, co);
	}

	r = BLI_kdtree_range_search(*combined_tree, obj_centr, &n3, dist);

	/* use centroid dist based approach here, together with limit */
	for (i = 0; i < r; i++) {
		MeshIsland *mi2 = NULL;

		if (cc->constraint_target == MOD_FRACTURE_CENTROID) {
			mi2 = meshIslands[(n3 + i)->index];
		}
		else if(cc->constraint_target == MOD_FRACTURE_VERTEX) {
			int index = (n3 + i)->index;
			mi2 = BLI_ghash_lookup(vertex_island_map, SET_INT_IN_POINTER(index));
		}
		if ((mi != mi2) && (mi2 != NULL)) {
			float thresh = cc->breaking_threshold;
			int con_type = ob->rigidbody_constraint->type;

			if ((i >= limit) && (limit > 0)) {
				break;
			}

			connect_meshislands(ob, mi, mi2, con_type, thresh);
		}
	}

	if (n3 != NULL) {
		MEM_freeN(n3);
		n3 = NULL;
	}
}
static void do_prepare_constraint_search(Object *obj, MeshIsland ***mesh_islands, KDTree **combined_tree, GHash** vertex_index_map, int start, int target, int partner_index)
{
	MeshIsland *mi;
	FractureContainer *fc = NULL;
	FractureState *fs = NULL;
	Object* ob = NULL;

	int j = 0;

	if (!obj->rigidbody_constraint)
		return;

	if (partner_index == 1) {
		ob = obj->rigidbody_constraint->ob1;
	}
	else if (partner_index == 2) {
		ob = obj->rigidbody_constraint->ob2;
	}

	if (!ob)
		return;

	fc = ob->rigidbody_object->fracture_objects;
	fs = fc->current;

	do_island_vertex_index_map(ob, vertex_index_map, partner_index);

	if (target == MOD_FRACTURE_CENTROID)
	{
		for (mi = fs->island_map.first; mi; mi = mi->next)
		{
			float obj_centr[3];
			mi->partner_index = partner_index;
			(*mesh_islands)[start + j] = mi;
			mul_v3_m4v3(obj_centr, obj->obmat, (*mesh_islands)[start + j]->centroid);
			BLI_kdtree_insert(*combined_tree, start+j, obj_centr);

			j++;
		}
	}
	else if (target == MOD_FRACTURE_VERTEX)
	{
		int i = 0;
		int totvert = fs->visual_mesh->numVertData;
		MVert *mvert = fs->visual_mesh->getVertArray(fs->visual_mesh);
		MVert *mv;

		for (i = 0, mv = mvert; i < totvert; i++, mv++) {
			float co[3];
			mul_v3_m4v3(co, obj->obmat, mv->co);
			BLI_kdtree_insert(*combined_tree, start+i, co);
		}
	}
}
static DerivedMesh* combine_dm(Object* ob1, Object *ob2, DerivedMesh *dm1, DerivedMesh *dm2)
{
	DerivedMesh *dm = NULL;
	float mat[4][4]; /*splinter matrix, leave as unit_m4 for now, should be applied to visual mesh already*/
	FracMesh *fm = BKE_create_fracmesh();
	/*sigh, need to combine DMs too */
	Shard *s1 = BKE_create_fracture_shard(dm1->getVertArray(dm1), dm1->getPolyArray(dm1), dm1->getLoopArray(dm1),
	                                      dm1->numVertData, dm1->numPolyData, dm1->numLoopData, true);

	Shard *s2 = BKE_create_fracture_shard(dm2->getVertArray(dm2), dm2->getPolyArray(dm2), dm2->getLoopArray(dm2),
	                                      dm2->numVertData, dm2->numPolyData, dm2->numLoopData, true);
	int i = 0;
	unit_m4(mat);
	add_shard(fm, s1, mat);
	add_shard(fm, s2, mat);

	for (i = 0; i < s1->totvert; i++)
	{
		mul_m4_v3(ob1->obmat, s1->mvert[i].co);
	}

	for (i = 0; i < s2->totvert; i++)
	{
		mul_m4_v3(ob2->obmat, s2->mvert[i].co);
	}

	dm = BKE_fracture_create_dm(NULL, fm, false);
	BKE_fracmesh_free(fm, false);

	return dm;
}

static void create_constraints(Object *ob, MeshIsland **mesh_islands, int count, KDTree *coord_tree, GHash* vertex_island_map, int target)
{
	RigidBodyCon *rbc = ob->rigidbody_constraint;
	RigidBodyOb *rb1 = rbc->ob1->rigidbody_object;
	RigidBodyOb *rb2 = rbc->ob2->rigidbody_object;
	DerivedMesh *dm1 = rb1->fracture_objects->current->visual_mesh;
	DerivedMesh *dm2 = rb2->fracture_objects->current->visual_mesh;
	//DerivedMesh *dm = combine_dm(rbc->ob1, rbc->ob2, dm1, dm2);
	//FM_TODO, try with constraint object here....
	DerivedMesh *dm = combine_dm(ob, ob, dm1, dm2);
	int i = 0;

	for (i = 0; i < count; i++) {
		if (target == MOD_FRACTURE_CENTROID) {
			search_tree_based(ob, mesh_islands[i], mesh_islands, &coord_tree, vertex_island_map, NULL);
		}
		else if (target == MOD_FRACTURE_VERTEX) {
			MVert mv;
			MeshIsland *mi = NULL;
			dm->getVert(dm, i, &mv);
			mi = BLI_ghash_lookup(vertex_island_map, SET_INT_IN_POINTER(i));
			search_tree_based(ob, mi, mesh_islands, &coord_tree, vertex_island_map, mv.co);
		}
	}

	if (dm)
	{
		dm->needsFree = 1;
		DM_release(dm);
	}
}

static void fill_vgroup(Object *ob, MDeformVert *dvert)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->current->visual_mesh;

	/* use fallback over inner material (no more, now directly via tagged verts) */
	if (fc->inner_defgrp_name[0]) {
		int ind = 0;
		MPoly *mp = dm->getPolyArray(dm);
		MLoop *ml = dm->getLoopArray(dm);
		MVert *mv = dm->getVertArray(dm);
		int count = dm->getNumPolys(dm);
		int totvert = dm->getNumVerts(dm);
		const int inner_defgrp_index = defgroup_name_index(ob, fc->inner_defgrp_name);

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

#if 0
static void do_cache_regular(FractureModifierData* fmd, MeshIsland *mi, int thresh_defgrp_index,
                             int ground_defgrp_index, MVert** verts, MDeformVert** dvert, int *vertstart)
{
	int i;

	for (i = 0; i < mi->vertex_count; i++) {
		mi->vertices_cached[i] = (*verts) + (*vertstart) + i;

		/* sum up vertexweights and divide by vertcount to get islandweight*/
		if (*dvert && ((*dvert) + (*vertstart) + i)->dw && fmd->fracture->thresh_defgrp_name[0]) {
			float vweight = defvert_find_weight((*dvert) + (*vertstart) + i, thresh_defgrp_index);
			mi->thresh_weight += vweight;
		}

		if (*dvert && ((*dvert) + (*vertstart) + i)->dw && fmd->fracture->ground_defgrp_name[0]) {
			float gweight = defvert_find_weight((*dvert) + (*vertstart) + i, ground_defgrp_index);
			mi->ground_weight += gweight;
		}

		if (mi->vertno != NULL && (fmd->fracture->flag & FM_FLAG_FIX_NORMALS)) {
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
		if (index >= 0 && index <= fmd->fracture->visible_mesh->totvert) {
			mi->vertices_cached[i] = (*verts) + index;
		}
		else {
			mi->vertices_cached[i] = NULL;
		}

		if (*dvert && ((*dvert) + index)->dw && fmd->fracture->thresh_defgrp_name[0]) {
			float vweight = defvert_find_weight((*dvert) + index, thresh_defgrp_index);
			mi->thresh_weight += vweight;
		}

		if (*dvert && ((*dvert) + index)->dw && fmd->fracture->ground_defgrp_name[0]) {
			float gweight = defvert_find_weight((*dvert) + index, ground_defgrp_index);
			mi->ground_weight += gweight;
		}

		if (mi->vertno != NULL && (fmd->fracture->flag & FM_FLAG_FIX_NORMALS)) {
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
	const int thresh_defgrp_index = defgroup_name_index(ob, fmd->fracture->thresh_defgrp_name);
	const int ground_defgrp_index = defgroup_name_index(ob, fmd->fracture->ground_defgrp_name);
	bool orig_chosen = false;

	/*regular fracture case */
	if (fmd->fracture->dm && !(fmd->fracture->flag & FM_FLAG_SHARDS_TO_ISLANDS) && (fmd->fracture->dm->getNumPolys(fmd->fracture->dm) > 0)) {
		dm = CDDM_copy(fmd->fracture->dm);
	}
	/* split to islands or halving case (fast bisect e.g.) */
	else if (fmd->fracture->visible_mesh && (fmd->fracture->visible_mesh->totface > 0) && BLI_listbase_count(&fmd->fracture->meshIslands) > 1) {
		dm = CDDM_from_bmesh(fmd->fracture->visible_mesh, true);
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
	for (mi = fmd->fracture->meshIslands.first; mi; mi = mi->next) {
		if (mi->vertices_cached) {
			MEM_freeN(mi->vertices_cached);
			mi->vertices_cached = NULL;
		}

		if (fmd->fracture->thresh_defgrp_name[0]) {
			mi->thresh_weight = 0;
		}

		mi->vertices_cached = MEM_mallocN(sizeof(MVert *) * mi->vertex_count, "mi->vertices_cached");
		if (fmd->fracture->dm != NULL && !(fmd->fracture->flag & FM_FLAG_SHARDS_TO_ISLANDS) && !orig_chosen && fmd->fracture->visible_mesh == NULL) {
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
			mi->rigidbody->type = mi->ground_weight > 0.5f ? RBO_TYPE_PASSIVE : RBO_TYPE_ACTIVE;
		}

		/* use fallback over inner material*/
		fill_vgroup(fmd, dm, dvert, ob);
	}

	return dm;
}
#endif

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

static void make_face_pairs(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->current->visual_mesh; // for all post fracture ops use this, else ob->derivedFinal, make an ensure function!!!

	/* make kdtree of all faces of dm, then find closest face for each face*/
	MPoly *mp = NULL;
	MPoly *mpoly = dm->getPolyArray(dm);
	MLoop* mloop = dm->getLoopArray(dm);
	MVert* mvert = dm->getVertArray(dm);
	int totpoly = dm->getNumPolys(dm);
	KDTree *tree = BLI_kdtree_new(totpoly);
	int i = 0;

	fc->face_pairs = BLI_ghash_int_new("face_pairs");

	//printf("Make Face Pairs\n");

	for (i = 0, mp = mpoly; i < totpoly; mp++, i++) {
		float co[3];
		DM_face_calc_center_mean(dm, mp, co);
		if (mp->mat_nr == 1)
		{
			BLI_kdtree_insert(tree, i, co);
		}
	}

	BLI_kdtree_balance(tree);

	/*now find pairs of close faces*/

	for (i = 0, mp = mpoly; i < totpoly; mp++, i++) {
		if (mp->mat_nr == 1) { /* treat only inner faces ( with inner material) */
			int index = -1, j = 0, r = 0;
			KDTreeNearest *n;
			float co[3];

			DM_face_calc_center_mean(dm, mp, co);
			r = BLI_kdtree_range_search(tree, co, &n, fc->autohide_dist * 4);
			/*2nd nearest means not ourselves...*/
			if (r == 0)
				continue;

			index = n[0].index;
			while ((j < r) && i == index) {
				index = n[j].index;
				j++;
			}

			if (!BLI_ghash_haskey(fc->face_pairs, SET_INT_IN_POINTER(index))) {
				BLI_ghash_insert(fc->face_pairs, SET_INT_IN_POINTER(i), SET_INT_IN_POINTER(index));
				/*match normals...*/
				if (fc->flag & FM_FLAG_FIX_NORMALS) {
					do_match_normals(mp, mpoly+index, mvert, mloop);
				}
			}

			if (n != NULL) {
				MEM_freeN(n);
			}
		}
	}

	BLI_kdtree_free(tree);
}

static void find_other_face(Object* ob, int i, BMesh* bm, BMFace ***faces, int *del_faces)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	float f_centr[3], f_centr_other[3];
	BMFace *f1, *f2;
	int other = GET_INT_FROM_POINTER(BLI_ghash_lookup(fc->face_pairs, SET_INT_IN_POINTER(i)));

	if (other == i)
	{
		return;
	}

	f1 = BM_face_at_index(bm, i);
	f2 = BM_face_at_index(bm, other);

	if ((f1 == NULL) || (f2 == NULL)) {
		return;
	}

	BM_face_calc_center_mean(f1, f_centr);
	BM_face_calc_center_mean(f2, f_centr_other);


	if ((len_squared_v3v3(f_centr, f_centr_other) < fc->autohide_dist * fc->autohide_dist) && (f1 != f2) &&
	    (f1->mat_nr == 1) && (f2->mat_nr == 1))
	{
		/*intact face pairs */
		*faces = MEM_reallocN(*faces, sizeof(BMFace *) * ((*del_faces) + 2));
		(*faces)[*del_faces] = f1;
		(*faces)[(*del_faces) + 1] = f2;
		(*del_faces) += 2;
	}
}

DerivedMesh *BKE_fracture_autohide(Object* ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->current->visual_mesh;
	int i = 0;
	BMesh *bm;
	DerivedMesh *result;
	int del_faces = 0;
	BMFace **faces;
	int totpoly;

	if (fc->autohide_dist == 0.0f)
	{
		return CDDM_copy(dm);
	}

	totpoly = dm->getNumPolys(dm);
	bm = DM_to_bmesh(dm, true);
	faces = MEM_mallocN(sizeof(BMFace *), "faces");

	BM_mesh_elem_index_ensure(bm, BM_FACE);
	BM_mesh_elem_table_ensure(bm, BM_FACE);
	BM_mesh_elem_toolflags_ensure(bm);

	BM_mesh_elem_hflag_disable_all(bm, BM_FACE | BM_EDGE | BM_VERT , BM_ELEM_SELECT, false);

	for (i = 0; i < totpoly; i++) {
		find_other_face(ob, i, bm, &faces, &del_faces);
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
	BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	             "automerge_keep_normals verts=%hv dist=%f", BM_ELEM_SELECT,
	             fc->autohide_dist * 10); /*need to merge larger cracks*/

	//dissolve sharp edges with limit dissolve
	BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "dissolve_limit_keep_normals "
	             "angle_limit=%f use_dissolve_boundaries=%b verts=%av edges=%ae delimit=%i",
	             DEG2RADF(1.0f), false, 0);

	result = CDDM_from_bmesh(bm, true);
	BM_mesh_free(bm);
	MEM_freeN(faces);

	return result;
}

static void do_fix_normals_physics_mesh(Object* ob, Shard* s, MeshIsland* mi, int i)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	MVert *mv, *verts;
	int totvert;
	int j;
	DerivedMesh *orig_dm = fc->raw_mesh;

	mi->physics_mesh = BKE_shard_create_dm(s, true);
	totvert = mi->physics_mesh->getNumVerts(mi->physics_mesh);
	verts = mi->physics_mesh->getVertArray(mi->physics_mesh);

	mi->vertcos = MEM_mallocN(sizeof(float) * 3 * totvert, "vertco");
	mi->vertnos = MEM_mallocN(sizeof(short) * 3 * totvert, "vertno");

	for (mv = verts, j = 0; j < totvert; mv++, j++) {
		short no[3];

		copy_v3_v3(mi->vertcos[j], mv->co);

		/* either take orignormals or take ones from fractured mesh */
		if (fc->flag & FM_FLAG_FIX_NORMALS) {
			find_normal(orig_dm, fc->nor_tree, mv->co, mv->no, no, fc->nor_range);
		}

		copy_v3_v3_short(mi->vertnos[j], no);

		if (fc->flag & FM_FLAG_FIX_NORMALS) {
			copy_v3_v3_short(mi->vertices_cached[j]->no, no);
			copy_v3_v3_short(mv->no, no);
		}

		/* then eliminate centroid in vertex coords*/
		sub_v3_v3(mv->co, s->centroid);
	}

	if (fc->flag & FM_FLAG_FIX_NORMALS)
	{
		printf("Fixing Normals: %d\n", i);
	}
}

static void do_verts_weights(Object* ob, Shard *s, MeshIsland *mi, int vertstart,
                             int thresh_defgrp_index, int ground_defgrp_index)
{
	MVert *mverts;
	int k;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->current->visual_mesh;
	MDeformVert *dvert = dm->getVertDataArray(dm, CD_MDEFORMVERT);

	mi->vertices_cached = MEM_mallocN(sizeof(MVert *) * s->totvert, "vert_cache");
	mverts = CDDM_get_verts(dm);

	mi->vertex_indices = MEM_mallocN(sizeof(int) * mi->vertex_count, "mi->vertex_indices");

	for (k = 0; k < s->totvert; k++) {
		mi->vertices_cached[k] = mverts + vertstart + k;
		mi->vertex_indices[k] = vertstart + k;
		/* sum up vertexweights and divide by vertcount to get islandweight*/
		if (dvert && fc->thresh_defgrp_name[0]) {
			float vweight = defvert_find_weight(dvert + vertstart + k, thresh_defgrp_index);
			mi->thresh_weight += vweight;
		}

		if (dvert && fc->ground_defgrp_name[0]) {
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


#if 0
static void do_handle_parent_mi(Scene* scene, Object* ob, MeshIsland *mi, MeshIsland *par, int frame, bool is_parent)
{
	frame -= par->start_frame;
	BKE_match_vertex_coords(mi, par, ob, frame, is_parent);

	//hmm need a scene here... dang
	BKE_rigidbody_remove_shard(scene, par);
	//fmd->modifier.scene->rigidbody_world->flag |= RBW_FLAG_OBJECT_CHANGED; // ??? flag at object now ?

	par->rigidbody->flag |= RBO_FLAG_NEEDS_VALIDATE;
}
#endif

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

static void set_rigidbody_type(Object *ob, Shard *s, MeshIsland *mi)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	//how far is impact location away from this shard, if beyond a bbox, keep passive
	if (fs)
	{
		FractureState *prev = fs->prev;

		if (prev && (prev->prev == NULL)) //only affect primary fracture
		{
			Shard *par = BKE_shard_by_id(prev->frac_mesh, s->parent_id);
			if (par)
			{
				float impact_loc[3], impact_size[3];
				copy_v3_v3(impact_loc, par->impact_loc);
				copy_v3_v3(impact_size, par->impact_size);

				if (contains(impact_loc, impact_size, s->centroid))
				{
					mi->rigidbody->flag &= ~RBO_FLAG_KINEMATIC;
				}
				else
				{
					mi->rigidbody->flag |= RBO_FLAG_KINEMATIC;
				}
			}
		}
	}
}

static void do_island_from_shard(Object *ob, Shard* s, int i, int thresh_defgrp_index, int ground_defgrp_index, int vertstart)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	FractureContainer *fc = rb->fracture_objects;
	FractureState *fs = fc->current;

	MeshIsland *mi;
	MeshIsland *par = NULL;
//	bool is_parent = false;
	short rb_type = rb->type;
	float dummyloc[3], rot[4];

	if (s->totvert == 0) {
		return;
	}

	fc->current->frac_mesh->progress_counter++;

	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
	BLI_addtail(&fs->island_map, mi);

	mi->participating_constraints = NULL;
	mi->participating_constraint_count = 0;
	mi->thresh_weight = 0;
	mi->ground_weight = 0;
	mi->vertex_count = s->totvert;

	//call this later, when the DM has been built... hmmmmmm or rebuild... but vertrefs will be useless then
	do_verts_weights(ob, s, mi, vertstart, thresh_defgrp_index, ground_defgrp_index);

	/*copy fixed normals to physics mesh too (needed for convert to objects)*/

	do_fix_normals_physics_mesh(ob, s, mi, i);

	BKE_shard_calc_minmax(s);
	copy_v3_v3(mi->centroid, s->centroid);

	mat4_to_loc_quat(dummyloc, rot, ob->obmat);
	copy_v3_v3(mi->rot, rot);
	mi->id = s->shard_id;

#if 0
	if (fc->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		/*take care of previous transformation, if any*/
		FractureState *prev = NULL;

		if (fs) {
			prev = fs->prev;
		}

		if (prev)
		{
			int frame = prev->frame;

			par = find_meshisland(&prev->island_map, s->parent_id);
			if (par)
			{
				is_parent = true;
				do_handle_parent_mi(scene, ob, mi, par, frame, is_parent);
			}
			else
			{
				par = find_meshisland(&prev->island_map, s->shard_id);
				if (par)
				{
					is_parent = false;
					do_handle_parent_mi(scene, ob, mi, par, frame, is_parent);
				}
			}
		}
	}
#endif

	do_calc_volume(fc, mi);
	mi->bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(mi->bb, s->min, s->max);

	mi->particle_index = -1;
	//mi->neighbor_ids = s->neighbor_ids;
	//mi->neighbor_count = s->neighbor_count;

	//TODO, what was it good for ?
	//rb_type = do_vert_index_map(fmd, mi);
	do_rigidbody(ob, mi, rb_type);

	if (fc->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		if (fc->flag & FM_FLAG_LIMIT_IMPACT)
		{
			set_rigidbody_type(ob, s, mi);
		}

		if (par != NULL)
		{
			copy_v3_v3(mi->rigidbody->lin_vel, par->rigidbody->lin_vel);
			copy_v3_v3(mi->rigidbody->ang_vel, par->rigidbody->ang_vel);
		}
	}
}

static MDeformVert* do_islands_from_shards(Object* ob)
{
	/* can be created without shards even, when using fracturemethod = NONE (re-using islands)*/
	Shard *s;
	int i = 0, vertstart = 0;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

	MDeformVert *ivert = NULL;
	ListBase shardlist;
	const int thresh_defgrp_index = defgroup_name_index(ob, fc->thresh_defgrp_name);
	const int ground_defgrp_index = defgroup_name_index(ob, fc->ground_defgrp_name);

	/*XXX should rename this... this marks the fracture case, to distinguish from halving case */
	//fc->flag |= FM_FLAG_USE_FRACMESH;

	if (fc->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
#if 0
		/* exchange cached mesh after fracture, XXX looks like double code */
		if (fmd->fracture->visible_mesh_cached) {
			fmd->fracture->visible_mesh_cached->needsFree = 1;
			fmd->fracture->visible_mesh_cached->release(fmd->fracture->visible_mesh_cached);
			fmd->fracture->visible_mesh_cached = NULL;
		}

		fmd->fracture->visible_mesh_cached = CDDM_copy(fmd->fracture->dm);
#endif

		DerivedMesh *dm = fc->current->visual_mesh;
		/* to write to a vgroup (inner vgroup) use the copied cached mesh */
		ivert = dm->getVertDataArray(dm, CD_MDEFORMVERT);

		if (ivert == NULL) {    /* add, if not there */
			int totvert = dm->getNumVerts(dm);
			ivert = CustomData_add_layer(&dm->vertData, CD_MDEFORMVERT, CD_CALLOC,
										 NULL, totvert);
		}
	}

#if 0
	else
	{
		fmd->fracture->visible_mesh_cached = CDDM_copy(fmd->fracture->dm);
	}
#endif

	shardlist = fc->current->frac_mesh->shard_map;

	for (s = shardlist.first; s; s = s->next) {
		do_island_from_shard(ob, s, i, thresh_defgrp_index, ground_defgrp_index, vertstart);
		vertstart += s->totvert;
		i++;
	}

	return ivert;
}

#if 0
static DerivedMesh *output_dm(Object *ob DerivedMesh *dm, bool exploOK)
{
	FractureSetting *fs = NULL;
	DerivedMesh* dm_final = NULL;

	for (fs = fmd->fracture_settings.first; fs; fs = fs->next)
	{
		fmd->fracture = fs;

		if ((fmd->fracture->visible_mesh_cached != NULL) && exploOK) {

			if (fmd->fracture->autohide_dist > 0 && fmd->fracture->face_pairs) {
				//printf("Autohide2 \n");
				fs->final_mesh = do_autoHide(fmd, fmd->fracture->visible_mesh_cached);
			}
			else {
				fs->final_mesh = CDDM_copy(fmd->fracture->visible_mesh_cached);
			}
		}
		else {
			if (fmd->fracture->visible_mesh == NULL && fmd->fracture->visible_mesh_cached == NULL) {
				/* oops, something went definitely wrong... */
				fmd->fracture->flag |= FM_FLAG_REFRESH;
				freeData_internal(fmd, fmd->fracture_mode == MOD_FRACTURE_PREFRACTURED);
				fmd->fracture->visible_mesh_cached = NULL;
				fmd->fracture->flag &= ~FM_FLAG_REFRESH;
				fmd->fracture->final_mesh = NULL;
				return dm;
			}
		}
	}

	dm_final = BKE_fracture_create_dm(fmd, fmd->fracture, true, true);
	return dm_final;
}
#endif

static void do_post_island_creation(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	DerivedMesh *dm = fc->current->visual_mesh;

	/* fallback, this branch is executed when the modifier data has been loaded via readfile.c,
	 * although this might not be directly visible due to complex logic */

	MDeformVert* dvert = NULL;
	if (dm)
		dvert = dm->getVertDataArray(dm, CD_MDEFORMVERT);
	if ((dvert != NULL) && (dvert->dw == NULL))
		fill_vgroup(ob, dvert);


	/* need to ensure images are correct after loading... */
	if ((fc->fracture_mode == MOD_FRACTURE_PREFRACTURED) && (!(fc->flag & FM_FLAG_SKIP_MASS_CALC)))
	{
		refresh_customdata_image(ob->data, &dm->polyData, dm->getNumPolys(dm));
		DM_update_tessface_data(dm);
	}

#if 0
	if (fc->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		fc->current->is_new = false;
	}
#endif //TODO, necessary still ?

	if (fc->flag & FM_FLAG_EXECUTE_THREADED) {
		/* job done */
		fc->current->frac_mesh->running = 0;
	}
}



void do_prepare_autohide(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	/*HERE make a kdtree of the fractured derivedmesh,
	 * store pairs of faces (MPoly) here (will be most likely the inner faces) */
	if (fc->face_pairs != NULL) {
		BLI_ghash_free(fc->face_pairs, NULL, NULL);
		fc->face_pairs = NULL;
	}

	if (fc->current->visual_mesh)
	{
		make_face_pairs(ob);
	}
}

void BKE_fracture_prepare_autohide(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	if (fc && fc->flag & FM_FLAG_REFRESH_AUTOHIDE)
	{
		do_prepare_autohide(ob);
		fc->flag &= ~FM_FLAG_REFRESH_AUTOHIDE;
	}
}

static void do_halving(Object* ob)
{
	double start;

	start = PIL_check_seconds_timer();
	//printf("Steps: %d \n", fmd->fracture->frac_mesh->progress_counter);
	mesh_separate_loose(ob);
	printf("Splitting to islands done, %g \n", PIL_check_seconds_timer() - start);
}

static void do_refresh(Object *ob, bool do_rebuild)
{
	double start = 0.0;
	MDeformVert *ivert = NULL;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState* fs = fc->current;

	//pre-halving... making shards (NOT islands, watch it !)
	if (fc->flag & FM_FLAG_SHARDS_TO_ISLANDS)
		do_halving(ob);

	if (do_rebuild)
	{
		//watch it when overwriting this... free it better before
		if (fs->visual_mesh)
		{
			fs->visual_mesh->needsFree = 1;
			DM_release(fs->visual_mesh);
			fs->visual_mesh = NULL;
		}

		fs->visual_mesh = BKE_fracture_create_dm(ob, fs->frac_mesh, true);
	}

	DM_ensure_tessface(fs->visual_mesh);
	DM_ensure_normals(fs->visual_mesh);
	DM_update_tessface_data(fs->visual_mesh);

	if (fc->flag & FM_FLAG_FIX_NORMALS)
	{
		start = PIL_check_seconds_timer();
		fc->nor_tree = build_nor_tree(fc->raw_mesh);
	}

	//shards -> islands;
	ivert = do_islands_from_shards(ob);

	//post halving ? TODO

	if (fc->flag & FM_FLAG_FIX_NORMALS) {
		printf("Fixing normals done, %g\n", PIL_check_seconds_timer() - start);
	}

	fill_vgroup(ob, ivert);

	printf("Islands: %d\n", BLI_listbase_count(&fc->current->island_map));

#if 0
	if (fmd->fracture_mode == MOD_FRACTURE_DYNAMIC)
	{
		/* Grrr, due to stupid design of mine (listbase as value in struct instead of pointer)
		 * we have to synchronize the lists here again */

		/* need to ensure(!) old pointers keep valid, else the whole meshisland concept is broken */
		fmd->fracture->current_mi_entry->visible_dm = fmd->fracture->visible_mesh_cached;
		fmd->fracture->current_mi_entry->meshIslands = fmd->fracture->meshIslands;
	}
#endif //TODO, move states from rigidbody system...
}

static void do_island_vertex_index_map(Object *ob, GHash** vertex_island_map, int partner_index)
{
	MeshIsland *mi;
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	int start = 0;

	if (!(*vertex_island_map))
	{
		if (fc->vertex_island_map) {
			BLI_ghash_free(fc->vertex_island_map, NULL, NULL);
		}
		fc->vertex_island_map = BLI_ghash_ptr_new("island_index_map");
		*vertex_island_map = fc->vertex_island_map;
	}
	else
	{
		start = BLI_ghash_size(*vertex_island_map);
	}

	for (mi = fs->island_map.first; mi; mi = mi->next){
		int i = 0;
		mi->partner_index = partner_index;
		if (mi->vertex_indices != NULL)
		{	/* might not existing yet for older files ! */
			for (i = 0; i < mi->vertex_count; i++)
			{
				BLI_ghash_insert(*vertex_island_map, SET_INT_IN_POINTER(mi->vertex_indices[i] + start), mi);
			}
		}
	}
}


void BKE_fracture_create_islands(Object *ob, bool rebuild)
{
	if (thread_sentinel(ob))
		return;

	do_refresh(ob, rebuild);
	do_post_island_creation(ob);
	do_prepare_autohide(ob);
	//do_island_index_map(ob); //TODO... what was this good for ?
	do_clusters(ob);
	update_islands(ob);
}

static void add_fracture_state(Scene *scene, Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = MEM_callocN(sizeof(FractureState), "add_fracture_state");
	DerivedMesh *dm = fc->raw_mesh;
	float frame = BKE_scene_frame_get(scene);

	if (BLI_listbase_is_empty(&fc->states))
	{
		float mat[4][4];
		Shard *s = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
		                                     dm->numVertData, dm->numPolyData, dm->numLoopData, true);
		//pre-init
		unit_m4(mat);
		fs->frac_mesh = BKE_create_fracmesh();
		add_shard(fs->frac_mesh, s, mat);

		//make meshslands
		do_refresh(ob, true);
	}

	BLI_addtail(&fc->states, fs);
	if (fc->current)
		fc->current->frame = frame; //this is the endframe of the state
	fs->frame = fc->pointcache->endframe; //preliminary endframe is sim endframe
	fc->current = fs;
}

static void do_modifier(Scene *scene, Object* ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

	if (thread_sentinel(ob))
		return;

	/*HERE we must know which shard(s) to fracture... hmm shards... we should "merge" states which happen in the same frame automatically !*/
	if (fc->fracture_mode == MOD_FRACTURE_PREFRACTURED)
	{
		//have a shard 0 here...
		do_fracture(scene, ob, 0);
	}

	//hmm better call this from Rigidbody system directly... TODO
#if 0
	else
	{
		int frame = (int)BKE_scene_frame_get(scene);

		if (!(BKE_lookup_mesh_state(fmd, frame, false)))
		{
			/*simulation mode*/
			/* bullet callbacks may happen multiple times per frame, in next frame we can evaluate them all,
			 * so we need some array of shardIDs or shards to fracture each *
			 * we need to loop over those shard IDs here, but lookup of shard ids might be slow, but fracturing of many shards is slower...
			 * should not have a visible effect in general */

			int count = 0;

			if (fc->flag & FM_FLAG_UPDATE_DYNAMIC)
			{
				BKE_free_constraints(fmd);
				printf("ADD NEW 2: %s \n", ob->id.name);
				fmd->fracture->flag &= ~FM_FLAG_UPDATE_DYNAMIC;
				add_new_entries(fmd, dm, ob);
			}

			while(fmd->fracture->fracture_ids.first){
				FractureID* fid = (FractureID*)fmd->fracture->fracture_ids.first;
				do_fracture(fmd, fid->shardID, ob, dm);
				BLI_remlink(&fmd->fracture->fracture_ids, fid);
				MEM_freeN(fid);
				count++;
			}

			if (count > 0)
			{
				BKE_free_constraints(fmd);
				printf("REFRESH: %s \n", ob->id.name);
				fmd->modifier.scene->rigidbody_world->flag |= RBW_FLAG_OBJECT_CHANGED;
				fmd->fracture->flag |= FM_FLAG_REFRESH;
				//fmd->current_shard_entry->is_new = false;
			}
		}

		fmd->last_frame = frame;
	}
#endif
}

static void preprocess_dm(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

	/* may have messed up meshes from conversion */
	if (ob->type == OB_FONT || ob->type == OB_CURVE || ob->type == OB_SURF) {
		DerivedMesh *result = NULL;

		DerivedMesh *dm = fc->raw_mesh;
		/* convert to BMesh, remove doubles, limited dissolve and convert back */
		BMesh *bm = DM_to_bmesh(dm, true);

		BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
		             "remove_doubles verts=%av dist=%f", BM_VERTS_OF_MESH, 0.0001, false);

		BM_mesh_decimate_dissolve(bm, 0.087f, false, 0);
		result = CDDM_from_bmesh(bm, true);
		BM_mesh_free(bm);

		if (fc->raw_mesh)
		{
			fc->raw_mesh->needsFree = 1;
			DM_release(fc->raw_mesh);
			fc->raw_mesh = NULL;
		}

		fc->raw_mesh = result; //FM_TODO, maybe make a copy
	}
}

#if 0
static short collect_materials(Object* o, Object* ob, short matstart, GHash** mat_index_map)
{
	short *totcolp = NULL, k = 0;
	Material ***matarar = NULL;
	int j;

	/* append materials to target object, if not existing yet */
	totcolp = give_totcolp(o);
	matarar = give_matarar(o);

	for (j = 0; j < *totcolp; j++)
	{
		int index = find_material_index(ob, (*matarar)[j]);
		if (index == 0)
		{
			assign_material(ob, (*matarar)[j], matstart + k, BKE_MAT_ASSIGN_USERPREF);
			index = matstart + k;
			k++;
		}

		BLI_ghash_insert(*mat_index_map, SET_INT_IN_POINTER(matstart+j), SET_INT_IN_POINTER(index));
	}

	return *totcolp;
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


static DerivedMesh* get_object_dm(Object* o)
{
	DerivedMesh *dm_ob = NULL;

	/*ensure o->derivedFinal*/
	FractureModifierData* fmd2 = (FractureModifierData*) modifiers_findByType(o, eModifierType_Fracture);
	if (fmd2)
	{
		dm_ob = fmd2->fracture->visible_mesh_cached;
	}
	else
	{
		dm_ob = o->derivedFinal;
	}

	return dm_ob;
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

		totcol = collect_materials(o, ob, matstart, mat_index_map);

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
			dm_ob = fmd2->fracture->visible_mesh_cached;
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

	if (fmd->dm_group && ((fmd->fracture->flag & FM_FLAG_REFRESH) || (fmd->fracture->flag & FM_FLAG_AUTO_EXECUTE)))
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
#endif

static void add_shard(FracMesh *fm, Shard *s, float mat[4][4])
{
	MVert *mv;
	int i = 0;

	if (s->totvert == 0)
	{
		return; //skip this shard
	}

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

	CustomData_add_layer(&s->vertData, CD_MDEFORMVERT, CD_DUPLICATE, CustomData_get_layer(&dm->vertData, CD_MDEFORMVERT), s->totvert);
	CustomData_add_layer(&s->loopData, CD_MLOOPUV, CD_DUPLICATE, CustomData_get_layer(&dm->loopData, CD_MLOOPUV), s->totloop);
	CustomData_add_layer(&s->polyData, CD_MTEXPOLY, CD_DUPLICATE, CustomData_get_layer(&dm->polyData, CD_MTEXPOLY), s->totpoly);

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
Shard *BKE_shard_by_id(FracMesh *mesh, ShardID id) {

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

#if 0
	else if (id == -1 && dm != NULL)
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
#endif

void BKE_get_shard_minmax(FracMesh *mesh, ShardID id, float min_r[3], float max_r[3])
{
	Shard *shard = BKE_shard_by_id(mesh, id);
	if (shard != NULL) {
		BKE_shard_calc_minmax(shard);
		copy_v3_v3(min_r, shard->min);
		copy_v3_v3(max_r, shard->max);

		if (shard->shard_id == -2)
		{
			BKE_shard_free(shard, true);
		}
	}
	else
	{
		printf("OOOOPS: %d is NULL!!!\n", id);
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

	shard->shard_id = 0;
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

FracMesh *BKE_create_fracmesh(void)
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
                               float centroid[3], short inner_material_index, int parent_id, Shard **tempshards, Shard ***tempresults)
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

		s = BKE_fracture_shard_bisect(*bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL, false, true, index, centroid, inner_material_index);
		s2 = BKE_fracture_shard_bisect(*bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FAST_FILL, true, false, index, centroid, inner_material_index);

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
                                   DerivedMesh **dm_p)
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
		s = BKE_fracture_shard_boolean(obj, *dm_p, t, inner_material_index, num_cuts,fractal, &s2, matrix, radius, smooth, num_levels);

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
                                  int num_levels, float fractal, int *i, bool smooth, Shard*** tempresults, DerivedMesh **dm_p)
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
		s = BKE_fracture_shard_boolean(obj, dm_parent, t, inner_material_index, 0, 0.0f, NULL, NULL, 0.0f, false, 0);
	}
	else if (algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
		handle_boolean_fractal(s, t, expected_shards, dm_parent, obj, inner_material_index, num_cuts, fractal,
		                       num_levels, smooth, parent_id, i, tempresults, dm_p);
	}
	else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL) {
		float co[3] = {0, 0, 0};
		printf("Bisecting cell %d...\n", *i);
		s = BKE_fracture_shard_bisect(bm_parent, t, obmat, algorithm == MOD_FRACTURE_BISECT_FILL, false, true, 0, co, inner_material_index);
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
                             DerivedMesh **dm_parent, BMesh** bm_parent, Shard ***tempshards, Shard ***tempresults)
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
		fill_vn_i(deletemap, fm->shard_count, 1);
	}

	if (fm->last_shard_tree)
	{
		if (expected_shards <= fm->last_expected_shards)
		{
			fill_vn_i(deletemap, fm->shard_count, 1);
		}
		else
		{
			fill_vn_i(skipmap, expected_shards, 1);
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
		if (fm->cancel == 1) {
			break;
		}

		if (skipmap[i])
		{
			printf("Skipping shard: %d\n", i);
			(*tempshards)[i] = NULL;
			(*tempresults)[i] = NULL;
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
		if (deletemap[i])
		{
			Shard *t = fm->last_shards[i];
			if (t->shard_id > -2)
			{
				BLI_remlink_safe(&fm->shard_map, t);
				BKE_shard_free(t, true);
				fm->last_shards[i] = NULL;

				printf("Deleting shard: %d\n", i);
			}
		}
	}

	fm->last_expected_shards = expected_shards;

	MEM_freeN(skipmap);
	MEM_freeN(deletemap);
}


/* parse the voro++ cell data */
static void parse_cells(cell *cells, int expected_shards, ShardID parent_id, Object *obj, short inner_material_index, float mat[4][4])
{
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	FracMesh *fm = fs->frac_mesh;
	int algorithm = fc->frac_algorithm;
	bool reset = fc->flag & FM_FLAG_RESET_SHARDS;
	int mode = fc->fracture_mode;
	int num_cuts = fc->fractal_cuts;
	int num_levels = fc->fractal_iterations;
	float fractal = fc->fractal_amount;
	bool smooth = fc->flag & FM_FLAG_USE_SMOOTH;

	/*Parse voronoi raw data*/
	int i = 0, j = 0, count = 0;
	Shard *p = BKE_shard_by_id(fm, parent_id); // *t;
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

	if (p == NULL)
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

		return;
	}

	if (reset)
	{
		//TODO, add this to fracture by greasepencil and fracture by planes, too !!!!
		while (fm->shard_map.first)
		{
			Shard *t = fm->shard_map.first;
			BLI_remlink_safe(&fm->shard_map, t);
			printf("Resetting shard: %d\n", t->shard_id);
			BKE_shard_free(t, true);
		}

		/* if we reset all shards, we have to re-initialize the 1st one */
		BKE_fracmesh_free(fm, true);
		initialize_shard(obj);
		fm = fs->frac_mesh;
		p = BKE_shard_by_id(fm, 0);
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

	do_prepare_cells(fm, cells, expected_shards, algorithm, p, &centroid, &dm_parent, &bm_parent, &tempshards, &tempresults);

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
			handle_boolean_bisect(fm, obj, expected_shards, algorithm, parent_id, tempshards, dm_parent,
			                      bm_parent, obmat, inner_material_index, num_cuts, num_levels, fractal,
			                      &i, smooth, &tempresults, &dm_p);
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
			                   tempshards, &tempresults);
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


	if (mode == MOD_FRACTURE_DYNAMIC && p->shard_id == -2)
	{
		BKE_shard_free(p, true);
	}
	else if (mode == MOD_FRACTURE_PREFRACTURED && reset)
	{
		//now the original shard seems to be in the way, delete it from shardmap, but keep pointer
		BLI_remlink_safe(&fm->shard_map, p);
		fm->shard_count--;
		BKE_shard_free(p, true);
	}

	fm->shard_count = 0; /* may be not matching with expected shards, so reset... did increment this for
	                      *progressbar only */

	//keep empty ids... need to catch this later
	if (mode == MOD_FRACTURE_DYNAMIC)
	{
		j = 1;
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

		if (t != NULL) {
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

	s->flag &= ~(SHARD_SKIP | SHARD_DELETE);
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

static void stroke_to_faces(Object *ob, BMesh** bm, bGPDstroke *gps, int inner_material_index)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	BMVert *lastv1 = NULL;
	BMVert *lastv2 = NULL;
	int p = 0;
	float thresh = (float)fc->grease_decimate / 100.0f;
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
				if (fc->cutter_axis == MOD_FRACTURE_CUTTER_X)
				{
					nvec[0] = 1.0f;
					nvec[1] = 0.0f;
					nvec[2] = 0.0f;
				}

				if (fc->cutter_axis == MOD_FRACTURE_CUTTER_Y)
				{
					nvec[0] = 0.0f;
					nvec[1] = 1.0f;
					nvec[2] = 0.0f;
				}

				if (fc->cutter_axis == MOD_FRACTURE_CUTTER_Z)
				{
					nvec[0] = 0.0f;
					nvec[1] = 0.0f;
					nvec[2] = 1.0f;
				}

				mul_v3_fl(nvec, fc->grease_offset);
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
	}
}

static void do_intersect(Object* ob, Shard *t, short inner_mat_index,
                         bool is_zero, float mat[4][4], int **shard_counts, int* count,
                         int k, DerivedMesh **dm_parent, bool keep_other_shard)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;

	/*just keep appending items at the end here */
	MPoly *mpoly, *mp;
	int totpoly;
	Shard *parent = NULL;
	Shard *s = NULL, *s2 = NULL;
	int shards = 0, j = 0;

	if (is_zero == false) {
		parent = BLI_findlink(&fs->frac_mesh->shard_map, k);
		*dm_parent = BKE_shard_create_dm(parent, true);
	}

	mpoly = (*dm_parent)->getPolyArray(*dm_parent);
	totpoly = (*dm_parent)->getNumPolys(*dm_parent);

	for (j = 0, mp = mpoly; j < totpoly; j++, mp++) {
		mp->flag &= ~ME_FACE_SEL;
	}

	if (keep_other_shard)
	{
		s = BKE_fracture_shard_boolean(ob, *dm_parent, t, inner_mat_index, 0, 0.0f, &s2, NULL, 0.0f, false, 0);
	}
	else
	{
		s = BKE_fracture_shard_boolean(ob, *dm_parent, t, inner_mat_index, 0, 0.0f, NULL, NULL, 0.0f, false, 0);
	}

	//printf("Fractured: %d\n", k);

	if (s != NULL) {
		add_shard(fs->frac_mesh, s, mat);
		shards++;
		s = NULL;
	}

	if (s2 != NULL) {
		add_shard(fs->frac_mesh, s2, mat);
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



static void intersect_shards_by_dm(Object *ob, Object *ob2, DerivedMesh *d, short inner_mat_index, float mat[4][4],
                                   bool keep_other_shard)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;

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

	count = fs->frac_mesh->shard_count;

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
		do_intersect(ob, t, inner_mat_index, is_zero, mat, &shard_counts, &count, k, &dm_parent, keep_other_shard);
	}

	for (k = 0; k < count; k++)
	{
		int cnt = shard_counts[k];

		if (cnt > 0)
		{
			if (keep_other_shard)
			{
				/*clean up old entries here to avoid unnecessary shards*/
				Shard *first = fs->frac_mesh->shard_map.first;
				BLI_remlink_safe(&fs->frac_mesh->shard_map,first);
				BKE_shard_free(first, true);
				first = NULL;
			}

			/* keep asynchronous by intent, to keep track of original shard count */
			fs->frac_mesh->shard_count--;
		}
	}

	MEM_freeN(shard_counts);
	shard_counts = NULL;

	BKE_shard_free(t, true);
}

void BKE_fracture_shard_by_greasepencil(Object *obj, short inner_material_index, float mat[4][4])
{
	bGPDlayer *gpl;
	bGPDframe *gpf;
	bGPDstroke *gps;

	if ((obj->gpd) && (obj->gpd->layers.first)) {

		float imat[4][4];
		invert_m4_m4(imat, mat);
		for (gpl = obj->gpd->layers.first; gpl; gpl = gpl->next) {
			for (gpf = gpl->frames.first; gpf; gpf = gpf->next) {
				for (gps = gpf->strokes.first; gps; gps = gps->next) {
					BMesh *bm = BM_mesh_create(&bm_mesh_allocsize_default);
					DerivedMesh *dm = NULL;

					/*create stroke mesh */
					stroke_to_faces(obj, &bm, gps, inner_material_index);
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
					intersect_shards_by_dm(obj, NULL, dm, inner_material_index, mat, true);

					dm->needsFree = 1;
					dm->release(dm);
					dm = NULL;
				}
			}
		}
	}
}

void BKE_fracture_shard_by_planes(Object *obj, short inner_material_index, float mat[4][4])
{
	FractureContainer *fc = obj->rigidbody_object->fracture_objects;
	if (fc->frac_algorithm == MOD_FRACTURE_BOOLEAN && fc->cutter_group != NULL && obj->type == OB_MESH)
	{
		GroupObject* go;
		float imat[4][4];

		invert_m4_m4(imat, obj->obmat);

		for (go = fc->cutter_group->gobject.first; go; go = go->next)
		{
			Object* ob = go->ob;

			printf("Cutting with %s ...\n", ob->id.name);
			/*simple case....one cutter object per object*/
			if (ob->type == OB_MESH) {

				FractureContainer *fc2 = ob->rigidbody_object->fracture_objects;
				FractureState *fs2 = fc2->current;
				FracMesh *fm = fs2->frac_mesh;
				if (fc2 && BLI_listbase_count(&fm->shard_map) > 0)
				{
					MeshIsland* mi = NULL;
					int j = 0;
					//another inner loop over fracture settings necessary, TODO
					for (mi = fs2->island_map.first; mi; mi = mi->next)
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

						intersect_shards_by_dm(obj, ob, dm, inner_material_index, mat, false);

						dm->needsFree = 1;
						dm->release(dm);
						dm = NULL;
						j++;
					}

					/*now delete first shards, those are the old ones */
					while (fm->shard_count > 0)
					{
						/*clean up old entries here to avoid unnecessary shards*/
						Shard *first = fm->shard_map.first;
						BLI_remlink_safe(&fm->shard_map,first);
						BKE_shard_free(first, true);
						first = NULL;
						fm->shard_count--;
					}

					/* re-synchronize counts, was possibly different before */
					fm->shard_count = BLI_listbase_count(&fm->shard_map);
				}
				else
				{

					DerivedMesh *d;
					d = ob->derivedFinal; //TODO ensure this !
					if (d == NULL) {
						d = CDDM_from_mesh(ob->data);
					}

					intersect_shards_by_dm(obj, ob, d, inner_material_index, mat, true);

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

void BKE_fracture_shard_by_points(Object *obj, ShardID id, FracPointCloud *pointcloud, short inner_material_index, float mat[4][4]) {

	int n_size = 8;
	Shard *shard;
	
	float min[3], max[3];
	float theta = 0.1f; /* TODO, container enlargement, because boundbox exact container and boolean might create artifacts */
	int p;
	
	container *voro_container;
	particle_order *voro_particle_order;
	cell *voro_cells;

	FractureContainer *fc = obj->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	FracMesh *fmesh = fs->frac_mesh;

#ifdef USE_DEBUG_TIMER
	double time_start;
#endif
	
	shard = BKE_shard_by_id(fmesh, id);
	if (!shard /*|| shard->flag & SHARD_FRACTURED*/)
		return;

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
	parse_cells(voro_cells, pointcloud->totpoints, id, obj, inner_material_index, mat);

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


static void do_marking(Object* ob, DerivedMesh *result)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;

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

			if (fc->flag & FM_FLAG_USE_SMOOTH)
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

static DerivedMesh* do_create(FracMesh *frac_mesh, int num_verts, int num_loops, int num_polys,
                              bool doCustomData)
{
	int shard_count = 0;
	ListBase *shardlist;
	Shard *shard;

	int vertstart, polystart, loopstart;

	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;

	DerivedMesh *result = NULL;

	shard_count = BLI_listbase_count(&frac_mesh->shard_map);

	result = CDDM_new(num_verts, 0, 0, num_loops, num_polys);
	mverts = CDDM_get_verts(result);
	mloops = CDDM_get_loops(result);
	mpolys = CDDM_get_polys(result);

	if (doCustomData && shard_count > 0) {
		Shard *s = frac_mesh->shard_map.first;

		CustomData_merge(&s->vertData, &result->vertData, CD_MASK_MDEFORMVERT, CD_CALLOC, num_verts);
		CustomData_merge(&s->polyData, &result->polyData, CD_MASK_MTEXPOLY, CD_CALLOC, num_polys);
		CustomData_merge(&s->loopData, &result->loopData, CD_MASK_MLOOPUV, CD_CALLOC, num_loops);
	}

	vertstart = polystart = loopstart = 0;
	shardlist = &frac_mesh->shard_map;

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

	return result;
}

/* DerivedMesh */
DerivedMesh *BKE_fracture_create_dm(Object *ob, FracMesh *fm, bool doCustomData)
{
	Shard *s;
	int num_verts, num_polys, num_loops;
	DerivedMesh *result;
	
	num_verts = num_polys = num_loops = 0;

	for (s = fm->shard_map.first; s; s = s->next) {
		num_verts += s->totvert;
		num_polys += s->totpoly;
		num_loops += s->totloop;
	}
	
	result = do_create(fm, num_verts, num_loops, num_polys, doCustomData);

	CustomData_free(&result->edgeData, 0);
	CDDM_calc_edges(result);

	if (ob)
		do_marking(ob, result);
	
	result->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(result);
	return result;
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

void BKE_lookup_mesh_state(Object* ob, int frame)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->states.first;

	//TODO, simple but slow... maybe create a GHash ? or rebuild a quick access array when adding ? realloc and add
	while (fs && (frame > fs->frame))
	{
		fs = fs->next;
	}

	fc->current = fs;
}
#if 0
void BKE_match_vertex_coords(MeshIsland* mi, MeshIsland *par, Object *ob, int frame, bool is_parent)
{
	float loc[3] = {0.0f, 0.0f, 0.0f};
	float rot[4] = {0.0f, 0.0f, 0.0f, 1.0f};
	int j = 0;
	//float irot[4];
	float centr[3] = {0.0f, 0.0f, 0.0f};

	invert_m4_m4(ob->imat, ob->obmat);

	loc[0] = par->locs[3*frame];
	loc[1] = par->locs[3*frame+1];
	loc[2] = par->locs[3*frame+2];

	rot[0] = par->rots[4*frame];
	rot[1] = par->rots[4*frame+1];
	rot[2] = par->rots[4*frame+2];
	rot[3] = par->rots[4*frame+3];

	mi->locs[0] = loc[0];
	mi->locs[1] = loc[1];
	mi->locs[2] = loc[2];

	mi->rots[0] = rot[0];
	mi->rots[1] = rot[1];
	mi->rots[2] = rot[2];
	mi->rots[3] = rot[3];

	mul_m4_v3(ob->imat, loc);
	//mat4_to_quat(irot, ob->imat);
	//mul_qt_qtqt(rot, rot, irot);

	mul_qt_qtqt(rot, rot, par->rot);

	if (is_parent)
	{
		copy_v3_v3(centr, mi->centroid);
		mul_qt_v3(rot, centr);
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
		mul_qt_v3(rot, co);
		add_v3_v3(co, centr);
		copy_v3_v3(mi->vertices_cached[j]->co, co);

		co[0] = mi->vertco[3*j];
		co[1] = mi->vertco[3*j+1];
		co[2] = mi->vertco[3*j+2];

		sub_v3_v3(co, mi->centroid);
		mul_qt_v3(rot, co);
		add_v3_v3(co, centr);

		mi->vertco[3*j]   = co[0];
		mi->vertco[3*j+1] = co[1];
		mi->vertco[3*j+2] = co[2];
	}

	//init rigidbody properly ?
	copy_v3_v3(mi->centroid, centr);
	copy_qt_qt(mi->rot, rot);
}
#endif

static void free_constraint_container(Scene *scene, Object* ob)
{
	MeshIsland *mi = NULL;
	RigidBodyShardCon *rbsc = NULL;
	ConstraintContainer *cc = ob->rigidbody_constraint->fracture_constraints;
	Object *ob1 = ob->rigidbody_constraint->ob1;
	Object *ob2 = ob->rigidbody_constraint->ob2;
	FractureState *fs;

	if (cc->constraint_map.first == NULL)
	{
		/* we just loaded the blend and dont have any constraints yet, so dont attempt to free here*/
		return;
	}

	if (ob1 && ob1->rigidbody_object && ob1->rigidbody_object->fracture_objects && !scene)
	{
		FractureContainer *fc1 = ob1->rigidbody_object->fracture_objects;
		for (fs = fc1->states.first; fs; fs = fs->next)
		{
			for (mi = fs->island_map.first; mi; mi = mi->next) {
				if (mi->participating_constraints != NULL) {
					MEM_freeN(mi->participating_constraints);
					mi->participating_constraints = NULL;
					mi->participating_constraint_count = 0;
				}
			}
		}
	}

	if (ob2 && ob2->rigidbody_object && ob2->rigidbody_object->fracture_objects && !scene)
	{
		FractureContainer *fc2 = ob2->rigidbody_object->fracture_objects;
		if (ob1 != ob2)
		{
			for (fs = fc2->states.first; fs; fs = fs->next)
			{
				for (mi = fs->island_map.first; mi; mi = mi->next) {
					if (mi->participating_constraints != NULL) {
						MEM_freeN(mi->participating_constraints);
						mi->participating_constraints = NULL;
						mi->participating_constraint_count = 0;
					}
				}
			}
		}
	}

	rbsc = cc->constraint_map.first;
	while (rbsc) {
		if (scene)
		{
			BKE_rigidbody_remove_shard_con(scene->rigidbody_world, rbsc);
			rbsc = rbsc->next;
		}
		else
		{
			BLI_remlink(&cc->constraint_map, rbsc);
			MEM_freeN(rbsc);
			rbsc = cc->constraint_map.first;
		}
	}

	if (!scene)
	{
		cc->constraint_map.first = NULL;
		cc->constraint_map.last = NULL;
	}
}

/* inlined select_linked functionality here, because not easy to reach without modifications */
void BKE_select_linked(BMesh **bm_in)
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

void BKE_mesh_separate_selected(BMesh **bm_work, BMesh **bm_out, BMVert **orig_work, BMVert ***orig_out1, BMVert ***orig_out2)
{
	BMesh *bm_old = *bm_work;
	BMesh *bm_new = *bm_out;
	BMVert *v, **orig_new = NULL, **orig_mod = NULL;
	BMIter iter;
	int new_index = 0, mod_index = 0;

	if (orig_work && orig_out1 && orig_out2)
	{
		orig_new = *orig_out1, orig_mod = *orig_out2;
	}

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

	if (orig_work && orig_new && orig_mod)
	{
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
	}

	new_index = 0;
	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
		v->head.index = new_index;
		new_index++;
	}

	if (orig_work && orig_new && orig_mod)
	{
		BMO_op_callf(bm_old, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	             "delete geom=%hvef context=%i", BM_ELEM_TAG, DEL_FACES);
	}

	/* deselect loose data - this used to get deleted,
	 * we could de-select edges and verts only, but this turns out to be less complicated
	 * since de-selecting all skips selection flushing logic */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);

	BM_mesh_normals_update(bm_new);
}

static void build_constraints(Object *ob)
{
	GHash *vertex_island_map = NULL;
	KDTree *coord_tree = NULL;
	int count = 0;
	MeshIsland **mesh_islands = NULL;
	ConstraintContainer *cc = ob->rigidbody_constraint->fracture_constraints;
	Object* ob1 = ob->rigidbody_constraint->ob1;
	Object* ob2 = ob->rigidbody_constraint->ob2;
	bool outer = ob1 != ob2;
	FractureContainer* fc1, *fc2;
	int totvert1, totvert2, island_count1 = 0, island_count2 = 0;
	double start;

	if (!(cc->flag & FMC_FLAG_USE_CONSTRAINTS) && !(ob->rigidbody_constraint->flag & RBC_FLAG_ENABLED))
	{
		return;
	}

	if (!ob1 || !ob2 || !ob1->rigidbody_object || !ob2->rigidbody_object)
	{
		return;
	}

	fc1 = ob1->rigidbody_object->fracture_objects;
	fc2 = ob2->rigidbody_object->fracture_objects;
	totvert1 = fc1->current->visual_mesh->numVertData;
	totvert2 = fc2->current->visual_mesh->numVertData;

	//execute this on constraint object (without modifier)
	start = PIL_check_seconds_timer();

	island_count1 = fc1->current->island_count;
	if (outer) {
		island_count2 = fc2->current->island_count;
	}

	if (cc->constraint_target == MOD_FRACTURE_CENTROID)
	{
		count = BLI_listbase_count(&fc1->current->frac_mesh->shard_map);
		if (outer) {
			count += BLI_listbase_count(&fc2->current->frac_mesh->shard_map);
		}
	}
	else if (cc->constraint_target == MOD_FRACTURE_VERTEX)
	{
		count = totvert1;
		if (outer) {
			count += totvert2;
		}
	}

	if (outer)
	{
		vertex_island_map = BLI_ghash_int_new("vertex_island_map");
	}

	coord_tree = BLI_kdtree_new(count);
	mesh_islands = MEM_callocN(sizeof(MeshIsland*) * (island_count1 + island_count2), "mesh_islands(constraints)");
	do_prepare_constraint_search(ob, &mesh_islands, &coord_tree, &vertex_island_map, 0, cc->constraint_target, 1);
	if (outer) {
		do_prepare_constraint_search(ob, &mesh_islands, &coord_tree, &vertex_island_map, island_count1, cc->constraint_target, 2);
	}

	BLI_kdtree_balance(coord_tree);
	printf("Preparing constraints done, %g\n", PIL_check_seconds_timer() - start);

	start = PIL_check_seconds_timer();
	create_constraints(ob, mesh_islands, count, coord_tree, vertex_island_map, cc->constraint_target);
	/* check for actually creating the constraints inside*/
	printf("Building constraints done, %g\n", PIL_check_seconds_timer() - start);
	printf("Constraints: %d\n", BLI_listbase_count(&cc->constraint_map));

	BLI_kdtree_free(coord_tree);
	MEM_freeN(mesh_islands);
	if ((vertex_island_map != fc1->vertex_island_map) && (vertex_island_map != fc2->vertex_island_map))
	{
		if (vertex_island_map)
		{
			BLI_ghash_free(vertex_island_map, NULL, NULL);
			vertex_island_map = NULL;
		}
	}
}

void BKE_fracture_constraint_container_update(Object* ob)
{
	RigidBodyCon *rbc = ob->rigidbody_constraint;
	if (rbc && rbc->ob1 && rbc->ob2 && rbc->ob1->rigidbody_object && rbc->ob2->rigidbody_object)
	{
		ConstraintContainer *cc = rbc->fracture_constraints;
		rbc->flag |= RBC_FLAG_NEEDS_VALIDATE;
		if (cc->flag & FM_FLAG_REFRESH_CONSTRAINTS) {
			BKE_fracture_constraint_container_free(ob);
			do_clusters(rbc->ob1);
			if (rbc->ob1 != rbc->ob2)
				do_clusters(rbc->ob2);
			build_constraints(ob);
			cc->flag &= ~FM_FLAG_REFRESH_CONSTRAINTS;
		}
	}
}

void BKE_fracture_prefracture_mesh(Scene* scene, Object *ob, ShardID id)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;

	if (thread_sentinel(ob))
		return;

	fc->raw_mesh = BKE_fracture_ensure_mesh(scene, ob);

#if 0
	//very first, purge constraints... we are member in
	for (i = 0; i < fc->constraint_container_count; i++)
	{
		if (rbw && rbw->objects)
		{
			Object *ob = rbw->objects[fc->constraint_containers[i]];
		}
	}
#endif

	//first, delete all rigidbody data (?)
	if (id == 0)
	{
		//restore from original mesh....
		//if scene is NULL here dont touch rigidbodies at free (crashes)
		free_fracture_state((fc->flag & FM_FLAG_EXECUTE_THREADED) ? NULL : scene, fs, true);
		initialize_shard(ob);
	}
	else
	{	//dynamic case
		free_fracture_state(scene, fs, false);
	}

//	preprocess_dm(ob);
	//perform intial halving... or do clean...

	//operate on existing shard;
	do_modifier(scene, ob);
	BKE_fracture_create_islands(ob, false);
	//do_clear(ob); wtf....

	//update_islands(ob);

	//here reset all caches FM_TODO (elsewhere)
	//BKE_rigidbody_cache_reset(scene->rigidbody_world);

	//sync to be sure
	fs->frac_mesh->shard_count = BLI_listbase_count(&fs->frac_mesh->shard_map);

#if 0
	if (ob->type != OB_MESH)
	{
		/* free newly created derivedmeshes only, but keep derivedData and final_dm*/
		if ((clean_dm != group_dm) && (clean_dm != derivedData) && (clean_dm != final_dm))
		{
			clean_dm->needsFree = 1;
			clean_dm->release(clean_dm);
		}
	}

	if ((group_dm != derivedData) && (group_dm != final_dm))
	{
		group_dm->needsFree = 1;
		group_dm->release(group_dm);
	}

	return final_dm;
#endif

}

void BKE_dynamic_fracture_mesh(Scene* scene, Object *ob, ShardID id)
{
	add_fracture_state(scene, ob);
	BKE_fracture_prefracture_mesh(scene, ob, id);
}

ConstraintContainer* BKE_fracture_constraint_container_create(Object* ob)
{
	ConstraintContainer *cc = MEM_callocN(sizeof(ConstraintContainer) ,"fracture_constraints");
	cc->breaking_threshold = 10.0f;
	cc->contact_dist = 1.0f;
	cc->constraint_limit = 50;
	cc->cluster_breaking_threshold = 1000.0f;
	cc->flag |= FMC_FLAG_USE_BREAKING;
	cc->constraint_target = MOD_FRACTURE_CENTROID;
	return cc;
}

void BKE_fracture_constraint_container_free(Object *ob)
{
	if (ob->rigidbody_constraint->fracture_constraints) {
		if (ob->rigidbody_constraint->fracture_constraints->flag & FM_FLAG_REFRESH_CONSTRAINTS)
		{
			free_constraint_container(NULL, ob);
			//ob->rigidbody_constraint->fracture_constraints = NULL;
		}
	}
}

static void initialize_shard(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	DerivedMesh *dm = fc->raw_mesh;

	//init fracmesh with entire shard from ob->derivedFinal (ensure it)
	float mat[4][4]; //wood splinter scaling matrix, here it is unit_m4
	Shard *s;

	s = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
	                              dm->getNumVerts(dm), dm->getNumPolys(dm), dm->getNumLoops(dm), true);

	s = BKE_custom_data_to_shard(s, dm);
	fs->frac_mesh = BKE_create_fracmesh();
	unit_m4(mat);
	add_shard(fs->frac_mesh, s, mat);
}

static void update_islands(Object *ob)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	FractureState *fs = fc->current;
	int count, i = 0;
	MeshIsland *mi;

	//create fast access array
	count = BLI_listbase_count(&fs->island_map);
	fs->islands = MEM_callocN(sizeof(MeshIsland*) * count, "fs->islands");

	for (mi = fs->island_map.first; mi; mi = mi->next)
	{
		fs->islands[i] = mi;
		i++;
	}

	fs->island_count = count;
}

FractureContainer* BKE_fracture_container_create(Object *ob)
{
	FractureContainer *fc = MEM_callocN(sizeof(FractureContainer) ,"fracture_objects");
	FractureState *fs = MEM_callocN(sizeof(FractureState), "states.first");

	BLI_addtail(&fc->states, fs);
	fc->current = fs;

	//init pointcaches.... FM_TODO...
	fc->pointcache = BKE_ptcache_add(&(fc->ptcaches));
	fc->pointcache->step = 1;

	//init first shard
	//initialize_shard(scene, ob);

	//set useful defaults...
	fc->frac_algorithm = MOD_FRACTURE_BOOLEAN;
	fc->point_source = MOD_FRACTURE_UNIFORM;
	fc->shard_count = 1;
	fc->percentage = 100;

	/* XXX needed because of messy particle cache, shows incorrect positions when start/end on frame 1
	 * default use case is with this flag being enabled, disable at own risk */
	fc->flag = 0;
	fc->flag |= FM_FLAG_USE_PARTICLE_BIRTH_COORDS;
	fc->splinter_length = 1.0f;
	fc->nor_range = 1.0f;

	fc->fractal_cuts = 1;
	fc->fractal_amount = 1.0f;
	fc->fractal_iterations = 5;

	fc->grease_decimate = 100.0f;
	fc->grease_offset = 100.0f;
	fc->flag |= FM_FLAG_USE_GREASEPENCIL_EDGES;
	fc->cutter_axis = MOD_FRACTURE_CUTTER_Z;

	fc->fracture_mode = MOD_FRACTURE_PREFRACTURED;
	fc->dynamic_force = 10.0f;

	fc->effector_weights = BKE_add_effector_weights(NULL);
	fc->raw_mesh = NULL;

	return fc;
}

void BKE_fracture_container_initialize(Object* ob, DerivedMesh *dm)
{
	FractureContainer *fc = ob->rigidbody_object->fracture_objects;
	if (fc)
	{
		fc->raw_mesh = dm; //borrowed only
		//create meshislands
		//BKE_fracture_prefracture_mesh(scene, ob, 0);
		initialize_shard(ob);
		BKE_fracture_create_islands(ob, true); //FM_TODO rename this like initialize_islands()
	}
}

void BKE_fracture_container_free(Object *ob)
{
	if (ob->rigidbody_object->fracture_objects) {
		free_fracture_container(NULL, ob->rigidbody_object->fracture_objects);
		ob->rigidbody_object->fracture_objects = NULL;
	}
}

MVert* BKE_copy_visual_mesh(Object* ob, FractureState *fs)
{
	/* re-init cached verts here... before rebuild a visual mesh on the fly */
	fs->visual_mesh = BKE_fracture_create_dm(ob, fs->frac_mesh, true);
	DM_ensure_tessface(fs->visual_mesh);
	DM_ensure_normals(fs->visual_mesh);
	DM_update_tessface_data(fs->visual_mesh);

	return CDDM_get_verts(fs->visual_mesh);
}

int BKE_initialize_meshisland(MeshIsland** mii, MVert* mverts, int vertstart)
{
	int k = 0;
	MeshIsland* mi = *mii;

	mi->vertices_cached = MEM_mallocN(sizeof(MVert*) * mi->vertex_count, "mi->vertices_cached readfile");
	mi->vertex_indices = MEM_mallocN(sizeof(int) * mi->vertex_count, "mi->vertex_indices");

	for (k = 0; k < mi->vertex_count; k++) {
		MVert* v = mverts + vertstart + k ;
		mi->vertices_cached[k] = v;
		mi->vertex_indices[k] = vertstart + k;

#if 0
		if ((mi->vertnos != NULL) && (fc->flag & FM_FLAG_FIX_NORMALS)) {
			copy_v3_v3_short(v->no, mi->vertnos[k]);
		}
#endif

		//copy_v3_v3(mi->vertcos[k], v->co);
		//copy_v3_v3_short(mi->vertnos[k], v->no);
	}

	return mi->vertex_count;
}

#if 0
static void copyData(ModifierData *md, ModifierData *target)
{
	FractureModifierData *rmd  = (FractureModifierData *)md;
	FractureModifierData *trmd = (FractureModifierData *)target;

	/*todo -> copy fracture stuff as well, and dont forget readfile / writefile...*/
	zero_m4(trmd->origmat);

	/* vgroups  XXX TODO non ascii strings ?*/
	strncpy(trmd->fracture->thresh_defgrp_name, rmd->fracture->thresh_defgrp_name, strlen(rmd->fracture->thresh_defgrp_name));
	strncpy(trmd->fracture->ground_defgrp_name, rmd->fracture->ground_defgrp_name, strlen(rmd->fracture->ground_defgrp_name));
	strncpy(trmd->fracture->inner_defgrp_name, rmd->fracture->inner_defgrp_name, strlen(rmd->fracture->inner_defgrp_name));

	trmd->fracture->visible_mesh = NULL;
	trmd->fracture->visible_mesh_cached = NULL;
	trmd->fracture->meshIslands.first = NULL;
	trmd->fracture->meshIslands.last = NULL;
	trmd->constraint->meshConstraints.first = NULL;
	trmd->constraint->meshConstraints.last = NULL;
	trmd->fracture->face_pairs = NULL;
	trmd->vert_index_map = NULL;
	trmd->fracture->vertex_island_map = NULL;

	trmd->constraint->breaking_threshold = rmd->constraint->breaking_threshold;
	trmd->constraint->flag = rmd->constraint->flag;
	trmd->constraint->contact_dist = rmd->constraint->contact_dist;
	trmd->flag = rmd->flag;
	trmd->fracture->flag = rmd->fracture->flag;

	trmd->fracture->flag &= ~FM_FLAG_REFRESH;
	trmd->constraint->constraint_limit = rmd->constraint->constraint_limit;
	trmd->constraint->breaking_angle = rmd->constraint->breaking_angle;
	trmd->constraint->breaking_distance = rmd->constraint->breaking_distance;
	trmd->constraint->breaking_percentage = rmd->constraint->breaking_percentage;
	trmd->fracture->flag &= ~FM_FLAG_REFRESH_CONSTRAINTS;

	trmd->constraint->cluster_count = rmd->constraint->cluster_count;
	trmd->constraint->cluster_breaking_threshold = rmd->constraint->cluster_breaking_threshold;
	trmd->constraint->solver_iterations_override = rmd->constraint->solver_iterations_override;

	trmd->fracture->shard_count = rmd->fracture->shard_count;
	trmd->fracture->frac_algorithm = rmd->fracture->frac_algorithm;

	trmd->fracture->autohide_dist = rmd->fracture->autohide_dist;
	trmd->fracture->point_seed = rmd->fracture->point_seed;
	trmd->fracture->point_source = rmd->fracture->point_source;

	/*id refs ?*/
	trmd->fracture->inner_material = rmd->fracture->inner_material;
	trmd->fracture->extra_group = rmd->fracture->extra_group;

	/* sub object group  XXX Do we keep this ?*/
	trmd->dm_group = rmd->dm_group;

	trmd->constraint->cluster_group = rmd->constraint->cluster_group;
	trmd->fracture->cutter_group = rmd->fracture->cutter_group;

	trmd->fracture->splinter_length = rmd->fracture->splinter_length;
	trmd->constraint->cluster_solver_iterations_override = rmd->constraint->cluster_solver_iterations_override;

	trmd->constraint->cluster_breaking_angle = rmd->constraint->cluster_breaking_angle;
	trmd->constraint->cluster_breaking_distance = rmd->constraint->cluster_breaking_distance;
	trmd->constraint->cluster_breaking_percentage = rmd->constraint->cluster_breaking_percentage;

	trmd->fracture->fractal_cuts = rmd->fracture->fractal_cuts;
	trmd->fracture->fractal_amount = rmd->fracture->fractal_amount;

	trmd->fracture->grease_decimate = rmd->fracture->grease_decimate;
	trmd->fracture->grease_offset = rmd->fracture->grease_offset;
	trmd->fracture->cutter_axis = rmd->fracture->cutter_axis;

	trmd->constraint->cluster_constraint_type = rmd->constraint->cluster_constraint_type;
	trmd->constraint->constraint_target = rmd->constraint->constraint_target;

	trmd->fracture_mode = rmd->fracture_mode;
	trmd->last_frame = rmd->last_frame;
	trmd->fracture->dynamic_force = rmd->fracture->dynamic_force;

	trmd->fracture->flag &= ~FM_FLAG_UPDATE_DYNAMIC;
	trmd->fracture->flag &= ~FM_FLAG_RESET_SHARDS;
}
#endif

static ConstraintContainer *copy_constraint_container(ConstraintContainer *cc)
{
	ConstraintContainer *ccN = MEM_dupallocN(cc);

	//ccN->con_settings = MEM_dupallocN(cc->con_settings);
	//ccN->con_settings->flag |= RBC_FLAG_NEEDS_VALIDATE;

	return ccN;
}

static void copy_mesh_island(Object *ob, MeshIsland *miN, MeshIsland *mi)
{
	miN->vertcos = MEM_dupallocN(miN->vertcos);
	miN->vertnos = MEM_dupallocN(miN->vertnos);
	miN->vertex_indices = NULL;
	miN->vertices_cached = NULL;
	miN->vertex_count = mi->vertex_count;

	miN->bb = MEM_dupallocN(mi->bb);
	miN->physics_mesh = CDDM_copy(mi->physics_mesh);
	miN->participating_constraints = MEM_dupallocN(mi->participating_constraints);
	miN->participating_constraint_count = mi->participating_constraint_count;
	miN->linear_index = mi->linear_index;
	miN->thresh_weight = mi->thresh_weight;
	copy_v3_v3(miN->centroid, mi->centroid);
	miN->id = mi->id;
	miN->partner_index = mi->partner_index;
	miN->ground_weight = mi->ground_weight;
	miN->particle_index = mi->particle_index;

	//this has been recalculated in the shards, need to update this here too FM_TODO
	//copy_v3_v3(mi->centroid, centroid);

	miN->rigidbody = NULL;
	miN->rigidbody = BKE_rigidbody_create_shard(ob, miN);
	miN->rigidbody->type = mi->rigidbody->type;
	miN->rigidbody->flag = mi->rigidbody->flag;
	miN->rigidbody->mass = mi->rigidbody->mass;
}

static void copy_fracture_state(FractureState *fsN, FractureState *fs, Object *obN)
{
	MeshIsland *mi, *miN;
	MVert *mvert;
	int vertstart = 0, i = 0;

	//copy shards, meshislands.... and stuff TODO or just nullify! no, need a copy !!!
	fsN->frac_mesh = BKE_copy_fracmesh(fs->frac_mesh);
	fsN->visual_mesh = NULL;
	mvert = BKE_copy_visual_mesh(obN, fsN);

	//s = fsN->frac_mesh->shard_map.first;
	fsN->islands = MEM_callocN(sizeof(MeshIsland*) * fs->island_count, "fsN->islands");
	fsN->island_count = fs->island_count;

	for (mi = fs->island_map.first; mi; mi = mi->next)
	{
		miN = MEM_callocN(sizeof(MeshIsland), "copy_fracture_state, mi");
		copy_mesh_island(obN, miN, mi /*s->centroid*/);
		vertstart += BKE_initialize_meshisland(&miN, mvert, vertstart);
		BLI_addtail(&fsN->island_map, miN);
		fsN->islands[i] = miN;
		i++;
	}
}


static FractureContainer* copy_fracture_container(Object* ob, Object *obN)
{
	RigidBodyOb *rb = ob->rigidbody_object;
	FractureContainer *fc = rb->fracture_objects;
	FractureContainer *fcN = MEM_dupallocN(fc);
	FractureState *fsN, *fs;

	fcN->raw_mesh = NULL;
	fcN->states.first = NULL;
	fcN->states.last = NULL;

	//fs = fc->states.first;
	for (fs = fc->states.first; fs; fs = fs->next)
	{
		fsN = MEM_callocN(sizeof(FractureState), "fsN");
		copy_fracture_state(fsN, fs, obN);
		BLI_addtail(&fcN->states, fsN);
	}

	fcN->current = fcN->states.first;

	fcN->pointcache = BKE_ptcache_copy_list(&fcN->ptcaches, &fc->ptcaches, false);
	fcN->effector_weights = MEM_dupallocN(fc->effector_weights);
	make_face_pairs(obN);

	rb->flag |= RBO_FLAG_NEEDS_VALIDATE;

	return fcN;
}

FractureContainer *BKE_fracture_container_copy(Object *ob, Object *obN)
{
	return copy_fracture_container(ob, obN);
}

/* get the appropriate DerivedMesh based on rigid body mesh source */
DerivedMesh *BKE_fracture_ensure_mesh(Scene* scene, Object* ob)
{
	CustomDataMask mask = CD_MASK_BAREMESH;
	RigidBodyOb *rb = ob->rigidbody_object;
	FractureContainer *fc = rb->fracture_objects;
	DerivedMesh *dm = NULL;

	if (rb->mesh_source == RBO_MESH_DEFORM) {
		fc->flag |= FM_FLAG_REFRESH;

		if (ob->derivedDeform)
		{
			ob->derivedDeform->needsFree = 1;
			DM_release(ob->derivedDeform);
			ob->derivedDeform = NULL;
		}

		dm = mesh_get_derived_deform(scene, ob, mask);

		fc->flag &= ~FM_FLAG_REFRESH;

		return CDDM_copy(dm);
	}
	else if (rb->mesh_source == RBO_MESH_FINAL) {

		fc->flag |= FM_FLAG_REFRESH;

		if (ob->derivedFinal)
		{
			ob->derivedFinal->needsFree = 1;
			DM_release(ob->derivedFinal);
			ob->derivedFinal = NULL;
		}

		dm = mesh_get_derived_final(scene, ob, mask);

		fc->flag &= ~FM_FLAG_REFRESH;

		return CDDM_copy(dm);
	}
	else {
		return CDDM_from_mesh(ob->data);
	}
}


void BKE_fracture_constraint_container_empty(Scene *scene, Object *ob)
{
	free_constraint_container(scene, ob);
}

void BKE_fracture_container_empty(Scene *scene, Object *ob)
{
	free_fracture_container(scene, ob->rigidbody_object->fracture_objects);
}

void BKE_fracture_relink_cache(Scene *scene, Object *ob, bool remove)
{
	RigidBodyWorld *rbw = scene->rigidbody_world;

	if (rbw && rbw->group)
	{
		GroupObject *first = rbw->group->gobject.first;
		if (first && (first->ob != ob || !remove))
		{
			if (first->ob && first->ob->rigidbody_object)
			{
				FractureContainer *fc = first->ob->rigidbody_object->fracture_objects;
				if (fc)
				{
					rbw->pointcache = fc->pointcache;
					return;
				}
			}
		}
	}

	rbw->pointcache = NULL;
}

void BKE_fracture_synchronize_caches(Scene* scene)
{
	//set start and endframe equally to all involved caches
	RigidBodyWorld *rbw = scene->rigidbody_world;
	PointCache *cache = rbw->pointcache;

	if (rbw && rbw->group && !cache)
	{
		GroupObject *go = rbw->group->gobject.first;
		FractureContainer *fc = go->ob->rigidbody_object->fracture_objects;
		rbw->pointcache = cache = fc->pointcache;
	}

	if (rbw && rbw->group && cache)
	{
		GroupObject *go;
		for (go = rbw->group->gobject.first; go; go = go->next)
		{
			if (go->ob && go->ob->rigidbody_object)
			{
				FractureContainer *fc = go->ob->rigidbody_object->fracture_objects;
				if (fc && (fc->pointcache != rbw->pointcache))
				{
					PTCacheID *pid = NULL;

					pid = MEM_callocN(sizeof(PTCacheID), "PTCacheID");
					BKE_ptcache_id_from_rigidbody(pid, go->ob, fc);

					pid->cache->startframe = cache->startframe;
					pid->cache->endframe = cache->endframe;
					pid->cache->flag |= PTCACHE_OUTDATED;

					MEM_freeN(pid);

					DAG_id_tag_update(&go->ob->id, OB_RECALC_ALL);
				}
			}
		}

		BKE_scene_frame_set(scene, cache->startframe);
		DAG_id_tag_update(&scene->id, 0);
	}

	if (rbw && rbw->constraints)
	{
		//atleast revalidate all constraints in this cache sync case....
		GroupObject *go;
		for (go = rbw->constraints->gobject.first; go; go = go->next)
		{
			if (go->ob && go->ob->rigidbody_constraint)
			{
				go->ob->rigidbody_constraint->flag |= RBC_FLAG_NEEDS_VALIDATE;
			}
		}
	}
}
