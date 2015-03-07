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
}

static void freeMeshIsland(FractureModifierData *rmd, MeshIsland *mi, bool remove_rigidbody)
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

	if (mi->rots)
	{
		MEM_freeN(mi->rots);
		mi->rots = NULL;
	}

	if (mi->locs)
	{
		MEM_freeN(mi->locs);
		mi->locs = NULL;
	}

	mi->frame_count = 0;

	MEM_freeN(mi);
	mi = NULL;
}

static void freeData_internal(ModifierData *md)
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
		if (rmd->shards_to_islands) {
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
			freeMeshIsland(rmd, mi, false);
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

static void freeData(ModifierData *md)
{
	FractureModifierData *rmd = (FractureModifierData *) md;
	MeshIsland *mi;

	freeData_internal(md);

	/*force deletion of meshshards here, it slips through improper state detection*/
	/*here we know the modifier is about to be deleted completely*/
	if (rmd->frac_mesh) {
		BKE_fracmesh_free(rmd->frac_mesh, true);
		MEM_freeN(rmd->frac_mesh);
		rmd->frac_mesh = NULL;
	}

	while (rmd->meshIslands.first) {
		mi = rmd->meshIslands.first;
		BLI_remlink(&rmd->meshIslands, mi);
		freeMeshIsland(rmd, mi, false);
		mi = NULL;
	}

	rmd->meshIslands.first = NULL;
	rmd->meshIslands.last = NULL;

}

static void doClusters(FractureModifierData *fmd, Object* obj)
{
	/*grow clusters from all meshIslands */
	int k = 0;
	KDTree *tree;
	MeshIsland *mi, **seeds;
	int seed_count;
	ListBase mi_list;

	/*for now, keep 1 hierarchy level only, should be sufficient */
	//int levels = 1;

	mi_list = fmd->meshIslands;

	/*initialize cluster "colors" -> membership of meshislands to clusters, initally all shards are "free" */
	for (mi = mi_list.first; mi; mi = mi->next ) {
		mi->particle_index = -1;
	}

	if (fmd->cluster_group)
	{
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
			for (mi = mi_list.first; mi; mi = mi->next ) {
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
	else
	{
		int mi_count;
		/* zero clusters or one mean no clusters, all shards keep free */
		if (fmd->cluster_count < 2) {
			return;
		}

		mi_count = BLI_listbase_count(&fmd->meshIslands);
		seed_count = (fmd->cluster_count > mi_count ? mi_count : fmd->cluster_count);
		seeds = MEM_mallocN(sizeof(MeshIsland *) * seed_count, "seeds");
		tree = BLI_kdtree_new(seed_count);

		/* pick n seed locations, randomly scattered over the object */
		for (k = 0; k < seed_count; k++) {
			int which_index = k * (int)(mi_count / seed_count);
			MeshIsland *which = (MeshIsland *)BLI_findlink(&mi_list, which_index);
			which->particle_index = k;
			BLI_kdtree_insert(tree, k, which->centroid);
			seeds[k] = which;
		}

		BLI_kdtree_balance(tree);


		/* assign each shard to its closest center */
		for (mi = mi_list.first; mi; mi = mi->next ) {
			KDTreeNearest n;
			int index;

			index = BLI_kdtree_find_nearest(tree, mi->centroid, &n);
			mi->particle_index = seeds[index]->particle_index;
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
			Object *o = go[i];
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

		CustomData_add_layer(&result->vertData, CD_MDEFORMVERT, CD_CALLOC, NULL, num_verts);
		CustomData_add_layer(&result->loopData, CD_MLOOPUV, CD_CALLOC, NULL, num_loops);
		CustomData_add_layer(&result->polyData, CD_MTEXPOLY, CD_CALLOC, NULL, num_polys);

		vertstart = polystart = loopstart = 0;
		for (i = 0; i < totgroup; i++)
		{
			MPoly *mp;
			MLoop *ml;
			int j, v;
			MVert *mv;

			Object *o = go[i];

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

			if (dm_ob == NULL)
			{   /* avoid crash atleast...*/
				return dm;
			}

			memcpy(mverts + vertstart, dm_ob->getVertArray(dm_ob), dm_ob->getNumVerts(dm_ob) * sizeof(MVert));
			memcpy(mpolys + polystart, dm_ob->getPolyArray(dm_ob), dm_ob->getNumPolys(dm_ob) * sizeof(MPoly));

			for (j = 0, mp = mpolys + polystart; j < dm_ob->getNumPolys(dm_ob); ++j, ++mp) {
				/* adjust loopstart index */
				if (CustomData_has_layer(&dm_ob->polyData, CD_MTEXPOLY))
				{
					MTexPoly *mtp = CustomData_get(&dm_ob->polyData, j, CD_MTEXPOLY);
					if (mtp)
						CustomData_set(&result->polyData, polystart + j, CD_MTEXPOLY, mtp);
				}
				mp->loopstart += loopstart;
			}

			memcpy(mloops + loopstart, dm_ob->getLoopArray(dm_ob), dm_ob->getNumLoops(dm_ob) * sizeof(MLoop));

			for (j = 0, ml = mloops + loopstart; j < dm_ob->getNumLoops(dm_ob); ++j, ++ml) {
				/* adjust vertex index */
				if (CustomData_has_layer(&dm_ob->loopData, CD_MLOOPUV))
				{
					MLoopUV *mluv = CustomData_get(&dm_ob->loopData, j, CD_MLOOPUV);
					if (mluv)
						CustomData_set(&result->loopData, loopstart + j, CD_MLOOPUV, mluv);
				}
				ml->v += vertstart;
			}

			for (v = 0, mv = mverts + vertstart; v < dm_ob->getNumVerts(dm_ob); v++, mv++)
			{
				if (CustomData_has_layer(&dm_ob->vertData, CD_MDEFORMVERT))
				{
					MDeformVert *mdv = CustomData_get(&dm_ob->vertData, v, CD_MDEFORMVERT);
					if (mdv)
						CustomData_set(&result->vertData, vertstart + v, CD_MDEFORMVERT, mdv);
				}
				mul_m4_v3(o->obmat, mv->co);
			}

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

	if (emd->point_source & MOD_FRACTURE_GREASEPENCIL && !emd->use_greasepencil_edges) {
		points_from_greasepencil(go, totgroup, &points, ob->obmat, thresh);
	}


	/* local settings, apply per shard!!! Or globally too first. */
	if (emd->point_source & MOD_FRACTURE_UNIFORM)
	{
		int count = emd->shard_count;
		INIT_MINMAX(min, max);
		BKE_get_shard_minmax(emd->frac_mesh, -1, min, max, fracmesh); //id 0 should be entire mesh

		if (emd->frac_algorithm == MOD_FRACTURE_BISECT_FAST || emd->frac_algorithm == MOD_FRACTURE_BISECT_FAST_FILL ||
		    emd->frac_algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL) {
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

static void do_fracture(FractureModifierData *fracmd, ShardID id, Object *obj, DerivedMesh *dm)
{
	/* dummy point cloud, random */
	FracPointCloud points;

	points = get_points_global(fracmd, obj, dm);

	if (points.totpoints > 0 || fracmd->use_greasepencil_edges) {
		bool temp = fracmd->shards_to_islands;
		short mat_index = 0;
		float mat[4][4], imat[4][4];
		float mat2[4][4];

		unit_m4(mat);
		unit_m4(mat2);

		/*splinters... just global axises and a length, for rotation rotate the object */
		if (fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_X)
		{
			mat[0][0] *= fracmd->splinter_length;
		}
		if (fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_Y)
		{
			mat[1][1] *= fracmd->splinter_length;
		}
		if (fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_Z)
		{
			mat[2][2] *= fracmd->splinter_length;
		}


		if ((fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_X) ||
			(fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_Y) ||
			(fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_Z))
		{
			int i = 0;
			MVert* mvert = dm->getVertArray(dm), *mv;
			invert_m4_m4(imat, mat);
			invert_m4_m4(obj->imat, obj->obmat);

			for (i = 0; i < points.totpoints; i++)
			{
				mul_m4_v3(imat, points.points[i].co);
			}

			for (i = 0, mv = mvert; i < dm->getNumVerts(dm); i++, mv++)
			{
				//mul_m4_v3(obj->imat, mv->co);
				mul_m4_v3(imat, mv->co);
			}

			copy_m4_m4(mat2, mat);
		}

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

				fracmd->inner_material = mat_inner;
			}
			else if (*totmat == 1)
			{
				char* matname = BLI_strdupcat(name, "_Inner");
				Material* mat_inner = find_material(matname);
				object_add_material_slot(obj);
				assign_material(obj, mat_inner, obj->totcol, BKE_MAT_ASSIGN_OBDATA);
				MEM_freeN(matname);
				matname = NULL;

				fracmd->inner_material = mat_inner;
			}
			else /*use 2nd material slot*/
			{
				Material* mat_inner = give_current_material(obj, 2);

				fracmd->inner_material = mat_inner;
			}

			mat_index = 2;
		}

		mat_index = mat_index > 0 ? mat_index - 1 : mat_index;

		if (points.totpoints > 0)
		{
			BKE_fracture_shard_by_points(fracmd->frac_mesh, id, &points, fracmd->frac_algorithm, obj, dm, mat_index, mat2,
								 fracmd->fractal_cuts, fracmd->fractal_amount, fracmd->use_smooth, fracmd->fractal_iterations);
		}

		if (fracmd->point_source & MOD_FRACTURE_GREASEPENCIL && fracmd->use_greasepencil_edges)
		{
			BKE_fracture_shard_by_greasepencil(fracmd, obj, mat_index, mat2);
		}

		if (fracmd->frac_algorithm == MOD_FRACTURE_BOOLEAN && fracmd->cutter_group != NULL)
		{
			BKE_fracture_shard_by_planes(fracmd, obj, mat_index, mat2);
		}

		/* job has been cancelled, throw away all data */
		if (fracmd->frac_mesh->cancel == 1)
		{
			fracmd->frac_mesh->running = 0;
			fracmd->refresh = true;
			freeData_internal((ModifierData *)fracmd);
			fracmd->frac_mesh = NULL;
			fracmd->refresh = false;
			MEM_freeN(points.points);
			return;
		}

#if 0
		if (fracmd->frac_mesh->shard_count > 0)
		{
			doClusters(fracmd, obj);
		}
#endif

		/* here we REALLY need to fracture so deactivate the shards to islands flag and activate afterwards */
		fracmd->shards_to_islands = false;
		BKE_fracture_create_dm(fracmd, true);
		fracmd->shards_to_islands = temp;

		if ((fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_X) ||
			(fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_Y) ||
			(fracmd->splinter_axis & MOD_FRACTURE_SPLINTER_Z))
		{

			int i = 0;
			MVert* mvert = dm->getVertArray(dm), *mv;
			for (i = 0, mv = mvert; i < dm->getNumVerts(dm); i++, mv++)
			{
				mul_m4_v3(mat, mv->co);
				//mul_m4_v3(obj->obmat, mv->co);
			}
		}
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

	if ((rmd->shards_to_islands || rmd->frac_mesh->shard_count < 2) && (!rmd->dm_group)) {
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
	mi->locs = MEM_mallocN(sizeof(float)*3, "mi->locs");
	mi->rots = MEM_mallocN(sizeof(float)*4, "mi->rots");
	mi->frame_count = 0;

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

	if (rmd->frac_algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL)
	{
		/* cant be kept together in other ways */
		rmd->use_constraints = true;
		rmd->contact_dist = 2.0f;
		rmd->breaking_angle = DEG2RADF(1.0f);

		/* this most likely will only work with "Mesh" shape*/
		mi->rigidbody->shape = RB_SHAPE_TRIMESH;
		mi->rigidbody->margin = 0.0f;

		/* set values on "handle object" as well */
		ob->rigidbody_object->shape = RB_SHAPE_TRIMESH;
		ob->rigidbody_object->margin = 0.0f;
	}

	mi->start_frame = rmd->modifier.scene->rigidbody_world->pointcache->startframe;


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

static void mesh_separate_loose_partition(FractureModifierData *rmd, Object *ob, BMesh *bm_work, BMVert **orig_work, DerivedMesh *dm)
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
			short vno[3];

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

			normal_float_to_short_v3(vno, v_seed->no);
			if (rmd->fix_normals)
				find_normal(dm, rmd->nor_tree, v_seed->co, vno, no, rmd->nor_range);
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
				short vno[3];

				BM_elem_flag_enable(e->v1, BM_ELEM_TAG);
				BM_elem_flag_enable(e->v1, BM_ELEM_INTERNAL_TAG);
				v_tag = MEM_reallocN(v_tag, sizeof(BMVert *) * (tag_counter + 1));
				v_tag[tag_counter] = orig_work[e->v1->head.index];

				startco = MEM_reallocN(startco, (tag_counter + 1) * 3 * sizeof(float));
				startco[3 * tag_counter] = e->v1->co[0];
				startco[3 * tag_counter + 1] = e->v1->co[1];
				startco[3 * tag_counter + 2] = e->v1->co[2];


				startno = MEM_reallocN(startno, (tag_counter + 1) * 3 * sizeof(short));

				normal_float_to_short_v3(vno, e->v1->no);
				if (rmd->fix_normals)
					find_normal(dm, rmd->nor_tree, e->v1->co, vno, no, rmd->nor_range);
				startno[3 * tag_counter] = no[0];
				startno[3 * tag_counter + 1] = no[1];
				startno[3 * tag_counter + 2] = no[2];

				tot++;
				tag_counter++;
			}
			if (!BM_elem_flag_test(e->v2, BM_ELEM_TAG) && !BM_elem_flag_test(e->v2, BM_ELEM_INTERNAL_TAG)) {
				short no[3];
				short vno[3];

				BM_elem_flag_enable(e->v2, BM_ELEM_TAG);
				BM_elem_flag_enable(e->v2, BM_ELEM_INTERNAL_TAG);

				v_tag = MEM_reallocN(v_tag, sizeof(BMVert *) * (tag_counter + 1));
				v_tag[tag_counter] = orig_work[e->v2->head.index];

				startco = MEM_reallocN(startco, (tag_counter + 1) * 3 * sizeof(float));
				startco[3 * tag_counter] = e->v2->co[0];
				startco[3 * tag_counter + 1] = e->v2->co[1];
				startco[3 * tag_counter + 2] = e->v2->co[2];

				startno = MEM_reallocN(startno, (tag_counter + 1) * 3 * sizeof(short));

				normal_float_to_short_v3(vno, e->v2->no);
				if (rmd->fix_normals)
					find_normal(dm, rmd->nor_tree, e->v2->co, vno, no, rmd->nor_range);
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
			if (thresh == 0 || rmd->use_breaking == false) {
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

	islands = BLI_listbase_count(&rmd->meshIslands);
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
					//v->flag &= ~ME_VERT_TMP_TAG;
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
	bool orig_chosen = false;

	if (rmd->dm && !rmd->shards_to_islands && (rmd->dm->getNumPolys(rmd->dm) > 0)) {
		dm = CDDM_copy(rmd->dm);
	}
	else if (rmd->visible_mesh && (rmd->visible_mesh->totface > 0) && BLI_listbase_count(&rmd->meshIslands) > 1) {
		dm = CDDM_from_bmesh(rmd->visible_mesh, true);
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
		if (rmd->dm != NULL && !rmd->shards_to_islands && !orig_chosen && rmd->visible_mesh == NULL) {
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
					//float no[3];
					short sno[3];
					sno[0] = mi->vertno[i * 3];
					sno[1] = mi->vertno[i * 3 + 1];
					sno[2] = mi->vertno[i * 3 + 2];
					//normal_float_to_short_v3(sno, no);
					copy_v3_v3_short(mi->vertices_cached[i]->no, sno);
				}
			}

			vertstart += mi->vertex_count;
		}
		else {  /* halving case... */
			for (i = 0; i < mi->vertex_count; i++) {
				//float no[3];
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
					sno[0] = mi->vertno[i * 3];
					sno[1] = mi->vertno[i * 3 + 1];
					sno[2] = mi->vertno[i * 3 + 2];
					//normal_float_to_short_v3(sno, no);
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

static void make_face_pairs(FractureModifierData *fmd, DerivedMesh *dm)
{
	/* make kdtree of all faces of dm, then find closest face for each face*/
	MPoly *mp = NULL;
	MPoly *mpoly = dm->getPolyArray(dm);
	MLoop* mloop = dm->getLoopArray(dm);
	MVert* mvert = dm->getVertArray(dm);
	int totpoly = dm->getNumPolys(dm);
	KDTree *tree = BLI_kdtree_new(totpoly);
	int i = 0;

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

	for (i = 0, mp = mpoly; i < totpoly; mp++, i++)
	{
		if (mp->mat_nr == 1)
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

				int j = 0;

				BLI_ghash_insert(fmd->face_pairs, SET_INT_IN_POINTER(i), SET_INT_IN_POINTER(index));

				/*match normals...*/
				if (fmd->fix_normals)
				{
					MLoop ml, ml2;
					MVert *v, *v2;
					short sno[3];
					float fno[3], fno2[3];
					if (mp->totloop == (mpoly+index)->totloop)
					{
						for (j = 0; j < mp->totloop; j++)
						{
							ml = mloop[mp->loopstart + j];
							ml2 = mloop[(mpoly+index)->loopstart + j];
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
			}

			if (n != NULL) {
				MEM_freeN(n);
			}
		}
	}

	BLI_kdtree_free(tree);
}

static DerivedMesh *do_autoHide(FractureModifierData *fmd, DerivedMesh *dm)
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

	BM_mesh_elem_hflag_disable_all(bm, BM_FACE | BM_EDGE | BM_VERT , BM_ELEM_SELECT, false);

	for (i = 0; i < totpoly; i++) {
		BMFace *f1, *f2;
		other = GET_INT_FROM_POINTER(BLI_ghash_lookup(fmd->face_pairs, SET_INT_IN_POINTER(i)));

		if (other == i)
		{
			continue;
		}

		f1 = BM_face_at_index(bm, i);
		f2 = BM_face_at_index(bm, other);

		if ((f1 == NULL) || (f2 == NULL)) {
			continue;
		}

		BM_face_calc_center_mean(f1, f_centr);
		BM_face_calc_center_mean(f2, f_centr_other);


		if ((len_squared_v3v3(f_centr, f_centr_other) < (fmd->autohide_dist)) && (f1 != f2) &&
		    (f1->mat_nr == 1) && (f2->mat_nr == 1))
		{
			/*intact face pairs */
			faces = MEM_reallocN(faces, sizeof(BMFace *) * (del_faces + 2));
			faces[del_faces] = f1;
			faces[del_faces + 1] = f2;
			del_faces += 2;
		}
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
	             fmd->autohide_dist * 10); /*need to merge larger cracks*/

	//dissolve sharp edges with limit dissolve
	BMO_op_callf(bm, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "dissolve_limit_keep_normals "
	             "angle_limit=%f use_dissolve_boundaries=%b verts=%av edges=%ae delimit=%i",
	             DEG2RADF(1.0f), false, 0);

	result = CDDM_from_bmesh(bm, true);
	BM_mesh_free(bm);
	MEM_freeN(faces);

	return result;
}


static DerivedMesh *doSimulate(FractureModifierData *fmd, Object *ob, DerivedMesh *dm, DerivedMesh *orig_dm)
{
	bool exploOK = false; /* doFracture */
	double start = 0.0;

	if ((fmd->refresh) || (fmd->refresh_constraints && !fmd->execute_threaded) ||
	    (fmd->refresh_constraints && fmd->execute_threaded && fmd->frac_mesh && fmd->frac_mesh->running == 0))
	{
		/* if we changed the fracture parameters */

		freeData_internal((ModifierData *)fmd);

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
			    !fmd->shards_to_islands /*&& !fmd->dm_group*/)
			{
				Shard *s;
				MeshIsland *mi; /* can be created without shards even, when using fracturemethod = NONE (re-using islands)*/

				int i = 0, j, vertstart = 0, polystart = 0;

				float dummyloc[3], rot[4];
				MDeformVert *dvert = fmd->dm->getVertDataArray(fmd->dm, CD_MDEFORMVERT);
				MDeformVert *ivert;
				ListBase shardlist;
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

				if (fmd->fix_normals)
				{
					start = PIL_check_seconds_timer();
				}

				shardlist = fmd->frac_mesh->shard_map;

				for (s = shardlist.first; s; s = s->next) {
					MVert *mv, *verts, *mverts;
					int totvert, k;

					if (s->totvert == 0) {
						continue;
					}

					fmd->frac_mesh->progress_counter++;

					mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
					mi->locs = MEM_mallocN(sizeof(float)*3, "mi->locs");
					mi->rots = MEM_mallocN(sizeof(float)*4, "mi->rots");
					mi->frame_count = 0;

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

					/*copy fixed normals to physics mesh too (needed for convert to objects)*/

					BKE_shard_calc_minmax(s);
					mi->vertex_count = s->totvert;

					copy_v3_v3(mi->centroid, s->centroid);

					mat4_to_loc_quat(dummyloc, rot, ob->obmat);
					copy_v3_v3(mi->rot, rot);

					mi->bb = BKE_boundbox_alloc_unit();
					BKE_boundbox_init_from_minmax(mi->bb, s->min, s->max);

					mi->id = s->shard_id;
					mi->particle_index = -1; // s->cluster_colors[0];
					mi->neighbor_ids = s->neighbor_ids;
					mi->neighbor_count = s->neighbor_count;

					if (mi->vertex_count > 0) {
						mi->thresh_weight /= mi->vertex_count;
						mi->ground_weight /= mi->vertex_count;
					}

					mi->rigidbody = BKE_rigidbody_create_shard(fmd->modifier.scene, ob, mi);
					BKE_rigidbody_calc_shard_mass(ob, mi, orig_dm);

					if (fmd->frac_algorithm == MOD_FRACTURE_BOOLEAN_FRACTAL)
					{
						/* cant be kept together in other ways */
						fmd->use_constraints = true;
						fmd->contact_dist = 2.0f;
						fmd->breaking_angle = DEG2RADF(1.0f);

						/* this most likely will only work with "Mesh" shape*/
						mi->rigidbody->shape = RB_SHAPE_TRIMESH;
						mi->rigidbody->margin = 0.0f;

						/* set values on "handle object" as well */
						ob->rigidbody_object->shape = RB_SHAPE_TRIMESH;
						ob->rigidbody_object->margin = 0.0f;
					}

					mi->vertex_indices = NULL;
					mi->start_frame = fmd->modifier.scene->rigidbody_world->pointcache->startframe;

					polystart += s->totpoly;
					i++;
				}

				if (fmd->fix_normals) {
					printf("Fixing normals done, %g\n", PIL_check_seconds_timer() - start);
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

			printf("Islands: %d\n", BLI_listbase_count(&fmd->meshIslands));
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
		else
		{
			/* fallback, this branch is executed when the modifier data has been loaded via readfile.c,
			 * although this might not be directly visible due to complex logic */

			MDeformVert* dvert = NULL;
			if (fmd->visible_mesh_cached)
				dvert = fmd->visible_mesh_cached->getVertDataArray(fmd->visible_mesh_cached, CD_MDEFORMVERT);
			if ((dvert != NULL) && (dvert->dw == NULL))
				fill_vgroup(fmd, fmd->visible_mesh_cached, dvert, ob);
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
		fmd->refresh_autohide = true;

		if (fmd->execute_threaded) {
			/* job done */
			fmd->frac_mesh->running = 0;
		}
	}

	if (fmd->refresh_autohide)
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
			make_face_pairs(fmd, fmd->dm);
		}
		else if (fmd->visible_mesh)
		{
			DerivedMesh *fdm = CDDM_from_bmesh(fmd->visible_mesh, true);
			make_face_pairs(fmd, fdm);

			fdm->needsFree = 1;
			fdm->release(fdm);
			fdm = NULL;
		}
	}

	if (fmd->refresh_constraints) {

		start = PIL_check_seconds_timer();
		doClusters(fmd, ob);
		printf("Clustering done, %g\n", PIL_check_seconds_timer() - start);

		start = PIL_check_seconds_timer();

		create_constraints(fmd); /* check for actually creating the constraints inside*/
		fmd->refresh_constraints = false;

		printf("Building constraints done, %g\n", PIL_check_seconds_timer() - start);
		printf("Constraints: %d\n", BLI_listbase_count(&fmd->meshConstraints));
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

		if (fmd->autohide_dist > 0 && fmd->face_pairs) {
			dm_final = do_autoHide(fmd, fmd->visible_mesh_cached);
			//printf("Autohide1 \n");
		}
		else {
			dm_final = CDDM_copy(fmd->visible_mesh_cached);
		}
		return dm_final;
	}
	else if ((fmd->visible_mesh_cached != NULL) && exploOK) {
		DerivedMesh *dm_final;

		if (fmd->autohide_dist > 0 && fmd->face_pairs) {
			//printf("Autohide2 \n");
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
			freeData_internal((ModifierData *)fmd);
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
	walk(userData, ob, (ID **)&fmd->cluster_group);
	walk(userData, ob, (ID **)&fmd->cutter_group);
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

	if (fmd->cutter_group) {
		GroupObject *go;
		for (go = fmd->cutter_group->gobject.first; go; go = go->next) {
			if (go->ob) {
				walk(userData, ob, &go->ob);
			}
		}
	}
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
			if (fmd->nor_tree != NULL) {
				BLI_kdtree_free(fmd->nor_tree);
				fmd->nor_tree = NULL;
			}

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

		if (fmd->dm && fmd->frac_mesh && (fmd->dm->getNumPolys(fmd->dm) > 0)) {
			final_dm = doSimulate(fmd, ob, fmd->dm, clean_dm);
		}
		else {
			final_dm = doSimulate(fmd, ob, clean_dm, clean_dm);
		}
	}

	/* free newly created derivedmeshes only, but keep derivedData and final_dm*/
	if ((clean_dm != group_dm) && (clean_dm != derivedData) && (clean_dm != final_dm))
	{
		clean_dm->needsFree = 1;
		clean_dm->release(clean_dm);
	}

	if ((group_dm != derivedData) && (group_dm != final_dm))
	{
		group_dm->needsFree = 1;
		group_dm->release(group_dm);
	}

	return final_dm;
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
			if (fmd->nor_tree != NULL) {
				BLI_kdtree_free(fmd->nor_tree);
				fmd->nor_tree = NULL;
			}

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
		if (fmd->dm && fmd->frac_mesh && (fmd->dm->getNumPolys(fmd->dm) > 0)) {
			final_dm = doSimulate(fmd, ob, fmd->dm, clean_dm);
		}
		else {
			final_dm = doSimulate(fmd, ob, clean_dm, clean_dm);
		}
	}

	/* free newly created derivedmeshes only, but keep derivedData and final_dm*/
	if ((clean_dm != group_dm) && (clean_dm != derivedData) && (clean_dm != final_dm))
	{
		clean_dm->needsFree = 1;
		clean_dm->release(clean_dm);
	}

	if ((group_dm != derivedData) && (group_dm != final_dm))
	{
		group_dm->needsFree = 1;
		group_dm->release(group_dm);
	}

	return final_dm;
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
	/* dependsOnTime */ dependsOnTime,
	/* dependsOnNormals */ dependsOnNormals,
	/* foreachObjectLink */ foreachObjectLink,
	/* foreachIDLink */ foreachIDLink,
};
