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
* ***** END GPL LICENSE BLOCK *****
*
*/

/** \file blender/modifiers/intern/MOD_fracture.c
*  \ingroup modifiers
*/

//#include "BLI_string_utf8.h"
#include "MEM_guardedalloc.h"

#include "BLI_utildefines.h"
#include "BLI_math.h"
#include "BLI_listbase.h"
#include "BLI_kdtree.h"
#include "BLI_edgehash.h"
#include "BLI_ghash.h"
#include "BLI_math_vector.h"
#include "BLI_math_matrix.h"
#include "BLI_rand.h"

#include "BKE_fracture.h"
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
#include "BKE_material.h"
#include "BKE_deform.h"

#include "bmesh.h"

#include "DNA_object_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"
#include "DNA_listBase.h"
#include "DNA_group_types.h"
#include "DNA_fracture_types.h"
#include "DNA_material_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_particle_types.h"

#include "MOD_util.h"

#include "../../rigidbody/RBI_api.h"
#include "PIL_time.h"

static void do_fracture(FractureModifierData *fracmd, ShardID id, Object *obj, DerivedMesh* dm);
//int vol_check(RigidBodyModifierData *rmd, MeshIsland* mi);
//void destroy_compound(FractureModifierData* rmd, Object *ob, MeshIsland* mi, float cfra);
void buildCompounds(FractureModifierData *rmd, Object *ob);
void freeMeshIsland(FractureModifierData *rmd, MeshIsland *mi);
void connect_constraints(FractureModifierData* rmd,  Object* ob, MeshIsland **meshIslands, int count, BMesh **combined_mesh, KDTree **combined_tree);
DerivedMesh* doSimulate(FractureModifierData *fmd, Object *ob, DerivedMesh *dm);

static void meshisland_init_verts(FractureModifierData* fmd)
{
	int i = 0;
	MeshIsland* mi;
	int vertstart = 0;

	BM_mesh_elem_index_ensure(fmd->visible_mesh, BM_VERT);
	BM_mesh_elem_table_ensure(fmd->visible_mesh, BM_VERT);

	for (mi = fmd->meshIslands.first; mi; mi = mi->next)
	{
		if (mi->vertices != NULL)
		{
			break;
		}

		mi->vertices = MEM_mallocN(sizeof(BMVert*) * mi->vertex_count, "mi->vertices");
		for (i = 0; i < mi->vertex_count; i++)
		{
			mi->vertices[i] = BM_vert_at_index(fmd->visible_mesh, vertstart+i);
		}
		vertstart += mi->vertex_count;
	}
}

static void initData(ModifierData *md)
{

		FractureModifierData *fmd = (FractureModifierData*) md;

		fmd->cluster_count = 5;
		fmd->extra_group = NULL;
		fmd->frac_algorithm = MOD_FRACTURE_BISECT;
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
		//rmd->constraint_group = NULL;
		fmd->contact_dist = 1.0f;
		//rmd->group_breaking_threshold = 1.0f;
		//rmd->group_contact_dist = 0.0001f;
		fmd->mass_dependent_thresholds = false;
		//rmd->auto_merge = FALSE;
		//fmd->sel_indexes = NULL;
		//fmd->sel_counter = 0;
		//rmd->auto_merge_dist = 0.0001f;
		fmd->inner_constraint_type = RBC_TYPE_FIXED;
		//rmd->outer_constraint_type = RBC_TYPE_FIXED;
		//rmd->outer_constraint_location = MOD_RIGIDBODY_CENTER;
		//rmd->outer_constraint_pattern = MOD_RIGIDBODY_SELECTED_TO_ACTIVE;
		fmd->idmap = NULL;
		fmd->explo_shared = false;
		fmd->constraint_limit = 0;
		fmd->dist_dependent_thresholds = false;
		fmd->contact_dist_meaning = MOD_RIGIDBODY_CENTROIDS;
		fmd->breaking_distance = 0;
		fmd->breaking_angle = 0;
		fmd->breaking_percentage = 0; //disable by default
		fmd->use_both_directions = false;
		fmd->use_proportional_distance = false;
		fmd->use_proportional_limit = false;
		fmd->max_vol = 0;
		fmd->cell_size = 1.0f;
		fmd->refresh_constraints = false;
//		fmd->split = destroy_compound;
//		fmd->join = buildCompounds;
		fmd->use_cellbased_sim = false;
		fmd->framecount = 0;
		fmd->framemap = NULL;
		fmd->disable_self_collision = true;
		fmd->cluster_breaking_threshold = 1000.0f;
		fmd->use_proportional_solver_iterations = false;
		fmd->solver_iterations_override = 0;
		fmd->shards_to_islands = false;
		fmd->execute_threaded = false;
}

static void freeData(ModifierData *md)
{
	FractureModifierData *rmd = (FractureModifierData*) md;
	MeshIsland *mi;
	RigidBodyShardCon *rbsc;
	int i;
	
	if ((!rmd->refresh && !rmd->refresh_constraints) || (rmd->frac_mesh && rmd->frac_mesh->cancel == 1))
	{
		//called on deleting modifier, object or quitting blender...
		if (rmd->dm) {
			rmd->dm->needsFree = 1;
			rmd->dm->release(rmd->dm);
			rmd->dm = NULL;
		}

		if (rmd->visible_mesh_cached)
		{
			rmd->visible_mesh_cached->needsFree = 1;
			rmd->visible_mesh_cached->release(rmd->visible_mesh_cached);
			rmd->visible_mesh_cached = NULL;
		}

		if (rmd->frac_mesh)
		{
			BKE_fracmesh_free(rmd->frac_mesh, rmd->frac_algorithm != MOD_FRACTURE_VORONOI);
			MEM_freeN(rmd->frac_mesh);
			rmd->frac_mesh = NULL;
		}

		if (rmd->noisemap && rmd->noise_count > 0)
		{
			MEM_freeN(rmd->noisemap);
			rmd->noisemap = NULL;
			rmd->noise_count = 0;
		}

		if (rmd->visible_mesh != NULL)
		{
			BM_mesh_free(rmd->visible_mesh);
			rmd->visible_mesh = NULL;
		}

		while (rmd->meshIslands.first) {
			mi = rmd->meshIslands.first;
			BLI_remlink(&rmd->meshIslands, mi);
			freeMeshIsland(rmd, mi);
			mi = NULL;
		}

		rmd->meshIslands.first = NULL;
		rmd->meshIslands.last = NULL;

		//if (rmd->shards_to_islands)
		{
			while (rmd->islandShards.first) {
				Shard* s = rmd->islandShards.first;
				BLI_remlink(&rmd->islandShards, s);
				BKE_shard_free(s, true);
				s = NULL;
			}

			rmd->islandShards.first = NULL;
			rmd->islandShards.last = NULL;
		}
	}

	if (rmd->visible_mesh_cached && !rmd->shards_to_islands &&
	   (/*rmd->visible_mesh ||*/ (!rmd->refresh && !rmd->refresh_constraints)))
	{
		//DM_release(rmd->visible_mesh_cached);
		rmd->visible_mesh_cached->needsFree = 1;
		rmd->visible_mesh_cached->release(rmd->visible_mesh_cached);
		//MEM_freeN(rmd->visible_mesh_cached);
		rmd->visible_mesh_cached = NULL;
	}

	//simulation data...
	if (!rmd->refresh_constraints)
	{
		if (rmd->shards_to_islands)
		{
			while (rmd->islandShards.first) {
				Shard* s = rmd->islandShards.first;
				BLI_remlink(&rmd->islandShards, s);
				BKE_shard_free(s, true);
				s = NULL;
			}

			rmd->islandShards.first = NULL;
			rmd->islandShards.last = NULL;
		}

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

		if (rmd->sel_indexes != NULL && rmd->refresh == false) {
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

	/*if (rmd->visible_mesh_cached && (rmd->visible_mesh || (!rmd->refresh && !rmd->refresh_constraints)))
	{
		//DM_release(rmd->visible_mesh_cached);
		rmd->visible_mesh_cached->needsFree = 1;
		rmd->visible_mesh_cached->release(rmd->visible_mesh_cached);
		//MEM_freeN(rmd->visible_mesh_cached);
		rmd->visible_mesh_cached = NULL;
	}*/

/*	while (rmd->cells.first)
	{
		NeighborhoodCell* c = rmd->cells.first;
		BLI_remlink(&rmd->cells, c);
		MEM_freeN(c);
	}
	rmd->cells.first = NULL;
	rmd->cells.last = NULL;*/

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

static void growCluster(FractureModifierData *fmd, Shard* seed, int sindex, ListBase* lbVisit, KDTree* tree, int depth)
{
	int i = 0, ret = 0, count = 0;
	float size = 0.1f;
	KDTreeNearest* nearest;

	count = BLI_kdtree_range_search(tree, seed->centroid, &nearest, size*depth);
	for (i = 0; i < count; i++)
	{
		Shard* neighbor;
		int index = nearest[i].index;
		neighbor = fmd->frac_mesh->shard_map[index];
		if (neighbor->cluster_colors[0] == -1)
		{
			neighbor->cluster_colors[0] = sindex;
			//BKE_shard_assign_material(neighbor, (short)sindex);
		}

		if (BLI_findindex(lbVisit, seed) == -1)
		{
			BLI_addtail(lbVisit, seed);
		}
	}
	MEM_freeN(nearest);
}

static void doClusters(FractureModifierData* fmd, int levels, Object *ob)
{
	//grow clusters from all shards...
	ListBase lbVisit;
	Material* mat;
	int i = 0, j = 0, k = 0, sindex = 0, counter = 0, depth = 1;
	KDTree *tree = BLI_kdtree_new(fmd->frac_mesh->shard_count);


	lbVisit.first = NULL;
	lbVisit.last = NULL;

	for (j = 0; j < levels; j++)
	{
		//Shard* nwhich = NULL;
		Shard **seeds;
		int seed_count;
		int last = 0;
		//prepare shard as list
		for (i = 0; i < fmd->frac_mesh->shard_count; i++)
		{
			Shard *s = fmd->frac_mesh->shard_map[i];
			s->cluster_colors = MEM_mallocN(sizeof(int) * levels, "cluster_colors");
			for (k = 0; k < levels; k++)
			{
				s->cluster_colors[k] = -1;
			}

			BLI_kdtree_insert(tree, i, s->centroid);
		}

		if (fmd->cluster_count < 2)
		{
			BLI_kdtree_free(tree);
			return;
		}

		BLI_kdtree_balance(tree);

		//remove materials...
		for (i = 0; i < ob->totcol; i++)
		{
			//BKE_material_pop_id(ob, i, true);
		}

		seed_count = (fmd->cluster_count > fmd->frac_mesh->shard_count ? fmd->frac_mesh->shard_count : fmd->cluster_count);// (int)(BLI_frand() * BLI_countlist(&lb));
		seeds = MEM_mallocN(sizeof(Shard*) * seed_count, "seeds");

#if 0
		for (k = 0; k < seed_count; k++)
		{
			mat = BKE_material_add(G.main, "Cluster");
			mat->r = BLI_frand();
			mat->g = BLI_frand();
			mat->b = BLI_frand();
			assign_material(ob, mat, (short)k+1, BKE_MAT_ASSIGN_OBDATA);
		}
#endif

		for (k = 0; k < seed_count; k++)
		{
			int color = k;
			int which_index = k * (int)(fmd->frac_mesh->shard_count / seed_count);
			Shard *which = fmd->frac_mesh->shard_map[which_index];
			which->cluster_colors[j] = color;
			//BKE_shard_assign_material(which, (short)color);
			BLI_addtail(&lbVisit, which);
			seeds[k] = which;
		}

		while (BLI_countlist(&lbVisit) < fmd->frac_mesh->shard_count)
		{
			Shard* seed;
			float size;

			if (sindex == seed_count)
			{
				sindex = 0;
				depth++;
			}

			/*for (i = BLI_countlist(&lbVisit)-1; i > 0; i--)
			{
				seed = BLI_findlink(&lbVisit,i);
				if (seed->cluster_colors[0] == sindex)
				{
					break;
				}
			}*/

			//seed = BLI_findlink(&lbVisit, //seeds[sindex];
			seed = seeds[sindex];
			growCluster(fmd, seed, sindex, &lbVisit, tree, depth);
			sindex++;

			if (counter == 10000)
			{
				break;
			}

			counter++;
		}

		BLI_kdtree_free(tree);

#if 0
		while (lbVisit.first != lbVisit.last)
		{
			int sindex = (int) (BLI_frand() * (seed_count - 1));
			Shard* which = (nwhich == NULL) ? seeds[sindex] : nwhich;


			int ncount = which->neighbor_count;
			int many = ncount;//(int)(BLI_frand() * (ncount-1));
			bool found = false;

			for (i = 0; i < many; i++)
			{
				int nindex = i;// (int)(BLI_frand() * (ncount-1));
				int id = which->neighbor_ids[nindex];
				if (id > -1)
				{
					nwhich = fmd->frac_mesh->shard_map[id];
					BLI_addtail(&lbVisit, nwhich);

					if (nwhich->cluster_colors[j] == -1)
					{
						nwhich->cluster_colors[j] = sindex;
						BKE_shard_assign_material(nwhich, (short)sindex);
						nwhich = NULL;
						found = true;
					}
				}
				else
				{
					nwhich = NULL;
					//break;
				}
			}

			if (!found)
			{
				BLI_remlink_safe()
			}
#endif
			// make a seed pass and then grow the seeds, limit by cluster count, cluster size (in members)
		//}
		MEM_freeN(seeds);
	}
}


static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
                                  DerivedMesh *derivedData,
                                  ModifierApplyFlag UNUSED(flag))
{
	//also show single islands via selected index... hmm but how to apply to it ?
	//apply to selected shards; this must be in kernel as well (via index) but there prepare
	//an array of indexes; for multiselection
	//outliner ? hmm
	//all not so good...

	FractureModifierData *fmd = (FractureModifierData*) md;
	DerivedMesh *final_dm = derivedData;

	if (fmd->frac_mesh != NULL && fmd->frac_mesh->running == 1 && fmd->execute_threaded)
	{
		//skip modifier execution when job is running
		return final_dm;
	}

	if (fmd->refresh)
	{
		if (fmd->dm != NULL)
		{
			fmd->dm->needsFree = 1;
			fmd->dm->release(fmd->dm);
			fmd->dm = NULL;
		}

		if (fmd->frac_mesh != NULL)
		{
			BKE_fracmesh_free(fmd->frac_mesh, fmd->frac_algorithm != MOD_FRACTURE_VORONOI);
			MEM_freeN(fmd->frac_mesh);
			fmd->frac_mesh = NULL;
		}

		if (fmd->frac_mesh == NULL)
		{
			fmd->frac_mesh = BKE_create_fracture_container(derivedData);
			if (fmd->execute_threaded)
			{
				fmd->frac_mesh->running = 1;
			}
		}
	}

	//if (!fmd->dm)
	{
		if (fmd->refresh)
		{
			do_fracture(fmd, -1, ob, derivedData);

			if (!fmd->refresh) //might have been changed from outside, job cancel
			{
				return derivedData;
			}
		}
		if (fmd->dm && fmd->frac_mesh && (fmd->dm->getNumPolys(fmd->dm) > 0))
		{
			final_dm = doSimulate(fmd, ob, fmd->dm);
		}
		else
		{
			final_dm = doSimulate(fmd, ob, derivedData);
		}
	}

/*	if (fmd->dm)
		final_dm = CDDM_copy(fmd->dm);
	else
		final_dm = derivedData;*/

	return final_dm;
}

static DerivedMesh *applyModifierEM(ModifierData *md, Object *ob,
                                    struct BMEditMesh *UNUSED(editData),
                                    DerivedMesh *derivedData,
                                    ModifierApplyFlag UNUSED(flag))
{
	FractureModifierData *fmd = (FractureModifierData*) md;
	DerivedMesh *final_dm = derivedData;

	if (fmd->frac_mesh != NULL && fmd->frac_mesh->running == 1 && fmd->execute_threaded)
	{
		//skip modifier execution when job is running
		return final_dm;
	}

	if (fmd->refresh)
	{
		if (fmd->dm != NULL)
		{
			fmd->dm->needsFree = 1;
			fmd->dm->release(fmd->dm);
			fmd->dm = NULL;
		}

		if (fmd->frac_mesh != NULL)
		{
			BKE_fracmesh_free(fmd->frac_mesh, fmd->frac_algorithm != MOD_FRACTURE_VORONOI);
			MEM_freeN(fmd->frac_mesh);
			fmd->frac_mesh = NULL;
		}

		if (fmd->frac_mesh == NULL)
		{
			fmd->frac_mesh = BKE_create_fracture_container(derivedData);
			if (fmd->execute_threaded)
			{
				fmd->frac_mesh->running = 1;
			}
		}
	}

	//if (!fmd->dm)
	{
		if (fmd->refresh)
		{
			do_fracture(fmd, -1, ob, derivedData);

			if (!fmd->refresh) //might have been changed from outside, job cancel
			{
				return derivedData;
			}
		}
		if (fmd->dm && fmd->frac_mesh)
		{
			final_dm = doSimulate(fmd, ob, fmd->dm);
		}
		else
		{
			final_dm = doSimulate(fmd, ob, derivedData);
		}
	}

	return final_dm;
}

static void points_from_verts(Object** ob, int totobj, FracPointCloud* points, float mat[4][4], float thresh, FractureModifierData *emd, DerivedMesh* dm, Object* obj)
{
	int v, o, pt = points->totpoints;
	float co[3];

	for (o = 0; o < totobj; o++)
	{
		if (ob[o]->type == OB_MESH)
		{
			float imat[4][4];
			//Mesh* me = (Mesh*)ob[o]->data;
			DerivedMesh *d;
			MVert* vert;

			if (ob[o] == obj)
			{
				d = dm;
			}
			else
			{
				d = mesh_get_derived_final(emd->modifier.scene, ob[o], 0);
			}

			invert_m4_m4(imat, mat);
			vert = d->getVertArray(d);

			for (v = 0; v < d->getNumVerts(d); v++)
			{
				if (BLI_frand() < thresh) {
					//*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
					//(*points)[pt] = MEM_callocN(3 * sizeof(float), "points[pt]");
					points->points = MEM_reallocN((*points).points, (pt+1) * sizeof(FracPoint));

					//copy_v3_v3(co, me->mvert[v].co);
					copy_v3_v3(co, vert[v].co);

					if ((o > 0) ||
					   ((emd->point_source & MOD_FRACTURE_EXTRA_VERTS) &&
					   (!(emd->point_source & MOD_FRACTURE_OWN_VERTS)) && (o == 0)))
					{
						mul_m4_v3(ob[o]->obmat, co);
						mul_m4_v3(imat, co);
					}

					//copy_v3_v3((*points)[pt], co);
					copy_v3_v3(points->points[pt].co, co);
					pt++;
				}
			}
		}
	}

	points->totpoints = pt;
}

static void points_from_particles(Object** ob, int totobj, Scene* scene, FracPointCloud* points, float mat[4][4],
								 float thresh, FractureModifierData* emd)
{
	int o, p, pt = points->totpoints;
	ParticleSystemModifierData* psmd;
	ParticleData* pa;
	ParticleSimulationData sim = {NULL};
	ParticleKey birth;
	ModifierData* mod;

	for (o = 0; o < totobj; o++)
	{
		for (mod = ob[o]->modifiers.first; mod; mod = mod->next)
		{
			if (mod->type == eModifierType_ParticleSystem)
			{
				float imat[4][4];
				psmd = (ParticleSystemModifierData*)mod;
				sim.scene = scene;
				sim.ob = ob[o];
				sim.psys = psmd->psys;
				sim.psmd = psmd;
				invert_m4_m4(imat, mat);

				for (p = 0, pa = psmd->psys->particles; p < psmd->psys->totpart; p++, pa++)
				{
					bool particle_unborn = pa->alive == PARS_UNBORN;// && (emd->flag & eExplodeFlag_Unborn);
					bool particle_alive = pa->alive == PARS_ALIVE; //&& (emd->flag & eExplodeFlag_Alive);
					bool particle_dead = pa->alive == PARS_DEAD; //&& (emd->flag & eExplodeFlag_Dead);
					bool particle_mask = particle_unborn || particle_alive || particle_dead;

					if ((BLI_frand() < thresh) && particle_mask) {
						float co[3];
						psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);
						//*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
						//(*points)[pt] = MEM_callocN(3*sizeof(float), "points[pt]");
						points->points = MEM_reallocN(points->points, (pt+1) * sizeof(FracPoint));
						copy_v3_v3(co, birth.co);

						if ((o > 0) ||
						   ((emd->point_source & MOD_FRACTURE_EXTRA_PARTICLES) &&
						   (!(emd->point_source & MOD_FRACTURE_OWN_PARTICLES)) && (o == 0)))
						{
							//mul_m4_v3(ob[o]->obmat, co);
							mul_m4_v3(imat, co);
							//add_v3_v3(co, centr);
						}
						//copy_v3_v3((*points)[pt], co);
						copy_v3_v3(points->points[pt].co, co);
						pt++;
					}
				}
			}
		}
	}

	points->totpoints = pt;
}

static void points_from_greasepencil(Object** ob, int totobj, FracPointCloud* points, float mat[4][4], float thresh)
{
	bGPDlayer* gpl;
	bGPDframe* gpf;
	bGPDstroke* gps;
	int pt = points->totpoints, p, o;

	for (o = 0; o < totobj; o++)
	{
		if ((ob[o]->gpd) && (ob[o]->gpd->layers.first))
		{
			float imat[4][4];
			invert_m4_m4(imat, mat);
			for (gpl = ob[o]->gpd->layers.first; gpl; gpl = gpl->next)
			{
				for (gpf = gpl->frames.first; gpf; gpf = gpf->next) {
				//gpf = gpl->actframe;
					for (gps = gpf->strokes.first; gps; gps = gps->next)
					{
						for (p = 0; p < gps->totpoints; p++)
						{
							if (BLI_frand() < thresh)
							{
								float point[3] = {0, 0, 0};
								//*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
								//(*points)[pt] = MEM_callocN(3*sizeof(float), "points[pt]");
								points->points = MEM_reallocN(points->points, (pt+1) * sizeof(FracPoint));

								point[0] = gps->points[p].x;
								point[1] = gps->points[p].y;
								point[2] = gps->points[p].z;

								mul_m4_v3(imat, point);
								//copy_v3_v3((*points)[pt], point);
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

		*obs = MEM_reallocN(*obs, sizeof(Object*) * (ctr+1));
		(*obs)[ctr] = go->ob;
		ctr++;
	}

	return ctr;
}

static FracPointCloud get_points_global(FractureModifierData *emd, Object *ob, DerivedMesh* fracmesh)
{
	Scene* scene = emd->modifier.scene;
	FracPointCloud points;

	//global settings, for first fracture only, or global secondary and so on fracture, apply to entire fracmesh
	int totgroup = 0, t = 0;
	Object** go = MEM_mallocN(sizeof(Object*), "groupobjects");
	float thresh = (float)emd->percentage / 100.0f;
	BoundBox* bb;
	float* noisemap = NULL;

	float min[3], max[3];
	int i;

	points.points = MEM_mallocN(sizeof(FracPoint), "points");
	points.totpoints = 0;

	if (emd->point_source & (MOD_FRACTURE_EXTRA_PARTICLES | MOD_FRACTURE_EXTRA_VERTS ))
	{
		if (((emd->point_source & MOD_FRACTURE_OWN_PARTICLES) && (emd->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
			((emd->point_source & MOD_FRACTURE_OWN_VERTS) && (emd->point_source & MOD_FRACTURE_EXTRA_VERTS)) ||
			((emd->point_source & MOD_FRACTURE_GREASEPENCIL) && (emd->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
		    ((emd->point_source & MOD_FRACTURE_GREASEPENCIL) && (emd->point_source & MOD_FRACTURE_EXTRA_VERTS)))
		{
			go = MEM_reallocN(go, sizeof(Object*)*(totgroup+1));
			go[totgroup] = ob;
			totgroup++;
		}

		totgroup = getGroupObjects(emd->extra_group, &go, totgroup);
	}
	else
	{
		totgroup = 1;
		go[0] = ob;
	}

	if (emd->point_source & (MOD_FRACTURE_OWN_PARTICLES | MOD_FRACTURE_EXTRA_PARTICLES))
	{
		points_from_particles(go, totgroup, scene, &points, ob->obmat, thresh, emd);
	}

	if (emd->point_source & (MOD_FRACTURE_OWN_VERTS | MOD_FRACTURE_EXTRA_VERTS))
	{
		points_from_verts(go, totgroup, &points, ob->obmat, thresh, emd, fracmesh, ob);
	}

	if (emd->point_source & MOD_FRACTURE_GREASEPENCIL)
	{
		points_from_greasepencil(go, totgroup, &points, ob->obmat, thresh);
	}

	//greasecut, imagebased... backend is via projected curvemeshes

	//random "chips" (cut out via boolean, no voronoi)

	//secondary, tertiary fracture.... levels... (or 3 or 4 enough ?) each has same options, different parameters

	//cracks... in glass its ok to have visible faces, but solids ? very small debris must fall off, or just bevel the shards near the impact ?


	//local settings, apply per shard!!! Or globally too first.
	if (emd->point_source & MOD_FRACTURE_UNIFORM)
	{
		int count = emd->shard_count;
		//this is pointsource "uniform", make seed settable
		INIT_MINMAX(min, max);
		BKE_get_shard_minmax(emd->frac_mesh, -1, min, max, fracmesh); //id 0 should be entire mesh
		//points.totpoints += emd->shard_count;

		if (emd->frac_algorithm == MOD_FRACTURE_BISECT_FAST)
		{
			//need double amount of shards, because we create 2 islands at each cut... so this matches the input count
			count *= 2;
		}

		//points.points = MEM_reallocN(points.points, sizeof(FracPoint) * points.totpoints);
		BLI_srandom(emd->point_seed);
		for (i = 0; i < count; ++i) {
			if (BLI_frand() < thresh)
			{
				float *co;
				points.points = MEM_reallocN(points.points, sizeof(FracPoint) * (points.totpoints+1));
				co = points.points[points.totpoints].co;
				co[0] = min[0] + (max[0] - min[0]) * BLI_frand();
				co[1] = min[1] + (max[1] - min[1]) * BLI_frand();
				co[2] = min[2] + (max[2] - min[2]) * BLI_frand();
				points.totpoints++;
			}
		}
	}


	// do this afterwards...
	//scaling point cloud for splinters (see cell fracture....)
	//apply noise

	bb = BKE_object_boundbox_get(ob);

	if (emd->noisemap == NULL && points.totpoints > 0)
	{
		noisemap = MEM_callocN(sizeof(float)*3 *points.totpoints, "noisemap");
	}

	for (t = 0; t < points.totpoints; t++) {
		float bbox_min[3], bbox_max[3];

		if (emd->noise > 0.0f)
		{
			if (emd->noisemap != NULL) {
				float noise[3];
				noise[0] = emd->noisemap[3*t];
				noise[1] = emd->noisemap[3*t+1];
				noise[2] = emd->noisemap[3*t+2];
				//add_v3_v3((*points)[t], noise);
				add_v3_v3(points.points[t].co, noise);
			}
			else
			{
				float scalar, size[3], rand[3] = {0, 0, 0}, temp_x[3], temp_y[3], temp_z[3], rand_x[3], rand_y[3], rand_z[3];
				noisemap[3*t] = 0.0f;
				noisemap[3*t+1] = 0.0f;
				noisemap[3*t+2] = 0.0f;

				mul_v3_m4v3(bbox_min, ob->obmat, bb->vec[0]);
				mul_v3_m4v3(bbox_max, ob->obmat, bb->vec[6]);
				sub_v3_v3v3(size, bbox_max, bbox_min);

				scalar = emd->noise * len_v3(size) / 2.0f;
				rand[0] = 2.0f * (BLI_frand() - 0.5f);
				rand[1] = 2.0f * (BLI_frand() - 0.5f);
				rand[2] = 2.0f * (BLI_frand() - 0.5f);

				rand[0] *= (scalar * BLI_frand());
				rand[1] *= (scalar * BLI_frand());
				rand[2] *= (scalar * BLI_frand());

				//test each component separately
				zero_v3(rand_x);
				rand_x[0] = rand[0];

				zero_v3(rand_y);
				rand_y[1] = rand[1];

				zero_v3(rand_z);
				rand_z[2] = rand[2];

				/*add_v3_v3v3(temp_x, (*points)[t], rand_x);
				add_v3_v3v3(temp_y, (*points)[t], rand_y);
				add_v3_v3v3(temp_z, (*points)[t], rand_z);*/

				add_v3_v3v3(temp_x, points.points[t].co, rand_x);
				add_v3_v3v3(temp_y, points.points[t].co, rand_y);
				add_v3_v3v3(temp_z, points.points[t].co, rand_z);

			//stay inside bounds !!
			if ((temp_x[0] >= bb->vec[0][0]) && (temp_x[0] <= bb->vec[6][0])) {
				//add_v3_v3((*points)[t], rand_x);
				add_v3_v3(points.points[t].co, rand_x);
				noisemap[3*t] = rand_x[1];
			}

			if ((temp_y[1] >= bb->vec[0][1]) && (temp_y[1] <= bb->vec[6][1])) {
				//add_v3_v3((*points)[t], rand_y);
				add_v3_v3(points.points[t].co, rand_y);
				noisemap[3*t+1] = rand_y[1];
			}

			if ((temp_z[2] >= bb->vec[0][2]) && (temp_z[2] <= bb->vec[6][2])) {
				//add_v3_v3((*points)[t], rand_z);
				add_v3_v3(points.points[t].co, rand_z);
				noisemap[3*t+2] = rand_z[2];
			}
		}
	}
	}

	if (emd->noisemap == NULL)
	{
		emd->noisemap = noisemap;
		emd->noise_count = points.totpoints;
	}

	//MEM_freeN(bb);
	MEM_freeN(go);
	return points;
}

static void do_fracture(FractureModifierData *fracmd, ShardID id, Object* obj, DerivedMesh *dm)
{
	/* dummy point cloud, random */
	FracPointCloud points;

	points = get_points_global(fracmd, obj, dm);

	//local settings, apply per shard!!! Or globally too first.
	/*if (fracmd->point_source & MOD_FRACTURE_UNIFORM)
	{
		//this is pointsource "uniform", make seed settable
		INIT_MINMAX(min, max);
		BKE_get_shard_minmax(fracmd->frac_mesh, 0, min, max); //id 0 should be entire mesh
		points.totpoints += fracmd->shard_count;

		points.points = MEM_reallocN(points.points, sizeof(FracPoint) * points.totpoints);
		BLI_srandom(fracmd->point_seed);
		for (i = 0; i < points.totpoints; ++i) {
			float *co = points.points[i].co;
			co[0] = min[0] + (max[0] - min[0]) * BLI_frand();
			co[1] = min[1] + (max[1] - min[1]) * BLI_frand();
			co[2] = min[2] + (max[2] - min[2]) * BLI_frand();
		}
	}*/

	if (points.totpoints > 0)
	{
	//if (emd->point_source & eRadial)
	//{
		//maybe add radial too // rings, rays, bias 0...1, ring random, ray random
	//}

	//non-uniform pointcloud ... max cluster min clustersize, cluster count / "variation", min max distance
		bool temp = fracmd->shards_to_islands;

		BKE_fracture_shard_by_points(fracmd->frac_mesh, id, &points, fracmd->frac_algorithm, obj, dm);

		//job has been cancelled, throw away all data
		if (fracmd->frac_mesh->cancel == 1)
		{
			fracmd->frac_mesh->running = 0;
			fracmd->refresh = true;
			freeData(fracmd);
			fracmd->frac_mesh = NULL;
			fracmd->refresh = false;
			//fracmd->frac_mesh->cancel = 0;
			MEM_freeN(points.points);
			return;
		}

		//MEM_freeN(points.points);
		if (fracmd->frac_mesh->shard_count > 0)
		{
			doClusters(fracmd, 1, obj);
		}

		//Here we REALLY need to fracture so deactivate the shards to islands flag and activate afterwards.
		fracmd->shards_to_islands = false;
		BKE_fracture_create_dm(fracmd, false);
		fracmd->shards_to_islands = temp;
	}
	MEM_freeN(points.points);

	//BKE_fracture_create_dm(fracmd, false);
}


static void copyData(ModifierData *md, ModifierData *target)
{
	FractureModifierData *rmd  = (FractureModifierData *)md;
	FractureModifierData *trmd = (FractureModifierData *)target;

	//todo -> copy fracture stuff as well, and dont forget readfile / writefile...
	zero_m4(trmd->origmat);
	//trmd->refresh = rmd->refresh;
	//trmd->auto_merge = rmd->auto_merge;
	trmd->breaking_threshold = rmd->breaking_threshold;
	trmd->use_constraints = rmd->use_constraints;
	//trmd->constraint_group = rmd->constraint_group;
	trmd->contact_dist = rmd->contact_dist;
	//trmd->group_breaking_threshold = rmd->group_breaking_threshold;
	//trmd->group_contact_dist = rmd->group_contact_dist;
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

	//trmd->auto_merge_dist = rmd->auto_merge_dist;
	trmd->refresh = false;
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
	trmd->refresh_constraints = false;
	//trmd->outer_constraint_location = rmd->outer_constraint_location;
	//trmd->outer_constraint_pattern = rmd->outer_constraint_pattern;
	//trmd->outer_constraint_type = rmd->outer_constraint_type;
	trmd->inner_constraint_type = rmd->inner_constraint_type;

	trmd->framecount = 0;
	trmd->framemap = NULL;
//	trmd->join = buildCompounds;
//	trmd->split = destroy_compound;
	trmd->disable_self_collision = rmd->disable_self_collision;
	trmd->cluster_breaking_threshold = rmd->cluster_breaking_threshold;
	trmd->use_proportional_solver_iterations = rmd->use_proportional_solver_iterations;
	trmd->solver_iterations_override = rmd->solver_iterations_override;
	trmd->shards_to_islands = rmd->shards_to_islands;
}

void freeMeshIsland(FractureModifierData* rmd, MeshIsland* mi)
{
	int i;

	if (mi->physics_mesh) {
		mi->physics_mesh->needsFree = 1;
		mi->physics_mesh->release(mi->physics_mesh);
		mi->physics_mesh = NULL;
	}
	if (mi->rigidbody) {
		//BKE_rigidbody_remove_shard(rmd->modifier.scene, mi);
		MEM_freeN(mi->rigidbody);
		mi->rigidbody = NULL;
	}

	if (1) { //(!rmd->explo_shared || mi->compound_count > 0) {
		if (mi->vertco /*&& rmd->refresh == FALSE*/) {
			MEM_freeN(mi->vertco);
			mi->vertco = NULL;
		}
		if (mi->vertices /*&& rmd->refresh == FALSE*/) {
			MEM_freeN(mi->vertices);
			mi->vertices = NULL; //borrowed only !!!
		}
	}

	if (mi->vertices_cached /*&& rmd->explo_shared*/)
	{
		//int i;
		//for (i = 0; i < mi->vertex_count; i++)
		{
			//MEM_freeN(mi->vertices_cached[i]);
		}
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

	if (mi->vertex_indices)
	{
		MEM_freeN(mi->vertex_indices);
		mi->vertex_indices = NULL;
	}

	MEM_freeN(mi);
	mi = NULL;
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

/*int vol_check(RigidBodyModifierData *rmd, MeshIsland* mi)
{
	float vol, cellvol;
	vol = bbox_vol(mi->bb);
	cellvol = rmd->cell_size * rmd->cell_size * rmd->cell_size;

	return vol > cellvol;
}*/

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
		copy_v3_v3(cent, BM_vert_at_index_find(bm, 0)->co);
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

/*static ExplodeModifierData *findPrecedingExploModifier(Object *ob, RigidBodyModifierData *rmd)
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
}*/

static float mesh_separate_tagged(FractureModifierData* rmd, Object *ob, BMVert** v_tag, int v_count, float** startco, BMesh* bm_work /*, MeshIsland*** mi_array, int* mi_count*/)
{
	BMesh *bm_new;
	BMesh *bm_old = bm_work;
	MeshIsland *mi;
	float centroid[3], dummyloc[3], rot[4], min[3], max[3], vol = 0;
	BMVert* v;
	BMIter iter;
	DerivedMesh *dm = NULL, *dmtemp = NULL;
	Shard *s;
	int i = 0;

	if (rmd->frac_mesh->cancel == 1)
		return 0.0f;

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

	BM_calc_center_centroid(bm_new, centroid, false);
	BM_mesh_elem_index_ensure(bm_new, BM_VERT | BM_EDGE | BM_FACE);

	if (rmd->shards_to_islands || rmd->frac_mesh->shard_count < 2)
	{
		//store temporary shards for each island
		dmtemp = CDDM_from_bmesh(bm_new, true);
		s = BKE_create_fracture_shard(dmtemp->getVertArray(dmtemp), dmtemp->getPolyArray(dmtemp), dmtemp->getLoopArray(dmtemp),
								  dmtemp->getNumVerts(dmtemp), dmtemp->getNumPolys(dmtemp), dmtemp->getNumLoops(dmtemp), true);
		s = BKE_custom_data_to_shard(s, dmtemp);
		BLI_addtail(&rmd->islandShards, s);

		dmtemp->needsFree = 1;
		dmtemp->release(dmtemp);
		dmtemp = NULL;
	}

	//use unmodified coords for bbox here
	//BM_mesh_minmax(bm_new, min, max);

	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
		//then eliminate centroid in vertex coords ?
		sub_v3_v3(v->co, centroid);
	}


	// add 1 MeshIsland
	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
	BLI_addtail(&rmd->meshIslands, mi);

	mi->thresh_weight = 0;
	mi->vertices = v_tag;
	mi->vertco = *startco;
//	MEM_freeN(*startco);
	mi->compound_children = NULL;
	mi->compound_count = 0;
	mi->compound_parent = NULL;
	zero_v3(mi->start_co);

	BM_mesh_normals_update(bm_new);
	BM_mesh_minmax(bm_new, min, max, false);
	dm = CDDM_from_bmesh(bm_new, true);
	BM_mesh_free(bm_new);
	bm_new = NULL;

	mi->physics_mesh = dm;
	mi->vertex_count = v_count;

	mi->vertex_indices = MEM_mallocN(sizeof(int) * mi->vertex_count, "mi->vertex_indices");
	//mi->vertco = MEM_mallocN(sizeof(float) * 3 * mi->vertex_count, "mi->vertco");
	for (i = 0; i < mi->vertex_count; i++)
	{
		mi->vertex_indices[i] = mi->vertices[i]->head.index;
		/*mi->vertco[i*3] = mi->vertices[i]->co[0];
		mi->vertco[i*3+1] = mi->vertices[i]->co[1];
		mi->vertco[i*3+2] = mi->vertices[i]->co[2];*/
	}

	copy_v3_v3(mi->centroid, centroid);
	mat4_to_loc_quat(dummyloc, rot, ob->obmat);
	copy_v3_v3(mi->rot, rot);
	//mi->parent_mod = rmd;
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
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
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

void mesh_separate_loose_partition(FractureModifierData* rmd, Object* ob, BMesh* bm_work, BMVert** orig_work/*, int doSplit*/)
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


	if (max_iter > 0)
	{
		rmd->frac_mesh->progress_counter++;
	}

	/* Clear all selected vertices */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_INTERNAL_TAG | BM_ELEM_TAG, false);

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
	//	printf("mesh_separate_tagged: %d %d\n", tot, bm_old->totvert);

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
	BM_mesh_minmax(bm, min, max, true);
	bb = BKE_boundbox_alloc_unit();
	BKE_boundbox_init_from_minmax(bb, min, max);
	vol = bbox_vol(bb);
	MEM_freeN(bb);
	return vol;
}

void halve(FractureModifierData* rmd, Object* ob, int minsize, BMesh** bm_work, BMVert*** orig_work, bool separated/*, bool doSplit*/)
{

	int half;
	int i = 0, new_count = 0;
	float vol_old = 0, vol_new = 0, min[3], max[3], cellvol = 0;
	BMIter iter;
	BMVert **orig_old = *orig_work, **orig_new, **orig_mod;
	BMVert *v;
	BMesh* bm_old = *bm_work;
	BMesh* bm_new = NULL;
	separated = false;

	if (rmd->frac_mesh->cancel == 1)
	{
		return;
	}

	bm_new = BM_mesh_create(&bm_mesh_allocsize_default);
	{
		/*if (rmd->shards_to_islands)
		{
			BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, false);
		}
		else*/
		{
			BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);
		}

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
		//if (!rmd->shards_to_islands)
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
		/*else
		{
			//try to DEselect here...
			half = bm_old->totvertsel / 2;
			BM_ITER_MESH(v, &iter, bm_old, BM_VERTS_OF_MESH)
			{
				if (i >= half) {
					break;
				}
				BM_elem_select_set(bm_old, v, false);
				i++;
			}
		}*/

		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_SELECT);
		select_linked(&bm_old);

		new_count = bm_old->totvertsel;
		printf("Halving...%d => %d %d\n", bm_old->totvert, new_count, bm_old->totvert - new_count);

		orig_new = MEM_callocN(sizeof(BMVert*) * new_count, "orig_new");
		orig_mod = MEM_callocN(sizeof(BMVert*) * bm_old->totvert - new_count, "orig_mod");
		mesh_separate_selected(&bm_old, &bm_new, orig_old, &orig_new, &orig_mod);

		/*if (rmd->shards_to_islands)
		{
			//BM_mesh_elem_hflag_disable_all(bm_new, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);
		}*/
	}

	printf("Old New: %d %d\n", bm_old->totvert, bm_new->totvert);
	if ((bm_old->totvert <= minsize && bm_old->totvert > 0) || (bm_new->totvert == 0)) {
		//if (!rmd->shards_to_islands)
		{
			mesh_separate_loose_partition(rmd, ob, bm_old, orig_mod);
			separated = true;
		}
	}

	if ((bm_new->totvert <= minsize && bm_new->totvert > 0) || (bm_old->totvert == 0) /*|| rmd->shards_to_islands*/) {
		mesh_separate_loose_partition(rmd, ob, bm_new, orig_new);
		separated = true;
	}

	//if (!rmd->shards_to_islands)
	{
		if ((bm_old->totvert > minsize && bm_new->totvert > 0) || (bm_new->totvert == 0 && !separated)) {
			halve(rmd, ob, minsize, &bm_old, &orig_mod, separated);
		}

		if ((bm_new->totvert > minsize && bm_old->totvert > 0) || (bm_old->totvert == 0 && !separated)) {
			halve(rmd, ob, minsize, &bm_new, &orig_new, separated);
		}
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

void mesh_separate_loose(FractureModifierData* rmd, Object* ob)
{
	int minsize = 1000;
	//GHash* vhash = BLI_ghash_ptr_new("VertHash");
	BMesh* bm_work;
	BMVert* vert, **orig_start;
	BMIter iter;
	//int lastparticle = -1;
	//VertParticle **fps = MEM_callocN(sizeof(VertParticle*) * rmd->visible_mesh->totface, "faceparticles");
	//IF HAVE CLASSIC EXPLO, DO NOT SPLIT, TAKE ITS ISLANDS AS IS...
	{
		//bool temp = rmd->shards_to_islands;
		BM_mesh_elem_hflag_disable_all(rmd->visible_mesh, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT | BM_ELEM_TAG, false);
		bm_work = BM_mesh_copy(rmd->visible_mesh);

		orig_start = MEM_callocN(sizeof(BMVert*) * rmd->visible_mesh->totvert, "orig_start");
		//associate new verts with old verts, here indexes should match still
		BM_ITER_MESH(vert, &iter, rmd->visible_mesh, BM_VERTS_OF_MESH)
		{
			orig_start[vert->head.index] = vert;
		}

		BM_mesh_elem_index_ensure(bm_work, BM_VERT);
		BM_mesh_elem_table_ensure(bm_work, BM_VERT);

		//free old islandshards first, if any
		while (rmd->islandShards.first) {
			Shard* s = rmd->islandShards.first;
			BLI_remlink(&rmd->islandShards, s);
			BKE_shard_free(s, true);
			s = NULL;
		}

		rmd->islandShards.first = NULL;
		rmd->islandShards.last = NULL;


		halve(rmd, ob, minsize, &bm_work, &orig_start, false);


	//	BLI_ghash_free(vhash, NULL, NULL);
		MEM_freeN(orig_start);
		orig_start = NULL;
		BM_mesh_free(bm_work);
		bm_work = NULL;
	}
}

void destroy_compound(FractureModifierData* rmd, Object* ob, MeshIsland *mi, float cfra)
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

void select_inner_faces(FractureModifierData* rmd, KDTree* tree, MeshIsland* mi) {

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
		n = BLI_kdtree_find_nearest(tree, co, NULL);
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

void select_inner_faces_of_vert(FractureModifierData* rmd, KDTree* tree, BMVert* vert) {

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
		n = BLI_kdtree_find_nearest_n(tree, co, face_near, 2);
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

static void connect_meshislands(FractureModifierData* rmd, Object* ob, MeshIsland* mi1, MeshIsland* mi2, int con_type, float thresh)
{
	int con_found = false;
	RigidBodyShardCon *con, *rbsc;
	bool ok = mi1 && mi1->rigidbody && !(mi1->rigidbody->flag & RBO_FLAG_ACTIVE_COMPOUND);
	ok = ok && mi2 && mi2->rigidbody && !(mi2->rigidbody->flag & RBO_FLAG_ACTIVE_COMPOUND);

	if (((!rmd->use_both_directions)/* || (mi1->parent_mod != mi2->parent_mod)*/) && ok)
	{
		//search local constraint list instead of global one !!! saves lots of time
		int i;
		for (i = 0; i < mi1->participating_constraint_count; i++)
		{
			con = mi1->participating_constraints[i];
			if ((con->mi1 == mi2) || (con->mi2 == mi2)) {
				con_found = true;
				break;
			}
		}

		if (!con_found)
		{
			for (i = 0; i < mi2->participating_constraint_count; i++)
			{
				con = mi2->participating_constraints[i];
				if ((con->mi1 == mi1) || (con->mi2 == mi1)){
					con_found = true;
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
			/*if (((rmd->constraint_group != NULL) &&
				(!BKE_group_object_exists(rmd->constraint_group, ob))) ||
				(rmd->constraint_group == NULL)) {*/
				{

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
					//ExplodeModifierData *emd = findPrecedingExploModifier(ob, rmd);
					//if ((emd != NULL) && (emd->cluster_size > 1))
					if (rmd->cluster_count > 1)
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

				if (rmd->thresh_defgrp_name[0])
				{
					//modify maximum threshold by average weight
					rbsc->breaking_threshold = thresh * (mi1->thresh_weight + mi2->thresh_weight) * 0.5f;
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

static int check_meshislands_adjacency(FractureModifierData* rmd, MeshIsland* mi, MeshIsland* mi2, BMesh **combined_mesh, KDTree* face_tree, Object* ob)
{
	BMOperator op;
	BMOpSlot *slot;
	int same = true;
	int shared = 0, island_vert_key_index = 0, island_vert_map_index = 0;
	int* island_verts_key = MEM_mallocN(sizeof(int), "island_verts_key");
	int* island_verts_map = MEM_mallocN(sizeof(int), "island_verts_map");
	int v;

	//check whether we are in the same object or not
	float thresh, dist;
	int con_type;// equal = mi->parent_mod == mi2->parent_mod;
	//equal = equal && (mi->parent_mod == rmd);


	thresh = rmd->breaking_threshold;
	con_type = rmd->inner_constraint_type;
	dist = rmd->contact_dist;

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
//		same = ((equal && ((ind1 == 0) || (ind2 == 0))) || (!equal) || (rmd->inner_constraint_type != RBC_TYPE_FIXED));
		//"same" does not work anymore for other constraint types than FIXED, so use it for this as distinction only
	}

	//if ((slot->data.ghash) && (slot->data.ghash->nentries > 2))
	//	printf("%d %d\n", island_vert_key_index, island_vert_map_index);

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

static int bbox_intersect(FractureModifierData *rmd, MeshIsland *mi, MeshIsland *mi2)
{
	float cent_vec[3], test_x1[3], test_y1[3], test_z1[3], test_x2[3], test_y2[3], test_z2[3];

	//pre test with bounding boxes
	//vec between centroids -> v, if v[0] > bb1[4] - bb1[0] / 2 + bb2[4] - bb[0] / 2 in x range ?
	// analogous y and z, if one overlaps, bboxes touch, make expensive test then
//	int equal = mi->parent_mod == mi2->parent_mod;
	//float dist = equal ? rmd->contact_dist : rmd->group_contact_dist;
	float dist = rmd->contact_dist; // equal ? rmd->contact_dist : rmd->outer_constraint_type == RBC_TYPE_FIXED ? rmd->group_contact_dist : 0;

	if ((mi->bb == NULL) || (mi2->bb == NULL)) {
		//compat with older files, where bb test missed
		return true;
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
				return true;
			}
		}
	}
	return false;
}

static void search_centroid_based(FractureModifierData *rmd, Object* ob, MeshIsland* mi, MeshIsland** meshIslands, KDTree**combined_tree, float centr[3])
{
	int r = 0, limit = 0, i = 0;
	KDTreeNearest* n3 = NULL;
	float dist, obj_centr[3], ratio = 1;

	//mi = meshIslands[j/*(n2+j)->index*/];
	//if (j == 0)
	//	first = mi;
	limit = rmd->constraint_limit;
	dist = rmd->contact_dist; //mi->parent_mod == rmd ? rmd->contact_dist : rmd->group_contact_dist;

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

/*	if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELL_CENTROIDS)
	{
		copy_v3_v3(obj_centr, centr);
	}*/
	else
	{
		mul_v3_m4v3(obj_centr, rmd->origmat, mi->centroid );
	}

	r = BLI_kdtree_range_search(*combined_tree, obj_centr, &n3, dist);

	//use centroid dist based approach here, together with limit ?
	for (i = 0; i < r; i++)
	{
		MeshIsland *mi2 = meshIslands[(n3+i)->index];
		if ((mi != mi2) && (mi2 != NULL))
		{
			float thresh;
			int con_type, equal, ok;
//			equal = mi->parent_mod == mi2->parent_mod;
			ok = true; // equal || (!equal && rmd->outer_constraint_type == RBC_TYPE_FIXED);
			thresh = rmd->breaking_threshold;//equal ? rmd->breaking_threshold : rmd->group_breaking_threshold;
			con_type = rmd->inner_constraint_type;//= equal ? rmd->inner_constraint_type : rmd->outer_constraint_type;

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

#if 0
KDTree* make_cell_tree(FractureModifierData* rmd, Object* ob)
{
	float min[3], max[3], start[3], dim[3], co[3], csize, size[3];
	int cells[3], index = 0, i, j, k;
	KDTree* tree;
	BoundBox* bb = BKE_boundbox_alloc_unit();
	DerivedMesh* dm = CDDM_from_mesh(ob->data);
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
				BLI_kdtree_insert(tree, index, co);
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

void search_cell_based(FractureModifierData *rmd, Object* ob,  MeshIsland *mi, KDTree** cells)
{
	KDTreeNearest *n = NULL;
	//use search distance based on cell volume ?
	int i, r = 0, j;
	float obj_centr[3], dim[3], dist;
	bbox_dim(mi->bb, dim);

	dist = MAX3(dim[0], dim[1], dim[2]) + rmd->cell_size + rmd->contact_dist;
	mul_v3_m4v3(obj_centr, ob->obmat, mi->centroid);
	r = BLI_kdtree_range_search(*cells, obj_centr, &n, dist);

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

void search_cell_centroid_based(FractureModifierData *rmd, Object* ob,  MeshIsland *mi, MeshIsland** meshIslands, KDTree** combined_tree, KDTree** cells)
{
	KDTreeNearest *n = NULL;
	//use search distance based on cell volume ?
	int i, r = 0, j;
	float obj_centr[3], dim[3], dist, co[3];
	bbox_dim(mi->bb, dim);

	dist = MAX2(MAX3(dim[0], dim[1], dim[2]), rmd->cell_size);
	mul_v3_m4v3(obj_centr, ob->obmat, mi->centroid);
	//copy_v3_v3(obj_centr, mi->centroid);
	r = BLI_kdtree_range_search(*cells, obj_centr, &n, dist);
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
#endif

void connect_constraints(FractureModifierData* rmd,  Object* ob, MeshIsland **meshIslands, int count, BMesh **combined_mesh, KDTree **combined_tree)
{

	MeshIsland *mi, *first, *last;
	int i, j, v; //, sel_counter;
	KDTreeNearest *n = MEM_mallocN(sizeof(KDTreeNearest)*count, "kdtreenearest");
	KDTreeNearest *n2 = MEM_mallocN(sizeof(KDTreeNearest)*count, "kdtreenearest2");
	//ExplodeModifierData *emd = NULL;

	KDTree* face_tree;
	BMFace *fa;
	BMIter bmi;

	face_tree = BLI_kdtree_new(rmd->visible_mesh->totface);
	BM_ITER_MESH(fa, &bmi, rmd->visible_mesh, BM_FACES_OF_MESH)
	{
		float co[3];
		BM_face_calc_center_bounds(fa, co);
		BLI_kdtree_insert(face_tree, fa->head.index, co);
	}

	BLI_kdtree_balance(face_tree);

	//Do we have a explo modifier, if yes, use its neighborhood info before calculating (inner) neighborhoods here

	//emd = findPrecedingExploModifier(ob, rmd);
	//if (emd != NULL && emd->cells != NULL && ((!emd->use_clipping && !rmd->use_cellbased_sim  && !emd->use_boolean) ||
	//	((rmd->contact_dist_meaning == MOD_RIGIDBODY_VERTICES) && (rmd->contact_dist == 0.0f)))) {
	if (rmd->contact_dist_meaning == MOD_RIGIDBODY_VERTICES && rmd->explo_shared) //check whether it was FRACTURED!!! TODO...XXXXX
	{
		//KDTree* cells = NULL;
		int i = 0, j;
		GHash* visited_ids = BLI_ghash_pair_new("visited_ids");
		for (mi = rmd->meshIslands.first; mi; mi = mi->next)
		{
			MeshIsland* mi2;
			int shared = 0;

			for (i = 0; i < mi->neighbor_count; i++)
			{
				int id = mi->neighbor_ids[i];
				int index;
				if (id >= 0) {
					index = GET_INT_FROM_POINTER(BLI_ghash_lookup(rmd->idmap, SET_INT_IN_POINTER(id)));
					mi2 = BLI_findlink(&rmd->meshIslands, index);
					if ((mi != mi2) && (mi2 != NULL)) {
						GHashPair* id_pair = BLI_ghashutil_pairalloc(id, mi->id);

						if (!BLI_ghash_haskey(visited_ids, id_pair)) {
							//shared = check_meshislands_adjacency(rmd, mi, mi2, combined_mesh, face_tree, ob);
							//RigidBodyShardCon *con;
							//int con_found = FALSE;
							BLI_ghash_insert(visited_ids, id_pair, SET_INT_IN_POINTER(i));
							connect_meshislands(rmd, ob, mi, mi2, rmd->inner_constraint_type, rmd->breaking_threshold);
						}
						else
						{
							BLI_ghashutil_pairfree(id_pair);
						}
					}
				}
			}
		}

		BLI_ghash_free(visited_ids, BLI_ghashutil_pairfree, NULL);
		visited_ids = NULL;
	}
	else {

		//if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELLS || rmd->contact_dist_meaning == MOD_RIGIDBODY_CELL_CENTROIDS)
		{
			//cells = make_cell_tree(rmd, ob);
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
//			else if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELLS)
//			{
				//search_cell_based(rmd, ob, meshIslands[j], &cells);
//			}
//			else if (rmd->contact_dist_meaning == MOD_RIGIDBODY_CELL_CENTROIDS)
//			{
				//search_cell_centroid_based(rmd, ob, meshIslands[j], meshIslands, combined_tree, &cells);
//			}
			else if (rmd->contact_dist_meaning == MOD_RIGIDBODY_VERTICES && !rmd->explo_shared)//use vertex distance as FALLBACK
			{
				int shared = 0;
				BLI_kdtree_find_nearest_n(*combined_tree, meshIslands[0]->centroid, n2, count);
				mi = meshIslands[(n2+j)->index];

				for (v = 0; v < mi->vertex_count; v++) {
					BM_elem_flag_enable(BM_vert_at_index(*combined_mesh, mi->combined_index_map[v]), BM_ELEM_TAG);
				}

				BLI_kdtree_find_nearest_n(*combined_tree, mi->centroid, n, count);
				for (i = 0; i < count; i++) {
					MeshIsland *mi2 = meshIslands[(n+i)->index];
					//printf("Nearest: %d %d %f %f %f\n",m, (n+i)->index, (n+i)->co[0], (n+i)->co[2], (n+i)->co[2]);
					if ((mi != mi2) && (mi2 != NULL)) {
						//pre test with bounding boxes
						//vec between centroids -> v, if v[0] > bb1[4] - bb1[0] / 2 + bb2[4] - bb[0] / 2 in x range ?
						// analogous y and z, if one overlaps, bboxes touch, make expensive test then
						int bbox_int = bbox_intersect(rmd, mi, mi2);
						//printf("Overlap %d %d %d\n", bbox_int, (n2+j)->index, (n+i)->index);
						if ((bbox_int == false)) // && (mi->parent_mod == mi2->parent_mod))
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
#if 0
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
							equal = true; //mi->parent_mod == mi2->parent_mod;
							ok = equal; //|| (!equal && rmd->outer_constraint_type == RBC_TYPE_FIXED);
							thresh = rmd->breaking_threshold; //equal ? rmd->breaking_threshold : rmd->group_breaking_threshold;
							con_type = rmd->inner_constraint_type;//  equal ? rmd->inner_constraint_type : rmd->outer_constraint_type;
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
#endif
	}
	MEM_freeN(n);
	MEM_freeN(n2);
	BLI_kdtree_free(face_tree);
	face_tree = NULL;
}

static int create_combined_neighborhood(FractureModifierData *rmd, MeshIsland ***mesh_islands, BMesh **combined_mesh, KDTree **combined_tree)
{
	ModifierData* md;
	FractureModifierData* rmd2;
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
			if (mi->vertices)
			{
				copy_v3_v3(co, mi->vertices[v]->co);
			}
			else
			{
				copy_v3_v3(co, mi->vertices_cached[v]->co);
			}
			BM_vert_create(*combined_mesh, co, NULL, 0);
			mi->combined_index_map[v] = vert_counter;
			vert_counter++;
		}
		(*mesh_islands)[i] = mi;
		i++;
	}

	*combined_tree = BLI_kdtree_new(islands);
	for (i = 0; i < islands; i++) {
		float obj_centr[3];
		mul_v3_m4v3(obj_centr, rmd->origmat, (*mesh_islands)[i]->centroid );
		BLI_kdtree_insert(*combined_tree, i, obj_centr);
	}

	BLI_kdtree_balance(*combined_tree);

	return islands;
}

static void create_constraints(FractureModifierData *rmd, Object* ob)
{
	KDTree* combined_tree = NULL;
	BMesh* combined_mesh = NULL;
	MeshIsland** mesh_islands = MEM_mallocN(sizeof(MeshIsland*), "mesh_islands");
	int count, i = 0;

	if (rmd->visible_mesh && rmd->contact_dist == 0.0f)
	{
		float min[3], max[3], dim[3];
		BoundBox *bb = BKE_boundbox_alloc_unit();
		BM_mesh_minmax(rmd->visible_mesh, min, max, 0);
		BKE_boundbox_init_from_minmax(bb, min, max);
		bbox_dim(bb, dim);
		rmd->contact_dist = MAX3(dim[0], dim[1], dim[2]);
		MEM_freeN(bb);
	}


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
	BLI_kdtree_find_nearest_n(tree, co, n, 2);

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

#if 0
void buildCompounds(FractureModifierData *rmd, Object *ob)
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
		BLI_kdtree_insert(centroidtree, index, obj_centr);
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
		r = BLI_kdtree_range_search(centroidtree, co, &n, rmd->cell_size);
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
		BM_mesh_minmax(bm_compound, min, max, false);
		if (ob->rigidbody_object->shape == RB_SHAPE_COMPOUND || rmd->use_constraints)
		{
			//compounds need center of mass
			BM_calc_center_centroid(bm_compound, centroid, false);
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

		mi_compound->physics_mesh = CDDM_from_bmesh(bm_compound, true);
		BM_mesh_free(bm_compound);
		copy_v3_v3(mi_compound->centroid, centroid);
		mat4_to_loc_quat(dummyloc, rot, ob->obmat);
		copy_v3_v3(mi_compound->rot, rot);
//		mi_compound->parent_mod = rmd;
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
#endif

static DerivedMesh* createCache(FractureModifierData *rmd, Object* ob, DerivedMesh *origdm)
{
	MeshIsland *mi;
	BMVert* v;
	DerivedMesh *dm;
	MVert *verts;
	MDeformVert *dvert;
	int vertstart = 0;
	const int thresh_defgrp_index = defgroup_name_index(ob, rmd->thresh_defgrp_name);

	if (rmd->dm && !rmd->shards_to_islands && (rmd->dm->getNumPolys(rmd->dm) > 0))
	{
		dm = CDDM_copy(rmd->dm);
	}
	else if (rmd->visible_mesh && (rmd->visible_mesh->totface > 0) && BLI_countlist(&rmd->meshIslands) > 1)
	{
		dm = CDDM_from_bmesh(rmd->visible_mesh, true);
	}
	else if (origdm != NULL)
	{
		dm = CDDM_copy(origdm);
	}
	else
	{
		return NULL;
	}

	DM_ensure_tessface(dm);
	DM_ensure_normals(dm);
	DM_update_tessface_data(dm);

	verts = dm->getVertArray(dm);
	dvert = dm->getVertDataArray(dm, CD_MDEFORMVERT);

	for (mi = rmd->meshIslands.first; mi; mi = mi->next)
	{
		int i = 0;
		if (mi->vertices_cached)
		{
			MEM_freeN(mi->vertices_cached);
			mi->vertices_cached = NULL;
		}

		mi->vertices_cached = MEM_mallocN(sizeof(MVert*) * mi->vertex_count, "mi->vertices_cached");
		if (rmd->dm != NULL && !rmd->shards_to_islands)
		{
			for (i = 0; i < mi->vertex_count; i++)
			{
				mi->vertices_cached[i] = verts + vertstart + i;
			}

			//sum up vertexweights and divide by vertcount to get islandweight
			if (dvert && dvert->dw && rmd->thresh_defgrp_name[0]) {
				float vweight = defvert_find_weight(dvert + vertstart + i, thresh_defgrp_index);
				mi->thresh_weight += vweight;
			}

			vertstart += mi->vertex_count;
		}
		else
		{
			for (i = 0; i < mi->vertex_count; i++)
			{
				//int index = mi->vertices[i]->head.index;
				int index = mi->vertex_indices[i];
				if (index >= 0 && index <= rmd->visible_mesh->totvert)
				{
					mi->vertices_cached[i] = verts + index;
					/*mi->vertco[3*i] = mi->vertices_cached[i]->co[0];
					mi->vertco[3*i+1] = mi->vertices_cached[i]->co[1];
					mi->vertco[3*i+2] = mi->vertices_cached[i]->co[2];*/
				}
				else
				{
					mi->vertices_cached[i] = NULL;
				}

				if (dvert && dvert->dw && rmd->thresh_defgrp_name[0]) {
					float vweight = defvert_find_weight(dvert + index, thresh_defgrp_index);
					mi->thresh_weight += vweight;
				}
			}
			//vertstart += mi->vertex_count;
		}

		if (mi->vertex_count > 0)
		{
			mi->thresh_weight /= mi->vertex_count;
		}
	}

	return dm;
}

static void refresh_customdata_image(Mesh* me, CustomData *pdata, int totface)
{
	int i;

	for (i=0; i < pdata->totlayer; i++) {
		CustomDataLayer *layer = &pdata->layers[i];

		if (layer->type == CD_MTEXPOLY) {
			MTexPoly *tf= layer->data;
			int j;

			for (j = 0; j < totface; j++, tf++) {
				//simply use first image here...
				tf->tpage = me->mtpoly->tpage;
				tf->mode = me->mtpoly->mode;
				tf->flag= me->mtpoly->flag;
				tf->tile = me->mtpoly->tile;
				tf->transp = me->mtpoly->transp;

				if (tf->tpage && tf->tpage->id.us == 0) {
					tf->tpage->id.us = 1;
				}
			}
		}
	}
}

DerivedMesh* doSimulate(FractureModifierData *fmd, Object* ob, DerivedMesh* dm)
{
	//BMesh* temp = NULL;
	bool exploOK = false; //doFracture
	double start;
	//bool shared = false;

	if ((fmd->refresh) || (fmd->refresh_constraints && !fmd->execute_threaded) ||
	        (fmd->refresh_constraints && fmd->execute_threaded && fmd->frac_mesh && fmd->frac_mesh->running == 0))
	{
		//shared = fmd->explo_shared;
		//fmd->explo_shared = false;
		//if we changed the fracture parameters (or the derivedmesh ???

		freeData(fmd);
		//fmd->explo_shared = shared;

		if (fmd->visible_mesh != NULL && !fmd->shards_to_islands && fmd->frac_mesh->shard_count > 0 && fmd->refresh)
		{
			if (fmd->visible_mesh_cached)
			{
				fmd->visible_mesh_cached->needsFree = 1;
				fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
			}
			fmd->visible_mesh_cached = NULL;
		}

		//fmd->split = destroy_compound;
		//fmd->join = buildCompounds;

//		if (fmd->contact_dist_meaning != MOD_RIGIDBODY_CELL_CENTROIDS)
		{
			fmd->use_cellbased_sim = false;
		}

		if (fmd->refresh)
		{
			fmd->idmap = BLI_ghash_int_new("fmd->idmap");
			copy_m4_m4(fmd->origmat, ob->obmat);

			//grab neighborhood info (and whole fracture info -> cells) if available, if explo before rmd
			//emd = findPrecedingExploModifier(ob, rmd);
//			if (emd != NULL && emd->cells != NULL && ((!emd->use_clipping && !rmd->use_cellbased_sim /* && !emd->use_boolean*/) || rmd->contact_dist_meaning == ////MOD_RIGIDBODY_VERTICES))
			if (fmd->frac_mesh && fmd->frac_mesh->shard_count > 0 && fmd->dm && fmd->dm->numVertData > 0 && !fmd->shards_to_islands)
			{
				Shard *s;
				MeshIsland* mi; // can be created without shards even, when using fracturemethod = NONE
				//BMIter iter;
				int i, j, vertstart = 0;
				//BMVert *v;
				MVert* v;
				float dummyloc[3], rot[4], min[3], max[3];
				MDeformVert *dvert = fmd->dm->getVertDataArray(fmd->dm, CD_MDEFORMVERT);
				const int thresh_defgrp_index = defgroup_name_index(ob, fmd->thresh_defgrp_name);

				//good idea to simply reference this ? Hmm, what about removing the explo modifier later, crash ?)
				fmd->explo_shared = true;

				if (fmd->visible_mesh_cached)
				{
					fmd->visible_mesh_cached->needsFree = 1;
					fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
					fmd->visible_mesh_cached = NULL;
				}

				fmd->visible_mesh_cached = CDDM_copy(fmd->dm); //DM_to_bmesh(fmd->dm, true);

				for (i = 0; i < fmd->frac_mesh->shard_count; i++)
				{
					MVert *mv, *verts, *mverts;
					int totvert, k;

					s = fmd->frac_mesh->shard_map[i];
					if (s->totvert == 0)
					{
						continue;
					}

					fmd->frac_mesh->progress_counter++;

					// add 1 MeshIsland
					mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
					BLI_addtail(&fmd->meshIslands, mi);

					mi->participating_constraints = NULL;
					mi->participating_constraint_count = 0;
					mi->thresh_weight = 0;

					//mi->is_at_boundary = vc->is_at_boundary;
					//mi->vertices = MEM_mallocN(sizeof(BMVert*) * s->totvert, "vert_cache");
					mi->vertices_cached = MEM_mallocN(sizeof(MVert*) * s->totvert, "vert_cache");
					mverts = CDDM_get_verts(fmd->visible_mesh_cached);
					for (k = 0; k < s->totvert; k++)
					{
						mi->vertices_cached[k] = mverts + vertstart + k;
						//mi->vertices[k] = BM_vert_at_index_find(fmd->visible_mesh, vertstart+k);
						//sum up vertexweights and divide by vertcount to get islandweight
						if (dvert && fmd->thresh_defgrp_name[0]) {
							float vweight = defvert_find_weight(dvert + vertstart + k, thresh_defgrp_index);
							mi->thresh_weight += vweight;
						}
					}
					vertstart += s->totvert;
					//mi->vertco = get_vertco(s);// ? starting coordinates ???
					mi->physics_mesh = BKE_shard_create_dm(s, false);
					totvert = mi->physics_mesh->numVertData;
					verts = mi->physics_mesh->getVertArray(mi->physics_mesh);
					mi->vertco = MEM_mallocN(sizeof(float), "vertco");
					for (mv = verts, j = 0; j < totvert; mv++, j++)
					{
						mi->vertco = MEM_reallocN(mi->vertco, sizeof(float) * 3 * (j+1));
						mi->vertco[j*3] = mv->co[0];
						mi->vertco[j*3+1] = mv->co[1];
						mi->vertco[j*3+2] = mv->co[2];
						//then eliminate centroid in vertex coords ?
						sub_v3_v3(mv->co, s->centroid);
					}

					BKE_shard_calc_minmax(s);
					mi->vertex_count = s->totvert;
					copy_v3_v3(mi->centroid, s->centroid);
					mat4_to_loc_quat(dummyloc, rot, ob->obmat);
					copy_v3_v3(mi->rot, rot);

					//mi->parent_mod = fmd; //Fracture individual islands, fracture all, island detection afterwards ?

					mi->bb = BKE_boundbox_alloc_unit();
					BKE_boundbox_init_from_minmax(mi->bb, s->min, s->max);

					mi->id = s->shard_id;
					mi->particle_index = s->cluster_colors[0];
					BLI_ghash_insert(fmd->idmap, SET_INT_IN_POINTER(mi->id), SET_INT_IN_POINTER(i));
					mi->neighbor_ids = s->neighbor_ids;
					mi->neighbor_count = s->neighbor_count;
					//mi->global_face_map = vc->global_face_map;

					mi->rigidbody = NULL;
					//mi->vertices_cached = NULL;
					if (!fmd->use_cellbased_sim)
					{
						mi->destruction_frame = -1;
						mi->rigidbody = BKE_rigidbody_create_shard(fmd->modifier.scene, ob, mi);
						mi->rigidbody->flag &= ~RBO_FLAG_ACTIVE_COMPOUND;
						BKE_rigidbody_calc_shard_mass(ob, mi);
					}
					mi->vertex_indices = NULL;

					if (mi->vertex_count > 0)
					{
						mi->thresh_weight /= mi->vertex_count;
					}
				}
			}
			else
			{
				if (fmd->visible_mesh == NULL)
				{
					if (fmd->dm && fmd->shards_to_islands)
					{
						fmd->visible_mesh = DM_to_bmesh(fmd->dm, true);
					}
					else
					{
						//split to meshislands now
						fmd->visible_mesh = DM_to_bmesh(dm, true); //ensures indexes automatically
					}

					start = PIL_check_seconds_timer();
					printf("Steps: %d \n", fmd->frac_mesh->progress_counter);
					mesh_separate_loose(fmd, ob);
					printf("Splitting to islands done, %g  Steps: %d \n", PIL_check_seconds_timer() - start, fmd->frac_mesh->progress_counter);
				}

				fmd->explo_shared = false;
			}

			printf("Islands: %d\n", BLI_countlist(&fmd->meshIslands));

#if 0
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

			if (fmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED && fmd->framemap != NULL)
			{
				int index = 0;
				for (mi = fmd->meshIslands.first; mi; mi = mi->next)
				{
					destroy_compound(fmd, ob, mi, fmd->framemap[index]);
					index++;
					//if (mi->rigidbody)
						//mi->rigidbody->flag &= ~RBO_FLAG_BAKED_COMPOUND;
				}
			}

			if (!fmd->refresh_constraints)
			{
				//rebuild framemap after using it when refreshing all data
				if (!(fmd->modifier.scene->rigidbody_world->pointcache->flag & PTCACHE_BAKED))
				{
					if (fmd->framemap != NULL)
					{
						MEM_freeN(rmd->framemap);
						fmd->framemap = NULL;
						fmd->framecount = 0;
					}
				}

				if (fmd->framemap == NULL)
				{
					fmd->framecount = count;
					fmd->framemap = MEM_callocN(sizeof(float) * fmd->framecount, "framemap");
					for (i = 0; i < fmd->framecount; i++)
					{
						fmd->framemap[i] = -1;
					}
				}
			}
		}
#endif
		}

		start = PIL_check_seconds_timer();

		if ((fmd->visible_mesh != NULL && fmd->refresh && (!fmd->explo_shared)) || (fmd->visible_mesh_cached == NULL))
		{
			start = PIL_check_seconds_timer();
			//post process ... convert to DerivedMesh only at refresh times, saves permanent conversion during execution
			if (fmd->visible_mesh_cached != NULL)
			{
				fmd->visible_mesh_cached->needsFree = 1;
				fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
				fmd->visible_mesh_cached = NULL;
			}

			if (fmd->refresh_images && fmd->dm)
			{
				//need to ensure images are correct after loading...
				refresh_customdata_image(ob->data, &fmd->dm->polyData,
				                         fmd->dm->getNumPolys(fmd->dm));
				fmd->refresh_images = false;
				//DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
				//ob->recalc &= OB_RECALC_ALL;
			}

			if ((fmd->visible_mesh != NULL) && (fmd->shards_to_islands))
			{
				//meshisland_init_verts(fmd);
			}

			fmd->visible_mesh_cached = createCache(fmd, ob, dm);


			printf("Building cached DerivedMesh done, %g\n", PIL_check_seconds_timer() - start);
		}

		if (fmd->refresh_images && fmd->visible_mesh_cached && fmd->shards_to_islands)
		{
			//need to ensure images are correct after loading...
			refresh_customdata_image(ob->data, &fmd->visible_mesh_cached->polyData,
			                         fmd->visible_mesh_cached->getNumPolys(fmd->visible_mesh_cached));
			fmd->refresh_images = false;
			DM_update_tessface_data(fmd->visible_mesh_cached);
			//DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
			//ob->recalc &= OB_RECALC_ALL;
		}

		if ((fmd->visible_mesh != NULL || fmd->visible_mesh_cached != NULL )  && (fmd->use_constraints))
		{
			if (fmd->visible_mesh == NULL)
			{	//ugh, needed to build constraints...
				fmd->visible_mesh = DM_to_bmesh(fmd->visible_mesh_cached, true);
			}
			create_constraints(fmd, ob); //check for actually creating the constraints inside

			if (fmd->visible_mesh_cached != NULL)
			{
				BM_mesh_free(fmd->visible_mesh);
				fmd->visible_mesh = NULL;
			}
		}

		printf("Building constraints done, %g\n", PIL_check_seconds_timer() - start);
		printf("Constraints: %d\n", BLI_countlist(&fmd->meshConstraints));

		fmd->refresh = false;
		fmd->refresh_constraints = false;

		if (fmd->execute_threaded)
		{
			//job done
			fmd->frac_mesh->running = 0;
		}
	}

	//emd = findPrecedingExploModifier(ob, rmd);
	exploOK = !fmd->explo_shared || (fmd->explo_shared && fmd->dm && fmd->frac_mesh);
	//exploOK = true;//shards and shards there or no shards...

	if ((!exploOK) || (fmd->visible_mesh == NULL && fmd->visible_mesh_cached == NULL))
	{
		MeshIsland* mi;
		//nullify invalid data
		for (mi = fmd->meshIslands.first; mi; mi = mi->next)
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

		if (fmd->visible_mesh_cached)
		{
			fmd->visible_mesh_cached->needsFree = 1;
			fmd->visible_mesh_cached->release(fmd->visible_mesh_cached);
			fmd->visible_mesh_cached = NULL;
		}
	}

	if ((fmd->visible_mesh != NULL) && exploOK)
	{
		DerivedMesh *dm_final;
		{
			//dm_final = rmd->visible_mesh_cached;
			dm_final = CDDM_copy(fmd->visible_mesh_cached);
			//dm_final = CDDM_from_bmesh(rmd->visible_mesh, TRUE);
		}

		return dm_final;
	}
	else if ((fmd->visible_mesh_cached != NULL) && exploOK)
	{
		DerivedMesh* dm_final;
		dm_final = CDDM_copy(fmd->visible_mesh_cached);
		return dm_final;
	}
	else
	{
		if (fmd->visible_mesh == NULL && fmd->visible_mesh_cached == NULL)
		{
			//oops, something went definitely wrong...
			/*if (emd)
			{
				rmd->refresh = TRUE;
			}*/
			fmd->refresh = true;
			freeData(fmd);
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

/*static void foreachIDLink(ModifierData *md, Object *ob,
						  IDWalkFunc walk, void *userData)
{
	RigidBodyModifierData *rmd = (RigidBodyModifierData *) md;

	walk(userData, ob, (ID **)&rmd->constraint_group);
}*/

static CustomDataMask requiredDataMask(Object *UNUSED(ob), ModifierData *UNUSED(md))
{
	CustomDataMask dataMask = 0;
	dataMask |= CD_MASK_MDEFORMVERT;
	return dataMask;
}

#if 0
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
#endif


ModifierTypeInfo modifierType_Fracture = {
        /* name */              "Fracture",
        /* structName */        "FractureModifierData",
        /* structSize */        sizeof(FractureModifierData),
        /* type */              eModifierTypeType_Constructive,//eModifierTypeType_OnlyDeform,
        /* flags */             eModifierTypeFlag_AcceptsMesh |
                                eModifierTypeFlag_Single |
                                eModifierTypeFlag_SupportsEditmode |
                                eModifierTypeFlag_SupportsMapping,
        /* copyData */          copyData,
        /* deformVerts */       NULL,
        /* deformMatrices */    NULL,
        /* deformVertsEM */     NULL,
        /* deformMatricesEM */  NULL,
        /* applyModifier */     applyModifier,
        /* applyModifierEM */   applyModifierEM,
        /* initData */          initData,
        /* requiredDataMask */  requiredDataMask,
        /* freeData */          freeData,
        /* isDisabled */        NULL,
        /* updateDepgraph */    NULL,
        /* dependsOnTime */     dependsOnTime,
        /* dependsOnNormals */  dependsOnNormals,
        /* foreachObjectLink */ NULL,
        /* foreachIDLink */     NULL,
};
