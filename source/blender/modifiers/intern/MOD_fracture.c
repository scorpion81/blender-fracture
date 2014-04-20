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

#include "DNA_fracture_types.h"
#include "BKE_fracture.h"
#include "BKE_cdderivedmesh.h"
#include "BLI_rand.h"
#include "MOD_util.h"
#include "MEM_guardedalloc.h"
#include "BLI_math_vector.h"
#include "BLI_math_matrix.h"
#include "DNA_scene_types.h"
#include "DNA_particle_types.h"
#include "BKE_particle.h"
#include "DNA_gpencil_types.h"
#include "DNA_group_types.h"
#include "BKE_object.h"
#include "BLI_listbase.h"
#include "BLI_string_utf8.h"
#include "BKE_material.h"
#include "DNA_material_types.h"
#include "BKE_global.h"
#include "BLI_kdtree.h"

static void do_fracture(FractureModifierData *fracmd, ShardID id, Object *obj, DerivedMesh* dm, FractureLevel *fl);

static void initData(ModifierData *md)
{

		FractureModifierData *fmd = (FractureModifierData*) md;
		//fmd->totlevel = 1;
		FractureLevel *fl = MEM_callocN(sizeof(FractureLevel), "fraclevel");

		fmd->cluster_count = 5;
		//char* str;
		//int length;
		fl->name = "Fracture Level 1\0";
		/*str = "Fracture Level 1";
		length = BLI_strlen_utf8(str) * 2;

		fl->name = MEM_mallocN(sizeof(char) * length, "str");
		fl->name = BLI_strncpy_utf8(fl->name, str, length);*/
		fl->extra_group = NULL;
		fl->frac_algorithm = MOD_FRACTURE_VORONOI;
		fl->shard_id = -1;
		fl->shard_count = 10;
		fl->percentage = 100;
		BLI_addtail(&fmd->fracture_levels, fl);
		fmd->active_index = 0;
}

static void freeData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData*) md;
	FractureLevel* fl;
	
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
	
	BKE_fracmesh_free(fmd->frac_mesh);
	MEM_freeN(fmd->frac_mesh);

	for (fl = fmd->fracture_levels.first; fl; fl = fl->next)
	{
		MEM_freeN(fl->noisemap);
		MEM_freeN(fl);
	}
}

static void growCluster(FractureModifierData *fmd, Shard* seed, int sindex, ListBase* lbVisit, KDTree* tree, int depth)
{
	int i = 0, ret = 0, count = 0;
	float size = 0.1f;
	KDTreeNearest* nearest;

	/*if (BLI_findindex(lbVisit, seed) == -1)
	{
		BLI_addtail(lbVisit, seed);
	}

	if (seed->cluster_colors[0] == -1)
	{
		seed->cluster_colors[0] = sindex;
		BKE_shard_assign_material(seed, (short)sindex);
	}
	else*/
	{
		count = BLI_kdtree_range_search(tree, seed->centroid, &nearest, size*depth);
		for (i = 0; i < count; i++)
		{
			Shard* neighbor;
			int index = nearest[i].index;
			neighbor = fmd->frac_mesh->shard_map[index];
			if (neighbor->cluster_colors[0] == -1)
			{
				neighbor->cluster_colors[0] = sindex;
				BKE_shard_assign_material(neighbor, (short)sindex);
			}

			if (BLI_findindex(lbVisit, seed) == -1)
			{
				BLI_addtail(lbVisit, seed);
			}
		}
		MEM_freeN(nearest);

		//growCluster(fmd, neighbor, sindex, lbVisit, tree, depth+1);

		/*for (i = 0; i < seed->neighbor_count; i++)
		{
			Shard* neighbor;
			int n_id = seed->neighbor_ids[i];
			if (n_id > -1)
			{
				neighbor = fmd->frac_mesh->shard_map[n_id];
				//if (BLI_findindex(lbVisit, neighbor) == -1)
				if (neighbor->cluster_colors[0] == -1)
				{
					ret += growCluster(fmd, neighbor, sindex, lbVisit);
					break;
				}
			}
		}*/
	}

	//return ret;
}

/*static void growCluster(FractureModifierData *fmd, Shard* seed, int sindex, ListBase* lbVisit)
{
	int i = 0 , n_id;
	Shard *neighbor = NULL;

	if (BLI_findindex(lbVisit, seed) == -1)
	{
		if (seed->cluster_colors[0] == -1)
		{
			seed->cluster_colors[0] = sindex;
			BKE_shard_assign_material(seed, (short)sindex);
		}

		BLI_addtail(lbVisit, seed);
	}
	else
	{
		do
		{
			if (i == seed->neighbor_count)
			{
				//try with a neighbor..
				n_id = seed->neighbor_ids[0];.
				if (n_id > -1)
				{
					neighbor = fmd->frac_mesh->shard_map[n_id];
					growCluster(fmd, neighbor, sindex, lbVisit);
				}
			}

			n_id = seed->neighbor_ids[i];
			if (n_id > -1)
			{
				neighbor = fmd->frac_mesh->shard_map[n_id];
			}
			i++;
		}
		while ((neighbor == NULL) || (neighbor->cluster_colors[0] != -1));

		//for (i = 0; i < seed->neighbor_count; i++)
		if (BLI_findindex(lbVisit, neighbor) == -1)
		{
			growCluster(fmd, neighbor, sindex, lbVisit);
		}
	}
}*/

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

		BLI_kdtree_balance(tree);

		//remove materials...
		for (i = 0; i < ob->totcol; i++)
		{
			BKE_material_pop_id(ob, i, true);
		}

		seed_count = fmd->cluster_count;// (int)(BLI_frand() * BLI_countlist(&lb));
		seeds = MEM_mallocN(sizeof(Shard*) * seed_count, "seeds");

		for (k = 0; k < seed_count; k++)
		{
			mat = BKE_material_add(G.main, "Cluster");
			mat->r = BLI_frand();
			mat->g = BLI_frand();
			mat->b = BLI_frand();
			assign_material(ob, mat, (short)k+1, BKE_MAT_ASSIGN_OBDATA);
		}

		for (k = 0; k < seed_count; k++)
		{
			int color = k;
			int which_index = k * (int)(fmd->frac_mesh->shard_count / seed_count);
			Shard *which = fmd->frac_mesh->shard_map[which_index];
			which->cluster_colors[j] = color;
			BKE_shard_assign_material(which, (short)color);
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
	FractureLevel *fl;

	//int shard_requested = 0;

	/*if (fmd->frac_mesh != NULL)
	{
		BKE_fracmesh_free(fmd->frac_mesh);
		MEM_freeN(fmd->frac_mesh);
	}*/

	if (fmd->dm != NULL)
	{
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}

	if (fmd->frac_mesh != NULL)
	{
		BKE_fracmesh_free(fmd->frac_mesh);
		MEM_freeN(fmd->frac_mesh);
	}

	fmd->frac_mesh = BKE_create_fracture_container(derivedData);

	if (!fmd->dm) {
		do_fracture(fmd, -1, ob, derivedData, fmd->fracture_levels.first);
	}

	if (fmd->dm)
		final_dm = CDDM_copy(fmd->dm);
	else
		final_dm = derivedData;

#if 0
	for (fl = fmd->fracture_levels.first; fl; fl = fl->next)
	{
		/*int i = 0;
		int shard_total = fmd->frac_mesh->shard_count;

		shard_requested += fl->shard_count;
		for (i = shard_total; i < shard_requested; i++)
		{*/
		do_fracture(fmd, fl->shard_id, ob, derivedData, fl);
		//}

		if (fmd->frac_mesh && fl->shard_id == 0 && fmd->dm)
		{	//recreate container, if shard 0 is to be fractured
			BKE_fracmesh_free(fmd->frac_mesh);
			MEM_freeN(fmd->frac_mesh);

			fmd->frac_mesh = BKE_create_fracture_container(fmd->dm);
			final_dm = CDDM_copy(fmd->dm);

			//fmd->dm->needsFree = 1;
			//fmd->dm->release(fmd->dm);
			//fmd->dm = NULL;
		}
		else
		{
			if (fmd->dm)
				final_dm = CDDM_copy(fmd->dm);
			else
				final_dm = derivedData;
		}
	}
#endif

	return final_dm;
}

static DerivedMesh *applyModifierEM(ModifierData *md, Object *ob,
                                    struct BMEditMesh *UNUSED(editData),
                                    DerivedMesh *derivedData,
                                    ModifierApplyFlag UNUSED(flag))
{
	FractureModifierData *fmd = (FractureModifierData*) md;
	DerivedMesh *final_dm = derivedData;
	FractureLevel *fl = fmd->fracture_levels.first;
	//int shard_requested = 0;

	/*if (fmd->frac_mesh != NULL)
	{
		BKE_fracmesh_free(fmd->frac_mesh);
		MEM_freeN(fmd->frac_mesh);
	}*/

	if (fmd->dm != NULL)
	{
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}

	if (fmd->frac_mesh != NULL)
	{
		BKE_fracmesh_free(fmd->frac_mesh);
		MEM_freeN(fmd->frac_mesh);
	}

	fmd->frac_mesh = BKE_create_fracture_container(derivedData);

	if (!fmd->dm) {
		do_fracture(fmd, -1, ob, derivedData, fmd->fracture_levels.first);
	}

	if (fmd->dm)
		final_dm = CDDM_copy(fmd->dm);
	else
		final_dm = derivedData;

#if 0
	for (fl = fmd->fracture_levels.first; fl; fl = fl->next)
	{
		/*int i = 0;
		int shard_total = fmd->frac_mesh->shard_count;

		shard_requested += fl->shard_count;
		for (i = shard_total; i < shard_requested; i++)
		{*/
		do_fracture(fmd, fl->shard_id, ob, derivedData, fl);
		//}

		if (fmd->frac_mesh && fl->shard_id == 0 && fmd->dm)
		{	//recreate container, if shard 0 is to be fractured
			BKE_fracmesh_free(fmd->frac_mesh);
			MEM_freeN(fmd->frac_mesh);

			fmd->frac_mesh = BKE_create_fracture_container(fmd->dm);
			final_dm = CDDM_copy(fmd->dm);

			//fmd->dm->needsFree = 1;
			//fmd->dm->release(fmd->dm);
			//fmd->dm = NULL;
		}
		else
		{
			if (fmd->dm)
				final_dm = CDDM_copy(fmd->dm);
			else
				final_dm = derivedData;
		}
	}

#endif

	return final_dm;
}




static void points_from_verts(Object** ob, int totobj, FracPointCloud* points, float mat[4][4], float thresh, FractureModifierData *emd, FractureLevel* fl, DerivedMesh* dm, Object* obj)
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
					   ((fl->point_source & MOD_FRACTURE_EXTRA_VERTS) &&
					   (!(fl->point_source & MOD_FRACTURE_OWN_VERTS)) && (o == 0)))
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
								 float thresh, FractureModifierData* emd, FractureLevel* fl)
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
						   ((fl->point_source & MOD_FRACTURE_EXTRA_PARTICLES) &&
						   (!(fl->point_source & MOD_FRACTURE_OWN_PARTICLES)) && (o == 0)))
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

static FracPointCloud get_points_global(FractureModifierData *emd, Object *ob, DerivedMesh* fracmesh, FractureLevel* fl)
{
	Scene* scene = emd->modifier.scene;
	FracPointCloud points;

	//global settings, for first fracture only, or global secondary and so on fracture, apply to entire fracmesh
	int totgroup = 0, t = 0;
	Object** go = MEM_mallocN(sizeof(Object*), "groupobjects");
	float thresh = (float)fl->percentage / 100.0f;
	BoundBox* bb;
	float* noisemap = NULL;

	float min[3], max[3];
	int i;

	points.points = MEM_mallocN(sizeof(FracPoint), "points");
	points.totpoints = 0;

	if (fl->point_source & (MOD_FRACTURE_EXTRA_PARTICLES | MOD_FRACTURE_EXTRA_VERTS ))
	{
		if (((fl->point_source & MOD_FRACTURE_OWN_PARTICLES) && (fl->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
			((fl->point_source & MOD_FRACTURE_OWN_VERTS) && (fl->point_source & MOD_FRACTURE_EXTRA_VERTS)) ||
			((fl->point_source & MOD_FRACTURE_GREASEPENCIL) && (fl->point_source & MOD_FRACTURE_EXTRA_PARTICLES)) ||
		    ((fl->point_source & MOD_FRACTURE_GREASEPENCIL) && (fl->point_source & MOD_FRACTURE_EXTRA_VERTS)))
		{
			go = MEM_reallocN(go, sizeof(Object*)*(totgroup+1));
			go[totgroup] = ob;
			totgroup++;
		}

		totgroup = getGroupObjects(fl->extra_group, &go, totgroup);
	}
	else
	{
		totgroup = 1;
		go[0] = ob;
	}

	if (fl->point_source & (MOD_FRACTURE_OWN_PARTICLES | MOD_FRACTURE_EXTRA_PARTICLES))
	{
		points_from_particles(go, totgroup, scene, &points, ob->obmat, thresh, emd, fl);
	}

	if (fl->point_source & (MOD_FRACTURE_OWN_VERTS | MOD_FRACTURE_EXTRA_VERTS))
	{
		points_from_verts(go, totgroup, &points, ob->obmat, thresh, emd, fl, fracmesh, ob);
	}

	if (fl->point_source & MOD_FRACTURE_GREASEPENCIL)
	{
		points_from_greasepencil(go, totgroup, &points, ob->obmat, thresh);
	}

	//greasecut, imagebased... backend is via projected curvemeshes

	//random "chips" (cut out via boolean, no voronoi)

	//secondary, tertiary fracture.... levels... (or 3 or 4 enough ?) each has same options, different parameters

	//cracks... in glass its ok to have visible faces, but solids ? very small debris must fall off, or just bevel the shards near the impact ?


	//local settings, apply per shard!!! Or globally too first.
	if (fl->point_source & MOD_FRACTURE_UNIFORM)
	{
		//this is pointsource "uniform", make seed settable
		INIT_MINMAX(min, max);
		BKE_get_shard_minmax(emd->frac_mesh, -1, min, max, fracmesh); //id 0 should be entire mesh
		//points.totpoints += emd->shard_count;

		//points.points = MEM_reallocN(points.points, sizeof(FracPoint) * points.totpoints);
		BLI_srandom(fl->point_seed);
		for (i = 0; i < fl->shard_count; ++i) {
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

	if (fl->noisemap == NULL)
	{
		noisemap = MEM_callocN(sizeof(float)*3 *points.totpoints, "noisemap");
	}

	for (t = 0; t < points.totpoints; t++) {
		float bbox_min[3], bbox_max[3];

		if (fl->noise > 0.0f)
		{
			if (fl->noisemap != NULL) {
				float noise[3];
				noise[0] = fl->noisemap[3*t];
				noise[1] = fl->noisemap[3*t+1];
				noise[2] = fl->noisemap[3*t+2];
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

				scalar = fl->noise * len_v3(size) / 2.0f;
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

	if (fl->noisemap == NULL)
	{
		fl->noisemap = noisemap;
		fl->noise_count = points.totpoints;
	}

	//MEM_freeN(bb);
	MEM_freeN(go);
	return points;
}

static void do_fracture(FractureModifierData *fracmd, ShardID id, Object* obj, DerivedMesh *dm, FractureLevel* fl)
{
	/* dummy point cloud, random */
	FracPointCloud points;

	points = get_points_global(fracmd, obj, dm, fl);

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

		BKE_fracture_shard_by_points(fracmd->frac_mesh, id, &points, fl->frac_algorithm, obj, dm);
		//MEM_freeN(points.points);
		if (fracmd->frac_mesh->shard_count > 0 && fracmd->cluster_count > 1)
			doClusters(fracmd, 1, obj);
		BKE_fracture_create_dm(fracmd, false);
	}
	MEM_freeN(points.points);

	//BKE_fracture_create_dm(fracmd, false);
}

ModifierTypeInfo modifierType_Fracture = {
        /* name */              "Fracture",
        /* structName */        "FractureModifierData",
        /* structSize */        sizeof(FractureModifierData),
        /* type */              eModifierTypeType_Constructive,
        /* flags */             eModifierTypeFlag_AcceptsMesh |
                                eModifierTypeFlag_Single |
                                eModifierTypeFlag_SupportsEditmode |
                                eModifierTypeFlag_SupportsMapping,
        /* copyData */          NULL,
        /* deformVerts */       NULL,
        /* deformMatrices */    NULL,
        /* deformVertsEM */     NULL,
        /* deformMatricesEM */  NULL,
        /* applyModifier */     applyModifier,
        /* applyModifierEM */   applyModifierEM,
        /* initData */          initData,
        /* requiredDataMask */  NULL,
        /* freeData */          freeData,
        /* isDisabled */        NULL,
        /* updateDepgraph */    NULL,
        /* dependsOnTime */     NULL,//dependsOnTime,
        /* dependsOnNormals */  NULL,
        /* foreachObjectLink */ NULL,
        /* foreachIDLink */     NULL,
};
