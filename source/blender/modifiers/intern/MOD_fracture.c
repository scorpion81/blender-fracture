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

static void do_fracture(FractureModifierData *fracmd, ShardID id, Object *obj, DerivedMesh* dm);

static void initData(ModifierData *md)
{
		FractureModifierData *fmd = (FractureModifierData*) md;
		fmd->frac_algorithm = MOD_FRACTURE_VORONOI;
		fmd->shard_count = 10;
		fmd->shard_id = 0;
		fmd->percentage = 100;
}

static void freeData(ModifierData *md)
{
	FractureModifierData *fmd = (FractureModifierData*) md;
	
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
	
	BKE_fracmesh_free(fmd->frac_mesh);
	MEM_freeN(fmd->frac_mesh);
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
	DerivedMesh *final_dm;

	if (fmd->frac_mesh && fmd->shard_id == 0)
	{	//recreate container, if shard 0 is to be fractured
		BKE_fracmesh_free(fmd->frac_mesh);
		MEM_freeN(fmd->frac_mesh);

		if (fmd->dm) {
			fmd->dm->needsFree = 1;
			fmd->dm->release(fmd->dm);
			fmd->dm = NULL;
		}

		fmd->frac_mesh = BKE_create_fracture_container(derivedData);
	}
	
	if (fmd->frac_mesh == NULL) {
		fmd->frac_mesh = BKE_create_fracture_container(derivedData);
	}
	
	do_fracture(fmd, fmd->shard_id, ob, derivedData);

	if (fmd->dm)
		final_dm = CDDM_copy(fmd->dm);
	else
		final_dm = derivedData;
	
	return final_dm;
}

static DerivedMesh *applyModifierEM(ModifierData *md, Object *ob,
                                    struct BMEditMesh *UNUSED(editData),
                                    DerivedMesh *derivedData,
                                    ModifierApplyFlag UNUSED(flag))
{
	FractureModifierData *fmd = (FractureModifierData*) md;
	DerivedMesh *final_dm;

	if (fmd->frac_mesh == NULL) {
		fmd->frac_mesh = BKE_create_fracture_container(derivedData);
	}
	
	//do_fracture(fmd, fmd->shard_id, ob);
	
	if (fmd->dm)
		final_dm = CDDM_copy(fmd->dm);
	else
		final_dm = derivedData;
	
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
	int totpoint = 0, totgroup = 0, t = 0;
	Object** go = MEM_mallocN(sizeof(Object*), "groupobjects");
	float thresh = (float)emd->percentage / 100.0f;
	BoundBox* bb;
	float* noisemap = NULL;

	points.points = MEM_mallocN(sizeof(FracPoint), "points");

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


	// do this afterwards...
	//scaling point cloud for splinters (see cell fracture....)
	//apply noise

	bb = BKE_object_boundbox_get(ob);

	if (emd->noisemap == NULL)
	{
		noisemap = MEM_callocN(sizeof(float)*3 *totpoint, "noisemap");
	}

	for (t = 0; t < totpoint; t++) {
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
	float min[3], max[3];
	/* dummy point cloud, random */
	FracPointCloud points;
	int i;

	points = get_points_global(fracmd, obj, dm);

	//local settings, apply per shard!!! Or globally too first.
	if (fracmd->point_source & MOD_FRACTURE_UNIFORM)
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
	}

	if (points.totpoints > 0)
	{
	//if (emd->point_source & eRadial)
	//{
		//maybe add radial too // rings, rays, bias 0...1, ring random, ray random
	//}

	//non-uniform pointcloud ... max cluster min clustersize, cluster count / "variation", min max distance

		BKE_fracture_shard_by_points(fracmd->frac_mesh, id, &points, fracmd->frac_algorithm, obj);
		MEM_freeN(points.points);
	}

	BKE_fracture_create_dm(fracmd, false);
}

ModifierTypeInfo modifierType_Fracture = {
        /* name */              "Fracture",
        /* structName */        "FractureModifierData",
        /* structSize */        sizeof(FractureModifierData),
        /* type */              eModifierTypeType_Constructive,
        /* flags */             eModifierTypeFlag_AcceptsMesh |
                                eModifierTypeFlag_Single, //|
                                //eModifierTypeFlag_SupportsEditmode,
        /* copyData */          NULL,
        /* deformVerts */       NULL,
        /* deformMatrices */    NULL,
        /* deformVertsEM */     NULL,
        /* deformMatricesEM */  NULL,
        /* applyModifier */     applyModifier,
        /* applyModifierEM */   NULL, //applyModifierEM,
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
