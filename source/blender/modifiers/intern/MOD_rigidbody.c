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

#include "BKE_cdderivedmesh.h"
#include "BKE_modifier.h"
#include "BKE_rigidbody.h"
#include "BKE_pointcache.h"
#include "BKE_scene.h"
#include "BKE_object.h"

#include "bmesh.h"

#include "DNA_object_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"
#include "DNA_listBase.h"

#include "../../rigidbody/RBI_api.h"

static void initData(ModifierData *md)
{
	RigidBodyModifierData *rmd = (RigidBodyModifierData *) md;
	rmd->visible_mesh = NULL;
	rmd->refresh = TRUE;
}

static void copyData(ModifierData *md, ModifierData *target)
{
	RigidBodyModifierData *rmd  = (RigidBodyModifierData *)md;
	RigidBodyModifierData *trmd = (RigidBodyModifierData *)target;
	
	//trmd->meshIslands = rmd->meshIslands;
	trmd->refresh = TRUE;
}

static void freeData(ModifierData *md)
{
	RigidBodyModifierData *rmd  = (RigidBodyModifierData *)md;
	MeshIsland *mi;

	while (rmd->meshIslands.first) {
		mi = rmd->meshIslands.first;
		BLI_remlink(&rmd->meshIslands, mi);
		BM_mesh_free(mi->physics_mesh);
		MEM_freeN(mi->rigidbody);
		MEM_freeN(mi->vertco);
		MEM_freeN(mi);
	}

	if (rmd->visible_mesh)
	{
		BM_mesh_free(rmd->visible_mesh);
		rmd->visible_mesh = NULL;
	}
}

int BM_calc_center_centroid(BMesh *bm, float cent[3])
{
	BMFace *f;
	BMIter iter;
	float face_area;
	float total_area = 0.0f;
	float face_cent[3];

	zero_v3(cent);

	/* calculate a weighted average of face centroids */
	BM_ITER_MESH (f, &iter, bm, BM_FACES_OF_MESH) {
		BM_face_calc_center_mean (f, face_cent);
		face_area = BM_face_calc_area(f);

		madd_v3_v3fl(cent, face_cent, face_area);
		total_area += face_area;
	}
	/* otherwise we get NAN for 0 polys */
	if (bm->totface) {
		mul_v3_fl(cent, 1.0f / total_area);
	}

	return (bm->totface != 0);
}

static void mesh_separate_tagged(RigidBodyModifierData* rmd, Object *ob)
{
	BMesh *bm_new;
	BMesh *bm_old = rmd->visible_mesh;
	MeshIsland *mi;
	int vertcount = 0;
	float centroid[3], dummyloc[3], rot[4], *startco;
	BMVert* v, **verts;
	BMIter iter;

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

	//need to delete old geometry ?
	//BMO_op_callf(bm_old, (BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE),
	//             "delete geom=%hvef context=%i", BM_ELEM_TAG, DEL_FACES);

	BM_calc_center_centroid(bm_new, centroid);
	verts = MEM_callocN(sizeof(BMVert*), "mesh_separate_tagged->verts");
	startco = MEM_callocN(sizeof(float), "mesh_separate_tagged->startco");

	//store tagged vertices from old bmesh, important for later manipulation
	//create rigidbody objects with island verts here
	//invert_m4_m4(imat, ob->obmat);

	BM_ITER_MESH (v, &iter, bm_new, BM_VERTS_OF_MESH) {
		//eliminate centroid in vertex coords ?
		sub_v3_v3(v->co, centroid);
	}

	BM_ITER_MESH (v, &iter, bm_old, BM_VERTS_OF_MESH) {

		if (BM_elem_flag_test(v, BM_ELEM_TAG)) {
			verts = MEM_reallocN(verts, sizeof(BMVert*) * (vertcount + 1));
			verts[vertcount] = v;
			//sub_v3_v3(v->co, centroid);

			startco = MEM_reallocN(startco, (vertcount+1) * 3 * sizeof(float));
			startco[3 * vertcount] = v->co[0];
			startco[3 * vertcount+1] = v->co[1];
			startco[3 * vertcount+2] = v->co[2];
			vertcount++;
		}
	}

	// add 1 MeshIsland
	mi = MEM_callocN(sizeof(MeshIsland), "meshIsland");
	BLI_addtail(&rmd->meshIslands, mi);

	mi->vertices = verts;
	mi->vertco = startco;
	mi->physics_mesh = bm_new;
	mi->vertex_count = vertcount;
	copy_v3_v3(mi->centroid, centroid);
	//TODO Unsure , maybe object loc is correct ?
//	mi->rigidbody = BKE_rigidbody_create_shard(rmd->modifier.scene, ob, mi, RBO_TYPE_ACTIVE);
	//copy_v3_v3(mi->rigidbody->pos, centroid);
//	mat4_to_loc_quat(dummyloc, rot, ob->obmat);
//	copy_v4_v4(mi->rigidbody->orn, rot);
//	copy_v3_v3(mi->rigidbody->pos, dummyloc);
	//BKE_rigidbody_validate_sim_shard(rmd->modifier.scene->rigidbody_world, mi, ob, true);

	/* deselect loose data - this used to get deleted,
	 * we could de-select edges and verts only, but this turns out to be less complicated
	 * since de-selecting all skips selection flushing logic */
	//BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT, FALSE);
	BM_mesh_normals_update(bm_new, FALSE);
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


void mesh_separate_loose(RigidBodyModifierData* rmd, Object* ob)
{
	int i;
	BMEdge *e;
	BMVert *v_seed;
	BMWalker walker;
	int max_iter = 0;
	int tot = 0;
	BMesh* bm_old = rmd->visible_mesh;
	GHash* hash = BLI_ghash_ptr_new("VertIslands");

	max_iter = bm_old->totvert;

	/* Clear all selected vertices */
	BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_TAG, FALSE);

	/* A "while (true)" loop should work here as each iteration should
	 * select and remove at least one vertex and when all vertices
	 * are selected the loop will break out. But guard against bad
	 * behavior by limiting iterations to the number of vertices in the
	 * original mesh.*/
	for (i = 0; i < max_iter; i++) {
		//int tot = 0;
		BMIter iter;
		BM_ITER_MESH (v_seed, &iter, bm_old, BM_VERTS_OF_MESH) {
			/* Get a seed vertex to start the walk */
			//v_seed = BM_iter_at_index(bm_old, BM_VERTS_OF_MESH, NULL, 0);
			if (!BM_elem_flag_test(v_seed, BM_ELEM_TAG) && !BLI_ghash_haskey(hash, v_seed)) {	//find untagged vertex, better iterate over all verts ?
				//delete old tags HERE, if found untagged vertex, should be on right island now, but... must not be existing yet
				BM_mesh_elem_hflag_disable_all(bm_old, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT| BM_ELEM_TAG, FALSE);
				break;
			}
		}

		/* No vertices available, can't do anything */
		if (v_seed == NULL){
			break;
		}

		/* Select the seed explicitly, in case it has no edges */
		if (!BM_elem_flag_test(v_seed, BM_ELEM_TAG) && !BLI_ghash_haskey(hash, v_seed)) {
			BLI_ghash_insert(hash, v_seed, v_seed);
			BM_elem_flag_enable(v_seed, BM_ELEM_TAG);
			tot++;
		}

		/* Walk from the single vertex, selecting everything connected
		 * to it */
		BMW_init(&walker, bm_old, BMW_SHELL,
					BMW_MASK_NOP, BMW_MASK_NOP, BMW_MASK_NOP,
					BMW_FLAG_NOP,
					BMW_NIL_LAY);

		e = BMW_begin(&walker, v_seed);
		for (; e; e = BMW_step(&walker)) {
			if (!BM_elem_flag_test(e->v1, BM_ELEM_TAG) && !BLI_ghash_haskey(hash, e->v1)){
				BLI_ghash_insert(hash, e->v1, e->v1);
				BM_elem_flag_enable(e->v1, BM_ELEM_TAG);
				tot++;
			}
			if (!BM_elem_flag_test(e->v2, BM_ELEM_TAG) && !BLI_ghash_haskey(hash, e->v2)) {
				BLI_ghash_insert(hash, e->v2, e->v2);
				BM_elem_flag_enable(e->v2, BM_ELEM_TAG);
				tot++;
			}
		}
		BMW_end(&walker);

		/* Flush the selection to get edge/face selections matching
		 * the vertex selection */
		bm_mesh_hflag_flush_vert(bm_old, BM_ELEM_TAG);


		/* Move selection into a separate object */
		mesh_separate_tagged(rmd, ob);

		if ((tot >= bm_old->totvert) && (BLI_countlist(&rmd->meshIslands) > 1)) {
			// Nothing more to select, work is done
			break;
		}
	}
	BLI_ghash_free(hash, NULL, NULL);
}

static DerivedMesh *applyModifier(ModifierData *md, Object *ob,
								  DerivedMesh *dm,
								  ModifierApplyFlag UNUSED(flag))
{

	RigidBodyModifierData *rmd = (RigidBodyModifierData *) md;
	//freeData(md); // reset on each run
	if (rmd->refresh)
	{
		freeData(md);
		rmd->visible_mesh = DM_to_bmesh(dm);
		mesh_separate_loose(rmd, ob);
		rmd->refresh = FALSE;
	}

	return CDDM_from_bmesh(rmd->visible_mesh, TRUE);
}

static int dependsOnTime(ModifierData *UNUSED(md))
{
	return 1;
}

ModifierTypeInfo modifierType_RigidBody = {
	/* name */              "RigidBody",
	/* structName */        "RigidBodyModifierData",
	/* structSize */        sizeof(RigidBodyModifierData),
	/* type */              eModifierTypeType_Constructive,
	/* flags */             eModifierTypeFlag_AcceptsMesh |
							eModifierTypeFlag_UsesPointCache |
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
	/* dependsOnNormals */	NULL,
	/* foreachObjectLink */ NULL,
	/* foreachIDLink */     NULL,
	/* foreachTexLink */    NULL
};
