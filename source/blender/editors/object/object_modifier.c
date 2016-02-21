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
 * The Original Code is Copyright (C) 2001-2002 by NaN Holding BV.
 * All rights reserved.
 *
 * Contributor(s): Blender Foundation, 2009
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/editors/object/object_modifier.c
 *  \ingroup edobj
 */


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "MEM_guardedalloc.h"

#include "DNA_anim_types.h"
#include "DNA_armature_types.h"
#include "DNA_curve_types.h"
#include "DNA_key_types.h"
#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_force.h"
#include "DNA_scene_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_fracture_types.h"
#include "DNA_group_types.h"

#include "BLI_bitmap.h"
#include "BLI_math.h"
#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_string_utf8.h"
#include "BLI_path_util.h"
#include "BLI_utildefines.h"
#include "BLI_kdtree.h"

#include "BKE_animsys.h"
#include "BKE_curve.h"
#include "BKE_context.h"
#include "BKE_depsgraph.h"
#include "BKE_displist.h"
#include "BKE_DerivedMesh.h"
#include "BKE_effect.h"
#include "BKE_fracture.h"
#include "BKE_global.h"
#include "BKE_key.h"
#include "BKE_lattice.h"
#include "BKE_main.h"
#include "BKE_mesh.h"
#include "BKE_mesh_mapping.h"
#include "BKE_modifier.h"
#include "BKE_multires.h"
#include "BKE_report.h"
#include "BKE_object.h"
#include "BKE_object_deform.h"
#include "BKE_ocean.h"
#include "BKE_paint.h"
#include "BKE_particle.h"
#include "BKE_pointcache.h"
#include "BKE_report.h"
#include "BKE_softbody.h"
#include "BKE_editmesh.h"
#include "BKE_scene.h"
#include "BKE_material.h"
#include "BKE_library.h"
#include "BKE_rigidbody.h"
#include "BKE_group.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "ED_armature.h"
#include "ED_object.h"
#include "ED_screen.h"
#include "ED_mesh.h"
#include "ED_physics.h"
#include "ED_keyframing.h"
#include "ED_anim_api.h" //clean keyframes

#include "WM_api.h"
#include "WM_types.h"

#include "object_intern.h"

#include "PIL_time.h"

static void modifier_skin_customdata_delete(struct Object *ob);

/******************************** API ****************************/

ModifierData *ED_object_modifier_add(ReportList *reports, Main *bmain, Scene *scene, Object *ob, const char *name, int type)
{
	ModifierData *md = NULL, *new_md = NULL;
	const ModifierTypeInfo *mti = modifierType_getInfo(type);
	
	/* only geometry objects should be able to get modifiers [#25291] */
	if (!ELEM(ob->type, OB_MESH, OB_CURVE, OB_SURF, OB_FONT, OB_LATTICE)) {
		BKE_reportf(reports, RPT_WARNING, "Modifiers cannot be added to object '%s'", ob->id.name + 2);
		return NULL;
	}
	
	if (mti->flags & eModifierTypeFlag_Single) {
		if (modifiers_findByType(ob, type)) {
			BKE_report(reports, RPT_WARNING, "Only one modifier of this type is allowed");
			return NULL;
		}
	}
	
	if (type == eModifierType_ParticleSystem) {
		/* don't need to worry about the new modifier's name, since that is set to the number
		 * of particle systems which shouldn't have too many duplicates 
		 */
		new_md = object_add_particle_system(scene, ob, name);
	}
	else {
		/* get new modifier data to add */
		new_md = modifier_new(type);
		
		if (mti->flags & eModifierTypeFlag_RequiresOriginalData) {
			md = ob->modifiers.first;
			
			while (md && modifierType_getInfo(md->type)->type == eModifierTypeType_OnlyDeform)
				md = md->next;
			
			BLI_insertlinkbefore(&ob->modifiers, md, new_md);
		}
		else
			BLI_addtail(&ob->modifiers, new_md);

		if (name) {
			BLI_strncpy_utf8(new_md->name, name, sizeof(new_md->name));
		}

		/* make sure modifier data has unique name */

		modifier_unique_name(&ob->modifiers, new_md);
		
		/* special cases */
		if (type == eModifierType_Softbody) {
			if (!ob->soft) {
				ob->soft = sbNew(scene);
				ob->softflag |= OB_SB_GOAL | OB_SB_EDGES;
			}
		}
		else if (type == eModifierType_Collision) {
			if (!ob->pd)
				ob->pd = object_add_collision_fields(0);
			
			ob->pd->deflect = 1;
		}
		else if (type == eModifierType_Surface) {
			/* pass */
		}
		else if (type == eModifierType_Multires) {
			/* set totlvl from existing MDISPS layer if object already had it */
			multiresModifier_set_levels_from_disps((MultiresModifierData *)new_md, ob);

			if (ob->mode & OB_MODE_SCULPT) {
				/* ensure that grid paint mask layer is created */
				BKE_sculpt_mask_layers_ensure(ob, (MultiresModifierData *)new_md);
			}
		}
		else if (type == eModifierType_Skin) {
			/* ensure skin-node customdata exists */
			BKE_mesh_ensure_skin_customdata(ob->data);
		}
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	DAG_relations_tag_update(bmain);

	return new_md;
}

/* Return true if the object has a modifier of type 'type' other than
 * the modifier pointed to be 'exclude', otherwise returns false. */
static bool object_has_modifier(const Object *ob, const ModifierData *exclude,
                                ModifierType type)
{
	ModifierData *md;

	for (md = ob->modifiers.first; md; md = md->next) {
		if ((md != exclude) && (md->type == type))
			return true;
	}

	return false;
}

/* If the object data of 'orig_ob' has other users, run 'callback' on
 * each of them.
 *
 * If include_orig is true, the callback will run on 'orig_ob' too.
 * 
 * If the callback ever returns true, iteration will stop and the
 * function value will be true. Otherwise the function returns false.
 */
bool ED_object_iter_other(Main *bmain, Object *orig_ob, const bool include_orig,
                          bool (*callback)(Object *ob, void *callback_data),
                          void *callback_data)
{
	ID *ob_data_id = orig_ob->data;
	int users = ob_data_id->us;

	if (ob_data_id->flag & LIB_FAKEUSER)
		users--;

	/* First check that the object's data has multiple users */
	if (users > 1) {
		Object *ob;
		int totfound = include_orig ? 0 : 1;

		for (ob = bmain->object.first; ob && totfound < users;
		     ob = ob->id.next)
		{
			if (((ob != orig_ob) || include_orig) &&
			    (ob->data == orig_ob->data))
			{
				if (callback(ob, callback_data))
					return true;

				totfound++;
			}
		}
	}
	else if (include_orig) {
		return callback(orig_ob, callback_data);
	}

	return false;
}

static bool object_has_modifier_cb(Object *ob, void *data)
{
	ModifierType type = *((ModifierType *)data);

	return object_has_modifier(ob, NULL, type);
}

/* Use with ED_object_iter_other(). Sets the total number of levels
 * for any multires modifiers on the object to the int pointed to by
 * callback_data. */
bool ED_object_multires_update_totlevels_cb(Object *ob, void *totlevel_v)
{
	ModifierData *md;
	int totlevel = *((char *)totlevel_v);

	for (md = ob->modifiers.first; md; md = md->next) {
		if (md->type == eModifierType_Multires) {
			multires_set_tot_level(ob, (MultiresModifierData *)md, totlevel);
			DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
		}
	}
	return false;
}

/* Return true if no modifier of type 'type' other than 'exclude' */
static bool object_modifier_safe_to_delete(Main *bmain, Object *ob,
                                           ModifierData *exclude,
                                           ModifierType type)
{
	return (!object_has_modifier(ob, exclude, type) &&
	        !ED_object_iter_other(bmain, ob, false,
	                              object_has_modifier_cb, &type));
}

static bool object_modifier_remove(Main *bmain, Object *ob, ModifierData *md,
                                   bool *r_sort_depsgraph)
{
	/* keep some data from modifier which is necessary for the afterwards cleanup */
	bool do_rigidbody_cleanup = (md->type == eModifierType_Fracture);
	Scene *scene = md->scene;

	/* It seems on rapid delete it is possible to
	 * get called twice on same modifier, so make
	 * sure it is in list. */
	if (BLI_findindex(&ob->modifiers, md) == -1) {
		return 0;
	}

	/* special cases */
	if (md->type == eModifierType_ParticleSystem) {
		ParticleSystemModifierData *psmd = (ParticleSystemModifierData *)md;

		BLI_remlink(&ob->particlesystem, psmd->psys);
		psys_free(ob, psmd->psys);
		psmd->psys = NULL;
	}
	else if (md->type == eModifierType_Softbody) {
		if (ob->soft) {
			sbFree(ob->soft);
			ob->soft = NULL;
			ob->softflag = 0;
		}
	}
	else if (md->type == eModifierType_Collision) {
		if (ob->pd)
			ob->pd->deflect = 0;

		*r_sort_depsgraph = true;
	}
	else if (md->type == eModifierType_Surface) {
		*r_sort_depsgraph = true;
	}
	else if (md->type == eModifierType_Multires) {
		/* Delete MDisps layer if not used by another multires modifier */
		if (object_modifier_safe_to_delete(bmain, ob, md, eModifierType_Multires))
			multires_customdata_delete(ob->data);
	}
	else if (md->type == eModifierType_Skin) {
		/* Delete MVertSkin layer if not used by another skin modifier */
		if (object_modifier_safe_to_delete(bmain, ob, md, eModifierType_Skin))
			modifier_skin_customdata_delete(ob);
	}

	if (ELEM(md->type, eModifierType_Softbody, eModifierType_Cloth) &&
	    BLI_listbase_is_empty(&ob->particlesystem))
	{
		ob->mode &= ~OB_MODE_PARTICLE_EDIT;
	}

	DAG_relations_tag_update(bmain);

	BLI_remlink(&ob->modifiers, md);
	modifier_free(md);
	BKE_object_free_derived_caches(ob);

	if (do_rigidbody_cleanup)
	{
		/* need to clean up modifier remainders inside the rigidbody world
		 * AFTER the modifier is gone...  but only from the operator ?*/
		if (scene->rigidbody_world)
		{
			BKE_rigidbody_rebuild_world(scene, -1);
		}
		BKE_scene_frame_set(scene, 1.0);
	}

	return 1;
}

bool ED_object_modifier_remove(ReportList *reports, Main *bmain, Object *ob, ModifierData *md)
{
	bool sort_depsgraph = false;
	bool ok;

	ok = object_modifier_remove(bmain, ob, md, &sort_depsgraph);

	if (!ok) {
		BKE_reportf(reports, RPT_ERROR, "Modifier '%s' not in object '%s'", md->name, ob->id.name);
		return 0;
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	DAG_relations_tag_update(bmain);

	return 1;
}

void ED_object_modifier_clear(Main *bmain, Object *ob)
{
	ModifierData *md = ob->modifiers.first;
	bool sort_depsgraph = false;

	if (!md)
		return;

	while (md) {
		ModifierData *next_md;

		next_md = md->next;

		object_modifier_remove(bmain, ob, md, &sort_depsgraph);

		md = next_md;
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	DAG_relations_tag_update(bmain);
}

int ED_object_modifier_move_up(ReportList *reports, Object *ob, ModifierData *md)
{
	if (md->prev) {
		const ModifierTypeInfo *mti = modifierType_getInfo(md->type);

		if (mti->type != eModifierTypeType_OnlyDeform) {
			const ModifierTypeInfo *nmti = modifierType_getInfo(md->prev->type);

			if (nmti->flags & eModifierTypeFlag_RequiresOriginalData) {
				BKE_report(reports, RPT_WARNING, "Cannot move above a modifier requiring original data");
				return 0;
			}
		}

		BLI_remlink(&ob->modifiers, md);
		BLI_insertlinkbefore(&ob->modifiers, md->prev, md);
	}

	return 1;
}

int ED_object_modifier_move_down(ReportList *reports, Object *ob, ModifierData *md)
{
	if (md->next) {
		const ModifierTypeInfo *mti = modifierType_getInfo(md->type);

		if (mti->flags & eModifierTypeFlag_RequiresOriginalData) {
			const ModifierTypeInfo *nmti = modifierType_getInfo(md->next->type);

			/*make an exception here for fluidsim, in case: it is beyond a fracture modifier and this may have some other mods above it*/
			if ((nmti->type != eModifierTypeType_OnlyDeform) && (md->type != eModifierType_Fluidsim)) {
				BKE_report(reports, RPT_WARNING, "Cannot move beyond a non-deforming modifier");
				return 0;
			}
		}

		BLI_remlink(&ob->modifiers, md);
		BLI_insertlinkafter(&ob->modifiers, md->next, md);
	}

	return 1;
}

int ED_object_modifier_convert(ReportList *UNUSED(reports), Main *bmain, Scene *scene, Object *ob, ModifierData *md)
{
	Object *obn;
	ParticleSystem *psys;
	ParticleCacheKey *key, **cache;
	ParticleSettings *part;
	Mesh *me;
	MVert *mvert;
	MEdge *medge;
	int a, k, kmax;
	int totvert = 0, totedge = 0, cvert = 0;
	int totpart = 0, totchild = 0;

	if (md->type != eModifierType_ParticleSystem) return 0;
	if (ob && ob->mode & OB_MODE_PARTICLE_EDIT) return 0;

	psys = ((ParticleSystemModifierData *)md)->psys;
	part = psys->part;

	if (part->ren_as != PART_DRAW_PATH || psys->pathcache == NULL)
		return 0;

	totpart = psys->totcached;
	totchild = psys->totchildcache;

	if (totchild && (part->draw & PART_DRAW_PARENT) == 0)
		totpart = 0;

	/* count */
	cache = psys->pathcache;
	for (a = 0; a < totpart; a++) {
		key = cache[a];

		if (key->segments > 0) {
			totvert += key->segments + 1;
			totedge += key->segments;
		}
	}

	cache = psys->childcache;
	for (a = 0; a < totchild; a++) {
		key = cache[a];

		if (key->segments > 0) {
			totvert += key->segments + 1;
			totedge += key->segments;
		}
	}

	if (totvert == 0) return 0;

	/* add new mesh */
	obn = BKE_object_add(bmain, scene, OB_MESH, NULL);
	me = obn->data;
	
	me->totvert = totvert;
	me->totedge = totedge;
	
	me->mvert = CustomData_add_layer(&me->vdata, CD_MVERT, CD_CALLOC, NULL, totvert);
	me->medge = CustomData_add_layer(&me->edata, CD_MEDGE, CD_CALLOC, NULL, totedge);
	me->mface = CustomData_add_layer(&me->fdata, CD_MFACE, CD_CALLOC, NULL, 0);
	
	mvert = me->mvert;
	medge = me->medge;

	/* copy coordinates */
	cache = psys->pathcache;
	for (a = 0; a < totpart; a++) {
		key = cache[a];
		kmax = key->segments;
		for (k = 0; k <= kmax; k++, key++, cvert++, mvert++) {
			copy_v3_v3(mvert->co, key->co);
			if (k) {
				medge->v1 = cvert - 1;
				medge->v2 = cvert;
				medge->flag = ME_EDGEDRAW | ME_EDGERENDER | ME_LOOSEEDGE;
				medge++;
			}
			else {
				/* cheap trick to select the roots */
				mvert->flag |= SELECT;
			}
		}
	}

	cache = psys->childcache;
	for (a = 0; a < totchild; a++) {
		key = cache[a];
		kmax = key->segments;
		for (k = 0; k <= kmax; k++, key++, cvert++, mvert++) {
			copy_v3_v3(mvert->co, key->co);
			if (k) {
				medge->v1 = cvert - 1;
				medge->v2 = cvert;
				medge->flag = ME_EDGEDRAW | ME_EDGERENDER | ME_LOOSEEDGE;
				medge++;
			}
			else {
				/* cheap trick to select the roots */
				mvert->flag |= SELECT;
			}
		}
	}

	DAG_relations_tag_update(bmain);

	return 1;
}

static int modifier_apply_shape(ReportList *reports, Scene *scene, Object *ob, ModifierData *md)
{
	const ModifierTypeInfo *mti = modifierType_getInfo(md->type);

	md->scene = scene;

	if (mti->isDisabled && mti->isDisabled(md, 0)) {
		BKE_report(reports, RPT_ERROR, "Modifier is disabled, skipping apply");
		return 0;
	}

	/*
	 * It should be ridiculously easy to extract the original verts that we want
	 * and form the shape data.  We can probably use the CD KEYINDEX layer (or
	 * whatever I ended up calling it, too tired to check now), though this would
	 * by necessity have to make some potentially ugly assumptions about the order
	 * of the mesh data :-/  you can probably assume in 99% of cases that the first
	 * element of a given index is the original, and any subsequent duplicates are
	 * copies/interpolates, but that's an assumption that would need to be tested
	 * and then predominantly stated in comments in a half dozen headers.
	 */

	if (ob->type == OB_MESH) {
		DerivedMesh *dm;
		Mesh *me = ob->data;
		Key *key = me->key;
		KeyBlock *kb;
		
		if (!modifier_isSameTopology(md) || mti->type == eModifierTypeType_NonGeometrical) {
			BKE_report(reports, RPT_ERROR, "Only deforming modifiers can be applied to shapes");
			return 0;
		}
		
		dm = mesh_create_derived_for_modifier(scene, ob, md, 0);
		if (!dm) {
			BKE_report(reports, RPT_ERROR, "Modifier is disabled or returned error, skipping apply");
			return 0;
		}
		
		if (key == NULL) {
			key = me->key = BKE_key_add((ID *)me);
			key->type = KEY_RELATIVE;
			/* if that was the first key block added, then it was the basis.
			 * Initialize it with the mesh, and add another for the modifier */
			kb = BKE_keyblock_add(key, NULL);
			BKE_keyblock_convert_from_mesh(me, kb);
		}

		kb = BKE_keyblock_add(key, md->name);
		DM_to_meshkey(dm, me, kb);
		
		dm->release(dm);
	}
	else {
		BKE_report(reports, RPT_ERROR, "Cannot apply modifier for this object type");
		return 0;
	}
	return 1;
}

static int modifier_apply_obdata(ReportList *reports, Scene *scene, Object *ob, ModifierData *md)
{
	const ModifierTypeInfo *mti = modifierType_getInfo(md->type);

	md->scene = scene;

	if (mti->isDisabled && mti->isDisabled(md, 0)) {
		BKE_report(reports, RPT_ERROR, "Modifier is disabled, skipping apply");
		return 0;
	}

	if (ob->type == OB_MESH) {
		DerivedMesh *dm;
		Mesh *me = ob->data;
		MultiresModifierData *mmd = find_multires_modifier_before(scene, md);

		if (me->key && mti->type != eModifierTypeType_NonGeometrical) {
			BKE_report(reports, RPT_ERROR, "Modifier cannot be applied to a mesh with shape keys");
			return 0;
		}

		/* Multires: ensure that recent sculpting is applied */
		if (md->type == eModifierType_Multires)
			multires_force_update(ob);

		if (mmd && mmd->totlvl && mti->type == eModifierTypeType_OnlyDeform) {
			if (!multiresModifier_reshapeFromDeformMod(scene, mmd, ob, md)) {
				BKE_report(reports, RPT_ERROR, "Multires modifier returned error, skipping apply");
				return 0;
			}
		}
		else {
			dm = mesh_create_derived_for_modifier(scene, ob, md, 1);
			if (!dm) {
				BKE_report(reports, RPT_ERROR, "Modifier returned error, skipping apply");
				return 0;
			}

			DM_to_mesh(dm, me, ob, CD_MASK_MESH, true);

			if (md->type == eModifierType_Multires)
				multires_customdata_delete(me);
		}
	}
	else if (ELEM(ob->type, OB_CURVE, OB_SURF)) {
		Curve *cu;
		int numVerts;
		float (*vertexCos)[3];

		if (ELEM(mti->type, eModifierTypeType_Constructive, eModifierTypeType_Nonconstructive)) {
			BKE_report(reports, RPT_ERROR, "Cannot apply constructive modifiers on curve");
			return 0;
		}

		cu = ob->data;
		BKE_report(reports, RPT_INFO, "Applied modifier only changed CV points, not tessellated/bevel vertices");

		vertexCos = BKE_curve_nurbs_vertexCos_get(&cu->nurb, &numVerts);
		mti->deformVerts(md, ob, NULL, vertexCos, numVerts, 0);
		BK_curve_nurbs_vertexCos_apply(&cu->nurb, vertexCos);

		MEM_freeN(vertexCos);

		DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	}
	else {
		BKE_report(reports, RPT_ERROR, "Cannot apply modifier for this object type");
		return 0;
	}

	/* lattice modifier can be applied to particle system too */
	if (ob->particlesystem.first) {

		ParticleSystem *psys = ob->particlesystem.first;

		for (; psys; psys = psys->next) {
			
			if (psys->part->type != PART_HAIR)
				continue;

			psys_apply_hair_lattice(scene, ob, psys);
		}
	}

	return 1;
}

int ED_object_modifier_apply(ReportList *reports, Scene *scene, Object *ob, ModifierData *md, int mode)
{
	int prev_mode;

	if (scene->obedit) {
		BKE_report(reports, RPT_ERROR, "Modifiers cannot be applied in edit mode");
		return 0;
	}
	else if (((ID *) ob->data)->us > 1) {
		BKE_report(reports, RPT_ERROR, "Modifiers cannot be applied to multi-user data");
		return 0;
	}
	else if ((ob->mode & OB_MODE_SCULPT) &&
	         (find_multires_modifier_before(scene, md)) &&
	         (modifier_isSameTopology(md) == false))
	{
		BKE_report(reports, RPT_ERROR, "Constructive modifier cannot be applied to multi-res data in sculpt mode");
		return 0;
	}

	if (md != ob->modifiers.first)
		BKE_report(reports, RPT_INFO, "Applied modifier was not first, result may not be as expected");

	/* allow apply of a not-realtime modifier, by first re-enabling realtime. */
	prev_mode = md->mode;
	md->mode |= eModifierMode_Realtime;

	if (mode == MODIFIER_APPLY_SHAPE) {
		if (!modifier_apply_shape(reports, scene, ob, md)) {
			md->mode = prev_mode;
			return 0;
		}
	}
	else {
		if (!modifier_apply_obdata(reports, scene, ob, md)) {
			md->mode = prev_mode;
			return 0;
		}
	}

	BLI_remlink(&ob->modifiers, md);
	modifier_free(md);

	BKE_object_free_derived_caches(ob);

	return 1;
}

int ED_object_modifier_copy(ReportList *UNUSED(reports), Object *ob, ModifierData *md)
{
	ModifierData *nmd;
	
	nmd = modifier_new(md->type);
	modifier_copyData(md, nmd);
	BLI_insertlinkafter(&ob->modifiers, md, nmd);
	modifier_unique_name(&ob->modifiers, nmd);

	return 1;
}

/************************ add modifier operator *********************/

static int modifier_add_exec(bContext *C, wmOperator *op)
{
	Main *bmain = CTX_data_main(C);
	Scene *scene = CTX_data_scene(C);
	Object *ob = ED_object_active_context(C);
	int type = RNA_enum_get(op->ptr, "type");

	if (!ED_object_modifier_add(op->reports, bmain, scene, ob, NULL, type))
		return OPERATOR_CANCELLED;

	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static EnumPropertyItem *modifier_add_itemf(bContext *C, PointerRNA *UNUSED(ptr), PropertyRNA *UNUSED(prop), bool *r_free)
{	
	Object *ob = ED_object_active_context(C);
	EnumPropertyItem *item = NULL, *md_item, *group_item = NULL;
	const ModifierTypeInfo *mti;
	int totitem = 0, a;
	
	if (!ob)
		return rna_enum_object_modifier_type_items;

	for (a = 0; rna_enum_object_modifier_type_items[a].identifier; a++) {
		md_item = &rna_enum_object_modifier_type_items[a];

		if (md_item->identifier[0]) {
			mti = modifierType_getInfo(md_item->value);

			if (mti->flags & eModifierTypeFlag_NoUserAdd)
				continue;

			if (!BKE_object_support_modifier_type_check(ob, md_item->value))
				continue;
		}
		else {
			group_item = md_item;
			md_item = NULL;

			continue;
		}

		if (group_item) {
			RNA_enum_item_add(&item, &totitem, group_item);
			group_item = NULL;
		}

		RNA_enum_item_add(&item, &totitem, md_item);
	}

	RNA_enum_item_end(&item, &totitem);
	*r_free = true;

	return item;
}

void OBJECT_OT_modifier_add(wmOperatorType *ot)
{
	PropertyRNA *prop;

	/* identifiers */
	ot->name = "Add Modifier";
	ot->description = "Add a modifier to the active object";
	ot->idname = "OBJECT_OT_modifier_add";
	
	/* api callbacks */
	ot->invoke = WM_menu_invoke;
	ot->exec = modifier_add_exec;
	ot->poll = ED_operator_object_active_editable;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
	
	/* properties */
	prop = RNA_def_enum(ot->srna, "type", rna_enum_object_modifier_type_items, eModifierType_Subsurf, "Type", "");
	RNA_def_enum_funcs(prop, modifier_add_itemf);
	ot->prop = prop;
}

/************************ generic functions for operators using mod names and data context *********************/

int edit_modifier_poll_generic(bContext *C, StructRNA *rna_type, int obtype_flag)
{
	PointerRNA ptr = CTX_data_pointer_get_type(C, "modifier", rna_type);
	Object *ob = (ptr.id.data) ? ptr.id.data : ED_object_active_context(C);
	
	if (!ob || ob->id.lib) return 0;
	if (obtype_flag && ((1 << ob->type) & obtype_flag) == 0) return 0;
	if (ptr.id.data && ((ID *)ptr.id.data)->lib) return 0;
	
	return 1;
}

int edit_modifier_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_Modifier, 0);
}

void edit_modifier_properties(wmOperatorType *ot)
{
	RNA_def_string(ot->srna, "modifier", NULL, MAX_NAME, "Modifier", "Name of the modifier to edit");
}

int edit_modifier_invoke_properties(bContext *C, wmOperator *op)
{
	ModifierData *md;
	
	if (RNA_struct_property_is_set(op->ptr, "modifier")) {
		return true;
	}
	else {
		PointerRNA ptr = CTX_data_pointer_get_type(C, "modifier", &RNA_Modifier);
		if (ptr.data) {
			md = ptr.data;
			RNA_string_set(op->ptr, "modifier", md->name);
			return true;
		}
	}

	return false;
}

ModifierData *edit_modifier_property_get(wmOperator *op, Object *ob, int type)
{
	char modifier_name[MAX_NAME];
	ModifierData *md;
	RNA_string_get(op->ptr, "modifier", modifier_name);
	
	md = modifiers_findByName(ob, modifier_name);
	
	if (md && type != 0 && md->type != type)
		md = NULL;

	return md;
}

/************************ remove modifier operator *********************/

static int modifier_remove_exec(bContext *C, wmOperator *op)
{
	Main *bmain = CTX_data_main(C);
	Scene *scene = CTX_data_scene(C);
	Object *ob = ED_object_active_context(C);
	ModifierData *md = edit_modifier_property_get(op, ob, 0);
	int mode_orig = ob->mode;

	//if we have a running fracture job, dont remove the modifier
	if (md && md->type == eModifierType_Fracture)
	{
		FractureModifierData* fmd = (FractureModifierData*)md;
		if (fmd->execute_threaded && fmd->frac_mesh && fmd->frac_mesh->running == 1)
		{
			return OPERATOR_CANCELLED;
		}
	}
	
	if (!md || !ED_object_modifier_remove(op->reports, bmain, ob, md))
		return OPERATOR_CANCELLED;

	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

	/* if cloth/softbody was removed, particle mode could be cleared */
	if (mode_orig & OB_MODE_PARTICLE_EDIT)
		if ((ob->mode & OB_MODE_PARTICLE_EDIT) == 0)
			if (scene->basact && scene->basact->object == ob)
				WM_event_add_notifier(C, NC_SCENE | ND_MODE | NS_MODE_OBJECT, NULL);
	
	return OPERATOR_FINISHED;
}

static int modifier_remove_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return modifier_remove_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_modifier_remove(wmOperatorType *ot)
{
	ot->name = "Remove Modifier";
	ot->description = "Remove a modifier from the active object";
	ot->idname = "OBJECT_OT_modifier_remove";

	ot->invoke = modifier_remove_invoke;
	ot->exec = modifier_remove_exec;
	ot->poll = edit_modifier_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/************************ move up modifier operator *********************/

static int modifier_move_up_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	ModifierData *md = edit_modifier_property_get(op, ob, 0);

	if (!md || !ED_object_modifier_move_up(op->reports, ob, md))
		return OPERATOR_CANCELLED;

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int modifier_move_up_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return modifier_move_up_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_modifier_move_up(wmOperatorType *ot)
{
	ot->name = "Move Up Modifier";
	ot->description = "Move modifier up in the stack";
	ot->idname = "OBJECT_OT_modifier_move_up";

	ot->invoke = modifier_move_up_invoke;
	ot->exec = modifier_move_up_exec;
	ot->poll = edit_modifier_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/************************ move down modifier operator *********************/

static int modifier_move_down_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	ModifierData *md = edit_modifier_property_get(op, ob, 0);

	if (!md || !ED_object_modifier_move_down(op->reports, ob, md))
		return OPERATOR_CANCELLED;

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int modifier_move_down_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return modifier_move_down_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_modifier_move_down(wmOperatorType *ot)
{
	ot->name = "Move Down Modifier";
	ot->description = "Move modifier down in the stack";
	ot->idname = "OBJECT_OT_modifier_move_down";

	ot->invoke = modifier_move_down_invoke;
	ot->exec = modifier_move_down_exec;
	ot->poll = edit_modifier_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/************************ apply modifier operator *********************/

static int modifier_apply_exec(bContext *C, wmOperator *op)
{
	Scene *scene = CTX_data_scene(C);
	Object *ob = ED_object_active_context(C);
	ModifierData *md = edit_modifier_property_get(op, ob, 0);
	int apply_as = RNA_enum_get(op->ptr, "apply_as");

	if (!md || !ED_object_modifier_apply(op->reports, scene, ob, md, apply_as)) {
		return OPERATOR_CANCELLED;
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int modifier_apply_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return modifier_apply_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

static EnumPropertyItem modifier_apply_as_items[] = {
	{MODIFIER_APPLY_DATA, "DATA", 0, "Object Data", "Apply modifier to the object's data"},
	{MODIFIER_APPLY_SHAPE, "SHAPE", 0, "New Shape", "Apply deform-only modifier to a new shape on this object"},
	{0, NULL, 0, NULL, NULL}
};

void OBJECT_OT_modifier_apply(wmOperatorType *ot)
{
	ot->name = "Apply Modifier";
	ot->description = "Apply modifier and remove from the stack";
	ot->idname = "OBJECT_OT_modifier_apply";

	ot->invoke = modifier_apply_invoke;
	ot->exec = modifier_apply_exec;
	ot->poll = edit_modifier_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	
	RNA_def_enum(ot->srna, "apply_as", modifier_apply_as_items, MODIFIER_APPLY_DATA, "Apply as", "How to apply the modifier to the geometry");
	edit_modifier_properties(ot);
}

/************************ convert modifier operator *********************/

static int modifier_convert_exec(bContext *C, wmOperator *op)
{
	Main *bmain = CTX_data_main(C);
	Scene *scene = CTX_data_scene(C);
	Object *ob = ED_object_active_context(C);
	ModifierData *md = edit_modifier_property_get(op, ob, 0);
	
	if (!md || !ED_object_modifier_convert(op->reports, bmain, scene, ob, md))
		return OPERATOR_CANCELLED;

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int modifier_convert_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return modifier_convert_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_modifier_convert(wmOperatorType *ot)
{
	ot->name = "Convert Modifier";
	ot->description = "Convert particles to a mesh object";
	ot->idname = "OBJECT_OT_modifier_convert";

	ot->invoke = modifier_convert_invoke;
	ot->exec = modifier_convert_exec;
	ot->poll = edit_modifier_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/************************ copy modifier operator *********************/

static int modifier_copy_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	ModifierData *md = edit_modifier_property_get(op, ob, 0);

	if (!md || !ED_object_modifier_copy(op->reports, ob, md))
		return OPERATOR_CANCELLED;

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int modifier_copy_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return modifier_copy_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_modifier_copy(wmOperatorType *ot)
{
	ot->name = "Copy Modifier";
	ot->description = "Duplicate modifier at the same position in the stack";
	ot->idname = "OBJECT_OT_modifier_copy";

	ot->invoke = modifier_copy_invoke;
	ot->exec = modifier_copy_exec;
	ot->poll = edit_modifier_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/************* multires delete higher levels operator ****************/

static int multires_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_MultiresModifier, (1 << OB_MESH));
}

static int multires_higher_levels_delete_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	MultiresModifierData *mmd = (MultiresModifierData *)edit_modifier_property_get(op, ob, eModifierType_Multires);
	
	if (!mmd)
		return OPERATOR_CANCELLED;
	
	multiresModifier_del_levels(mmd, ob, 1);

	ED_object_iter_other(CTX_data_main(C), ob, true,
	                     ED_object_multires_update_totlevels_cb,
	                     &mmd->totlvl);
	
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int multires_higher_levels_delete_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return multires_higher_levels_delete_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_multires_higher_levels_delete(wmOperatorType *ot)
{
	ot->name = "Delete Higher Levels";
	ot->description = "Deletes the higher resolution mesh, potential loss of detail";
	ot->idname = "OBJECT_OT_multires_higher_levels_delete";

	ot->poll = multires_poll;
	ot->invoke = multires_higher_levels_delete_invoke;
	ot->exec = multires_higher_levels_delete_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/****************** multires subdivide operator *********************/

static int multires_subdivide_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	MultiresModifierData *mmd = (MultiresModifierData *)edit_modifier_property_get(op, ob, eModifierType_Multires);
	
	if (!mmd)
		return OPERATOR_CANCELLED;
	
	multiresModifier_subdivide(mmd, ob, 0, mmd->simple);

	ED_object_iter_other(CTX_data_main(C), ob, true,
	                     ED_object_multires_update_totlevels_cb,
	                     &mmd->totlvl);

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

	if (ob->mode & OB_MODE_SCULPT) {
		/* ensure that grid paint mask layer is created */
		BKE_sculpt_mask_layers_ensure(ob, mmd);
	}
	
	return OPERATOR_FINISHED;
}

static int multires_subdivide_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return multires_subdivide_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_multires_subdivide(wmOperatorType *ot)
{
	ot->name = "Multires Subdivide";
	ot->description = "Add a new level of subdivision";
	ot->idname = "OBJECT_OT_multires_subdivide";

	ot->poll = multires_poll;
	ot->invoke = multires_subdivide_invoke;
	ot->exec = multires_subdivide_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/****************** multires reshape operator *********************/

static int multires_reshape_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C), *secondob = NULL;
	Scene *scene = CTX_data_scene(C);
	MultiresModifierData *mmd = (MultiresModifierData *)edit_modifier_property_get(op, ob, eModifierType_Multires);

	if (!mmd)
		return OPERATOR_CANCELLED;

	if (mmd->lvl == 0) {
		BKE_report(op->reports, RPT_ERROR, "Reshape can work only with higher levels of subdivisions");
		return OPERATOR_CANCELLED;
	}

	CTX_DATA_BEGIN(C, Object *, selob, selected_editable_objects)
	{
		if (selob->type == OB_MESH && selob != ob) {
			secondob = selob;
			break;
		}
	}
	CTX_DATA_END;

	if (!secondob) {
		BKE_report(op->reports, RPT_ERROR, "Second selected mesh object required to copy shape from");
		return OPERATOR_CANCELLED;
	}

	if (!multiresModifier_reshape(scene, mmd, ob, secondob)) {
		BKE_report(op->reports, RPT_ERROR, "Objects do not have the same number of vertices");
		return OPERATOR_CANCELLED;
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

	return OPERATOR_FINISHED;
}

static int multires_reshape_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return multires_reshape_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_multires_reshape(wmOperatorType *ot)
{
	ot->name = "Multires Reshape";
	ot->description = "Copy vertex coordinates from other object";
	ot->idname = "OBJECT_OT_multires_reshape";

	ot->poll = multires_poll;
	ot->invoke = multires_reshape_invoke;
	ot->exec = multires_reshape_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}


		
/****************** multires save external operator *********************/

static int multires_external_save_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	Mesh *me = (ob) ? ob->data : op->customdata;
	char path[FILE_MAX];
	const bool relative = RNA_boolean_get(op->ptr, "relative_path");

	if (!me)
		return OPERATOR_CANCELLED;

	if (CustomData_external_test(&me->ldata, CD_MDISPS))
		return OPERATOR_CANCELLED;
	
	RNA_string_get(op->ptr, "filepath", path);

	if (relative)
		BLI_path_rel(path, G.main->name);

	CustomData_external_add(&me->ldata, &me->id, CD_MDISPS, me->totloop, path);
	CustomData_external_write(&me->ldata, &me->id, CD_MASK_MESH, me->totloop, 0);
	
	return OPERATOR_FINISHED;
}

static int multires_external_save_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	Object *ob = ED_object_active_context(C);
	MultiresModifierData *mmd;
	Mesh *me = ob->data;
	char path[FILE_MAX];

	if (!edit_modifier_invoke_properties(C, op))
		return OPERATOR_CANCELLED;
	
	mmd = (MultiresModifierData *)edit_modifier_property_get(op, ob, eModifierType_Multires);
	
	if (!mmd)
		return OPERATOR_CANCELLED;
	
	if (CustomData_external_test(&me->ldata, CD_MDISPS))
		return OPERATOR_CANCELLED;

	if (RNA_struct_property_is_set(op->ptr, "filepath"))
		return multires_external_save_exec(C, op);
	
	op->customdata = me;

	BLI_snprintf(path, sizeof(path), "//%s.btx", me->id.name + 2);
	RNA_string_set(op->ptr, "filepath", path);
	
	WM_event_add_fileselect(C, op);

	return OPERATOR_RUNNING_MODAL;
}

void OBJECT_OT_multires_external_save(wmOperatorType *ot)
{
	ot->name = "Multires Save External";
	ot->description = "Save displacements to an external file";
	ot->idname = "OBJECT_OT_multires_external_save";

	/* XXX modifier no longer in context after file browser .. ot->poll = multires_poll; */
	ot->exec = multires_external_save_exec;
	ot->invoke = multires_external_save_invoke;
	ot->poll = multires_poll;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;

	WM_operator_properties_filesel(
	        ot, FILE_TYPE_FOLDER | FILE_TYPE_BTX, FILE_SPECIAL, FILE_SAVE,
	        WM_FILESEL_FILEPATH | WM_FILESEL_RELPATH, FILE_DEFAULTDISPLAY, FILE_SORT_ALPHA);
	edit_modifier_properties(ot);
}

/****************** multires pack operator *********************/

static int multires_external_pack_exec(bContext *C, wmOperator *UNUSED(op))
{
	Object *ob = ED_object_active_context(C);
	Mesh *me = ob->data;

	if (!CustomData_external_test(&me->ldata, CD_MDISPS))
		return OPERATOR_CANCELLED;

	/* XXX don't remove.. */
	CustomData_external_remove(&me->ldata, &me->id, CD_MDISPS, me->totloop);
	
	return OPERATOR_FINISHED;
}

void OBJECT_OT_multires_external_pack(wmOperatorType *ot)
{
	ot->name = "Multires Pack External";
	ot->description = "Pack displacements from an external file";
	ot->idname = "OBJECT_OT_multires_external_pack";

	ot->poll = multires_poll;
	ot->exec = multires_external_pack_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/********************* multires apply base ***********************/
static int multires_base_apply_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	MultiresModifierData *mmd = (MultiresModifierData *)edit_modifier_property_get(op, ob, eModifierType_Multires);
	
	if (!mmd)
		return OPERATOR_CANCELLED;
	
	multiresModifier_base_apply(mmd, ob);

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int multires_base_apply_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return multires_base_apply_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}


void OBJECT_OT_multires_base_apply(wmOperatorType *ot)
{
	ot->name = "Multires Apply Base";
	ot->description = "Modify the base mesh to conform to the displaced mesh";
	ot->idname = "OBJECT_OT_multires_base_apply";

	ot->poll = multires_poll;
	ot->invoke = multires_base_apply_invoke;
	ot->exec = multires_base_apply_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}


/************************** skin modifier ***********************/

static void modifier_skin_customdata_delete(Object *ob)
{
	Mesh *me = ob->data;
	BMEditMesh *em = me->edit_btmesh;
	
	if (em)
		BM_data_layer_free(em->bm, &em->bm->vdata, CD_MVERT_SKIN);
	else
		CustomData_free_layer_active(&me->vdata, CD_MVERT_SKIN, me->totvert);
}

static int skin_poll(bContext *C)
{
	return (!CTX_data_edit_object(C) &&
	        edit_modifier_poll_generic(C, &RNA_SkinModifier, (1 << OB_MESH)));
}

static int skin_edit_poll(bContext *C)
{
	return (CTX_data_edit_object(C) &&
	        edit_modifier_poll_generic(C, &RNA_SkinModifier, (1 << OB_MESH)));
}

static void skin_root_clear(BMesh *bm, BMVert *bm_vert, GSet *visited)
{
	BMEdge *bm_edge;
	BMIter bm_iter;
	
	BM_ITER_ELEM (bm_edge, &bm_iter, bm_vert, BM_EDGES_OF_VERT) {
		BMVert *v2 = BM_edge_other_vert(bm_edge, bm_vert);

		if (!BLI_gset_haskey(visited, v2)) {
			MVertSkin *vs = CustomData_bmesh_get(&bm->vdata,
			                                     v2->head.data,
			                                     CD_MVERT_SKIN);

			/* clear vertex root flag and add to visited set */
			vs->flag &= ~MVERT_SKIN_ROOT;
			BLI_gset_insert(visited, v2);

			skin_root_clear(bm, v2, visited);
		}
	}
}

static int skin_root_mark_exec(bContext *C, wmOperator *UNUSED(op))
{
	Object *ob = CTX_data_edit_object(C);
	BMEditMesh *em = BKE_editmesh_from_object(ob);
	BMesh *bm = em->bm;
	BMVert *bm_vert;
	BMIter bm_iter;
	GSet *visited;

	visited = BLI_gset_ptr_new(__func__);

	BKE_mesh_ensure_skin_customdata(ob->data);

	BM_ITER_MESH (bm_vert, &bm_iter, bm, BM_VERTS_OF_MESH) {
		if (!BLI_gset_haskey(visited, bm_vert) &&
		    BM_elem_flag_test(bm_vert, BM_ELEM_SELECT))
		{
			MVertSkin *vs = CustomData_bmesh_get(&bm->vdata,
			                                     bm_vert->head.data,
			                                     CD_MVERT_SKIN);

			/* mark vertex as root and add to visited set */
			vs->flag |= MVERT_SKIN_ROOT;
			BLI_gset_insert(visited, bm_vert);

			/* clear root flag from all connected vertices (recursively) */
			skin_root_clear(bm, bm_vert, visited);
		}
	}

	BLI_gset_free(visited, NULL);

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

void OBJECT_OT_skin_root_mark(wmOperatorType *ot)
{
	ot->name = "Skin Root Mark";
	ot->description = "Mark selected vertices as roots";
	ot->idname = "OBJECT_OT_skin_root_mark";

	ot->poll = skin_edit_poll;
	ot->exec = skin_root_mark_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

typedef enum {
	SKIN_LOOSE_MARK,
	SKIN_LOOSE_CLEAR
} SkinLooseAction;

static int skin_loose_mark_clear_exec(bContext *C, wmOperator *op)
{
	Object *ob = CTX_data_edit_object(C);
	BMEditMesh *em = BKE_editmesh_from_object(ob);
	BMesh *bm = em->bm;
	BMVert *bm_vert;
	BMIter bm_iter;
	SkinLooseAction action = RNA_enum_get(op->ptr, "action");

	if (!CustomData_has_layer(&bm->vdata, CD_MVERT_SKIN)) {
		return OPERATOR_CANCELLED;
	}

	BM_ITER_MESH (bm_vert, &bm_iter, bm, BM_VERTS_OF_MESH) {
		if (BM_elem_flag_test(bm_vert, BM_ELEM_SELECT)) {
			MVertSkin *vs = CustomData_bmesh_get(&bm->vdata,
			                                     bm_vert->head.data,
			                                     CD_MVERT_SKIN);


			switch (action) {
				case SKIN_LOOSE_MARK:
					vs->flag |= MVERT_SKIN_LOOSE;
					break;
				case SKIN_LOOSE_CLEAR:
					vs->flag &= ~MVERT_SKIN_LOOSE;
					break;
			}
		}
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

void OBJECT_OT_skin_loose_mark_clear(wmOperatorType *ot)
{
	static EnumPropertyItem action_items[] = {
		{SKIN_LOOSE_MARK, "MARK", 0, "Mark", "Mark selected vertices as loose"},
		{SKIN_LOOSE_CLEAR, "CLEAR", 0, "Clear", "Set selected vertices as not loose"},
		{0, NULL, 0, NULL, NULL}
	};

	ot->name = "Skin Mark/Clear Loose";
	ot->description = "Mark/clear selected vertices as loose";
	ot->idname = "OBJECT_OT_skin_loose_mark_clear";

	ot->poll = skin_edit_poll;
	ot->exec = skin_loose_mark_clear_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

	RNA_def_enum(ot->srna, "action", action_items, SKIN_LOOSE_MARK, "Action", NULL);
}

static int skin_radii_equalize_exec(bContext *C, wmOperator *UNUSED(op))
{
	Object *ob = CTX_data_edit_object(C);
	BMEditMesh *em = BKE_editmesh_from_object(ob);
	BMesh *bm = em->bm;
	BMVert *bm_vert;
	BMIter bm_iter;

	if (!CustomData_has_layer(&bm->vdata, CD_MVERT_SKIN)) {
		return OPERATOR_CANCELLED;
	}

	BM_ITER_MESH (bm_vert, &bm_iter, bm, BM_VERTS_OF_MESH) {
		if (BM_elem_flag_test(bm_vert, BM_ELEM_SELECT)) {
			MVertSkin *vs = CustomData_bmesh_get(&bm->vdata,
			                                     bm_vert->head.data,
			                                     CD_MVERT_SKIN);
			float avg = (vs->radius[0] + vs->radius[1]) * 0.5f;

			vs->radius[0] = vs->radius[1] = avg;
		}
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

void OBJECT_OT_skin_radii_equalize(wmOperatorType *ot)
{
	ot->name = "Skin Radii Equalize";
	ot->description = "Make skin radii of selected vertices equal on each axis";
	ot->idname = "OBJECT_OT_skin_radii_equalize";

	ot->poll = skin_edit_poll;
	ot->exec = skin_radii_equalize_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

static void skin_armature_bone_create(Object *skin_ob,
                                      MVert *mvert, MEdge *medge,
                                      bArmature *arm,
                                      BLI_bitmap *edges_visited,
                                      const MeshElemMap *emap,
                                      EditBone *parent_bone,
                                      int parent_v)
{
	int i;

	for (i = 0; i < emap[parent_v].count; i++) {
		int endx = emap[parent_v].indices[i];
		const MEdge *e = &medge[endx];
		EditBone *bone;
		bDeformGroup *dg;
		int v;

		/* ignore edge if already visited */
		if (BLI_BITMAP_TEST(edges_visited, endx))
			continue;
		BLI_BITMAP_ENABLE(edges_visited, endx);

		v = (e->v1 == parent_v ? e->v2 : e->v1);

		bone = ED_armature_edit_bone_add(arm, "Bone");

		bone->parent = parent_bone;
		bone->flag |= BONE_CONNECTED;

		copy_v3_v3(bone->head, mvert[parent_v].co);
		copy_v3_v3(bone->tail, mvert[v].co);
		bone->rad_head = bone->rad_tail = 0.25;
		BLI_snprintf(bone->name, sizeof(bone->name), "Bone.%.2d", endx);

		/* add bDeformGroup */
		if ((dg = BKE_object_defgroup_add_name(skin_ob, bone->name))) {
			ED_vgroup_vert_add(skin_ob, dg, parent_v, 1, WEIGHT_REPLACE);
			ED_vgroup_vert_add(skin_ob, dg, v, 1, WEIGHT_REPLACE);
		}
		
		skin_armature_bone_create(skin_ob,
		                          mvert, medge,
		                          arm,
		                          edges_visited,
		                          emap,
		                          bone,
		                          v);
	}
}

static Object *modifier_skin_armature_create(Main *bmain, Scene *scene, Object *skin_ob)
{
	BLI_bitmap *edges_visited;
	DerivedMesh *deform_dm;
	MVert *mvert;
	Mesh *me = skin_ob->data;
	Object *arm_ob;
	bArmature *arm;
	MVertSkin *mvert_skin;
	MeshElemMap *emap;
	int *emap_mem;
	int v;

	deform_dm = mesh_get_derived_deform(scene, skin_ob, CD_MASK_BAREMESH);
	mvert = deform_dm->getVertArray(deform_dm);

	/* add vertex weights to original mesh */
	CustomData_add_layer(&me->vdata,
	                     CD_MDEFORMVERT,
	                     CD_CALLOC,
	                     NULL,
	                     me->totvert);
	
	arm_ob = BKE_object_add(bmain, scene, OB_ARMATURE, NULL);
	BKE_object_transform_copy(arm_ob, skin_ob);
	arm = arm_ob->data;
	arm->layer = 1;
	arm_ob->dtx |= OB_DRAWXRAY;
	arm->drawtype = ARM_LINE;
	arm->edbo = MEM_callocN(sizeof(ListBase), "edbo armature");

	mvert_skin = CustomData_get_layer(&me->vdata, CD_MVERT_SKIN);
	BKE_mesh_vert_edge_map_create(&emap, &emap_mem,
	                     me->medge, me->totvert, me->totedge);

	edges_visited = BLI_BITMAP_NEW(me->totedge, "edge_visited");

	/* note: we use EditBones here, easier to set them up and use
	 * edit-armature functions to convert back to regular bones */
	for (v = 0; v < me->totvert; v++) {
		if (mvert_skin[v].flag & MVERT_SKIN_ROOT) {
			EditBone *bone = NULL;

			/* Unless the skin root has just one adjacent edge, create
			 * a fake root bone (have it going off in the Y direction
			 * (arbitrary) */
			if (emap[v].count > 1) {
				bone = ED_armature_edit_bone_add(arm, "Bone");

				copy_v3_v3(bone->head, me->mvert[v].co);
				copy_v3_v3(bone->tail, me->mvert[v].co);

				bone->head[1] = 1.0f;
				bone->rad_head = bone->rad_tail = 0.25;
			}
			
			if (emap[v].count >= 1) {
				skin_armature_bone_create(skin_ob,
				                          mvert, me->medge,
				                          arm,
				                          edges_visited,
				                          emap,
				                          bone,
				                          v);
			}
		}
	}

	MEM_freeN(edges_visited);
	MEM_freeN(emap);
	MEM_freeN(emap_mem);

	ED_armature_from_edit(arm);
	ED_armature_edit_free(arm);

	return arm_ob;
}

static int skin_armature_create_exec(bContext *C, wmOperator *op)
{
	Main *bmain = CTX_data_main(C);
	Scene *scene = CTX_data_scene(C);
	Object *ob = CTX_data_active_object(C), *arm_ob;
	Mesh *me = ob->data;
	ModifierData *skin_md;
	ArmatureModifierData *arm_md;

	if (!CustomData_has_layer(&me->vdata, CD_MVERT_SKIN)) {
		BKE_reportf(op->reports, RPT_WARNING, "Mesh '%s' has no skin vertex data", me->id.name + 2);
		return OPERATOR_CANCELLED;
	}

	/* create new armature */
	arm_ob = modifier_skin_armature_create(bmain, scene, ob);

	/* add a modifier to connect the new armature to the mesh */
	arm_md = (ArmatureModifierData *)modifier_new(eModifierType_Armature);
	if (arm_md) {
		skin_md = edit_modifier_property_get(op, ob, eModifierType_Skin);
		BLI_insertlinkafter(&ob->modifiers, skin_md, arm_md);

		arm_md->object = arm_ob;
		arm_md->deformflag = ARM_DEF_VGROUP | ARM_DEF_QUATERNION;
		DAG_relations_tag_update(bmain);
		DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	}

	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

	return OPERATOR_FINISHED;
}

static int skin_armature_create_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return skin_armature_create_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_skin_armature_create(wmOperatorType *ot)
{
	ot->name = "Skin Armature Create";
	ot->description = "Create an armature that parallels the skin layout";
	ot->idname = "OBJECT_OT_skin_armature_create";

	ot->poll = skin_poll;
	ot->invoke = skin_armature_create_invoke;
	ot->exec = skin_armature_create_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}
/************************ delta mush bind operator *********************/

static int correctivesmooth_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_CorrectiveSmoothModifier, 0);
}

static int correctivesmooth_bind_exec(bContext *C, wmOperator *op)
{
	Scene *scene = CTX_data_scene(C);
	Object *ob = ED_object_active_context(C);
	CorrectiveSmoothModifierData *csmd = (CorrectiveSmoothModifierData *)edit_modifier_property_get(op, ob, eModifierType_CorrectiveSmooth);
	bool is_bind;

	if (!csmd) {
		return OPERATOR_CANCELLED;
	}

	if (!modifier_isEnabled(scene, &csmd->modifier, eModifierMode_Realtime)) {
		BKE_report(op->reports, RPT_ERROR, "Modifier is disabled");
		return OPERATOR_CANCELLED;
	}

	is_bind = (csmd->bind_coords != NULL);

	MEM_SAFE_FREE(csmd->bind_coords);
	MEM_SAFE_FREE(csmd->delta_cache);

	if (is_bind) {
		/* toggle off */
		csmd->bind_coords_num = 0;
	}
	else {
		/* signal to modifier to recalculate */
		csmd->bind_coords_num = (unsigned int)-1;
	}

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);

	return OPERATOR_FINISHED;
}

static int correctivesmooth_bind_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return correctivesmooth_bind_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_correctivesmooth_bind(wmOperatorType *ot)
{
	/* identifiers */
	ot->name = "Corrective Smooth Bind";
	ot->description = "Bind base pose in Corrective Smooth modifier";
	ot->idname = "OBJECT_OT_correctivesmooth_bind";

	/* api callbacks */
	ot->poll = correctivesmooth_poll;
	ot->invoke = correctivesmooth_bind_invoke;
	ot->exec = correctivesmooth_bind_exec;

	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/************************ mdef bind operator *********************/

static int meshdeform_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_MeshDeformModifier, 0);
}

static int meshdeform_bind_exec(bContext *C, wmOperator *op)
{
	Scene *scene = CTX_data_scene(C);
	Object *ob = ED_object_active_context(C);
	MeshDeformModifierData *mmd = (MeshDeformModifierData *)edit_modifier_property_get(op, ob, eModifierType_MeshDeform);
	
	if (!mmd)
		return OPERATOR_CANCELLED;

	if (mmd->bindcagecos) {
		MEM_freeN(mmd->bindcagecos);
		if (mmd->dyngrid) MEM_freeN(mmd->dyngrid);
		if (mmd->dyninfluences) MEM_freeN(mmd->dyninfluences);
		if (mmd->bindinfluences) MEM_freeN(mmd->bindinfluences);
		if (mmd->bindoffsets) MEM_freeN(mmd->bindoffsets);
		if (mmd->dynverts) MEM_freeN(mmd->dynverts);
		if (mmd->bindweights) MEM_freeN(mmd->bindweights);  /* deprecated */
		if (mmd->bindcos) MEM_freeN(mmd->bindcos);  /* deprecated */

		mmd->bindcagecos = NULL;
		mmd->dyngrid = NULL;
		mmd->dyninfluences = NULL;
		mmd->bindinfluences = NULL;
		mmd->bindoffsets = NULL;
		mmd->dynverts = NULL;
		mmd->bindweights = NULL; /* deprecated */
		mmd->bindcos = NULL; /* deprecated */
		mmd->totvert = 0;
		mmd->totcagevert = 0;
		mmd->totinfluence = 0;
		
		DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
		WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	}
	else {
		DerivedMesh *dm;
		int mode = mmd->modifier.mode;

		/* force modifier to run, it will call binding routine */
		mmd->bindfunc = mesh_deform_bind;
		mmd->modifier.mode |= eModifierMode_Realtime;

		if (ob->type == OB_MESH) {
			dm = mesh_create_derived_view(scene, ob, 0);
			dm->release(dm);
		}
		else if (ob->type == OB_LATTICE) {
			BKE_lattice_modifiers_calc(scene, ob);
		}
		else if (ob->type == OB_MBALL) {
			BKE_displist_make_mball(CTX_data_main(C)->eval_ctx, scene, ob);
		}
		else if (ELEM(ob->type, OB_CURVE, OB_SURF, OB_FONT)) {
			BKE_displist_make_curveTypes(scene, ob, 0);
		}

		mmd->bindfunc = NULL;
		mmd->modifier.mode = mode;
	}
	
	return OPERATOR_FINISHED;
}

static int meshdeform_bind_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return meshdeform_bind_exec(C, op);
	else 
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_meshdeform_bind(wmOperatorType *ot)
{
	/* identifiers */
	ot->name = "Mesh Deform Bind";
	ot->description = "Bind mesh to cage in mesh deform modifier";
	ot->idname = "OBJECT_OT_meshdeform_bind";
	
	/* api callbacks */
	ot->poll = meshdeform_poll;
	ot->invoke = meshdeform_bind_invoke;
	ot->exec = meshdeform_bind_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/****************** explode refresh operator *********************/

static int explode_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_ExplodeModifier, 0);
}

static int explode_refresh_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	ExplodeModifierData *emd = (ExplodeModifierData *)edit_modifier_property_get(op, ob, eModifierType_Explode);
	
	if (!emd)
		return OPERATOR_CANCELLED;

	emd->flag |= eExplodeFlag_CalcFaces;

	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	
	return OPERATOR_FINISHED;
}

static int explode_refresh_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return explode_refresh_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}


void OBJECT_OT_explode_refresh(wmOperatorType *ot)
{
	ot->name = "Explode Refresh";
	ot->description = "Refresh data in the Explode modifier";
	ot->idname = "OBJECT_OT_explode_refresh";

	ot->poll = explode_poll;
	ot->invoke = explode_refresh_invoke;
	ot->exec = explode_refresh_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}


/****************** ocean bake operator *********************/

static int ocean_bake_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_OceanModifier, 0);
}

/* copied from init_ocean_modifier, MOD_ocean.c */
static void init_ocean_modifier_bake(struct Ocean *oc, struct OceanModifierData *omd)
{
	int do_heightfield, do_chop, do_normals, do_jacobian;
	
	if (!omd || !oc) return; 
	
	do_heightfield = true;
	do_chop = (omd->chop_amount > 0);
	do_normals = (omd->flag & MOD_OCEAN_GENERATE_NORMALS);
	do_jacobian = (omd->flag & MOD_OCEAN_GENERATE_FOAM);
	
	BKE_ocean_init(oc, omd->resolution * omd->resolution, omd->resolution * omd->resolution, omd->spatial_size, omd->spatial_size,
	               omd->wind_velocity, omd->smallest_wave, 1.0, omd->wave_direction, omd->damp, omd->wave_alignment,
	               omd->depth, omd->time,
	               do_heightfield, do_chop, do_normals, do_jacobian,
	               omd->seed);
}

typedef struct OceanBakeJob {
	/* from wmJob */
	void *owner;
	short *stop, *do_update;
	float *progress;
	int current_frame;
	struct OceanCache *och;
	struct Ocean *ocean;
	struct OceanModifierData *omd;
} OceanBakeJob;

static void oceanbake_free(void *customdata)
{
	OceanBakeJob *oj = customdata;
	MEM_freeN(oj);
}

/* called by oceanbake, only to check job 'stop' value */
static int oceanbake_breakjob(void *UNUSED(customdata))
{
	//OceanBakeJob *ob = (OceanBakeJob *)customdata;
	//return *(ob->stop);
	
	/* this is not nice yet, need to make the jobs list template better 
	 * for identifying/acting upon various different jobs */
	/* but for now we'll reuse the render break... */
	return (G.is_break);
}

/* called by oceanbake, wmJob sends notifier */
static void oceanbake_update(void *customdata, float progress, int *cancel)
{
	OceanBakeJob *oj = customdata;
	
	if (oceanbake_breakjob(oj))
		*cancel = 1;
	
	*(oj->do_update) = true;
	*(oj->progress) = progress;
}

static void oceanbake_startjob(void *customdata, short *stop, short *do_update, float *progress)
{
	OceanBakeJob *oj = customdata;
	
	oj->stop = stop;
	oj->do_update = do_update;
	oj->progress = progress;
	
	G.is_break = false;   /* XXX shared with render - replace with job 'stop' switch */
	
	BKE_ocean_bake(oj->ocean, oj->och, oceanbake_update, (void *)oj);
	
	*do_update = true;
	*stop = 0;
}

static void oceanbake_endjob(void *customdata)
{
	OceanBakeJob *oj = customdata;
	
	if (oj->ocean) {
		BKE_ocean_free(oj->ocean);
		oj->ocean = NULL;
	}
	
	oj->omd->oceancache = oj->och;
	oj->omd->cached = true;
}

static int ocean_bake_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	OceanModifierData *omd = (OceanModifierData *)edit_modifier_property_get(op, ob, eModifierType_Ocean);
	Scene *scene = CTX_data_scene(C);
	OceanCache *och;
	struct Ocean *ocean;
	int f, cfra, i = 0;
	const bool free = RNA_boolean_get(op->ptr, "free");
	
	wmJob *wm_job;
	OceanBakeJob *oj;
	
	if (!omd)
		return OPERATOR_CANCELLED;
	
	if (free) {
		omd->refresh |= MOD_OCEAN_REFRESH_CLEAR_CACHE;
		DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
		WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
		return OPERATOR_FINISHED;
	}

	och = BKE_ocean_init_cache(omd->cachepath, modifier_path_relbase(ob),
	                           omd->bakestart, omd->bakeend, omd->wave_scale,
	                           omd->chop_amount, omd->foam_coverage, omd->foam_fade, omd->resolution);
	
	och->time = MEM_mallocN(och->duration * sizeof(float), "foam bake time");
	
	cfra = scene->r.cfra;
	
	/* precalculate time variable before baking */
	for (f = omd->bakestart; f <= omd->bakeend; f++) {
		/* from physics_fluid.c:
		 *
		 * XXX: This can't be used due to an anim sys optimization that ignores recalc object animation,
		 * leaving it for the depgraph (this ignores object animation such as modifier properties though... :/ )
		 * --> BKE_animsys_evaluate_all_animation(G.main, eval_time);
		 * This doesn't work with drivers:
		 * --> BKE_animsys_evaluate_animdata(&fsDomain->id, fsDomain->adt, eval_time, ADT_RECALC_ALL);
		 */
		
		/* Modifying the global scene isn't nice, but we can do it in 
		 * this part of the process before a threaded job is created */
		
		//scene->r.cfra = f;
		//ED_update_for_newframe(CTX_data_main(C), scene, 1);
		
		/* ok, this doesn't work with drivers, but is way faster. 
		 * let's use this for now and hope nobody wants to drive the time value... */
		BKE_animsys_evaluate_animdata(scene, (ID *)ob, ob->adt, f, ADT_RECALC_ANIM);
		
		och->time[i] = omd->time;
		i++;
	}
	
	/* make a copy of ocean to use for baking - threadsafety */
	ocean = BKE_ocean_add();
	init_ocean_modifier_bake(ocean, omd);
	
#if 0
	BKE_ocean_bake(ocean, och);
	
	omd->oceancache = och;
	omd->cached = true;
	
	scene->r.cfra = cfra;
	
	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
#endif
	
	/* job stuff */
	
	scene->r.cfra = cfra;
	
	/* setup job */
	wm_job = WM_jobs_get(CTX_wm_manager(C), CTX_wm_window(C), scene, "Ocean Simulation",
	                     WM_JOB_PROGRESS, WM_JOB_TYPE_OBJECT_SIM_OCEAN);
	oj = MEM_callocN(sizeof(OceanBakeJob), "ocean bake job");
	oj->ocean = ocean;
	oj->och = och;
	oj->omd = omd;
	
	WM_jobs_customdata_set(wm_job, oj, oceanbake_free);
	WM_jobs_timer(wm_job, 0.1, NC_OBJECT | ND_MODIFIER, NC_OBJECT | ND_MODIFIER);
	WM_jobs_callbacks(wm_job, oceanbake_startjob, NULL, NULL, oceanbake_endjob);
	
	WM_jobs_start(CTX_wm_manager(C), wm_job);
	
	
	
	return OPERATOR_FINISHED;
}

static int ocean_bake_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return ocean_bake_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}


void OBJECT_OT_ocean_bake(wmOperatorType *ot)
{
	ot->name = "Bake Ocean";
	ot->description = "Bake an image sequence of ocean data";
	ot->idname = "OBJECT_OT_ocean_bake";
	
	ot->poll = ocean_bake_poll;
	ot->invoke = ocean_bake_invoke;
	ot->exec = ocean_bake_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
	
	RNA_def_boolean(ot->srna, "free", false, "Free", "Free the bake, rather than generating it");
}

/************************ LaplacianDeform bind operator *********************/

static int laplaciandeform_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_LaplacianDeformModifier, 0);
}

static int laplaciandeform_bind_exec(bContext *C, wmOperator *op)
{
	Object *ob = ED_object_active_context(C);
	LaplacianDeformModifierData *lmd = (LaplacianDeformModifierData *)edit_modifier_property_get(op, ob, eModifierType_LaplacianDeform);

	if (!lmd)
		return OPERATOR_CANCELLED;
	if (lmd->flag & MOD_LAPLACIANDEFORM_BIND) {
		lmd->flag &= ~MOD_LAPLACIANDEFORM_BIND;
	}
	else {
		lmd->flag |= MOD_LAPLACIANDEFORM_BIND;
	}
	DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
	return OPERATOR_FINISHED;
}

static int laplaciandeform_bind_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
		return laplaciandeform_bind_exec(C, op);
	else 
		return OPERATOR_CANCELLED;
}

void OBJECT_OT_laplaciandeform_bind(wmOperatorType *ot)
{
	/* identifiers */
	ot->name = "Laplacian Deform Bind";
	ot->description = "Bind mesh to system in laplacian deform modifier";
	ot->idname = "OBJECT_OT_laplaciandeform_bind";
	
	/* api callbacks */
	ot->poll = laplaciandeform_poll;
	ot->invoke = laplaciandeform_bind_invoke;
	ot->exec = laplaciandeform_bind_exec;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

/****************** rigidbody modifier refresh operator *********************/

typedef struct FractureJob {
	/* from wmJob */
	void *owner;
	short *stop, *do_update;
	float *progress;
	int current_frame, total_progress;
	int start, end;
	struct FractureModifierData *fmd;
	struct Object *ob;
	struct Scene *scene;
	struct Group *gr;
} FractureJob;


static void fracture_free(void *customdata)
{
	FractureJob *fj = customdata;
	MEM_freeN(fj);
}

static int fracture_breakjob(void *UNUSED(customdata))
{
	/* FractureJob *fj = (FractureJob *)customdata;
	 * return *(fj->stop); */
	return G.is_break; /* a workaround solution */
}

static void fracture_update(void *customdata)
{
	FractureJob *fj = customdata;
	float progress;

	if (fj->fmd->frac_mesh == NULL)
		(*fj->progress) = 0.0f;

	if (fracture_breakjob(fj) && fj->fmd->frac_mesh)
		fj->fmd->frac_mesh->cancel = 1;

	/*(fj->do_update) = true;  useless here... because in wm_jobs.c its set to false again, preventing update*/
	if (fj->fmd->frac_mesh)
	{
		progress = (float)(fj->fmd->frac_mesh->progress_counter) / (float)(fj->total_progress);
		(*fj->progress) = progress;
	}
}

static void fracture_startjob(void *customdata, short *stop, short *do_update, float *progress)
{
	FractureJob *fj = customdata;
	FractureModifierData *fmd = fj->fmd;
	Object *ob = fj->ob;
	Scene* scene = fj->scene;

	fj->stop = stop;
	fj->do_update = do_update;
	fj->progress = progress;
	*(fj->stop) = 0; /*false*/

	G.is_break = false;   /* XXX shared with render - replace with job 'stop' switch */

	/* arm the modifier... */
	fmd->refresh = true;
	*(fj->do_update) = true;
	*do_update = true;
	*stop = 0;

	/*...and trigger modifier execution HERE*/
	makeDerivedMesh(scene, ob, NULL, scene->customdata_mask | CD_MASK_BAREMESH, 0);
}

static void fracture_endjob(void *customdata)
{
	FractureJob *fj = customdata;
	FractureModifierData *fmd = fj->fmd;
	fmd->refresh = false;
}

static int fracture_poll(bContext *C)
{
	return edit_modifier_poll_generic(C, &RNA_FractureModifier, 0);
}

static int fracture_refresh_exec(bContext *C, wmOperator *op)
{
	Object *obact = ED_object_active_context(C);
	Scene *scene = CTX_data_scene(C);
	float cfra = BKE_scene_frame_get(scene);
	double start = 1.0;
	FractureModifierData *rmd;
	FractureJob *fj;
	wmJob* wm_job;
	//RegionView3D *rv3d = CTX_wm_region_view3d(C);

	rmd = (FractureModifierData *)modifiers_findByType(obact, eModifierType_Fracture);
	if (!rmd)
		return OPERATOR_CANCELLED;

	rmd->reset_shards = RNA_boolean_get(op->ptr, "reset");

	if (rmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		DerivedMesh *dm = rmd->visible_mesh_cached;
		if (dm)
		{
			dm->needsFree = 1;
			dm->release(dm);
			dm = rmd->visible_mesh_cached = NULL;
		}

		BKE_fracture_update_visual_mesh(rmd, obact, true);
	}

	if (scene->rigidbody_world != NULL)
	{
		start = (double)scene->rigidbody_world->pointcache->startframe;
	}

	BKE_scene_frame_set(scene, start);
	DAG_relations_tag_update(G.main);
	WM_event_add_notifier(C, NC_OBJECT | ND_TRANSFORM, NULL);
	WM_event_add_notifier(C, NC_OBJECT | ND_PARENT, NULL);
	WM_event_add_notifier(C, NC_SCENE | ND_FRAME, NULL);

	if (!rmd || (rmd && rmd->refresh) /*|| (scene->rigidbody_world && cfra != scene->rigidbody_world->pointcache->startframe)*/) {
		rmd->refresh = false;
		return OPERATOR_CANCELLED;
	}
	
	if (!rmd->execute_threaded) {
#if 0
		float vec[3] = {0.0f, 0.0f, 0.0f};
		if (rv3d != NULL)
		{
			/* need 2, 6, and 10 as forward vector, as seen in a 16-float array */
			vec[0] = rv3d->viewmat[0][2];
			vec[1] = rv3d->viewmat[1][2];
			vec[2] = rv3d->viewmat[2][2];
		}
		copy_v3_v3(rmd->forward_vector, vec);
#endif
		rmd->refresh = true;
		rmd->last_frame = INT_MAX; // delete dynamic data as well
		DAG_id_tag_update(&obact->id, OB_RECALC_DATA);
		WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, obact);
	}
	else {
		/* job stuff */
		int factor, verts, shardprogress, halvingprogress, totalprogress;
		scene->r.cfra = cfra;

		/* setup job */
		wm_job = WM_jobs_get(CTX_wm_manager(C), CTX_wm_window(C), scene, "Fracture",
							 WM_JOB_PROGRESS, WM_JOB_TYPE_OBJECT_FRACTURE);
		fj = MEM_callocN(sizeof(FractureJob), "object fracture job");
		fj->fmd = rmd;
		fj->ob = obact;
		fj->scene = scene;

		/* if we have shards, totalprogress = shards + islands
		 * if we dont have shards, then calculate number of processed halving steps
		 * if we split island to shards, add both */
		factor = (fj->fmd->frac_algorithm == MOD_FRACTURE_BISECT_FAST) ? 4 : 2;
		shardprogress = fj->fmd->shard_count * (factor+1); /* +1 for the meshisland creation */

		if (obact->derivedFinal) {
			verts = obact->derivedFinal->getNumVerts(obact->derivedFinal);
		}
		else {
			verts = ((Mesh*)obact->data)->totvert;
		}

		halvingprogress = (int)(verts / 1000) + (fj->fmd->shard_count * factor); /*-> 1000 size of each partitioned separate loose*/
		totalprogress = (rmd->shards_to_islands || rmd->point_source != MOD_FRACTURE_UNIFORM) ? shardprogress + halvingprogress : shardprogress;
		fj->total_progress = totalprogress;

		WM_jobs_customdata_set(wm_job, fj, fracture_free);
		WM_jobs_timer(wm_job, 0.1, NC_WM | ND_JOB, NC_OBJECT | ND_MODIFIER);
		WM_jobs_callbacks(wm_job, fracture_startjob, NULL, fracture_update, fracture_endjob);

		WM_jobs_start(CTX_wm_manager(C), wm_job);
	}

	return OPERATOR_FINISHED;
}

static void apply_scale(Object* ob, Scene* scene)
{
	float mat[4][4], smat[3][3];
	/*better apply scale prior to fracture, else shards get distorted*/
	BKE_object_scale_to_mat3(ob, smat);
	copy_m4_m3(mat, smat);

	/* apply to object data */
	if (ob->type == OB_MESH) {
		Mesh *me = ob->data;

		multiresModifier_scale_disp(scene, ob);

		/* adjust data */
		BKE_mesh_transform(me, mat, true);

		/* update normals */
		BKE_mesh_calc_normals(me);
	}
	else if (ELEM(ob->type, OB_CURVE, OB_SURF, OB_FONT)) {
		float scale = 1.0f;
		Curve *cu = ob->data;

		scale = mat3_to_scale(smat);
		BKE_curve_transform_ex(cu, mat, true, scale);
	}

	/*clear scale too*/
	ob->size[0] = ob->size[1] = ob->size[2] = 1.0f;
}

static int fracture_refresh_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	Scene* scene = CTX_data_scene(C);
	Object* ob = CTX_data_active_object(C);

	if (WM_jobs_test(CTX_wm_manager(C), scene, WM_JOB_TYPE_OBJECT_FRACTURE))
		return OPERATOR_CANCELLED;

	if (edit_modifier_invoke_properties(C, op))
	{
		apply_scale(ob, scene);
		return fracture_refresh_exec(C, op);
	}

	return OPERATOR_CANCELLED;
}


void OBJECT_OT_fracture_refresh(wmOperatorType *ot)
{
	ot->name = "Fracture Refresh";
	ot->description = "Refresh data in the Fracture modifier";
	ot->idname = "OBJECT_OT_fracture_refresh";

	ot->poll = fracture_poll;
	ot->invoke = fracture_refresh_invoke;
	ot->exec = fracture_refresh_exec;

	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);

	RNA_def_boolean(ot->srna, "reset", false, "Reset Shards", "Reset all shards in next refracture, instead of keeping similar ones");
}

/****************** rigidbody constraint refresh operator *********************/

#if 0
static int rigidbody_refresh_constraints_exec(bContext *C, wmOperator *UNUSED(op))
{
	Object *obact = ED_object_active_context(C);
	FractureModifierData *rmd;
	Scene *scene = CTX_data_scene(C);
	float cfra = BKE_scene_frame_get(scene);
	
	rmd = (FractureModifierData *)modifiers_findByType(obact, eModifierType_Fracture);

	if (!rmd || (rmd && rmd->refresh) || (scene->rigidbody_world && cfra != scene->rigidbody_world->pointcache->startframe))
		return OPERATOR_CANCELLED;

	rmd->refresh_constraints = true;

	DAG_id_tag_update(&obact->id, OB_RECALC_DATA);
	WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, obact);

	return OPERATOR_FINISHED;
}

static int rigidbody_refresh_constraints_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op) || true)
		return rigidbody_refresh_constraints_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}


void OBJECT_OT_rigidbody_constraints_refresh(wmOperatorType *ot)
{
	ot->name = "RigidBody Constraints Refresh";
	ot->description = "Refresh constraints in the Rigid Body modifier";
	ot->idname = "OBJECT_OT_rigidbody_constraints_refresh";

	ot->poll = fracture_poll;
	ot->invoke = rigidbody_refresh_constraints_invoke;
	ot->exec = rigidbody_refresh_constraints_exec;

	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}
#endif

static void do_add_group_unchecked(Group* group, Object *ob, Base *bas)
{
	GroupObject *go;

	go = MEM_callocN(sizeof(GroupObject), "groupobject");
	BLI_addtail(&group->gobject, go);
	go->ob = ob;

	ob->flag |= OB_FROMGROUP;
	bas->flag |= OB_FROMGROUP;
}

static bool do_unchecked_constraint_add(Scene *scene, Object *ob, RigidBodyShardCon *con, ReportList *reports, Base* base)
{
	RigidBodyWorld *rbw = BKE_rigidbody_get_world(scene);

	/* check that object doesn't already have a constraint */
	if (ob->rigidbody_constraint) {
		BKE_reportf(reports, RPT_INFO, "Object '%s' already has a Rigid Body Constraint", ob->id.name + 2);
		return false;
	}
	/* create constraint group if it doesn't already exits */
	if (rbw->constraints == NULL) {
		rbw->constraints = BKE_group_add(G.main, "RigidBodyConstraints");
	}
	/* make rigidbody constraint settings */
	ob->rigidbody_constraint = BKE_rigidbody_create_constraint(scene, ob, con->type, con);
	ob->rigidbody_constraint->flag |= RBC_FLAG_NEEDS_VALIDATE;

	/* add constraint to rigid body constraint group */
	//BKE_group_object_add(rbw->constraints, ob, scene, NULL);
	do_add_group_unchecked(rbw->constraints, ob, base);

	DAG_id_tag_update(&ob->id, OB_RECALC_OB);
	return true;
}

static Object* do_convert_meshisland_to_object(MeshIsland *mi, Scene* scene, Group* g, Object* ob,
                                            RigidBodyWorld *rbw, int i, Object*** objs, KDTree **objtree, Base** base,
                                             FractureModifierData *fmd)
{
	float cent[3];
	Mesh* me;
	ModifierData *md;
	bool foundFracture = false;
	Object* ob_new = NULL;
	bool mode = fmd->fracture_mode == MOD_FRACTURE_EXTERNAL;

	char *name = mode ? BLI_strdupn(mi->name, MAX_ID_NAME) : BLI_strdupcat(ob->id.name + 2, "_shard");

	/* create separate objects for meshislands */
#if 0
	if (ob->type == OB_MESH) {
		base_new = ED_object_add_duplicate(G.main, scene, base_old, USER_DUP_MESH);
		ob_new = base_new->object;
	}
	else {
#endif

	ob_new = BKE_object_add(G.main, scene, OB_MESH, name);
	/*set by BKE_object_add ! */
	*base = scene->basact;

	copy_m4_m4(ob_new->obmat, ob->obmat);
	copy_v3_v3(ob_new->rot, ob->rot);
	copy_qt_qt(ob_new->quat, ob->quat);
	copy_v3_v3(ob_new->rotAxis, ob->rotAxis);
	ob_new->rotAngle = ob->rotAngle;
	copy_v3_v3(ob_new->size, ob->size);

	if (rbw) {
		rbw->pointcache->flag |= PTCACHE_OUTDATED;
		/* make rigidbody object settings */
		if (ob_new->rigidbody_object == NULL) {
			ob_new->rigidbody_object = BKE_rigidbody_create_object(scene, ob_new, RBO_TYPE_ACTIVE, mi);
		}
		//ob_new->rigidbody_object->type = RBO_TYPE_ACTIVE;
		ob_new->rigidbody_object->flag |= RBO_FLAG_NEEDS_VALIDATE;

		/* add object to rigid body group */
		//BKE_group_object_add(rbw->group, ob_new, scene, NULL);
		do_add_group_unchecked(rbw->group, ob_new, *base);

		DAG_id_tag_update(&ob_new->id, OB_RECALC_OB);
	}
//}

	//BKE_group_object_add(g, ob_new, scene, NULL);
	do_add_group_unchecked(g, ob_new, *base);

	/* throw away all modifiers before fracture, result is stored inside it */
	while (ob_new->modifiers.first != NULL) {
		md = ob_new->modifiers.first;
		if (md->type == eModifierType_Fracture) {
			/*remove fracture itself too*/
			foundFracture = true;
			BLI_remlink(&ob_new->modifiers, md);
			modifier_free(md);
			md = NULL;
		}
		else if (!foundFracture) {
			BLI_remlink(&ob_new->modifiers, md);
			modifier_free(md);
			md = NULL;
		}
		/* XXX else keep following modifiers, or apply them ? */
	}

	assign_matarar(ob_new, give_matarar(ob), *give_totcolp(ob));

	me = (Mesh*)ob_new->data;
	me->edit_btmesh = NULL;

	DM_to_mesh(mi->physics_mesh, me, ob_new, CD_MASK_MESH, false);

	if (fmd->fracture_mode != MOD_FRACTURE_EXTERNAL)
	{
		/*set origin to centroid*/
		copy_v3_v3(cent, mi->centroid);
		mul_m4_v3(ob_new->obmat, cent);
		copy_v3_v3(ob_new->loc, cent);
	}
	else
	{
		int i;
		float loc[3] = {0.0f, 0.0f, 0.0f}, mat[4][4], imat[4][4];

		Shard *s = BLI_findlink(&fmd->frac_mesh->shard_map, mi->id);
		if (s)
		{
			copy_v3_v3(ob_new->size, s->impact_size);
			copy_v3_v3(ob_new->loc, mi->centroid);
			copy_qt_qt(ob_new->quat, mi->rot);
			loc_quat_size_to_mat4(ob_new->obmat, ob_new->loc, ob_new->quat, ob_new->size);
			invert_m4_m4(ob_new->imat, ob_new->obmat);

			loc_quat_size_to_mat4(mat, loc, ob_new->quat, ob_new->size);
			invert_m4_m4(imat, mat);
			//compensate for rot, size
			for (i = 0; i < me->totvert; i++)
			{
				sub_v3_v3(me->mvert[i].co, mi->centroid);
				add_v3_v3(me->mvert[i].co, s->raw_centroid);
				mul_m4_v3(imat, me->mvert[i].co);
			}

			mat4_to_axis_angle(ob_new->rotAxis, &ob_new->rotAngle, ob_new->obmat);
			mat4_to_eulO(ob_new->rot, ob_new->rotmode, ob_new->obmat);
		}
	}

	/*set mass*/
	//ob_new->rigidbody_object->mass = mi->rigidbody->mass;

	/*store obj indexes in kdtree and objs in array*/
	BLI_kdtree_insert(*objtree, i, mi->centroid);
	(*objs)[i] = ob_new;
	//i++;

	BKE_rigidbody_remove_shard(scene, mi);

	return ob_new;
}

static void do_relink_scene(Object* obj, Scene* scene, Scene* bgscene, Base ***basarray, int i, int count, int *j, Base ***basarray_old)
{
	/* add link to 2nd scene as well */
	Base *bas = BKE_scene_base_add(bgscene, obj);
	(*basarray)[i] = bas;

	if (((i > 0) && ((i % 10) == 0)) || i == count-1) {
		/*only add 10 objects to scene, then delete them in current scene (keep links being set to another scene,
		 *so the scene is empty again -> way faster */
		int k = 0;
		for (k = (*j); k < i+1; k++)
		{
			/* keep rigidbody settings ! -> dont use BKE_scene_base_unlink() */
			Base *b = (*basarray_old)[k];
			//Base *ba = BKE_scene_base_find(scene, b->object);
			BLI_remlink(&scene->base, b);
			MEM_freeN(b);
			(*basarray_old)[k] = NULL;
		}
		(*j) = i+1;
	}
}

static void do_restore_scene_link(Scene* scene, int count, Scene **bgscene, Base ***basarray, Base ***basarray_old)
{
	int i = 0;
	/*after conversion link all from second scene back to first one*/
	for (i = 0; i < count; i++) {
		Base *bas = (*basarray)[i];
		Base *b = NULL;

		/* keep rigidbody settings ! -> dont use BKE_scene_base_unlink() */
		BLI_remlink(&(*bgscene)->base, bas);
		b = BKE_scene_base_add(scene, bas->object);
		ED_base_object_select(b, BA_SELECT);
		scene->basact = b;
		MEM_freeN(bas);
		(*basarray)[i] = NULL;

		//DAG_id_tag_update(&b->object->id, OB_RECALC_DATA);
	}

	/*delete 2nd scene and basarrays*/
	MEM_freeN(*basarray);
	*basarray = NULL;

	MEM_freeN(*basarray_old);
	*basarray_old = NULL;

	BKE_scene_unlink(G.main, *bgscene, scene);
	*bgscene = NULL;
}

static Object* do_convert_constraints(FractureModifierData *fmd, RigidBodyShardCon* con, Scene* scene, Object* ob,
                                   KDTree *objtree, Object **objs, float max_con_mass, ReportList* reports, Base **base)
{
	int index1 = BLI_kdtree_find_nearest(objtree, con->mi1->centroid, NULL);
	int index2 =  BLI_kdtree_find_nearest(objtree, con->mi2->centroid, NULL);
	Object* ob1 = objs[index1];
	Object* ob2 = objs[index2];
	char *name;
	Object* rbcon;
	int iterations;

	if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		name = BLI_strdupn(con->name, MAX_ID_NAME);
	}
	else
	{
		name = BLI_strdupcat(ob->id.name + 2, "_con");
	}

	rbcon = BKE_object_add(G.main, scene, OB_EMPTY, name);

	*base = scene->basact;

	if (fmd->solver_iterations_override == 0) {
		iterations = fmd->modifier.scene->rigidbody_world->num_solver_iterations;
	}
	else {
		iterations = fmd->solver_iterations_override;
	}

	if (iterations > 0 && fmd->fracture_mode != MOD_FRACTURE_EXTERNAL) {
		con->flag |= RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS;
		con->num_solver_iterations = iterations;
	}

	if ((fmd->use_mass_dependent_thresholds)) {
		BKE_rigidbody_calc_threshold(max_con_mass, fmd, con);
	}

	if (fmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		/*TODO XXX, take own transform into account too*/
		float loc[3], rot[4], mat[4][4], size[3] = {1.0f, 1.0f, 1.0f};
		mat4_to_quat(rot, ob->obmat);
		mul_v3_m4v3(loc, ob->obmat, con->pos);
		mul_qt_qtqt(rot, rot, con->orn);

		copy_v3_v3(rbcon->loc, loc);
		copy_qt_qt(rbcon->quat, rot);
		quat_to_eulO(rbcon->rot, rbcon->rotmode, rot);
		quat_to_axis_angle(rbcon->rotAxis, &rbcon->rotAngle, rot);

		loc_quat_size_to_mat4(mat, loc, rot, size);
		copy_m4_m4(rbcon->obmat, mat);
	}
	else
	{
		if ((fmd->use_mass_dependent_thresholds)) {
			BKE_rigidbody_calc_threshold(max_con_mass, fmd, con);
		}

		/*use same settings as in modifier
		 *XXX Maybe use the CENTER between objects ? Might be correct for Non fixed constraints*/
		/* location for fixed constraints doesnt matter, so keep old setting */
		/* keep in sync with rigidbody.c, BKE_rigidbody_validate_sim_shard_constraint() */
		if (con->type == RBC_TYPE_FIXED) {
			copy_v3_v3(rbcon->loc, ob1->loc);
		}
		else {
			/* else set location to center */
			add_v3_v3v3(rbcon->loc, ob1->loc, ob2->loc);
			mul_v3_fl(rbcon->loc, 0.5f);
		}
	}

	/*omit check for existing objects in group, since this seems very slow, and should not be necessary in this internal function*/
	do_unchecked_constraint_add(scene, rbcon, con, reports, *base);

	rbcon->rigidbody_constraint->ob1 = ob1;
	rbcon->rigidbody_constraint->ob2 = ob2;
	//rbcon->rigidbody_constraint->breaking_threshold = con->breaking_threshold;
	//rbcon->rigidbody_constraint->flag |= RBC_FLAG_USE_BREAKING;

	if (con->flag & RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS) {
		rbcon->rigidbody_constraint->flag |= RBC_FLAG_OVERRIDE_SOLVER_ITERATIONS;
		rbcon->rigidbody_constraint->num_solver_iterations = iterations;
	}

	BKE_rigidbody_remove_shard_con(scene, con);

	return rbcon;
}

static void convert_modifier_to_objects(ReportList *reports, Scene* scene, Object* ob, FractureModifierData *rmd)
{
//	Base *base_new, *base_old = BKE_scene_base_find(scene, ob);
	MeshIsland *mi;
	RigidBodyShardCon* con;
	int i = 0, j = 0;
	RigidBodyWorld *rbw = scene->rigidbody_world;
	const char *name = BLI_strdupcat(ob->id.name, "_conv");
	Group *g = BKE_group_add(G.main, name);

	int count = BLI_listbase_count(&rmd->meshIslands);
	int count_con = BLI_listbase_count(&rmd->meshConstraints);
	KDTree* objtree = BLI_kdtree_new(count);
	Object** objs = MEM_callocN(sizeof(Object*) * count, "convert_objs");
	float max_con_mass = 0;

	/*use a common array for both constraints and objects ! */
	Scene *bgscene = BKE_scene_add(G.main, "Conversion");
	Base** basarray = MEM_mallocN(sizeof(Base*) * (count + count_con), "conversion_tempbases_island_object");
	Base** basarray_old = MEM_mallocN(sizeof(Base*) * (count + count_con), "conversion_tempbases_island_object_old");
	double start;

	rmd->refresh = false;
	MEM_freeN((void*)name);

	if (rbw)
		rbw->pointcache->flag |= PTCACHE_OUTDATED;

	start = PIL_check_seconds_timer();

	for (mi = rmd->meshIslands.first; mi; mi = mi->next) {
		Object* obj = do_convert_meshisland_to_object(mi, scene, g, ob, rbw, i, &objs, &objtree, &basarray_old[i], rmd);
		if (!obj) {
			return;
		}
		else {
			do_relink_scene(obj, scene, bgscene, &basarray, i, count + count_con, &j, &basarray_old);
		}
		i++;
	}

	//do_restore_scene_link(scene, count, &bgscene, &basarray);

	printf("Converting Islands to Objects done, %g\n", PIL_check_seconds_timer() - start);

	BLI_kdtree_balance(objtree);

	/* go through constraints and find objects by position
	 * constrain them with regular constraints */

	if (rmd->use_mass_dependent_thresholds) {
		max_con_mass = BKE_rigidbody_calc_max_con_mass(ob);
	}

	/* now do scene trick again for constraints */

	start = PIL_check_seconds_timer();

	//i = 0;
	//j = 0;
	//count = BLI_listbase_count(&rmd->meshConstraints);
	//bgscene = BKE_scene_add(G.main, "Conversion");
	//basarray = MEM_mallocN(sizeof(Base*) * count, "conversion_tempbases_island_constraints");

	if (rmd->use_constraints || rmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		for (con = rmd->meshConstraints.first; con; con = con->next) {
			Object* obj = do_convert_constraints(rmd, con, scene, ob, objtree, objs, max_con_mass, reports, &basarray_old[i]);
			if (!obj) {
				return;
			}
			else {
				do_relink_scene(obj, scene, bgscene, &basarray, i, count + count_con, &j, &basarray_old);
			}
			i++;
		}
	}

	/*clean up scenes again */
	do_restore_scene_link(scene, count + count_con, &bgscene, &basarray, &basarray_old);

	printf("Converting Constraints to Objects done, %g\n", PIL_check_seconds_timer() - start);

	/* free array and kdtree*/
	MEM_freeN(objs);
	BLI_kdtree_free(objtree);

	/*argh, need to trigger a world rebuild, by all means */
	/*if (rbw)
		BKE_rigidbody_rebuild_world(scene, rbw->pointcache->startframe+1);*/
}

static int rigidbody_convert_exec(bContext *C, wmOperator *op)
{
	Object *obact = ED_object_active_context(C);
	Scene *scene = CTX_data_scene(C);
	Main* bmain = CTX_data_main(C);
	wmWindowManager *wm = CTX_wm_manager(C);
	wmWindow *win;
	float cfra = BKE_scene_frame_get(scene);
	FractureModifierData *rmd;
	RigidBodyWorld *rbw = scene->rigidbody_world;
	
	rmd = (FractureModifierData *)modifiers_findByType(obact, eModifierType_Fracture);
	if (rmd && rmd->refresh) {
		return OPERATOR_CANCELLED;
	}

	if (!rmd || (scene->rigidbody_world && cfra != scene->rigidbody_world->pointcache->startframe))
	{
		BKE_report(op->reports, RPT_WARNING, "Unable to convert to objects, either no Fracture Modifier present or not on startframe" );
		return OPERATOR_CANCELLED;
	}

	convert_modifier_to_objects(op->reports, scene, obact, rmd);

	if (rbw) {
		/* flatten the cache and throw away all traces of the modifiers */
		short steps_per_second = rbw->steps_per_second;
		short num_solver_iterations = rbw->num_solver_iterations;
		int flag = rbw->flag;
		float time_scale = rbw->time_scale;
		struct Group* constraints = rbw->constraints;
		struct Group* group = rbw->group;
		RigidBodyWorld *rbwn = NULL;
		
		BKE_rigidbody_cache_reset(rbw);
		BKE_rigidbody_free_world(rbw);
		scene->rigidbody_world = NULL;
		rbwn = BKE_rigidbody_create_world(scene);
		rbwn->time_scale = time_scale;
		rbwn->flag = flag | RBW_FLAG_NEEDS_REBUILD;
		rbwn->num_solver_iterations = num_solver_iterations;
		rbwn->steps_per_second = steps_per_second;
		rbwn->group = group;
		rbwn->constraints = constraints;
		
		scene->rigidbody_world = rbwn;
	}

	if (rmd->fracture_mode == MOD_FRACTURE_EXTERNAL)
	{
		Base *bas = BKE_scene_base_find(scene, obact);
		bas->object->flag &= ~SELECT;
		ED_base_object_free_and_unlink(bmain, scene, bas);

		/* delete has to handle all open scenes, copied from delete operator */
		BKE_main_id_flag_listbase(&bmain->scene, LIB_TAG_DOIT, 1);
		for (win = wm->windows.first; win; win = win->next) {
			scene = win->screen->scene;

			if (scene->id.flag & LIB_TAG_DOIT) {
				scene->id.flag &= ~LIB_TAG_DOIT;

				DAG_relations_tag_update(bmain);

				WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
				WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, scene);
			}
		}
	}
	else
	{
		//DAG_relations_tag_update(bmain);
		WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
		WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, scene);
	}
	
	return OPERATOR_FINISHED;
}

static int rigidbody_convert_invoke(bContext *C, wmOperator *op, wmEvent *UNUSED(event))
{
	
	if (edit_modifier_invoke_properties(C, op))
		return rigidbody_convert_exec(C, op);
	else
		return OPERATOR_CANCELLED;
}


void OBJECT_OT_rigidbody_convert_to_objects(wmOperatorType *ot)
{
	ot->name = "RigidBody Convert To Objects";
	ot->description = "Convert the Rigid Body modifier shards to real objects";
	ot->idname = "OBJECT_OT_rigidbody_convert_to_objects";

	ot->poll = fracture_poll;
	ot->invoke = rigidbody_convert_invoke;
	ot->exec = rigidbody_convert_exec;

	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}

static bAnimContext* make_anim_context(Scene* scene, Object* ob)
{
	bAnimContext *ac = MEM_callocN(sizeof(bAnimContext), "make_anim_context");

	/* craft a dummy animcontext here, sigh... why cant we just pass the object and action ? */
	ac->scene = scene;
	if (scene) {
		ac->markers = &scene->markers;
		ac->obact = ob;
	}

	ac->sa = NULL;
	ac->ar = NULL;
	ac->sl = NULL;
	ac->spacetype = 0;
	ac->regiontype = 0;

	if (ob->adt)
	{
		ac->data = ob->adt->action;
		ac->datatype = ANIMCONT_ACTION;
		ac->mode = SACTCONT_ACTION;
	}

	return ac;
}

static Object* do_convert_meshIsland(FractureModifierData* fmd, MeshIsland *mi, Group* gr, Object* ob, Scene* scene,
                                  int start, int end, int count, Object* parent, bool is_baked,
                                  PTCacheID* pid, PointCache *cache, float obloc[3], float diff[3], int *j, Base **base,
                                  float threshold, bool clean_chan)
{
	int i = 0;
	Object* ob_new = NULL;
	Mesh* me;
	float cent[3];

	char *name = BLI_strdupcat(ob->id.name + 2, "_key");

	if (fmd->frac_mesh->cancel == 1)
	{
		fmd->frac_mesh->cancel = 0;
		fmd->frac_mesh->running = 0;
		return NULL;
	}

	ob_new = BKE_object_add(G.main, scene, OB_MESH, name);
	*base = scene->basact;

	//MEM_freeN((void*)name);

	ED_object_parent_set(NULL, G.main, scene, ob_new, parent, PAR_OBJECT, false, false, NULL);

	assign_matarar(ob_new, give_matarar(ob), *give_totcolp(ob));

	do_add_group_unchecked(gr, ob_new, *base);
	//bas = BKE_scene_base_find(scene, ob_new);
	//BKE_group_object_add(gr, ob_new, scene, bas);
	//scene->basact = bas;

	//old
	//ED_base_object_select(bas, BA_SELECT);

	me = (Mesh*)ob_new->data;
	me->edit_btmesh = NULL;

	DM_to_mesh(mi->physics_mesh, me, ob_new, CD_MASK_MESH, false);

	ED_rigidbody_object_add(scene, ob_new, RBO_TYPE_ACTIVE, NULL);
	ob_new->rigidbody_object->flag |= RBO_FLAG_KINEMATIC;
	ob_new->rigidbody_object->mass = mi->rigidbody->mass;

	/*set origin to centroid*/
	copy_v3_v3(cent, mi->centroid);
	mul_m4_v3(ob_new->obmat, cent);
	copy_v3_v3(ob_new->loc, cent);

	if (start < mi->start_frame) {
		start = mi->start_frame;
	}

	if (end != mi->start_frame + mi->frame_count) {
		end = mi->start_frame + mi->frame_count;
	}

	if (mi->rigidbody->type == RBO_TYPE_ACTIVE)
	{
		bAnimContext *ac;
		short flag = INSERTKEY_NEEDED | INSERTKEY_XYZ2RGB;
		for (i = start; i < end; i++)
		{
			float size[3];
			copy_v3_v3(size, ob->size);

			/*move object (loc, rot)*/
			if (i > start)
			{
				float loc[3] = {0.0f, 0.0f, 0.0f}, rot[4] = {0.0f, 0.0f, 0.0f, 0.0f};
				float mat[4][4];

				//is there a bake, if yes... use that (disabled for now, odd probs...)
				if (is_baked)
				{
					BKE_ptcache_id_time(pid, scene, (float)i, NULL, NULL, NULL);
					if (BKE_ptcache_read(pid, (float)i))
					{
						BKE_ptcache_validate(cache, i);
						copy_v3_v3(loc, mi->rigidbody->pos);
						copy_qt_qt(rot, mi->rigidbody->orn);
					}
				}
				else
				{
					loc[0] = mi->locs[i*3];
					loc[1] = mi->locs[i*3+1];
					loc[2] = mi->locs[i*3+2];

					rot[0] = mi->rots[i*4];
					rot[1] = mi->rots[i*4+1];
					rot[2] = mi->rots[i*4+2];
					rot[3] = mi->rots[i*4+3];
				}

				sub_v3_v3(loc, obloc);
				add_v3_v3(loc, diff);

				loc_quat_size_to_mat4(mat, loc, rot, size);
				BKE_scene_frame_set(scene, (double)i);

				copy_m4_m4(ob_new->obmat, mat);

				copy_v3_v3(ob_new->loc, loc);
				copy_qt_qt(ob_new->quat, rot);
				quat_to_eul(ob_new->rot, rot);
			}
			else
			{
				mul_m4_v3(ob->obmat, ob_new->loc);
				sub_v3_v3(ob_new->loc, obloc);
				add_v3_v3(ob_new->loc, diff);

				copy_qt_qt(ob_new->quat, ob->quat);
				copy_v3_v3(ob_new->rot, ob->rot);
				copy_v3_v3(ob_new->size, size);
			}

			insert_keyframe(NULL, (ID*)ob_new, NULL, "Location", "location", 0, i, flag);
			insert_keyframe(NULL, (ID*)ob_new, NULL, "Location", "location", 1, i, flag);
			insert_keyframe(NULL, (ID*)ob_new, NULL, "Location", "location", 2, i, flag);

			insert_keyframe(NULL, (ID*)ob_new, NULL, "Rotation", "rotation_euler", 0, i, flag);
			insert_keyframe(NULL, (ID*)ob_new, NULL, "Rotation", "rotation_euler", 1, i, flag);
			insert_keyframe(NULL, (ID*)ob_new, NULL, "Rotation", "rotation_euler", 2, i, flag);

			insert_keyframe(NULL, (ID*)ob_new, NULL, "Scale", "scale", 0, i, flag);
			insert_keyframe(NULL, (ID*)ob_new, NULL, "Scale", "scale", 1, i, flag);
			insert_keyframe(NULL, (ID*)ob_new, NULL, "Scale", "scale", 2, i, flag);
		}

		if (i == end)
		{
			ac = make_anim_context(scene, ob_new);
			clean_action_keys(ac, threshold, clean_chan);
			MEM_freeN(ac);
		}
	}
	else
	{
		printf("Skipping keyframing for passive shard...\n");

		mul_m4_v3(ob->obmat, ob_new->loc);
		sub_v3_v3(ob_new->loc, obloc);
		add_v3_v3(ob_new->loc, diff);

		copy_qt_qt(ob_new->quat, ob->quat);
		copy_v3_v3(ob_new->rot, ob->rot);
		copy_v3_v3(ob_new->size, ob->size);
	}

	(*j)++;
	printf("Converted %d from %d objects....\n", *j, count);

	return ob_new;
}

static bool convert_modifier_to_keyframes(FractureModifierData* fmd, Group* gr, Object* ob, Scene* scene, int start, int end,
                                          float threshold, bool clean_chan)
{
	bool is_baked = false;
	PointCache* cache = NULL;
	PTCacheID pid;
	MeshIsland *mi = NULL;
	Object *parent = NULL;
	int count = BLI_listbase_count(&fmd->meshIslands);
	char *name = BLI_strdupcat(ob->id.name + 2, "_p_key");
	float diff[3] = {0.0f, 0.0f, 0.0f};
	float obloc[3];
	int i = 0, j = 0, k = 0;
	Scene *bgscene = BKE_scene_add(G.main, "Conversion");
	Base** basarray = MEM_mallocN(sizeof(Base*) * count, "conversion_tempbases");
	Base** basarray_old = MEM_mallocN(sizeof(Base*) * count, "conversion_tempbases_old");
	double starttime;

	if (scene->rigidbody_world && scene->rigidbody_world)
	{
		cache = scene->rigidbody_world->pointcache;
	}

	if (cache && cache->flag & PTCACHE_BAKED)
	{
		start = cache->startframe;
		end = cache->endframe;
		/* need to "fill" the rigidbody world by doing 1 sim step, else bake cant be read properly */
		BKE_rigidbody_do_simulation(scene, (float)(start+1));
		BKE_ptcache_id_from_rigidbody(&pid, NULL, scene->rigidbody_world);
		is_baked = true;
	}

	parent = BKE_object_add(G.main, scene, OB_EMPTY, name);
	BKE_mesh_center_centroid(ob->data, obloc);
	copy_v3_v3(parent->loc, ob->loc);
	sub_v3_v3v3(diff, obloc, parent->loc);
	//MEM_freeN((void*)name);

	starttime = PIL_check_seconds_timer();
	for (mi = fmd->meshIslands.first; mi; mi = mi->next)
	{
		Object *obj = do_convert_meshIsland(fmd, mi, gr, ob, scene, start, end, count,
		                                    parent, is_baked, &pid, cache, obloc, diff, &k, &basarray_old[i],
		                                    threshold, clean_chan);
		if (!obj) {
			return false;
		}
		else
		{
			do_relink_scene(obj, scene, bgscene, &basarray, i, count, &j, &basarray_old);
		}
		i++;
	}

	do_restore_scene_link(scene, count, &bgscene, &basarray, &basarray_old);
	printf("Converting Islands to Keyframed Objects done, %g\n", PIL_check_seconds_timer() - starttime);

	BKE_scene_frame_set(scene, (double)start);

	//select / make FM object active
	/*bas = BKE_scene_base_find(scene, ob);
	scene->basact = bas;
	ED_base_object_select(bas, BA_SELECT);*/

	return true;
}

static void convert_free(void *customdata)
{
	FractureJob *fj = customdata;
	MEM_freeN(fj);
}

static void convert_update(void *customdata)
{
	FractureJob *fj = customdata;
	FractureModifierData *fmd = fj->fmd;
	float progress;

	if ((G.is_break) && (fmd->frac_mesh))
		fmd->frac_mesh->cancel = 1;

	progress = (float)(BLI_listbase_count(&fj->gr->gobject)) / (float)(fj->total_progress);
	(*fj->progress) = progress;
}

static void convert_startjob(void *customdata, short *stop, short *do_update, float *progress)
{
	FractureJob *fj = customdata;
	FractureModifierData *fmd = fj->fmd;
	Group* gr = fj->gr;
	Object* ob = fj->ob;
	Scene *scene = fj->scene;
	int start = fj->start;
	int end = fj->end;

	fj->stop = stop;
	fj->do_update = do_update;
	fj->progress = progress;
	*(fj->stop) = 0; /*false*/

	G.is_break = false;   /* XXX shared with render - replace with job 'stop' switch */
	*(fj->do_update) = true;
	*do_update = true;
	*stop = 0;

	if (fmd->frac_mesh) {
		fmd->frac_mesh->cancel = 0;
		fmd->frac_mesh->running = 1;
	}

#if 0 //XXX: deactivated for now
	convert_modifier_to_keyframes(fmd, gr, ob, scene, start, end);
#endif
}

static int rigidbody_convert_keyframes_exec(bContext *C, wmOperator *op)
{
	Object *obact = ED_object_active_context(C);
	Scene *scene = CTX_data_scene(C);
	float cfra = BKE_scene_frame_get(scene);
	Group *gr;
	FractureModifierData *rmd;
	FractureJob *fj;
	wmJob* wm_job;

	bool convertable = true;

	if (scene->rigidbody_world && scene->rigidbody_world)
	{
		PointCache* cache = NULL;
		cache = scene->rigidbody_world->pointcache;
		if (cache && (cache->flag & PTCACHE_OUTDATED) && !(cache->flag & PTCACHE_BAKED))
		{
			convertable = false;
		}
	}

	if (convertable)
	{
		float threshold = RNA_float_get(op->ptr, "threshold");
		bool clean_chan = RNA_boolean_get(op->ptr, "channels");

		CTX_DATA_BEGIN(C, Object *, selob, selected_objects)
		{
			rmd = (FractureModifierData *)modifiers_findByType(selob, eModifierType_Fracture);
			if (rmd && rmd->refresh) {
				return OPERATOR_CANCELLED;
			}

			if (rmd) {
				int count = BLI_listbase_count(&rmd->meshIslands);
				int start = RNA_int_get(op->ptr, "start_frame");
				int end = RNA_int_get(op->ptr, "end_frame");

				if (count == 0)
				{
					//gaaah, undo frees sim data, rebuild....
					rmd->refresh = true;
					makeDerivedMesh(scene, selob, NULL, scene->customdata_mask | CD_MASK_BAREMESH, 0);
					count = BLI_listbase_count(&rmd->meshIslands);

					if (count == 0)
					{
						BKE_report(op->reports, RPT_WARNING, "No meshislands found, please execute fracture and simulate first");
						return OPERATOR_CANCELLED;
					}
				}

				gr = BKE_group_add(G.main, "Converted");

				if (rmd->execute_threaded)
				{
					PointerRNA *ptr;
					/* what a dirty hack.... disable poll function */
					wmOperatorType *ot = WM_operatortype_find("ANIM_OT_keyframe_insert_menu", 0);
					ot->poll = NULL;

					ptr = MEM_dupallocN(op->ptr);
					ptr->type = op->ptr->type;
					ptr->id = op->ptr->id;
					ptr->id.data = op->ptr->id.data;
					ptr->data = op->ptr->data;

					/* job stuff */
					scene->r.cfra = cfra;

					/* setup job */
					wm_job = WM_jobs_get(CTX_wm_manager(C), CTX_wm_window(C), scene, "Convert to Keyframed Objects",
										 WM_JOB_PROGRESS, WM_JOB_TYPE_OBJECT_FRACTURE);
					fj = MEM_callocN(sizeof(FractureJob), "convert to Keyframes job");
					fj->fmd = rmd;
					fj->total_progress = count;
					fj->gr = gr;
					fj->ob = obact;
					fj->scene = scene;
					fj->start = start;
					fj->end = end;

					WM_jobs_customdata_set(wm_job, fj, convert_free);
					WM_jobs_timer(wm_job, 0.1, NC_WM | ND_JOB, NC_OBJECT | ND_MODIFIER);
					WM_jobs_callbacks(wm_job, convert_startjob, NULL, convert_update, NULL);

					WM_jobs_start(CTX_wm_manager(C), wm_job);

					return OPERATOR_FINISHED;
				}
				else
				{
					convert_modifier_to_keyframes(rmd, gr, selob, scene, start, end, threshold, clean_chan);
				}
			}
		}

		CTX_DATA_END;
		DAG_relations_tag_update(G.main);
		WM_event_add_notifier(C, NC_OBJECT | ND_TRANSFORM, NULL);
		WM_event_add_notifier(C, NC_OBJECT | ND_PARENT, NULL);
		WM_event_add_notifier(C, NC_SCENE | ND_FRAME, NULL);
		return OPERATOR_FINISHED;
	}
	else
	{
		BKE_report(op->reports, RPT_WARNING, "No valid cache data found, please run or bake simulation first");
		return OPERATOR_CANCELLED;
	}
}

static int rigidbody_convert_keyframes_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
	if (edit_modifier_invoke_properties(C, op))
	{
		return rigidbody_convert_keyframes_exec(C, op);
	}

	return OPERATOR_CANCELLED;
}

void OBJECT_OT_rigidbody_convert_to_keyframes(wmOperatorType *ot)
{

	ot->name = "Convert To Keyframed Objects";
	ot->description = "Convert the Rigid Body modifier shards to keyframed real objects";
	ot->idname = "OBJECT_OT_rigidbody_convert_to_keyframes";

	ot->poll = fracture_poll;
	ot->exec = rigidbody_convert_keyframes_exec;
	ot->invoke = rigidbody_convert_keyframes_invoke;

	RNA_def_int(ot->srna, "start_frame", 1,  0, 100000, "Start Frame", "", 0, 100000);
	RNA_def_int(ot->srna, "end_frame", 250, 0, 100000, "End Frame", "", 0, 100000);

	RNA_def_float(ot->srna, "threshold", 0.05f, 0.0f, FLT_MAX, "Threshold", "", 0.0f, 1000.0f);
	RNA_def_boolean(ot->srna, "channels", false, "Channels", "");

	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO | OPTYPE_INTERNAL;
	edit_modifier_properties(ot);
}



