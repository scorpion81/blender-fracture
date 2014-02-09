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

/** \file blender/editors/physics/physics_fracture.c
 *  \ingroup editors_physics
 *
 * Fractures Objects to smaller shards
 *
 */

#include "MEM_guardedalloc.h"

#include "BLI_math.h"
#include "BLI_rand.h"

#include "DNA_object_types.h"

#include "BKE_fracture.h"
#include "BKE_modifier.h"
#include "BKE_object.h"
#include "BKE_context.h"
#include "BKE_depsgraph.h"

#include "ED_screen.h"

#include "WM_types.h"
#include "WM_api.h"

#include "physics_intern.h" // own include

//#define NAMELEN 64 //namebased, yuck... better dont try longer names.... NOT happy with this, but how to find according group else ? cant store it anywhere
static int mesh_fracture_exec(bContext *C, wmOperator *UNUSED(op))
{
	//go through all selected objects !
	/*Scene* scene = CTX_data_scene(C);
	
	//needed for correctly update DAG after delete
	Main *bmain = CTX_data_main(C);
	wmWindowManager *wm = CTX_wm_manager(C);
	wmWindow *win;
	
	float imat[4][4], oldobmat[4][4];
	int use_boolean = RNA_boolean_get(op->ptr, "use_boolean");
	int solver_iterations = RNA_int_get(op->ptr, "iterations");
	int breaking_threshold = RNA_float_get(op->ptr, "threshold");*/
	
	//call kernel functions... just the fracture ! but need current ob
	Object *ob = CTX_data_active_object(C);
	FractureModifierData* fracmd;
	
	//find modifierdata, getFracmesh
	fracmd = (FractureModifierData *)modifiers_findByType(ob, eModifierType_Fracture);
	if (fracmd->frac_mesh != NULL)
	{
		BoundBox *bb;
		float min[3], max[3];
		/* dummy point cloud, random */
		FracPointCloud points;
		int i;
		
		bb = BKE_object_boundbox_get(ob);
		INIT_MINMAX(min, max);
		for (i = 0; i < 8; i++)
			minmax_v3v3_v3(min, max, bb->vec[i]);
		
		points.totpoints = 10;
		
		points.points = MEM_mallocN(sizeof(FracPoint) * points.totpoints, "random points");
		BLI_srandom(12345);
		for (i = 0; i < points.totpoints; ++i) {
			float *co = points.points[i].co;
			co[0] = min[0] + (max[0] - min[0]) * BLI_frand();
			co[1] = min[1] + (max[1] - min[1]) * BLI_frand();
			co[2] = min[2] + (max[2] - min[2]) * BLI_frand();
		}
		
		//pick 1st shard, hardcoded by now
		//execute fracture....
		BKE_fracture_shard_by_points(fracmd->frac_mesh, 0, &points, fracmd->frac_algorithm, ob);
		
		MEM_freeN(points.points);
		
		BKE_fracture_create_dm(fracmd, false);
		
		//WM_event_add_notifier(C, NC_OBJECT | ND_TRANSFORM, NULL);
		//WM_event_add_notifier(C, NC_OBJECT | ND_POINTCACHE, NULL);
		DAG_id_tag_update(&ob->id, OB_RECALC_DATA);
		WM_event_add_notifier(C, NC_OBJECT | ND_MODIFIER, ob);
		
		return OPERATOR_FINISHED;
	}
	
	return OPERATOR_CANCELLED;
}


void OBJECT_OT_fracture(wmOperatorType *ot)
{
	/*PropertyRNA *prop;
	
	static EnumPropertyItem prop_point_source_items[] = {
		{eOwnParticles, "OWN_PARTICLES", 0, "Own Particles", "Use own particles as point cloud"},
		{eOwnVerts, "OWN_VERTS", 0, "Own Vertices", "Use own vertices as point cloud"},
		{eExtraParticles, "EXTRA_PARTICLES", 0, "Extra Particles", "Use particles of group objects as point cloud"},
		{eExtraVerts, "EXTRA_VERTS", 0, "Extra Vertices", "Use vertices of group objects as point cloud"},
		{eGreasePencil, "GREASE_PENCIL", 0, "Grease Pencil", "Use grease pencil points as point cloud"},
		{0, NULL, 0, NULL, NULL}
	};*/
	
	/* identifiers */
	ot->idname = "OBJECT_OT_fracture";
	ot->name = "Fracture Mesh";
	ot->description = "Fracture mesh to shards";

	/* callbacks */
	ot->exec = mesh_fracture_exec;
	ot->poll = ED_operator_object_active_editable;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

	/* properties */
	/*prop = RNA_def_enum(ot->srna, "point_source", prop_point_source_items, eOwnParticles, "Point Source", "Source of point cloud");
	RNA_def_property_flag(prop, PROP_ENUM_FLAG);

	RNA_def_boolean(ot->srna, "use_boolean", FALSE, "Use Boolean Intersection", "Intersect shards with original object shape");
	
	RNA_def_float(ot->srna, "noise", 0.0f, 0.0f, 1.0f, "Noise", "Noise to apply over pointcloud", 0.0f, 1.0f);
	RNA_def_int(ot->srna, "percentage", 100, 0, 100, "Percentage", "Percentage of point to actually use for fracture", 0, 100);
	
	RNA_def_float(ot->srna, "threshold", 10.0f, 0.0f, FLT_MAX, "Breaking threshold", "Noise to apply over pointcloud", 0.0f, FLT_MAX);
	RNA_def_int(ot->srna, "iterations", 30, 0, INT_MAX , "Solver Iterations", "Percentage of point to actually use for fracture", 0, INT_MAX);*/
}

static int fracture_mode_toggle_exec(bContext *C, wmOperator *UNUSED(op))
{
	Object *ob = CTX_data_active_object(C);
	const int mode_flag = OB_MODE_FRACTURE;
	const bool is_mode_set = (ob->mode & mode_flag) != 0;

	/*if (!is_mode_set) {
		if (!ED_object_mode_compat_set(C, ob, mode_flag, op->reports)) {
			return OPERATOR_CANCELLED;
		}
	}*/

	if (is_mode_set)
	{
		/* Leave fracturemode */
		ob->mode &= ~mode_flag;

		//could also set modifier here automagically, hmm, but leave this as is for now
	}
	else
	{
		/* Enter fracturemode */
		ob->mode |= mode_flag;
	}

	return OPERATOR_FINISHED;
}

void FRACTURE_OT_fracturemode_toggle(wmOperatorType* ot)
{
	/* identifiers */
	ot->idname = "FRACTURE_OT_fracturemode_toggle";
	ot->name = "Fracture Mode";
	ot->description = "Toggle fracture mode in 3D view";

	/* api callbacks */
	ot->exec = fracture_mode_toggle_exec;
	ot->poll = ED_operator_object_active_editable_mesh;

	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

