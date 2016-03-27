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
 * The Original Code is Copyright (C) 2016 Blender Foundation.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 *
 */

#ifdef WITH_ALEMBIC

#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"

#include "BKE_context.h"
#include "BKE_global.h"
#include "BKE_main.h"
#include "BKE_report.h"

#include "BLI_path_util.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "RNA_access.h"
#include "RNA_define.h"

#include "WM_api.h"
#include "WM_types.h"

#include "io_alembic.h"

#include "ABC_alembic.h"

static int wm_alembic_export_invoke(bContext *C, wmOperator *op, const wmEvent *event)
{
	if (!RNA_struct_property_is_set(op->ptr, "filepath")) {
		char filepath[FILE_MAX];
		BLI_strncpy(filepath, G.main->name, sizeof(filepath));
		BLI_replace_extension(filepath, sizeof(filepath), ".abc");
		RNA_string_set(op->ptr, "filepath", filepath);
	}

	WM_event_add_fileselect(C, op);

	return OPERATOR_RUNNING_MODAL;

	UNUSED_VARS(event);
}

/* function used for WM_OT_save_mainfile too */
static int wm_alembic_export_exec(bContext *C, wmOperator *op)
{
	if (!RNA_struct_property_is_set(op->ptr, "filepath")) {
		BKE_report(op->reports, RPT_ERROR, "No filename given");
		return OPERATOR_CANCELLED;
	}

	char filename[FILE_MAX];
	RNA_string_get(op->ptr, "filepath", filename);
	int start = RNA_int_get(op->ptr, "start");
	int end = RNA_int_get(op->ptr, "end");
	int xsamples = RNA_int_get(op->ptr, "xsamples");
	int gsamples = RNA_int_get(op->ptr, "gsamples");
	float sh_open = RNA_float_get(op->ptr, "sh_open");
	float sh_close = RNA_float_get(op->ptr, "sh_close");

	bool selected = RNA_boolean_get(op->ptr, "selected");
	bool uvs = RNA_boolean_get(op->ptr, "uvs");
	bool normals = RNA_boolean_get(op->ptr, "normals");
	bool vcolors = RNA_boolean_get(op->ptr, "vcolors");
	bool forcemeshes = RNA_boolean_get(op->ptr, "forcemeshes");
	bool flatten = RNA_boolean_get(op->ptr, "flatten");
	bool geoprops = RNA_boolean_get(op->ptr, "geoprops");
	bool renderable = RNA_boolean_get(op->ptr, "renderable");
	bool vislayers = RNA_boolean_get(op->ptr, "vislayers");
	bool facesets = RNA_boolean_get(op->ptr, "facesets");
	bool matindices = RNA_boolean_get(op->ptr, "matindices");
	bool subdiv_schem = RNA_boolean_get(op->ptr, "subdiv_schema");
	bool ogawa = RNA_boolean_get(op->ptr, "ogawa");
	bool packuv = RNA_boolean_get(op->ptr, "packuv");

	int result = ABC_export(CTX_data_scene(C), filename,
	                        start, end,
	                        (double) 1.0 / xsamples,
	                        (double) 1.0 / gsamples,
	                        sh_open, sh_close,
	                        selected, uvs, normals, vcolors,
	                        forcemeshes, flatten, geoprops,
	                        vislayers, renderable, facesets, matindices,
	                        subdiv_schem, ogawa, packuv);

	switch (result) {
		case BL_ABC_UNKNOWN_ERROR:
			BKE_report(op->reports, RPT_ERROR, "Unknown error found while exporting Alembic archive.");
		break;
	}

	return OPERATOR_FINISHED;
}

void WM_OT_alembic_export(wmOperatorType *ot)
{
	ot->name = "Export Alembic Archive";
	ot->idname = "WM_OT_alembic_export";

	ot->invoke = wm_alembic_export_invoke;
	ot->exec = wm_alembic_export_exec;
	ot->poll = WM_operator_winactive;

	WM_operator_properties_filesel(ot, 0, FILE_BLENDER, FILE_SAVE, WM_FILESEL_FILEPATH,
	                               FILE_DEFAULTDISPLAY, FILE_SORT_ALPHA);

	RNA_def_int(ot->srna, "start", 1, INT_MIN, INT_MAX, "Start frame", "Start Frame", INT_MIN, INT_MAX);
	RNA_def_int(ot->srna, "end", 1, INT_MIN, INT_MAX, "End frame", "End Frame", INT_MIN, INT_MAX);

	RNA_def_int(ot->srna, "xsamples", 1, 1, 128, "Xform samples / frame", "Transform samples per frame", 1, 128);
	RNA_def_int(ot->srna, "gsamples", 1, 1, 128, "Geom samples / frame", "Geometry samples per frame", 1, 128);
	RNA_def_float(ot->srna, "sh_open", 0.0f, -1.0f, 1.0f, "Shutter open", "", -1.0f, 1.0f);
	RNA_def_float(ot->srna, "sh_close", 1.0f, -1.0f, 1.0f, "Shutter close", "", -1.0f, 1.0f);

	RNA_def_boolean(ot->srna, "selected"	, 0, "Selected objects only", "Export only selected objects");
	RNA_def_boolean(ot->srna, "renderable"	, 1, "Renderable objects only", "Export only objects marked renderable in the outliner");
	RNA_def_boolean(ot->srna, "vislayers"	, 0, "Visible layers only", "Export only objects in visible layers");
	RNA_def_boolean(ot->srna, "flatten"		, 0, "Flatten hierarchy", "Flatten hierarchy");
	RNA_def_boolean(ot->srna, "uvs"			, 1, "UVs", "Export UVs");
	RNA_def_boolean(ot->srna, "normals"		, 1, "Normals", "Export normals");
	RNA_def_boolean(ot->srna, "vcolors"		, 0, "Vertex colors", "Export vertex colors");
	RNA_def_boolean(ot->srna, "facesets"	, 0, "Facesets", "Export facesets");
	RNA_def_boolean(ot->srna, "matindices"	, 0, "Material indices", "Export per face material indices");
	RNA_def_boolean(ot->srna, "subdiv_schema"	, 0, "Use Alembic subdiv schema", "Export with subdiv schema, else with mesh schema");
	RNA_def_boolean(ot->srna, "geoprops"	, 0, "Custom props as geom data", "Write custom properties as geometry props (If not checked, hey are written as user data)");
	RNA_def_boolean(ot->srna, "forcemeshes"	, 0, "Subsurfs as meshes", "Export subdivision surfaces as meshes");
	RNA_def_boolean(ot->srna, "ogawa"		, 0, "Export Ogawa", "Export as Ogawa archive type");
	RNA_def_boolean(ot->srna, "packuv"		, 1, "Pack UV islands", "Export UV with packed island");
}

#endif
