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
 * Contributor(s): Blender Foundation 2014, Martin Felke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file rna_fracture.c
 *  \ingroup rna
 *  \brief RNA property definitions for Fracture datatypes
 */

#include <stdlib.h>
#include <string.h>

#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "rna_internal.h"

#include "DNA_group_types.h"
#include "DNA_object_types.h"
#include "DNA_fracture_types.h"

#include "WM_types.h"

void RNA_def_fracture(BlenderRNA *brna)
{
	StructRNA *srna;
	PropertyRNA *prop;
	
	/*srna = RNA_def_struct(brna, "FractureContainer", NULL);
	RNA_def_struct_sdna(srna, "FractureContainer");
	RNA_def_struct_ui_text(srna, "Fracture Container", "Settings for object containing fracture subgeometry");
	RNA_def_struct_path_func(srna, "rna_RigidBodyOb_path");*/
}
