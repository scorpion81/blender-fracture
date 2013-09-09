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

//#include <stdlib.h>
//#include <string.h>
#include <ctype.h>

#include "MEM_guardedalloc.h"

#include "DNA_group_types.h"
#include "DNA_object_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"
#include "DNA_gpencil_types.h"
#include "DNA_modifier_types.h" //NEEDED temporarily for enums related to fracture e.g. point source, TODO move this elsewhere
#include "DNA_material_types.h"
#include "DNA_mesh_types.h"

#include "BLI_blenlib.h"
#include "BLI_math.h"
#include "BLI_rand.h"

#include "BKE_context.h"
#include "BKE_depsgraph.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_object.h"
#include "BKE_report.h"
#include "BKE_rigidbody.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_mesh.h"
#include "BKE_particle.h"
#include "BKE_material.h"
#include "BKE_library.h"
#include "BKE_main.h"
#include "BKE_scene.h"

#include "RNA_access.h"
#include "RNA_define.h"
#include "RNA_enum_types.h"

#include "WM_api.h"
#include "WM_types.h"

#include "ED_physics.h"
#include "ED_screen.h"
#include "ED_object.h"

#include "physics_intern.h"
#include "bmesh.h"
#include "MOD_boolean_util.h"
#include "UI_interface.h"
#include "UI_interface_icons.h"

#ifdef WITH_MOD_VORONOI
#  include "../../../../extern/voro++/src/c_interface.hh"
#endif

//this is per shard
typedef struct neighborhood {
	int pid;
	int neighborcount;
	int* neighbor_pids;
} neighborhood;


static int points_from_verts(Object** ob, int totobj, float*** points, int p_exist, float mat[4][4], float thresh, DerivedMesh* dm, Object* obj, Scene* scene, wmOperator* op)
{
	int v, o, pt = p_exist;
	float co[3];
	int point_source = RNA_enum_get(op->ptr, "point_source");
	
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
				d = mesh_get_derived_final(scene, ob[o], 0);
			}

			invert_m4_m4(imat, mat);
			vert = d->getVertArray(d);

			for (v = 0; v < d->getNumVerts(d); v++)
			{
				if (BLI_frand() < thresh) {
					*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
					(*points)[pt] = MEM_callocN(3 * sizeof(float), "points[pt]");
				
					//copy_v3_v3(co, me->mvert[v].co);
					copy_v3_v3(co, vert[v].co);

					if ((o > 0) ||
					   ((point_source & eExtraVerts) &&
					   (!(point_source & eOwnVerts)) && (o == 0)))
					{
						mul_m4_v3(ob[o]->obmat, co);
						mul_m4_v3(imat, co);
					}

					copy_v3_v3((*points)[pt], co);
					pt++;
				}
			}
		}
	}
	
	return pt;
}

static int points_from_particles(Object** ob, int totobj, Scene* scene, float*** points, int p_exist, float mat[4][4], float thresh, wmOperator* op)
{
	int o, p, pt = p_exist;
	ParticleSystemModifierData* psmd;
	ParticleData* pa;
	ParticleSimulationData sim = {NULL};
	ParticleKey birth;
	ModifierData* mod;
	int point_source = RNA_enum_get(op->ptr, "point_source");
	
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
					if (BLI_frand() < thresh) {
						float co[3];
						psys_get_birth_coordinates(&sim, pa, &birth, 0, 0);
						*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
						(*points)[pt] = MEM_callocN(3*sizeof(float), "points[pt]");
						copy_v3_v3(co, birth.co);

						if ((o > 0) ||
						   ((point_source & eExtraParticles) &&
						   (!(point_source & eOwnParticles)) && (o == 0)))
						{
							//mul_m4_v3(ob[o]->obmat, co);
							mul_m4_v3(imat, co);
						}
						copy_v3_v3((*points)[pt], co);
						pt++;
					}
				}
			}
		}
	}
	
	return pt;
}

static int points_from_greasepencil(Object** ob, int totobj, float*** points, int p_exist, float mat[4][4], float thresh)
{
	bGPDlayer* gpl;
	bGPDframe* gpf;
	bGPDstroke* gps;
	int pt = p_exist, p, o;
	
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
								*points = MEM_reallocN(*points, (pt+1)*sizeof(float*));
								(*points)[pt] = MEM_callocN(3*sizeof(float), "points[pt]");

								point[0] = gps->points[p].x;
								point[1] = gps->points[p].y;
								point[2] = gps->points[p].z;

								mul_m4_v3(imat, point);
								copy_v3_v3((*points)[pt], point);
								pt++;
							}
						}
					}
				}
			}
		}
	}
	
	return pt;
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

static int get_points(Scene *scene, Object *ob, float ***points, float mat[4][4], DerivedMesh *derivedData, wmOperator* op)
{
	int totpoint = 0, totgroup = 0, t = 0;
	Object** go = MEM_mallocN(sizeof(Object*), "groupobjects");
	int percentage = RNA_int_get(op->ptr, "percentage");
	float thresh = (float)percentage / 100.0f;
	BoundBox* bb;
	int point_source = RNA_enum_get(op->ptr, "point_source");
	Group* extra_group = NULL;// (Group*)RNA_pointer_get(op->ptr, "extra_group").id.data;
	float noise = RNA_float_get(op->ptr, "noise");

	if (point_source & (eExtraParticles | eExtraVerts ))
	{
		if (((point_source & eOwnParticles) && (point_source & eExtraParticles)) ||
			((point_source & eOwnVerts) && (point_source & eExtraVerts)) ||
			((point_source & eGreasePencil) && (point_source & eExtraParticles)) ||
			((point_source & eGreasePencil) && (point_source & eExtraVerts)))
		{
			go = MEM_reallocN(go, sizeof(Object*)*(totgroup+1));
			go[totgroup] = ob;
			totgroup++;
		}

		totgroup = getGroupObjects(extra_group, &go, totgroup);
	}
	else
	{
		totgroup = 1;
		go[0] = ob;
	}
	
	if (point_source & (eOwnParticles | eExtraParticles))
	{
		totpoint = points_from_particles(go, totgroup, scene, points, totpoint, mat, thresh, op);
	}
	
	if (point_source & (eOwnVerts | eExtraVerts))
	{
		totpoint = points_from_verts(go, totgroup, points, totpoint, mat, thresh, derivedData, ob, scene, op);
	}
	
	if (point_source & eGreasePencil)
	{
		totpoint = points_from_greasepencil(go, totgroup, points, totpoint, mat, thresh);
	}

	//apply noise

	bb = BKE_object_boundbox_get(ob); //maybe derivedFinal here too ?
	for (t = 0; t < totpoint; t++) {
		float bbox_min[3], bbox_max[3];

		if (noise > 0.0f) {
			float scalar, size[3], rand[3] = {0, 0, 0}, temp_x[3], temp_y[3], temp_z[3], rand_x[3], rand_y[3], rand_z[3];
			mul_v3_m4v3(bbox_min, ob->obmat, bb->vec[0]);
			mul_v3_m4v3(bbox_max, ob->obmat, bb->vec[6]);
			sub_v3_v3v3(size, bbox_max, bbox_min);

			scalar = noise * len_v3(size) / 2.0f;
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

			add_v3_v3v3(temp_x, (*points)[t], rand_x);
			add_v3_v3v3(temp_y, (*points)[t], rand_y);
			add_v3_v3v3(temp_z, (*points)[t], rand_z);

			//stay inside bounds !!
			if ((temp_x[0] >= bb->vec[0][0]) && (temp_x[0] <= bb->vec[6][0]))
				add_v3_v3((*points)[t], rand_x);

			if ((temp_y[1] >= bb->vec[0][1]) && (temp_y[1] <= bb->vec[6][1]))
				add_v3_v3((*points)[t], rand_y);

			if ((temp_z[2] >= bb->vec[0][2]) && (temp_z[2] <= bb->vec[6][2]))
				add_v3_v3((*points)[t], rand_z);
		}
	}

	//MEM_freeN(bb);
	MEM_freeN(go);
	return totpoint;
}

int dm_minmax(DerivedMesh* dm, float min[3], float max[3])
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

// create the voronoi cell faces inside the existing mesh
int fractureToCells(Object *ob, float mat[4][4], wmOperator* op, Scene* scene, Object*** shards, neighborhood*** n, GHash** pid_to_index)
{
	void* container = NULL;
	particle_order* p_order = NULL;
	loop_order* l_order = NULL;
	float min[3], max[3];
	int p = 0;
	float co[3], vco[3];
	BMesh *bm = NULL, *bmtemp = NULL;

	FILE *fp = NULL;
	int vert_index = 0, edg_index = 0, fac_index = 0;
	int read = 0;
	BMVert **faceverts = NULL, **tempvert = NULL, *vert = NULL;
	BMEdge **faceedges = NULL, *edge = NULL;
	int face_index = 0;
	int edge_index = 0;
	char c;
	int facevert_index = 0;
	BMFace *face = NULL;

	DerivedMesh *dm = NULL, *boolresult = NULL;
	int tempvert_index;
	const char *file;
	char *path, *fullpath;
	float imat[4][4];
	float theta = 0.0f;
	int n_size = 8;

	float** points = NULL;
	int totpoint = 0;
	int neighbor_index = 0;
	float centroid[3];
	Object* o = NULL, *tempOb = NULL;
	int objcount = 0;
	Material* inner_material = NULL;// (Material*)RNA_pointer_get(op->ptr, "inner_material").data;
	DerivedMesh* derivedData = mesh_get_derived_final(scene, ob, 0);
	
	int use_boolean = RNA_boolean_get(op->ptr, "use_boolean");
	int point_source = RNA_enum_get(op->ptr, "point_source");
	
	INIT_MINMAX(min, max);

	if (use_boolean) {
		theta = 0.1f;
	}

	dm_minmax(derivedData, min, max);
	//BKE_mesh_minmax(ob->data, min, max); //or calc modifiers first ?, ob->derivedFinal maybe ?)
	//use global coordinates for container, necessary, since obmat is unity... hmmm
	mul_m4_v3(ob->obmat, min);
	mul_m4_v3(ob->obmat, max);
	
	points = MEM_mallocN(sizeof(float*), "points");
	mesh_get_derived_final(scene, ob, 0);
	totpoint = get_points(scene, ob, &points, mat, ob->derivedFinal, op); //trigger calculation of ob->derivedFinal somehow

	//no points, cant do anything
	if (totpoint == 0) {
		MEM_freeN(points);
		points = NULL;
		return 0;
	}

	if (point_source & eOwnVerts)
	{
		//make container a little bigger ? or some noise ?
		if (!use_boolean) {
			theta = 0.0001f;
		}
	}
	container = container_new(min[0]-theta, max[0]+theta, min[1]-theta, max[1]+theta, min[2]-theta, max[2]+theta,
							  n_size, n_size, n_size, FALSE, FALSE, FALSE, totpoint);

	p_order = particle_order_new();
	for (p = 0; p < totpoint; p++)
	{
		copy_v3_v3(co, points[p]);
		container_put(container, p_order, p, co[0], co[1], co[2]);
	}
	
	l_order = loop_order_new(container, p_order);

	//TODO: write results to temp file, ensure using the systems temp dir...
	// this prints out vertex positions and face indexes
	file = "test.out";
	path = MEM_mallocN(((strlen(BLI_temporary_dir()) + strlen(file) + 2) * sizeof(char)), "path");
	path = strcpy(path, BLI_temporary_dir());
	fullpath = strcat(path, file);
	fp = fopen(fullpath, "w+");

	//this triggers computation of the voronoi cells and produces the desired data:
	//see voro++ homepage for detailed description
	//TODO: insert link here

	// %P global vertex coordinates of voronoi vertices
	// v  the vertex -> face delimiter
	// %t the indexes to the cell vertices, describes which vertices build each face
	// f  the face -> centroid section delimiter
	// %C the centroid of the voronoi cell, used to find nearest particle in createCellpa
	// %n the neighbor indexes
	// n the neighbor delimiter

	//%i the particle index
	container_print_custom(l_order, container, "%i %P v %t f %C %n n", fp);
	fflush(fp);
	rewind(fp);
	
	if (points)
	{
		int t;

		for (t = 0; t < totpoint; t++) {
			MEM_freeN(points[t]);
			points[t] = NULL;
		}
		MEM_freeN(points);
		points = NULL;
	}
	
	if (totpoint == 0)
		return 0;

	while(feof(fp) == 0)
	{
		//printf("Reading line...\n");
		int *real_indexes = MEM_mallocN(sizeof(int*), "real_indexes");
		int len_real_indexes = 0;
		int pid;
		
		bmtemp = BM_mesh_create(&bm_mesh_chunksize_default);
		tempvert = MEM_mallocN(sizeof(BMVert*), "tempvert");
		tempvert_index = 0;
		
		// Read in the cell data, each line in the output file represents a voronoi cell
		fscanf(fp, "%d ", &pid);
		
		if (feof(fp) == 0) {
			
			BLI_ghash_insert(*pid_to_index, pid, objcount);
			*n = MEM_reallocN(*n, sizeof(neighborhood*) * (objcount+1));
			(*n)[objcount] = MEM_mallocN(sizeof(neighborhood), "neighbor");
			(*n)[objcount]->pid = pid;
			(*n)[objcount]->neighborcount = 0;
		}
		
		while (1)
		{
			// Read in the vertex coordinates of the cell vertices
			read = fscanf(fp, "(%f,%f,%f) ", &vco[0], &vco[1], &vco[2]);
			if (read < 3) break;

			//back to object space
			invert_m4_m4(imat, ob->obmat);
			mul_v3_m4v3(vco, imat, vco);

			tempvert = MEM_reallocN(tempvert, sizeof(BMVert*) * (tempvert_index + 1));
			vert = BM_vert_create(bmtemp, vco, NULL, 0);
			tempvert[tempvert_index] = vert;
			real_indexes = MEM_reallocN(real_indexes, sizeof(int*) * (len_real_indexes+1));
			real_indexes[len_real_indexes] = tempvert_index;

			if (tempvert_index > 0){
				int i;
				//compare all verts of array
				for (i = 0; i < tempvert_index; i++) {
					BMVert* last = tempvert[i];
					real_indexes = MEM_reallocN(real_indexes, sizeof(int*) * (len_real_indexes+1));

					if (compare_v3v3(last->co, vert->co, 1.0e-09)) {
						printf("Verts: (%f, %f, %f), (%f, %f, %f) %d %d\n",
							   last->co[0], last->co[1], last->co[2],
							   vert->co[0], vert->co[1], vert->co[2], i, tempvert_index);
						BM_vert_kill(bmtemp, vert);
						//MEM_freeN(vert);
						tempvert = MEM_reallocN(tempvert, sizeof(BMVert*) * (tempvert_index));
						real_indexes[len_real_indexes] = i;
						tempvert_index--;
						break;
					}
				}
			}

			len_real_indexes++;
			tempvert_index++;
		}

		faceverts = MEM_mallocN(sizeof(BMVert*), "faceverts");
		faceedges = MEM_mallocN(sizeof(BMEdge*), "faceedges");

		face_index = 0;
		edge_index = 0;

		while(1)
		{
			//printf ("Reading faces...\n");
			c = fgetc(fp);
			if (isdigit(c)) //maybe atoi !! (ascii value to int ?)
			{
				int double_vert = FALSE;

				//put/seek back and better do fscanf !
				fseek(fp, -sizeof(char), SEEK_CUR);
				facevert_index = 0;
				fscanf(fp, "%d", &facevert_index);
				faceverts = MEM_reallocN(faceverts, (face_index + 1) * sizeof(BMVert*));
				// find vertices for each face, store indexes here
				faceverts[face_index] = tempvert[real_indexes[facevert_index]];//emd->cells->data[emd->cells->count].vertices[facevert_index];

				if (face_index > 0) {
					//argh, need to determine edges manually...
					int i;
					for (i = 0; i < face_index; i++) {
						if (faceverts[face_index] == faceverts[i])
						{
							//avoid 2 equal verts in an edge or a face
							faceverts = MEM_reallocN(faceverts, (face_index) * sizeof(BMVert*));
							face_index--;
							double_vert = TRUE;
							break;
						}
					}
					if (!double_vert)
					{
						faceedges = MEM_reallocN(faceedges, (edge_index + 1) * sizeof(BMEdge*));
						//printf("Indexes: %d %d %d %p \n", face_index, facevert_index, real_indexes[facevert_index], faceverts[face_index]);
						edge = BM_edge_create(bmtemp, faceverts[face_index], faceverts[face_index-1], NULL, 0);
						faceedges[edge_index] = edge;
						edge_index++;
					}
				}

				face_index++;

			}
			else if ( c == ')') {
				float area;
				//Mesh *me = (Mesh*)ob->data;
				//end of face tuple, can create face now, but before create last edge to close the circle
				if (faceverts[0] != faceverts[face_index-1]) {
					faceedges = MEM_reallocN(faceedges, (edge_index + 1) * sizeof(BMEdge*));
					edge = BM_edge_create(bmtemp, faceverts[0], faceverts[face_index-1], NULL, 0);
					faceedges[edge_index] = edge;
				}
				else
				{
					face_index--;
				}

				face = BM_face_create(bmtemp, faceverts, faceedges, face_index, NULL, 0);

				if (face != NULL)
					area = BM_face_calc_area(face);
				//printf("Face Area: %f\n", area);

				if ((face != NULL) && ((area < 0.000005f) || (face_index < 3))) {
					//remove degenerate faces
					int i;
					BM_face_kill(bmtemp, face);
					face = NULL;

					for (i = 0; i < edge_index; i++) {
						BM_edge_kill(bmtemp, faceedges[i]);
					}

					for (i = 0; i < vert_index; i++) {
						BM_vert_kill(bmtemp, faceverts[i]);
					}
				}

				if (face != NULL) {
					BM_face_normal_flip(bmtemp, face);
				}

				edge_index = 0;
				face_index = 0;
			}
			else if ((c == 'f') || (feof(fp) != 0))
			{
				if (c == 'f')
				{
					//Intersection, use elements from temporary per-cell bmeshes and write to global bmesh, which
					//is passed around and whose vertices are manipulated directly.
					int mat_index = 0;
					//MPoly* mp; //hmm, might need this as well for each object too
					//MLoop* ml;
					boolresult = NULL;
					
					dm = CDDM_from_bmesh(bmtemp, TRUE);
					//printf(" %d Faces missing \n", (bmtemp->totface - dm->numPolyData));
					BM_mesh_free(bmtemp);
					bmtemp = NULL;

					if (use_boolean)
					{
						DM_ensure_tessface(derivedData);
						DM_ensure_tessface(dm);
						CDDM_calc_edges_tessface(dm);
						CDDM_tessfaces_to_faces(dm);
						CDDM_calc_normals(dm);

						//put temp bmesh to temp object ? Necessary ? Seems so.
						//TODO: maybe get along without temp object ?
						if (!tempOb)
						{
							tempOb = BKE_object_add_only_object(G.main, OB_MESH, "Intersect");
							//emd->tempOb = BKE_object_add(emd->modifier.scene, OB_MESH);
						}

						if (!tempOb->data)
						{
							tempOb->data = BKE_object_obdata_add_from_type(G.main, OB_MESH);
							//object_add_material_slot(emd->tempOb);
						}


						//assign inner material to temp Object
						if (inner_material)
						{
							//assign inner material as secondary mat to ob if not there already
							mat_index = find_material_index(ob, inner_material);
							if (mat_index == 0)
							{
								object_add_material_slot(ob);
								assign_material(ob, inner_material, ob->totcol, BKE_MAT_ASSIGN_OBDATA);
							}
							
							//shard gets inner material, maybe assign to all faces as well (in case this does not happen automatically
							assign_material(tempOb, inner_material, 1, BKE_MAT_ASSIGN_OBDATA);
						}
						
						DM_to_mesh(dm, tempOb->data, tempOb, 0);
						copy_m4_m4(tempOb->obmat, ob->obmat);
						
						boolresult = NewBooleanDerivedMesh(dm, tempOb, derivedData, ob, eBooleanModifierOp_Intersect);
						
						//if boolean fails, return original mesh, emit a warning
						if (!boolresult)
						{
							boolresult = dm;
							printf("Boolean Operation failed, using original mesh !\n");
						}
						else
						{

							DM_ensure_tessface(boolresult);
							CDDM_calc_edges_tessface(boolresult);
							CDDM_tessfaces_to_faces(boolresult);
							CDDM_calc_normals(boolresult);
						}
					}
					else
					{
						boolresult = dm;
						DM_ensure_tessface(boolresult);
						CDDM_calc_edges_tessface(boolresult);
						CDDM_tessfaces_to_faces(boolresult);
						CDDM_calc_normals(boolresult);
					}
					
					if (dm != boolresult) {
						DM_release(dm);
						MEM_freeN(dm);
						dm = NULL;
					}
					
					bm = DM_to_bmesh(boolresult, true);
					DM_release(boolresult);
					MEM_freeN(boolresult);
					boolresult = NULL;
				}

				edge_index = 0;
				face_index = 0;
				break;
			}
		}

		//read centroid, set origin of object there
		fscanf(fp, " %f %f %f ", &centroid[0], &centroid[1], &centroid[2]);

		//read neighbor id list - its per cell
		if (feof(fp) == 0)
		{
			(*n)[objcount]->neighbor_pids = MEM_mallocN(sizeof(int), "neighbor_pids");
		}
		neighbor_index = 0;
		while (feof(fp) == 0) {
			c = fgetc(fp);
			if (c != 'n') // end mark, cant use isdigit because negative numbers possible -> -
			{
				int n_index;
				//seek back one char,
				fseek(fp, -sizeof(char), SEEK_CUR);
				fscanf(fp, "%d ", &n_index);
				
				if (n_index > 0)
				{
					(*n)[objcount]->neighbor_pids = MEM_reallocN((*n)[objcount]->neighbor_pids, sizeof(int)*(neighbor_index+1));
					(*n)[objcount]->neighbor_pids[neighbor_index] = n_index;
					(*n)[objcount]->neighborcount++;
					neighbor_index++;
				}
			}
			else
			{
				//fseek(fp, -sizeof(char), SEEK_CUR);
				break;
			}
		}
		
		//skip newline
		if (feof(fp) == 0)
		{
			Mesh* me;
			int i;
#ifdef _WIN32
			//skip \r\n
			fseek(fp, 2*sizeof(char), SEEK_CUR);
#else
			//skip \n
			fseek(fp, sizeof(char), SEEK_CUR);
#endif
			BMO_op_callf(bm,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "recalc_face_normals faces=%af use_face_tag=%b", BM_FACES_OF_MESH, false);
			BM_mesh_normals_update(bm);
			BMO_op_callf(bm,(BMO_FLAG_DEFAULTS & ~BMO_FLAG_RESPECT_HIDE), "dissolve_limit edges=%ae verts=%av angle_limit=%f use_dissolve_boundaries=%b",
						BM_EDGES_OF_MESH, BM_VERTS_OF_MESH, 0.087f, false);
			
			//create objects and put them into listbase here
			o = BKE_object_add(G.main, scene, OB_MESH);
			BM_mesh_bm_to_me(bm, o->data, false);
			BM_mesh_free(bm);
			bm = NULL;
			
			me = (Mesh*)o->data;
			copy_m4_m4(o->obmat, mat);
			mul_m4_v3(o->obmat, centroid);
			for (i = 0; i < me->totvert; i++)
			{
				mul_m4_v3(o->obmat, me->mvert[i].co);
				sub_v3_v3(me->mvert[i].co, centroid);
			}
			
			//recenter... to centroid ?
			add_v3_v3(o->loc, centroid);
			
			*shards = MEM_reallocN(*shards, sizeof(Object*) * (objcount+1));
			(*shards)[objcount] = o;
			objcount++;
		}
		else
		{
			//EOF reached, free last vertco/vertices
			//MEM_freeN(emd->cells->data[emd->cells->count].neighbor_ids);
			//emd->cells->data[emd->cells->count].neighbor_ids = NULL;
		}

		vert_index = 0;
		edg_index = 0;
		fac_index = 0;

		MEM_freeN(tempvert);
		if (bmtemp) BM_mesh_free(bmtemp);
		MEM_freeN(faceverts);
		MEM_freeN(faceedges);

		MEM_freeN(real_indexes);
		len_real_indexes = 0;
	}

	fclose(fp);
	MEM_freeN(path);
	
	//remove tempOb here
	if (tempOb != NULL)
	{
		BKE_libblock_free_us(&(G.main->object), tempOb);
		BKE_object_unlink(tempOb);
		BKE_object_free(tempOb);
		tempOb = NULL;
	}
	
	printf("%d cells missing\n", totpoint - objcount); //use totpoint here
	
	return objcount;
}

void convertTessFaceToLoopPoly(Object** shards, int count)
{
	int i, j;
	for (i = 0; i < count; i++) {
		MTFace* mtface = MEM_mallocN(sizeof(MTFace), "mtface");
		MTexPoly* mtps = MEM_mallocN(sizeof(MTexPoly), "mtps"), mtp;
		MLoopUV* mluvs = MEM_mallocN(sizeof(MLoopUV), "mluvs"), mluv;
		MTFace* mtf = NULL;
		MTFace tf;
		MLoop* mla = NULL, *ml;
		MPoly* mpa = NULL, *mp;
		int f_index = 0, f = 0;
		int ml_index = 0;
		
		DerivedMesh* d = CDDM_from_mesh(shards[i]->data, shards[i]);
		
		DM_ensure_tessface(d);
		CDDM_calc_edges_tessface(d);
		CDDM_tessfaces_to_faces(d);
		CDDM_calc_normals(d);

		//hope this merges the MTFace data... successfully...
		mtf = DM_get_tessface_data_layer(d, CD_MTFACE);
		if (!mtf)
		{
			//argh, something went wrong, data will be missing...
			MEM_freeN(mtface);
			MEM_freeN(mtps);
			MEM_freeN(mluvs);
			mtface = NULL;
			
			DM_to_mesh(d, shards[i]->data, shards[i], 0);
			DM_release(d);
			MEM_freeN(d);
			d = NULL;
			break;
		}

		mpa = d->getPolyArray(d);
		mla = d->getLoopArray(d);

		for (f = 0; f < d->numTessFaceData; f++)
		{
			mtface = MEM_reallocN(mtface, sizeof(MTFace) * (f_index+1));
			tf = mtf[f];

			mtps = MEM_reallocN(mtps, sizeof(MTexPoly) * (f_index+1));
			mtp.tpage = tf.tpage;
			mtp.flag = tf.flag;
			mtp.mode = tf.mode;
			mtp.tile = tf.tile;
			mtp.transp = tf.transp;
			mtps[f_index] = mtp;

			//assume facecount = polycount and faces = polys since only tris and quads available
			//because of boolean

			mp = mpa + f; // get Polygon according to face index
			mluvs = MEM_reallocN(mluvs, sizeof(MLoopUV)* (mp->totloop + ml_index));

			for (j = mp->loopstart; j < mp->loopstart + mp->totloop; j++)
			{
				//assume vertindex = uvindex, sigh, is that so ?, and no more than 4 loops !
				ml = mla + j;
				mluv.uv[0] = tf.uv[j-mp->loopstart][0];
				mluv.uv[1] = tf.uv[j-mp->loopstart][1];
				mluvs[ml_index] = mluv;
				ml_index++;
			}

			mtface[f_index] = tf;
			f_index++;
		}
		
		if (mtface)
		{
			CustomData *fdata, *pdata, *ldata;
			fdata = &d->faceData;
			ldata = &d->loopData;
			pdata = &d->polyData;
	
			CustomData_add_layer(fdata, CD_MTFACE , CD_DUPLICATE, mtface, f_index);
			CustomData_add_layer(pdata, CD_MTEXPOLY, CD_DUPLICATE, mtps, f_index);
			CustomData_add_layer(ldata, CD_MLOOPUV, CD_DUPLICATE, mluvs, ml_index);
			
			MEM_freeN(mtface);
			MEM_freeN(mtps);
			MEM_freeN(mluvs);
			
			DM_to_mesh(d, shards[i]->data, shards[i], 0);
			DM_release(d);
			MEM_freeN(d);
			d = NULL;
		}
	}
}

Group* getGroup(const char* name) 
{
	ID *id;
	
	for (id = G.main->group.first; id; id = id->next)
	{
		int len = strlen(id->name);
		char idname[64];
		strncpy(idname, &id->name[2], len-2);
		idname[len-2] = '\0';
		
		if (strcmp(idname, name) == 0)
		{
			return (Group*)id;
		}
	}
	
	return BKE_group_add(G.main, name);
}

#define NAMELEN 64 //namebased, yuck... better dont try longer names.... NOT happy with this, but how to find according group else ? cant store it anywhere
int object_fracture_exec(bContext *C, wmOperator *op)
{
	//go through all selected objects !
	Scene* scene = CTX_data_scene(C);
	
	//needed for correctly update DAG after delete
	Main *bmain = CTX_data_main(C);
	wmWindowManager *wm = CTX_wm_manager(C);
	wmWindow *win;
	
	float imat[4][4], oldobmat[4][4];
	int use_boolean = RNA_boolean_get(op->ptr, "use_boolean");
	int solver_iterations = RNA_int_get(op->ptr, "iterations");
	int breaking_threshold = RNA_float_get(op->ptr, "threshold");
	

#ifdef WITH_MOD_VORONOI
	
	CTX_DATA_BEGIN(C, Object *, ob, selected_objects) {
		if (ob->type == OB_MESH)
		{
			GroupObject* go;
			char name[NAMELEN];
			int i,j, shardcount = 0;
			neighborhood** n = MEM_mallocN(sizeof(neighborhood*), "neighborhood");
			GHash* pid_to_index = BLI_ghash_int_new("pid_to_index");
			Object** shards = MEM_mallocN(sizeof(Object*), "shards");
			Group* rbos = getGroup(ob->id.name);
			Group* cons;
			Group* combined;
			
			strncpy(name, ob->id.name, NAMELEN);
			strncat(name, "_con", NAMELEN);
			cons = getGroup(name);
			
			strncpy(name, ob->id.name, NAMELEN);
			strncat(name, "_com", NAMELEN);
			combined = getGroup(name);
			
			
			for (go = cons->gobject.first; go; go = go->next)
			{
				Base *bas = BKE_scene_base_find(scene, go->ob);
				ED_rigidbody_constraint_remove(scene, go->ob);
				BKE_group_object_unlink(cons, go->ob, scene, NULL);
				BKE_group_object_unlink(combined, go->ob, scene, NULL);
				ED_base_object_free_and_unlink(G.main, scene, bas);
			}
			
			//clean up old objects, if desired (add user option)
			for (go = rbos->gobject.first; go; go = go->next)
			{
				Base *bas = BKE_scene_base_find(scene, go->ob);
				ED_rigidbody_object_remove(scene, go->ob);
				BKE_group_object_unlink(rbos, go->ob, scene, NULL);
				BKE_group_object_unlink(combined, go->ob, scene, NULL);
				ED_base_object_free_and_unlink(G.main, scene, bas);
			}
			
			
			/* delete has to handle all open scenes */ //copied from delete operator
			flag_listbase_ids(&bmain->scene, LIB_DOIT, 1);
			for (win = wm->windows.first; win; win = win->next) {
				scene = win->screen->scene;
				
				if (scene->id.flag & LIB_DOIT) {
					scene->id.flag &= ~LIB_DOIT;
					
					DAG_relations_tag_update(bmain);
		
					WM_event_add_notifier(C, NC_SCENE | ND_OB_ACTIVE, scene);
					WM_event_add_notifier(C, NC_SCENE | ND_LAYER_CONTENT, scene);
				}
			}
			
			invert_m4_m4(imat, ob->obmat);
			copy_m4_m4(oldobmat, ob->obmat);
			mul_m4_m4m4(ob->obmat, imat, ob->obmat); //neutralize obmat due to rotation problem with container
			shardcount = fractureToCells(ob, oldobmat, op, scene, &shards, &n, &pid_to_index);
			copy_m4_m4(ob->obmat, oldobmat); // restore obmat
			
			if (use_boolean) // get options from op now, do this per object as well
			{
				convertTessFaceToLoopPoly(shards, shardcount);
			}
			//build (inner, inactive) constraints with neighborhood indexes (rmd-idmap ?) maybe this was messed up ?
			for (i = 0; i < shardcount; i++)
			{
				Object* ob1 = shards[i];
				BKE_group_object_add(rbos, ob1, scene, NULL);
				BKE_group_object_add(combined, ob1, scene, NULL);
				
				ED_rigidbody_object_add(scene, ob1, RBO_TYPE_ACTIVE, op->reports);
				for (j = 0; j < n[i]->neighborcount; j++)
				{
					Object* con;
					float loc[3];
					bool con_found = false;
					GroupObject* go;
					
					int pid = n[i]->neighbor_pids[j];
					int index = BLI_ghash_lookup(pid_to_index, pid);
					Object* ob2 = shards[index];
					ED_rigidbody_object_add(scene, ob2, RBO_TYPE_ACTIVE, op->reports);
					//add testwise constraints to check correctness of neighborhood info
					
					//avoid constraints in both directions
					for (go = cons->gobject.first; go; go = go->next)
					{
						RigidBodyCon *rbc = go->ob->rigidbody_constraint;
						if ((rbc != NULL) && ( ((rbc->ob1 == ob1) && (rbc->ob2 == ob2)) ||
							((rbc->ob2 == ob1) && (rbc->ob1 == ob2)))){
							con_found = true;
							break;
						}
					}
					if (con_found == false)
					{
						con = BKE_object_add(G.main, scene, OB_EMPTY);
						BKE_group_object_add(cons, con, scene, NULL);
						BKE_group_object_add(combined, con, scene, NULL);
						//add_v3_v3v3(loc, ob1->loc, ob2->loc);
						//mul_v3_fl(loc, 0.5f);
						
						ED_rigidbody_constraint_add(scene, con, RBC_TYPE_FIXED, op->reports);
						con->rigidbody_constraint->ob1 = ob1;
						con->rigidbody_constraint->ob2 = ob2;
						con->rigidbody_constraint->flag |= RBC_FLAG_USE_BREAKING;
						con->rigidbody_constraint->breaking_threshold = breaking_threshold;
						con->rigidbody_constraint->num_solver_iterations = solver_iterations;
						
						//parent and hide by default
						//con->parent = ob1;
						//sub_v3_v3(loc, ob1->loc);
						copy_v3_v3(con->loc, ob1->loc);
						//con->restrictflag |= OB_RESTRICT_VIEW;
						
					}
				}
				MEM_freeN(n[i]->neighbor_pids);
				MEM_freeN(n[i]);
				n[i] = NULL;
			}
			
			//or how to alloc/free this correctly ??
			BLI_ghash_free(pid_to_index, NULL, NULL);
			MEM_freeN(n);
			n = NULL;
			MEM_freeN(shards);
			shards = NULL;
		}
	}
	CTX_DATA_END;
	
	WM_event_add_notifier(C, NC_OBJECT | ND_TRANSFORM, NULL);
	WM_event_add_notifier(C, NC_OBJECT | ND_POINTCACHE, NULL);
	
	return OPERATOR_FINISHED;
#else
	//Operator warning...
	return OPERATOR_CANCELLED;
#endif
}

/*void object_fracture_ui(bContext* C, wmOperator* op)
{
	uiLayout* layout = op->layout;
	PointerRNA group, group_key, mat, mat_key;
	Main* main = CTX_data_main(C);
	RNA_pointer_create(NULL, op->type->srna, op->properties, &group);
	RNA_id_pointer_create((ID *)main->group.first, &group_key);
	
	uiItemPointerR(layout, &group, "extra_group", &group_key, "group", "", ICON_SIZE_ICON);
}*/


void OBJECT_OT_fracture(wmOperatorType *ot)
{
	PropertyRNA *prop;
	
	static EnumPropertyItem prop_point_source_items[] = {
		{eOwnParticles, "OWN_PARTICLES", 0, "Own Particles", "Use own particles as point cloud"},
		{eOwnVerts, "OWN_VERTS", 0, "Own Vertices", "Use own vertices as point cloud"},
		{eExtraParticles, "EXTRA_PARTICLES", 0, "Extra Particles", "Use particles of group objects as point cloud"},
		{eExtraVerts, "EXTRA_VERTS", 0, "Extra Vertices", "Use vertices of group objects as point cloud"},
		{eGreasePencil, "GREASE_PENCIL", 0, "Grease Pencil", "Use grease pencil points as point cloud"},
		{0, NULL, 0, NULL, NULL}
	};
	
	/* identifiers */
	ot->idname = "OBJECT_OT_fracture";
	ot->name = "Fracture Object";
	ot->description = "Fracture Object to shard objects";

	/* callbacks */
	ot->exec = object_fracture_exec;
	ot->poll = ED_operator_object_active_editable;
	//ot->ui = object_fracture_ui;
	
	/* flags */
	ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

	/* properties */
	prop = RNA_def_enum(ot->srna, "point_source", prop_point_source_items, eOwnParticles, "Point Source", "Source of point cloud");
	RNA_def_property_flag(prop, PROP_ENUM_FLAG);

	RNA_def_boolean(ot->srna, "use_boolean", FALSE, "Use Boolean Intersection", "Intersect shards with original object shape");
	
	/*prop = RNA_def_pointer(ot->srna, "inner_material", "Material", "Inner Material", "Material to be applied on inner faces (boolean only)");
	RNA_def_property_flag(prop, PROP_EDITABLE);

	prop = RNA_def_pointer(ot->srna, "extra_group", "Group", "Extra Group", "Group of helper objects being used for creating pointcloud");
	RNA_def_property_flag(prop, PROP_EDITABLE);*/
	
	RNA_def_float(ot->srna, "noise", 0.0f, 0.0f, 1.0f, "Noise", "Noise to apply over pointcloud", 0.0f, 1.0f);
	RNA_def_int(ot->srna, "percentage", 100, 0, 100, "Percentage", "Percentage of point to actually use for fracture", 0, 100);
	
	RNA_def_float(ot->srna, "threshold", 10.0f, 0.0f, FLT_MAX, "Breaking threshold", "Noise to apply over pointcloud", 0.0f, FLT_MAX);
	RNA_def_int(ot->srna, "iterations", 30, 0, INT_MAX , "Solver Iterations", "Percentage of point to actually use for fracture", 0, INT_MAX);
}
