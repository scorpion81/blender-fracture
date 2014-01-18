#include "BKE_fracture.h"
#include "MEM_guardedalloc.h"
#include "BLI_mempool.h"
#include "BLI_utildefines.h"
#include "BLI_math_vector.h"
#include "BKE_mesh.h"
#include "RBI_api.h"

static float point[3]; //hrrrrm, need this in sorting algorithm as part of the key, cant pass it directly

/* iterator functions for efficient looping over shards */
ShardIterator* BKE_shards_begin(FracMesh *mesh) {
	ShardIterator *iter = MEM_mallocN(sizeof(ShardIterator), __func__);
	iter->frac_mesh = mesh;
	iter->current = 0;
	
	return iter;
}

ShardIterator* BKE_shards_next(ShardIterator *iter) {
	iter->current++;
	return iter;
}

bool BKE_shards_valid(ShardIterator* iter) {
	return iter->current < iter->frac_mesh->shard_count;
}

void BKE_shards_end(ShardIterator* iter) {
	MEM_freeN(iter);
	iter = NULL;
}

/* access shard during loop */
Shard* BKE_shard_by_iterator(ShardIterator *iter) {
	if (BKE_shards_valid(iter)) {
		return iter->frac_mesh->shard_map[iter->current];
	}
	
	return NULL;
}

/*access shard directly by index / id*/
Shard *BKE_shard_by_id(FracMesh* mesh, ShardID id) {
	if ((index < mesh->shard_count) && (id >= 0))
	{
		return mesh->shard_map[id];
	}
	
	return NULL;
}

Shard *BKE_shard_by_position(FracMesh* mesh, float position[3]) {
	//find centroid via kdtree, should be fast
	return NULL;
	
}

void BKE_get_shard_geometry(FracMesh* mesh, ShardID id, MVert** vert, int *totvert)
{
	Shard* shard = BKE_shard_by_id(mesh, id);
	if (shard != NULL) {
		vert = shard->vertices;
		totvert = &shard->vertex_count;
	}
}

void BKE_get_shard_bbox(FracMesh* mesh, ShardID id, BoundBox* bbox)
{
	Shard* shard = BKE_shard_by_id(mesh, id);
	if (shard != NULL) {
		bbox = shard->bb;
	}
}

/*FracMesh* BKE_dm_to_fracmesh(DerivedMesh* dm) {
	FracMesh* fmesh = MEM_mallocN(sizeof(FracMesh), __func__);
	fmesh->fractured_mesh = dm;
	fmesh->shard_list = MEM_mallocN(sizeof(ShardList), __func__);
	fmesh->shard_list->shards = MEM_mallocN(sizeof(Shard) * 100, __func__); //allocate in chunks, better use proper blender functions for this
	fmesh->shard_list->shard_count = 1; //the original "shard"
	fmesh->shard_list->shards[0].vertex_count = dm->getNumVerts(dm);
	fmesh->shard_list->shards[0].vertices = dm->getVertArray(dm);
}*/

ShardList fracture_shard_by_points(FracMesh* mesh, ShardID id, PointCloud* pointcloud) {
	//do cell fracture code here on shardbbox, AND intersect with boolean, hmm would need a temp object for this ? or bisect with cell planes.
	//lets test bisect but this is a bmesh operator, so need to convert, do bisection and convert back, slow. and cellfrac also uses bmesh, hmm
	//so better keep this around
	return NULL;
}

/*static int point_cmp(void *v1, void *v2)
{
	//const int64_t x1 = *(const int64_t *)v1;
	//const int64_t x2 = *(const int64_t *)v2;
	float x1[3] = (float[])v1;
	float x2[3] = (float[])v2;
	float l1 = len_squared_v3v3(x1, point);
	float l2 = len_squared_v3v3(x2, point);
	
	if (l1 > l2) {
		return 1;
	}
	else if (l1 < l2) {
		return -1;
	}

	return 0;
}*/


/* note: this function could be optimized by some spatial structure */
/*void points_in_planes(float (*planes)[4], unsigned int planes_len, float (*verts)[3], unsigned int* verts_len, int *plane_indices, unsigned int* used_planes_len)
{
	const float eps = 0.0001f;
	const unsigned int len = (unsigned int)planes_len;
	unsigned int i, j, k, l;

	float n1n2[3], n2n3[3], n3n1[3];
	float potentialVertex[3];
	char *planes_used = MEM_mallocN(sizeof(char) * len, __func__);
	memset(planes_used, 0, sizeof(char) * len);
	
	verts = MEM_mallocN(sizeof(float) * 3, __func__);
	plane_indices = MEM_mallocN(sizeof(int), __func__);
	used_planes_len = MEM_mallocN(sizeof(int), __func__);
	*used_planes_len = 0;
	
	verts_len = MEM_mallocN(sizeof(int), __func__);
	*verts_len = 0;

	for (i = 0; i < len; i++) {
		const float *N1 = planes[i];
		for (j = i + 1; j < len; j++) {
			const float *N2 = planes[j];
			cross_v3_v3v3(n1n2, N1, N2);
			if (len_squared_v3(n1n2) > eps) {
				for (k = j + 1; k < len; k++) {
					const float *N3 = planes[k];
					cross_v3_v3v3(n2n3, N2, N3);
					if (len_squared_v3(n2n3) > eps) {
						cross_v3_v3v3(n3n1, N3, N1);
						if (len_squared_v3(n3n1) > eps) {
							const float quotient = dot_v3v3(N1, n2n3);
							if (fabsf(quotient) > eps) {
								// potentialVertex = (n2n3 * N1[3] + n3n1 * N2[3] + n1n2 * N3[3]) * (-1.0 / quotient); 
								const float quotient_ninv = -1.0f / quotient;
								potentialVertex[0] = ((n2n3[0] * N1[3]) + (n3n1[0] * N2[3]) + (n1n2[0] * N3[3])) * quotient_ninv;
								potentialVertex[1] = ((n2n3[1] * N1[3]) + (n3n1[1] * N2[3]) + (n1n2[1] * N3[3])) * quotient_ninv;
								potentialVertex[2] = ((n2n3[2] * N1[3]) + (n3n1[2] * N2[3]) + (n1n2[2] * N3[3])) * quotient_ninv;
								for (l = 0; l < len; l++) {
									const float *NP = planes[l];
									if ((dot_v3v3(NP, potentialVertex) + NP[3]) > 0.000001f) {
										break;
									}
								}

								if (l == len) { /* ok *
									verts = MEM_reallocN(verts, sizeof(float) * 3 *(*verts_len)+1);
									verts[(*verts_len)] = potentialVertex;
									(*verts_len)++;
									planes_used[i] = planes_used[j] = planes_used[k] = true;
								}
							}
						}
					}
				}
			}
		}

		MEM_freeN(planes);

		/* now make a list of used planes *
		for (i = 0; i < len; i++) {
			if (planes_used[i]) {
				plane_indices = MEM_reallocN(plane_indices, sizeof((int) * (*plane_indices_len)+1));
				plane_indices[*plane_indices_len] = i;
				(*plane_indices)++;
			}
		}
	}
}*/

#ifdef WITH_VORO++
#endif


/*ShardList fracture_by_points(FracMesh *fmesh, PointCloud *pointcloud) {
	//find shards by positions in pointcloud ? and dont forget updating the kdtree as well...
	//iterate over pointcloud positions, find according shards and refracture 'em
	
	float convex_planes[6][4] = {{1, 0, 0, 0},
								 {-1, 0, 0, 0},
								 {0, 1, 0, 0},
								 {0, -1, 0, 0},
								 {0, 0, 1, 0},
								 {0, 0, -1, 0}};
	//float planes[6][4];
	float max = 100000000000.0;

	
	for (int i = 0; i < pointcloud->totpoints; i++) {
		float points_sorted[][3];
		Shard* shard = BKE_shard_by_position(fmesh, point);
		float xmin = ymin = zmin = FLT_MAX;
		float xmax = ymax = zmax = -FLT_MAX;
		float (*planes)[4]; // 6 planes at start, according to bbox, getting more...
		int len_planes = 6; 
		
		copy_v3v3(point, pointcloud->points[i]);
				
		for (int j = 0; j < shard->vertex_count; j++) {
			MVert *v = shard->vertices_cached[j];
			xmin = MIN2(v->co[0], xmin) - margin_bounds;
			ymin = MIN2(v->co[1], ymin) - margin_bounds;
			zmin = MIN2(v->co[2], zmin) - margin_bounds;
			
			xmax = MAX2(v->co[0], xmax) + margin_bounds;
			ymax = MAX2(v->co[1], ymax) + margin_bounds;
			zmax = MAX2(v->co[2], zmax) + margin_bounds;
			
		}
		
		convex_planes[0][3] = -xmax;
		convex_planes[1][3] = xmax;
		convex_planes[2][3] = -ymax;
		convex_planes[3][3] = ymin;
		convex_planes[4][3] = -zmax;
		convex_planes[5][3] = zmin;
		
		for (int p = 0; p < 6; p++) {
			planes[p] = convex_planes[p];
			planes[p][3] += dot_v3v3(planes[p], point);
		}
		
		qsort(points_sorted, 3, sizeof(float), point_compare);
		
		for (int k = 0; k < pointcloud->totpoints; k++) {
			float normal[3], nlen;
			float plane[4];
			
			float (*verts)[3];
			int *len_verts;
			int *plane_indices;
			int *len_plane_indices;
			
			sub_v3_v3v3(normal, points_sorted[k], point);
			nlen = len_v3(normal);
			
			if (nlen > max)
				break;
			
			copy_v3_v3(plane, normal);
			normalize_v3(plane);
			plane[3] = (-nlength / 2.0) + margin_cell;
			planes = MEM_reallocN(planes, sizeof(float) * 4 * (len_planes + 1));
			copy_v4_v4(planes[len_planes], plane);
			len_planes++;
			
			points_in_planes(planes, len_planes, verts, len_verts, plane_indices, len_plane_indices);
			
			
			if (len_verts == 0) {
				break;
			}
			
			if (len_planes != len_plane_indices) {
				
			}
	
	/*def points_as_bmesh_cells(verts,
	                          points,
	                          points_scale=None,
	                          margin_bounds=0.05,
	                          margin_cell=0.0):
	    from math import sqrt
	    import mathutils
	    from mathutils import Vector
	
	    cells = []
	
	    points_sorted_current = [p for p in points]
	    plane_indices = []
	    vertices = []
	
	    if points_scale is not None:
	        points_scale = tuple(points_scale)
	    if points_scale == (1.0, 1.0, 1.0):
	        points_scale = None
	
	    # there are many ways we could get planes - convex hull for eg
	    # but it ends up fastest if we just use bounding box
	    if 1:
	        xa = [v[0] for v in verts]
	        ya = [v[1] for v in verts]
	        za = [v[2] for v in verts]
	
	        xmin, xmax = min(xa) - margin_bounds, max(xa) + margin_bounds
	        ymin, ymax = min(ya) - margin_bounds, max(ya) + margin_bounds
	        zmin, zmax = min(za) - margin_bounds, max(za) + margin_bounds
	        convexPlanes = [
	            Vector((+1.0, 0.0, 0.0, -xmax)),
	            Vector((-1.0, 0.0, 0.0, +xmin)),
	            Vector((0.0, +1.0, 0.0, -ymax)),
	            Vector((0.0, -1.0, 0.0, +ymin)),
	            Vector((0.0, 0.0, +1.0, -zmax)),
	            Vector((0.0, 0.0, -1.0, +zmin)),
	            ]
	
	    for i, point_cell_current in enumerate(points):
	        planes = [None] * len(convexPlanes)
	        for j in range(len(convexPlanes)):
	            planes[j] = convexPlanes[j].copy()
	            planes[j][3] += planes[j].xyz.dot(point_cell_current)
	        distance_max = 10000000000.0  # a big value!
	
	        points_sorted_current.sort(key=lambda p: (p - point_cell_current).length_squared)
	
	        for j in range(1, len(points)):
	            normal = points_sorted_current[j] - point_cell_current
	            nlength = normal.length
	
	            if points_scale is not None:
	                normal_alt = normal.copy()
	                normal_alt.x *= points_scale[0]
	                normal_alt.y *= points_scale[1]
	                normal_alt.z *= points_scale[2]
	
	                # rotate plane to new distance
	                # should always be positive!! - but abs incase
	                scalar = normal_alt.normalized().dot(normal.normalized())
	                # assert(scalar >= 0.0)
	                nlength *= scalar
	                normal = normal_alt
	
	            if nlength > distance_max:
	                break
	
	            plane = normal.normalized()
	            plane.resize_4d()
	            plane[3] = (-nlength / 2.0) + margin_cell
	            planes.append(plane)
	
	            vertices[:], plane_indices[:] = mathutils.geometry.points_in_planes(planes)
	            if len(vertices) == 0:
	                break
	
	            if len(plane_indices) != len(planes):
	                planes[:] = [planes[k] for k in plane_indices]
	
	            # for comparisons use length_squared and delay
	            # converting to a real length until the end.
	            distance_max = 10000000000.0  # a big value!
	            for v in vertices:
	                distance = v.length_squared
	                if distance_max < distance:
	                    distance_max = distance
	            distance_max = sqrt(distance_max)  # make real length
	            distance_max *= 2.0
	
	        if len(vertices) == 0:
	            continue
	
	        cells.append((point_cell_current, vertices[:]))
	        del vertices[:]
	
	    return cells
			  
			  def cell_fracture_objects(scene, obj,
			                            source={'PARTICLE_OWN'},
			                            source_limit=0,
			                            source_noise=0.0,
			                            clean=True,
			                            # operator options
			                            use_smooth_faces=False,
			                            use_data_match=False,
			                            use_debug_points=False,
			                            margin=0.0,
			                            material_index=0,
			                            use_debug_redraw=False,
			                            cell_scale=(1.0, 1.0, 1.0),
			                            ):
			  
			      from . import fracture_cell_calc
			  
			      # -------------------------------------------------------------------------
			      # GET POINTS
			  
			      points = _points_from_object(obj, source)
			  
			      if not points:
			          # print using fallback
			          points = _points_from_object(obj, {'VERT_OWN'})
			  
			      if not points:
			          print("no points found")
			          return []
			  
			      # apply optional clamp
			      if source_limit != 0 and source_limit < len(points):
			          import random
			          random.shuffle(points)
			          points[source_limit:] = []
			  
			      # saddly we cant be sure there are no doubles
			      from mathutils import Vector
			      to_tuple = Vector.to_tuple
			      points = list({to_tuple(p, 4): p for p in points}.values())
			      del to_tuple
			      del Vector
			  
			      # end remove doubles
			      # ------------------
			  
			      if source_noise > 0.0:
			          from random import random
			          # boundbox approx of overall scale
			          from mathutils import Vector
			          matrix = obj.matrix_world.copy()
			          bb_world = [matrix * Vector(v) for v in obj.bound_box]
			          scalar = source_noise * ((bb_world[0] - bb_world[6]).length / 2.0)
			  
			          from mathutils.noise import random_unit_vector
			  
			          points[:] = [p + (random_unit_vector() * (scalar * random())) for p in points]
			  
			      if use_debug_points:
			          bm = bmesh.new()
			          for p in points:
			              bm.verts.new(p)
			          mesh_tmp = bpy.data.meshes.new(name="DebugPoints")
			          bm.to_mesh(mesh_tmp)
			          bm.free()
			          obj_tmp = bpy.data.objects.new(name=mesh_tmp.name, object_data=mesh_tmp)
			          scene.objects.link(obj_tmp)
			          del obj_tmp, mesh_tmp
			  
			      mesh = obj.data
			      matrix = obj.matrix_world.copy()
			      verts = [matrix * v.co for v in mesh.vertices]
			  
			      cells = fracture_cell_calc.points_as_bmesh_cells(verts,
			                                                       points,
			                                                       cell_scale,
			                                                       margin_cell=margin)
			  
			      # some hacks here :S
			      cell_name = obj.name + "_cell"
			  
			      objects = []
			  
			      for center_point, cell_points in cells:
			  
			          # ---------------------------------------------------------------------
			          # BMESH
			  
			          # create the convex hulls
			          bm = bmesh.new()
			  
			          # WORKAROUND FOR CONVEX HULL BUG/LIMIT
			          # XXX small noise
			          import random
			          def R():
			              return (random.random() - 0.5) * 0.001
			          # XXX small noise
			  
			          for i, co in enumerate(cell_points):
			  
			              # XXX small noise
			              co.x += R()
			              co.y += R()
			              co.z += R()
			              # XXX small noise
			  
			              bm_vert = bm.verts.new(co)
			  
			          import mathutils
			          bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.005)
			          try:
			              bmesh.ops.convex_hull(bm, input=bm.verts)
			          except RuntimeError:
			              import traceback
			              traceback.print_exc()
			  
			          if clean:
			              bm.normal_update()
			              try:
			                  bmesh.ops.dissolve_limit(bm, verts=bm.verts, angle_limit=0.001)
			              except RuntimeError:
			                  import traceback
			                  traceback.print_exc()
			  
			          if use_smooth_faces:
			              for bm_face in bm.faces:
			                  bm_face.smooth = True
			  
			          if material_index != 0:
			              for bm_face in bm.faces:
			                  bm_face.material_index = material_index
			  
			  
			          # ---------------------------------------------------------------------
			          # MESH
			          mesh_dst = bpy.data.meshes.new(name=cell_name)
			  
			          bm.to_mesh(mesh_dst)
			          bm.free()
			          del bm
			  
			          if use_data_match:
			              # match materials and data layers so boolean displays them
			              # currently only materials + data layers, could do others...
			              mesh_src = obj.data
			              for mat in mesh_src.materials:
			                  mesh_dst.materials.append(mat)
			              for lay_attr in ("vertex_colors", "uv_textures"):
			                  lay_src = getattr(mesh_src, lay_attr)
			                  lay_dst = getattr(mesh_dst, lay_attr)
			                  for key in lay_src.keys():
			                      lay_dst.new(name=key)
			  
			          # ---------------------------------------------------------------------
			          # OBJECT
			  
			          obj_cell = bpy.data.objects.new(name=cell_name, object_data=mesh_dst)
			          scene.objects.link(obj_cell)
			          # scene.objects.active = obj_cell
			          obj_cell.location = center_point
			  
			          objects.append(obj_cell)
			  
			          # support for object materials
			          if use_data_match:
			              for i in range(len(mesh_dst.materials)):
			                  slot_src = obj.material_slots[i]
			                  slot_dst = obj_cell.material_slots[i]
			  
			                  slot_dst.link = slot_src.link
			                  slot_dst.material = slot_src.material
			  
			          if use_debug_redraw:
			              scene.update()
			              _redraw_yasiamevil()
			  
			      scene.update()
			  
			      # move this elsewhere...
			      for obj_cell in objects:
			          game = obj_cell.game
			          game.physics_type = 'RIGID_BODY'
			          game.use_collision_bounds = True
			          game.collision_bounds_type = 'CONVEX_HULL'
			  
			      return objects*
	
}*/


