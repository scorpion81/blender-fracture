#include <stdio.h>
#include <stdlib.h>

#include "MEM_guardedalloc.h"

#include "BLI_math_vector.h"
#include "BLI_mempool.h"
#include "BLI_utildefines.h"
#include "BLI_path_util.h"

#include "DNA_fracture_types.h"
#include "DNA_group_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"

#include "BKE_DerivedMesh.h"
#include "BKE_cdderivedmesh.h"
#include "BKE_fracture.h"
#include "BKE_fracture_util.h"
#include "BKE_mesh.h"
#include "BKE_object.h"
#include "BKE_customdata.h"

#include "bmesh.h"

#include "RBI_api.h"

/*boolean support */
//#include "CSG_BooleanOps.h"

/* debug timing */
#define USE_DEBUG_TIMER

#ifdef USE_DEBUG_TIMER
#include "PIL_time.h"
#endif

#ifdef WITH_VORO
#include "../../../../extern/voro++/src/c_interface.hh"
#endif

//utility... bbox / centroid calc

/*prototypes*/
static void parse_stream(FILE *fp, int expected_shards, ShardID shard_id, FracMesh *fm, int algorithm, Object *obj, DerivedMesh *dm);
static Shard *parse_shard(FILE *fp);
static void parse_verts(FILE *fp, MVert *mvert, int totvert);
static void parse_polys(FILE *fp, MPoly *mpoly, int totpoly, int *r_totloop);
static void parse_loops(FILE *fp, MLoop *mloop, int totloop, MPoly *mpoly, int totpoly);
static void parse_neighbors(FILE *fp, int *neighbors, int totpoly);
static void add_shard(FracMesh *fm, Shard *s);

static void add_shard(FracMesh *fm, Shard *s)
{
	fm->shard_map = MEM_reallocN(fm->shard_map, sizeof(Shard*) * (fm->shard_count+1));
	fm->shard_map[fm->shard_count] = s;
	s->shard_id = fm->shard_count;
	fm->shard_count++;
}

/* parse the voro++ raw data */
static void parse_stream(FILE *fp, int expected_shards, ShardID parent_id, FracMesh *fm, int algorithm, Object* obj, DerivedMesh *dm)
{
	/*Parse voronoi raw data*/
	int i = 0;
	Shard* s = NULL, *p = BKE_shard_by_id(fm, parent_id, dm);
	float obmat[4][4]; /* use unit matrix for now */
	BMesh* bm_parent = NULL;
	DerivedMesh *dm_parent = NULL;

	p->flag = 0;
	p->flag |= SHARD_FRACTURED;
	unit_m4(obmat);

	// FOR NOW, delete OLD shard...
	/*s = fm->shard_map[0];
	MEM_freeN(s->mvert);
	MEM_freeN(s->mloop);
	MEM_freeN(s->mpoly);
	if (s->neighbor_ids)
		MEM_freeN(s->neighbor_ids);
	MEM_freeN(s);

	fm->shard_map[0] = NULL;
	fm->shard_count = 0;*/

	//while (feof(fp) != 0) //doesnt work with memory stream...
	/* TODO, might occur that we have LESS than expected shards, what then...*/

	if (algorithm == MOD_FRACTURE_BOOLEAN)
	{
		dm_parent = BKE_shard_create_dm(p, true);
	}
	else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL)
	{
#define MYTAG (1 << 6)
		//BMVert *v, *e, *f;
		//BMIter iter, eiter, fiter;
		//int index, eindex, findex;

		dm_parent = BKE_shard_create_dm(p, true);
		bm_parent = DM_to_bmesh(dm_parent, true);

		//mark all geometry...
		//BM_mesh_elem_hflag_enable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, MYTAG, false);
		//BM_mesh_elem_hflag_disable_all(bm_parent, BM_VERT | BM_EDGE | BM_FACE, BM_ELEM_SELECT, false);

		//create lookup tables
		//BM_mesh_elem_table_ensure(bm_parent, BM_VERT | BM_EDGE | BM_FACE);

		//create ORIGINDEX layer and fill with indexes
		/*CustomData_add_layer(&bm_parent->vdata, CD_ORIGINDEX, CD_CALLOC, NULL, bm_parent->totvert);
		CustomData_bmesh_init_pool(&bm_parent->vdata, bm_parent->totvert, BM_VERT);

		BM_ITER_MESH_INDEX(v, &iter, bm_parent, BM_VERTS_OF_MESH, index)
		{
			CustomData_bmesh_set_default(&bm_parent->vdata, &v->head.data);
			CustomData_bmesh_set(&bm_parent->vdata, v->head.data, CD_ORIGINDEX, &index);
		}*/

		//create ORIGINDEX layer and fill with indexes
		/*CustomData_add_layer(&bm_parent->edata, CD_ORIGINDEX, CD_CALLOC, NULL, bm_parent->totedge);
		CustomData_bmesh_init_pool(&bm_parent->edata, bm_parent->totedge, BM_EDGE);

		BM_ITER_MESH_INDEX(e, &eiter, bm_parent, BM_EDGES_OF_MESH, eindex)
		{
			CustomData_bmesh_set_default(&bm_parent->edata, &e->head.data);
			CustomData_bmesh_set(&bm_parent->edata, e->head.data, CD_ORIGINDEX, &eindex);
		}

		//create ORIGINDEX layer and fill with indexes
		CustomData_add_layer(&bm_parent->pdata, CD_ORIGINDEX, CD_CALLOC, NULL, bm_parent->totface);
		CustomData_bmesh_init_pool(&bm_parent->pdata, bm_parent->totface, BM_FACE);

		BM_ITER_MESH_INDEX(f, &fiter, bm_parent, BM_FACES_OF_MESH, findex)
		{
			CustomData_bmesh_set_default(&bm_parent->pdata, &f->head.data);
			CustomData_bmesh_set(&bm_parent->pdata, f->head.data, CD_ORIGINDEX, &findex);
		}*/

		dm_parent->needsFree = 1;
		dm_parent->release(dm_parent);
		dm_parent = NULL;
	}

	for (i = 0; i < expected_shards; i++)
	{
		//printf("Parsing shard: %d\n", i);
		s = parse_shard(fp);
		if (s != NULL)
		{
			s->parent_id = parent_id;
			s->flag = SHARD_INTACT;
		}
		/* XXX TODO, need object for material as well, or atleast a material index... */
		if (algorithm == MOD_FRACTURE_BOOLEAN)
		{
			s = BKE_fracture_shard_boolean(obj, dm_parent, s);
		}
		else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL)
		{
			printf("Bisecting cell %d...\n", i);
			s = BKE_fracture_shard_bisect(bm_parent, s, obmat, algorithm == MOD_FRACTURE_BISECT_FILL);
		}
		if (s != NULL)
		{
			s->parent_id = parent_id;
			s->flag = SHARD_INTACT;
			add_shard(fm, s);
		}
	}

	if (bm_parent != NULL)
	{
		BM_mesh_free(bm_parent);
		bm_parent = NULL;
	}

	if (dm_parent != NULL)
	{
		dm_parent->needsFree = 1;
		dm_parent->release(dm_parent);
		dm_parent = NULL;
	}

	if (parent_id == -1)
	{
		BKE_shard_free(p, true);
	}
}

static Shard* parse_shard(FILE *fp)
{
	Shard *s;
	MVert *mvert = NULL;
	MPoly *mpoly = NULL;
	MLoop *mloop = NULL;
	int *neighbors = NULL;
	int totpoly, totloop, totvert;
	float centr[3];
	int shard_id;

	fscanf(fp, "%d ", &shard_id);

	fscanf(fp, "%d ", &totvert);
	if (totvert > 0) {
		mvert = MEM_callocN(sizeof(MVert) * totvert, __func__);
		parse_verts(fp, mvert, totvert);
	}
	
	/* skip "v "*/
	fseek(fp, 2*sizeof(char), SEEK_CUR);
	
	fscanf(fp, "%d ", &totpoly);
	if (totpoly > 0) {
		mpoly = MEM_callocN(sizeof(MPoly) * totpoly, __func__);
		parse_polys(fp, mpoly, totpoly, &totloop);
	}
	else
		totloop = 0;
	
	if (totloop > 0) {
		mloop = MEM_callocN(sizeof(MLoop) * totloop, __func__);
		parse_loops(fp, mloop, totloop, mpoly, totpoly);
	}
	
	if (totpoly > 0) {
		neighbors = MEM_callocN(sizeof(int) * totpoly, __func__);
		parse_neighbors(fp, neighbors, totpoly);
	}
	
	/* skip "f "*/
	fseek(fp, 2*sizeof(char), SEEK_CUR);
	
	/* parse centroid */
	fscanf(fp, "%f %f %f ", &centr[0], &centr[1], &centr[2]);
	
	/* skip "c"*/
	fseek(fp, sizeof(char), SEEK_CUR);
	
	s = BKE_create_fracture_shard(mvert, mpoly, mloop, totvert, totpoly, totloop, false);

	s->neighbor_ids = neighbors;
	s->neighbor_count = totpoly;
	//s->shard_id = shard_id;
	copy_v3_v3(s->centroid, centr);
	
	/* if not at end of file yet, skip newlines */
	if (feof(fp) == 0) {
#ifdef _WIN32
		//skip \r\n
		fseek(fp, 2*sizeof(char), SEEK_CUR);
#else
		//skip \n
		fseek(fp, sizeof(char), SEEK_CUR);
#endif
	}

	return s;
}

static void parse_verts(FILE *fp, MVert *mvert, int totvert)
{
	int i;

	for (i = 0; i < totvert; i++) {
		float *co = mvert[i].co;
		fscanf(fp, "(%f,%f,%f) ", &co[0], &co[1], &co[2]);
	}
}

static void parse_polys(FILE *fp, MPoly *mpoly, int totpoly, int *r_totloop)
{
	int i;
	int totloop = 0;

	for (i = 0; i < totpoly; ++i) {
		int numloop;
		
		fscanf(fp, "%d ", &numloop);
		
		mpoly[i].loopstart = totloop;
		mpoly[i].totloop = numloop;
		
		totloop += numloop;
	}
	
	*r_totloop = totloop;
}

static void parse_loops(FILE *fp, MLoop *mloop, int UNUSED(totloop), MPoly *mpoly, int totpoly)
{
	int i, k;

	for (i = 0; i < totpoly; ++i) {
		int loopstart = mpoly[i].loopstart;
		int numloop = mpoly[i].totloop;
		
		/* skip "(" */
		fseek(fp, sizeof(char), SEEK_CUR);
		
		for (k = 0; k < numloop; ++k) {
			int index;
			
			fscanf(fp, "%d", &index);
			
			/* note: invert vertex order here,
			 * otherwise normals are pointing inward
			 */
			mloop[loopstart + (numloop - 1) - k].v = index;
			
			/* skip "," or ")" */
			fseek(fp, sizeof(char), SEEK_CUR);
		}
		
		/* skip " " */
		fseek(fp, sizeof(char), SEEK_CUR);
	}
}

static void parse_neighbors(FILE* fp, int *neighbors, int totpoly)
{
	int i;

	for (i = 0; i < totpoly; i++) {
		int n;
		fscanf(fp, "%d ", &n);
		neighbors[i] = n;
	}
}

Shard* BKE_custom_data_to_shard(Shard* s, DerivedMesh* dm)
{
	CustomData_copy(&dm->vertData, &s->vertData, CD_MASK_MESH, CD_CALLOC, s->totvert);
	CustomData_copy_data(&dm->vertData, &s->vertData,
	                     0, 0, s->totvert);

	CustomData_copy(&dm->loopData, &s->loopData, CD_MASK_MESH, CD_CALLOC, s->totloop);
	CustomData_copy_data(&dm->loopData, &s->loopData,
	                     0, 0, s->totloop);

	CustomData_copy(&dm->polyData, &s->polyData, CD_MASK_MESH, CD_CALLOC, s->totpoly);
	CustomData_copy_data(&dm->polyData, &s->polyData,
	                     0, 0, s->totpoly);

	return s;
}

/* modified from BKE_mesh_center_median */
bool BKE_fracture_shard_center_median(Shard *shard, float cent[3])
{
	int i = shard->totvert;
	MVert *mvert;
	zero_v3(cent);
	for (mvert = shard->mvert; i--; mvert++) {
		add_v3_v3(cent, mvert->co);
	}
	/* otherwise we get NAN for 0 verts */
	if (shard->totvert) {
		mul_v3_fl(cent, 1.0f / (float)shard->totvert);
	}

	return (shard->totvert != 0);
}

/* modified from BKE_mesh_center_centroid */
bool BKE_fracture_shard_center_centroid(Shard *shard, float cent[3])
{
	int i = shard->totpoly;
	MPoly *mpoly;
	float poly_area;
	float total_area = 0.0f;
	float poly_cent[3];

	zero_v3(cent);

	/* calculate a weighted average of polygon centroids */
	for (mpoly = shard->mpoly; i--; mpoly++) {
		BKE_mesh_calc_poly_center(mpoly, shard->mloop + mpoly->loopstart, shard->mvert, poly_cent);
		poly_area = BKE_mesh_calc_poly_area(mpoly, shard->mloop + mpoly->loopstart, shard->mvert);
		madd_v3_v3fl(cent, poly_cent, poly_area);
		total_area += poly_area;
	}
	/* otherwise we get NAN for 0 polys */
	if (shard->totpoly) {
		mul_v3_fl(cent, 1.0f / total_area);
	}

	/* zero area faces cause this, fallback to median */
	if (UNLIKELY(!is_finite_v3(cent))) {
		return BKE_fracture_shard_center_median(shard, cent);
	}
	copy_v3_v3(shard->centroid, cent);

	return (shard->totpoly != 0);
}

void BKE_shard_free(Shard *s, bool doCustomData)
{
	MEM_freeN(s->mvert);
	MEM_freeN(s->mloop);
	MEM_freeN(s->mpoly);
	if (s->neighbor_ids)
		MEM_freeN(s->neighbor_ids);
	if (s->cluster_colors)
		MEM_freeN(s->cluster_colors);

	//not used here... (i think... because it crashes on free ?)
	/*s->vertData.external = NULL;
	s->loopData.external = NULL;
	s->polyData.external = NULL;*/

	if (doCustomData)
	{
		CustomData_free(&s->vertData, s->totvert);
		CustomData_free(&s->loopData, s->totloop);
		CustomData_free(&s->polyData, s->totpoly);
	}

	MEM_freeN(s);
}

float BKE_shard_calc_minmax(Shard *shard)
{
	float min[3], max[3], diff[3];
	int i;
	
	INIT_MINMAX(min, max);
	for (i = 0; i < shard->totvert; i++) {
		minmax_v3v3_v3(min, max, shard->mvert[i].co);
	}
	
	copy_v3_v3(shard->min, min);
	copy_v3_v3(shard->max, max);

	sub_v3_v3v3(diff, max, min);
	return len_v3(diff);
	//return MAX3(diff[0], diff[1], diff[2]);
}

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
Shard *BKE_shard_by_id(FracMesh* mesh, ShardID id, DerivedMesh* dm) {
	if ((id < mesh->shard_count) && (id >= 0))
	{
		return mesh->shard_map[id];
	}
	else if (id == -1)
	{
		//create temporary shard
		Shard* s = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
		                                  dm->numVertData, dm->numPolyData, dm->numLoopData, true);
		s = BKE_custom_data_to_shard(s, dm);
		s->flag = SHARD_INTACT;
		return s;
	}
	
	return NULL;
}

#if 0
Shard *BKE_shard_by_position(FracMesh* mesh, float position[3]) {
	//find centroid via kdtree, should be fast
	return NULL;
	
}
#endif

void BKE_get_shard_geometry(FracMesh* mesh, ShardID id, MVert** vert, int *totvert, DerivedMesh *dm)
{
	/* XXX incompatible pointer types, bad! */
	Shard* shard = BKE_shard_by_id(mesh, id, dm);
	if (shard != NULL) {
		*vert = shard->mvert;
		*totvert = shard->totvert;
	}
}

void BKE_get_shard_minmax(FracMesh* mesh, ShardID id, float min_r[3], float max_r[3], DerivedMesh *dm)
{
	Shard* shard = BKE_shard_by_id(mesh, id, dm);
	if (shard != NULL) {
		copy_v3_v3(min_r, shard->min);
		copy_v3_v3(max_r, shard->max);
	}

	if (id == -1)
	{
		BKE_shard_free(shard, true);
	}
}

Shard *BKE_create_fracture_shard(MVert *mvert, MPoly *mpoly, MLoop *mloop, int totvert, int totpoly, int totloop, bool copy)
{
	Shard* shard = MEM_mallocN(sizeof(Shard), __func__);
	shard->totvert = totvert;
	shard->totpoly = totpoly;
	shard->totloop = totloop;
	shard->cluster_colors = NULL;
	shard->neighbor_ids = NULL;
	shard->neighbor_count = 0;
	
	if (copy) {
		shard->mvert = MEM_mallocN(sizeof(MVert) * totvert, "shard vertices");
		shard->mpoly = MEM_mallocN(sizeof(MPoly) * totpoly, "shard polys");
		shard->mloop = MEM_mallocN(sizeof(MLoop) * totloop, "shard loops");
		memcpy(shard->mvert, mvert, sizeof(MVert) * totvert);
		memcpy(shard->mpoly, mpoly, sizeof(MPoly) * totpoly);
		memcpy(shard->mloop, mloop, sizeof(MLoop) * totloop);
	}
	else {
		shard->mvert = mvert;
		shard->mpoly = mpoly;
		shard->mloop = mloop;
	}

	shard->flag |= SHARD_INTACT;
	BKE_shard_calc_minmax(shard);

	//omit for now, makes problems...
	BKE_fracture_shard_center_centroid(shard, shard->centroid);
	
	//neighborhood info ? optional from fracture process...
	//id is created when inserted into fracmesh, externally then...
	return shard;
}

FracMesh *BKE_create_fracture_container(DerivedMesh* dm)
{
	FracMesh* fmesh;
	//Shard *shard;
	
	fmesh = MEM_mallocN(sizeof(FracMesh), __func__);
	
	fmesh->shard_map = MEM_mallocN(sizeof(Shard*), __func__); //allocate in chunks ?, better use proper blender functions for this
	fmesh->shard_count = 0;
	
	/*shard = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
	                                  dm->numVertData, dm->numPolyData, dm->numLoopData, true);
	shard->shard_id = 0; //the original "shard"
	fmesh->shard_map[0] = shard;
	shard->neighbor_ids = NULL;
	shard->neighbor_count = 0;
	shard->parent_id = -1; //no backup id means this shard has not been re-fractured*/
	
	return fmesh;
}



void BKE_fracture_shard_by_points(FracMesh *fmesh, ShardID id, FracPointCloud *pointcloud, int algorithm, Object* obj, DerivedMesh* dm) {
	int n_size = 8;
	
	Shard *shard;
	
	float min[3], max[3];
	float theta = 0.1f; /* TODO */
	int p;
	
	container *voro_container;
	particle_order *voro_particle_order;
	loop_order *voro_loop_order;

#if defined(_WIN32) || defined(__APPLE__)
	const char *filename = "test.out";
	char *path, *fullpath;
#else
	char *bp;
	size_t size;
#endif

	FILE *stream;
#ifdef USE_DEBUG_TIMER
	double time_start;
#endif
	
	shard = BKE_shard_by_id(fmesh, id, dm);
	if (!shard || shard->flag & SHARD_FRACTURED)
		return;

	
	/* calculate bounding box with theta margin */
	copy_v3_v3(min, shard->min);
	copy_v3_v3(max, shard->max);

	if (id == -1)
	{
		BKE_shard_free(shard, true);
	}

	add_v3_fl(min, -theta);
	add_v3_fl(max, theta);
	
	voro_container = container_new(min[0], max[0], min[1], max[1], min[2], max[2],
	                               n_size, n_size, n_size, false, false, false,
	                               pointcloud->totpoints);
	
	voro_particle_order = particle_order_new();
	for (p = 0; p < pointcloud->totpoints; p++) {
		float *co = pointcloud->points[p].co;
		container_put(voro_container, voro_particle_order, p, co[0], co[1], co[2]);
	}
	
	voro_loop_order = loop_order_new(voro_container, voro_particle_order);
	
	/* Compute the voronoi cells and place output in stream
	 * See voro++ homepage for detailed description
	 * http://math.lbl.gov/voro++/
	 */

	/* %i the particle/cell index
	 * 
	 * %w number of vertices (totvert)
	 * %P global vertex coordinates
	 * v  vertex section delimiter
	 * 
	 * %s number of faces (totpoly)
	 * %a number of vertices in each face (sum is totloop)
	 * %t the indices to the cell vertices, describes which vertices build each face
	 * %n neighboring cell index for each face
	 * f  face section delimiter
	 * 
	 * %C the centroid of the voronoi cell
	 * c  centroid section delimiter
	 */

	/* argh, WIN32 doesnt support open_memstream, too bad, so we fall back to a regular file here....*/

#if defined(_WIN32) || defined(__APPLE__)
	path = MEM_mallocN(((strlen(BLI_temporary_dir()) + strlen(filename) + 2) * sizeof(char)), "path");
	path = strcpy(path, BLI_temporary_dir());
	fullpath = strcat(path, filename);
	stream = fopen(fullpath, "w+");
#else
	stream = open_memstream(&bp, &size);
#endif

	container_print_custom(voro_loop_order, voro_container, "%i %w %P v %s %a %t %n f %C c", stream);
#if 0
	{ /* DEBUG PRINT */
		fflush (stream);
		printf("================================\n");
		printf(bp);
		printf("================================\n");
	}
#endif
	fclose (stream);
	
	free(voro_particle_order);
	free(voro_loop_order);
	free(voro_container);
	
#ifdef USE_DEBUG_TIMER
	time_start = PIL_check_seconds_timer();
#endif

#if defined(_WIN32) || defined(__APPLE__)
	stream = fopen(fullpath, "r");
#else
	stream = fmemopen(bp, size, "r");
#endif

	parse_stream(stream, pointcloud->totpoints, id, fmesh, algorithm, obj, dm);
	fclose (stream);

#ifdef USE_DEBUG_TIMER
	printf("Fracture done, %g\n", PIL_check_seconds_timer() - time_start);
#endif
	

#if defined(_WIN32) || defined(__APPLE__)
	MEM_freeN(path);
#else
	free(bp);
#endif
}

void BKE_fracmesh_free(FracMesh* fm, bool doCustomData)
{
	int i = 0;
	for (i = 0; i < fm->shard_count; i++)
	{
		Shard* s = fm->shard_map[i];
		BKE_shard_free(s, doCustomData);
		/*MEM_freeN(s->mvert);
		MEM_freeN(s->mpoly);
		MEM_freeN(s->mloop);
		if (s->neighbor_ids)
			MEM_freeN(s->neighbor_ids);*/
		//MEM_freeN(s);
	}

	if (fm->shard_map)
	{
		MEM_freeN(fm->shard_map);
		fm->shard_map = NULL;
	}
}


/* DerivedMesh */

void BKE_fracture_release_dm(FractureModifierData *fmd)
{
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
}

static DerivedMesh *create_dm(FracMesh *fracmesh, bool doCustomData)
{
	int shard_count = fracmesh->shard_count;
	Shard **shard_map = fracmesh->shard_map;
	Shard *shard;
	int s, i;
	
	int num_verts, num_polys, num_loops;
	int vertstart, polystart, loopstart;
	DerivedMesh *result;
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;
	MEdge *medges, *me;
	
	num_verts = num_polys = num_loops = 0;
	for (s = 0; s < shard_count; ++s) {
		shard = shard_map[s];

		if (shard->shard_id == 0 && shard->flag & SHARD_FRACTURED)
		{
			//dont display first shard when its fractured (relevant for plain voronoi only)
			//continue;
		}
		
		num_verts += shard->totvert;
		num_polys += shard->totpoly;
		num_loops += shard->totloop;
	}
	
	result = CDDM_new(num_verts, 0, 0, num_loops, num_polys);
	mverts = CDDM_get_verts(result);
	mloops = CDDM_get_loops(result);
	mpolys = CDDM_get_polys(result);

	if (doCustomData && shard_count > 0)
	{
		CustomData_copy(&shard_map[0]->vertData, &result->vertData, CD_MASK_MESH, CD_CALLOC, num_verts);
		CustomData_copy(&shard_map[0]->polyData, &result->polyData, CD_MASK_MESH, CD_CALLOC, num_polys);
		CustomData_copy(&shard_map[0]->loopData, &result->loopData, CD_MASK_MESH, CD_CALLOC, num_loops);
	}
	
	vertstart = polystart = loopstart = 0;
	for (s = 0; s < shard_count; ++s) {
		MPoly *mp;
		MLoop *ml;
		int i;
		shard = shard_map[s];

		if (shard->shard_id == 0 && shard->flag & SHARD_FRACTURED)
		{
			//dont display first shard when its fractured (relevant for plain voronoi only)
			//continue;
		}

		if (doCustomData)
		{
			CustomData_copy_data(&shard->vertData, &result->vertData, 0, vertstart, shard->totvert);
			CustomData_copy_data(&shard->loopData, &result->loopData, 0, loopstart, shard->totloop);
			CustomData_copy_data(&shard->polyData, &result->polyData, 0, polystart, shard->totpoly);
		}
		
		memcpy(mverts + vertstart, shard->mvert, shard->totvert * sizeof(MVert));
		memcpy(mpolys + polystart, shard->mpoly, shard->totpoly * sizeof(MPoly));

		for (i = 0, mp = mpolys + polystart; i < shard->totpoly; ++i, ++mp) {
			/* adjust loopstart index */
			mp->loopstart += loopstart;
			if (doCustomData)
			{
				CustomData_set(&result->polyData, i+polystart, CD_MPOLY, mp);
			}
		}
		
		memcpy(mloops + loopstart, shard->mloop, shard->totloop * sizeof(MLoop));

		for (i = 0, ml = mloops + loopstart; i < shard->totloop; ++i, ++ml) {
			/* adjust vertex index */
			ml->v += vertstart;
			if (doCustomData)
			{
				CustomData_set(&result->loopData, i+loopstart, CD_MLOOP, ml);
			}
		}
		
		vertstart += shard->totvert;
		polystart += shard->totpoly;
		loopstart += shard->totloop;
	}
	
	CDDM_calc_edges(result);

	if (doCustomData)
	{
		MLoop* ml;

		for (i = 0, ml = mloops; i < result->numLoopData; ++i, ++ml) {
			CustomData_set(&result->loopData, i, CD_MLOOP, ml);
		}

		//disable edge drawing... why is this enabled ??
		/*medges = CDDM_get_edges(result);
		for (i = 0, me = medges + i; i < result->numEdgeData; i++)
		{
			//me->flag &~ ME_EDGEDRAW;
			CustomData_set(&result->edgeData, i, CD_MEDGE, me);
		}*/
	}
	
	result->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(result);
	return result;
}

void BKE_fracture_create_dm(FractureModifierData *fmd, bool do_merge)
{
	DerivedMesh *dm_final = NULL;
	FractureLevel* fl = fmd->fracture_levels.first;
	bool doCustomData = fl->frac_algorithm != MOD_FRACTURE_NONE && fl->frac_algorithm != MOD_FRACTURE_VORONOI;
	
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
	
	//if ((fmd->frac_mesh->shard_map != NULL) && (fmd->frac_mesh->shard_count > 0))
	{
		dm_final = create_dm(fmd->frac_mesh, doCustomData);
	}
	
	fmd->dm = dm_final;
}

DerivedMesh *BKE_shard_create_dm(Shard *s, bool doCustomData)
{
	DerivedMesh *dm;
	MVert *mverts;
	MLoop *mloops;
	MPoly *mpolys;
	
	dm  = CDDM_new(s->totvert, 0, 0, s->totloop, s->totpoly);

	mverts = CDDM_get_verts(dm);
	mloops = CDDM_get_loops(dm);
	mpolys = CDDM_get_polys(dm);
	memcpy(mverts, s->mvert, s->totvert * sizeof(MVert));
	memcpy(mloops, s->mloop, s->totloop * sizeof(MLoop));
	memcpy(mpolys, s->mpoly, s->totpoly * sizeof(MPoly));

	CDDM_calc_edges(dm);

	if (doCustomData)
	{
		MLoop *ml;
		int i;

		//thats all banana here... why cant i just copy the data, without reallocing the layers ?!!! so this is done twice here
		//and results in a memory leak...

		CustomData_copy(&s->vertData, &dm->vertData, CD_MASK_MESH, CD_CALLOC, s->totvert);
		CustomData_copy_data(&s->vertData, &dm->vertData, 0, 0, s->totvert);

		CustomData_copy(&s->loopData, &dm->loopData, CD_MASK_MESH, CD_CALLOC, s->totloop);
		CustomData_copy_data(&s->loopData, &dm->loopData, 0, 0, s->totloop);

		CustomData_copy(&s->polyData, &dm->polyData, CD_MASK_MESH, CD_CALLOC, s->totpoly);
		CustomData_copy_data(&s->polyData, &dm->polyData, 0, 0, s->totpoly);

		//update custom data after calc edges ?
		for (i = 0, ml = mloops; i < dm->numLoopData; ++i, ++ml) {
			CustomData_set(&dm->loopData, i, CD_MLOOP, ml);
		}
	}
	
	dm->dirty |= DM_DIRTY_NORMALS;
	CDDM_calc_normals_mapping(dm);

	return dm;
}

void BKE_shard_assign_material(Shard* s, short mat_nr)
{
	MPoly* mp;
	int i;

	for (i = 0, mp = s->mpoly; i < s->totpoly; ++i, ++mp) {
		mp->mat_nr = mat_nr;
	}
}
