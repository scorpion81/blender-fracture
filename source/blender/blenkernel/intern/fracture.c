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

#include "bmesh.h"

#include "RBI_api.h"
#include "PIL_time.h"

/*boolean support */
//#include "CSG_BooleanOps.h"

#ifdef WITH_VORO
#include "../../../../extern/voro++/src/c_interface.hh"
#endif

//utility... bbox / centroid calc

/*prototypes*/
static void parse_stream(FILE *fp, int expected_shards, ShardID shard_id, FracMesh *fm, int algorithm);
static Shard *parse_shard(FILE *fp);
static int parse_verts(FILE *fp, MVert **vert);
static void parse_polys(FILE *fp, MPoly **poly, MLoop **loop, int *totpoly, int *totloop);
static int parse_neighbors(FILE *fp, int **neighbors);
static void add_shard(FracMesh *fm, Shard *s);

static void add_shard(FracMesh *fm, Shard *s)
{
	fm->shard_map = MEM_reallocN(fm->shard_map, sizeof(Shard*) * (fm->shard_count+1));
	fm->shard_map[fm->shard_count] = s;
	s->shard_id = fm->shard_count;
	fm->shard_count++;
}

/* parse the voro++ raw data */
static void parse_stream(FILE *fp, int expected_shards, ShardID parent_id, FracMesh *fm, int algorithm)
{
	/*Parse voronoi raw data*/
	int i = 0;
	Shard* s = NULL, *p = BKE_shard_by_id(fm, parent_id);
	float obmat[4][4]; /* use unit matrix for now */

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
			s = BKE_fracture_shard_boolean(p, s, obmat);
		}
		else if (algorithm == MOD_FRACTURE_BISECT || algorithm == MOD_FRACTURE_BISECT_FILL)
		{
			s = BKE_fracture_shard_bisect(p, s, obmat, algorithm == MOD_FRACTURE_BISECT_FILL);
		}
		if (s != NULL)
		{
			s->parent_id = parent_id;
			s->flag = SHARD_INTACT;
			add_shard(fm, s);
		}
	}
}

static Shard* parse_shard(FILE *fp)
{
	Shard *s;
	MVert *vert = NULL;
	MPoly *poly = NULL;
	MLoop *loop = NULL;
	int *neighbors = NULL;
	int totpoly = 0, totloop = 0, totvert = 0, totn = 0;
	float centr[3];
	int shard_id;

	fscanf(fp, "%d ", &shard_id);

	totvert = parse_verts(fp, &vert);
	parse_polys(fp, &poly, &loop, &totpoly, &totloop);

	/* parse centroid */
	fscanf(fp, "%f %f %f n ", &centr[0], &centr[1], &centr[2]);

	totn = parse_neighbors(fp, &neighbors);

	if ((totvert == 0) || (totloop == 0) || (totloop == 0) || (totn == 0))
	{
		MEM_freeN(vert);
		MEM_freeN(poly);
		MEM_freeN(loop);
		MEM_freeN(neighbors);
		return NULL;
	}

	s = BKE_create_fracture_shard(vert, poly, loop, totvert, totpoly, totloop, false);
	s->neighbor_ids = neighbors;
	s->neighbor_count = totn;
	//s->shard_id = shard_id;
	copy_v3_v3(s->centroid, centr);



	/* if not at end of file yet, skip newlines */
	if (feof(fp) == 0)
	{

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

static int parse_verts(FILE* fp, MVert** vert)
{
	int totvert = 0;
	while (1)
	{
		int readco = 0;
		float co[3];

		readco = fscanf(fp, "(%f,%f,%f) ", &co[0], &co[1], &co[2]);
		if (readco < 3) break;
		if (totvert == 0)
		{
			*vert = MEM_mallocN(sizeof(MVert), __func__);
		}
		else
		{
			*vert = MEM_reallocN(*vert, sizeof(MVert) * (totvert+1));
		}
		copy_v3_v3((*vert)[totvert].co, co);
		totvert++;
	}

	/* skip "v "*/
	fseek(fp, 2*sizeof(char), SEEK_CUR);

	return totvert;
}

static void parse_polys(FILE *fp, MPoly **poly, MLoop **loop, int *totpoly, int *totloop)
{
	int read_index = 0;
	int index = -1;

	while (1) {
		int faceloop = 0;
		read_index = fscanf(fp, "(%d", &index);
		if (read_index == 0) {
			/* skip ") f "*/
			fseek(fp, 4*sizeof(char), SEEK_CUR);
			return;
		}

		if (*totloop == 0)
		{
			*loop = MEM_mallocN(sizeof(MLoop), __func__);
		}
		else
		{
			*loop = MEM_reallocN(*loop, sizeof(MLoop) * ((*totloop)+1));
		}
		(*loop)[*totloop].v = index;
		(*loop)[*totloop].e = *totloop;
		(*totloop)++;

		faceloop++;

		while (1)
		{
			read_index = fscanf(fp, ",%d", &index);
			if (!read_index) break;
			// loop[totloop].e =  how on earth create edge indexes for loop ? */
			*loop = MEM_reallocN(*loop, sizeof(MLoop) * ((*totloop)+1));
			(*loop)[*totloop].v = index;
			(*loop)[*totloop].e = *totloop;
			(*totloop)++;
			faceloop++;
		}


		/* skip ") "*/
		fseek(fp, 2*sizeof(char), SEEK_CUR);

		/* faceloop, 3 loops at least necessary ?, prevent degenerates... disable for now */
		if (*totpoly == 0)
		{
			*poly = MEM_mallocN(sizeof(MPoly), __func__);
		}
		else
		{
			*poly = MEM_reallocN(*poly, sizeof(MPoly) * ((*totpoly)+1));
		}
		(*poly)[*totpoly].loopstart = (*totloop) - faceloop;
		(*poly)[*totpoly].totloop = faceloop;
		(*poly)[*totpoly].mat_nr = 0;
		(*poly)[*totpoly].flag = 0;
		(*totpoly)++;
	}
}

static int parse_neighbors(FILE* fp, int** neighbors)
{
	int totn = 0;
	int read_n = 0;
	while (1) {
		int n;
		read_n = fscanf(fp, "%d ", &n);
		if (!read_n) break;
		if (totn == 0)
		{
			*neighbors = MEM_mallocN(sizeof(int), __func__);
		}
		else
		{
			*neighbors = MEM_reallocN(*neighbors, sizeof(int) * (totn+1));
		}
		(*neighbors)[totn] = n;
		totn++;
	}

	//skip "x"
	fseek(fp, 1*sizeof(char), SEEK_CUR);

	return totn;
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
		poly_area = BKE_mesh_calc_poly_planar_area_centroid(mpoly, shard->mloop + mpoly->loopstart, shard->mvert, poly_cent);

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

void BKE_shard_free(Shard *s)
{
	MEM_freeN(s->mvert);
	MEM_freeN(s->mloop);
	MEM_freeN(s->mpoly);
	if (s->neighbor_ids)
		MEM_freeN(s->neighbor_ids);
	MEM_freeN(s);
}

void BKE_shard_calc_minmax(Shard *shard)
{
	float min[3], max[3];
	int i;
	
	INIT_MINMAX(min, max);
	for (i = 0; i < shard->totvert; i++) {
		minmax_v3v3_v3(min, max, shard->mvert[i].co);
	}
	
	copy_v3_v3(shard->min, min);
	copy_v3_v3(shard->max, max);
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
Shard *BKE_shard_by_id(FracMesh* mesh, ShardID id) {
	if ((id < mesh->shard_count) && (id >= 0))
	{
		return mesh->shard_map[id];
	}
	
	return NULL;
}

#if 0
Shard *BKE_shard_by_position(FracMesh* mesh, float position[3]) {
	//find centroid via kdtree, should be fast
	return NULL;
	
}
#endif

void BKE_get_shard_geometry(FracMesh* mesh, ShardID id, MVert** vert, int *totvert)
{
	/* XXX incompatible pointer types, bad! */
	Shard* shard = BKE_shard_by_id(mesh, id);
	if (shard != NULL) {
		*vert = shard->mvert;
		*totvert = shard->totvert;
	}
}

void BKE_get_shard_minmax(FracMesh* mesh, ShardID id, float min_r[3], float max_r[3])
{
	Shard* shard = BKE_shard_by_id(mesh, id);
	if (shard != NULL) {
		copy_v3_v3(min_r, shard->min);
		copy_v3_v3(max_r, shard->max);
	}
}

Shard *BKE_create_fracture_shard(MVert *mvert, MPoly *mpoly, MLoop *mloop, int totvert, int totpoly, int totloop, bool copy)
{
	Shard* shard = MEM_mallocN(sizeof(Shard), __func__);
	shard->totvert = totvert;
	shard->totpoly = totpoly;
	shard->totloop = totloop;
	
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
	Shard *shard;
	
	fmesh = MEM_mallocN(sizeof(FracMesh), __func__);
	
	fmesh->shard_map = MEM_mallocN(sizeof(Shard*), __func__); //allocate in chunks ?, better use proper blender functions for this
	fmesh->shard_count = 1;
	
	shard = BKE_create_fracture_shard(dm->getVertArray(dm), dm->getPolyArray(dm), dm->getLoopArray(dm),
	                                  dm->numVertData, dm->numPolyData, dm->numLoopData, true);
	shard->shard_id = 0; //the original "shard"
	fmesh->shard_map[0] = shard;
	shard->neighbor_ids = NULL;
	shard->neighbor_count = 0;
	shard->parent_id = -1; //no backup id means this shard has not been re-fractured
	
	return fmesh;
}



void BKE_fracture_shard_by_points(FracMesh *fmesh, ShardID id, FracPointCloud *pointcloud, int algorithm) {
	int n_size = 8;
	
	Shard *shard;
	
	float min[3], max[3];
	float theta = 0.1f; /* TODO */
	int p;
	
	container *voro_container;
	particle_order *voro_particle_order;
	loop_order *voro_loop_order;
	
	char *bp;
	size_t size;
	FILE *stream;


	double start;
	char *file, *path, *fullpath;

	shard = BKE_shard_by_id(fmesh, id);
	if (!shard || shard->flag & SHARD_FRACTURED)
		return;

	
	/* calculate bounding box with theta margin */
	copy_v3_v3(min, shard->min);
	copy_v3_v3(max, shard->max);
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

	/* %i the particle index
	 * %P global vertex coordinates of voronoi vertices
	 * v  the vertex -> face delimiter
	 * %t the indexes to the cell vertices, describes which vertices build each face
	 * f  the face -> centroid section delimiter
	 * %C the centroid of the voronoi cell
	 */
	
	start = PIL_check_seconds_timer();

	file = "test.out";
	path = MEM_mallocN(((strlen(BLI_temporary_dir()) + strlen(file) + 2) * sizeof(char)), "path");
	path = strcpy(path, BLI_temporary_dir());
	fullpath = strcat(path, file);
	stream = fopen(fullpath, "w+");
	//stream = open_memstream(&bp, &size); //this is DEADLY for performance, Better use a real file...
	container_print_custom(voro_loop_order, voro_container, "%i %P v %t f %C n %n x", stream);
	fflush(stream);
	rewind(stream);

	printf("Print custom done, %g\n", PIL_check_seconds_timer() - start);
	start = PIL_check_seconds_timer();
	parse_stream(stream, pointcloud->totpoints, id, fmesh, algorithm);
	//printf("%s", bp);
	fclose (stream);
//	free(bp);
	printf("Parse stream done, %g\n", PIL_check_seconds_timer() - start);

	MEM_freeN(path);
	free(voro_particle_order);
	free(voro_loop_order);
	free(voro_container);

#if 0
	//do cell fracture code here on shardbbox, AND intersect with boolean, hmm would need a temp object for this ? or bisect with cell planes.
	//lets test bisect but this is a bmesh operator, so need to convert, do bisection and convert back, slow. and cellfrac also uses bmesh, hmm
	//so better keep this around
//#ifdef WITH_VORO++
	//FracMesh will be initialized with a shard set. First need to check whether the id is there at all. Shard Id 0 intially is the first shard
	Shard* shard = BKE_shard_by_id(mesh,  id);
	Shard* shard2;
	
	// that will become the voro++ container...
	
	BoundBox *bb = &shard->bb;
	
	//dummyfrac
	float dim[3];
	int i;
	
	sub_v3_v3v3(dim, bb->vec[4], bb->vec[0]);
	mul_v3_fl(dim, 0.5f);
	
	//need to calc bbox...argh, need a mesh for it, dumb.
	//bb2 = MEM_mallocN(sizeof(BoundBox), __func__);
	//bb2->flag = bb->flag;
	
	// copy shard
	shard2 = BKE_create_fracture_shard(shard->mvert, shard->mpoly, shard->mloop, shard->totvert, shard->totpoly, shard->totloop);
	shard2->shard_id = mesh->shard_count;
	
	//scale and translate
	for (i = 0; i < shard->totvert; i++) {
		shard->mvert[i].co[0] *= 0.5f;
		shard->mvert[i].co[0] -= dim[0];
	}
	for (i = 0; i < shard2->totvert; ++i) {
		shard2->mvert[i].co[0] *= 0.5f;
		shard2->mvert[i].co[0] += dim[0];
	}
	
	mesh->shard_map = MEM_reallocN(mesh->shard_map, sizeof(Shard*) * (mesh->shard_count+1));
	mesh->shard_map[mesh->shard_count] = shard2;
	mesh->shard_count++;
	
	//scale down the original geometry in X direction by 0.5 and translate by halfbbox
	//duplicate it (add all verts again, + 
	
	
	//build the voro++ container now
	
	//add pointcloud points into container
	
	//and trigger voro++
	
	
	//parse result (new function)
	

//#else
		
//#endif
#endif
}

void BKE_fracmesh_free(FracMesh* fm)
{
	int i = 0;
	for (i = 0; i < fm->shard_count; i++)
	{
		Shard* s = fm->shard_map[i];
		MEM_freeN(s->mvert);
		MEM_freeN(s->mpoly);
		MEM_freeN(s->mloop);
		if (s->neighbor_ids)
			MEM_freeN(s->neighbor_ids);
		MEM_freeN(s);
	}
	
	MEM_freeN(fm->shard_map);
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

static DerivedMesh *create_dm(FracMesh *fracmesh)
{
	int shard_count = fracmesh->shard_count;
	Shard **shard_map = fracmesh->shard_map;
	Shard *shard;
	int s;
	
	int num_verts, num_polys, num_loops;
	int vertstart, polystart, loopstart;
	DerivedMesh *result;
	MVert *mverts;
	MPoly *mpolys;
	MLoop *mloops;
	
	num_verts = num_polys = num_loops = 0;
	for (s = 0; s < shard_count; ++s) {
		shard = shard_map[s];

		if (shard->shard_id == 0 && shard->flag & SHARD_FRACTURED)
		{
			//dont display first shard when its fractured (relevant for plain voronoi only)
			continue;
		}
		
		num_verts += shard->totvert;
		num_polys += shard->totpoly;
		num_loops += shard->totloop;
	}
	
	result = CDDM_new(num_verts, 0, 0, num_loops, num_polys);
	mverts = CDDM_get_verts(result);
	mloops = CDDM_get_loops(result);
	mpolys = CDDM_get_polys(result);
	
	vertstart = polystart = loopstart = 0;
	for (s = 0; s < shard_count; ++s) {
		MPoly *mp;
		MLoop *ml;
		int i;
		shard = shard_map[s];

		if (shard->shard_id == 0 && shard->flag & SHARD_FRACTURED)
		{
			//dont display first shard when its fractured (relevant for plain voronoi only)
			continue;
		}
		
		memcpy(mverts + vertstart, shard->mvert, shard->totvert * sizeof(MVert));
		
		memcpy(mpolys + polystart, shard->mpoly, shard->totpoly * sizeof(MPoly));
		for (i = 0, mp = mpolys + polystart; i < shard->totpoly; ++i, ++mp) {
			/* adjust loopstart index */
			mp->loopstart += loopstart;
		}
		
		memcpy(mloops + loopstart, shard->mloop, shard->totloop * sizeof(MLoop));
		for (i = 0, ml = mloops + loopstart; i < shard->totloop; ++i, ++ml) {
			/* adjust vertex index */
			ml->v += vertstart;
		}
		
		vertstart += shard->totvert;
		polystart += shard->totpoly;
		loopstart += shard->totloop;
	}
	
	CDDM_calc_edges(result);
	
	result->dirty |= DM_DIRTY_NORMALS;
	
	return result;
}

void BKE_fracture_create_dm(FractureModifierData *fmd, bool do_merge)
{
	DerivedMesh *dm_final;
	
	if (fmd->dm) {
		fmd->dm->needsFree = 1;
		fmd->dm->release(fmd->dm);
		fmd->dm = NULL;
	}
	
	dm_final = create_dm(fmd->frac_mesh);
	
	fmd->dm = dm_final;
}

DerivedMesh *BKE_shard_create_dm(Shard *s)
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
	dm->dirty |= DM_DIRTY_NORMALS;

	return dm;
}
