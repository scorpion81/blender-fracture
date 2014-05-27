#ifndef BKE_FRACTURE_H
#define BKE_FRACTURE_H

#include "BLI_sys_types.h"

struct FracMesh;
struct FracHistory;
struct Shard;

struct FractureModifierData;
struct DerivedMesh;
struct Object;

struct BoundBox;
struct MVert;
struct MPoly;
struct MLoop;

typedef int ShardID;

typedef struct ShardIterator {
	struct FracMesh* frac_mesh;
	int		current;
} ShardIterator;

typedef struct FracMeshIterator {
	struct FracHistory* frac_history;
	int		current_step;
} FracMeshIterator;

typedef struct FracPoint {
	float co[3];
} FracPoint;

typedef struct FracPointCloud {
	struct FracPoint *points;	// just a bunch of positions in space
	int totpoints; // number of positions
} FracPointCloud;

/* iterator functions for efficient looping over shards */
struct ShardIterator* BKE_shards_begin(struct FracMesh *fmesh);
struct ShardIterator* BKE_shards_next(struct ShardIterator *iter);
bool BKE_shards_valid(struct ShardIterator* iter);
void BKE_shards_end(struct ShardIterator* iter); 
struct Shard* BKE_shard_by_iterator(struct ShardIterator *iter);

/*direct access*/
struct Shard* BKE_shard_by_id(struct FracMesh* mesh, ShardID id, struct DerivedMesh *dm);

/* detailed info to the particular shards */
void BKE_get_shard_geometry(struct FracMesh* mesh, ShardID id, struct MVert** vert, int *totvert, struct DerivedMesh *dm);
void BKE_get_shard_minmax(struct FracMesh* mesh, ShardID id, float min_r[3], float max_r[3], struct DerivedMesh *dm);

/* container object handling functions */
//create container -> for each island or for whole mesh create shards in a fracmesh, init fracmodifier (function for opening container makes here a fracmesh, operator will
//receive it and init a fracmodifier (or this is called from modifiers init function, yeah, a single modifier as well, physics "Fracture")
//open container -> for each island create a temp object for user manip, in a containergroup, 
//execute fractures on objects....with random pointclouds / helper objects / psys on helpers, like in addon
//close container -> commit (or discard) changes in group to fracmesh, also create parenting hierarchy !!! (empty->with a child group for easy finding, but group would
// be unnecessary then, hmm, maybe one global containergroup...ya...
//drop container -> restore basemesh and purge modifier and fracmesh
//modifier serves as storage backend for container, but could use a new ID block as well....
//pointcache / modifier or id block could store mesh fracture history, best would be optionally in pointcache, only needed for replay...
//add fracture step -> write to pointcache, first ... write to array in modifier, make optional as well
//object topology will remain same... transforms can be altered, and written AS IS into fracmesh, 
//container hierarchy -> write info to fracmesh / shard (like parent id and child ids) 
//tag small shards for "standardbrösel" -> instances of simple primitives, selected from a group, randomly rotated scaled in a certain range
struct FracMesh *BKE_create_fracture_container(struct DerivedMesh* dm); //check for mesh; for curves / fonts... convert to mesh automatically, warn user (fonts, ensure remeshing before)
//FracMesh* BKE_close_fracture_container(Group* g);
//Group* BKE_open_fracture_container(FracMesh* fm);
//void BKE_drop_fracture_container(Object* ob); //delete in modifier->free...
struct Shard *BKE_create_fracture_shard(struct MVert *mvert, struct MPoly *mpoly, struct MLoop *mloop, int totvert, int totpoly, int totloop, bool copy);
struct Shard *BKE_custom_data_to_shard(struct Shard* s, struct DerivedMesh* dm);

//hmm maybe a listbase of steps, its dynamically created by user interaction(THIS is in hierarchy!!! prefractured) or a dynafrac step (THIS suits.)
void BKE_add_fracture_step(struct FracHistory* fh, struct FracMesh* ob);
void BKE_remove_fracture_step(struct FracHistory* fh); //do this via framemap entry ? needs to be a hash, index->frame->fracmesh ? hmm just append at end and remove the last, should suffice... just to grow / shrink it, no direct manip necessary....
//also throw away history when editing the fracmesh or even transforming it, because this invalidates the whole sim cache

/* iterator functions for efficient / abstract iterating over fracture history */
struct FracMeshIterator* BKE_fracture_steps_begin(struct Object *ob);
struct FracMeshIterator* BKE_fracture_steps_next(struct FracMeshIterator *iter);
struct FracMeshIterator* BKE_fracture_steps_prev(struct FracMeshIterator *iter);
bool BKE_fracture_steps_valid(struct FracMeshIterator* iter);
void BKE_fracture_steps_end(struct FracMeshIterator* iter); 

//utility functions
bool BKE_fracture_shard_center_median(struct Shard *shard, float cent[3]);
bool BKE_fracture_shard_center_centroid(struct Shard *shard, float cent[3]);
float BKE_shard_calc_minmax(struct Shard *shard);

void BKE_fracmesh_free(struct FracMesh* fm, bool doCustomData);
void BKE_shard_free(struct Shard* s, bool doCustomData);


/* DerivedMesh */
void BKE_fracture_release_dm(struct FractureModifierData *fmd);
void BKE_fracture_create_dm(struct FractureModifierData *fmd, bool do_merge);
struct DerivedMesh *BKE_shard_create_dm(struct Shard *s, bool doCustomData);

void BKE_shard_assign_material(struct Shard* s, short mat_nr);

/*** Bullet API erweiterungen für fracturing ***/

// erzeugt collision shape für ein einzelnes shard
//rbCollisionShape *make_collision_shape(FracMesh *fmesh, ShardID id);

// eine liste mit neuen shard aus fracture methoden
// muss vom caller gelöscht werden wenn es als return value verwendet wird
// alternativ könnte auch jedes shard eine "ShardID parent_id" speichern,
// dann muss der caller über alle shards loopen um die neuen sub-shards zu behandeln

// erzeuge shards aus dem basis mesh und einer liste von points (nicht weiter spezifiziert, können auch particles oder so sein)
void BKE_fracture_shard_by_points(struct FracMesh *fmesh, ShardID id, struct FracPointCloud *points, int algorithm, struct Object *obj, struct DerivedMesh *dm,
                                  short inner_material_index);

// Zerbreche ein einzelnes shard basierend auf collision
// btManifoldPoint ist eine Bullet class, das sollte wahrscheinlich etwas abstrahiert werden
// Jeder contact hat ein "applied impulse" was dem fracture system erlaubt dynamisch auf kollisionen zu reagieren (festigkeiten usw.)
// Die contact point Koordinaten werden in Object space umgerechnet, so dass fracture local arbeiten kann ohne die eigentliche world transform zu kennen
//ShardList fracture_by_impulse(FracMesh *fmesh, ShardID id, btManifoldPoint *contact, int num_contacts)


#endif // BKE_FRACTURE_H
