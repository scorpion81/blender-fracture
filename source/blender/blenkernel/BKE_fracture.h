#ifndef BKE_FRACTURE_H
#define BKE_FRACTURE_H

#include "DNA_fracture_types.h"
#include "DNA_meshdata_types.h"
#include "BLI_sys_types.h"
#include "RBI_api.h"

typedef int ShardID;

typedef Shard* ShardList;

typedef struct ShardIterator {
	FracMesh* frac_mesh;
	int		current;
} ShardIterator;



typedef struct FracMeshIterator {
	FracHistory* frac_history;
	int		current_step;
} FracMeshIterator;

/* iterator functions for efficient looping over shards */
ShardIterator* BKE_shards_begin(FracMesh *fmesh);
ShardIterator* BKE_shards_next(ShardIterator *iter);
bool BKE_shards_valid(ShardIterator* iter);
void BKE_shards_end(ShardIterator* iter); 
Shard* BKE_shard_by_iterator(ShardIterator *iter);

/*direct access*/
Shard* BKE_shard_by_id(FracMesh* mesh, ShardID id);

/* detailed info to the particular shards */
void BKE_get_shard_geometry(FracMesh* mesh, ShardID id, MVert** vert, int *totvert);
void BKE_get_shard_bbox(FracMesh* mesh, ShardID id, BoundBox* bbox);

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
FracMesh* BKE_create_fracture_container(Object* ob); //check for mesh; for curves / fonts... convert to mesh automatically, warn user (fonts, ensure remeshing before)
FracMesh* BKE_close_fracture_container(Group* g);
Group* BKE_open_fracture_container(FracMesh* fm);
void BKE_drop_fracture_container(Object* ob);

//hmm maybe a listbase of steps, its dynamically created by user interaction(THIS is in hierarchy!!! prefractured) or a dynafrac step (THIS suits.)
void BKE_add_fracture_step(FracHistory* fh, FracMesh* ob);
void BKE_remove_fracture_step(FracHistory* fh); //do this via framemap entry ? needs to be a hash, index->frame->fracmesh ? hmm just append at end and remove the last, should suffice... just to grow / shrink it, no direct manip necessary....
//also throw away history when editing the fracmesh or even transforming it, because this invalidates the whole sim cache

/* iterator functions for efficient / abstract iterating over fracture history */
FracMeshIterator* BKE_fracture_steps_begin(Object *ob);
FracMeshIterator* BKE_fracture_steps_next(FracMeshIterator *iter);
FracMeshIterator* BKE_fracture_steps_prev(FracMeshIterator *iter);
bool BKE_fracture_steps_valid(FracMeshIterator* iter);
void BKE_fracture_steps_end(FracMeshIterator* iter); 



/*** Bullet API erweiterungen für fracturing ***/

// erzeugt collision shape für ein einzelnes shard
rbCollisionShape *make_collision_shape(FracMesh *fmesh, ShardID id);

// eine liste mit neuen shard aus fracture methoden
// muss vom caller gelöscht werden wenn es als return value verwendet wird
// alternativ könnte auch jedes shard eine "ShardID parent_id" speichern,
// dann muss der caller über alle shards loopen um die neuen sub-shards zu behandeln

// erzeuge shards aus dem basis mesh und einer liste von points (nicht weiter spezifiziert, können auch particles oder so sein)
ShardList fracture_by_points(FracMesh *fmesh, PointCloud *points);

// Zerbreche ein einzelnes shard basierend auf collision
// btManifoldPoint ist eine Bullet class, das sollte wahrscheinlich etwas abstrahiert werden
// Jeder contact hat ein "applied impulse" was dem fracture system erlaubt dynamisch auf kollisionen zu reagieren (festigkeiten usw.)
// Die contact point Koordinaten werden in Object space umgerechnet, so dass fracture local arbeiten kann ohne die eigentliche world transform zu kennen
//ShardList fracture_by_impulse(FracMesh *fmesh, ShardID id, btManifoldPoint *contact, int num_contacts)


#endif // BKE_FRACTURE_H
