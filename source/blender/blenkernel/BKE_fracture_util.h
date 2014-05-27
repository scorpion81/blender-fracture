#ifndef BKE_FRACTURE_UTIL_H
#define BKE_FRACTURE_UTIL_H

#include "DNA_fracture_types.h"

Shard *BKE_fracture_shard_boolean(Object *obj, DerivedMesh *dm_parent, Shard* child, short inner_material_index);
Shard *BKE_fracture_shard_bisect(struct BMesh *bm_orig, Shard* child, float obmat[4][4], bool use_fill, bool clear_inner, bool clear_outer, int cutlimit, float centroid[],
								 short inner_mat_index);

#endif // BKE_FRACTURE_UTIL_H
