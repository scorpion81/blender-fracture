#ifndef BKE_FRACTURE_UTIL_H
#define BKE_FRACTURE_UTIL_H

#include "DNA_fracture_types.h"

Shard *BKE_fracture_shard_boolean(Shard *parent, Shard* child, float obmat[4][4]);

#endif // BKE_FRACTURE_UTIL_H
