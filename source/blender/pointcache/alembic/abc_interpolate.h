/*
 * Copyright 2015, Blender Foundation.
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
 */

#ifndef PTC_ABC_INTERPOLATE_H
#define PTC_ABC_INTERPOLATE_H

#include <Alembic/Abc/ISampleSelector.h>
#include <Alembic/Abc/IScalarProperty.h>
#include <Alembic/Abc/TypedPropertyTraits.h>
#include <Alembic/Abc/ITypedArrayProperty.h>
#include <Alembic/Abc/ITypedScalarProperty.h>
#include <Alembic/Abc/TypedArraySample.h>

extern "C" {
#include "BLI_math.h"
#include "BLI_utildefines.h"
}

namespace PTC {

using namespace Alembic;
using namespace Abc;

using Alembic::Util::shared_ptr;

enum InterpolateSemanticDefault {
	InterpolateSemanticDefault_None = 0,
};

enum InterpolateSemanticVector {
	InterpolateSemanticVector_Linear    = 0,
	InterpolateSemanticVector_Slerp     = 1,
};

/* linear blending for primitive types */
template <typename T>
BLI_INLINE T interpolate_sample(const T &val0, const T &val1, float t)
{
	return val0 * (1.0f-t) + val1 * t;
}

/* vector semantics */
BLI_INLINE V3f interpolate_sample(const V3f &val0, const V3f &val1, float t, InterpolateSemanticVector semantic)
{
	switch (semantic) {
		case InterpolateSemanticVector_Linear:
			return val0 * (1.0f-t) + val1 * t;
		case InterpolateSemanticVector_Slerp:
			V3f result;
			interp_v3_v3v3_slerp_safe(result.getValue(), val0.getValue(), val1.getValue(), t);
			return result;
	}
	return V3f(0.0f, 0.0f, 0.0f);
}

BLI_INLINE Quatf interpolate_sample(const Quatf &val0, const Quatf &val1, float t)
{
	float qt0[4] = {val0.r, val0.v.x, val0.v.y, val0.v.z};
	float qt1[4] = {val1.r, val1.v.x, val1.v.y, val1.v.z};
	float result[4];
	interp_qt_qtqt(result, qt0, qt1, t);
	return Quatf(result[0], result[1], result[2], result[3]);
}

BLI_INLINE M44f interpolate_sample(const M44f &val0, const M44f &val1, float t)
{
	float loc[3], quat[4], size[3];
	float loc0[3], quat0[4], size0[3];
	float loc1[3], quat1[4], size1[3];
	mat4_decompose(loc0, quat0, size0, (float (*)[4])val0.getValue());
	mat4_decompose(loc1, quat1, size1, (float (*)[4])val1.getValue());
	
	/* linear interpolation for rotation and scale */
	interp_v3_v3v3(loc, loc0, loc1, t);
	
	/* use simpe nlerp instead of slerp. it's faster and almost the same */
	interp_v4_v4v4(quat, quat0, quat1, t);
	normalize_qt(quat);
	
	interp_v3_v3v3(size, size0, size1, t);
	
	M44f result;
	loc_quat_size_to_mat4((float (*)[4])result.getValue(), loc, quat, size);
	
	return result;
}

/* ------------------------------------------------------------------------- */

/* These wrapper types allow calling all the interpolate_sample variants without knowing the semantics type
 * structs are required for this, since partial specialization does not work with functions.
 */

/* forward declaration */
template <typename ArraySampleT, typename SemanticT>
BLI_INLINE shared_ptr<ArraySampleT> interpolate_sample(const ArraySampleT &val0, const ArraySampleT &val1, float t, SemanticT semantic);

template <typename T, typename SemanticT>
struct InterpolateSampleCaller {
	BLI_INLINE T call(const T &val0, const T &val1, float t, SemanticT semantic)
	{
		return interpolate_sample(val0, val1, t, semantic);
	}
};

template <typename T>
struct InterpolateSampleCaller<T, InterpolateSemanticDefault> {
	BLI_INLINE T call(const T &val0, const T &val1, float t, InterpolateSemanticDefault)
	{
		return interpolate_sample(val0, val1, t);
	}
};

/* ------------------------------------------------------------------------- */

/* Scalar Properties */

template <typename PropT, typename SemanticT>
typename PropT::value_type abc_interpolate_sample_linear(const PropT &prop, chrono_t time, SemanticT semantic)
{
	typedef typename PropT::value_type value_type;
	
	ISampleSelector ss0(time, ISampleSelector::kFloorIndex);
	ISampleSelector ss1(time, ISampleSelector::kCeilIndex);
	
	index_t index0 = ss0.getIndex(prop.getTimeSampling(), prop.getNumSamples());
	index_t index1 = ss1.getIndex(prop.getTimeSampling(), prop.getNumSamples());
	if (index0 == index1) {
		/* no interpolation needed */
		return prop.getValue(ss0);
	}
	else {
		chrono_t time0 = prop.getTimeSampling()->getSampleTime(index0);
		chrono_t time1 = prop.getTimeSampling()->getSampleTime(index1);
		
		float t = (time1 > time0) ? (time - time0) / (time1 - time0) : 0.0f;
		return InterpolateSampleCaller<value_type, SemanticT>::call(prop.getValue(ss0), prop.getValue(ss1), t, semantic);
	}
}

template <typename PropT>
typename PropT::value_type abc_interpolate_sample_linear(const PropT &prop, chrono_t time)
{
	return abc_interpolate_sample_linear(prop, time, InterpolateSemanticDefault_None);
}


/* Array Properties */

template <typename ArraySampleT, typename SemanticT>
BLI_INLINE shared_ptr<ArraySampleT> interpolate_array_sample(const ArraySampleT &val0, const ArraySampleT &val1, float t, SemanticT semantic)
{
	typedef ArraySampleT sample_type;
	typedef shared_ptr<ArraySampleT> sample_ptr_type;
	typedef typename ArraySampleT::value_type value_type;
	
	size_t size0 = val0.size();
	size_t size1 = val1.size();
	size_t maxsize = size0 > size1 ? size0 : size1;
	size_t minsize = size0 < size1 ? size0 : size1;
	
	const value_type *data0 = val0.get();
	const value_type *data1 = val1.get();
	value_type *result = new value_type[maxsize];
	value_type *data = result;
	
	for (size_t i = 0; i < minsize; ++i) {
		*data = InterpolateSampleCaller<value_type, SemanticT>::call(*data0, *data1, t, semantic);
		++data;
		++data0;
		++data1;
	}
	
	if (size0 > minsize) {
		for (size_t i = minsize; i < size0; ++i) {
			*data = *data0;
			++data;
			++data0;
		}
	}
	else if (size1 > minsize) {
		for (size_t i = minsize; i < size1; ++i) {
			*data = *data1;
			++data;
			++data1;
		}
	}
	
	return sample_ptr_type(new sample_type(result, maxsize));
}

template <typename TraitsT, typename SemanticT>
shared_ptr<typename ITypedArrayProperty<TraitsT>::sample_type> abc_interpolate_sample_linear(const ITypedArrayProperty<TraitsT> &prop, chrono_t time, SemanticT semantic)
{
	ISampleSelector ss0(time, ISampleSelector::kFloorIndex);
	ISampleSelector ss1(time, ISampleSelector::kCeilIndex);
	
	index_t index0 = ss0.getIndex(prop.getTimeSampling(), prop.getNumSamples());
	index_t index1 = ss1.getIndex(prop.getTimeSampling(), prop.getNumSamples());
	if (index0 == index1) {
		/* no interpolation needed */
		return prop.getValue(ss0);
	}
	else {
		chrono_t time0 = prop.getTimeSampling()->getSampleTime(index0);
		chrono_t time1 = prop.getTimeSampling()->getSampleTime(index1);
		
		float t = (time1 > time0) ? (time - time0) / (time1 - time0) : 0.0f;
		return interpolate_array_sample(*prop.getValue(ss0), *prop.getValue(ss1), t, semantic);
	}
}

template <typename TraitsT>
shared_ptr<typename ITypedArrayProperty<TraitsT>::sample_type> abc_interpolate_sample_linear(const ITypedArrayProperty<TraitsT> &prop, chrono_t time)
{
	return abc_interpolate_sample_linear(prop, time, InterpolateSemanticDefault_None);
}

} /* namespace PTC */

#endif  /* PTC_ABC_INTERPOLATE_H */
