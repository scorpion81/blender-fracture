//-*****************************************************************************
//
// Copyright (c) 2009-2013,
//  Sony Pictures Imageworks, Inc. and
//  Industrial Light & Magic, a division of Lucasfilm Entertainment Company Ltd.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of Sony Pictures Imageworks, nor
// Industrial Light & Magic nor the names of their contributors may be used
// to endorse or promote products derived from this software without specific
// prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//-*****************************************************************************

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

#include <map>

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/Util/All.h>
#include <Alembic/Abc/TypedPropertyTraits.h>

#include "alembic.h"

extern "C" {
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "BKE_cache_library.h"
}

using namespace ::Alembic::AbcGeom;

namespace PTC {

static void slice_properties(ICompoundProperty iParent, OCompoundProperty out_parent, TimeSamplingPtr time_sampling, const AbcArchiveFrameFilter &filter);

static void slice_array_property(IArrayProperty iProp, OCompoundProperty out_parent, TimeSamplingPtr time_sampling, const AbcArchiveFrameFilter &filter)
{
	OArrayProperty out(out_parent, iProp.getName(), iProp.getDataType(), iProp.getMetaData(), time_sampling);
	
	ArrayPropertyReaderPtr reader = iProp.getPtr();
	ArrayPropertyWriterPtr writer = out.getPtr();
	
	size_t num_samples = iProp.getNumSamples();
	if (num_samples == 0)
		return;
	
//	index_t istart = reader->getFloorIndex(start).first;
//	index_t iend = reader->getFloorIndex(end).first;
	
	char *buf = NULL;
	
	for (index_t index = 0; index < iProp.getNumSamples(); ++index) {
		chrono_t time = time_sampling->getSampleTime(index);
		if (filter.use_time(time) || writer->getNumSamples() == 0) {
			ArraySamplePtr sample_ptr;
			reader->getSample(index, sample_ptr);
			
			writer->setSample(*sample_ptr);
		}
		else {
			writer->setFromPreviousSample();
		}
	}
	
	if (buf)
		delete[] buf;
}

static void slice_scalar_property(IScalarProperty iProp, OCompoundProperty out_parent, TimeSamplingPtr time_sampling, const AbcArchiveFrameFilter &filter)
{
	OScalarProperty out(out_parent, iProp.getName(), iProp.getDataType(), iProp.getMetaData(), time_sampling);
	
	ScalarPropertyReaderPtr reader = iProp.getPtr();
	ScalarPropertyWriterPtr writer = out.getPtr();
	size_t num_bytes = reader->getDataType().getNumBytes();
	
	size_t num_samples = iProp.getNumSamples();
	if (num_samples == 0)
		return;
	
//	index_t istart = reader->getFloorIndex(start).first;
//	index_t iend = reader->getFloorIndex(end).first;
	
	char *buf = new char[num_bytes];
	
#if 0
	if (istart > ostart) {
		/* fill the gap between start indices with the first sample,
		 * so that output sample times match input sample times as close as possible.
		 */
		for (index_t index = istart)
	}
#endif
	
	for (index_t index = 0; index < iProp.getNumSamples(); ++index) {
		chrono_t time = time_sampling->getSampleTime(index);
		if (filter.use_time(time) || writer->getNumSamples() == 0) {
			reader->getSample(index, (void*)buf);
			
			writer->setSample((void*)buf);
		}
		else {
			writer->setFromPreviousSample();
		}
	}
	
	delete[] buf;
}

static void slice_compound_property(ICompoundProperty iProp, OCompoundProperty out_parent, TimeSamplingPtr time_sampling, const AbcArchiveFrameFilter &filter)
{
	OCompoundProperty out(out_parent, iProp.getName(), iProp.getMetaData());
	
	slice_properties(iProp, out, time_sampling, filter);
}

static void slice_properties(ICompoundProperty iParent, OCompoundProperty out_parent, TimeSamplingPtr time_sampling, const AbcArchiveFrameFilter &filter)
{
	for (size_t i = 0 ; i < iParent.getNumProperties() ; i++) {
		PropertyHeader header = iParent.getPropertyHeader(i);
		
		if (header.isCompound()) {
			slice_compound_property(ICompoundProperty(iParent, header.getName()), out_parent, time_sampling, filter);
		}
		else if (header.isScalar()) {
			slice_scalar_property(IScalarProperty(iParent, header.getName()), out_parent, time_sampling, filter);
		}
		else {
			BLI_assert(header.isArray());
			slice_array_property(IArrayProperty(iParent, header.getName()), out_parent, time_sampling, filter);
		}
	}
}

typedef std::map<ObjectReaderPtr, ObjectWriterPtr> ObjectMap;
typedef std::pair<ObjectReaderPtr, ObjectWriterPtr> ObjectPair;

static void slice_object(IObject iObj, OObject out, TimeSamplingPtr time_sampling, const AbcArchiveFrameFilter &filter, ObjectMap &object_map)
{
	// Get the properties.
	slice_properties(iObj.getProperties(), out.getProperties(), time_sampling, filter);
	
	// now the child objects
	for (size_t i = 0 ; i < iObj.getNumChildren() ; i++) {
		const ObjectHeader &child_header = iObj.getChildHeader(i);
		IObject child = IObject(iObj, child_header.getName());
		
		/* Note: child instances are added later, once all actual objects have been copied */
		if (!child.isInstanceRoot()) {
			/* XXX reuse if the output object already exists.
			 * This should not happen, but currently some root objects are created
			 * in advance when opening writer archives. In the future these will not be needed
			 * and this check will become unnecessary.
			 */
			OObject out_child = out.getChild(child_header.getName());
			if (!out_child)
				out_child = OObject(out, child_header.getName(), child_header.getMetaData());
			object_map[child.getPtr()] = out_child.getPtr();
			
			slice_object(child, out_child, time_sampling, filter, object_map);
		}
	}
}

static void slice_object_instances(IObject iObj, OObject out, const ObjectMap &object_map)
{
	// now the child objects
	for (size_t i = 0 ; i < iObj.getNumChildren() ; i++) {
		const ObjectHeader &child_header = iObj.getChildHeader(i);
		IObject child = IObject(iObj, child_header.getName());
		
		if (child.isInstanceRoot()) {
			ObjectMap::const_iterator it = object_map.find(child.getPtr());
			BLI_assert(it != object_map.end());
			OObject out_target(it->second, kWrapExisting);
			
			out.addChildInstance(out_target, child_header.getName());
		}
		else {
			OObject out_child(out.getChild(child_header.getName()).getPtr(), kWrapExisting);
			slice_object_instances(child, out_child, object_map);
		}
	}
}

void abc_archive_slice(IArchive in, OArchive out, TimeSamplingPtr time_sampling, const AbcArchiveFrameFilter &filter)
{
	ObjectMap object_map;
	
	slice_object(in.getTop(), out.getTop(), time_sampling, filter, object_map);
	slice_object_instances(in.getTop(), out.getTop(), object_map);
}

} /* namespace PTC */
