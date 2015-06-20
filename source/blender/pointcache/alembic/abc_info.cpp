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

#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/Util/All.h>
#include <Alembic/Abc/TypedPropertyTraits.h>

#include <sstream>

#include "alembic.h"

extern "C" {
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "DNA_ID.h"

#include "BKE_cache_library.h"
#include "BKE_idprop.h"
}

using namespace ::Alembic::AbcGeom;

namespace PTC {

static void metadata_from_idprops(MetaData &md, IDProperty *prop);

void abc_metadata_from_idprops_group(MetaData &md, IDProperty *prop)
{
	if (!prop)
		return;
	
	IDProperty *child;
	for (child = (IDProperty *)prop->data.group.first; child; child = child->next)
		metadata_from_idprops(md, child);
}

static void metadata_from_idprops(MetaData &md, IDProperty *prop)
{
	/* skip default metadata entries, these are set explicitly */
	std::string key(prop->name);
	if (key.compare(kApplicationNameKey)==0 || key.compare(kDateWrittenKey)==0 || key.compare(kUserDescriptionKey)==0)
		return;
	
	switch (prop->type) {
#if 0 /* don't support recursion yet */
		case IDP_GROUP: {
			metadata_from_idprops_group(md, child);
			break;
		}
		
		case IDP_ARRAY: {
			if (prop->data.pointer) {
				IDProperty **array = (IDProperty **)prop->data.pointer;
				for (int a = 0; a < prop->len; a++)
					metadata_from_idprops(md, array[a]);
			}
			break;
		}
#endif
		
		/* only string properties are used */
		case IDP_STRING: {
			md.set(prop->name, IDP_String(prop));
			break;
		}
	}
}

void abc_metadata_to_idprops_group(const MetaData &md, IDProperty *prop)
{
	if (!prop)
		return;
	
	for (MetaData::const_iterator it = md.begin(); it != md.end(); ++it) {
		const std::string &key = it->first;
		const std::string &value = it->second;
		
		/* skip default metadata entries, these are stored in CacheArchiveInfo */
		if (key.compare(kApplicationNameKey)==0 || key.compare(kDateWrittenKey)==0 || key.compare(kUserDescriptionKey)==0)
			continue;
		
		IDPropertyTemplate val;
		val.string.str = value.c_str();
		val.string.len = value.length();
		IDP_ReplaceInGroup(prop, IDP_New(IDP_STRING, &val, key.c_str()));
	}
}

MetaData abc_create_archive_info(const char *app_name, const char *description, const struct tm *t, IDProperty *props)
{
	MetaData md;
	
	md.set(kApplicationNameKey, app_name);
	md.set(kUserDescriptionKey, description);
	
	if (!t) {
		time_t curtime = time(NULL);
		t = localtime(&curtime);
	}
	
	if (t) {
		char buf[256];
		strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", t);
		md.set(kDateWrittenKey, buf);
	}
	
	/* store custom properties as metadata */
	if (props && props->type == IDP_GROUP)
		abc_metadata_from_idprops_group(md, props);
	
	return md;
}

/* ========================================================================= */

struct stringstream {
	stringstream(void (*cb)(void *, const char *), void *userdata) :
	    cb(cb),
	    userdata(userdata)
	{
	}
	
	void (*cb)(void *, const char *);
	void *userdata;
	
	template <typename T>
	friend stringstream& operator << (stringstream &stream, T s);
};

template <typename T>
stringstream& operator << (stringstream &stream, T s)
{
	std::stringstream ss;
	ss << s;
	stream.cb(stream.userdata, ss.str().c_str());
	return stream;
}

static const std::string g_sep(";");
static const std::string g_endl("\n");

static void info_stream_properties(stringstream &ss, ICompoundProperty, std::string &);

template <class PROP>
static void info_stream_array_property(stringstream &ss, PROP iProp, const std::string &iIndent)
{
	std::string ptype = "ArrayProperty ";
	size_t asize = 0;
	
	AbcA::ArraySamplePtr samp;
	index_t maxSamples = iProp.getNumSamples();
	for (index_t i = 0 ; i < maxSamples; ++i) {
		iProp.get(samp, ISampleSelector(i));
		asize = samp->size();
	}
	
	std::string mdstring = "interpretation=";
	mdstring += iProp.getMetaData().get("interpretation");
	
	std::stringstream dtype;
	dtype << "datatype=";
	dtype << iProp.getDataType();
	
	std::stringstream asizestr;
	asizestr << ";arraysize=";
	asizestr << asize;
	
	mdstring += g_sep;
	
	mdstring += dtype.str();
	
	mdstring += asizestr.str();
	
	ss << iIndent << "  " << ptype << "name=" << iProp.getName()
	   << g_sep << mdstring << g_sep << "numsamps="
	   << iProp.getNumSamples() << g_endl;
}

template <class PROP>
static void info_stream_scalar_property(stringstream &ss, PROP iProp, const std::string &iIndent)
{
	std::string ptype = "ScalarProperty ";
	size_t asize = 0;
	
	const AbcA::DataType &dt = iProp.getDataType();
	const Alembic::Util ::uint8_t extent = dt.getExtent();
	Alembic::Util::Dimensions dims(extent);
	AbcA::ArraySamplePtr samp = AbcA::AllocateArraySample( dt, dims );
	index_t maxSamples = iProp.getNumSamples();
	for (index_t i = 0 ; i < maxSamples; ++i) {
		iProp.get(const_cast<void *>(samp->getData()), ISampleSelector(i));
		asize = samp->size();
	}
	
	std::string mdstring = "interpretation=";
	mdstring += iProp.getMetaData().get("interpretation");
	
	std::stringstream dtype;
	dtype << "datatype=";
	dtype << dt;
	
	std::stringstream asizestr;
	asizestr << ";arraysize=";
	asizestr << asize;
	
	mdstring += g_sep;
	
	mdstring += dtype.str();
	
	mdstring += asizestr.str();
	
	ss << iIndent << "  " << ptype << "name=" << iProp.getName()
	   << g_sep << mdstring << g_sep << "numsamps="
	   << iProp.getNumSamples() << g_endl;
}

static void info_stream_compound_property(stringstream &ss, ICompoundProperty iProp, std::string &ioIndent)
{
	std::string oldIndent = ioIndent;
	ioIndent += "  ";
	
	std::string interp = "schema=";
	interp += iProp.getMetaData().get("schema");
	
	ss << ioIndent << "CompoundProperty " << "name=" << iProp.getName()
	   << g_sep << interp << g_endl;
	
	info_stream_properties(ss, iProp, ioIndent);
	
	ioIndent = oldIndent;
}

static void info_stream_properties(stringstream &ss, ICompoundProperty iParent, std::string &ioIndent )
{
	std::string oldIndent = ioIndent;
	for (size_t i = 0 ; i < iParent.getNumProperties() ; i++) {
		PropertyHeader header = iParent.getPropertyHeader(i);
		
		if (header.isCompound()) {
			info_stream_compound_property(ss, ICompoundProperty(iParent, header.getName()), ioIndent);
		}
		else if (header.isScalar()) {
			info_stream_scalar_property(ss, IScalarProperty(iParent, header.getName()), ioIndent);
		}
		else {
			BLI_assert(header.isArray());
			info_stream_array_property(ss, IArrayProperty(iParent, header.getName()), ioIndent);
		}
	}
	
	ioIndent = oldIndent;
}

static void info_stream_object(stringstream &ss, IObject iObj, std::string iIndent)
{
	// Object has a name, a full name, some meta data,
	// and then it has a compound property full of properties.
	std::string path = iObj.getFullName();
	
	if (iObj.isInstanceRoot()) {
		if (path != "/") {
			ss << "Object " << "name=" << path
			   << " [Instance " << iObj.instanceSourcePath() << "]"
			   << g_endl;
		}
	}
	else if (iObj.isInstanceDescendant()) {
		/* skip non-root instances to avoid repetition */
		return;
	}
	else {
		if (path != "/") {
			ss << "Object " << "name=" << path << g_endl;
		}
		
		// Get the properties.
		ICompoundProperty props = iObj.getProperties();
		info_stream_properties(ss, props, iIndent);
		
		// now the child objects
		for (size_t i = 0 ; i < iObj.getNumChildren() ; i++) {
			info_stream_object(ss, IObject(iObj, iObj.getChildHeader(i).getName()), iIndent);
		}
	}
}

void abc_archive_info_stream(IArchive &archive, void (*stream)(void *, const char *), void *userdata)
{
	stringstream ss(stream, userdata);
	
	ss << "Alembic Archive Info for "
	   << Alembic::AbcCoreAbstract::GetLibraryVersion()
	   << g_endl;
	
	std::string appName;
	std::string libraryVersionString;
	Alembic::Util::uint32_t libraryVersion;
	std::string whenWritten;
	std::string userDescription;
	GetArchiveInfo(archive,
	               appName,
	               libraryVersionString,
	               libraryVersion,
	               whenWritten,
	               userDescription);
	
	if (appName != "") {
		ss << "  file written by: " << appName << g_endl;
		ss << "  using Alembic : " << libraryVersionString << g_endl;
		ss << "  written on : " << whenWritten << g_endl;
		ss << "  user description : " << userDescription << g_endl;
		ss << g_endl;
	}
	else {
//		ss << argv[1] << g_endl;
		ss << "  (file doesn't have any ArchiveInfo)"
		   << g_endl;
		ss << g_endl;
	}
	
	info_stream_object(ss, archive.getTop(), "");
}

/* ========================================================================= */

static void info_nodes_properties(CacheArchiveInfo *info, ICompoundProperty, CacheArchiveInfoNode *parent, bool calc_bytes_size);

template <class PROP>
static void info_nodes_array_property(CacheArchiveInfo *info, PROP iProp, CacheArchiveInfoNode *parent, bool calc_bytes_size)
{
	CacheArchiveInfoNode *node = BKE_cache_archive_info_add_node(info, parent, eCacheArchiveInfoNode_Type_ArrayProperty, iProp.getName().c_str());
	
	index_t num_samples = iProp.getNumSamples();
	
	const DataType &datatype = iProp.getDataType();
	
	node->num_samples = num_samples;
	BLI_strncpy(node->datatype_name, PODName(datatype.getPod()), sizeof(node->datatype_name));
	node->datatype_extent = (short)datatype.getExtent();
	
	if (calc_bytes_size) {
		size_t max_array_size = 0;
		size_t tot_array_size = 0;
		for (index_t i = 0; i < num_samples; ++i) {
			AbcA::ArraySamplePtr samp;
			iProp.get(samp, ISampleSelector(i));
			size_t array_size = samp->size();
			max_array_size = std::max(max_array_size, array_size);
			tot_array_size += array_size;
		}
		node->bytes_size = datatype.getNumBytes() * tot_array_size;
		node->array_size = max_array_size;
		
		if (parent)
			parent->bytes_size += node->bytes_size;
	}
}

template <class PROP>
static void info_nodes_scalar_property(CacheArchiveInfo *info, PROP iProp, CacheArchiveInfoNode *parent, bool calc_bytes_size)
{
	CacheArchiveInfoNode *node = BKE_cache_archive_info_add_node(info, parent, eCacheArchiveInfoNode_Type_ScalarProperty, iProp.getName().c_str());
	
	index_t num_samples = iProp.getNumSamples();
	
	const DataType &datatype = iProp.getDataType();
	
	node->num_samples = num_samples;
	BLI_strncpy(node->datatype_name, PODName(datatype.getPod()), sizeof(node->datatype_name));
	node->datatype_extent = (short)datatype.getExtent();
	
	if (calc_bytes_size) {
		node->bytes_size = datatype.getNumBytes() * num_samples;
		
		if (parent)
			parent->bytes_size += node->bytes_size;
	}
}

static void info_nodes_compound_property(CacheArchiveInfo *info, ICompoundProperty iProp, CacheArchiveInfoNode *parent, bool calc_bytes_size)
{
	CacheArchiveInfoNode *node = BKE_cache_archive_info_add_node(info, parent, eCacheArchiveInfoNode_Type_CompoundProperty, iProp.getName().c_str());
	
	info_nodes_properties(info, iProp, node, calc_bytes_size);
	
	if (calc_bytes_size && parent)
		parent->bytes_size += node->bytes_size;
}

static void info_nodes_properties(CacheArchiveInfo *info, ICompoundProperty iParent, CacheArchiveInfoNode *parent, bool calc_bytes_size)
{
	for (size_t i = 0 ; i < iParent.getNumProperties() ; i++) {
		PropertyHeader header = iParent.getPropertyHeader(i);
		
		if (header.isCompound()) {
			info_nodes_compound_property(info, ICompoundProperty(iParent, header.getName()), parent, calc_bytes_size);
		}
		else if (header.isScalar()) {
			info_nodes_scalar_property(info, IScalarProperty(iParent, header.getName()), parent, calc_bytes_size);
		}
		else {
			BLI_assert(header.isArray());
			info_nodes_array_property(info, IArrayProperty(iParent, header.getName()), parent, calc_bytes_size);
		}
	}
}

static void info_nodes_object(CacheArchiveInfo *info, IObject iObj, CacheArchiveInfoNode *parent, bool calc_bytes_size)
{
	CacheArchiveInfoNode *node = BKE_cache_archive_info_add_node(info, parent, eCacheArchiveInfoNode_Type_Object, iObj.getName().c_str());
	
	if (iObj.isInstanceRoot()) {
	}
	else if (iObj.isInstanceDescendant()) {
	}
	else {
		// Get the properties.
		ICompoundProperty props = iObj.getProperties();
		info_nodes_properties(info, props, node, calc_bytes_size);
		
		// now the child objects
		for (size_t i = 0 ; i < iObj.getNumChildren() ; i++) {
			info_nodes_object(info, IObject(iObj, iObj.getChildHeader(i).getName()), node, calc_bytes_size);
		}
	}
	
	if (calc_bytes_size && parent)
		parent->bytes_size += node->bytes_size;
}

void abc_archive_info_nodes(IArchive &archive, CacheArchiveInfo *info, IDProperty *metadata, bool calc_nodes, bool calc_bytes_size)
{
//	ss << "Alembic Archive Info for "
//	   << Alembic::AbcCoreAbstract::GetLibraryVersion()
//	   << g_endl;
	
	std::string appName;
	std::string libraryVersionString;
	uint32_t libraryVersion;
	std::string whenWritten;
	std::string userDescription;
	GetArchiveInfo(archive, appName, libraryVersionString, libraryVersion, whenWritten, userDescription);
	
	if (appName != "") {
		BLI_strncpy(info->app_name, appName.c_str(), sizeof(info->app_name));
		BLI_strncpy(info->date_written, whenWritten.c_str(), sizeof(info->date_written));
		BLI_strncpy(info->description, userDescription.c_str(), sizeof(info->description));
	}
	else {
		info->app_name[0] = '\0';
		info->date_written[0] = '\0';
		info->description[0] = '\0';
	}
	
	if (metadata)
		abc_metadata_to_idprops_group(archive.getPtr()->getMetaData(), metadata);
	
	if (calc_nodes)
		info_nodes_object(info, archive.getTop(), NULL, calc_bytes_size);
}

} /* namespace PTC */
