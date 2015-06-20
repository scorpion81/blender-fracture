/*
 * Copyright 2013, Blender Foundation.
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
#include <sstream>
#include <string>

#include <Alembic/Abc/IObject.h>
#include <Alembic/Abc/OObject.h>

#include "abc_mesh.h"
#include "abc_group.h"
#include "abc_interpolate.h"
#include "abc_object.h"
#include "abc_particles.h"
#include "abc_simdebug.h"

extern "C" {
#include "BLI_listbase.h"
#include "BLI_math.h"

#include "DNA_group_types.h"
#include "DNA_object_types.h"

#include "BKE_anim.h"
#include "BKE_cache_library.h"
#include "BKE_global.h"
#include "BKE_group.h"
#include "BKE_library.h"
#include "BKE_strands.h"
}

#include "util_task.h"

namespace PTC {

using namespace Abc;
using namespace AbcGeom;

AbcGroupWriter::AbcGroupWriter(const std::string &name, Group *group) :
    GroupWriter(group, name)
{
}

void AbcGroupWriter::init_abc()
{
	if (m_abc_object)
		return;
	
	m_abc_object = abc_archive()->add_id_object<OObject>((ID *)m_group);
}

void AbcGroupWriter::create_refs()
{
	GroupObject *gob = (GroupObject *)m_group->gobject.first;
	int i = 0;
	for (; gob; gob = gob->next, ++i) {
		OObject abc_object = abc_archive()->get_id_object((ID *)gob->ob);
		if (abc_object) {
			std::stringstream ss;
			ss << i;
			m_abc_object.addChildInstance(abc_object, std::string("group_object") + ss.str());
		}
	}
}

void AbcGroupWriter::write_sample()
{
	if (!m_abc_object)
		return;
}


AbcGroupReader::AbcGroupReader(const std::string &name, Group *group) :
    GroupReader(group, name)
{
}

void AbcGroupReader::init_abc(IObject object)
{
	if (m_abc_object)
		return;
	m_abc_object = object;
}

PTCReadSampleResult AbcGroupReader::read_sample_abc(chrono_t /*time*/)
{
	if (!m_abc_object)
		return PTC_READ_SAMPLE_INVALID;
	
	return PTC_READ_SAMPLE_EXACT;
}

/* ========================================================================= */

AbcDupligroupWriter::AbcDupligroupWriter(const std::string &name, EvaluationContext *eval_ctx, Scene *scene, Group *group, CacheLibrary *cachelib) :
    GroupWriter(group, name),
    m_eval_ctx(eval_ctx),
    m_scene(scene),
    m_cachelib(cachelib)
{
}

AbcDupligroupWriter::~AbcDupligroupWriter()
{
	for (IDWriterMap::iterator it = m_id_writers.begin(); it != m_id_writers.end(); ++it) {
		if (it->second)
			delete it->second;
	}
}

void AbcDupligroupWriter::init_abc()
{
	if (m_abc_group)
		return;
	
	m_abc_group = abc_archive()->add_id_object<OObject>((ID *)m_group);
}

void AbcDupligroupWriter::write_sample_object(Object *ob, bool write_data)
{
	AbcWriter *ob_writer;

	/* TODO(sergey): Optimize this out using RW mutex. */
	{
		thread_scoped_lock lock(m_init_mutex);
		ob_writer = find_id_writer((ID *)ob);
		if (!ob_writer) {
			bool do_mesh = write_data && (m_cachelib->data_types & CACHE_TYPE_DERIVED_MESH);
			bool do_hair = write_data && (m_cachelib->data_types & CACHE_TYPE_HAIR);

			ob_writer = new AbcObjectWriter(ob->id.name, m_scene, ob, do_mesh, do_hair);
			ob_writer->init(abc_archive());
			m_id_writers.insert(IDWriterPair((ID *)ob, ob_writer));
		}
	}

	ob_writer->write_sample();
}

void AbcDupligroupWriter::write_sample_dupli(DupliObject *dob, int index)
{
	OObject abc_object = abc_archive()->get_id_object((ID *)dob->ob);
	if (!abc_object)
		return;
	
	std::stringstream ss;
	ss << "DupliObject" << index;
	std::string name = ss.str();
	
	OObject abc_dupli = m_abc_group.getChild(name);
	OCompoundProperty props;
	OM44fProperty prop_matrix;
	OBoolProperty prop_visible;
	if (!abc_dupli) {
		abc_dupli = OObject(m_abc_group, name, 0);
		m_object_writers.push_back(abc_dupli.getPtr());
		props = abc_dupli.getProperties();
		
		abc_dupli.addChildInstance(abc_object, "object");
		
		prop_matrix = OM44fProperty(props, "matrix", abc_archive()->frame_sampling());
		m_property_writers.push_back(prop_matrix.getPtr());
		prop_visible = OBoolProperty(props, "visible", abc_archive()->frame_sampling());
		m_property_writers.push_back(prop_visible.getPtr());
	}
	else {
		props = abc_dupli.getProperties();
		
		prop_matrix = OM44fProperty(props.getProperty("matrix").getPtr()->asScalarPtr(), kWrapExisting);
		prop_visible = OBoolProperty(props.getProperty("visible").getPtr()->asScalarPtr(), kWrapExisting);
	}
	
	prop_matrix.set(M44f(dob->mat));
	
	bool show_object = (abc_archive()->use_render())? !(dob->ob->restrictflag & OB_RESTRICT_RENDER) : !(dob->ob->restrictflag & OB_RESTRICT_VIEW);
	bool visible = show_object && (!dob->no_draw);
	prop_visible.set(visible);
}

void AbcDupligroupWriter::write_sample()
{
	if (!m_abc_group)
		return;
	
	ListBase *duplilist = group_duplilist_ex(m_eval_ctx, m_scene, m_group, true);
	DupliObject *dob;
	int i;
	
	/* use a set to ensure each object is handled only once */
	std::set<Object*> objects;
	for (dob = (DupliObject *)duplilist->first; dob; dob = dob->next) {
		if (dob->ob)
			objects.insert(dob->ob);
	}

	/* tag objects for which to store data */
	BKE_cache_library_tag_used_objects(m_cachelib);

	UtilTaskPool pool;
	/* write actual object data: duplicator itself + all instanced objects */
	for (std::set<Object*>::const_iterator it = objects.begin(); it != objects.end(); ++it) {
		Object *ob = *it;
		bool write_data = (ob->id.flag & LIB_DOIT);
		pool.push(function_bind(&AbcDupligroupWriter::write_sample_object, this, ob, write_data));
	}

	pool.wait_work();

	/* write dupli instances */
	for (dob = (DupliObject *)duplilist->first, i = 0; dob; dob = dob->next, ++i) {
		write_sample_dupli(dob, i);
	}
	
	free_object_duplilist(duplilist);
}

AbcWriter *AbcDupligroupWriter::find_id_writer(ID *id) const
{
	IDWriterMap::const_iterator it = m_id_writers.find(id);
	if (it == m_id_writers.end())
		return NULL;
	else
		return it->second;
}

/* ------------------------------------------------------------------------- */

AbcDupliCacheWriter::AbcDupliCacheWriter(const std::string &name, Group *group, DupliCache *dupcache, int data_types, bool do_sim_debug) :
    GroupWriter(group, name),
    m_dupcache(dupcache),
    m_data_types(data_types),
    m_simdebug_writer(NULL)
{
	if (do_sim_debug) {
		BKE_sim_debug_data_set_enabled(true);
		if (_sim_debug_data)
			m_simdebug_writer = new AbcSimDebugWriter("sim_debug", _sim_debug_data);
	}
}

AbcDupliCacheWriter::~AbcDupliCacheWriter()
{
	for (IDWriterMap::iterator it = m_id_writers.begin(); it != m_id_writers.end(); ++it) {
		if (it->second)
			delete it->second;
	}
	
	if (m_simdebug_writer)
		delete m_simdebug_writer;
}

void AbcDupliCacheWriter::init_abc()
{
	if (m_abc_group)
		return;
	
	m_abc_group = abc_archive()->add_id_object<OObject>((ID *)m_group);
	
	if (m_simdebug_writer) {
		m_simdebug_writer->init(abc_archive());
		m_simdebug_writer->init_abc(abc_archive()->root());
	}
}

void AbcDupliCacheWriter::write_sample_object_data(DupliObjectData *data)
{
	AbcWriter *ob_writer = find_id_writer((ID *)data->ob);
	if (!ob_writer) {
		bool do_mesh = (m_data_types & CACHE_TYPE_DERIVED_MESH);
		bool do_hair = (m_data_types & CACHE_TYPE_HAIR);
		
		ob_writer = new AbcDupliObjectWriter(data->ob->id.name, data, do_mesh, do_hair);
		ob_writer->init(abc_archive());
		m_id_writers.insert(IDWriterPair((ID *)data->ob, ob_writer));
	}
	
	ob_writer->write_sample();
}

void AbcDupliCacheWriter::write_sample_dupli(DupliObject *dob, int index)
{
	OObject abc_object = abc_archive()->get_id_object((ID *)dob->ob);
	if (!abc_object)
		return;
	
	std::stringstream ss;
	ss << "DupliObject" << index;
	std::string name = ss.str();
	
	OObject abc_dupli = m_abc_group.getChild(name);
	OCompoundProperty props;
	OBoolProperty prop_visible;
	OM44fProperty prop_matrix;
	if (!abc_dupli) {
		abc_dupli = OObject(m_abc_group, name, 0);
		m_object_writers.push_back(abc_dupli.getPtr());
		props = abc_dupli.getProperties();
		
		abc_dupli.addChildInstance(abc_object, "object");
		
		prop_matrix = OM44fProperty(props, "matrix", abc_archive()->frame_sampling());
		m_property_writers.push_back(prop_matrix.getPtr());
		prop_visible = OBoolProperty(props, "visible", abc_archive()->frame_sampling());
		m_property_writers.push_back(prop_visible.getPtr());
	}
	else {
		props = abc_dupli.getProperties();
		
		prop_matrix = OM44fProperty(props.getProperty("matrix").getPtr()->asScalarPtr(), kWrapExisting);
		prop_visible = OBoolProperty(props.getProperty("visible").getPtr()->asScalarPtr(), kWrapExisting);
	}
	
	prop_matrix.set(M44f(dob->mat));
	
	bool show_object = (abc_archive()->use_render())? !(dob->ob->restrictflag & OB_RESTRICT_RENDER) : !(dob->ob->restrictflag & OB_RESTRICT_VIEW);
	bool visible = show_object && (!dob->no_draw);
	prop_visible.set(visible);
}

void AbcDupliCacheWriter::write_sample()
{
	if (!m_abc_group)
		return;
	
	DupliObject *dob;
	int i;
	
	struct DupliCacheIterator *iter = BKE_dupli_cache_iter_new(m_dupcache);
	for (; BKE_dupli_cache_iter_valid(iter); BKE_dupli_cache_iter_next(iter)) {
		DupliObjectData *data = BKE_dupli_cache_iter_get(iter);
		
		write_sample_object_data(data);
	}
	BKE_dupli_cache_iter_free(iter);
	
	/* write dupli instances */
	for (dob = (DupliObject *)m_dupcache->duplilist.first, i = 0; dob; dob = dob->next, ++i) {
		write_sample_dupli(dob, i);
	}
	
	if (m_simdebug_writer) {
		m_simdebug_writer->write_sample();
	}
}

AbcWriter *AbcDupliCacheWriter::find_id_writer(ID *id) const
{
	IDWriterMap::const_iterator it = m_id_writers.find(id);
	if (it == m_id_writers.end())
		return NULL;
	else
		return it->second;
}

/* ------------------------------------------------------------------------- */

AbcDupliCacheReader::AbcDupliCacheReader(const std::string &name, Group *group, DupliCache *dupli_cache,
                                         bool read_strands_motion, bool read_strands_children, bool read_sim_debug) :
    GroupReader(group, name),
    dupli_cache(dupli_cache),
    m_read_strands_motion(read_strands_motion),
    m_read_strands_children(read_strands_children),
    m_simdebug_reader(NULL)
{
	/* XXX this mapping allows fast lookup of existing objects in Blender data
	 * to associate with duplis. Later i may be possible to create instances of
	 * non-DNA data, but for the time being this is a requirement due to other code parts (drawing, rendering)
	 */
	build_object_map(G.main, group);
	
	if (read_sim_debug) {
		BKE_sim_debug_data_set_enabled(true);
		if (_sim_debug_data)
			m_simdebug_reader = new AbcSimDebugReader(_sim_debug_data);
	}
}

AbcDupliCacheReader::~AbcDupliCacheReader()
{
	if (m_simdebug_reader)
		delete m_simdebug_reader;
}

void AbcDupliCacheReader::init_abc(IObject /*object*/)
{
}

void AbcDupliCacheReader::read_dupligroup_object(IObject object, chrono_t time)
{
	if (GS(object.getName().c_str()) == ID_OB) {
		/* instances are handled later, we create true object data here */
		if (object.isInstanceDescendant())
			return;
		
		Object *b_ob = find_object(object.getName());
		if (!b_ob)
			return;
		
		/* Always add dupli data, even if no geometry is stored.
		 * Any missing geometry data will be replaced by the original uncached data in drawing/rendering if available.
		 */
		DupliObjectData *dupli_data = BKE_dupli_cache_add_object(dupli_cache, b_ob);
		insert_dupli_data(object.getPtr(), dupli_data);
		
		for (int i = 0; i < object.getNumChildren(); ++i) {
			IObject child = object.getChild(i);
			const MetaData &metadata = child.getMetaData();
			
			if (IPolyMeshSchema::matches(metadata)) {
				AbcDerivedMeshReader dm_reader("mesh", b_ob);
				dm_reader.init(abc_archive());
				dm_reader.init_abc(child);
				if (dm_reader.read_sample_abc(time) != PTC_READ_SAMPLE_INVALID) {
					BKE_dupli_object_data_set_mesh(dupli_data, dm_reader.acquire_result());
				}
				else {
					dm_reader.discard_result();
				}
			}
			else if (ICurvesSchema::matches(metadata)) {
				Strands *strands;
				StrandsChildren *children;
				BKE_dupli_object_data_find_strands(dupli_data, child.getName().c_str(), &strands, &children);
				
				AbcStrandsReader strands_reader(strands, children, m_read_strands_motion, m_read_strands_children);
				strands_reader.init(abc_archive());
				strands_reader.init_abc(child);
				if (strands_reader.read_sample_abc(time) != PTC_READ_SAMPLE_INVALID) {
					Strands *newstrands = strands_reader.acquire_result();
					if (strands && strands != newstrands) {
						/* reader can replace strands internally if topology does not match */
						BKE_strands_free(strands);
					}
					BKE_dupli_object_data_add_strands(dupli_data, child.getName().c_str(), newstrands);
					
					StrandsChildren *newchildren = strands_reader.child_reader().acquire_result();
					if (children && children != newchildren) {
						/* reader can replace strands internally if topology does not match */
						BKE_strands_children_free(children);
					}
					BKE_dupli_object_data_add_strands_children(dupli_data, child.getName().c_str(), newchildren);
				}
				else {
					strands_reader.discard_result();
					strands_reader.child_reader().discard_result();
				}
			}
		}
	}
}

void AbcDupliCacheReader::read_dupligroup_group(IObject abc_group, chrono_t time)
{
	ISampleSelector ss = get_frame_sample_selector(time);
	
	if (GS(abc_group.getName().c_str()) == ID_GR) {
		size_t num_child = abc_group.getNumChildren();
		
		for (size_t i = 0; i < num_child; ++i) {
			IObject abc_dupli = abc_group.getChild(i);
			ICompoundProperty props = abc_dupli.getProperties();
			
			IM44fProperty prop_matrix(props, "matrix", 0);
			M44f abc_matrix = abc_interpolate_sample_linear(prop_matrix, time);
			float matrix[4][4];
			memcpy(matrix, abc_matrix.getValue(), sizeof(matrix));
			
			IBoolProperty prop_visible(props, "visible", 0);
			bool visible = prop_visible.getValue(ss);
			
			IObject abc_dupli_object = abc_dupli.getChild("object");
			if (abc_dupli_object.isInstanceRoot()) {
				DupliObjectData *dupli_data = find_dupli_data(abc_dupli_object.getPtr());
				if (dupli_data) {
					DupliObject *dob = BKE_dupli_cache_add_instance(dupli_cache, matrix, dupli_data);
					dob->no_draw = !visible;
				}
			}
		}
	}
}

PTCReadSampleResult AbcDupliCacheReader::read_sample_abc(chrono_t time)
{
	IObject abc_top = abc_archive()->root();
	IObject abc_group = abc_archive()->get_id_object((ID *)m_group);
	if (!abc_group)
		return PTC_READ_SAMPLE_INVALID;
	
	/* first create shared object data */
	for (size_t i = 0; i < abc_top.getNumChildren(); ++i) {
		read_dupligroup_object(abc_top.getChild(i), time);
	}
	
	BKE_dupli_cache_clear_instances(dupli_cache);
	
	/* now generate dupli instances for the group */
	read_dupligroup_group(abc_group, time);
	
	// XXX reader init is a mess ...
	if (m_simdebug_reader) {
		if (abc_top.getChildHeader("sim_debug")) {
			m_simdebug_reader->init(abc_archive());
			m_simdebug_reader->init_abc(abc_top.getChild("sim_debug"));
			
			m_simdebug_reader->read_sample_abc(time);
		}
	}
	
	return PTC_READ_SAMPLE_EXACT;
}

DupliObjectData *AbcDupliCacheReader::find_dupli_data(ObjectReaderPtr ptr) const
{
	DupliMap::const_iterator it = dupli_map.find(ptr);
	if (it == dupli_map.end())
		return NULL;
	else
		return it->second;
}

void AbcDupliCacheReader::insert_dupli_data(ObjectReaderPtr ptr, DupliObjectData *data)
{
	dupli_map.insert(DupliPair(ptr, data));
}

void AbcDupliCacheReader::build_object_map(Main *bmain, Group *group)
{
	BKE_main_id_tag_idcode(bmain, ID_OB, false);
	BKE_main_id_tag_idcode(bmain, ID_GR, false);
	object_map.clear();
	
	build_object_map_add_group(group);
}

Object *AbcDupliCacheReader::find_object(const std::string &name) const
{
	ObjectMap::const_iterator it = object_map.find(name);
	if (it == object_map.end())
		return NULL;
	else
		return it->second;
}

void AbcDupliCacheReader::build_object_map_add_group(Group *group)
{
	if (group->id.flag & LIB_DOIT)
		return;
	group->id.flag |= LIB_DOIT;
	
	for (GroupObject *gob = (GroupObject *)group->gobject.first; gob; gob = gob->next) {
		Object *ob = gob->ob;
		if (ob->id.flag & LIB_DOIT)
			continue;
		ob->id.flag |= LIB_DOIT;
		object_map.insert(ObjectPair(ob->id.name, ob));
		
		if ((ob->transflag & OB_DUPLIGROUP) && ob->dup_group) {
			build_object_map_add_group(ob->dup_group);
		}
	}
}

/* ------------------------------------------------------------------------- */

AbcDupliObjectWriter::AbcDupliObjectWriter(const std::string &name, DupliObjectData *dupdata, bool do_mesh, bool do_strands) :
    ObjectWriter(dupdata->ob, name),
    m_dupdata(dupdata),
    m_do_strands(do_strands),
    m_dm_writer(0)
{
	if (do_mesh) {
		if (m_ob && m_ob->type == OB_MESH) {
			m_dm_writer = new AbcDerivedMeshWriter("mesh", dupdata->ob, &dupdata->dm);
		}
	}
}

AbcStrandsWriter *AbcDupliObjectWriter::find_strands_writer(const std::string &name) const
{
	StrandsWriters::const_iterator it = m_strands_writers.find(name);
	return (it != m_strands_writers.end()) ? it->second : NULL;
}

AbcStrandsWriter *AbcDupliObjectWriter::add_strands_writer(const std::string &name)
{
	StrandsWriters::const_iterator it = m_strands_writers.find(name);
	if (it != m_strands_writers.end()) {
		return it->second;
	}
	else {
		AbcStrandsWriter *writer = new AbcStrandsWriter(name, m_dupdata);
		m_strands_writers.insert(StrandsWritersPair(name, writer));
		
		writer->init(abc_archive());
		writer->init_abc(m_abc_object);
		
		return writer;
	}
}

AbcDupliObjectWriter::~AbcDupliObjectWriter()
{
	if (m_dm_writer)
		delete m_dm_writer;
	for (StrandsWriters::iterator it = m_strands_writers.begin(); it != m_strands_writers.end(); ++it) {
		if (it->second)
			delete it->second;
	}
}

void AbcDupliObjectWriter::init_abc()
{
	if (m_abc_object)
		return;
	
	m_abc_object = abc_archive()->add_id_object<OObject>((ID *)m_ob);
	
	if (m_dm_writer) {
		/* XXX not nice */
		m_dm_writer->init(abc_archive());
		m_dm_writer->init_abc(m_abc_object);
	}
}

void AbcDupliObjectWriter::write_sample()
{
	if (!m_abc_object)
		return;
	
	if (m_dm_writer) {
		m_dm_writer->write_sample();
	}
	
	if (m_do_strands) {
		int index = 0;
		for (DupliObjectDataStrands *link = (DupliObjectDataStrands *)m_dupdata->strands.first;
		     link;
		     link = link->next, ++index) {
			AbcStrandsWriter *writer = add_strands_writer(link->name);
			writer->write_sample();
		}
	}
}

/* ------------------------------------------------------------------------- */

AbcDupliObjectReader::AbcDupliObjectReader(const std::string &name, Object *ob, DupliObjectData *dupli_data,
                                           bool read_strands_motion, bool read_strands_children) :
    ObjectReader(ob, name),
    dupli_data(dupli_data),
    m_read_strands_motion(read_strands_motion),
    m_read_strands_children(read_strands_children)
{
}

AbcDupliObjectReader::~AbcDupliObjectReader()
{
}

void AbcDupliObjectReader::init(ReaderArchive *archive)
{
	AbcReader::init(archive);
	
	if (abc_archive()->root().getChildHeader(m_name))
		m_abc_object = abc_archive()->root().getChild(m_name);
}

void AbcDupliObjectReader::init_abc(IObject object)
{
	m_abc_object = object;
}

void AbcDupliObjectReader::read_dupligroup_object(IObject object, chrono_t time)
{
	if (GS(object.getName().c_str()) == ID_OB) {
		/* instances are handled later, we create true object data here */
		if (object.isInstanceDescendant())
			return;
		
		BKE_dupli_object_data_init(dupli_data, m_ob);
		
		for (int i = 0; i < object.getNumChildren(); ++i) {
			IObject child = object.getChild(i);
			const MetaData &metadata = child.getMetaData();
			
			if (IPolyMeshSchema::matches(metadata)) {
				AbcDerivedMeshReader dm_reader("mesh", m_ob);
				dm_reader.init(abc_archive());
				dm_reader.init_abc(child);
				if (dm_reader.read_sample_abc(time) != PTC_READ_SAMPLE_INVALID) {
					BKE_dupli_object_data_set_mesh(dupli_data, dm_reader.acquire_result());
				}
				else {
					dm_reader.discard_result();
				}
			}
			else if (ICurvesSchema::matches(metadata)) {
				Strands *strands;
				StrandsChildren *children;
				BKE_dupli_object_data_find_strands(dupli_data, child.getName().c_str(), &strands, &children);
				
				AbcStrandsReader strands_reader(strands, children, m_read_strands_motion, m_read_strands_children);
				strands_reader.init(abc_archive());
				strands_reader.init_abc(child);
				if (strands_reader.read_sample_abc(time) != PTC_READ_SAMPLE_INVALID) {
					Strands *newstrands = strands_reader.acquire_result();
					if (strands && strands != newstrands) {
						/* reader can replace strands internally if topology does not match */
						BKE_strands_free(strands);
					}
					BKE_dupli_object_data_add_strands(dupli_data, child.getName().c_str(), newstrands);
					
					StrandsChildren *newchildren = strands_reader.child_reader().acquire_result();
					if (children && children != newchildren) {
						/* reader can replace strands internally if topology does not match */
						BKE_strands_children_free(children);
					}
					BKE_dupli_object_data_add_strands_children(dupli_data, child.getName().c_str(), newchildren);
				}
				else {
					strands_reader.discard_result();
					strands_reader.child_reader().discard_result();
				}
			}
		}
	}
}

PTCReadSampleResult AbcDupliObjectReader::read_sample_abc(chrono_t time)
{
	if (!m_abc_object)
		return PTC_READ_SAMPLE_INVALID;
	
	read_dupligroup_object(m_abc_object, time);
	
	return PTC_READ_SAMPLE_EXACT;
}

DupliObjectData *AbcDupliObjectReader::find_dupli_data(ObjectReaderPtr ptr) const
{
	DupliMap::const_iterator it = dupli_map.find(ptr);
	if (it == dupli_map.end())
		return NULL;
	else
		return it->second;
}

void AbcDupliObjectReader::insert_dupli_data(ObjectReaderPtr ptr, DupliObjectData *data)
{
	dupli_map.insert(DupliPair(ptr, data));
}

} /* namespace PTC */
