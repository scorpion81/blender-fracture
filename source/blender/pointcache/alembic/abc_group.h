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

#ifndef PTC_ABC_GROUP_H
#define PTC_ABC_GROUP_H

#include "ptc_types.h"

#include "abc_reader.h"
#include "abc_schema.h"
#include "abc_writer.h"

#include "util_thread.h"

struct CacheLibrary;
struct DupliCache;
struct DupliObject;
struct DupliObjectData;
struct Group;
struct Object;
struct Scene;

namespace PTC {

class AbcDerivedMeshWriter;
class AbcStrandsWriter;
class AbcSimDebugWriter;
class AbcSimDebugReader;

class AbcGroupWriter : public GroupWriter, public AbcWriter {
public:
	AbcGroupWriter(const std::string &name, Group *group);
	
	void init_abc();
	void create_refs();
	
	void write_sample();
	
private:
	Abc::OObject m_abc_object;
};

class AbcGroupReader : public GroupReader, public AbcReader {
public:
	AbcGroupReader(const std::string &name, Group *group);
	
	void init_abc(Abc::IObject object);
	
	PTCReadSampleResult read_sample_abc(chrono_t time);
	
private:
	Abc::IObject m_abc_object;
};

/* ========================================================================= */

class AbcDupligroupWriter : public GroupWriter, public AbcWriter {
public:
	typedef std::vector<Abc::ObjectWriterPtr> ObjectWriterList;
	typedef std::vector<Abc::BasePropertyWriterPtr> PropertyWriterList;
	
	typedef std::map<ID *, AbcWriter *> IDWriterMap;
	typedef std::pair<ID *, AbcWriter *> IDWriterPair;
	
	AbcDupligroupWriter(const std::string &name, EvaluationContext *eval_ctx, Scene *scene, Group *group, CacheLibrary *cachelib);
	~AbcDupligroupWriter();
	
	void init_abc();
	
	void write_sample();
	void write_sample_object(Object *ob, bool write_data);
	void write_sample_dupli(DupliObject *dob, int index);
	
	AbcWriter *find_id_writer(ID *id) const;
	
private:
	EvaluationContext *m_eval_ctx;
	Scene *m_scene;
	CacheLibrary *m_cachelib;
	
	Abc::OObject m_abc_group;
	ObjectWriterList m_object_writers;
	PropertyWriterList m_property_writers;
	IDWriterMap m_id_writers;
	thread_mutex m_init_mutex;
};

class AbcDupliCacheWriter : public GroupWriter, public AbcWriter {
public:
	typedef std::vector<Abc::ObjectWriterPtr> ObjectWriterList;
	typedef std::vector<Abc::BasePropertyWriterPtr> PropertyWriterList;
	
	typedef std::map<ID *, AbcWriter *> IDWriterMap;
	typedef std::pair<ID *, AbcWriter *> IDWriterPair;
	
	AbcDupliCacheWriter(const std::string &name, Group *group, DupliCache *dupcache, int data_types, bool do_sim_debug = false);
	~AbcDupliCacheWriter();
	
	void init_abc();
	
	void write_sample();
	void write_sample_object_data(DupliObjectData *data);
	void write_sample_dupli(DupliObject *dob, int index);
	
	AbcWriter *find_id_writer(ID *id) const;
	
private:
	DupliCache *m_dupcache;
	int m_data_types;
	
	Abc::OObject m_abc_group;
	ObjectWriterList m_object_writers;
	PropertyWriterList m_property_writers;
	IDWriterMap m_id_writers;
	AbcSimDebugWriter *m_simdebug_writer;
};

class AbcDupliCacheReader : public GroupReader, public AbcReader {
public:
	typedef std::map<Abc::ObjectReaderPtr, DupliObjectData*> DupliMap;
	typedef std::pair<Abc::ObjectReaderPtr, DupliObjectData*> DupliPair;
	
	typedef std::map<std::string, Object*> ObjectMap;
	typedef std::pair<std::string, Object*> ObjectPair;
	
public:
	AbcDupliCacheReader(const std::string &name, Group *group, DupliCache *dupcache,
	                    bool read_strands_motion, bool read_strands_children, bool read_sim_debug);
	~AbcDupliCacheReader();
	
	void init_abc(Abc::IObject object);
	
	PTCReadSampleResult read_sample_abc(chrono_t time);
	
protected:
	void read_dupligroup_object(Abc::IObject object, chrono_t time);
	void read_dupligroup_group(Abc::IObject abc_group, chrono_t time);
	
	DupliObjectData *find_dupli_data(Abc::ObjectReaderPtr ptr) const;
	void insert_dupli_data(Abc::ObjectReaderPtr ptr, DupliObjectData *data);
	
	void build_object_map(Main *bmain, Group *group);
	void build_object_map_add_group(Group *group);
	Object *find_object(const std::string &name) const;
	
private:
	DupliMap dupli_map;
	DupliCache *dupli_cache;
	
	ObjectMap object_map;
	bool m_read_strands_motion, m_read_strands_children;
	AbcSimDebugReader *m_simdebug_reader;
};


class AbcDupliObjectWriter : public ObjectWriter, public AbcWriter {
public:
	typedef std::map<std::string, AbcStrandsWriter *> StrandsWriters;
	typedef std::pair<std::string, AbcStrandsWriter *> StrandsWritersPair;
	
	AbcDupliObjectWriter(const std::string &name, DupliObjectData *dupdata, bool do_mesh, bool do_strands);
	~AbcDupliObjectWriter();
	
	void init_abc();
	
	void write_sample();
	
	AbcStrandsWriter *find_strands_writer(const std::string &name) const;
	AbcStrandsWriter *add_strands_writer(const std::string &name);
	
private:
	DupliObjectData *m_dupdata;
	bool m_do_strands;
	
	Abc::OObject m_abc_object;
	AbcDerivedMeshWriter *m_dm_writer;
	StrandsWriters m_strands_writers;
};

class AbcDupliObjectReader : public ObjectReader, public AbcReader {
public:
	typedef std::map<Abc::ObjectReaderPtr, DupliObjectData*> DupliMap;
	typedef std::pair<Abc::ObjectReaderPtr, DupliObjectData*> DupliPair;
	
public:
	AbcDupliObjectReader(const std::string &name, Object *ob, DupliObjectData *dupli_data,
	                     bool read_strands_motion, bool read_strands_children);
	~AbcDupliObjectReader();
	
	void init(ReaderArchive *archive);
	void init_abc(Abc::IObject object);
	
	PTCReadSampleResult read_sample_abc(chrono_t time);
	
protected:
	void read_dupligroup_object(Abc::IObject object, chrono_t time);
	
	DupliObjectData *find_dupli_data(Abc::ObjectReaderPtr ptr) const;
	void insert_dupli_data(Abc::ObjectReaderPtr ptr, DupliObjectData *data);
	
private:
	DupliMap dupli_map;
	DupliObjectData *dupli_data;
	bool m_read_strands_motion, m_read_strands_children;
	
	Abc::IObject m_abc_object;
};

} /* namespace PTC */

#endif  /* PTC_OBJECT_H */
