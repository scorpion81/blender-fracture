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

#ifndef PTC_TYPES_H
#define PTC_TYPES_H

#include "reader.h"
#include "writer.h"

extern "C" {
#include "DNA_group_types.h"
#include "DNA_modifier_types.h"
#include "DNA_object_types.h"
#include "DNA_object_force.h"
#include "DNA_particle_types.h"
}

struct CacheLibrary;
struct IDProperty;

namespace PTC {

class ClothWriter {
public:
	ClothWriter(Object *ob, ClothModifierData *clmd, const std::string &name) :
	    m_ob(ob),
	    m_clmd(clmd),
	    m_name(name)
	{}
	
	~ClothWriter()
	{}
	
protected:
	Object *m_ob;
	ClothModifierData *m_clmd;
	std::string m_name;
};

class ClothReader {
public:
	ClothReader(Object *ob, ClothModifierData *clmd, const std::string &name) :
	    m_ob(ob),
	    m_clmd(clmd),
	    m_name(name)
	{}
	
	~ClothReader()
	{}
	
protected:
	Object *m_ob;
	ClothModifierData *m_clmd;
	std::string m_name;
};


class DerivedMeshWriter {
public:
	/** \note Targeted DerivedMesh at \a dm_ptr must be available only on \fn write_sample calls */
	DerivedMeshWriter(Object *ob, DerivedMesh **dm_ptr, const std::string &name) :
	    m_ob(ob),
	    m_dm_ptr(dm_ptr),
	    m_name(name)
	{}
	
	~DerivedMeshWriter()
	{}
	
protected:
	Object *m_ob;
	DerivedMesh **m_dm_ptr;
	std::string m_name;
};

class DerivedMeshReader {
public:
	DerivedMeshReader(Object *ob, const std::string &name) :
	    m_ob(ob),
	    m_result(0),
	    m_name(name)
	{}
	
	~DerivedMeshReader()
	{
		discard_result();
	}
	
	DerivedMesh *acquire_result();
	void discard_result();
	
protected:
	Object *m_ob;
	DerivedMesh *m_result;
	std::string m_name;
};

class GroupWriter {
public:
	GroupWriter(Group *group, const std::string &name) :
	    m_group(group),
	    m_name(name)
	{}
	
protected:
	Group *m_group;
	std::string m_name;
};

class GroupReader {
public:
	GroupReader(Group *group, const std::string &name) :
	    m_group(group),
	    m_name(name)
	{}
	
protected:
	Group *m_group;
	std::string m_name;
};

class ObjectWriter {
public:
	ObjectWriter(Object *ob, const std::string &name) :
	    m_ob(ob),
	    m_name(name)
	{}
	
protected:
	Object *m_ob;
	std::string m_name;
};

class ObjectReader {
public:
	ObjectReader(Object *ob, const std::string &name) :
	    m_ob(ob),
	    m_name(name)
	{}
	
protected:
	Object *m_ob;
	std::string m_name;
};

class ParticlesWriter {
public:
	ParticlesWriter(Object *ob, ParticleSystem *psys, const std::string &name) :
	    m_ob(ob),
	    m_psys(psys),
	    m_name(name)
	{}
	
	~ParticlesWriter()
	{}
	
protected:
	Object *m_ob;
	ParticleSystem *m_psys;
	std::string m_name;
};

class ParticlesReader {
public:
	ParticlesReader(Object *ob, ParticleSystem *psys, const std::string &name) :
	    m_ob(ob),
	    m_psys(psys),
	    m_name(name),
	    m_totpoint(0)
	{}
	
	~ParticlesReader()
	{}
	
	int totpoint() const { return m_totpoint; }
	
protected:
	Object *m_ob;
	ParticleSystem *m_psys;
	std::string m_name;
	
	int m_totpoint;
};

struct Factory {
	virtual const std::string &get_default_extension() = 0;
	virtual WriterArchive *open_writer_archive(double fps, float start_frame, const std::string &name, PTCArchiveResolution resolutions,
	                                           const char *app_name, const char *description, const struct tm *time, struct IDProperty *metadata, ErrorHandler *error_handler) = 0;
	virtual ReaderArchive *open_reader_archive(double fps, float start_frame, const std::string &name, ErrorHandler *error_handler) = 0;
	
	virtual void slice(ReaderArchive *in, WriterArchive *out, struct ListBase *slices) = 0;
	
	virtual Writer *create_writer_object(const std::string &name, Scene *scene, Object *ob) = 0;
	virtual Reader *create_reader_object(const std::string &name, Object *ob) = 0;
	
	virtual Writer *create_writer_group(const std::string &name, Group *group) = 0;
	virtual Reader *create_reader_group(const std::string &name, Group *group) = 0;
	
	/* Cloth */
	virtual Writer *create_writer_cloth(const std::string &name, Object *ob, ClothModifierData *clmd) = 0;
	virtual Reader *create_reader_cloth(const std::string &name, Object *ob, ClothModifierData *clmd) = 0;
	
	/* Modifier Stack */
	virtual Writer *create_writer_derived_mesh(const std::string &name, Object *ob, DerivedMesh **dm_ptr) = 0;
	virtual Reader *create_reader_derived_mesh(const std::string &name, Object *ob) = 0;
	
	virtual Writer *create_writer_derived_final_realtime(const std::string &name, Object *ob) = 0;
	virtual Writer *create_writer_derived_final_render(const std::string &name, Scene *scene, Object *ob, DerivedMesh **render_dm_ptr) = 0;
	
	virtual Writer *create_writer_dupligroup(const std::string &name, EvaluationContext *eval_ctx, Scene *scene, Group *group, CacheLibrary *cachelib) = 0;
	virtual Writer *create_writer_duplicache(const std::string &name, Group *group, DupliCache *dupcache, int datatypes, bool do_sim_debug) = 0;
	virtual Reader *create_reader_duplicache(const std::string &name, Group *group, DupliCache *dupcache,
	                                         bool read_strands_motion, bool read_strands_children, bool read_sim_debug) = 0;
	virtual Reader *create_reader_duplicache_object(const std::string &name, Object *ob, DupliObjectData *data,
	                                                bool read_strands_motion, bool read_strands_children) = 0;
	
	static Factory *alembic;
};

} /* namespace PTC */

#endif  /* PTC_EXPORT_H */
