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

#include "PTC_api.h"

#include "ptc_types.h"

#include "abc_reader.h"
#include "abc_writer.h"
#include "abc_cloth.h"
#include "abc_group.h"
#include "abc_mesh.h"
#include "abc_object.h"
#include "abc_particles.h"

#include "alembic.h"

namespace PTC {

struct ArchiveSlicesFilter : public AbcArchiveFrameFilter {
	std::vector<Abc::chrono_t> start;
	std::vector<Abc::chrono_t> end;
	
	bool use_time(Abc::chrono_t time) const
	{
		assert(start.size() == end.size());
		for (size_t i = 0; i < start.size(); ++i) {
			if (time >= start[i] && time <= end[i])
				return true;
		}
		return false;
	}
};

class AbcFactory : public Factory {
	const std::string &get_default_extension()
	{
		static std::string ext = "abc";
		return ext;
	}
	
	WriterArchive *open_writer_archive(double fps, float start_frame, const std::string &name, PTCArchiveResolution resolutions,
	                                   const char *app_name, const char *description, const struct tm *time, struct IDProperty *metadata, ErrorHandler *error_handler)
	{
		return AbcWriterArchive::open(fps, start_frame, name, resolutions, app_name, description, time, metadata, error_handler);
	}
	
	ReaderArchive *open_reader_archive(double fps, float start_frame, const std::string &name, ErrorHandler *error_handler)
	{
		return AbcReaderArchive::open(fps, start_frame, name, error_handler);
	}
	
	void slice(ReaderArchive *in, WriterArchive *out, struct ListBase *slices)
	{
		BLI_assert(dynamic_cast<AbcReaderArchive*>(in));
		BLI_assert(dynamic_cast<AbcWriterArchive*>(out));
		AbcReaderArchive *abc_in = static_cast<AbcReaderArchive*>(in);
		AbcWriterArchive *abc_out = static_cast<AbcWriterArchive*>(out);
		
		ArchiveSlicesFilter abc_filter;
		for (CacheSlice *slice = (CacheSlice *)slices->first; slice; slice = slice->next) {
			abc_filter.start.push_back(abc_in->frame_to_time(slice->start));
			abc_filter.end.push_back(abc_in->frame_to_time(slice->end));
		}
		
		abc_archive_slice(abc_in->abc_archive(), abc_out->abc_archive(), abc_out->frame_sampling(), abc_filter);
	}
	
	Writer *create_writer_object(const std::string &name, Scene *scene, Object *ob)
	{
		return new AbcObjectWriter(name, scene, ob, true, true);
	}

	Reader *create_reader_object(const std::string &name, Object *ob)
	{
		return new AbcObjectReader(name, ob);
	}
	
	Writer *create_writer_group(const std::string &name, Group *group)
	{
		return new AbcGroupWriter(name, group);
	}
	
	Reader *create_reader_group(const std::string &name, Group *group)
	{
		return new AbcGroupReader(name, group);
	}
	
	
	/* Cloth */
	Writer *create_writer_cloth(const std::string &name, Object *ob, ClothModifierData *clmd)
	{
		return new AbcClothWriter(name, ob, clmd);
	}
	
	Reader *create_reader_cloth(const std::string &name, Object *ob, ClothModifierData *clmd)
	{
		return new AbcClothReader(name, ob, clmd);
	}
	
	/* Modifier Stack */
	Writer *create_writer_derived_mesh(const std::string &name, Object *ob, DerivedMesh **dm_ptr)
	{
		return new AbcDerivedMeshWriter(name, ob, dm_ptr);
	}
	
	Reader *create_reader_derived_mesh(const std::string &name, Object *ob)
	{
		return new AbcDerivedMeshReader(name, ob);
	}
	
	Writer *create_writer_derived_final_realtime(const std::string &name, Object *ob)
	{
		return new AbcDerivedFinalRealtimeWriter(name, ob);
	}
	
	Writer *create_writer_derived_final_render(const std::string &name, Scene *scene, Object *ob, DerivedMesh **render_dm_ptr)
	{
		return new AbcDerivedFinalRenderWriter(name, scene, ob, render_dm_ptr);
	}
	
	
	Writer *create_writer_dupligroup(const std::string &name, EvaluationContext *eval_ctx, Scene *scene, Group *group, CacheLibrary *cachelib)
	{
		return new AbcDupligroupWriter(name, eval_ctx, scene, group, cachelib);
	}
	
	Writer *create_writer_duplicache(const std::string &name, Group *group, DupliCache *dupcache, int datatypes, bool do_sim_debug)
	{
		return new AbcDupliCacheWriter(name, group, dupcache, datatypes, do_sim_debug);
	}
	
	Reader *create_reader_duplicache(const std::string &name, Group *group, DupliCache *dupcache,
	                                 bool read_strands_motion, bool read_strands_children, bool read_sim_debug)
	{
		return new AbcDupliCacheReader(name, group, dupcache, read_strands_motion, read_strands_children, read_sim_debug);
	}
	
	Reader *create_reader_duplicache_object(const std::string &name, Object *ob, DupliObjectData *data,
	                                        bool read_strands_motion, bool read_strands_children)
	{
		return new AbcDupliObjectReader(name, ob, data, read_strands_motion, read_strands_children);
	}
};

}

void PTC_alembic_init()
{
	static PTC::AbcFactory abc_factory;
	
	PTC::Factory::alembic = &abc_factory;
}
