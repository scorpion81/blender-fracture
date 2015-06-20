/*
 * ***** BEGIN GPL LICENSE BLOCK *****
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
 *
 * Copyright 2013, Blender Foundation.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "MEM_guardedalloc.h"

#include "PTC_api.h"

#include "util/util_error_handler.h"

#include "reader.h"
#include "writer.h"

#include "ptc_types.h"

extern "C" {
#include "BLI_listbase.h"
#include "BLI_math.h"

#include "DNA_listBase.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"

#include "BKE_DerivedMesh.h"
#include "BKE_modifier.h"
#include "BKE_report.h"
#include "BKE_scene.h"

#include "RNA_access.h"
}

using namespace PTC;

class StubFactory : public Factory {
	const std::string &get_default_extension() { static std::string ext = ""; return ext; }
	WriterArchive *open_writer_archive(double /*fps*/, float /*start_frame*/, const std::string &/*name*/, PTCArchiveResolution /*resolutions*/,
	                                   const char */*app_name*/, const char */*description*/, const struct tm */*time*/, struct IDProperty */*metadata*/, ErrorHandler */*error_handler*/) { return NULL; }
	ReaderArchive *open_reader_archive(double /*fps*/, float /*start_frame*/, const std::string &/*name*/, ErrorHandler * /*error_handler*/) { return NULL; }
	void slice(ReaderArchive * /*in*/, WriterArchive * /*out*/, float /*start_frame*/, float /*end_frame*/) {}
	Writer *create_writer_object(const std::string &/*name*/, Scene */*scene*/, Object */*ob*/) { return NULL; }
	Reader *create_reader_object(const std::string &/*name*/, Object */*ob*/) { return NULL; }
	Writer *create_writer_group(const std::string &/*name*/, Group */*group*/) { return NULL; }
	Reader *create_reader_group(const std::string &/*name*/, Group */*group*/) { return NULL; }
	Writer *create_writer_cloth(const std::string &/*name*/, Object */*ob*/, ClothModifierData */*clmd*/) { return NULL; }
	Reader *create_reader_cloth(const std::string &/*name*/, Object */*ob*/, ClothModifierData */*clmd*/) { return NULL; }
	Writer *create_writer_derived_mesh(const std::string &/*name*/, Object */*ob*/, DerivedMesh **/*dm_ptr*/) { return NULL; }
	Reader *create_reader_derived_mesh(const std::string &/*name*/, Object */*ob*/) { return NULL; }
	Writer *create_writer_derived_final_realtime(const std::string &/*name*/, Object */*ob*/) { return NULL; }
	Writer *create_writer_derived_final_render(const std::string &/*name*/, Scene */*scene*/, Object */*ob*/, DerivedMesh **/*render_dm_ptr*/) { return NULL; }
	Writer *create_writer_dupligroup(const std::string &/*name*/, EvaluationContext */*eval_ctx*/, Scene */*scene*/, Group */*group*/, CacheLibrary */*cachelib*/) { return NULL; }
	Writer *create_writer_duplicache(const std::string &/*name*/, Group */*group*/, DupliCache */*dupcache*/, int /*datatypes*/, bool /*do_sim_debug*/) { return NULL; }
	Reader *create_reader_duplicache(const std::string &/*name*/, Group */*group*/, DupliCache */*dupcache*/, bool /*read_strands_motion*/, bool /*read_strands_children*/, bool /*do_sim_debug*/) { return NULL; }
	Reader *create_reader_duplicache_object(const std::string &/*name*/, Object */*ob*/, DupliObjectData */*data*/, bool /*read_strands_motion*/, bool /*read_strands_children*/) { return NULL; }
};

#ifndef WITH_PTC_ALEMBIC
void PTC_alembic_init()
{
	static StubFactory stub_factory;
	PTC::Factory::alembic = &stub_factory;
}
#endif

void PTC_error_handler_std(void)
{
	ErrorHandler::clear_default_handler();
}

void PTC_error_handler_callback(PTCErrorCallback cb, void *userdata)
{
	ErrorHandler::set_default_handler(new CallbackErrorHandler(cb, userdata));
}

static ReportType report_type_from_error_level(PTCErrorLevel level)
{
	switch (level) {
		case PTC_ERROR_NONE:        return RPT_DEBUG;
		case PTC_ERROR_INFO:        return RPT_INFO;
		case PTC_ERROR_WARNING:     return RPT_WARNING;
		case PTC_ERROR_CRITICAL:    return RPT_ERROR;
	}
	return RPT_ERROR;
}

static void error_handler_reports_cb(void *vreports, PTCErrorLevel level, const char *message)
{
	ReportList *reports = (ReportList *)vreports;
	
	BKE_report(reports, report_type_from_error_level(level), message);
}

void PTC_error_handler_reports(struct ReportList *reports)
{
	ErrorHandler::set_default_handler(new CallbackErrorHandler(error_handler_reports_cb, reports));
}

static void error_handler_modifier_cb(void *vmd, PTCErrorLevel UNUSED(level), const char *message)
{
	ModifierData *md = (ModifierData *)vmd;
	
	modifier_setError(md, "%s", message);
}

void PTC_error_handler_modifier(struct ModifierData *md)
{
	ErrorHandler::set_default_handler(new CallbackErrorHandler(error_handler_modifier_cb, md));
}


const char *PTC_get_default_archive_extension(void)
{
	return PTC::Factory::alembic->get_default_extension().c_str();
}

PTCWriterArchive *PTC_open_writer_archive(double fps, float start_frame, const char *path, PTCArchiveResolution resolutions,
                                          const char *app_name, const char *description, const struct tm *time, struct IDProperty *metadata)
{
	return (PTCWriterArchive *)PTC::Factory::alembic->open_writer_archive(fps, start_frame, path, resolutions, app_name, description, time, metadata, NULL);
}

void PTC_close_writer_archive(PTCWriterArchive *_archive)
{
	PTC::WriterArchive *archive = (PTC::WriterArchive *)_archive;
	delete archive;
}

void PTC_writer_archive_use_render(PTCWriterArchive *_archive, bool enable)
{
	PTC::WriterArchive *archive = (PTC::WriterArchive *)_archive;
	archive->use_render(enable);
}

PTCReaderArchive *PTC_open_reader_archive(Scene *scene, const char *path)
{
	double fps = FPS;
	float start_frame = scene->r.sfra;
	return PTC_open_reader_archive_ex(fps, start_frame, path);
}

PTCReaderArchive *PTC_open_reader_archive_ex(double fps, float start_frame, const char *path)
{
	return (PTCReaderArchive *)PTC::Factory::alembic->open_reader_archive(fps, start_frame, path, NULL);
}

void PTC_close_reader_archive(PTCReaderArchive *_archive)
{
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	delete archive;
}

PTCArchiveResolution PTC_reader_archive_get_resolutions(PTCReaderArchive *_archive)
{
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	return archive->get_resolutions();
}

void PTC_reader_archive_use_render(PTCReaderArchive *_archive, bool enable)
{
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	archive->use_render(enable);
}

bool PTC_reader_archive_get_frame_range(PTCReaderArchive *_archive, int *start_frame, int *end_frame)
{
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	return archive->get_frame_range(*start_frame, *end_frame);
}

void PTC_writer_init(PTCWriter *_writer, PTCWriterArchive *_archive)
{
	PTC::Writer *writer = (PTC::Writer *)_writer;
	PTC::WriterArchive *archive = (PTC::WriterArchive *)_archive;
	writer->init(archive);
}

void PTC_writer_create_refs(PTCWriter *_writer)
{
	PTC::Writer *writer = (PTC::Writer *)_writer;
	writer->create_refs();
}

void PTC_reader_init(PTCReader *_reader, PTCReaderArchive *_archive)
{
	PTC::Reader *reader = (PTC::Reader *)_reader;
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	reader->init(archive);
}

/* ========================================================================= */

void PTC_writer_free(PTCWriter *_writer)
{
	PTC::Writer *writer = (PTC::Writer *)_writer;
	delete writer;
}

void PTC_write_sample(struct PTCWriter *_writer)
{
	PTC::Writer *writer = (PTC::Writer *)_writer;
	writer->write_sample();
}


void PTC_reader_free(PTCReader *_reader)
{
	PTC::Reader *reader = (PTC::Reader *)_reader;
	delete reader;
}

bool PTC_reader_get_frame_range(PTCReader *_reader, int *start_frame, int *end_frame)
{
	PTC::Reader *reader = (PTC::Reader *)_reader;
	int sfra, efra;
	if (reader->get_frame_range(sfra, efra)) {
		if (start_frame) *start_frame = sfra;
		if (end_frame) *end_frame = efra;
		return true;
	}
	else {
		return false;
	}
}

PTCReadSampleResult PTC_read_sample(PTCReader *_reader, float frame)
{
	PTC::Reader *reader = (PTC::Reader *)_reader;
	return reader->read_sample(frame);
}

PTCReadSampleResult PTC_test_sample(PTCReader *_reader, float frame)
{
	PTC::Reader *reader = (PTC::Reader *)_reader;
	return reader->test_sample(frame);
}

void PTC_get_archive_info_stream(PTCReaderArchive *_archive, void (*stream)(void *, const char *), void *userdata)
{
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	archive->get_info_stream(stream, userdata);
}

void PTC_get_archive_info(PTCReaderArchive *_archive, struct CacheArchiveInfo *info, IDProperty *metadata)
{
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	archive->get_info(info, metadata);
}

void PTC_get_archive_info_nodes(PTCReaderArchive *_archive, struct CacheArchiveInfo *info, bool calc_bytes_size)
{
	PTC::ReaderArchive *archive = (PTC::ReaderArchive *)_archive;
	archive->get_info_nodes(info, calc_bytes_size);
}

void PTC_archive_slice(PTCReaderArchive *_in, PTCWriterArchive *_out, struct ListBase *slices)
{
	PTC::ReaderArchive *in = (PTC::ReaderArchive *)_in;
	PTC::WriterArchive *out = (PTC::WriterArchive *)_out;
	
	PTC::Factory::alembic->slice(in, out, slices);
}


PTCWriter *PTC_writer_dupligroup(const char *name, struct EvaluationContext *eval_ctx, struct Scene *scene, struct Group *group, struct CacheLibrary *cachelib)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_dupligroup(name, eval_ctx, scene, group, cachelib);
}

PTCWriter *PTC_writer_duplicache(const char *name, struct Group *group, struct DupliCache *dupcache, int datatypes, bool do_sim_debug)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_duplicache(name, group, dupcache, datatypes, do_sim_debug);
}

PTCReader *PTC_reader_duplicache(const char *name, struct Group *group, struct DupliCache *dupcache,
                                 bool read_strands_motion, bool read_strands_children, bool read_sim_debug)
{
	return (PTCReader *)PTC::Factory::alembic->create_reader_duplicache(name, group, dupcache,
	                                                                    read_strands_motion, read_strands_children, read_sim_debug);
}

PTCReader *PTC_reader_duplicache_object(const char *name, struct Object *ob, struct DupliObjectData *data,
                                        bool read_strands_motion, bool read_strands_children)
{
	return (PTCReader *)PTC::Factory::alembic->create_reader_duplicache_object(name, ob, data, read_strands_motion, read_strands_children);
}


/* get writer/reader from RNA type */
PTCWriter *PTC_writer_from_rna(Scene */*scene*/, PointerRNA */*ptr*/)
{
#if 0
#if 0
	if (RNA_struct_is_a(ptr->type, &RNA_ParticleSystem)) {
		Object *ob = (Object *)ptr->id.data;
		ParticleSystem *psys = (ParticleSystem *)ptr->data;
		return PTC_writer_particles_combined(scene, ob, psys);
	}
#endif
	if (RNA_struct_is_a(ptr->type, &RNA_ClothModifier)) {
		Object *ob = (Object *)ptr->id.data;
		ClothModifierData *clmd = (ClothModifierData *)ptr->data;
		return PTC_writer_cloth(scene, ob, clmd);
	}
#endif
	return NULL;
}

PTCReader *PTC_reader_from_rna(Scene */*scene*/, PointerRNA */*ptr*/)
{
#if 0
	if (RNA_struct_is_a(ptr->type, &RNA_ParticleSystem)) {
		Object *ob = (Object *)ptr->id.data;
		ParticleSystem *psys = (ParticleSystem *)ptr->data;
		/* XXX particles are bad ...
		 * this can be either the actual particle cache or the hair dynamics cache,
		 * which is actually the cache of the internal cloth modifier
		 */
		bool use_cloth_cache = psys->part->type == PART_HAIR && (psys->flag & PSYS_HAIR_DYNAMICS);
		if (use_cloth_cache && psys->clmd)
			return PTC_reader_cloth(scene, ob, psys->clmd);
		else
			return PTC_reader_particles(scene, ob, psys);
	}
	if (RNA_struct_is_a(ptr->type, &RNA_ClothModifier)) {
		Object *ob = (Object *)ptr->id.data;
		ClothModifierData *clmd = (ClothModifierData *)ptr->data;
		return PTC_reader_cloth(scene, ob, clmd);
	}
#endif
	return NULL;
}


/* ==== CLOTH ==== */

PTCWriter *PTC_writer_cloth(const char *name, Object *ob, ClothModifierData *clmd)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_cloth(name, ob, clmd);
}

PTCReader *PTC_reader_cloth(const char *name, Object *ob, ClothModifierData *clmd)
{
	return (PTCReader *)PTC::Factory::alembic->create_reader_cloth(name, ob, clmd);
}


/* ==== MESH ==== */

PTCWriter *PTC_writer_derived_mesh(const char *name, Object *ob, DerivedMesh **dm_ptr)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_derived_mesh(name, ob, dm_ptr);
}

PTCReader *PTC_reader_derived_mesh(const char *name, Object *ob)
{
	return (PTCReader *)PTC::Factory::alembic->create_reader_derived_mesh(name, ob);
}

struct DerivedMesh *PTC_reader_derived_mesh_acquire_result(PTCReader *_reader)
{
	DerivedMeshReader *reader = (DerivedMeshReader *)_reader;
	return reader->acquire_result();
}

void PTC_reader_derived_mesh_discard_result(PTCReader *_reader)
{
	DerivedMeshReader *reader = (DerivedMeshReader *)_reader;
	reader->discard_result();
}


PTCWriter *PTC_writer_derived_final_realtime(const char *name, Object *ob)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_derived_final_realtime(name, ob);
}

PTCWriter *PTC_writer_derived_final_render(const char *name, Scene *scene, Object *ob, DerivedMesh **render_dm_ptr)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_derived_final_render(name, scene, ob, render_dm_ptr);
}


/* ==== OBJECT ==== */

PTCWriter *PTC_writer_object(const char *name, Scene *scene, Object *ob)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_object(name, scene, ob);
}

PTCReader *PTC_reader_object(const char *name, Object *ob)
{
	return (PTCReader *)PTC::Factory::alembic->create_reader_object(name, ob);
}


/* ==== GROUP ==== */

PTCWriter *PTC_writer_group(const char *name, Group *group)
{
	return (PTCWriter *)PTC::Factory::alembic->create_writer_group(name, group);
}

PTCReader *PTC_reader_group(const char *name, Group *group)
{
	return (PTCReader *)PTC::Factory::alembic->create_writer_group(name, group);
}
