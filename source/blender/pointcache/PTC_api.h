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

#ifndef __PTC_API_H__
#define __PTC_API_H__

#include "util/util_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct tm;

struct Main;
struct Scene;
struct EvaluationContext;
struct ListBase;
struct PointerRNA;
struct ReportList;
struct CacheArchiveInfo;

struct DupliCache;
struct ClothModifierData;
struct DerivedMesh;
struct Group;
struct IDProperty;
struct ModifierData;
struct Object;
struct ParticleSystem;
struct SoftBody;

struct PTCWriterArchive;
struct PTCReaderArchive;
struct PTCWriter;
struct PTCReader;

void PTC_alembic_init(void);

/*** Error Handling ***/
void PTC_error_handler_std(void);
void PTC_error_handler_callback(PTCErrorCallback cb, void *userdata);
void PTC_error_handler_reports(struct ReportList *reports);
void PTC_error_handler_modifier(struct ModifierData *md);

/*** Archive ***/

const char *PTC_get_default_archive_extension(void);

struct PTCWriterArchive *PTC_open_writer_archive(double fps, float start_frame, const char *path, PTCArchiveResolution resolutions,
                                                 const char *app_name, const char *description, const struct tm *time, struct IDProperty *metadata);
void PTC_close_writer_archive(struct PTCWriterArchive *archive);
void PTC_writer_archive_use_render(struct PTCWriterArchive *archive, bool enable);

struct PTCReaderArchive *PTC_open_reader_archive(struct Scene *scene, const char *path);
struct PTCReaderArchive *PTC_open_reader_archive_ex(double fps, float start_frame, const char *path);
void PTC_close_reader_archive(struct PTCReaderArchive *archive);
PTCArchiveResolution PTC_reader_archive_get_resolutions(struct PTCReaderArchive *archive);
void PTC_reader_archive_use_render(struct PTCReaderArchive *archive, bool enable);
bool PTC_reader_archive_get_frame_range(struct PTCReaderArchive *_archive, int *start_frame, int *end_frame);

void PTC_writer_init(struct PTCWriter *writer, struct PTCWriterArchive *archive);
void PTC_writer_create_refs(struct PTCWriter *writer);
void PTC_reader_init(struct PTCReader *reader, struct PTCReaderArchive *archive);

/*** Reader/Writer Interface ***/

void PTC_writer_free(struct PTCWriter *writer);
void PTC_write_sample(struct PTCWriter *writer);

void PTC_reader_free(struct PTCReader *reader);
bool PTC_reader_get_frame_range(struct PTCReader *reader, int *start_frame, int *end_frame);
PTCReadSampleResult PTC_read_sample(struct PTCReader *reader, float frame);
PTCReadSampleResult PTC_test_sample(struct PTCReader *reader, float frame);

void PTC_get_archive_info_stream(struct PTCReaderArchive *archive, void (*stream)(void *, const char *), void *userdata);
void PTC_get_archive_info(struct PTCReaderArchive *_archive, struct CacheArchiveInfo *info, struct IDProperty *metadata);
void PTC_get_archive_info_nodes(struct PTCReaderArchive *_archive, struct CacheArchiveInfo *info, bool calc_bytes_size);

typedef struct CacheSlice {
	struct CacheSlice *next, *prev;
	int start, end;
} CacheSlice;

void PTC_archive_slice(struct PTCReaderArchive *in, struct PTCWriterArchive *out, struct ListBase *slices);

struct PTCWriter *PTC_writer_dupligroup(const char *name, struct EvaluationContext *eval_ctx, struct Scene *scene, struct Group *group, struct CacheLibrary *cachelib);
struct PTCWriter *PTC_writer_duplicache(const char *name, struct Group *group, struct DupliCache *dupcache, int datatypes, bool do_sim_debug);

struct PTCReader *PTC_reader_duplicache(const char *name, struct Group *group, struct DupliCache *dupcache,
                                        bool read_strands_motion, bool read_strands_children, bool read_sim_debug);
struct PTCReader *PTC_reader_duplicache_object(const char *name, struct Object *ob, struct DupliObjectData *data,
                                               bool read_strands_motion, bool read_strands_children);

/* get writer/reader from RNA type */
struct PTCWriter *PTC_writer_from_rna(struct Scene *scene, struct PointerRNA *ptr);
struct PTCReader *PTC_reader_from_rna(struct Scene *scene, struct PointerRNA *ptr);

/* Object */
struct PTCWriter *PTC_writer_object(const char *name, struct Scene *scene, struct Object *ob);
struct PTCReader *PTC_reader_object(const char *name, struct Object *ob);

/* Group */
struct PTCWriter *PTC_writer_group(const char *name, struct Group *group);
struct PTCReader *PTC_reader_group(const char *name, struct Group *group);

/* Cloth */
struct PTCWriter *PTC_writer_cloth(const char *name, struct Object *ob, struct ClothModifierData *clmd);
struct PTCReader *PTC_reader_cloth(const char *name, struct Object *ob, struct ClothModifierData *clmd);

struct PTCWriter *PTC_writer_derived_mesh(const char *name, struct Object *ob, struct DerivedMesh **dm_ptr);
struct PTCReader *PTC_reader_derived_mesh(const char *name, struct Object *ob);
struct DerivedMesh *PTC_reader_derived_mesh_acquire_result(struct PTCReader *reader);
void PTC_reader_derived_mesh_discard_result(struct PTCReader *reader);

struct PTCWriter *PTC_writer_derived_final_realtime(const char *name, struct Object *ob);
struct PTCWriter *PTC_writer_derived_final_render(const char *name, struct Scene *scene, struct Object *ob, struct DerivedMesh **render_dm_ptr);

#ifdef __cplusplus
} /* extern C */
#endif

#endif  /* __PTC_API_H__ */
