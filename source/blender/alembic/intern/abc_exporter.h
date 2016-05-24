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
 * Contributor(s): Esteban Tovagliari, Cedric Paille, Kevin Dietrich
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#ifndef __ABC_EXPORTER_H__
#define __ABC_EXPORTER_H__

#include <map>
#include <vector>

#include "abc_object.h"
#include "abc_transform.h"

class AbcExporter {
	ExportSettings &m_settings;

	const char *m_filename;

    Alembic::Abc::OArchive m_archive;
    unsigned int m_trans_sampling_index, m_shape_sampling_index;

	Scene *m_scene;
	double m_saved_frame;

	std::map<std::string, AbcTransformWriter *> m_xforms;
	std::vector<AbcObjectWriter *> m_shapes;

public:
	AbcExporter(Scene *scene, const char *filename, ExportSettings &settings);
	~AbcExporter();

	void operator()();

protected:
	double getCurrentFrame() const;
	void setCurrentFrame(double t);

private:
	void getShutterSamples(double shutterOpen, double shutterClose,
							double step, bool timeRelative,
							std::vector<double> &samples);

	Alembic::Abc::TimeSamplingPtr createTimeSampling(int start, int end, double step,
													double shutterOpen, double shutterClose);

	void getFrameSet(int start, int end, double step, double shutterOpen, double shutterClose, std::set<double> &frames);

	void createTransformWritersHierarchy();
	void createTransformWritersFlat();
    void createTransformWriter(Object *ob,  Object *parent, Object *dupliObParent);
    void exploreTransform(Object *ob, Object *parent, Object *dupliObParent = NULL);
    void exploreObject(Object *ob, Object *dupliObParent);
	void createShapeWriters();
    void createShapeWriter(Object *ob, Object *dupliObParent);

	AbcTransformWriter *getXForm(const std::string &name);

	bool objectIsShape(Object *ob);
	bool objectIsSmokeSim(Object *ob);
};

#endif  /* __ABC_EXPORTER_H__ */
