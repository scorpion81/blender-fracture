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

#include "abc_exporter.h"

#include <cmath>

#include <Alembic/AbcCoreHDF5/All.h>
#include <Alembic/AbcCoreOgawa/All.h>

#include <boost/progress.hpp>

#include "abc_camera.h"
#include "abc_mesh.h"
#include "abc_nurbs.h"
#include "abc_hair.h"
#include "abc_util.h"

extern "C" {
#include "DNA_camera_types.h"
#include "DNA_curve_types.h"
#include "DNA_mesh_types.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"
#include "DNA_space_types.h"  /* for FILE_MAX */

#include "BLI_string.h"

#include "BKE_anim.h"
#include "BKE_global.h"
#include "BKE_idprop.h"
#include "BKE_main.h"
#include "BKE_modifier.h"
#include "BKE_particle.h"
#include "BKE_scene.h"
}

AbcExporter::AbcExporter(Scene *scene, const char *filename, AbcExportOptions &opts)
    : m_options(opts)
    , m_filename(filename)
    , m_scene(scene)
    , m_saved_frame(getCurrentFrame())
{}

AbcExporter::~AbcExporter()
{
	for (std::map<std::string, AbcTransformWriter*>::iterator it = m_xforms.begin(), e = m_xforms.end(); it != e; ++it)
		delete it->second;

	for (int i = 0, e = m_shapes.size(); i != e; ++i) {
		delete m_shapes[i];
	}

	if (getCurrentFrame() != m_saved_frame) {
		setCurrentFrame(m_saved_frame);
	}
}

void AbcExporter::getShutterSamples(double shutterOpen, double shutterClose,
                                        double step, bool timeRelative,
                                        std::vector<double> &samples)
{
	samples.clear();

	double timeFactor = timeRelative ? m_scene->r.frs_sec : 1.0;

	// sample all frame
	if (shutterOpen == 0.0 && shutterClose == 1.0) {
		for (double t = 0; t < 1.0; t += step) {
			samples.push_back(t / timeFactor);
		}
	}
	else {
		// sample between shutter open & close
		int nsamples = std::max((1.0 / step) - 1.0, 1.0);
		double timeInc = (shutterClose - shutterOpen) / nsamples;

		for (double t = shutterOpen; t <= shutterClose; t += timeInc) {
			samples.push_back(t / timeFactor);
		}
	}
}

Alembic::Abc::TimeSamplingPtr AbcExporter::createTimeSampling(int start, int end, double step,
                                                                  double shutterOpen, double shutterClose)
{
	Alembic::Abc::TimeSamplingPtr timeSampling;
	std::vector<double> samples;

	if (start == end) {
		timeSampling.reset(new Alembic::Abc::TimeSampling());
		return timeSampling;
	}

	getShutterSamples(shutterOpen, shutterClose, step, true, samples);
	Alembic::Abc::TimeSamplingType ts(static_cast<Alembic::Util::uint32_t>(samples.size()), 1.0 / m_scene->r.frs_sec);
	timeSampling.reset(new Alembic::Abc::TimeSampling(ts, samples));
	return timeSampling;
}

void AbcExporter::getFrameSet(int start, int end, double step, double shutterOpen, double shutterClose, std::set<double> &frames)
{
	frames.clear();

	std::vector<double> shutterSamples;
	getShutterSamples(shutterOpen, shutterClose, step, false, shutterSamples);

	for (int frame = start; frame <= end; ++frame) {
		for (int j = 0, e = shutterSamples.size(); j < e; ++j)
			frames.insert(frame + shutterSamples[j]);
	}
}

void AbcExporter::operator()()
{
	// Create archive here
	std::string sceneName;
	char buf[16];

	if (G.main->name[0] !=0) {
		char sceneFileName[FILE_MAX];
		BLI_strncpy(sceneFileName, G.main->name, FILE_MAX);
		sceneName = sceneFileName;
	}
	else {
		sceneName = "untitled";
	}

	Scene *scene = m_scene;
	int fps = FPS;
	snprintf(buf, 15, "%d", fps);
	const std::string str_fps = buf;
	Alembic::AbcCoreAbstract::MetaData md;
	md.set("FramesPerTimeUnit", str_fps);

	Alembic::Abc::Argument arg(md);

	if (!m_options.export_ogawa) {
		m_archive = Alembic::Abc::CreateArchiveWithInfo(Alembic::AbcCoreHDF5::WriteArchive(), m_filename, "Blender",
		                                               sceneName, Alembic::Abc::ErrorHandler::kThrowPolicy, arg);
	}
	else {
		m_archive = Alembic::Abc::CreateArchiveWithInfo(Alembic::AbcCoreOgawa::WriteArchive(), m_filename, "Blender",
		                                               sceneName, Alembic::Abc::ErrorHandler::kThrowPolicy, arg);
	}

	// Create time samplings for transforms and shapes
	Alembic::Abc::TimeSamplingPtr transTime = createTimeSampling(m_options.startframe, m_options.endframe,
	                                                             m_options.xform_frame_step, m_options.shutter_open,
	                                                             m_options.shutter_close);

	m_trans_sampling_index = m_archive.addTimeSampling(*transTime);

	Alembic::Abc::TimeSamplingPtr shapeTime;

	if ((m_options.shape_frame_step == m_options.xform_frame_step) || (m_options.startframe == m_options.endframe))
	{
		shapeTime = transTime;
		m_shape_sampling_index = m_trans_sampling_index;
	}
	else
	{
		shapeTime = createTimeSampling(m_options.startframe, m_options.endframe,
		                               m_options.shape_frame_step, m_options.shutter_open,
		                               m_options.shutter_close);

		m_shape_sampling_index = m_archive.addTimeSampling(*shapeTime);
	}

	Alembic::Abc::OBox3dProperty archiveBoxProp = Alembic::AbcGeom::CreateOArchiveBounds(m_archive, m_trans_sampling_index);

	if (m_options.flatten_hierarchy)
		createTransformWritersFlat();
	else
		createTransformWritersHierarchy();

	createShapeWriters();

	// make a list of frames to export
	std::set<double> xformFrames;
	getFrameSet(m_options.startframe, m_options.endframe, m_options.xform_frame_step, m_options.shutter_open, m_options.shutter_close, xformFrames);

	std::set<double> shapeFrames;
	getFrameSet(m_options.startframe, m_options.endframe, m_options.shape_frame_step, m_options.shutter_open, m_options.shutter_close, shapeFrames);

	// merge all frames needed
	std::set<double> allFrames(xformFrames);
	allFrames.insert(shapeFrames.begin(), shapeFrames.end());

	// export all frames

	// TODO: replace this with some kind of progress report
	std::cout << "Exporting Alembic archive: " << m_filename << std::endl;
	boost::progress_timer timer;
	boost::progress_display progress(allFrames.size());

	for (std::set<double>::const_iterator it(allFrames.begin()), e(allFrames.end()); it != e; ++it)
	{
		double f = *it;
		setCurrentFrame(f);

		if (shapeFrames.count(f) != 0)
		{
			for (int i = 0, e = m_shapes.size(); i != e; ++i)
				m_shapes[i]->write();
		}

		if (xformFrames.count(f) != 0)
		{
			for (std::map<std::string, AbcTransformWriter*>::iterator xit = m_xforms.begin(), xe = m_xforms.end(); xit != xe; ++xit)
				xit->second->write();

			// Save the archive's bounding box.
			Alembic::Abc::Box3d bounds;

			for (std::map<std::string, AbcTransformWriter*>::iterator xit = m_xforms.begin(), xe = m_xforms.end(); xit != xe; ++xit)
			{
				Alembic::Abc::Box3d box = xit->second->bounds();
				bounds.extendBy(box);
			}

			archiveBoxProp.set(bounds);
		}

		++progress;
	}
}

void AbcExporter::createTransformWritersHierarchy()
{
	Base *base = (Base *) m_scene->base.first;

	while (base) {
		Object *ob = base->object;

		if (m_options.exportObject(ob)) {
			switch(ob->type) {
				case OB_LAMP:
				case OB_LATTICE:
				case OB_MBALL:
				case OB_SPEAKER:
					// we do not export transforms for objects of these classes.
					break;

				default:
					exploreTransform(ob, ob->parent, NULL);
			}
		}

		base = base->next;
	}
}

void AbcExporter::createTransformWritersFlat()
{
	Base *base = (Base *) m_scene->base.first;

	while (base) {
		Object *ob = base->object;

		if (m_options.exportObject(ob) && objectIsShape(ob)) {
			std::string name = getObjectName(ob);
			m_xforms[name] = new AbcTransformWriter(ob, m_archive.getTop(), 0, m_trans_sampling_index, m_options);
		}

		base = base->next;
	}
}

void AbcExporter::exploreTransform(Object *ob, Object *parent, Object *dupliObParent)
{
	Object *dupliob = NULL;
	Object *dupliParent = NULL;
	
	struct DupliObject *link = NULL;
	struct ListBase *lb = NULL;

	createTransformWriter(ob, parent, dupliObParent);
	
	lb = object_duplilist(G.main->eval_ctx, m_scene, ob);

	if (lb) {
		link = (DupliObject*)lb->first;
		
		while (link) {
			dupliob = link->ob;

			if (dupliob->parent)
				dupliParent = dupliob->parent;
			else
				dupliParent = ob;

			if (link->type == OB_DUPLIGROUP)
				exploreTransform(dupliob, dupliParent, ob);

			link = link->next;
		}
	}

	free_object_duplilist(lb);

}

void AbcExporter::createTransformWriter(Object *ob, Object *parent, Object *dupliObParent)
{
	std::string name = getObjectDagPathName(ob, dupliObParent);
	AbcTransformWriter *xParent = NULL;
	std::string parentname = "";

	// check if we have already created a transform writer for this object
	if (m_xforms.find(name) == m_xforms.end()) {
		if (parent) {
			parentname = getObjectDagPathName(parent, dupliObParent);
			xParent = getXForm(parentname);

			if (!xParent) {
				if (parent->parent)
					createTransformWriter(parent, parent->parent, dupliObParent);
				else
					createTransformWriter(parent, dupliObParent, dupliObParent);

				xParent = getXForm(parentname);
			}
		}

		if (xParent) {
			m_xforms[name] = new AbcTransformWriter(ob, xParent->alembicXform(), xParent, m_trans_sampling_index, m_options);
			m_xforms[name]->setParent(parent);
		}
		else {
			m_xforms[name] = new AbcTransformWriter(ob, m_archive.getTop(), 0, m_trans_sampling_index, m_options);
		}
	}
	else {
		std::cerr << "xform " << name << " already exists\n";
	}
}

void AbcExporter::createShapeWriters()
{
	Base *base = (Base *) m_scene->base.first;

	while (base) {
		Object *ob = base->object;
		exploreObject(ob, NULL);

		base = base->next;
	}
}

void AbcExporter::exploreObject(Object *ob, Object *dupliObParent)
{
	ListBase *lb = object_duplilist(G.main->eval_ctx, m_scene, ob);
	
	createShapeWriter(ob, dupliObParent);
	
	if (lb) {
		DupliObject *link = static_cast<DupliObject *>(lb->first);
		Object *dupliob = NULL;
		// TODO(kevin): unused?
		//Object *dupliParent = NULL;

		while (link) {
			dupliob = link->ob;
			//dupliParent = (dupliob->parent) ? dupliob->parent : ob;

			if (link->type == OB_DUPLIGROUP) {
				exploreObject(dupliob, ob);
			}

			link = link->next;
		}
	}

	free_object_duplilist(lb);
}

void AbcExporter::createShapeWriter(Object *ob, Object *dupliObParent)
{
	if (!objectIsShape(ob))
		return;

	if (!m_options.exportObject(ob))
		return;

	std::string name = getObjectDagPathName(ob, dupliObParent);
	
	AbcTransformWriter *xform = getXForm(name);

	if (!xform) {
		std::cerr << "xform " << name << "is NULL" <<  std::endl;
		return;
	}

	ParticleSystem *psys = (ParticleSystem *)ob->particlesystem.first;

	int enable_hair 	  = true;
	int enable_hair_child = true;
	int enable_geo  	  = true;

	ID *id = reinterpret_cast<ID *>(ob);
	IDProperty *xport_props = IDP_GetProperties(id, 0);

	// Check for special export object flags
	if (xport_props) {
		IDProperty *enable_prop = IDP_GetPropertyFromGroup(xport_props, "abc_hair");
		if (enable_prop) {
			enable_hair = IDP_Int(enable_prop);
		}

		enable_prop = IDP_GetPropertyFromGroup(xport_props, "abc_geo");
		if (enable_prop) {
			if (IDP_Int(enable_prop) == 2) {
				enable_geo = false;
			}
			else {
				enable_geo = IDP_Int(enable_prop);
			}
		}

		enable_prop = IDP_GetPropertyFromGroup(xport_props, "abc_hair_child");
		if (enable_prop) {
			enable_hair_child = IDP_Int(enable_prop);
		}
	}

	for (; psys; psys = psys->next) {
		if (!psys_check_enabled(ob, psys))
			continue;

		if (enable_hair && psys->part && (psys->part->type == PART_HAIR)) {
			m_options.export_child_hairs = enable_hair_child;

			m_shapes.push_back(new AbcHairWriter(m_scene, ob, xform, m_shape_sampling_index, m_options, psys));
			m_shapes.back()->setRotateMatrix(true);
		}
	}

	switch(ob->type) {
		case OB_MESH:
		{
			if (enable_geo) {
				Mesh *me = static_cast<Mesh *>(ob->data);

				if (!me || me->totvert == 0) {
					return;
				}

				m_shapes.push_back(new AbcMeshWriter(m_scene, ob, xform, m_shape_sampling_index, m_options));
				m_shapes.back()->setRotateMatrix(true);
			}

			break;
		}
		case OB_SURF:
		{
			if (enable_geo) {
				Curve *cu = static_cast<Curve *>(ob->data);

				if (!cu) {
					return;
				}

				m_shapes.push_back(new AbcNurbsWriter(m_scene, ob, xform, m_shape_sampling_index, m_options));
				m_shapes.back()->setRotateMatrix(true);
			}

			break;
		}
		case OB_CAMERA:
		{
			Camera *cam = static_cast<Camera *>(ob->data);

			if (cam->type == CAM_PERSP) {
				m_shapes.push_back(new AbcCameraWriter(m_scene, ob, xform, m_shape_sampling_index, m_options));
			}

			break;
		}
	}
}

AbcTransformWriter *AbcExporter::getXForm(const std::string &name)
{
	std::map<std::string,AbcTransformWriter*>::iterator it = m_xforms.find(name);
	if (it == m_xforms.end())
		return NULL;
	return it->second;
}

double AbcExporter::getCurrentFrame() const
{
	return m_scene->r.cfra + m_scene->r.subframe;
}

void AbcExporter::setCurrentFrame(double t)
{
	m_scene->r.cfra = std::floor(t);
	m_scene->r.subframe = t - m_scene->r.cfra;
	BKE_scene_update_for_newframe(G.main->eval_ctx, G.main, m_scene, (1<<20)-1);
}

bool AbcExporter::objectIsShape(Object *ob)
{
	switch(ob->type) {
		case OB_MESH:
			if (objectIsSmokeSim(ob)) {
				return false;
			}

			return true;
			break;
		case OB_SURF:
		case OB_CAMERA:
			return true;
		default:
			return false;
	}
}

bool AbcExporter::objectIsSmokeSim(Object *ob)
{
	ModifierData *md = modifiers_findByType(ob, eModifierType_Smoke);

	if (md) {
		SmokeModifierData *smd = reinterpret_cast<SmokeModifierData *>(md);
		return (smd->type == MOD_SMOKE_TYPE_DOMAIN);
	}

	return false;
}
