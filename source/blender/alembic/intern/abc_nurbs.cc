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

#include "abc_nurbs.h"

#include "abc_transform.h"
#include "abc_util.h"

extern "C" {
#include "DNA_curve_types.h"
#include "DNA_object_types.h"

#include "BLI_listbase.h"

#include "BKE_curve.h"
}

AbcNurbsWriter::AbcNurbsWriter(Scene *sce, Object *obj, AbcTransformWriter *parent,
										Alembic::Util::uint32_t timeSampling,
										AbcExportOptions& opts) : AbcShapeWriter(sce, obj, parent, timeSampling, opts)
{
	m_is_animated = isAnimated();

	// if the object is static, use the default static time sampling
	if (!m_is_animated)
		timeSampling = 0;

	Curve *curve = static_cast<Curve *>(m_object->data);
	size_t numNurbs = BLI_listbase_count(&curve->nurb);

	for (size_t i = 0; i < numNurbs; ++i) {
		std::stringstream str;
		str << getObjectName(m_object) << "Shape_" << i;

		while (parent->alembicXform().getChildHeader(str.str())) {
			str << "_";
		}

		Alembic::AbcGeom::ONuPatch nurbs(parent->alembicXform(), str.str().c_str(), m_time_sampling);
		m_nurbs_schema.push_back(nurbs.getSchema());
	}
}

AbcNurbsWriter::~AbcNurbsWriter()
{

}

bool AbcNurbsWriter::isAnimated() const
{
	// check if object has shape keys
	Curve *cu = static_cast<Curve *>(m_object->data);

	if (cu->key)
		return true;

	return false;
}

static
void recompute_pnts_cyclic(const BPoint *bps, const int num_u, const int num_v, const int add_u, const int add_v,
						   std::vector<Alembic::Abc::V3f>& pos, std::vector<float>& posWeight, bool rotate)
{
	const int new_u = num_u;// + add_u;
	const int new_v = num_v;// + add_v;
	const int new_size = new_u * new_v;

	pos.reserve(new_size);
	posWeight.reserve(new_size);

	std::vector< std::vector<Imath::Vec4<float> > > pnts;
	pnts.resize(new_u);
	for (int u = 0; u < new_u; ++u) {
		pnts[u].resize(new_v);
		for (int v = 0; v < new_v; ++v) {
			const BPoint& bp = bps[u + (v * new_u)];
			pnts[u][v] = Imath::Vec4<float>(bp.vec[0], bp.vec[1], bp.vec[2], bp.vec[3]);
		}
	}

	for (int u = 0; u < new_u; ++u) {
		for (int v = 0; v < new_v; ++v) {
			Imath::Vec4<float>& pnt = pnts[u][v];
			if (!rotate)
				pos.push_back(Alembic::Abc::V3f(pnt.x, pnt.y, pnt.z));
			else
				pos.push_back(Alembic::Abc::V3f(pnt.x, pnt.z, -pnt.y));
			posWeight.push_back(pnt.z);
		}
	}

}

void AbcNurbsWriter::do_write()
{
	// we have already stored a sample for this object.
	if (!m_first_frame && !m_is_animated)
		return;

	if (!ELEM(m_object->type, OB_SURF, OB_CURVE)) {
		return;
	}

	Curve *curve = static_cast<Curve *>(m_object->data);
	ListBase *nulb;

	if (m_object->curve_cache->deformed_nurbs.first != NULL) {
		nulb = &m_object->curve_cache->deformed_nurbs;
	}
	else {
		nulb = BKE_curve_nurbs_get(curve);
	}

	size_t count = 0;
	for (Nurb *nu = static_cast<Nurb *>(nulb->first); nu; nu = nu->next, count++) {
		const int numKnotsU = KNOTSU(nu);
		std::vector<float> knotsU;
		knotsU.reserve(numKnotsU);

		for (int i = 0; i < numKnotsU; ++i) {
			knotsU.push_back(nu->knotsu[i]);
		}

		const int numKnotsV = KNOTSV(nu);
		std::vector<float> knotsV;
		knotsV.reserve(numKnotsV);

		for (int i = 0; i < numKnotsV; ++i) {
			knotsV.push_back(nu->knotsv[i]);
		}

		Alembic::AbcGeom::ONuPatchSchema::Sample nuSamp;
		nuSamp.setUOrder(nu->orderu);
		nuSamp.setVOrder(nu->orderv);

		const int add_u = (nu->flagu & CU_NURB_CYCLIC) ? nu->orderu - 1 : 0;
		const int add_v = (nu->flagv & CU_NURB_CYCLIC) ? nu->orderv - 1 : 0;

		std::vector<Alembic::Abc::V3f> sampPos;
		std::vector<float> sampPosWeights;
		recompute_pnts_cyclic(nu->bp, nu->pntsu, nu->pntsv, add_u, add_v, sampPos, sampPosWeights, m_rotate_matrix);

		nuSamp.setPositions(sampPos);
		nuSamp.setPositionWeights(sampPosWeights);
		nuSamp.setUKnot(Alembic::Abc::FloatArraySample(knotsU));
		nuSamp.setVKnot(Alembic::Abc::FloatArraySample(knotsV));
		nuSamp.setNu(nu->pntsu);
		nuSamp.setNv(nu->pntsv);

		bool endu = nu->flagu & CU_NURB_ENDPOINT;
		bool endv = nu->flagv & CU_NURB_ENDPOINT;

		Alembic::AbcGeom::OCompoundProperty typeContainer = m_nurbs_schema[count].getUserProperties();
		Alembic::AbcGeom::OBoolProperty enduprop(typeContainer, "endU");
		enduprop.set(endu);
		Alembic::AbcGeom::OBoolProperty endvprop(typeContainer, "endV");
		endvprop.set(endv);

		m_nurbs_schema[count].set(nuSamp);
	}
}

void AbcNurbsWriter::writeNurbs()
{

}
