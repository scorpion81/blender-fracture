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

#include "abc_util.h"

#include <algorithm>

extern "C" {
#include "DNA_object_types.h"
}

std::string getObjectName(Object *ob)
{
	if (!ob) {
		return "";
	}

	ID *id = reinterpret_cast<ID *>(ob);

	std::string name(id->name + 2);
	std::replace(name.begin(), name.end(), ' ', '_');
	std::replace(name.begin(), name.end(), '.', '_');
	std::replace(name.begin(), name.end(), ':', '_');

	return name;
}

std::string getObjectDagPathName(Object *ob, Object *dupliParent)
{
    std::string name = getObjectName(ob);

    Object *p = ob->parent;

    while (p) {
        name = getObjectName(p) + "/" + name;
        p = p->parent;
    }

    if (dupliParent && (ob != dupliParent))
        name = getObjectName(dupliParent) + "/" + name;

    return name;
}

bool objectIsSelected(Object *ob)
{
	return ob->flag & SELECT;
}

Alembic::Abc::M44d convertMatrix(float mat[4][4])
{
	Alembic::Abc::M44d m;

	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			m[i][j] = mat[i][j];

	return m;
}
