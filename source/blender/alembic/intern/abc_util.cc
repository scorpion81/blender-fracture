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

std::string get_id_name(Object *ob)
{
	if (!ob) {
		return "";
	}

	ID *id = reinterpret_cast<ID *>(ob);
	return get_id_name(id);
}

std::string get_id_name(ID *id)
{
	std::string name(id->name + 2);
	std::replace(name.begin(), name.end(), ' ', '_');
	std::replace(name.begin(), name.end(), '.', '_');
	std::replace(name.begin(), name.end(), ':', '_');

	return name;
}

std::string get_object_dag_path_name(Object *ob, Object *dupli_parent)
{
    std::string name = get_id_name(ob);

    Object *p = ob->parent;

    while (p) {
        name = get_id_name(p) + "/" + name;
        p = p->parent;
    }

	if (dupli_parent && (ob != dupli_parent)) {
		name = get_id_name(dupli_parent) + "/" + name;
	}

    return name;
}

bool object_selected(Object *ob)
{
	return ob->flag & SELECT;
}

bool parent_selected(Object *ob)
{
   if (object_selected(ob)) {
	   return true;
   }

   bool do_export = false;

   Object *parent = ob->parent;

   while (parent != NULL) {
	   if (object_selected(parent)) {
		   do_export = true;
		   break;
	   }

	   parent = parent->parent;
   }

   return do_export;
}

Imath::M44d convert_matrix(float mat[4][4])
{
	Imath::M44d m;

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			m[i][j] = mat[i][j];
		}
	}

	return m;
}

void split(const std::string &s, const char delim, std::vector<std::string> &tokens)
{
	tokens.clear();

	std::stringstream ss(s);
	std::string item;

	while (std::getline(ss, item, delim)) {
		if (!item.empty()) {
			tokens.push_back(item);
		}
	}
}
