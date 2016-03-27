# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

# <pep8 compliant>

import bpy
from bpy.props import *
from bpy.types import Menu, Panel, UIList

from bpy_extras import object_utils


def find_object(obj_path, parent=None):
    if parent is None:
        parent = bpy.context.scene.objects
    else:
        parent = parent.children
    parts = obj_path.split('/')[1:]
    value = None
    for o in parent:
        if o.name == parts[0]:
            value = o

    if not value:
        return None

    if len(parts) == 1:
        return value

    return find_object("/" + "/".join(parts[1:]), value)


def _create_hierarchy(dag, filename=None, freeze=False, parent=None, namespace=None):
    parts = dag.split("/")
    parts.remove("")
    
    sub_object = ""
    for part in parts:
        sub_object += "/" + part
        
        if namespace is not None:
            part = "%s:%s" % (namespace, part)
        p = find_object("/" + part, parent)
        
        if p:
            parent = p
            continue

        if sub_object is "/":
            parent = None
            continue
        
        # bpy.ops.object.add(type='EMPTY')
        empty = bpy.data.objects.new('Empty', None)
        bpy.context.scene.objects.link(empty)
        empty.name = part

        empty.hide = True
        empty.parent = parent
        if filename is not None:
            empty.abc_file = filename
            empty.abc_subobject = sub_object
            empty.use_abc_xform = True
            empty.set_alembic_props()
        bpy.context.scene.update()
        if freeze:
            empty.lock_location[0] = True
            empty.lock_location[1] = True
            empty.lock_location[2] = True
            empty.lock_rotation[0] = True
            empty.lock_rotation[1] = True
            empty.lock_rotation[2] = True
            empty.lock_scale[0] = True
            empty.lock_scale[1] = True
            empty.lock_scale[2] = True
        else:
            empty.abc_file = ""
            empty.abc_subobject = ""
            empty.use_abc_xform = False
        parent = empty

    return parent


def _import_abc(filename, sub_object, object_name=None, object_type='mesh', freeze=False, parent=None, namespace=None):
    parts = sub_object.split("/")
    parts.remove("")
    if object_name is None:
        object_name = parts[-2]
    data_name = parts[-1]
    mesh = None
    camera = None

    if namespace is not None:
        object_name = "%s:%s" % (namespace, object_name)
        data_name = "%s:%s" % (namespace, data_name)

    if object_type == 'nurbs':
        nurbs = bpy.data.objects.new_nurbs_from_alembic(filename, sub_object, object_name, True)
        nurbs.data.name = data_name
        bpy.context.scene.objects.link(nurbs)
        obj = nurbs
    elif object_type == 'camera':
        cameradata = bpy.data.cameras.new(data_name)
        camera = bpy.data.objects.new(object_name, cameradata)
        bpy.context.scene.objects.link(camera)
        obj = camera
    elif object_type == 'mesh':
        mesh = bpy.data.objects.new_from_alembic(filename, sub_object, object_name, True)
        mesh.data.name = data_name
        bpy.context.scene.objects.link(mesh)
        obj = mesh

    obj.name = object_name
    # Set Alembic transforms

    obj.parent = parent
    obj.abc_subobject = "/" + "/".join(parts[:-1])
    obj.abc_file = filename
    obj.use_abc_xform = True
    obj.set_alembic_props()
    bpy.context.scene.update()

    if object_type in ('mesh', 'nurb'):
        # Set cache modifier
        mod = obj.modifiers.new(type="MESH_CACHE", name="Mesh Cache")
        mod.cache_format = 'ABC'
        mod.time_mode = 'TIME'
        mod.filepath = filename
        mod.sub_object = sub_object

    if freeze:
        obj.lock_location[0] = True
        obj.lock_location[1] = True
        obj.lock_location[2] = True
        obj.lock_rotation[0] = True
        obj.lock_rotation[1] = True
        obj.lock_rotation[2] = True
        obj.lock_scale[0] = True
        obj.lock_scale[1] = True
        obj.lock_scale[2] = True
    else:
        obj.abc_file = ""
        obj.abc_subobject = ""
        obj.use_abc_xform = False

    return obj


def get_dag(obj):
    value = ""
    if obj.parent:
        value = get_dag(obj.parent)
    return value + "|" + obj.name


def import_abc(filename, sub_object=None, freeze=True, parent=None, namespace=None):
    value = []
    old_cursor_location = tuple(bpy.context.scene.cursor_location)  # TODO HANDLE CURSOR

    bpy.context.scene.cursor_location = (0.0, 0.0, 0.0)
    objectsMesh = bpy.context.scene.alembic_get_objects(filename).split(";")
    objectsNurbs = bpy.context.scene.alembic_get_nurbs(filename).split(";")
    objectsCamera = bpy.context.scene.alembic_get_cameras(filename).split(";")

    # inverse list to get from top to bottom
    objectsMesh.reverse()
    objectsNurbs.reverse()
    objectsCamera.reverse()
    
    objectsMesh = filter(None, objectsMesh)
    objectsNurbs = filter(None, objectsNurbs)
    objectsCamera = filter(None, objectsCamera)

    
    for dag in objectsMesh:
        if sub_object is not None:
            if not dag.startswith(sub_object):
                continue
        
        parts = dag.split('/')
        parts.remove("")
        p = _create_hierarchy("/" + "/".join(parts[:-2]), filename=filename, freeze=freeze, parent=parent, namespace=namespace)
        e = _import_abc(filename, dag, object_type="mesh", freeze=freeze, parent=p, namespace=namespace)
        value.append(e)

    for dag in objectsNurbs:
        if sub_object is not None:
            if not dag.startswith(sub_object):
                continue
        parts = dag.split('/')
        parts.remove("")
        p = _create_hierarchy("/" + "/".join(parts[:-2]), filename=filename, freeze=freeze, parent=parent, namespace=namespace)
        value.append(_import_abc(filename, dag, object_type="nurbs", freeze=freeze, parent=p, namespace=namespace))

    for dag in objectsCamera:
        if sub_object is not None:
            if not dag.startswith(sub_object):
                continue
        parts = dag.split('/')
        parts.remove("")
        p = _create_hierarchy("/" + "/".join(parts[:-2]), filename=filename, freeze=freeze, parent=parent, namespace=namespace)
        value.append(_import_abc(filename, dag, object_type="camera", freeze=freeze, parent=p, namespace=namespace))

    bpy.context.scene.cursor_location = old_cursor_location
    return value
    
    
class OBJECT_OT_import_alembic(bpy.types.Operator):
    bl_idname = "alembic.import"
    bl_label = "Import Alembic file"
    filepath = StringProperty(subtype="FILE_PATH")
    filter_glob = StringProperty(default="*.abc")
    filename_ext = ".abc"
    
    def invoke(self, context, event):
        bpy.context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
        
    def execute(self, context):
        import_abc(self.filepath)
        return {'FINISHED'}
