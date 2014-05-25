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

import bpy
from bpy.types import Operator
from bpy.props import FloatProperty

class ModHelper:

    def get_mod(self, context):
        for md in context.object.modifiers:
            if md.type == 'DYNAMIC_PAINT':
                return md
        bpy.ops.object.modifier_add(type='DYNAMIC_PAINT')
        for md in context.object.modifiers:
            if md.type == 'DYNAMIC_PAINT':
                return md
        return None # should not happen....

    def check_mod(self, context):
        for md in context.object.modifiers:
            if md.type == 'DYNAMIC_PAINT':
                return True
        return False

class FRACTURE_OT_toggle_ground(Operator, ModHelper):
    #set up dynamic paint here, brush
    """Mark or unmark this object as ground object"""
    bl_idname = "fracture.toggle_ground"
    bl_label = "Toggle Ground"
    bl_options = {'REGISTER', 'UNDO'}

    paint_distance = FloatProperty(
    name="Influence Range",
    description="Influence Range (=dynamic paint brush paint distance) of this Ground Object",
    min=0, max=10000.0,
    default=0.1,
    )

    def execute(self, context):
        if not self.check_mod(context):
            md = self.get_mod(context)
            md.ui_type = 'BRUSH'
            bpy.ops.dpaint.type_toggle(type='BRUSH')
            md.brush_settings.paint_source = 'VOLUME_DISTANCE'
            md.brush_settings.paint_distance = self.paint_distance
        else:
            bpy.ops.object.modifier_remove(modifier="Dynamic Paint")
        return {'FINISHED'}

class FRACTURE_OT_toggle_connect_ground(Operator, ModHelper):
    #set up dynamic paint here, canvas
    """Connect or disconnect this object from ground object"""
    bl_idname = "fracture.toggle_connect_ground"
    bl_label = "Toggle Ground Connection"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if not self.check_mod(context):
            md = self.get_mod(context)
            bpy.ops.dpaint.type_toggle(type='CANVAS')
            md.canvas_settings.canvas_surfaces["Surface"].surface_type = 'WEIGHT'
            bpy.ops.dpaint.output_toggle(output='A')

            for m in context.object.modifiers:
                if m.type == 'FRACTURE':
                    m.ground_vertex_group = md.canvas_settings.canvas_surfaces["Surface"].output_name_a
        else:
            for m in context.object.modifiers:
                if m.type == 'FRACTURE':
                    m.ground_vertex_group = ""
            bpy.ops.object.modifier_remove(modifier="Dynamic Paint")
        return {'FINISHED'}

