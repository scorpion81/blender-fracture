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
from bpy.types import Panel, Menu
from bpy.app.translations import pgettext_iface as iface_


class FLUID_MT_presets(Menu):
    bl_label = "Fluid Presets"
    preset_subdir = "fluid"
    preset_operator = "script.execute_preset"
    draw = Menu.draw_preset


class PhysicButtonsPanel():
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"

    @classmethod
    def poll(cls, context):
        ob = context.object
        rd = context.scene.render
        return (ob and ob.type == 'MESH') and (not rd.use_game_engine) and (context.fracture)


class PHYSICS_PT_fracture(PhysicButtonsPanel, Panel):
    bl_label = "Fracture"

    def draw(self, context):
        layout = self.layout

        md = context.fracture
        layout.prop(md, "frac_algorithm")
        layout.operator("object.fracture", icon="MOD_EXPLODE")

if __name__ == "__main__":  # only for live edit.
    bpy.utils.register_module(__name__)
