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
from bpy.types import Panel, Menu, UIList
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

class FRACTURE_UL_fracture_levels(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        fl = item
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.prop(fl, "name", text="", emboss=False, icon_value=icon)
        elif self.layout_type in {'GRID'}:
            layout.alignment = 'CENTER'
            layout.label(text="", icon_value=icon)

class PHYSICS_PT_fracture(PhysicButtonsPanel, Panel):
    bl_label = "Fracture"

    def icon(self, bool):
        if bool:
            return 'TRIA_DOWN'
        else:
            return 'TRIA_RIGHT'

    def draw(self, context):
        layout = self.layout

        md = context.fracture
        col = layout.column()
        col.prop(md, "frac_algorithm")
        col.prop(md, "shard_count")
        col.label("Point Source:")
        col.prop(md, "point_source")
        col.prop(md, "extra_group")
        col.prop(md, "noise")
        col.prop(md, "percentage")
        col.prop(md, "point_seed")
        col.prop(md, "cluster_count")
        col.prop(md, "shards_to_islands")
        layout.operator("object.fracture_refresh", text="Refresh All Data")

class PHYSICS_PT_fracture_simulation(PhysicButtonsPanel, Panel):
    bl_label = "Fracture Simulation"

    def icon(self, bool):
        if bool:
            return 'TRIA_DOWN'
        else:
            return 'TRIA_RIGHT'

    def draw(self, context):
        layout = self.layout
        md = context.fracture

        layout.operator("object.rigidbody_constraints_refresh", text="Refresh Constraints Only")
        layout.label("Constraint Building Settings")
        layout.prop(md, "use_constraints")
        layout.prop(md, "contact_dist_meaning")
        if (md.contact_dist_meaning == 'CELLS') or (md.contact_dist_meaning == 'CELL_CENTROIDS'):
            layout.prop(md, "cell_size")
            layout.prop(md, "use_cellbased_sim", text="Use Compounds")
        layout.prop(md, "contact_dist")
        layout.label("Constraint Breaking Settings")
        col = layout.column(align=True)
        row = col.row(align=True)
        row.prop(md, "breaking_threshold", text="Threshold")
        row.prop(md, "breaking_percentage", text="Percentage")
        row = col.row(align=True)
        row.prop(md, "breaking_angle", text="Angle")
        row.prop(md, "breaking_distance", text="Distance")
        row = layout.row(align=True)
        row.operator("object.rigidbody_convert_to_objects", text = "Convert To Objects")

        #experimental stuff
        box = layout.box()
        box.prop(md, "use_experimental", text="Experimental, use with caution !", icon=self.icon(md.use_experimental), emboss = False)
        if md.use_experimental:
            box.prop(md, "use_both_directions")
            box.prop(md, "disable_self_collision")
            box.prop(md, "constraint_limit", text="Constraint limit, per MeshIsland")
            box.prop(md, "inner_constraint_type")
            box.prop(md, "use_proportional_limit")
            box.prop(md, "use_proportional_distance")
            box.prop(md, "mass_dependent_thresholds")
            box.prop(md, "dist_dependent_thresholds")
            box.prop(md, "solver_iterations_override")
            box.prop(md, "use_proportional_solver_iterations")
            box.prop(md, "cluster_breaking_threshold")
            if not(md.refresh):
                box.prop(md, "execute_threaded")

if __name__ == "__main__":  # only for live edit.
    bpy.utils.register_module(__name__)
