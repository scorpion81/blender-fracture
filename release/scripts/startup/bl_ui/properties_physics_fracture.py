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


class FRACTURE_MT_presets(Menu):
    bl_label = "Fracture Presets"
    preset_subdir = "fracture"
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
        return (ob and (ob.type == 'MESH' or ob.type == 'CURVE' or ob.type == 'SURFACE' or ob.type == 'FONT')) and (not rd.use_game_engine) and (context.fracture)

class FRACTURE_UL_fracture_settings(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        fl = item
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.prop(fl, "name", text="", emboss=False, icon_value=icon)
        elif self.layout_type in {'GRID'}:
            layout.alignment = 'CENTER'
            layout.label(text="", icon_value=icon)

#class PHYSICS_PT_fracture_settings(PhysicButtonsPanel, Panel):
#    bl_label = "Fracture Settings"

#    def draw(self, context):
#       layout = self.layout
#       md = context.fracture

#       layout.template_list("FRACTURE_UL_fracture_settings", "", md, "fracture_settings", md, "active_setting", rows=3)

class PHYSICS_PT_fracture(PhysicButtonsPanel, Panel):
    bl_label = "Fracture Settings"

    def icon(self, bool):
        if bool:
            return 'TRIA_DOWN'
        else:
            return 'TRIA_RIGHT'

    def draw(self, context):
        layout = self.layout

        md = context.fracture
        ob = context.object

        if md.fracture_mode != 'EXTERNAL':
            layout.label(text="Presets:")
            sub = layout.row(align=True)
            sub.menu("FRACTURE_MT_presets", text=bpy.types.FRACTURE_MT_presets.bl_label)
            sub.operator("fracture.preset_add", text="", icon='ZOOMIN')
            sub.operator("fracture.preset_add", text="", icon='ZOOMOUT').remove_active = True
        else:
            layout.label(text="No UI controls here!")
            layout.label(text="Control happens via Python")

        row = layout.row()
        row.prop(md, "fracture_mode")

        if md.fracture_mode == 'EXTERNAL':
            layout.context_pointer_set("modifier", md)
            layout.operator("object.rigidbody_convert_to_objects", text = "Convert To Objects")
            return

        if md.fracture_mode == 'DYNAMIC':
            layout.prop(md, "dynamic_force")
            layout.prop(md, "limit_impact")

        layout.prop(md, "frac_algorithm")
        col = layout.column(align=True)
        col.prop(md, "shard_count")
        col.prop(md, "cluster_count")
        col.prop(md, "point_seed")
        layout.prop(md, "cluster_group")
        layout.prop(md, "cluster_constraint_type")
        if md.frac_algorithm in {'BOOLEAN', 'BISECT_FILL', 'BISECT_FAST_FILL', 'BOOLEAN_FRACTAL'}:
            col = layout.column()
            col.prop(md, "inner_material")
            col.prop_search(md, "uv_layer", ob.data, "uv_textures")
        if md.frac_algorithm == 'BOOLEAN_FRACTAL':
            col = layout.column(align=True)
            row = col.row(align=True)
            row.prop(md, "fractal_cuts")
            row.prop(md, "fractal_iterations")
            row = col.row(align=True)
            row.prop(md, "fractal_amount")
            row.prop(md, "physics_mesh_scale")
        row = layout.row()
        row.prop(md, "shards_to_islands")
        row.prop(md, "auto_execute")
        row.prop(md, "use_smooth")
        row = layout.row(align=True)
        row.prop(md, "splinter_axis")
        layout.prop(md, "splinter_length")

        box = layout.box()
        box.prop(md, "use_experimental", text="Advanced Fracture Settings", icon=self.icon(md.use_experimental), emboss = False)
        if md.use_experimental:
            box.label("Fracture Point Source:")
            col = box.column()
            col.prop(md, "point_source")
            if 'GREASE_PENCIL' in md.point_source:
                col.prop(md, "use_greasepencil_edges")
                col.prop(md, "grease_offset")
                col.prop(md, "grease_decimate")
                col.prop(md, "cutter_axis")
            col.prop(md, "extra_group")
            col.prop(md, "dm_group")
            if md.frac_algorithm == 'BOOLEAN':
                col.prop(md, "cutter_group")
            col.prop(md, "use_particle_birth_coordinates")

            box.prop(md, "percentage")
            box.label("Threshold Vertex Group:")
            box.prop_search(md, "thresh_vertex_group", ob, "vertex_groups", text = "")
            box.label("Passive Vertex Group:")
            box.prop_search(md, "ground_vertex_group", ob, "vertex_groups", text = "")
            box.label("Inner Vertex Group:")
            box.prop_search(md, "inner_vertex_group", ob, "vertex_groups", text = "")

        layout.context_pointer_set("modifier", md)
        layout.operator("object.fracture_refresh", text="Execute Fracture", icon='MOD_EXPLODE').reset = True

class PHYSICS_PT_fracture_simulation(PhysicButtonsPanel, Panel):
    bl_label = "Fracture Constraint Settings"

    @classmethod
    def poll(cls, context):
        md = context.fracture
        return PhysicButtonsPanel.poll(context) and md.fracture_mode != 'EXTERNAL'

    def draw(self, context):
        layout = self.layout
        md = context.fracture
        ob = context.object

        layout.label("Constraint Building Settings")
        row = layout.row()
        row.prop(md, "use_constraints")
        row.prop(md, "use_breaking")
        row.prop(md, "use_compounds")
        layout.prop(md, "constraint_target")
        col = layout.column(align=True)
        col.prop(md, "constraint_limit", text="Constraint limit, per MeshIsland")
        col.prop(md, "contact_dist")

        if md.use_compounds:
            layout.label("Compound Breaking Settings")
        else:
            layout.label("Constraint Breaking Settings")

        col = layout.column(align=True)
        col.prop(md, "breaking_threshold", text="Threshold")
        col.prop(md, "cluster_breaking_threshold")

        if md.use_compounds:
            #layout.label("Compound Damage Propagation Settings")
            col = layout.column(align=True)
            col.prop(md, "minimum_impulse")
            #col.prop(md, "impulse_dampening")
            #col.prop(md, "directional_factor")
            col.prop(md, "mass_threshold_factor")
        else:
            layout.label("Constraint Special Breaking Settings")
            col = layout.column(align=True)
            row = col.row(align=True)
            row.prop(md, "breaking_percentage", text="Percentage")
            row.prop(md, "cluster_breaking_percentage", text="Cluster Percentage")

            row = col.row(align=True)
            row.prop(md, "breaking_angle", text="Angle")
            row.prop(md, "cluster_breaking_angle", text="Cluster Angle")

            row = col.row(align=True)
            row.prop(md, "breaking_distance", text="Distance")
            row.prop(md, "cluster_breaking_distance", text="Cluster Distance")

            row = col.row(align=True)
            row.prop(md, "breaking_percentage_weighted")
            row.prop(md, "breaking_angle_weighted")
            row.prop(md, "breaking_distance_weighted")

            col = layout.column(align=True)
            col.prop(md, "solver_iterations_override")
            col.prop(md, "cluster_solver_iterations_override")
            layout.prop(md, "use_mass_dependent_thresholds")

class PHYSICS_PT_fracture_utilities(PhysicButtonsPanel, Panel):
    bl_label = "Fracture Utilities"

    @classmethod
    def poll(cls, context):
        md = context.fracture
        return PhysicButtonsPanel.poll(context) and md.fracture_mode != 'EXTERNAL'

    def draw(self, context):
        layout = self.layout
        md = context.fracture
        layout.prop(md, "autohide_filter_group", text = "Filter Group")
        col = layout.column(align=True)
        col.prop(md, "autohide_dist")
        col.prop(md, "automerge_dist")
        row = layout.row()
        row.prop(md, "fix_normals")
        row.prop(md, "nor_range")
        if not(md.refresh):
           layout.prop(md, "execute_threaded")

        layout.context_pointer_set("modifier", md)
        layout.operator("object.rigidbody_convert_to_objects", text = "Convert To Objects")
        layout.operator("object.rigidbody_convert_to_keyframes", text = "Convert To Keyframed Objects")

if __name__ == "__main__":  # only for live edit.
    bpy.utils.register_module(__name__)
