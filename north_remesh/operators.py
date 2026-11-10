# SPDX-License-Identifier: GPL-3.0-or-later
"""North Remesh - Blender operator (modal with live progress, Esc to cancel)."""

import math
import time
import traceback

import bmesh
import bpy
from bpy.props import BoolProperty, EnumProperty, FloatProperty, IntProperty

from .engine import RemeshError
from .region import Options, RegionJob

_NAV_EVENTS = {
    'MIDDLEMOUSE', 'WHEELUPMOUSE', 'WHEELDOWNMOUSE', 'WHEELINMOUSE', 'WHEELOUTMOUSE',
    'TRACKPADPAN', 'TRACKPADZOOM', 'MOUSEMOVE', 'INBETWEEN_MOUSEMOVE',
    'NDOF_MOTION', 'NDOF_BUTTON_FIT',
}
_SLICE_SECONDS = 0.05


class MESH_OT_north_remesh(bpy.types.Operator):
    """Rebuild the selected faces as evenly sized triangles or quads.
In Object Mode the whole mesh is processed"""

    bl_idname = "mesh.north_remesh"
    bl_label = "North Remesh"
    bl_options = {'REGISTER', 'UNDO'}

    output: EnumProperty(
        name="Output",
        items=[
            ('TRIS', "Triangles", "Evenly sized, near-equilateral triangles"),
            ('QUADS', "Quads", "Quad faces (see Quad Method)"),
        ],
        default='TRIS',
    )
    quad_method: EnumProperty(
        name="Quad Method",
        items=[
            ('PAIR', "Pair Triangles",
             "Join triangle pairs into quads. Clean, regular quads; some triangles remain"),
            ('SPLIT', "All Quads",
             "Split every triangle into 3 quads. Quads everywhere except along a locked border"),
        ],
        default='PAIR',
    )
    size_mode: EnumProperty(
        name="Size",
        items=[
            ('MATCH', "Match Surroundings",
             "Use the edge length of the faces around the selection"),
            ('RELATIVE', "Relative", "Percentage of the selection's size"),
            ('ABSOLUTE', "Absolute", "Fixed edge length"),
        ],
        default='MATCH',
    )
    density: FloatProperty(
        name="Scale",
        description="Multiplier on the measured edge length (below 1 = denser)",
        default=1.0, min=0.05, max=20.0, soft_min=0.25, soft_max=4.0,
    )
    adaptive: BoolProperty(
        name="Blend Into Surroundings",
        description="Grade the size from the border: dense next to dense neighbours, "
                    "coarse next to coarse ones",
        default=True,
    )
    relative_size: FloatProperty(
        name="Edge Length",
        description="Target edge length as a percentage of the selection's bounding diagonal",
        default=1.0, min=0.01, max=50.0, subtype='PERCENTAGE',
    )
    edge_length: FloatProperty(
        name="Edge Length",
        description="Target edge length (world units)",
        default=0.05, min=1e-6, soft_min=1e-4, subtype='DISTANCE', precision=4,
    )
    iterations: IntProperty(
        name="Iterations", description="Number of improvement passes",
        default=5, min=1, max=30,
    )
    smoothing: FloatProperty(
        name="Smoothing", description="How strongly vertices are evened out each pass",
        default=0.6, min=0.0, max=1.0, subtype='FACTOR',
    )
    feature_angle: FloatProperty(
        name="Keep Creases Above",
        description="Edges sharper than this angle are preserved",
        default=math.radians(45.0), min=0.0, max=math.pi, subtype='ANGLE',
    )
    keep_sharp: BoolProperty(
        name="Keep Marked Sharp", description="Preserve edges marked as sharp", default=True,
    )
    keep_materials: BoolProperty(
        name="Keep Material Borders", description="Preserve edges between materials",
        default=True,
    )
    project: BoolProperty(
        name="Stick To Surface",
        description="Keep vertices on the original surface while smoothing",
        default=True,
    )
    transfer: BoolProperty(
        name="Transfer Attributes",
        description="Give new vertices UVs, vertex group weights, shape keys and colours "
                    "sampled from the original surface",
        default=True,
    )
    refine_border: BoolProperty(
        name="Allow Border Splits",
        description="Let the selection border be subdivided too. "
                    "Neighbouring faces receive the new vertices",
        default=False,
    )
    fix_neighbors: BoolProperty(
        name="Triangulate Affected Neighbours",
        description="Triangulate outside faces that received new border vertices",
        default=True,
    )
    max_faces: IntProperty(
        name="Face Limit",
        description="Maximum number of faces to create. If the chosen size would "
                    "exceed it, the size is increased automatically",
        default=2_000_000, min=100, soft_max=10_000_000,
    )

    # ----------------------------------------------------------------- poll

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return obj is not None and obj.type == 'MESH' and context.mode in {'EDIT_MESH', 'OBJECT'}

    # ----------------------------------------------------------------- draw

    def draw(self, context):
        layout = self.layout
        layout.use_property_split = True
        layout.use_property_decorate = False
        layout.row().prop(self, "output", expand=True)
        if self.output == 'QUADS':
            layout.prop(self, "quad_method")
        col = layout.column(align=True)
        col.prop(self, "size_mode")
        if self.size_mode == 'MATCH':
            col.prop(self, "density")
            col.prop(self, "adaptive")
        elif self.size_mode == 'RELATIVE':
            col.prop(self, "relative_size")
        else:
            col.prop(self, "edge_length")
        col = layout.column(align=True)
        col.prop(self, "iterations")
        col.prop(self, "smoothing")
        col.prop(self, "project")
        col.prop(self, "transfer")
        col = layout.column(heading="Preserve", align=True)
        col.prop(self, "feature_angle")
        col.prop(self, "keep_sharp")
        col.prop(self, "keep_materials")
        col = layout.column(heading="Border", align=True)
        col.prop(self, "refine_border")
        sub = col.column()
        sub.active = self.refine_border
        sub.prop(self, "fix_neighbors")
        layout.prop(self, "max_faces")
        box = layout.box()
        box.scale_y = 0.7
        box.label(text="Changes here re-run without progress or Esc.", icon='INFO')
        box.label(text="On very large selections, undo and run again instead.")

    # ------------------------------------------------------------- lifecycle

    def _options(self):
        return Options(
            size_mode=self.size_mode,
            density=self.density,
            adaptive=self.adaptive,
            relative_size=self.relative_size / 100.0,
            edge_length=self.edge_length,
            quads=self.output == 'QUADS',
            quad_method=self.quad_method,
            iterations=self.iterations,
            feature_angle=self.feature_angle,
            keep_sharp=self.keep_sharp,
            keep_materials=self.keep_materials,
            refine_border=self.refine_border,
            fix_neighbors=self.fix_neighbors,
            smoothing=self.smoothing,
            project=self.project,
            transfer=self.transfer,
            max_faces=self.max_faces,
        )

    def _begin(self, context):
        obj = context.active_object
        self._obj = obj
        self._edit = obj.mode == 'EDIT'
        if self._edit:
            bm = bmesh.from_edit_mesh(obj.data)
            faces = [f for f in bm.faces if f.select]
        else:
            bm = bmesh.new()
            bm.from_mesh(obj.data)
            faces = list(bm.faces)
        self._bm = bm
        self._job = RegionJob(bm, faces, self._options(), obj.matrix_world)
        self._t0 = time.perf_counter()
        try:
            self._job.prepare()
        except Exception:
            self._discard()
            raise

    def _discard(self):
        job = getattr(self, "_job", None)
        if job is not None:
            job.free()
        if not getattr(self, "_edit", True) and getattr(self, "_bm", None) is not None:
            self._bm.free()
        self._bm = None

    def _restore(self, snapshot):
        """Put the edit mesh back exactly as it was before the commit."""
        me = self._obj.data
        bpy.ops.object.mode_set(mode='OBJECT')
        snapshot.to_mesh(me)
        me.update()
        bpy.ops.object.mode_set(mode='EDIT')

    def _commit(self):
        job = self._job
        me = self._obj.data
        if self._edit:
            snapshot = self._bm.copy()
            try:
                job.commit(select=True)
            except Exception:
                self._restore(snapshot)
                raise
            finally:
                snapshot.free()
            bmesh.update_edit_mesh(me, loop_triangles=True, destructive=True)
        else:
            job.commit(select=False)
            self._bm.to_mesh(me)
            self._bm.free()
            self._bm = None
            me.update()
        self._report_done()

    def _report_done(self):
        job = self._job
        dt = time.perf_counter() - self._t0
        q = job.quality or {}
        parts = ["North Remesh: {:,} -> {:,} faces".format(job.start_faces, job.end_faces)]
        if q.get("quads"):
            parts.append("{:.0f}% quads".format(100.0 * q["quads"] / max(1, job.end_faces)))
        parts.append("edge {:.4g}".format(job.length))
        if q:
            parts.append("min angle {:.0f}°, {:.1f}% poor".format(q["worst_angle"], q["poor_pct"]))
        parts.append("{:.1f}s".format(dt))
        msg = " | ".join(parts)
        if job.size_scale > 1.001:
            msg += " | size raised x{:.2f} to stay under the Face Limit".format(job.size_scale)
            self.report({'WARNING'}, msg)
        else:
            self.report({'INFO'}, msg)

    def _fail(self, ex):
        self._discard()
        if isinstance(ex, RemeshError):
            self.report({'WARNING'}, str(ex))
        else:
            traceback.print_exc()
            self.report({'ERROR'}, "North Remesh failed: {} (mesh unchanged)".format(ex))
        return {'CANCELLED'}

    # --------------------------------------------------------------- execute

    def execute(self, context):
        try:
            self._begin(context)
        except Exception as ex:
            return self._fail(ex)
        wm = context.window_manager
        wm.progress_begin(0, 100)
        try:
            for frac, _msg in self._job.run():
                wm.progress_update(int(frac * 100))
            self._commit()
        except Exception as ex:
            return self._fail(ex)
        finally:
            wm.progress_end()
        return {'FINISHED'}

    # ----------------------------------------------------------------- modal

    def invoke(self, context, event):
        try:
            self._begin(context)
        except Exception as ex:
            return self._fail(ex)
        self._gen = self._job.run()
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.01, window=context.window)
        wm.progress_begin(0, 100)
        wm.modal_handler_add(self)
        self._status(context, 0.0, "Starting")
        return {'RUNNING_MODAL'}

    def _status(self, context, frac, msg):
        text = "North Remesh: {} - {:d}%   (Esc to cancel)".format(msg, int(frac * 100))
        if context.workspace is not None:
            context.workspace.status_text_set(text)
        if context.area is not None:
            context.area.header_text_set(text)
        context.window_manager.progress_update(int(frac * 100))

    def _end_ui(self, context):
        wm = context.window_manager
        if getattr(self, "_timer", None) is not None:
            wm.event_timer_remove(self._timer)
            self._timer = None
        wm.progress_end()
        if context.workspace is not None:
            context.workspace.status_text_set(None)
        if context.area is not None:
            context.area.header_text_set(None)

    def modal(self, context, event):
        if event.type == 'ESC' and event.value == 'PRESS':
            self._end_ui(context)
            self._gen.close()
            self._discard()
            self.report({'INFO'}, "North Remesh cancelled (mesh unchanged)")
            return {'CANCELLED'}
        if event.type in _NAV_EVENTS:
            return {'PASS_THROUGH'}
        if event.type != 'TIMER':
            return {'RUNNING_MODAL'}

        start = time.perf_counter()
        frac, msg = 0.0, ""
        try:
            while time.perf_counter() - start < _SLICE_SECONDS:
                frac, msg = next(self._gen)
            self._status(context, frac, msg)
        except StopIteration:
            self._end_ui(context)
            try:
                self._commit()
            except Exception as ex:
                return self._fail(ex)
            return {'FINISHED'}
        except Exception as ex:
            self._end_ui(context)
            return self._fail(ex)
        return {'RUNNING_MODAL'}


def menu_func(self, context):
    self.layout.separator()
    self.layout.operator_context = 'INVOKE_DEFAULT'
    self.layout.operator(MESH_OT_north_remesh.bl_idname, icon='MOD_REMESH')


_MENUS = (
    "VIEW3D_MT_edit_mesh_context_menu",
    "VIEW3D_MT_edit_mesh_faces",
    "VIEW3D_MT_object_context_menu",
)


def register():
    bpy.utils.register_class(MESH_OT_north_remesh)
    for name in _MENUS:
        menu = getattr(bpy.types, name, None)
        if menu is not None:
            menu.append(menu_func)


def unregister():
    for name in _MENUS:
        menu = getattr(bpy.types, name, None)
        if menu is not None:
            menu.remove(menu_func)
    bpy.utils.unregister_class(MESH_OT_north_remesh)
