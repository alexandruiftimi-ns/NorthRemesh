"""
North Remesh test suite. Run headless:

    blender -b --factory-startup --python tests/test_north_remesh.py

Exits with code 1 if any check fails. Every scenario verifies that the result
adds no non-manifold edges/vertices, no degenerate faces and no new holes.
"""

import os
import statistics
import sys
import time
import traceback

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import bmesh  # noqa: E402
import bpy  # noqa: E402

import north_remesh  # noqa: E402

north_remesh.register()

RESULTS = []


# ------------------------------------------------------------------ helpers

def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def obj():
    return bpy.context.active_object


def object_mode():
    if obj().mode == 'EDIT':
        bpy.ops.object.mode_set(mode='OBJECT')


def boundary_loops(bm):
    """Number of separate boundary loops (holes / open rims)."""
    parent = {}

    def find(x):
        while parent.get(x, x) != x:
            parent[x] = parent.get(parent[x], parent[x])
            x = parent[x]
        return x

    verts = set()
    for e in bm.edges:
        if len(e.link_faces) == 1:
            a, b = (v.index for v in e.verts)
            verts.update((a, b))
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb
    return len({find(v) for v in verts})


def topo(o):
    """(boundary loops, >2-face edges, non-manifold verts, degenerate faces, faces)."""
    bm = bmesh.new()
    bm.from_mesh(o.data)
    bm.verts.index_update()
    open_e = boundary_loops(bm)
    multi = sum(1 for e in bm.edges if len(e.link_faces) > 2)
    nmv = sum(1 for v in bm.verts if not v.is_manifold)
    deg = sum(1 for f in bm.faces if f.calc_area() < 1e-12)
    n = len(bm.faces)
    bm.free()
    return open_e, multi, nmv, deg, n


def selected_faces(o):
    return [p for p in o.data.polygons if p.select]


def edge_stats(o, polys, world=False):
    me = o.data
    M = o.matrix_world
    seen = set()
    lengths = []
    for p in polys:
        for ek in p.edge_keys:
            if ek in seen:
                continue
            seen.add(ek)
            a, b = me.vertices[ek[0]].co, me.vertices[ek[1]].co
            if world:
                a, b = M @ a, M @ b
            lengths.append((a - b).length)
    mean = statistics.mean(lengths)
    return mean, statistics.pstdev(lengths) / mean


def run_op(**kw):
    t = time.perf_counter()
    try:
        r = bpy.ops.mesh.north_remesh('EXEC_DEFAULT', **kw)
    except RuntimeError as ex:
        r = {'CANCELLED'}
        print("   op message:", str(ex).strip())
    return r, time.perf_counter() - t


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print("{} {:<44} {}".format("PASS" if ok else "FAIL", name, detail), flush=True)


def clean_result(name, before, after, dt, extra=""):
    open0, multi0, nmv0, _d0, _n0 = before
    open1, multi1, nmv1, deg1, n1 = after
    ok = open1 == open0 and multi1 <= multi0 and nmv1 <= nmv0 and deg1 == 0
    check(name, ok, "{:.2f}s faces={:,} holes={}/{} nonmanifold e={} v={} degenerate={} {}".format(
        dt, n1, open1, open0, multi1, nmv1, deg1, extra))
    return ok


def scenario(fn):
    def wrapper():
        try:
            fn()
        except Exception:
            traceback.print_exc()
            check(fn.__name__, False, "exception")
    wrapper.__name__ = fn.__name__
    return wrapper


def make_cap_sphere(subdiv=8):
    reset()
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=subdiv, radius=1.0)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.bisect(plane_co=(0, 0, 0.3), plane_no=(0.2, 0.1, 1),
                        clear_outer=True, use_fill=True)
    bm = bmesh.from_edit_mesh(obj().data)
    big = max(bm.faces, key=lambda f: len(f.verts))
    for f in bm.faces:
        f.select = False
    big.select = True
    bm.select_flush(True)
    bmesh.update_edit_mesh(obj().data)


# ---------------------------------------------------------------- scenarios

@scenario
def flat_ngon_tris():
    reset()
    bpy.ops.mesh.primitive_circle_add(vertices=64, radius=1.0, fill_type='NGON')
    before = topo(obj())
    r, dt = run_op(size_mode='ABSOLUTE', edge_length=0.03)
    object_mode()
    mean, cv = edge_stats(obj(), obj().data.polygons)
    clean_result("flat n-gon -> triangles", before, topo(obj()), dt,
                 "edge={:.4f} cv={:.3f}".format(mean, cv))
    check("flat n-gon: edge length ~ target", 'FINISHED' in r and abs(mean - 0.03) < 0.006 and cv < 0.15)


@scenario
def flat_ngon_quads():
    for method in ('PAIR', 'SPLIT'):
        reset()
        bpy.ops.mesh.primitive_circle_add(vertices=64, radius=1.0, fill_type='NGON')
        before = topo(obj())
        r, dt = run_op(size_mode='ABSOLUTE', edge_length=0.03, output='QUADS', quad_method=method)
        object_mode()
        polys = obj().data.polygons
        quads = sum(1 for p in polys if len(p.vertices) == 4)
        pct = 100.0 * quads / len(polys)
        clean_result("flat n-gon -> quads ({})".format(method), before, topo(obj()), dt,
                     "quads={:.0f}%".format(pct))
        need = 95.0 if method == 'SPLIT' else 70.0
        check("quads {} ratio >= {:.0f}%".format(method, need), 'FINISHED' in r and pct >= need,
              "{:.1f}%".format(pct))


@scenario
def heavy_cap():
    make_cap_sphere()
    object_mode()
    before = topo(obj())
    bpy.ops.object.mode_set(mode='EDIT')
    r, dt = run_op()
    object_mode()
    after = topo(obj())
    mean, cv = edge_stats(obj(), selected_faces(obj()))
    clean_result("327k sphere: cut + cap, match", before, after, dt,
                 "edge={:.4f} cv={:.3f}".format(mean, cv))
    check("heavy cap: finished", 'FINISHED' in r)


@scenario
def heavy_cap_quads():
    make_cap_sphere()
    object_mode()
    before = topo(obj())
    bpy.ops.object.mode_set(mode='EDIT')
    r, dt = run_op(output='QUADS', quad_method='SPLIT')
    object_mode()
    clean_result("327k sphere: cap -> all quads", before, topo(obj()), dt)
    check("heavy cap quads: finished", 'FINISHED' in r)


@scenario
def face_limit():
    make_cap_sphere()
    object_mode()
    before = topo(obj())
    bpy.ops.object.mode_set(mode='EDIT')
    r, dt = run_op(max_faces=5000)
    object_mode()
    n = len(selected_faces(obj()))
    clean_result("face limit 5,000 on large cap", before, topo(obj()), dt, "created={:,}".format(n))
    check("face limit respected", 'FINISHED' in r and n <= 5000, "{:,} <= 5,000".format(n))
    for method, limit in (('PAIR', 3000), ('SPLIT', 6000)):
        make_cap_sphere(6)
        r, dt = run_op(max_faces=limit, output='QUADS', quad_method=method)
        object_mode()
        n = len(selected_faces(obj()))
        check("face limit respected ({})".format(method), 'FINISHED' in r and n <= limit,
              "{:,} <= {:,}".format(n, limit))
    make_cap_sphere(6)
    object_mode()
    n0 = len(obj().data.polygons)
    bpy.ops.object.mode_set(mode='EDIT')
    r, dt = run_op(max_faces=200)
    object_mode()
    check("impossible face limit refused, mesh unchanged",
          'CANCELLED' in r and len(obj().data.polygons) == n0)


@scenario
def cube_sharp_quads():
    reset()
    bpy.ops.mesh.primitive_cube_add()
    before = topo(obj())
    r, dt = run_op(size_mode='RELATIVE', relative_size=4.0, output='QUADS')
    object_mode()
    bm = bmesh.new()
    bm.from_mesh(obj().data)
    from mathutils import Vector
    corners_kept = all(any((v.co - Vector(c)).length < 1e-5 for v in bm.verts)
                       for c in [(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    flat = all(abs(abs(v.co.x) - 1) < 1e-4 or abs(abs(v.co.y) - 1) < 1e-4
               or abs(abs(v.co.z) - 1) < 1e-4 for v in bm.verts)
    bm.free()
    clean_result("cube with sharp edges -> quads", before, topo(obj()), dt)
    check("cube: corners and faces preserved", 'FINISHED' in r and corners_kept and flat)


@scenario
def patch_border_splits():
    reset()
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    before = topo(obj())
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(obj().data)
    for f in bm.faces:
        f.select = f.calc_center_median().z > 0.5
    bm.select_flush(True)
    bmesh.update_edit_mesh(obj().data)
    r, dt = run_op(density=0.4, refine_border=True)
    object_mode()
    clean_result("patch with border splits (stitching)", before, topo(obj()), dt)
    check("border splits: finished", 'FINISHED' in r)


@scenario
def whole_heavy_object():
    reset()
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=8, radius=1.0)
    before = topo(obj())
    r, dt = run_op(iterations=3)
    clean_result("whole 327k object", before, topo(obj()), dt)
    check("whole object: finished", 'FINISHED' in r)


@scenario
def size_guard():
    reset()
    bpy.ops.mesh.primitive_plane_add(size=1000)
    r, dt = run_op(size_mode='ABSOLUTE', edge_length=0.001, max_faces=20000)
    n = len(obj().data.polygons)
    check("tiny size is capped by the face limit", 'FINISHED' in r and n <= 20000,
          "{:,} faces in {:.2f}s".format(n, dt))


@scenario
def attribute_transfer():
    reset()
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    o = obj()
    vg = o.vertex_groups.new(name="arm")
    vg.add(list(range(len(o.data.vertices))), 1.0, 'REPLACE')
    o.shape_key_add(name="Basis")
    key = o.shape_key_add(name="Up")
    for p in key.data:
        p.co.z += 0.5
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(o.data)
    for f in bm.faces:
        f.select = f.calc_center_median().z > 0.5
    bm.select_flush(True)
    bmesh.update_edit_mesh(o.data)
    r, dt = run_op(density=0.5)
    object_mode()
    me = o.data
    up = me.shape_keys.key_blocks["Up"]
    no_group = sum(1 for v in me.vertices if not v.groups or abs(v.groups[0].weight - 1.0) > 1e-3)
    bad_shape = sum(1 for i, v in enumerate(me.vertices)
                    if abs((up.data[i].co - v.co).z - 0.5) > 1e-3)
    uv = me.uv_layers.active.data
    zero_uv = sum(1 for p in selected_faces(o) for li in p.loop_indices if uv[li].uv.length < 1e-9)
    check("attributes: vertex groups kept", 'FINISHED' in r and no_group == 0, "missing={}".format(no_group))
    check("attributes: shape keys kept", bad_shape == 0, "wrong={}".format(bad_shape))
    check("attributes: UVs filled", zero_uv == 0, "blank corners={}".format(zero_uv))


@scenario
def nonuniform_scale():
    reset()
    bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16)
    obj().scale = (1.0, 1.0, 3.0)
    before = topo(obj())
    r, dt = run_op(size_mode='ABSOLUTE', edge_length=0.15)
    mean, cv = edge_stats(obj(), obj().data.polygons, world=True)
    clean_result("non-uniform scale (1,1,3)", before, topo(obj()), dt,
                 "world edge={:.3f} cv={:.3f}".format(mean, cv))
    check("non-uniform scale: even in world space", 'FINISHED' in r and abs(mean - 0.15) < 0.03 and cv < 0.2)


@scenario
def border_doubles():
    reset()
    bm = bmesh.new()
    for x0 in (-1.0, 0.0):
        bmesh.ops.create_grid(bm, x_segments=4, y_segments=4, size=0.5,
                              matrix=__import__("mathutils").Matrix.Translation((x0 + 0.5, 0, 0)))
    me = bpy.data.meshes.new("doubles")
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new("doubles", me)
    bpy.context.collection.objects.link(o)
    bpy.context.view_layer.objects.active = o
    o.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(me)
    for f in bm.faces:
        f.select = f.calc_center_median().length < 0.3
    bm.select_flush(True)
    bmesh.update_edit_mesh(me)
    faces_before = len(bm.faces)
    r, dt = run_op(density=0.5)
    object_mode()
    check("border doubles are refused safely", 'CANCELLED' in r and len(me.polygons) == faces_before)


@scenario
def graded_fill():
    reset()
    bpy.ops.mesh.primitive_grid_add(x_subdivisions=24, y_subdivisions=24, size=2)
    o = obj()
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(o.data)
    for f in bm.faces:
        f.select = f.calc_center_median().x < 0
    bm.select_flush(True)
    bmesh.update_edit_mesh(o.data)
    bpy.ops.mesh.subdivide(number_cuts=3)
    bm = bmesh.from_edit_mesh(o.data)
    hole = [f for f in bm.faces if f.calc_center_median().length < 0.55]
    bmesh.ops.delete(bm, geom=hole, context='FACES')
    ring = [e for e in bm.edges if e.is_boundary and e.verts[0].co.length < 0.9]
    res = bmesh.ops.edgeloop_fill(bm, edges=ring)
    for f in bm.faces:
        f.select = False
    for f in res["faces"]:
        f.select = True
    bm.select_flush(True)
    bmesh.update_edit_mesh(o.data)
    object_mode()
    before = topo(o)
    bpy.ops.object.mode_set(mode='EDIT')
    r, dt = run_op()
    object_mode()
    polys = selected_faces(o)
    me = o.data

    def mean_edge(cond):
        ls = []
        for p in polys:
            c = p.center
            if cond(c):
                for ek in p.edge_keys:
                    ls.append((me.vertices[ek[0]].co - me.vertices[ek[1]].co).length)
        return statistics.mean(ls) if ls else 0.0

    dense = mean_edge(lambda c: c.x < -0.35)
    coarse = mean_edge(lambda c: c.x > 0.35)
    clean_result("graded fill between dense/coarse", before, topo(o), dt,
                 "dense side={:.4f} coarse side={:.4f}".format(dense, coarse))
    check("graded: denser next to dense neighbours", 'FINISHED' in r and 0 < dense < coarse * 0.75)


@scenario
def suzanne_cut():
    for out in ('TRIS', 'QUADS'):
        reset()
        bpy.ops.mesh.primitive_monkey_add()
        o = obj()
        m = o.modifiers.new("s", 'SUBSURF')
        m.levels = 4
        bpy.ops.object.modifier_apply(modifier="s")
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.quads_convert_to_tris()
        bpy.ops.mesh.bisect(plane_co=(0, 0, 0.2), plane_no=(0.3, 0.5, 1),
                            clear_outer=True, use_fill=True)
        object_mode()
        before = topo(o)
        bpy.ops.object.mode_set(mode='EDIT')
        bm = bmesh.from_edit_mesh(o.data)
        caps = [f for f in bm.faces if len(f.verts) > 4]
        for f in bm.faces:
            f.select = False
        for f in caps:
            f.select = True
        bm.select_flush(True)
        bmesh.update_edit_mesh(o.data)
        r, dt = run_op(output=out)
        object_mode()
        clean_result("250k Suzanne cut + caps -> {}".format(out.lower()), before, topo(o), dt)


# --------------------------------------------------------------------- main

if __name__ == "__main__":
    t0 = time.perf_counter()
    for fn in (flat_ngon_tris, flat_ngon_quads, heavy_cap, heavy_cap_quads, face_limit,
               cube_sharp_quads, patch_border_splits, whole_heavy_object, size_guard,
               attribute_transfer, nonuniform_scale, border_doubles, graded_fill,
               suzanne_cut):
        print("\n==", fn.__name__, flush=True)
        fn()
    failed = [r for r in RESULTS if not r[1]]
    print("\n{} checks, {} failed, {:.0f}s total".format(len(RESULTS), len(failed),
                                                         time.perf_counter() - t0))
    for name, _ok, detail in failed:
        print("  FAILED:", name, detail)
    sys.exit(1 if failed else 0)
