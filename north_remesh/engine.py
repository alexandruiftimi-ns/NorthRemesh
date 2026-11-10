# SPDX-License-Identifier: GPL-3.0-or-later
"""
North Remesh - remeshing engine.

Runs on a private, isolated BMesh that contains only the faces being rebuilt
(see region.py). Its open boundary is the seam with the rest of the model and
is treated as locked.

Pipeline
    refine            initial split, one-shot fill of large faces, split
                      until no edge exceeds 4/3 of its target (bounded)
    N x  split        edges > 4/3 target          (topological split)
         collapse     edges < 4/5 target          (link condition, no fold-over)
         flip         towards valence 6
         relax        area-weighted tangential smoothing, snapped back onto
                      the source surface; crease vertices slide along creases
    finish            Delaunay flips, degenerate cleanup
    quads (optional)  pair triangles, or split every triangle into 3 quads
    validate          refuse anything non-manifold, degenerate or leaky

The target edge length comes from a SizeField: uniform, or graded from the
border so a fill blends into neighbours of different density.

Every loop is bounded. Heavy loops yield periodically so the UI stays live.
"""

import math
import os

import bmesh
from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import delaunay_2d_cdt
from mathutils.geometry import normal as poly_normal
from mathutils.kdtree import KDTree

FEATURE = "_nr_feature"            # edge int: crease / material seam / marked sharp
SHARP = "_nr_sharp"                # edge int: user had this edge marked sharp
SEAM = "_nr_seam"                  # edge int: shared with faces outside the region
PIN = "_nr_pin"                    # vert int: 1 = seam vertex, 2 = non-manifold input
SIZE = "_nr_size"                  # vert float: target edge length at this vertex
ORIGIN = ("_nr_ox", "_nr_oy", "_nr_oz")  # vert float: original local coordinate

TRI_AREA = math.sqrt(3.0) / 4.0
SPLIT_QUAD_FACTOR = 2.6            # triangle size that yields quads of the target size
_CHAIN_STRAIGHT = math.radians(150.0)
_MAX_REFINE_PASSES = 40
_MAX_CUTS = 4096
_CHUNK = 0x1FFF                    # yield to the UI every ~8k elements
_DEBUG = bool(os.environ.get("NORTH_REMESH_DEBUG"))


class RemeshError(Exception):
    """User-facing failure. The user's mesh is left unchanged."""


class BudgetExceeded(RemeshError):
    """Internal: the run would exceed the face limit; retry with larger size."""


# --------------------------------------------------------------------- sizing

class SizeField:
    """Target edge length in space: uniform, or graded from border samples.

    Graded: h(p) = min(h_max, min_i(h_i + g * |p - s_i|)) over nearby border
    samples s_i with local size h_i. Sizes grow smoothly away from dense
    borders and never exceed the coarsest border size.
    """

    GRADING = 0.3

    def __init__(self, uniform, samples=None):
        self.uniform = float(uniform)
        self.kd = None
        self.h = []
        if samples and len(samples) >= 2:
            hs = [h for _p, h in samples]
            if max(hs) > min(hs) * 1.5:     # only real density changes
                kd = KDTree(len(samples))
                for i, (p, _h) in enumerate(samples):
                    kd.insert(p, i)
                kd.balance()
                self.kd = kd
                self.h = hs
        self._refresh()

    def _refresh(self):
        if self.kd is None:
            self.hmin = self.hmax = self.uniform
        else:
            self.hmin = min(self.h)
            self.hmax = max(self.h)

    @property
    def graded(self):
        return self.kd is not None

    def scale(self, k):
        self.uniform *= k
        self.h = [x * k for x in self.h]
        self._refresh()

    def at(self, p):
        if self.kd is None:
            return self.uniform
        best = self.hmax
        g = self.GRADING
        h = self.h
        for _co, i, d in self.kd.find_n(p, 6):
            val = h[i] + g * d
            if val < best:
                best = val
        return best


def estimate_triangles(area, length):
    return int(area / (TRI_AREA * length * length))


# --------------------------------------------------------------------- engine

class Engine:
    def __init__(self, bm, field, opts, face_cap):
        self.bm = bm
        self.o = opts
        self.field = field
        self.graded = field.graded
        self.L = field.uniform
        self.Lmin = field.hmin
        el, vl = bm.edges.layers, bm.verts.layers
        self.et = el.int[FEATURE]
        self.st = el.int[SHARP]
        self.sm = el.int[SEAM]
        self.pt = vl.int[PIN]
        self.sz = vl.float[SIZE]
        self._flip_cos = math.cos(min(opts.feature_angle, math.radians(30.0)))
        self._budget = int(face_cap * 1.3) + 1000
        # (2 * triangle area)^2 below which a triangle counts as a sliver.
        self._min_area2 = (self.Lmin * self.Lmin * 1e-3) ** 2
        self._p = (0.0, "Preparing")
        self._dirty = None               # None = everything needs checking
        self.bvh = None
        if opts.project and not self._is_planar():
            self.bvh = BVHTree.FromBMesh(bm)
        self._base_seam = self._count_seam()
        self._base_euler = self._euler()
        self._pins = self._pin_coords()
        self.quality = None

    # -------------------------------------------------------------------- run

    def run(self):
        """Generator yielding (fraction 0..1, message)."""
        self._p = (0.01, "Preparing")
        yield self._p
        if self.graded:
            yield from self._update_sizes(list(self.bm.verts))
        self._p = (0.03, "Refining")
        yield from self._refine()

        n = max(1, int(self.o.iterations))
        span = 0.6 / n
        for i in range(n):
            base = 0.25 + i * span
            tag = "Pass {}/{}".format(i + 1, n)
            self._p = (base, tag + ": split")
            yield from self._split()
            self._triangulate()
            self._check_budget()
            self._dbg("split")
            self._p = (base + span * 0.25, tag + ": collapse")
            yield from self._collapse()
            self._dbg("collapse")
            self._p = (base + span * 0.5, tag + ": flip")
            yield from self._flip(valence=True)
            self._dbg("flip")
            self._p = (base + span * 0.75, tag + ": smooth")
            yield from self._relax(self.o.smoothing)
            yield self._p

        self._p = (0.86, "Finishing")
        yield from self._flip(valence=False, full=True)
        yield from self._relax(self.o.smoothing * 0.5)
        yield from self._cleanup()
        self._dbg("cleanup")

        if self.o.quads:
            self._p = (0.92, "Building quads")
            if self.o.quad_method == 'SPLIT':
                yield from self._split_quads()
            else:
                yield from self._pair_quads()
            self._dbg("quads")

        self._p = (0.96, "Validating")
        yield from self._validate()
        self.quality = yield from self._measure_quality()
        self._p = (1.0, "Done")
        yield self._p

    # ---------------------------------------------------------------- helpers

    def _is_planar(self):
        """Flat source (e.g. a cap): tangential smoothing is already exact,
        so the surface projection can be skipped entirely."""
        bm = self.bm
        if not bm.faces:
            return True
        bm.normal_update()
        n = Vector((0.0, 0.0, 0.0))
        for f in bm.faces:
            n += f.normal * f.calc_area()
        if n.length_squared < 1e-30:
            return False
        n.normalize()
        bm.verts.ensure_lookup_table()
        p0 = bm.verts[0].co.copy()
        tol = self.Lmin * 1e-2
        for v in bm.verts:
            if abs(n.dot(v.co - p0)) > tol:
                return False
        return True

    def _pin_coords(self):
        pt = self.pt
        return {tuple(v.co) for v in self.bm.verts if v[pt] == 1}

    def _euler(self):
        bm = self.bm
        return len(bm.verts) - len(bm.edges) + len(bm.faces)

    def _count_seam(self):
        sm = self.sm
        return sum(1 for e in self.bm.edges if e[sm])

    def _dbg(self, stage):
        if _DEBUG:
            print("  [north] {:<10} faces={} seam={}/{} euler={}/{}".format(
                stage, len(self.bm.faces), self._count_seam(), self._base_seam,
                self._euler(), self._base_euler))

    def _check_budget(self):
        if len(self.bm.faces) > self._budget:
            raise BudgetExceeded("Face limit exceeded while refining")

    def _locked(self, v):
        if v.is_boundary or not v.is_manifold:
            return True
        et = self.et
        for e in v.link_edges:
            if e[et]:
                return True
        return False

    def _target(self, e):
        if not self.graded:
            return self.L
        a, b = e.verts
        s = (a[self.sz] + b[self.sz]) * 0.5
        return s if s > 0.0 else self.L

    def _size(self, v):
        if not self.graded:
            return self.L
        s = v[self.sz]
        return s if s > 0.0 else self.L

    def _update_sizes(self, verts):
        if not self.graded:
            return
        at = self.field.at
        sz = self.sz
        for i, v in enumerate(verts):
            if not i & _CHUNK:
                yield self._p
            if v.is_valid:
                v[sz] = at(v.co)

    def _mark(self, faces):
        if self._dirty is not None:
            self._dirty.update(faces)

    # ----------------------------------------------------------------- refine

    def _refine(self):
        yield from self._split(initial=True)
        self._dbg("isplit")
        yield from self._fill_large_faces()
        self._dbg("fill")
        self._triangulate()
        self._check_budget()
        yield from self._delaunay(3)
        best = None
        stalled = 0
        for k in range(_MAX_REFINE_PASSES):
            self._p = (min(0.25, 0.05 + 0.02 * k), "Refining")
            count = yield from self._split()
            if count == 0:
                break
            self._triangulate()
            self._check_budget()
            yield from self._delaunay(2)
            self._dbg("refine")
            # Stop if splitting no longer makes progress.
            if best is not None and count >= best:
                stalled += 1
                if stalled >= 3:
                    break
            else:
                best = count
                stalled = 0

    def _delaunay(self, rounds):
        for _ in range(rounds):
            flips = yield from self._flip(valence=False)
            if flips == 0:
                break

    # ------------------------------------------------------------------- fill

    def _fill_large_faces(self):
        """Fill big faces (e.g. a planar cap) in one step: lattice points + CDT.

        Much faster than repeatedly splitting a fan of long slivers, and gives
        near-ideal triangles from the start. Faces that can't be filled safely
        are left for the normal triangulate/split path.
        """
        big = []
        for f in self.bm.faces:
            if len(f.verts) <= 3:
                continue
            t = self.field.at(f.calc_center_median()) if self.graded else self.L
            if f.calc_area() > 6.0 * TRI_AREA * t * t:
                big.append(f)
        for f in big:
            if f.is_valid:
                yield from self._fill_face(f)

    def _fill_face(self, f):
        graded = self.graded
        n = f.normal.copy()
        if n.length_squared < 1e-20:
            n = poly_normal([v.co for v in f.verts])
            if n.length_squared < 1e-20:
                return False
        n.normalize()
        ref = Vector((1.0, 0.0, 0.0)) if abs(n.x) < 0.9 else Vector((0.0, 1.0, 0.0))
        u = n.cross(ref).normalized()
        w = n.cross(u)
        center = f.calc_center_median()

        ring = list(f.verts)
        pts = []
        for v in ring:
            d = v.co - center
            pts.append(Vector((d.dot(u), d.dot(w))))
        xs = [p.x for p in pts]
        ys = [p.y for p in pts]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)

        if graded:
            sizes = sorted(self._size(v) for v in ring)
            L = sizes[len(sizes) // 2]
        else:
            L = self.L

        # Sample the boundary densely so the distance test sees the edges,
        # not just the corners (avoids slivers along long border edges).
        step = self.Lmin * 0.2
        samples = []
        nb = len(pts)
        for i in range(nb):
            p, q = pts[i], pts[(i + 1) % nb]
            k = max(1, int((q - p).length / step))
            for j in range(k):
                samples.append(p.lerp(q, j / k))
        kd = KDTree(len(samples))
        for i, p in enumerate(samples):
            kd.insert((p.x, p.y, 0.0), i)
        kd.balance()

        dy = L * math.sqrt(3.0) / 2.0
        rows = int((y1 - y0) / dy) + 1
        cols = int((x1 - x0) / L) + 2
        if rows * cols > self._budget * 2:
            raise BudgetExceeded("Fill would exceed the face limit")
        lattice = []
        at = self.field.at
        count = 0
        for r in range(rows):
            y = y0 + (r + 0.5) * dy
            x = x0 + (0.5 * L if r & 1 else 0.0)
            for _ in range(cols):
                if x > x1:
                    break
                count += 1
                if not count & _CHUNK:
                    yield self._p
                keep_off = 0.6 * (at(center + u * x + w * y) if graded else L)
                if kd.find((x, y, 0.0))[2] >= keep_off:
                    lattice.append(Vector((x, y)))
                x += L

        coords = pts + lattice
        try:
            out = delaunay_2d_cdt(coords, [], [list(range(nb))], 1, self.Lmin * 1e-5, True)
        except Exception:
            return False
        o_verts, _o_edges, o_faces, o_orig, _oe, _of = out

        # Map CDT output back. Any vertex the CDT invented (self-crossing
        # outline, merged points) means the polygon isn't simple -> give up.
        bm_verts = [None] * len(o_verts)
        interior = {}
        for i, origs in enumerate(o_orig):
            if not origs:
                return False
            k = min(origs)
            if k < nb:
                if len(origs) > 1 and max(origs) < nb:
                    return False
                bm_verts[i] = ring[k]
            else:
                interior[i] = k
        if not o_faces:
            return False

        t0 = o_faces[0]
        a, b, c = o_verts[t0[0]], o_verts[t0[1]], o_verts[t0[2]]
        flip = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) < 0.0

        mat, smooth = f.material_index, f.smooth
        bmesh.ops.delete(self.bm, geom=[f], context='FACES_ONLY')
        new_vert = self.bm.verts.new
        pt, sz = self.pt, self.sz
        for i, k in interior.items():
            p = coords[k]
            nv = new_vert(center + u * p.x + w * p.y)
            nv[pt] = 0
            if graded:
                nv[sz] = at(nv.co)
            bm_verts[i] = nv
        new_face = self.bm.faces.new
        made = []
        for t in o_faces:
            if len(t) != 3:
                continue
            vs = [bm_verts[t[0]], bm_verts[t[1]], bm_verts[t[2]]]
            if vs[0] is vs[1] or vs[1] is vs[2] or vs[0] is vs[2]:
                continue
            if flip:
                vs.reverse()
            try:
                nf = new_face(vs)
            except ValueError:
                continue
            nf.material_index = mat
            nf.smooth = smooth
            made.append(nf)
        self._mark(made)
        loose = [bm_verts[i] for i in interior if not bm_verts[i].link_faces]
        if loose:
            bmesh.ops.delete(self.bm, geom=loose, context='VERTS')
        return True

    # ------------------------------------------------------------------ split

    def _split(self, initial=False):
        border_ok = self.o.refine_border
        sm = self.sm
        groups = {}
        for i, e in enumerate(self.bm.edges):
            if not i & _CHUNK:
                yield self._p
            length = e.calc_length()
            t = self._target(e)
            if length <= t * (4.0 / 3.0):
                continue
            if e[sm] and not border_ok:
                continue
            lf = e.link_faces
            if len(lf) > 2:
                continue
            if not border_ok and length <= self._seam_limit(e, lf):
                continue
            cuts = min(_MAX_CUTS, max(1, math.ceil(length / t) - 1)) if initial else 1
            groups.setdefault(cuts, []).append(e)

        total = 0
        if not initial:
            # Topological split: midpoint + connect to the opposite corners.
            # The new vertex is fresh, so no edge can ever be duplicated.
            leftovers = []
            for i, e in enumerate(groups.get(1, ())):
                if not i & _CHUNK:
                    yield self._p
                if not e.is_valid:
                    continue
                if self._split_edge(e):
                    total += 1
                else:
                    leftovers.append(e)
            groups = {1: leftovers} if leftovers else {}

        pt = self.pt
        for cuts, edges in groups.items():
            # Note: bisect_edges invalidates Python vertex references.
            res = bmesh.ops.bisect_edges(self.bm, edges=edges, cuts=cuts)
            for x in res["geom_split"]:
                if isinstance(x, bmesh.types.BMVert):
                    x[pt] = 0          # don't inherit an interpolated PIN value
            total += len(edges)
        return total

    def _split_edge(self, e):
        lf = e.link_faces
        if not lf:
            return False
        for f in lf:
            if len(f.verts) != 3:
                return False
        faces = list(lf)
        _ne, nv = bmesh.utils.edge_split(e, e.verts[0], 0.5)
        nv[self.pt] = 0
        et, st, sm = self.et, self.st, self.sm
        changed = []
        for f in faces:
            # f is now a quad; find the corner opposite the new vertex.
            opp = None
            for loop in f.loops:
                if loop.vert == nv:
                    opp = loop.link_loop_next.link_loop_next.vert
                    break
            if opp is None:
                continue
            res = bmesh.utils.face_split(f, nv, opp)
            if res is None or res[0] is None:
                continue
            new_f, loop = res
            le = loop.edge
            le[et] = 0
            le[st] = 0
            le[sm] = 0
            changed.append(f)
            changed.append(new_f)
        self._mark(changed)
        return True

    def _seam_limit(self, e, lf):
        """A triangle on a locked seam edge of length s can't have its other
        edges shorter than ~s/2; relax the limit there instead of splitting
        forever towards the seam."""
        sm = self.sm
        longest = 0.0
        for f in lf:
            for x in f.edges:
                if x is not e and x[sm]:
                    xl = x.calc_length()
                    if xl > longest:
                        longest = xl
        return longest * 0.75

    def _triangulate(self):
        faces = [f for f in self.bm.faces if len(f.verts) > 3]
        if not faces:
            return
        res = bmesh.ops.triangulate(
            self.bm, faces=faces, quad_method='BEAUTY', ngon_method='BEAUTY'
        )
        et, st, sm = self.et, self.st, self.sm
        for e in res["edges"]:
            e[et] = 0
            e[st] = 0
            e[sm] = 0
        self._mark(res["faces"])
        doubles = [f for f in res["face_map_double"].keys() if f.is_valid]
        if doubles:
            bmesh.ops.delete(self.bm, geom=doubles, context='FACES_ONLY')

    # --------------------------------------------------------------- collapse

    def _collapse(self):
        et, st, sm = self.et, self.st, self.sm
        cands = []
        for i, e in enumerate(self.bm.edges):
            if not i & _CHUNK:
                yield self._p
            length = e.calc_length()
            if length < self._target(e) * 0.8:
                cands.append((length, e))
        if not cands:
            return 0
        cands.sort(key=lambda t: t[0])

        touched = set()
        targetmap = {}
        restore = []
        for i, (_l, e) in enumerate(cands):
            if not i & _CHUNK:
                yield self._p
            a, b = e.verts
            if a in touched or b in touched:
                continue
            plan = self._plan_collapse(e, a, b)
            if plan is None:
                continue
            rem, keep, pos, size = plan
            for v in (a, b):
                touched.add(v)
                for e2 in v.link_edges:
                    touched.add(e2.other_vert(v))
            for v in (keep, rem):
                for e2 in v.link_edges:
                    o = e2.other_vert(v)
                    if o is keep or o is rem:
                        continue
                    if e2[et] or e2[st] or e2[sm]:
                        restore.append((keep, o, e2[et], e2[st], e2[sm]))
            keep.co = pos
            if self.graded:
                keep[self.sz] = size
            targetmap[rem] = keep

        if not targetmap:
            return 0
        keeps = list(targetmap.values())
        bmesh.ops.weld_verts(self.bm, targetmap=targetmap)
        for k in keeps:
            if k.is_valid:
                self._mark(k.link_faces)
        # Edges merged by the weld may have dropped their flags; put them back.
        get = self.bm.edges.get
        for k, o, f, s, m in restore:
            if k.is_valid and o.is_valid:
                ed = get((k, o))
                if ed is not None:
                    if f:
                        ed[et] = 1
                    if s:
                        ed[st] = 1
                    if m:
                        ed[sm] = 1
        return len(targetmap)

    def _plan_collapse(self, e, a, b):
        la, lb = self._locked(a), self._locked(b)
        if not la and not lb:
            rem, keep, pos = a, b, (a.co + b.co) * 0.5
            size = (self._size(a) + self._size(b)) * 0.5
        elif la and lb:
            if not e[self.et] and not e.is_boundary:
                return None
            if self._chain_removable(a, e):
                rem, keep = a, b
            elif self._chain_removable(b, e):
                rem, keep = b, a
            else:
                return None
            pos = keep.co.copy()
            size = self._size(keep)
        elif la:
            rem, keep, pos = b, a, a.co.copy()
            size = self._size(a)
        else:
            rem, keep, pos = a, b, b.co.copy()
            size = self._size(b)

        # Link condition (keeps the surface a 2-manifold).
        na = {x.other_vert(a) for x in a.link_edges}
        nb = {x.other_vert(b) for x in b.link_edges}
        common = na & nb
        lf = e.link_faces
        if len(lf) == 0 or len(lf) > 2:
            return None
        for f in lf:
            if len(f.verts) != 3:
                return None
        if len(common) != len(lf):
            return None
        for c in common:
            if len(c.link_edges) <= 3:
                return None
        if a.is_boundary and b.is_boundary and not e.is_boundary:
            return None   # would pinch the mesh

        graded = self.graded
        for n in na | nb:
            if n is a or n is b:
                continue
            limit = ((self._size(n) + size) * 0.5 if graded else self.L) * (4.0 / 3.0)
            if (n.co - pos).length > limit:
                return None

        min_area2 = self._min_area2
        for v in (a, b):
            for f in v.link_faces:
                fv = f.verts
                if len(fv) != 3:
                    return None
                has_a = has_b = False
                for x in fv:
                    if x is a:
                        has_a = True
                    elif x is b:
                        has_b = True
                if has_a and has_b:
                    continue
                p0, p1, p2 = fv[0].co, fv[1].co, fv[2].co
                n_old = (p1 - p0).cross(p2 - p0)
                q = [pos if (x is a or x is b) else x.co for x in fv]
                n_new = (q[1] - q[0]).cross(q[2] - q[0])
                if n_new.length_squared < min_area2:
                    return None
                if n_old.length_squared < 1e-30:
                    continue
                if n_new.normalized().dot(n_old.normalized()) < 0.5:
                    return None
        return rem, keep, pos, size

    def _chain_removable(self, v, e):
        if v[self.pt]:
            return False
        et = self.et
        line = [x for x in v.link_edges if x[et] or x.is_boundary]
        if len(line) != 2 or e not in line:
            return False
        other = line[0] if line[1] == e else line[1]
        p = v.co
        d1 = e.other_vert(v).co - p
        d2 = other.other_vert(v).co - p
        if d1.length_squared < 1e-30 or d2.length_squared < 1e-30:
            return True
        return d1.angle(d2) >= _CHAIN_STRAIGHT

    # ------------------------------------------------------------------- flip

    def _take_candidates(self, full):
        """Edges worth re-checking: around faces changed since the last flip."""
        bm = self.bm
        dirty = self._dirty
        self._dirty = set()
        if full or dirty is None or len(dirty) * 3 > len(bm.faces):
            return list(bm.edges)
        edges = set()
        for f in dirty:
            if not f.is_valid:
                continue
            for v in f.verts:
                for g in v.link_faces:
                    edges.update(g.edges)
        return list(edges)

    def _flip(self, valence, full=False):
        et = self.et
        get = self.bm.edges.get
        rotate = bmesh.utils.edge_rotate
        flips = 0
        queue = self._take_candidates(full)
        budget = len(queue) * 3 + 1000
        changed = []
        i = 0
        while i < len(queue) and i < budget:
            if not i & _CHUNK:
                yield self._p
            e = queue[i]
            i += 1
            if not e.is_valid or e[et]:
                continue
            lf = e.link_faces
            if len(lf) != 2:
                continue
            v1, v2 = lf[0].verts, lf[1].verts
            if len(v1) != 3 or len(v2) != 3:
                continue
            a, b = e.verts
            c = d = None
            for x in v1:
                if x is not a and x is not b:
                    c = x
            for x in v2:
                if x is not a and x is not b:
                    d = x
            if c is None or d is None or c is d or get((c, d)) is not None:
                continue

            if valence:
                ta = 4 if a.is_boundary else 6
                tb = 4 if b.is_boundary else 6
                tc = 4 if c.is_boundary else 6
                td = 4 if d.is_boundary else 6
                va, vb = len(a.link_edges), len(b.link_edges)
                vc, vd = len(c.link_edges), len(d.link_edges)
                if va <= 3 or vb <= 3:
                    continue
                before = (va - ta) ** 2 + (vb - tb) ** 2 + (vc - tc) ** 2 + (vd - td) ** 2
                after = ((va - 1 - ta) ** 2 + (vb - 1 - tb) ** 2
                         + (vc + 1 - tc) ** 2 + (vd + 1 - td) ** 2)
                if after >= before:
                    continue
            else:
                ca, cb = a.co - c.co, b.co - c.co
                da, db = a.co - d.co, b.co - d.co
                if (ca.length_squared < 1e-30 or cb.length_squared < 1e-30
                        or da.length_squared < 1e-30 or db.length_squared < 1e-30):
                    continue
                if ca.angle(cb) + da.angle(db) <= math.pi + 1e-6:
                    continue

            if not self._flip_ok(a, b, c, d, v1, v2):
                continue
            ne = rotate(e, False)
            if ne is None:
                continue
            ne[et] = 0
            ne[self.st] = 0
            ne[self.sm] = 0
            flips += 1
            for f in ne.link_faces:
                changed.append(f)
                if not valence:
                    # Delaunay: a flip can make the surrounding edges flippable.
                    for x in f.edges:
                        if x != ne:
                            queue.append(x)
        self._mark(changed)
        return flips

    def _flip_ok(self, a, b, c, d, v1, v2):
        n0 = poly_normal([x.co for x in v1]) + poly_normal([x.co for x in v2])
        if n0.length_squared < 1e-20:
            return False
        n0.normalize()
        pc = c.co
        dc = d.co - pc
        na = dc.cross(a.co - pc)
        nb = dc.cross(b.co - pc)
        sa, sb = n0.dot(na), n0.dot(nb)
        if sa * sb >= 0.0:                     # quad not convex -> fold-over
            return False
        if na.length_squared < self._min_area2 or nb.length_squared < self._min_area2:
            return False
        if sa < 0.0:
            na.negate()
        if sb < 0.0:
            nb.negate()
        return na.normalized().dot(nb.normalized()) >= self._flip_cos

    # ------------------------------------------------------------------ relax

    def _slidable(self, v):
        """Crease vertex that may slide along its (nearly straight) crease."""
        if v[self.pt] or v.is_boundary or not v.is_manifold:
            return False
        et = self.et
        fe = [e for e in v.link_edges if e[et]]
        if len(fe) != 2:
            return False
        d1 = fe[0].other_vert(v).co - v.co
        d2 = fe[1].other_vert(v).co - v.co
        if d1.length_squared < 1e-30 or d2.length_squared < 1e-30:
            return False
        return d1.angle(d2) >= _CHAIN_STRAIGHT

    def _relax(self, strength, iterations=1):
        """Area-weighted tangential smoothing.

        Each free vertex moves towards the centroid of its neighbours,
        weighted by their vertex area (divided by the squared target size
        when graded), restricted to the tangent plane and snapped back onto
        the source surface.
        """
        if strength <= 0.0:
            return
        bm = self.bm
        sz = self.sz
        graded = self.graded
        free = []
        chains = []
        for i, v in enumerate(bm.verts):
            if not i & _CHUNK:
                yield self._p
            if self._locked(v):
                if self._slidable(v):
                    chains.append(v)
            else:
                free.append(v)
        bvh = self.bvh
        reach = self.field.hmax * 2.0

        for _ in range(iterations):
            bm.normal_update()
            bm.verts.index_update()
            area = [0.0] * len(bm.verts)
            for i, f in enumerate(bm.faces):
                if not i & _CHUNK:
                    yield self._p
                verts = f.verts
                share = f.calc_area() / len(verts)
                for v in verts:
                    area[v.index] += share

            targets = []
            for i, v in enumerate(free):
                if not i & _CHUNK:
                    yield self._p
                edges = v.link_edges
                if len(edges) < 3:
                    targets.append(None)
                    continue
                acc = Vector((0.0, 0.0, 0.0))
                wsum = 0.0
                for e in edges:
                    o = e.other_vert(v)
                    wgt = area[o.index]
                    if graded:
                        s = o[sz]
                        if s > 0.0:
                            wgt /= s * s
                    acc += o.co * wgt
                    wsum += wgt
                if wsum <= 1e-30:
                    acc = Vector((0.0, 0.0, 0.0))
                    for e in edges:
                        acc += e.other_vert(v).co
                    wsum = len(edges)
                delta = acc / wsum - v.co
                nrm = v.normal
                delta -= nrm * nrm.dot(delta)
                targets.append(v.co + delta * strength)

            for i, (v, p) in enumerate(zip(free, targets)):
                if not i & _CHUNK:
                    yield self._p
                if p is None:
                    continue
                if bvh is not None:
                    loc, nrm, _i, _d = bvh.find_nearest(p, reach)
                    if loc is not None and nrm.dot(v.normal) > 0.2:
                        p = loc
                v.co = p

            if chains:
                self._slide_creases(chains, strength)
            if graded:
                yield from self._update_sizes(free + chains)

    def _slide_creases(self, chains, strength):
        """Even out spacing along creases: move along the crease edge only."""
        et = self.et
        for v in chains:
            fe = [e for e in v.link_edges if e[et]]
            if len(fe) != 2:
                continue
            d1v = fe[0].other_vert(v).co - v.co
            d2v = fe[1].other_vert(v).co - v.co
            d1, d2 = d1v.length, d2v.length
            if d1 < 1e-12 or d2 < 1e-12:
                continue
            shift = (d2 - d1) * 0.5 * strength
            if shift > 0.0:
                move = d2v * (min(shift, d2 * 0.45) / d2)
            else:
                move = d1v * (min(-shift, d1 * 0.45) / d1)
            if move.length_squared < 1e-30:
                continue
            faces = list(v.link_faces)
            before = [poly_normal([x.co for x in f.verts]) for f in faces]
            old = v.co.copy()
            v.co = old + move
            for f, nb in zip(faces, before):
                if poly_normal([x.co for x in f.verts]).dot(nb) <= 0.2:
                    v.co = old
                    break

    # ---------------------------------------------------------------- cleanup

    def _cleanup(self):
        bm = self.bm
        sm, pt = self.sm, self.pt
        safe = [e for e in bm.edges
                if not e[sm] and not e.verts[0][pt] and not e.verts[1][pt]]
        yield self._p
        bmesh.ops.dissolve_degenerate(bm, dist=self.Lmin * 1e-4, edges=safe)
        self._triangulate()
        loose = [v for v in bm.verts if not v.link_faces]
        if loose:
            bmesh.ops.delete(bm, geom=loose, context='VERTS')
        yield self._p

    # ------------------------------------------------------------------ quads

    def _pair_quads(self):
        """Pair triangles into quads; a few triangles remain where pairing
        would produce bad quads."""
        bm = self.bm
        et, st = self.et, self.st
        for e in bm.edges:
            e.smooth = not e[et]      # join_triangles respects sharp edges
        yield self._p
        kw = dict(
            faces=list(bm.faces), cmp_seam=False, cmp_sharp=True, cmp_uvs=False,
            cmp_vcols=False, cmp_materials=True,
            angle_face_threshold=math.radians(40.0),
            angle_shape_threshold=math.radians(60.0),
        )
        try:
            bmesh.ops.join_triangles(bm, topology_influence=1.0, **kw)
        except TypeError:             # Blender without topology-aware pairing
            bmesh.ops.join_triangles(bm, **kw)
        for e in bm.edges:
            e.smooth = not e[st]
        yield from self._relax(self.o.smoothing * 0.5, iterations=2)

    def _split_quads(self):
        """Split every triangle into 3 quads (corner, two edge midpoints,
        centre). Edges on a locked border are not split; triangles there
        produce 2 quads + 1 triangle."""
        bm = self.bm
        et, st, sm, pt, sz = self.et, self.st, self.sm, self.pt, self.sz
        graded = self.graded
        border_ok = self.o.refine_border

        mids = {}
        halves = []
        new_vert = bm.verts.new
        for i, e in enumerate(list(bm.edges)):
            if not i & _CHUNK:
                yield self._p
            if e[sm] and not border_ok:
                continue
            if not any(len(f.verts) == 3 for f in e.link_faces):
                continue
            a, b = e.verts
            m = new_vert((a.co + b.co) * 0.5)
            m[pt] = 0
            if graded:
                m[sz] = (a[sz] + b[sz]) * 0.5
            mids[e] = m
            halves.append((a, b, m, e[et], e[st], e[sm]))
        mid_set = set(mids.values())

        tris = [f for f in bm.faces if len(f.verts) == 3]
        new_face = bm.faces.new
        for i, f in enumerate(tris):
            if not i & _CHUNK:
                yield self._p
            ring = []
            for loop in f.loops:
                ring.append(loop.vert)
                m = mids.get(loop.edge)
                if m is not None:
                    ring.append(m)
            midpos = [k for k, x in enumerate(ring) if x in mid_set]
            if not midpos:
                continue
            o = new_vert(f.calc_center_median())
            o[pt] = 0
            if graded:
                o[sz] = sum(v[sz] for v in f.verts) / 3.0
            polys = []
            if len(midpos) == 1:
                k = midpos[0]
                rot = ring[k:] + ring[:k]          # [M, A, B, C]
                polys.append([rot[0], rot[1], rot[2], o])
                polys.append([rot[2], rot[3], rot[0], o])
            else:
                cnt = len(midpos)
                for j in range(cnt):
                    k0, k1 = midpos[j], midpos[(j + 1) % cnt]
                    seg = ring[k0:k1 + 1] if k1 > k0 else ring[k0:] + ring[:k1 + 1]
                    if len(seg) == 4:              # [Mi, X, Y, Mj] -> quad + tri
                        polys.append([seg[0], seg[1], seg[2], o])
                        polys.append([seg[2], seg[3], o])
                    else:
                        polys.append(seg + [o])
            mat, smooth = f.material_index, f.smooth
            for poly in polys:
                try:
                    nf = new_face(poly)
                except ValueError:
                    raise RemeshError("Quad split failed. Nothing was changed.")
                nf.material_index = mat
                nf.smooth = smooth

        old = [f for f in tris if f.is_valid and any(x in mids for x in f.edges)]
        bmesh.ops.delete(bm, geom=old, context='FACES_ONLY')
        dead = [e for e in mids if e.is_valid and not e.link_faces]
        if dead:
            bmesh.ops.delete(bm, geom=dead, context='EDGES')
        get = bm.edges.get
        for a, b, m, f_, s_, m_ in halves:
            for x in (a, b):
                ed = get((x, m))
                if ed is not None:
                    ed[et], ed[st], ed[sm] = f_, s_, m_
        yield from self._relax(self.o.smoothing, iterations=3)

    # --------------------------------------------------------------- validate

    def _validate(self):
        """Exact checks: every problem element must have been there before
        (pinned input), not merely 'the same number of problems'."""
        bm = self.bm
        pt = self.pt
        if not bm.faces:
            raise RemeshError("Result was empty. Nothing was changed.")
        for i, e in enumerate(bm.edges):
            if not i & _CHUNK:
                yield self._p
            n = len(e.link_faces)
            if n == 0:
                raise RemeshError("Result would contain loose edges. Nothing was changed.")
            if n > 2 and not (e.verts[0][pt] and e.verts[1][pt]):
                raise RemeshError(
                    "Result would contain non-manifold edges. Nothing was changed. "
                    "Try a larger size or fewer iterations.")
        for i, v in enumerate(bm.verts):
            if not i & _CHUNK:
                yield self._p
            if not v.link_faces:
                raise RemeshError("Result would contain loose vertices. Nothing was changed.")
            if not v.is_manifold and not v[pt]:
                raise RemeshError(
                    "Result would contain non-manifold vertices. Nothing was changed. "
                    "Try a larger size or fewer iterations.")
        if self._euler() != self._base_euler:
            raise RemeshError(
                "Result would change the surface topology (hole or merge). "
                "Nothing was changed. Try a larger size.")
        if not self._pins <= self._pin_coords():
            raise RemeshError("Border would be broken. Nothing was changed.")
        if not self.o.refine_border and self._count_seam() != self._base_seam:
            raise RemeshError("Border would change. Nothing was changed.")
        tiny = (self.Lmin * 1e-4) ** 2
        for i, f in enumerate(bm.faces):
            if not i & _CHUNK:
                yield self._p
            if f.calc_area() < tiny:
                raise RemeshError("Result would contain degenerate faces. Nothing was changed.")

    def _measure_quality(self):
        tris = quads = other = low = 0
        worst = 180.0
        for i, f in enumerate(self.bm.faces):
            if not i & _CHUNK:
                yield self._p
            loops = f.loops
            n = len(loops)
            if n == 3:
                tris += 1
                limit = 20.0
            elif n == 4:
                quads += 1
                limit = 45.0
            else:
                other += 1
                limit = 20.0
            m = math.degrees(min(l.calc_angle() for l in loops))
            if m < worst:
                worst = m
            if m < limit:
                low += 1
        total = max(1, tris + quads + other)
        return {
            "tris": tris, "quads": quads, "other": other,
            "worst_angle": worst, "poor_pct": 100.0 * low / total,
        }
