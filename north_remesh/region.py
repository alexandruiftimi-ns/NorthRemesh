# SPDX-License-Identifier: GPL-3.0-or-later
"""
North Remesh - region handling.

The faces to rebuild are copied into a private BMesh (in world space, so
non-uniform object scale doesn't distort the result). The engine works only
on that copy, so:
    * cost scales with the selection, not with the whole model
    * the user's mesh is untouched until the result has been validated
    * cancelling is free (just drop the copy)

When the result is accepted it is written back in one step: the old faces
are removed and the new ones are stitched onto the original border vertices,
which are reused (never duplicated). The written result is verified again;
if that fails, the caller restores its snapshot.
"""

import math
import statistics

import bmesh
from mathutils import Matrix

from .attributes import SurfaceSampler
from .engine import (
    FEATURE, ORIGIN, PIN, SEAM, SHARP, SIZE, SPLIT_QUAD_FACTOR, TRI_AREA,
    BudgetExceeded, Engine, RemeshError, SizeField,
)

_MAX_ATTEMPTS = 4


class Options:
    def __init__(self, **kw):
        self.size_mode = kw.get("size_mode", 'MATCH')       # MATCH / RELATIVE / ABSOLUTE
        self.density = kw.get("density", 1.0)               # multiplier for MATCH
        self.adaptive = kw.get("adaptive", True)            # graded size from border
        self.relative_size = kw.get("relative_size", 0.01)  # fraction of bounding diagonal
        self.edge_length = kw.get("edge_length", 0.05)      # world units
        self.quads = kw.get("quads", False)
        self.quad_method = kw.get("quad_method", 'PAIR')    # PAIR / SPLIT
        self.iterations = kw.get("iterations", 5)
        self.feature_angle = kw.get("feature_angle", math.radians(45.0))
        self.keep_sharp = kw.get("keep_sharp", True)
        self.keep_materials = kw.get("keep_materials", True)
        self.refine_border = kw.get("refine_border", False)
        self.fix_neighbors = kw.get("fix_neighbors", True)
        self.smoothing = kw.get("smoothing", 0.6)
        self.project = kw.get("project", True)
        self.transfer = kw.get("transfer", True)
        self.max_faces = kw.get("max_faces", 2_000_000)


class RegionJob:
    """prepare() -> run() generator -> commit(). Raises RemeshError on problems."""

    def __init__(self, src, faces, opts, matrix=None):
        self.src = src
        self.faces = faces
        self.o = opts
        self.M = matrix.copy() if matrix is not None else Matrix.Identity(4)
        self.identity = self.M == Matrix.Identity(4)
        self.Minv = self.M.inverted_safe()
        self.iso = None
        self.engine = None
        self.field = None
        self.sampler = None
        self.size_scale = 1.0          # >1 when the size was raised for the face limit
        self.length = 0.0              # target edge length used (world units)
        self.start_faces = len(faces)
        self.end_faces = 0
        self.quality = None
        self.attempts = 0

    # ------------------------------------------------------------- utilities

    def _w(self, co):
        return co.copy() if self.identity else self.M @ co

    def _local(self, co):
        return co.copy() if self.identity else self.Minv @ co

    # ---------------------------------------------------------------- prepare

    def prepare(self):
        if not self.faces:
            raise RemeshError("No faces selected")
        region = set(self.faces)
        self._region = region
        self._border = self._border_verts(region)
        self._check_border_doubles(region)
        self.field = self._make_field(region)
        self._build(region)

        area = sum(f.calc_area() for f in self.iso.faces)
        if area <= 0.0:
            self.free()
            raise RemeshError("Selected faces have no area")
        self._area = area

        if self.o.quads and self.o.quad_method == 'SPLIT':
            self.field.scale(SPLIT_QUAD_FACTOR)

        # A locked border needs about one face per border edge whatever the
        # size, so it is a fixed floor on the count.
        seam = sum(1 for e in self.iso.edges if e.is_boundary)
        if self.o.quads:
            per = 3.0 if self.o.quad_method == 'SPLIT' else 0.5
        else:
            per = 1.0
        floor = 0 if self.o.refine_border else int(seam * per)
        if floor > self.o.max_faces:
            self.free()
            raise RemeshError(
                "The selection border has {:,} edges, so at least about {:,} faces are "
                "needed. Raise the Face Limit.".format(seam, floor))
        self._floor = floor

        est = self._estimate()
        if est > self.o.max_faces:
            room = max(self.o.max_faces * 0.9 - floor, self.o.max_faces * 0.1)
            k = math.sqrt((est - floor) / room) * 1.03
            if k > 1.0:
                self.field.scale(k)
                self.size_scale *= k
        self.estimate = self._estimate()

        if self.o.transfer:
            sampler = SurfaceSampler(self.src, self.faces)
            self.sampler = sampler if sampler.active else None
        self._new_engine()

    def _estimate(self):
        """Expected number of final faces for the current field."""
        L = self.field.uniform
        tris = self._area / (TRI_AREA * L * L)
        if not self.o.quads:
            body = tris
        elif self.o.quad_method == 'SPLIT':
            body = tris * 3.0
        else:
            body = tris * 0.6
        return body + getattr(self, "_floor", 0)

    def _face_cap(self):
        m = self.o.max_faces
        if not self.o.quads:
            return m
        if self.o.quad_method == 'SPLIT':
            return m / 3.0
        return m * 2.0

    def _new_engine(self):
        self.engine = Engine(self.iso, self.field, self.o, self._face_cap())
        self.length = self.field.uniform / (
            SPLIT_QUAD_FACTOR if (self.o.quads and self.o.quad_method == 'SPLIT') else 1.0)

    def _border_verts(self, region):
        """Selected-face verts that also touch an unselected face (computed once)."""
        if len(region) == len(self.src.faces):
            return set()
        verts = {v for f in self.faces for v in f.verts}
        return {v for v in verts if any(g not in region for g in v.link_faces)}

    def _check_border_doubles(self, region):
        seen = {}
        for v in self._border:
            key = tuple(v.co)
            if key in seen:
                raise RemeshError(
                    "The selection border has overlapping vertices (doubles). "
                    "Use Merge by Distance first.")
            seen[key] = v

    def _make_field(self, region):
        o = self.o
        if o.size_mode == 'ABSOLUTE':
            L = o.edge_length
            field = SizeField(L)
        elif o.size_mode == 'RELATIVE':
            lo = [math.inf] * 3
            hi = [-math.inf] * 3
            for f in self.faces:
                for v in f.verts:
                    c = self._w(v.co)
                    for k in range(3):
                        lo[k] = min(lo[k], c[k])
                        hi[k] = max(hi[k], c[k])
            diag = math.sqrt(sum((hi[k] - lo[k]) ** 2 for k in range(3)))
            field = SizeField(diag * o.relative_size)
        else:
            field = self._surrounding_field(region)
        if not (field.uniform > 0.0 and math.isfinite(field.uniform)):
            raise RemeshError("Could not determine a valid edge length")
        return field

    def _surrounding_field(self, region):
        """Local edge length of the unselected faces around each border vertex."""
        o = self.o
        border = self._border
        pts = []
        for v in border:
            ring1 = [g for g in v.link_faces if g not in region]
            # Look one ring further out and skip edges touching the border:
            # a fresh cut leaves sliced, unrepresentative faces right there.
            inner = {u for g in ring1 for u in g.verts if u not in border}
            lengths = []
            seen = set()
            for u in inner:
                for g in u.link_faces:
                    if g in region:
                        continue
                    for e in g.edges:
                        if e in seen:
                            continue
                        seen.add(e)
                        a, b = e.verts
                        if a in border or b in border:
                            continue
                        x = (self._w(a.co) - self._w(b.co)).length
                        if x > 0.0:
                            lengths.append(x)
            if not lengths:   # thin strip: fall back to the first ring
                for g in ring1:
                    for e in g.edges:
                        a, b = e.verts
                        x = (self._w(a.co) - self._w(b.co)).length
                        if x > 0.0:
                            lengths.append(x)
            if lengths:
                pts.append((self._w(v.co), statistics.median(lengths)))
        if pts:
            med = statistics.median(h for _p, h in pts)
            if o.adaptive and len(pts) >= 4:
                pts = self._smooth_samples(pts, med)
                return SizeField(med * o.density, [(p, h * o.density) for p, h in pts])
            return SizeField(med * o.density)
        # Whole object / isolated part: keep its own median density.
        seen = set()
        lengths = []
        for f in self.faces:
            for e in f.edges:
                if e not in seen:
                    seen.add(e)
                    a, b = e.verts
                    x = (self._w(a.co) - self._w(b.co)).length
                    if x > 0.0:
                        lengths.append(x)
        if not lengths:
            raise RemeshError("Could not measure the edge length")
        return SizeField(statistics.median(lengths) * o.density)

    @staticmethod
    def _smooth_samples(pts, med):
        """Remove noise (e.g. tiny sliver faces left by a cut): clamp each
        sample to [0.25, 4] x the median, then take the median of its
        neighbourhood. Only real density changes survive."""
        from mathutils.kdtree import KDTree
        lo, hi = med * 0.25, med * 4.0
        hs = [min(hi, max(lo, h)) for _p, h in pts]
        kd = KDTree(len(pts))
        for i, (p, _h) in enumerate(pts):
            kd.insert(p, i)
        kd.balance()
        k = min(len(pts), 12)
        out = []
        for p, _h in pts:
            near = [hs[i] for _c, i, _d in kd.find_n(p, k)]
            out.append((p, statistics.median(near)))
        return out

    def _build(self, region):
        o = self.o
        iso = bmesh.new()
        et = iso.edges.layers.int.new(FEATURE)
        st = iso.edges.layers.int.new(SHARP)
        sm = iso.edges.layers.int.new(SEAM)
        pt = iso.verts.layers.int.new(PIN)
        iso.verts.layers.float.new(SIZE)
        ox, oy, oz = (iso.verts.layers.float.new(n) for n in ORIGIN)

        vmap = {}
        new_vert = iso.verts.new
        new_face = iso.faces.new
        for f in self.faces:
            ring = []
            for v in f.verts:
                iv = vmap.get(v)
                if iv is None:
                    iv = vmap[v] = new_vert(self._w(v.co))
                    iv[ox], iv[oy], iv[oz] = v.co
                ring.append(iv)
            try:
                nf = new_face(ring)
            except ValueError:
                continue  # duplicate face in the source; skip it
            nf.material_index = f.material_index
            nf.smooth = f.smooth

        use_angle = o.feature_angle < math.pi - 1e-6
        seen = set()
        get = iso.edges.get
        for f in self.faces:
            for e in f.edges:
                if e in seen:
                    continue
                seen.add(e)
                a, b = e.verts
                ie = get((vmap[a], vmap[b]))
                if ie is None:
                    continue
                lf = e.link_faces
                sharp = not e.smooth
                seam = any(g not in region for g in lf)
                feat = len(lf) != 2 or seam
                if not feat and o.keep_sharp and sharp:
                    feat = True
                if not feat and o.keep_materials and lf[0].material_index != lf[1].material_index:
                    feat = True
                if not feat and use_angle and e.calc_face_angle(0.0) > o.feature_angle:
                    feat = True
                ie[et] = 1 if feat else 0
                ie[st] = 1 if sharp else 0
                ie[sm] = 1 if seam else 0
                ie.smooth = not sharp

        # PIN = 1: seam vertex shared with outside faces. It never moves and is
        # matched back by its original coordinate. PIN = 2: non-manifold input.
        border = self._border
        for v, iv in vmap.items():
            if v in border:
                iv[pt] = 1
            elif not iv.is_manifold:
                iv[pt] = 2
        self.iso = iso

    # -------------------------------------------------------------------- run

    def run(self):
        """Generator yielding (fraction, message). Retries with a larger size
        if the result would exceed the face limit."""
        previous = None
        for attempt in range(_MAX_ATTEMPTS):
            self.attempts = attempt + 1
            try:
                yield from self.engine.run()
            except BudgetExceeded:
                count = None
            else:
                count = len(self.iso.faces)
                if count <= self.o.max_faces:
                    self.quality = self.engine.quality
                    return
            if count is not None and previous is not None and count > previous * 0.97:
                break   # a larger size no longer helps (the border dominates)
            previous = count
            k = 1.4 if count is None else math.sqrt(count / self.o.max_faces) * 1.03
            self.field.scale(k)
            self.size_scale *= k
            self.iso.free()
            self._build(self._region)
            self._new_engine()
        raise RemeshError(
            "Could not stay under the Face Limit ({:,}). Raise the limit or "
            "use a larger size.".format(self.o.max_faces))

    def free(self):
        if self.iso is not None:
            self.iso.free()
            self.iso = None

    # ----------------------------------------------------------------- commit

    def _anchors(self, region):
        """Original local coordinate -> source vertex, for border verts.
        Rebuilt from the faces each time (vertex refs go stale after splits)."""
        out = {}
        if not self._border:
            return out
        for f in self.faces:
            for v in f.verts:
                for g in v.link_faces:
                    if g not in region:
                        out[tuple(v.co)] = v
                        break
        return out

    def commit(self, select=True):
        """Write the result into the source mesh. On RemeshError after the
        source was touched, the caller must restore its snapshot."""
        src, iso = self.src, self.iso
        pt = iso.verts.layers.int[PIN]
        st = iso.edges.layers.int[SHARP]
        sm = iso.edges.layers.int[SEAM]
        ox, oy, oz = (iso.verts.layers.float[n] for n in ORIGIN)
        region = set(self.faces)

        def key(iv):
            return (iv[ox], iv[oy], iv[oz])

        # ---- checks that need no changes
        anchors = self._anchors(region)
        for iv in iso.verts:
            if iv[pt] == 1 and key(iv) not in anchors:
                raise RemeshError("Border could not be matched. Nothing was changed.")
        pre_bad = {k for k, v in anchors.items() if not v.is_manifold}
        open_edges = sum(1 for e in iso.edges if e.is_boundary and not e[sm])

        # ---- border splits (only with "Allow Border Splits")
        plans = self._plan_stitch(region, anchors, key) if self.o.refine_border else []
        chain_verts = {iv for p in plans for iv in p[3]}

        # Sample attributes while the original faces are still untouched.
        vdata = {}
        fdata = []
        sampler = self.sampler
        if sampler is not None:
            if sampler.has_vertex_data:
                for iv in iso.verts:
                    reused = (iv[pt] == 1 and key(iv) in anchors) or iv in chain_verts
                    if not reused:
                        vdata[iv] = sampler.sample_vertex(self._local(iv.co))
            if sampler.has_loop_data:
                for f in iso.faces:
                    corners = [self._local(v.co) for v in f.verts]
                    center = sum(corners, corners[0] * 0.0) / len(corners)
                    fdata.append(sampler.sample_face(center, corners))

        touched_outside = []
        stitched = {}
        if plans:
            touched_outside, stitched = self._apply_stitch(plans)
            anchors = self._anchors(region)

        old_edges = {e for f in self.faces for e in f.edges}
        old_verts = {v for f in self.faces for v in f.verts}
        bmesh.ops.delete(src, geom=list(self.faces), context='FACES_ONLY')

        out = {}
        new_verts = []
        new_vert = src.verts.new
        for iv in iso.verts:
            sv = None
            if iv[pt] == 1:
                sv = anchors.get(key(iv))
            elif iv in stitched:
                sv = stitched[iv]
            if sv is None:
                sv = new_vert(self._local(iv.co))
                new_verts.append(sv)
                if iv in vdata:
                    sampler.write_vertex(sv, vdata[iv])
            out[iv] = sv

        new_faces = []
        new_face = src.faces.new
        for f in iso.faces:
            try:
                nf = new_face([out[v] for v in f.verts])
            except ValueError:
                raise RemeshError("Could not create a face while writing the result.")
            nf.material_index = f.material_index
            nf.smooth = f.smooth
            if fdata:
                sampler.write_face(nf, fdata[len(new_faces)])
            new_faces.append(nf)
        get = src.edges.get
        for ie in iso.edges:
            if ie[st]:
                se = get((out[ie.verts[0]], out[ie.verts[1]]))
                if se is not None:
                    se.smooth = False

        loose_e = [e for e in old_edges if e.is_valid and not e.link_faces]
        if loose_e:
            bmesh.ops.delete(src, geom=loose_e, context='EDGES')
        loose_v = [v for v in old_verts if v.is_valid and not v.link_edges]
        if loose_v:
            bmesh.ops.delete(src, geom=loose_v, context='VERTS')

        if touched_outside and self.o.fix_neighbors:
            fix = [f for f in touched_outside if f.is_valid and len(f.verts) > 3]
            if fix:
                bmesh.ops.triangulate(src, faces=fix, quad_method='BEAUTY', ngon_method='BEAUTY')

        self._verify_written(new_faces, pre_bad, open_edges)

        src.normal_update()
        if select:
            for f in new_faces:
                if f.is_valid:
                    f.select = True
            src.select_flush(True)
        self.end_faces = len(new_faces)
        self.free()

    def _verify_written(self, new_faces, pre_bad, open_edges):
        """The written faces must be manifold and closed against the border."""
        edges = set()
        verts = set()
        for f in new_faces:
            if not f.is_valid:
                continue
            edges.update(f.edges)
            verts.update(f.verts)
        boundary = 0
        for e in edges:
            n = len(e.link_faces)
            if n > 2:
                raise RemeshError("Writing the result created non-manifold edges.")
            if n == 1:
                boundary += 1
        if boundary != open_edges:
            raise RemeshError("Writing the result left a gap at the border.")
        for v in verts:
            if not v.is_manifold and tuple(v.co) not in pre_bad:
                raise RemeshError("Writing the result created non-manifold vertices.")

    def _plan_stitch(self, region, anchors, key):
        """Find seam edges the engine split. Read-only: no mesh changes."""
        iso, src = self.iso, self.src
        pt = iso.verts.layers.int[PIN]
        get = src.edges.get
        visited = set()
        plans = []
        limit = len(iso.edges) + 1
        for start in iso.verts:
            if start[pt] != 1:
                continue
            for e0 in start.link_edges:
                if not e0.is_boundary or e0 in visited:
                    continue
                chain = []
                prev, edge, end = start, e0, None
                for _ in range(limit):
                    visited.add(edge)
                    nxt = edge.other_vert(prev)
                    if nxt[pt] == 1:
                        end = nxt
                        break
                    chain.append(nxt)
                    following = [x for x in nxt.link_edges if x.is_boundary and x != edge]
                    if len(following) != 1:
                        break
                    prev, edge = nxt, following[0]
                if end is None or not chain:
                    continue
                su = anchors.get(key(start))
                sw = anchors.get(key(end))
                if su is None or sw is None:
                    continue
                se = get((su, sw))
                if se is None:
                    continue
                outside = [g for g in se.link_faces if g not in region]
                if outside:
                    plans.append((se, su.co.copy(), sw.co.copy(), chain, outside))
        return plans

    def _apply_stitch(self, plans):
        """Split the outside side of each seam edge to match the new vertices.
        Returns (touched outside faces, {iso vert: new source vert})."""
        if not plans:
            return [], {}
        groups = {}
        for p in plans:
            groups.setdefault(len(p[3]), []).append(p[0])
        for cuts, edges in groups.items():
            bmesh.ops.bisect_edges(self.src, edges=edges, cuts=cuts)
        # Vertex refs are stale after bisect; find the new verts through faces.
        touched = []
        stitched = {}
        for _se, a, b, chain, outside in plans:
            ab = b - a
            len2 = ab.length_squared
            if len2 <= 0.0:
                raise RemeshError("Border stitching failed.")
            tol2 = len2 * 1e-8
            found = {}
            for g in outside:
                for v in g.verts:
                    t = (v.co - a).dot(ab) / len2
                    if 1e-6 < t < 1.0 - 1e-6 and ((a + ab * t) - v.co).length_squared <= tol2:
                        found[v] = t
            verts = sorted(found, key=found.get)
            if len(verts) != len(chain):
                raise RemeshError("Border stitching failed.")
            local = [(iv, self._local(iv.co)) for iv in chain]
            local.sort(key=lambda t: (t[1] - a).dot(ab))
            for v, (iv, c) in zip(verts, local):
                v.co = c
                stitched[iv] = v
            touched.extend(outside)
        return touched, stitched
