# SPDX-License-Identifier: GPL-3.0-or-later
"""
North Remesh - attribute transfer.

New geometry gets its data from the original surface. The original faces are
still intact when sampling happens (just before they are replaced), so the
sampler reads them directly instead of copying everything up front:

    * one surface lookup per new vertex  -> vertex groups, shape keys,
                                            point colours
    * one surface lookup per new face    -> UVs and corner colours for all
                                            its corners (taken from a single
                                            original face, so a new face never
                                            straddles a UV seam)

Values are interpolated barycentrically (linearly extrapolated for corners
just outside the original triangle). Vertices reused from the border keep
their own data.
"""

from mathutils import Vector
from mathutils.bvhtree import BVHTree
from mathutils.geometry import tessellate_polygon


class SurfaceSampler:
    def __init__(self, bm, faces):
        L = bm.loops.layers
        V = bm.verts.layers
        self.uv = list(L.uv.values())
        self.lcol = list(L.color.values()) + list(L.float_color.values())
        self.vcol = list(V.color.values()) + list(V.float_color.values())
        self.deform = V.deform.active
        self.shape = list(V.shape.values())
        self.has_vertex_data = bool(self.vcol or self.deform or self.shape)
        self.has_loop_data = bool(self.uv or self.lcol)
        self.active = self.has_vertex_data or self.has_loop_data
        self.bvh = None
        if not self.active:
            return
        # Triangles of the original faces (n-gons tessellated once), keeping
        # references to their corners. Sampling happens before the source is
        # modified, so these references stay valid.
        index = {}
        coords = []
        tris = []
        corners = []
        for f in faces:
            loops = list(f.loops)
            ids = []
            for l in loops:
                v = l.vert
                i = index.get(v)
                if i is None:
                    i = index[v] = len(coords)
                    coords.append(v.co.copy())
                ids.append(i)
            if len(loops) == 3:
                parts = ((0, 1, 2),)
            else:
                parts = tessellate_polygon([[l.vert.co for l in loops]])
            for a, b, c in parts:
                tris.append((ids[a], ids[b], ids[c]))
                corners.append((loops[a], loops[b], loops[c]))
        self.corners = corners
        self._frames = {}
        if tris:
            self.bvh = BVHTree.FromPolygons(coords, tris, all_triangles=True)
        else:
            self.active = False

    # ------------------------------------------------------------- sampling

    def _frame(self, idx):
        """Per-triangle barycentric constants, computed on first use."""
        fr = self._frames.get(idx)
        if fr is None:
            tri = self.corners[idx]
            a = tri[0].vert.co.copy()
            v0 = tri[1].vert.co - a
            v1 = tri[2].vert.co - a
            d00, d01, d11 = v0.dot(v0), v0.dot(v1), v1.dot(v1)
            den = d00 * d11 - d01 * d01
            inv = 1.0 / den if abs(den) > 1e-30 else 0.0
            fr = self._frames[idx] = (a, v0, v1, d00, d01, d11, inv)
        return fr

    @staticmethod
    def _weights(fr, p):
        a, v0, v1, d00, d01, d11, inv = fr
        if inv == 0.0:
            return 1.0, 0.0, 0.0
        v2 = p - a
        d20, d21 = v2.dot(v0), v2.dot(v1)
        v = (d11 * d20 - d01 * d21) * inv
        w = (d00 * d21 - d01 * d20) * inv
        return 1.0 - v - w, v, w

    def _locate(self, p, weights=True):
        """Nearest original triangle -> (index, three loops, weights of p)."""
        loc, _n, idx, _d = self.bvh.find_nearest(p)
        if idx is None:
            return None
        tri = self.corners[idx]
        return idx, tri, (self._weights(self._frame(idx), loc) if weights else None)

    def sample_vertex(self, p):
        if not self.has_vertex_data:
            return None
        hit = self._locate(p)
        if hit is None:
            return None
        _idx, tri, w = hit
        verts = [l.vert for l in tri]
        weights = None
        if self.deform:
            weights = {}
            total = 0.0
            for wi, v in zip(w, verts):
                if wi <= 0.0:
                    continue
                total += wi
                for g, x in v[self.deform].items():
                    weights[g] = weights.get(g, 0.0) + wi * x
            if total > 0.0:
                weights = {g: x / total for g, x in weights.items() if x / total > 1e-6}
        shapes = [sum(((v[s] - v.co) * wi for v, wi in zip(verts, w)), Vector())
                  for s in self.shape]
        cols = [sum((Vector(v[c]) * wi for v, wi in zip(verts, w)), Vector((0, 0, 0, 0)))
                for c in self.vcol]
        return weights, shapes, cols

    def sample_face(self, center, corners):
        if not self.has_loop_data:
            return None
        hit = self._locate(center, weights=False)
        if hit is None:
            return None
        idx, tri, _w = hit
        fr = self._frame(idx)
        tri_uv = [(tri[0][x].uv.copy(), tri[1][x].uv.copy(), tri[2][x].uv.copy())
                  for x in self.uv]
        tri_col = [(Vector(tri[0][x]), Vector(tri[1][x]), Vector(tri[2][x]))
                   for x in self.lcol]
        weights = self._weights
        out = []
        for p in corners:
            w0, w1, w2 = weights(fr, p)
            uvs = [a * w0 + b * w1 + c * w2 for a, b, c in tri_uv]
            cols = [a * w0 + b * w1 + c * w2 for a, b, c in tri_col]
            out.append((uvs, cols))
        return out

    # -------------------------------------------------------------- writing

    def write_vertex(self, v, data):
        if data is None:
            return
        weights, shapes, cols = data
        if weights is not None:
            dv = v[self.deform]
            dv.clear()
            for g, x in weights.items():
                dv[g] = x
        co = v.co
        for layer, off in zip(self.shape, shapes):
            v[layer] = co + off
        for layer, col in zip(self.vcol, cols):
            v[layer] = col

    def write_face(self, f, data):
        if data is None:
            return
        for loop, (uvs, cols) in zip(f.loops, data):
            for layer, uv in zip(self.uv, uvs):
                loop[layer].uv = uv
            for layer, col in zip(self.lcol, cols):
                loop[layer] = col
