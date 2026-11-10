# North Remesh

A Blender add-on that rebuilds faces as evenly sized triangles or quads.
Built for 3D-printing work on heavy meshes (hundreds of thousands of faces).

Typical use: cut part of a model, fill the hole with a flat face, select that
face and run **North Remesh**. The new faces match the density of the mesh
around them, blend from dense to coarse neighbours, and connect watertight to
the existing border.

## Install

Blender 4.2 or newer.

1. Get `north_remesh-<version>.zip` by running `python build.py`, which
   writes it to `dist/`.
2. In Blender: *Edit > Preferences > Get Extensions > ⌄ (top right) >
   Install from Disk…* and pick the zip.

## Use

- **Edit Mode:** select faces, then right-click > **North Remesh** (also in
  the *Face* menu). Only the selection is rebuilt.
- **Object Mode:** right-click > **North Remesh** rebuilds the whole mesh.
- Progress shows in the status bar. **Esc** cancels and leaves the mesh
  exactly as it was.
- Adjust settings afterwards in the *Adjust Last Operation* panel (F9). Note
  that changes made there re-run without progress or Esc, so on very large
  selections it's better to undo and run again.

## Options

| Option | Meaning |
|---|---|
| Output | Triangles or Quads |
| Quad Method | *Pair Triangles*: regular quads, some triangles remain. *All Quads*: every triangle becomes 3 quads |
| Size: Match Surroundings | Measures the faces around the selection. *Scale* below 1 = denser |
| Blend Into Surroundings | Grades the size from the border (dense next to dense, coarse next to coarse) |
| Size: Relative | Edge length as % of the selection's size |
| Size: Absolute | Fixed edge length in world units |
| Iterations | Improvement passes (5 is plenty) |
| Smoothing | How strongly the vertices are evened out |
| Stick To Surface | Keeps vertices on the original surface (skipped automatically on flat faces) |
| Transfer Attributes | New geometry gets UVs, vertex group weights, shape keys and colours from the original surface |
| Keep Creases Above | Edges sharper than this angle are preserved (vertices may slide along them) |
| Keep Marked Sharp / Material Borders | Preserve those edges |
| Allow Border Splits | Off: the border with the rest of the mesh never changes. On: border edges are subdivided too and neighbouring faces get matching vertices |
| Face Limit | Hard cap on the number of faces created. The size is raised automatically to stay under it |

The finish message reports the face count, the share of quads, the smallest
angle and the share of poor-quality faces.

## Safety

- The selection is copied into a private working mesh, in world space. Your
  mesh is not touched until the result has passed validation.
- A result is refused (and nothing changes) if it would add non-manifold
  edges or vertices, loose geometry, degenerate faces, holes (a change in
  V − E + F), or a changed border.
- After writing, the result is verified again. If anything is wrong, the mesh
  is restored from a snapshot.
- Overlapping vertices on the selection border are detected and refused
  (run *Merge by Distance* first).
- Every loop has a fixed upper bound, and the face count is capped, so a size
  that is too small can't run away.

## Development

```
blender -b --factory-startup --python tests/test_north_remesh.py   # run tests
python build.py                                                     # build dist/north_remesh-<version>.zip
```

The version lives in `north_remesh/blender_manifest.toml`; `build.py` checks
that `bl_info` in `north_remesh/__init__.py` matches. See
[CHANGELOG.md](CHANGELOG.md).

## License

GPL-3.0-or-later. See [LICENSE](LICENSE).
