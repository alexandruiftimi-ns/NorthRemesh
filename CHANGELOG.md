# Changelog

All notable changes to North Remesh. The project uses
[Semantic Versioning](https://semver.org/): MAJOR.MINOR.PATCH.

## [1.1.0] - 2026-11-10

### Added
- **Blend Into Surroundings**: target size is graded from the border, so a
  fill is denser next to dense neighbours and coarser next to coarse ones.
- **All Quads** method: splits every triangle into 3 quads (quads everywhere
  except along a locked border). The existing pairing method is kept as
  **Pair Triangles** and now uses topology-aware pairing where Blender
  supports it.
- **Transfer Attributes**: rebuilt geometry gets UV maps, vertex group
  weights, shape keys and colour attributes sampled from the original
  surface.
- Crease vertices now slide along their crease, evening out spacing there.
- Finish report shows the share of quads, the smallest angle and the share
  of poor-quality faces.
- Test suite (`tests/`) and build script (`build.py`).

### Changed
- **Face Limit** (was *Max Faces*) is now a hard limit: the size is raised
  automatically to stay under it, the final count is checked, and the run
  retries with a larger size if needed. The report says when this happened.
- Processing happens in world space, so non-uniform object scale no longer
  stretches the result. *Absolute* size is now always in world units.
- Smoothing is area-weighted, giving more even face sizes.
- Long steps now yield to the UI often, so progress and Esc respond quickly
  even on very large selections.
- Validation checks the exact elements instead of comparing counts.

### Fixed
- New vertices lost vertex groups and shape keys, and new faces had blank
  UVs.
- With *Allow Border Splits*, a failure while writing the result could leave
  the mesh half-changed. The result is now verified after writing, and the
  mesh is restored from a snapshot on any failure.
- A face that failed to be created was skipped silently (possible hole); it
  now aborts and restores.
- Overlapping (doubled) vertices on the selection border could be swapped,
  creating non-manifold geometry; this is now detected and refused safely.

## [1.0.0] - 2026-11-10

First release: isotropic triangle/quad remeshing of selected faces with
Match Surroundings sizing, one-step fill of large flat faces, border-locked
or border-splitting modes, modal progress with Esc, and validation that
leaves the mesh untouched if the result would be non-manifold.
