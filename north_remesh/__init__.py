# SPDX-License-Identifier: GPL-3.0-or-later
#
# North Remesh - rebuild faces as evenly sized triangles or quads.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.

# Keep in sync with blender_manifest.toml (build.py checks this).
bl_info = {
    "name": "North Remesh",
    "author": "Alexandru Iftimi",
    "version": (1, 1, 0),
    "blender": (4, 2, 0),
    "location": "View3D > Edit Mode > Right-click / Face menu  |  Object Mode > Right-click",
    "description": "Rebuild selected faces as evenly sized triangles or quads, "
                   "matched to the surrounding mesh density",
    "doc_url": "https://github.com/alexandruiftimi-ns/NorthRemesh",
    "tracker_url": "https://github.com/alexandruiftimi-ns/NorthRemesh/issues",
    "category": "Mesh",
}

if "operators" in locals():
    import importlib

    from . import engine, attributes, region, operators
    importlib.reload(engine)
    importlib.reload(attributes)
    importlib.reload(region)
    importlib.reload(operators)
else:
    from . import operators


def register():
    operators.register()


def unregister():
    operators.unregister()
