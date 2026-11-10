"""
Build the installable North Remesh package.

    python build.py [path/to/blender]

Reads the version from north_remesh/blender_manifest.toml, checks that
bl_info in __init__.py matches, and writes dist/north_remesh-<version>.zip.

If a Blender executable is available it uses Blender's own extension builder
(which also validates the manifest); otherwise it zips the folder directly.
"""

import ast
import os
import re
import shutil
import subprocess
import sys
import zipfile

ROOT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(ROOT, "north_remesh")
DIST = os.path.join(ROOT, "dist")
DEFAULT_BLENDER = r"C:\Program Files\Blender Foundation\Blender 5.2\blender.exe"
EXCLUDE_DIRS = {"__pycache__"}
EXCLUDE_EXT = {".pyc", ".zip"}


def manifest_version():
    with open(os.path.join(SRC, "blender_manifest.toml"), encoding="utf-8") as fh:
        m = re.search(r'^version\s*=\s*"(\d+\.\d+\.\d+)"', fh.read(), re.M)
    if not m:
        sys.exit("No version in blender_manifest.toml")
    return m.group(1)


def bl_info_version():
    with open(os.path.join(SRC, "__init__.py"), encoding="utf-8") as fh:
        tree = ast.parse(fh.read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, "id", None) == "bl_info" for t in node.targets):
            info = ast.literal_eval(node.value)
            return ".".join(str(x) for x in info["version"])
    sys.exit("No bl_info in __init__.py")


def zip_direct(out):
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for base, dirs, files in os.walk(SRC):
            dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
            for name in sorted(files):
                if os.path.splitext(name)[1] in EXCLUDE_EXT:
                    continue
                full = os.path.join(base, name)
                z.write(full, os.path.relpath(full, SRC))


def main():
    version = manifest_version()
    if bl_info_version() != version:
        sys.exit("Version mismatch: manifest {} vs bl_info {}".format(version, bl_info_version()))
    shutil.rmtree(os.path.join(SRC, "__pycache__"), ignore_errors=True)
    os.makedirs(DIST, exist_ok=True)
    out = os.path.join(DIST, "north_remesh-{}.zip".format(version))
    if os.path.exists(out):
        os.remove(out)

    blender = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_BLENDER
    if os.path.isfile(blender):
        cmd = [blender, "--factory-startup", "--command", "extension", "build",
               "--source-dir", SRC, "--output-filepath", out]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode != 0 or not os.path.exists(out):
            print(res.stdout, res.stderr)
            sys.exit("Blender extension build failed")
        subprocess.run([blender, "--factory-startup", "--command", "extension",
                        "validate", out], check=True)
    else:
        print("Blender not found, zipping directly")
        zip_direct(out)
    print("Built", out)


if __name__ == "__main__":
    main()
