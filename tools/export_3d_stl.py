"""Export the FreeCAD models in 3D/ to coarse STL files.

Run this with FreeCAD's headless runner (not plain Python):

    freecadcmd tools/export_3d_stl.py

Output: build/stl/*.stl (used by tools/render_3d_preview.py).
"""

import os

import FreeCAD
import Mesh
import MeshPart

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(ROOT, "3D", "20261028")
OUT = os.path.join(ROOT, "build", "stl")
os.makedirs(OUT, exist_ok=True)

FILES = [
    "01-IMU.FCStd",
    "04-IMU-box-A.FCStd",
    "04-IMU-box-B.FCStd",
    "open_imu_3D_1022B.FCStd",
]

for fn in FILES:
    doc = FreeCAD.openDocument(os.path.join(BASE, fn))
    shapes = [o for o in doc.Objects if hasattr(o, "Shape") and getattr(o.Shape, "Faces", None)]
    combined = Mesh.Mesh()
    for o in shapes:
        combined.addMesh(MeshPart.meshFromShape(
            Shape=o.Shape, LinearDeflection=1.5, AngularDeflection=0.5, Relative=False))
    out = os.path.join(OUT, fn.replace(".FCStd", ".stl"))
    combined.write(out)
    print("%-26s -> %6d facets  %s" % (fn, combined.CountFacets, out), flush=True)
    FreeCAD.closeDocument(doc.Name)
