"""Tiện ích đọc cache .brep và trích xuất hình học (mặt trụ, hộp bao) cho phân tích Roboter.STEP."""
import glob, os
import numpy as np
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.TopAbs import TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Shape
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

CACHE = os.path.join(os.path.dirname(__file__), "cad_cache")


def load(pattern):
    files = sorted(glob.glob(os.path.join(CACHE, pattern + ".brep")))
    assert files, pattern
    shape = TopoDS_Shape()
    BRepTools.Read_s(shape, files[0], BRep_Builder())
    return shape


def bbox(shape):
    b = Bnd_Box(); BRepBndLib.Add_s(shape, b)
    lo, hi = b.CornerMin(), b.CornerMax()
    return np.array([lo.X(), lo.Y(), lo.Z()]), np.array([hi.X(), hi.Y(), hi.Z()])


def faces(shape):
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        yield TopoDS.Face(exp.Current())
        exp.Next()


def face_area(face):
    p = GProp_GProps(); BRepGProp.SurfaceProperties_s(face, p)
    return p.Mass()


def cylinders(shape):
    """[(bán kính, điểm trên trục, hướng trục, diện tích)] của mọi mặt trụ."""
    out = []
    for f in faces(shape):
        s = BRepAdaptor_Surface(f)
        if s.GetType() == GeomAbs_Cylinder:
            c = s.Cylinder(); ax = c.Axis()
            p, d = ax.Location(), ax.Direction()
            out.append((c.Radius(), np.array([p.X(), p.Y(), p.Z()]), np.array([d.X(), d.Y(), d.Z()]), face_area(f)))
    return out


def planes(shape):
    out = []
    for f in faces(shape):
        s = BRepAdaptor_Surface(f)
        if s.GetType() == GeomAbs_Plane:
            pl = s.Plane(); ax = pl.Axis()
            p, d = ax.Location(), ax.Direction()
            out.append((np.array([p.X(), p.Y(), p.Z()]), np.array([d.X(), d.Y(), d.Z()]), face_area(f)))
    return out


def local_frame(origin, axis):
    """Ma trận 3x3 (hàng = trục u, v, w) với w = axis."""
    w = axis / np.linalg.norm(axis)
    a = np.array([1.0, 0, 0]) if abs(w[0]) < 0.9 else np.array([0, 1.0, 0])
    u = np.cross(w, a); u /= np.linalg.norm(u); v = np.cross(w, u)
    return np.array([u, v, w]), np.asarray(origin, float)
