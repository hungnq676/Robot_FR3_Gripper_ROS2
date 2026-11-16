#!/usr/bin/env python3
"""3D renders of the real gripper for the report, assembled exactly as in the robot model
(fairino_fr3_model.xacro, hardware_type:=fairino): FR3 wrist (wrist3_link), tool flange,
main frame and the two jaws (prismatic, mimic), drawn open (88 mm) and closed (~30 mm),
plus an exploded view. VTK, off-screen."""
import os

import numpy as np
import vtk

D = os.path.expanduser("~/fr3_hungros2/ws_fr3_moveit/src/fairino_fr3_description/meshes/visual")
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hinh")


def T(xyz, rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    M = np.eye(4)
    M[:3, :3] = Rz @ Ry @ Rx
    M[:3, 3] = xyz
    return M


# Joint origins from the URDF generated with hardware_type:=fairino (frame: base_link_gripper, m)
T_adapter = T([-0.0375, 0.077, 0.175], [0, 0, -1.5708])          # wrist3_link -> base_link_gripper
T_c1 = T([0.022, 0, 0.01], [1.5708, 0, 1.5708])                    # joint_clamp_1, axis -z
T_c2 = T([0.132, 0.047, 0.01], [1.5708, np.pi, -1.5708])            # joint_clamp_2, axis +z, mimic -1
WHITE, BLACK, GREY, METAL = (0.95, 0.95, 0.93), (0.20, 0.20, 0.22), (0.86, 0.86, 0.88), (0.62, 0.63, 0.67)


def actor(path, scale, M, rgb):
    rd = vtk.vtkSTLReader()
    rd.SetFileName(path)
    tf = vtk.vtkTransform()
    tf.SetMatrix(list((M @ np.diag([scale, scale, scale, 1])).ravel()))
    f = vtk.vtkTransformPolyDataFilter()
    f.SetInputConnection(rd.GetOutputPort())
    f.SetTransform(tf)
    n = vtk.vtkPolyDataNormals()
    n.SetInputConnection(f.GetOutputPort())
    n.SetFeatureAngle(35)
    mp = vtk.vtkPolyDataMapper()
    mp.SetInputConnection(n.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(mp)
    pr = a.GetProperty()
    pr.SetColor(*rgb)
    pr.SetAmbient(0.25)
    pr.SetDiffuse(0.75)
    pr.SetSpecular(0.25)
    pr.SetSpecularPower(30)
    return a


def box(center, size, rgb=(0.45, 0.65, 0.95), opacity=0.28):
    c = vtk.vtkCubeSource()
    c.SetCenter(*center)
    c.SetXLength(size[0]); c.SetYLength(size[1]); c.SetZLength(size[2])
    mp = vtk.vtkPolyDataMapper()
    mp.SetInputConnection(c.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(mp)
    a.GetProperty().SetColor(*rgb)
    a.GetProperty().SetOpacity(opacity)
    a.GetProperty().EdgeVisibilityOn()
    a.GetProperty().SetEdgeColor(0.2, 0.35, 0.7)
    return a


def cylinder(center, radius, height, rgb=(0.45, 0.65, 0.95), opacity=0.32):
    """Illustrative cylinder along z (vtkCylinderSource is along y)."""
    c = vtk.vtkCylinderSource()
    c.SetRadius(radius); c.SetHeight(height); c.SetResolution(96)
    tf = vtk.vtkTransform()
    tf.Translate(*center)
    tf.RotateX(90)
    f = vtk.vtkTransformPolyDataFilter()
    f.SetInputConnection(c.GetOutputPort())
    f.SetTransform(tf)
    mp = vtk.vtkPolyDataMapper()
    mp.SetInputConnection(f.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(mp)
    a.GetProperty().SetColor(*rgb)
    a.GetProperty().SetOpacity(opacity)
    return a


def parts(q1, wrist=True, explode=0.0):
    """q1 = joint_clamp_1 (0 = open 88 mm .. -0.029 = closed to ~30 mm). explode: spread along z, m."""
    e = lambda dz: T([0, 0, dz], [0, 0, 0])
    out = [actor(f"{D}/gripper/main-frame.stl", 1e-3, np.eye(4), WHITE),
           actor(f"{D}/gripper/fr3_tool_flange.stl", 1e-3, e(-1.5 * explode) @ T([0.077, 0.0375, -0.069], [0, 0, 0]), METAL),
           actor(f"{D}/gripper/clamp.stl", 1e-3, e(explode) @ T_c1 @ T([0, 0, -q1], [0, 0, 0]), BLACK),
           actor(f"{D}/gripper/clamp.stl", 1e-3, e(explode) @ T_c2 @ T([0, 0, -q1], [0, 0, 0]) @ T([0, 0, 0], [0, 0, np.pi]), BLACK)]
    # servo + BusLinker + ESP32 between flange and frame: no CAD. Illustration only: a cylinder as wide
    # as the tool flange (63 mm), as tall as the collision envelope (47 mm), on the flange axis.
    out.append(cylinder([0.077, 0.0375, -0.0235 - 0.5 * explode], 0.0315, 0.047))
    if wrist:
        out.append(actor(f"{D}/arm/wrist3_link.STL", 1.0, e(-3 * explode) @ np.linalg.inv(T_adapter), GREY))
    return out


def render(actors, eye, center, up, name, size=(1600, 1200), labels=()):
    ren = vtk.vtkRenderer()
    ren.SetBackground(1, 1, 1)
    for a in actors:
        ren.AddActor(a)
    for text, pos in labels:
        t = vtk.vtkBillboardTextActor3D()
        t.SetInput(text)
        t.SetPosition(*pos)
        tp = t.GetTextProperty()
        tp.SetFontSize(34)
        tp.SetColor(0.1, 0.1, 0.1)
        tp.SetFontFamilyToArial()
        tp.SetBackgroundColor(1, 1, 1)
        tp.SetBackgroundOpacity(0.8)
        ren.AddActor(t)
    cam = ren.GetActiveCamera()
    cam.SetPosition(*eye)
    cam.SetFocalPoint(*center)
    cam.SetViewUp(*up)
    cam.SetViewAngle(30)
    ren.ResetCamera()
    cam.Zoom(1.25)
    ren.ResetCameraClippingRange()
    lk = vtk.vtkLightKit()
    lk.AddLightsToRenderer(ren)
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.SetMultiSamples(8)
    win.AddRenderer(ren)
    win.SetSize(*size)
    win.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(win)
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(os.path.join(OUT, name))
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
    print("->", name)


if __name__ == "__main__":
    c = [0.077, 0.0375, 0.02]
    up = [0, 0, -1]            # flange at the top of the picture, jaws pointing down
    eye = [0.40, -0.33, -0.05]
    render(parts(0.0), eye, c, up, "gripper3d_mo.png")
    render(parts(-0.029), eye, c, up, "gripper3d_khep.png")
    render(parts(0.0, wrist=False), [0.077, -0.38, 0.02], c, up, "gripper3d_truoc.png")
    render(parts(0.0, explode=0.05), [0.077, -0.45, -0.05], [0.077, 0.0375, -0.04], up, "gripper3d_tach.png",
           size=(1200, 1600))
