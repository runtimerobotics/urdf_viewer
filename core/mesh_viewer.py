import sys
import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtWidgets
from pyqtgraph import Vector
from PyQt5.QtGui import QMatrix4x4

# Your create_mesh_item function here (add origin transform support)
def create_mesh_item(visual):
    mesh = None
    color = (0.7, 0.7, 0.7, 1.0)

    try:
        if visual.material and hasattr(visual.material, "color") and visual.material.color is not None:
            color = tuple(visual.material.color)

        geometry = visual.geometry

        if geometry.mesh:
            # Assume mesh_obj.to_trimesh() available
            trimesh_obj = geometry.mesh.to_trimesh()
            vertices = trimesh_obj.vertices
            faces = trimesh_obj.faces
            md = gl.MeshData(vertexes=vertices, faces=faces)
            mesh = gl.GLMeshItem(meshdata=md, smooth=True, color=color, shader="shaded")

        elif geometry.box:
            size = geometry.box.size
            mesh = gl.GLBoxItem(size=Vector(*size))
            mesh.setColor(color)

        elif geometry.sphere:
            r = geometry.sphere.radius
            md = gl.MeshData.sphere(rows=20, cols=20, radius=r)
            mesh = gl.GLMeshItem(meshdata=md, smooth=True, color=color, shader="shaded")

        elif geometry.cylinder:
            r = geometry.cylinder.radius
            h = geometry.cylinder.length
            md = gl.MeshData.cylinder(rows=20, cols=20, radius=[r, r], length=h)
            mesh = gl.GLMeshItem(meshdata=md, smooth=True, color=color, shader="shaded")

        # Apply origin transform if origin present
        if hasattr(visual, 'origin') and visual.origin is not None and mesh is not None:
            mat = QMatrix4x4()
            if hasattr(visual.origin, 'xyz') and hasattr(visual.origin, 'rpy'):
                mat.translate(*visual.origin.xyz)
                roll, pitch, yaw = visual.origin.rpy
                mat.rotate(np.degrees(roll), 1, 0, 0)
                mat.rotate(np.degrees(pitch), 0, 1, 0)
                mat.rotate(np.degrees(yaw), 0, 0, 1)
            elif isinstance(visual.origin, np.ndarray) and visual.origin.shape == (4, 4):
                mat = QMatrix4x4(*visual.origin.T.flatten())
            mesh.setTransform(mat)

        return mesh

    except Exception as e:
        print(f"[mesh_viewer] Error creating mesh: {e}")
        return None

# Minimal dummy data classes for testing without full URDF parser:
class Origin:
    def __init__(self, xyz=(0,0,0), rpy=(0,0,0)):
        self.xyz = xyz
        self.rpy = rpy
class Material:
    def __init__(self, color=(0.7,0.7,0.7,1)):
        self.color = color
class Geometry:
    def __init__(self, box=None, cylinder=None):
        self.box = box
        self.cylinder = cylinder
class Box:
    def __init__(self, size):
        self.size = size
class Cylinder:
    def __init__(self, radius, length):
        self.radius = radius
        self.length = length
class Visual:
    def __init__(self, geometry, material=None, origin=None):
        self.geometry = geometry
        self.material = material
        self.origin = origin or Origin()

def main():
    app = QtWidgets.QApplication([])

    view = gl.GLViewWidget()
    view.opts['distance'] = 2
    view.show()
    view.setWindowTitle('Diff Drive Robot Demo with Arm')

    grid = gl.GLGridItem()
    grid.scale(0.2, 0.2, 0.2)
    view.addItem(grid)

    # Base link visual
    base_visual = Visual(
        geometry=Geometry(box=Box((0.4, 0.3, 0.1))),
        material=Material((0.7,0.7,0.7,1.0)),
        origin=Origin((0,0,0), (0,0,0))
    )

    # Left wheel
    left_wheel_visual = Visual(
        geometry=Geometry(cylinder=Cylinder(0.05, 0.02)),
        material=Material((0,0.5,1,1)),
        origin=Origin((0,0,0), (1.5708,0,0))
    )

    # Right wheel
    right_wheel_visual = Visual(
        geometry=Geometry(cylinder=Cylinder(0.05, 0.02)),
        material=Material((0,0.5,1,1)),
        origin=Origin((0,0,0), (1.5708,0,0))
    )

    # Shoulder (arm_link_1)
    shoulder_visual = Visual(
        geometry=Geometry(box=Box((0.05, 0.05, 0.3))),
        material=Material((1.0, 0.0, 0.0, 1.0)),
        origin=Origin((0.0, 0.0, 0.1), (0, 0, 0))
    )

    # Elbow (arm_link_2)
    elbow_visual = Visual(
        geometry=Geometry(box=Box((0.04, 0.04, 0.2))),
        material=Material((0.0, 1.0, 0.0, 1.0)),
        origin=Origin((0.0, 0.0, 0.3), (0, 0, 0))
    )

    base_mesh = create_mesh_item(base_visual)
    left_mesh = create_mesh_item(left_wheel_visual)
    right_mesh = create_mesh_item(right_wheel_visual)
    shoulder_mesh = create_mesh_item(shoulder_visual)
    elbow_mesh = create_mesh_item(elbow_visual)

    if base_mesh:
        view.addItem(base_mesh)
    if left_mesh:
        left_mesh.translate(0, 0.15, -0.05)
        view.addItem(left_mesh)
    if right_mesh:
        right_mesh.translate(0, -0.15, -0.05)
        view.addItem(right_mesh)
    if shoulder_mesh:
        view.addItem(shoulder_mesh)
    if elbow_mesh:
        view.addItem(elbow_mesh)

    app.exec()

if __name__ == '__main__':
    main()
