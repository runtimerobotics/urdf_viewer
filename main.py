import sys
import os
import numpy as np
import traceback

# Compatibility fix for NumPy >= 1.24
if not hasattr(np, 'float'):
    np.float = float

# Force desktop OpenGL for Qt5 on Ubuntu 24.04
os.environ['QT_OPENGL'] = 'desktop'

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QVBoxLayout,
    QWidget, QAction, QSlider, QLabel, QHBoxLayout, QGroupBox, QScrollArea
)
from PyQt5.QtCore import Qt
import pyqtgraph.opengl as gl
from pyqtgraph import Vector

from core import urdf_loader
from core import mesh_viewer
from urdfpy import URDF
from scipy.spatial.transform import Rotation as R


class URDFViewer(QMainWindow):
    def __init__(self, urdf_path=None):
        super().__init__()
        self.setWindowTitle("URDF Viewer")
        self.resize(1200, 700)

        self.gl_widget = gl.GLViewWidget()
        self.gl_widget.setCameraPosition(distance=2)
        self.gl_widget.addItem(gl.GLGridItem())

        self.mesh_items = []
        self.mesh_map = {}  # UPDATED: link name to list of mesh items
        self.robot = None
        self.joint_slider_map = {}
        self.current_file_path = ""

        self.joint_slider_box = QGroupBox("Joint Controls")
        self.joint_slider_layout = QVBoxLayout()
        self.joint_slider_box.setLayout(self.joint_slider_layout)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(self.joint_slider_box)
        scroll.setMinimumWidth(250)

        main_layout = QHBoxLayout()
        main_layout.addWidget(self.gl_widget, stretch=4)
        main_layout.addWidget(scroll, stretch=1)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        self.setup_menus()

        # Load URDF if path is provided
        if urdf_path:
            self.load_urdf(urdf_path)

    def setup_menus(self):
        menubar = self.menuBar()
        file_menu = menubar.addMenu("File")
        edit_menu = menubar.addMenu("Edit")
        help_menu = menubar.addMenu("Help")

        open_action = QAction("Open URDF", self)
        open_action.triggered.connect(self.open_file)
        file_menu.addAction(open_action)

        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

    def open_file(self):
        file_path, _ = QFileDialog.getOpenFileName(self, "Open URDF File", "", "URDF Files (*.urdf)")
        if file_path and file_path != self.current_file_path:
            self.load_urdf(file_path)

    def load_urdf(self, file_path):
        try:
            robot = urdf_loader.load_urdf(file_path)
            if robot:
                print(f"Loaded robot: {robot.name}")
                self.setWindowTitle(f"URDF Viewer - {robot.name} ({os.path.basename(file_path)})")
                self.current_file_path = file_path
                self.robot = robot
                self.robot.update_cfg(np.zeros(len(self.robot.actuated_joints)))
                self.create_joint_sliders(robot)
                self.render_robot(robot)
                self.reset_view()
        except Exception as e:
            print(f"Failed to load URDF: {e}")
            traceback.print_exc()

    def create_joint_sliders(self, robot):
        for i in reversed(range(self.joint_slider_layout.count())):
            widget = self.joint_slider_layout.itemAt(i).widget()
            if widget:
                widget.setParent(None)
        self.joint_slider_map.clear()

        for joint in robot.joints:
            if joint.joint_type in ("revolute", "prismatic") and joint.limit:
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(int(joint.limit.lower * 100))
                slider.setMaximum(int(joint.limit.upper * 100))
                slider.setValue(0)
                slider.setSingleStep(1)
                slider.setToolTip(f"{joint.name}: {joint.limit.lower:.2f} to {joint.limit.upper:.2f}")
                slider.valueChanged.connect(self.update_joint)

                label = QLabel(f"{joint.name} ({joint.joint_type})")
                layout = QVBoxLayout()
                layout.addWidget(label)
                layout.addWidget(slider)

                joint_container = QWidget()
                joint_container.setLayout(layout)

                self.joint_slider_layout.addWidget(joint_container)
                self.joint_slider_map[joint.name] = slider

            elif joint.joint_type == "continuous":
                slider = QSlider(Qt.Horizontal)
                slider.setMinimum(-36000)
                slider.setMaximum(36000)
                slider.setValue(0)
                slider.setSingleStep(1)
                slider.setToolTip(f"{joint.name}: -360 to 360 degrees (continuous)")
                slider.valueChanged.connect(self.update_joint)

                label = QLabel(f"{joint.name} ({joint.joint_type})")
                layout = QVBoxLayout()
                layout.addWidget(label)
                layout.addWidget(slider)

                joint_container = QWidget()
                joint_container.setLayout(layout)

                self.joint_slider_layout.addWidget(joint_container)
                self.joint_slider_map[joint.name] = slider

    def update_joint(self):
        cfg = []
        for joint in self.robot.actuated_joints:
            slider = self.joint_slider_map.get(joint.name)
            if slider:
                val = slider.value() / 100.0
                cfg.append(val)
        self.robot.update_cfg(cfg)

        link_to_tf = self.robot.link_tf_tree()
        for link_name, mesh_list in self.mesh_map.items():
            link = next((l for l in self.robot.links if l.name == link_name), None)
            if not link:
                continue
            for i, visual in enumerate(link.visuals):
                try:
                    full_tf = link_to_tf[link] @ visual.origin
                    self.set_mesh_transform(mesh_list[i], full_tf)
                except Exception as e:
                    print(f"Update transform error for link '{link_name}': {e}")
                    traceback.print_exc()

    def set_mesh_transform(self, mesh, tf):
        mesh.resetTransform()
        rot = tf[:3, :3]
        pos = tf[:3, 3]

        r = R.from_matrix(rot)
        angle_rad = np.linalg.norm(r.as_rotvec())
        angle_deg = np.degrees(angle_rad)
        axis = r.as_rotvec()
        if np.linalg.norm(axis) > 1e-6:
            axis = axis / np.linalg.norm(axis)
            mesh.rotate(angle_deg, axis[0], axis[1], axis[2])

        mesh.translate(*pos)

    def render_robot(self, robot):
        for item in self.mesh_items:
            self.gl_widget.removeItem(item)
        self.mesh_items.clear()
        self.mesh_map.clear()

        link_to_tf = robot.link_tf_tree()

        for link in robot.links:
            tf = link_to_tf[link]
            mesh_list = []
            for visual in link.visuals:
                try:
                    mesh = mesh_viewer.create_mesh_item(visual)
                    if mesh:
                        full_tf = tf @ visual.origin
                        self.set_mesh_transform(mesh, full_tf)
                        self.gl_widget.addItem(mesh)
                        self.mesh_items.append(mesh)
                        mesh_list.append(mesh)
                except Exception as e:
                    print(f"Render error for link '{link.name}': {e}")
                    traceback.print_exc()
            if mesh_list:
                self.mesh_map[link.name] = mesh_list  # Store list of meshes per link

    def reset_view(self):
        self.gl_widget.setCameraPosition(distance=2)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = URDFViewer()
    viewer.show()
    sys.exit(app.exec_())
