![image alt](https://github.com/runtimerobotics/urdf_viewer/blob/main/6.jpg?raw=true)
![image_alt](https://github.com/runtimerobotics/urdf_viewer/blob/main/2.jpg?raw=true)



# URDF Viewer

**URDF Viewer** is a lightweight desktop application built with **Python**, **PyQt5**, and **OpenGL** for visualizing URDF (Unified Robot Description Format) models. It provides an interactive 3D interface to view robots, explore their structure, and control joint angles.

---

## ✨ Features

- 📂 Load and display any URDF robot model.
- 🦾 Visualize links and joints (supports `.stl`, `.dae`, `.obj`).
- 🎮 Interactive 3D view (zoom, pan, rotate).
- 🎚️ Auto-generated joint sliders to manipulate robot poses.
- 🌈 Material and color rendering support.
- 🧠 Works offline, no ROS or Gazebo required.

---

## 📁 Project Structure

```
urdf_viewer/
├── main.py                      # Entry point
├── core/
│   ├── mesh_viewer.py           # OpenGL 3D viewer logic
│   └── urdf_loader.py           # URDF parser using urdfpy
├── assets/
│   └── icons/                   # UI icons
├── ui/                          # (optional) for UI files
├── .vscode/                     # VSCode settings
├── build/                       # Temporary files (ignored)
├── *.urdf                      # Sample URDF robot files
├── requirements.txt             # Python dependencies
├── README.md                    # You're reading it
└── .gitignore                   # Git ignored files
```

---

## 🚀 Getting Started

### 🔧 Installation

```bash
git clone https://github.com/runtimerobotics/urdf_viewer.git
cd urdf_viewer
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### ▶️ Run the App

```bash
python main.py
```

---

## 🧪 Sample URDF Models Included

- `two_wheel_robot.urdf`
- `four_wheel_robot.urdf`
- `arm_robot.urdf`
- `drive_robot.urdf`
- `diff_drive_with_arm.urdf`
- `test_robot.urdf`

You can load your own URDF models as well!

---

## 📦 Dependencies

- `pyqt5`
- `pyqtgraph`
- `urdfpy`
- `numpy`

Install them via:

```bash
pip install -r requirements.txt
```

---

## 🙌 Credits

Developed by [Runtime Robotics](https://github.com/runtimerobotics)

---

## 📄 License

This project is licensed under the [MIT License](LICENSE) - feel free to use and modify.
