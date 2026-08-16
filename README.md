# Robotics Library for Everyone (RoLE)

<p align="center">
  <img src=https://github.com/rparak/RoLE/blob/main/images/RoLE.png width="700" height="300">
</p>

A readable, dependency-light Python library for the kinematics, collision detection, trajectory generation, workspace analysis and simulation of serial manipulators and linear mechanisms.

![Python](https://img.shields.io/badge/Python-3776AB?style=flat&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Blender](https://img.shields.io/badge/Blender-supported-orange.svg)
![PyBullet](https://img.shields.io/badge/PyBullet-supported-blue.svg)

## Table of Contents

- [Project Description](#project-description)
- [Key Features](#key-features)
- [Supported Robots and Mechanisms](#supported-robots-and-mechanisms)
- [Project Hierarchy](#project-hierarchy)
- [Requirements](#requirements)
- [Installation Dependencies](#installation-dependencies)
- [Getting Started / Usage](#getting-started--usage)
- [Evaluation / Examples Overview](#evaluation--examples-overview)
- [Gallery](#gallery)
- [Contact Info](#contact-info)
- [Citation (BibTex)](#citation-bibtex)
- [License](#license)

## Project Description

Robotics Library for Everyone (RoLE) is a pure-Python robotics library for the modelling, analysis and
simulation of serial robotic manipulators and linear mechanisms. It is written to be read: the core
mathematics depends only on [NumPy](https://numpy.org), each robot is described by a plain dataclass, and
the public functions carry explicit type hints. The goal is a code base that students, researchers and
engineers can follow line by line rather than treat as a black box.

The library covers the tasks that recur in manipulator work. It computes forward kinematics from
Denavit–Hartenberg (DH) parameters and inverse kinematics both analytically (for the supported SCARA
structure) and with four numerical solvers. It builds the geometric Jacobian, checks for proximity to
singularities, and detects self-collisions and external collisions using oriented and axis-aligned bounding
boxes. It generates trajectories with trapezoidal and polynomial velocity profiles, interpolates paths with
Bézier and B-spline curves, samples the reachable workspace, and can emit a Unified Robotics Description
Format (URDF) file for a described robot.

RoLE is organised in three layers that all share the same parameter structures. The **core library**
(`src/RoLE`) is simulator-independent and contains all of the mathematics. The **Blender integration**
(`src/Blender`) drives `.blend` scenes for rendering and offline visualisation, and its scripts are executed
from inside Blender. The **PyBullet integration** (`src/PyBullet`) loads the generated URDF files into a
physics environment for interactive control and collision testing. Because a robot is defined once as a
`Robot_Parameters_Str` object, the same description feeds the core solvers, the Blender scene and the
PyBullet simulation without duplication.

The design philosophy is visible directly in the code: dataclass-based robot and mechanism descriptions
(`Robot_Parameters_Str`, `Mechanism_Parameters_Str`), NumPy-only core math with no hidden global state, and
small composable classes for transformations, primitives and colliders. Robot, class, folder and joint
names are case- and underscore-sensitive throughout (for example `Robot_Parameters_Str`, `ABB_IRB_14000_L`,
`Multi_Segment_Cls`).

## Key Features

**Forward kinematics** (`src/RoLE/Kinematics/Core.py`)
- Standard DH (`DH_Standard`) and Modified DH (`DH_Modified`) transformation building blocks.
- `Forward_Kinematics(theta, method, Robot_Parameters_Str)` with three methods: `'Standard'`, `'Modified'`
  and `'Fast'` (a fast closed-form solution used internally by the IK solvers).
- `Get_Individual_Joint_Configuration(...)` returns the homogeneous transformation matrix (HTM) of each joint.

**Inverse kinematics** (`src/RoLE/Kinematics/Core.py`)
- `Inverse_Kinematics_Analytical(...)` for the RRPR (SCARA) structure, returning `'All'` or the `'Best'`
  solution.
- `Inverse_Kinematics_Numerical(...)` with four methods selected by name: `'Jacobian-Transpose'`,
  `'Newton-Raphson'`, `'Gauss-Newton'` and `'Levenberg-Marquardt'`, driven by an
  `ik_solver_properties` dictionary (`delta_time`, `num_of_iteration`, `tolerance`).
- Solver output reports success, iteration count, position/orientation error, quadratic error, proximity to
  singularity and self-collision information.

**Jacobian and singularities** (`src/RoLE/Kinematics/Core.py`)
- `Get_Geometric_Jacobian(...)` builds the 6×n geometric Jacobian for revolute and prismatic joints.
- Singularity proximity is checked while solving (`General.Is_Close_Singularity`).

**Collision detection** (`src/RoLE/Collider/Core.py`, `src/RoLE/Primitives/Core.py`)
- Axis-aligned (`AABB_Cls`) and oriented (`OBB_Cls`) bounding-box colliders with `Overlap`,
  `Is_Point_Inside`, `Raycast` and `Transformation` methods.
- Geometric primitives `Point_Cls`, `Line_Segment_Cls` and `Box_Cls`.
- Self-collision detection between joint colliders and external-collision detection against added objects.

**Trajectory generation** (`src/RoLE/Trajectory/Core.py`, `src/RoLE/Trajectory/Utilities.py`)
- Velocity profiles: `Trapezoidal_Profile_Cls`, `Polynomial_Profile_Cls` (quintic) and `Linear_Function_Cls`.
- Multi-segment blended trajectories: `Multi_Segment_Cls`.

**Interpolation and simplification** (`src/RoLE/Interpolation/`)
- `Bezier_Cls` and `B_Spline_Cls` curve classes with arc length, bounding box, derivative and control-point
  optimisation utilities.
- `Lerp` / `Slerp`, Bernstein polynomials, knot-vector generation (`Generate_Knot_Vector`), and path
  simplification via `Simple_Simplification` and Ramer–Douglas–Peucker (`RDP_Simplification`).

**Workspace and URDF** (`src/RoLE/Workspace/Core.py`, `src/RoLE/URDF/Core.py`)
- Workspace sampling and XYZ generation (`Generate_Absolute_Joint_Orientation`, `Generate_Workspace_XYZ`).
- URDF file generation from a robot description (`URDF_Generator_Cls` with `Generate` and `Save`).

**Transformations and utilities** (`src/RoLE/Transformation/`, `src/RoLE/Utilities/`, `src/RoLE/Simplification/`)
- `Homogeneous_Transformation_Matrix_Cls`, `Vector3_Cls`, `Euler_Angle_Cls`, `Quaternion_Cls` with
  translation, rotation, inverse and distance helpers.
- File I/O (`Load`, `Save`), moment of inertia (`MOI.Cube`), and SymPy-based symbolic simplification of the
  forward kinematics and Jacobian.

**Simulation front-ends** (`src/Blender/`, `src/PyBullet/`)
- Blender `Robot_Cls` / `Mechanism_Cls` for rendered, script-driven scenes.
- PyBullet `Robot_Cls` / `Mechanism_Cls` for interactive physics with `Reset`, `Step`,
  `Set_Absolute_Joint_Position`, `Get_Inverse_Kinematics_Solution` and external-object handling.

## Supported Robots and Mechanisms

The following structures are predefined in `src/RoLE/Parameters/Robot.py` and
`src/RoLE/Parameters/Mechanism.py`. The degrees of freedom (DoF) and joint types are taken directly from the
`Theta.Type` fields (`R` – revolute, `P` – prismatic). Analytical inverse kinematics is currently
implemented only for the SCARA structure; all other robots use the numerical solvers.

| Robot / Mechanism | Type | DoF | URDF | Blender | Analytical IK |
|---|---|---|---|---|---|
| `Universal_Robots_UR3` | 6-DoF serial (6R) | 6 | ✓ | ✓ | ✗ |
| `ABB_IRB_120` | 6-DoF serial (6R) | 6 | ✓ | ✓ | ✗ |
| `ABB_IRB_120_L_Ax` | 6-DoF serial on linear axis (1P + 6R) | 7 | ✓ | ✓ | ✗ |
| `ABB_IRB_14000_R` | 7-DoF serial, dual-arm right (7R) | 7 | ✓ | ✓ | ✗ |
| `ABB_IRB_14000_L` | 7-DoF serial, dual-arm left (7R) | 7 | ✓ | ✓ | ✗ |
| `EPSON_LS3_B401S` | 4-DoF SCARA (RRPR) | 4 | ✓ | ✓ | ✓ |
| `SMC_LEFB25_14000` | Linear axis mechanism (1P) | 1 | ✓ | ✓ | — |

*The two ABB IRB 14000 arms are the left and right arms of the dual-arm YuMi platform; their shared base is
provided separately as `ABB_IRB_14000_Base`. The `SMC_LEFB25_14000` mechanism is defined with two instances
(`Id = 1`, `Id = 2`).*

<p align="center">
  <img src=https://github.com/rparak/RoLE/blob/main/images/I4C_Robots.png width="800">
</p>

*The serial manipulators and linear mechanism supported by the library.*

## Project Hierarchy

```text
[RoLE]
├── src/                                # Source code of the library
│   ├── RoLE/                           # Core, simulator-independent library
│   │   ├── Kinematics/                 # FK, IK (analytical + numerical), geometric Jacobian
│   │   ├── Parameters/                 # Robot_Parameters_Str / Mechanism_Parameters_Str definitions
│   │   ├── Transformation/             # HTM, Vector3, Euler angle and quaternion classes
│   │   ├── Collider/                   # AABB / OBB colliders (overlap, raycast, point-inside)
│   │   ├── Primitives/                 # Point, line-segment and box primitives
│   │   ├── Trajectory/                 # Trapezoidal / polynomial profiles, multi-segment paths
│   │   ├── Interpolation/              # Bézier and B-spline curves, Lerp/Slerp, simplification
│   │   ├── Workspace/                  # Workspace sampling and XYZ generation
│   │   ├── URDF/                       # URDF file generator
│   │   ├── Simplification/             # SymPy-based symbolic FK / Jacobian
│   │   └── Utilities/                  # File I/O and moment-of-inertia helpers
│   ├── Blender/                        # Blender-side robot / mechanism API and configuration
│   └── PyBullet/                       # PyBullet-side robot / mechanism API and environment
├── Evaluation/                         # Runnable demos and benchmarks
│   ├── Kinematics/                     # FK, IK, and collision-pair scripts
│   ├── Blender/                        # Scripts executed inside Blender
│   ├── PyBullet/                       # PyBullet control and environment demos
│   ├── URDF/                           # URDF generation / loading tests
│   └── Workspace/                      # Workspace generation and 3D plotting
├── URDFs/                              # URDF descriptions and meshes
│   ├── Robots/                         # One folder per robot (URDF + visual/collision meshes)
│   ├── Mechanisms/                     # Linear-axis mechanism descriptions
│   ├── Primitives/                     # Cube / plane / sphere URDF primitives
│   └── Viewpoint/                      # End-effector viewpoint marker
├── Blender/                            # Blender scenes (.blend) for each robot and mechanism
├── Data/                              # Generated data sets
│   ├── Inverse_Kinematics/             # Per-robot IK input/result data
│   └── Workspace/                      # Per-robot workspace point clouds
├── Textures/                           # Texture images used by the scenes
└── images/                             # Figures used in this README
```

- **`src`** — the library itself. `src/RoLE` is the simulator-independent core; `src/Blender` and
  `src/PyBullet` are thin front-ends that reuse the same parameter structures.
- **`Evaluation`** — self-contained scripts that demonstrate and benchmark each capability. Each script sets
  its own `sys.path` bootstrap and exposes `CONST_*` constants at the top for choosing a robot or method.
- **`URDFs`** — Unified Robotics Description Format files and their visual/collision meshes for the robots,
  mechanisms and primitives.
- **`Blender`** — `.blend` scenes, one per robot and mechanism, opened before running the Blender evaluation
  scripts.
- **`Data`** — inverse-kinematics and workspace data generated by the evaluation scripts and consumed by the
  visualisation scripts.
- **`Textures`** — image textures referenced by the scenes.
- **`images`** — the figures embedded in this document.

## Requirements

**Programming Language**

```bash
Python
```

**Supported operating systems**

```bash
Linux, macOS, Windows
```

**Dependencies**

The core library depends only on NumPy. The remaining packages are needed only for specific subsets of the
repository, as noted below.

| Package | Purpose | Where it is used |
|---|---|---|
| `numpy` | Array computing; all core math | Everywhere (`src/RoLE`, `src/Blender`, `src/PyBullet`) |
| `sympy` | Symbolic FK and Jacobian simplification | `src/RoLE/Simplification/*.py` only |
| `pybullet` | Physics simulation and interactive control | `src/PyBullet`, `Evaluation/PyBullet` only |
| `bpy` (Blender API) | Blender scene control; available only inside Blender | `src/Blender`, `Evaluation/Blender` only |
| `matplotlib` | Plotting of results and workspaces | `Evaluation` visualisation scripts only |
| `scienceplots` | Matplotlib styling for result figures | `Evaluation/Kinematics/IK/Results`, `Evaluation/Workspace` only |

For per-script details, see the import block and header comments at the top of each `.py` file (for example
`# Numpy (Array computing) [pip3 install numpy]`).

## Installation Dependencies

It will be useful for the project to create a virtual environment using Conda. Conda is an open source
package management system and environment management system that runs on Windows, macOS, and Linux. Conda
quickly installs, runs and updates packages and their dependencies.

**Get the repository**

```bash
git clone https://github.com/rparak/RoLE.git
```

**Set up a new virtual environment called {name} with python {version}**

```
$ ../user_name> conda create -n {name} python={version}
$ ../user_name> conda activate {name}
```

**Installation of packages needed for the project**

```
Matplotlib
$ ../user_name> conda install -c conda-forge matplotlib

SciencePlots
$ ../user_name> conda install -c conda-forge scienceplots

PyBullet
$ ../user_name> conda install -c conda-forge pybullet
```

The library is not installed as a package. Instead, each script adds the `src/` folder to `sys.path`, exactly
as the evaluation scripts do:

```python
import sys
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
```

**Other useful commands for working with the Conda environment**

```
Deactivate environment.
$ ../user_name> conda deactivate

Remove environment.
$ ../user_name> conda remove --name {name} --all

To verify that the environment was removed, in your terminal window or an Anaconda Prompt, run.
$ ../user_name> conda info --envs

Rename the environment from the old name to the new one.
$ ../user_name> conda rename -n {old_name} {name_name}
```

## Getting Started / Usage

The examples below are distilled from the scripts in `Evaluation/`. The relative `sys.path` bootstrap must
match the folder depth of the script that uses it; the snippets here use the depth of
`Evaluation/Kinematics/...` (`'../../../' + 'src'`).

### 1. Forward kinematics of a predefined robot

Compute the homogeneous transformation matrix of the end-effector for a given joint configuration.

```python
import sys
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
import numpy as np
import RoLE.Parameters.Robot as Parameters
import RoLE.Kinematics.Core

# Choose a predefined robot.
Robot_Str = Parameters.Universal_Robots_UR3_Str

# Desired absolute joint positions (radians for revolute joints).
theta = Robot_Str.Theta.Home

# Forward kinematics; method = 'Standard', 'Modified' or 'Fast'.
(th_limit_err, T) = RoLE.Kinematics.Core.Forward_Kinematics(theta, 'Fast', Robot_Str)

print(T.p.all())                          # End-effector position [x, y, z].
print(T.Get_Rotation('QUATERNION').all()) # End-effector orientation as a quaternion.
```

### 2. Numerical inverse kinematics

Recover the joint positions that reach a desired end-effector pose using one of the four numerical solvers.

```python
import RoLE.Parameters.Robot as Parameters
import RoLE.Kinematics.Core

Robot_Str = Parameters.Universal_Robots_UR3_Str

# A desired TCP pose (here obtained from a known joint configuration).
T_desired = RoLE.Kinematics.Core.Forward_Kinematics(Robot_Str.Theta.Home, 'Fast', Robot_Str)[1]

# Numerical IK solver properties.
ik_properties = {'delta_time': 0.1, 'num_of_iteration': 500, 'tolerance': 1e-30}

# method = 'Jacobian-Transpose', 'Newton-Raphson', 'Gauss-Newton', 'Levenberg-Marquardt'.
(info, theta) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(
    T_desired, Robot_Str.Theta.Zero, 'Levenberg-Marquardt', Robot_Str, ik_properties)

print(info['successful'], info['iteration'])
print(info['error']['position'], info['error']['orientation'])
print(theta)
```

### 3. PyBullet simulation from a URDF

Instantiate a robot from its URDF file, reset it to the home configuration and command an absolute joint
position. (Requires `pybullet`.)

```python
import os
import RoLE.Parameters.Robot as Parameters
import PyBullet.Robot.Core

Robot_Str = Parameters.Universal_Robots_UR3_Str

# Locate the project folder and the URDF file.
project_folder = os.getcwd().split('RoLE')[0] + 'RoLE'
urdf_path = f'{project_folder}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf'

env_properties = {'Enable_GUI': True, 'fps': 100, 'External_Base': None, 'Env_ID': 0,
                  'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance': 1.3,
                             'Position': [0.05, -0.10, 0.06]}}

Robot = PyBullet.Robot.Core.Robot_Cls(Robot_Str, urdf_path, env_properties)
Robot.Reset('Home')

# Command an absolute joint position: theta, force, t_0, t_1.
Robot.Set_Absolute_Joint_Position(Robot_Str.Theta.Home, 100.0, 0.0, 1.0)
```

### 4. Generate a URDF file

Emit a URDF description for a predefined robot.

```python
import RoLE.Parameters.Robot as Parameters
import RoLE.URDF.Core

Robot_Str = Parameters.Universal_Robots_UR3_Str

# enable_mesh = True to reference meshes; base colour as RGBA.
URDF_Generator = RoLE.URDF.Core.URDF_Generator_Cls(Robot_Str, True, [0.90, 0.90, 0.90, 1.0])
URDF_Generator.Generate()
# URDF_Generator.Save(f'.../URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}')
```

### Running the evaluation scripts

- **Working directory and `sys.path`.** Each script appends `src/` to `sys.path` with a relative path that
  matches its folder depth (`'../../../' + 'src'`, `'../../../../' + 'src'`, etc.). Run each script from its
  own folder so the relative path resolves.
- **`CONST_*` constants.** The top of every evaluation script defines constants such as `CONST_ROBOT_TYPE`,
  `CONST_NIK_METHOD` and `CONST_IK_PROPERTIES`. Edit these to select the robot, the method or the solver
  properties before running.
- **Blender scripts.** Scripts under `Evaluation/Blender/` are not run from a terminal. Open the matching
  `.blend` scene from the `Blender/` folder, then copy the script into Blender's text editor and run it (the
  scripts `import bpy` and their headers document this).

## Evaluation / Examples Overview

| Folder | Demonstrates | Output |
|---|---|---|
| `Evaluation/Kinematics/FK` | Compares Standard / Modified / Fast forward kinematics and checks agreement | Timing and correctness printout |
| `Evaluation/Kinematics/IK` | Analytical and numerical IK tests and data collection | Console results; data in `Data/Inverse_Kinematics/` |
| `Evaluation/Kinematics/IK/Results` | Visualises collected IK results (error, TCP, time, iterations, joint positions) | Matplotlib / SciencePlots figures |
| `Evaluation/Kinematics/Collider` | Optimises the self-collision pair list | Optimised collision pairs |
| `Evaluation/Blender` | Robot / mechanism scenes, colliders, workspace and IK inside Blender | Rendered Blender scenes |
| `Evaluation/PyBullet/Control` | Interactive joint / configuration-space control of robots and mechanisms | PyBullet simulation |
| `Evaluation/PyBullet/Environment` | Environment set-up for robots and mechanisms | PyBullet simulation |
| `Evaluation/URDF` | Generates and loads URDF files for robots and mechanisms | URDF files under `URDFs/` |
| `Evaluation/Workspace` | Generates joint positions and XYZ workspace, then plots it | Data in `Data/Workspace/`; 3D plot |

## Gallery

<p align="center">
  <img src=https://github.com/rparak/RoLE/blob/main/images/I4C_Robots_Kinematics.png width="800">
</p>

*Forward and inverse kinematics of the supported robotic structures.*

<p align="center">
  <img src=https://github.com/rparak/RoLE/blob/main/images/I4C_Robots_Colliders.png width="800">
</p>

*Oriented and axis-aligned bounding-box colliders used for self- and external-collision detection.*

<p align="center">
  <img src=https://github.com/rparak/RoLE/blob/main/images/I4C_Robots_Workspace.png width="800">
</p>

*Sampled reachable workspace of the robotic structures.*

<p align="center">
  <img src=https://github.com/rparak/RoLE/blob/main/images/I4C_Robots_Trajectory.png width="800">
</p>

*Trajectory generation and Bézier / B-spline interpolation.*

<p align="center">
  <img src=https://github.com/rparak/RoLE/blob/main/images/I4C_PyBullet.png width="800">
</p>

*Robots loaded from their URDF descriptions in the PyBullet simulation environment.*

## Contact Info
Roman.Parak@outlook.com

## Citation (BibTex)
```bash
@misc{RomanParak_RoLE,
  author = {Roman Parak},
  title = {Robotics Library for Everyone (RoLE)},
  year = {2024},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/rparak/RoLE}}
}
```

## License
This project is licensed under the terms of the [MIT License](LICENSE). The same MIT header appears at the
top of the source files.
