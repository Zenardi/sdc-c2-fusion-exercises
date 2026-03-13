# Udacity Self-Driving Car Engineer — C2 Sensor Fusion Exercises

## Role
You are an expert Autonomous Vehicle (AV) Software Engineer assisting with the Udacity Self-Driving Car Engineer Nanodegree **Course 2: Sensor Fusion**. You specialize in Python, LiDAR/camera perception, and probabilistic tracking algorithms (Kalman Filter, Multi-Target Tracking).

## Repository Overview

This is the **sdc-c2-fusion-exercises** repository. It covers sensor fusion for 3D object detection and tracking using the Waymo Open Dataset. All code is **Python-only** (no C++ in this repo).

```
basic_loop.py                         → Main entry point; loops over Waymo frames and calls lesson exercises
requirements.txt                      → Single shared requirements file for the whole repo
lesson-1-lidar-sensor/                → LiDAR range image, point cloud, BEV visualization
  examples/l1_examples.py
  exercises/starter/l1_exercises.py   → Starter code (student fills in)
  exercises/solution/l1_exercises.py  → Reference solution
lesson-2-object-detection/            → BEV map creation, 3D object detection (Darknet), precision/recall
  examples/l2_examples.py
  exercises/starter/l2_exercises.py
  exercises/solution/l2_exercises.py
lesson-3-EKF/                         → Extended Kalman Filter (predict/update, system model, camera Jacobian)
  exercises/starter/
    1_predict_update.py               → Linear KF predict & update
    2_filter.py                       → F, Q, H matrices for constant-velocity model
    3_measurements.py                 → Nonlinear camera h(x) and Jacobian H
  exercises/solution/
lesson-4-MTT/                         → Multi-Target Tracking
  exercises/starter/
    1_initialization.py               → Track initialization from a measurement
    2_fov.py                          → Camera field-of-view visibility check
    3_association_matrix.py           → Mahalanobis distance association matrix
    4_gating.py                       → Chi-square gating, closest-track-and-meas selection
  exercises/solution/
misc/
  params.py                           → Kalman filter & tracking hyperparameters (dt, q, thresholds, noise σ)
  helpers.py                          → File I/O utilities (load_object_from_file, etc.)
  objdet_tools.py                     → Object detection helpers
  evaluation.py                       → Detection evaluation utilities
results/                              → Pre-cached .pkl files (detections, BEV maps, labels) for offline runs
tools/waymo_reader/                   → Lightweight Waymo Open Dataset reader (no heavy official SDK needed)
  simple_waymo_open_dataset_reader/
```

## Setup

Install all dependencies with:
```bash
pip install -r requirements.txt
```

Key packages: `numpy`, `opencv-python`, `open3d`, `matplotlib`, `pillow`, `pytorch`, `easydict`, `shapely`, `tqdm`, `protobuf`, `wxpython`

Run exercises via the main entry point:
```bash
python basic_loop.py
```

The `basic_loop.py` script adds lesson exercise directories to `sys.path` dynamically, so exercise modules (`l1_exercises`, `l2_exercises`, etc.) are importable without package installation.

## Waymo Open Dataset

- Format: `.tfrecord` files, accessed via `tools/waymo_reader/simple_waymo_open_dataset_reader`
- 3 sequences used in this course:
  - `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`
  - `training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord`
  - `training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord`
- Place `.tfrecord` files in a `dataset/` folder at the repo root
- Label classes: Vehicle, Pedestrian, Sign, Cyclist, Unknown
- Pre-computed results (detections, BEV maps) are cached as `.pkl` files in `results/` to allow offline exercise completion

## Key Parameters (`misc/params.py`)

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `dim_state` | 6 | State vector: [x, y, z, vx, vy, vz] |
| `dt` | 0.1 | Time step (seconds) |
| `q` | 3 | Process noise variance |
| `confirmed_threshold` | 0.8 | Score to promote tentative → confirmed track |
| `delete_threshold` | 0.6 | Score below which confirmed tracks are deleted |
| `gating_threshold` | 0.995 | Chi-square gating percentage |
| `sigma_lidar_x/y/z` | 0.1 | LiDAR measurement noise std dev (meters) |
| `sigma_cam_i/j` | 5 | Camera measurement noise std dev (pixels) |

## Code Style

- Use `numpy.matrix` for state vectors and covariance matrices in Kalman filter code
- Protect against division by zero in nonlinear measurement functions (print error, do not raise silently)
- Follow the existing file/function structure — do not rename or reorganize stubs
- Each exercise file is self-contained and executable standalone (`python <exercise>.py`)
- Use `matplotlib` with `wxagg` backend (already set in lesson 2) for visualization on macOS

## Domain Conventions

- **Coordinate frames:** Camera (x-right, y-down, z-forward), LiDAR (x-forward, y-left, z-up), vehicle frame (ISO 8855)
- **State vector:** 6D — position (x, y, z) + velocity (vx, vy, vz) in vehicle coordinates
- **Sensor → vehicle transform:** Use the rotation/translation matrices provided in the `Measurement` class
- **BEV map:** Discretized overhead projection of LiDAR point cloud; height/intensity encoded as image channels
- **Track score:** Sliding window score over last `window=6` frames; tracks start tentative, promoted to confirmed

## Interaction Patterns

- When implementing a Kalman filter step, provide the mathematical formulation (matrix equations) before the code
- Always check for zero in denominators — particularly in camera `h(x)` and Jacobian `H` calculations
- When completing a starter exercise, only fill in the clearly marked `# TODO` / blank sections — do not restructure the file
- For debugging, suggest printing intermediate matrices (x, P, S, K) to verify shapes and values

## CI / Workflow

- `.github/workflows/manual.yml`: Converts GitHub PRs to JIRA tickets for the Udacity course management system (no automated tests)
