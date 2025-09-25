Row Follower (ROS 2 + Gazebo)

Camera-based row following with lane-band perception, adaptive ROI (“software gimbal”), and a confidence-aware controller.

ROS 2: Humble

Sim: Gazebo 11

Image: 640×480, horizontal FOV ≈ 1.047 rad, ≈ 30 fps

Repository layout
row_follower/
├── row_follower/
│   ├── perception.py          # perception node
│   ├── controller.py          # controller node
│   ├── __init__.py
│   └── ...
├── launch/
│   └── row_follow_launch.py   # launches Gazebo + nodes + loads YAML
├── world/
│   └── crops.world            # baseline camera pose set here
├── config/
│   ├── perception.yaml        # packaged defaults
│   └── controller.yaml
├── tools/
│   ├── cam_sweep_plan.md      # Phase 1 sweep grid, table & procedure
│   ├── record_phase1.sh
│   └── compute_means.py
├── package.xml
├── setup.py
└── LICENSE


When installed, packaged config files resolve to:
$(ros2 pkg prefix row_follower)/share/row_follower/config/{perception.yaml,controller.yaml}

Build & run
cd ~/ros2_ws
colcon build
source install/setup.bash

ros2 launch row_follower row_follow_launch.py


The launch file will:

Start Gazebo with world/crops.world

Launch row_follower/perception and row_follower/controller

Load default parameters from config/perception.yaml and config/controller.yaml

Phase 0 — Baseline (done)
Goals

Bring-up camera, bridge, and control loop

Simple green segmentation → mask

Fixed vertical ROI → left/right inner edge fitting

Basic lateral + heading errors → /row_errors = [lateral_norm, heading]

Publish visualization topics

Key topics (Phase 0 and onward)

/perception/mask (mono8) — green mask

/perception/overlay (bgr8) — debug overlay (lane polygon + edges)

/perception/lane_edges (Float32MultiArray: [xL*, xR*] @ y*)

/perception/lane_width_px (Float32)

/perception/lateral_px, /perception/lateral_m (Float32)

/perception/crop_symmetry (Float32) ∈ [-0.5, +0.5]
0 = centered, >0 = bias to right, <0 = bias to left

/row_errors (Float32MultiArray) = [lateral_norm, heading]
where lateral_norm = crop_symmetry

/perception/state_str (String) — single-line debug state

Phase 1 — Camera Autoadapt (done)
What’s new

Lane-band metrics (at y*)

/perception/green_ratio_lane ∈ [0,1]

/perception/ground_ratio_lane = 1 − green_ratio_lane

Scene metrics

/perception/green_ratio_full ∈ [0,1]

/perception/lane_area_ratio ∈ [0,1] (lane polygon / image area)

Adaptive ROI (“software gimbal”)

3–4 vertical band candidates (e.g., [0.00,0.70], [0.00,0.95], [0.30,0.95], [0.50,0.95])

For each band: fit edges → lane polygon → compute metrics

Score = green_lane (primary) + lane_area + geometry – |sym|, plus hysteresis to avoid flapping

Selected index published: /perception/roi_band_idx (Int32: 0/1/2/3)

Confidence

/perception/confidence ∈ [0,1] and /perception/conf_level ∈ {0,1,2}

Combines lane-green, full-image green, lane area, and geometry; targets adapt to lane width fraction r

Controller scales forward speed with confidence_gain

Robustness

Prior-frame tracking: edge smoothing, step limits, width priors, local fallback search near last edges

Recovery: when ROI fails, publish safe defaults and lower confidence

Baseline camera pose

world/crops.world contains the default camera pose:

<pose>0.20 0 0.32 0 0.60 0</pose>


Typical steady-state (your logs):

y*≈228, lane_px≈639
green_lane≈0.000–0.001, ground_lane≈0.999
green_full≈0.032, lane_area≈0.92
confidence≈0.66, conf_level=2
roi_band_idx≈1


Note: In this sim, the texture yields tiny green_ratio_lane even when the view is excellent. We therefore use confidence as the main operating gate and also monitor green_ratio_full + lane_area_ratio.

Parameters (defaults)
Perception (config/perception.yaml)
perception:
  ros__parameters:
    # geometry / sampling
    lane_width_m: 1.0
    follow_mode: "center"        # or "left_offset"
    left_offset_m: 0.20
    num_bands: 24                # sampling bands (used per-ROI)
    y_low_frac: 0.00             # used when roi_enable=false
    y_high_frac: 0.95

    # adaptive ROI
    roi_enable: true
    roi_ground_min: 0.15
    roi_lane_area_min: 0.15
    roi_band_defs:               # optional; if empty, uses built-in defaults
      - [0.00, 0.70]             # idx 0: lower-heavy
      - [0.00, 0.95]             # idx 1: full/upper
      - [0.30, 0.95]             # idx 2: upper-biased
      - [0.50, 0.95]             # idx 3: top-only

    # ROI switching hysteresis
    roi_switch_delta: 0.05       # min score lead to switch
    roi_hold_frames: 5           # min hold before allowed to switch

    # temporal tracking & guards
    edge_max_step_px: 40.0       # max per-frame edge move
    edge_alpha: 0.35             # EMA smoothing
    min_lane_width_px: 120.0
    max_lane_width_px: 1200.0
    search_radius_px: 120.0      # local re-search radius

Controller (config/controller.yaml)
row_controller:
  ros__parameters:
    k_yaw: 1.2
    k_lat: 0.02
    v_forward: 0.50
    v_min: 0.05
    omega_limit: 1.50
    omega_acc_limit: 2.00
    sym_deadband: 0.01
    confidence_gain: 1.0


Both YAMLs are loaded by the launch file. You can override live and then restore defaults (see below).

How to tune online & restore defaults

Live tune examples

# perception ROI band window
ros2 param set /perception num_bands 24
ros2 param set /perception y_low_frac 0.00
ros2 param set /perception y_high_frac 0.95

# adaptive ROI toggles / thresholds
ros2 param set /perception roi_enable true
ros2 param set /perception roi_ground_min 0.15
ros2 param set /perception roi_lane_area_min 0.15
ros2 param set /perception roi_switch_delta 0.05
ros2 param set /perception roi_hold_frames 5

# controller balance
ros2 param set /row_controller k_yaw 1.2
ros2 param set /row_controller k_lat 0.02
ros2 param set /row_controller confidence_gain 1.0


Dump current runtime params

ros2 param dump /perception      > /tmp/perception.dump.yaml
ros2 param dump /row_controller  > /tmp/controller.dump.yaml


Restore packaged defaults

ros2 param load /perception     $(ros2 pkg prefix row_follower)/share/row_follower/config/perception.yaml
ros2 param load /row_controller $(ros2 pkg prefix row_follower)/share/row_follower/config/controller.yaml

Camera Pose Auto-Sweep (results & rule)

See full details in tools/cam_sweep_plan.md.

Rule used in this world (texture-specific):

confidence ≥ 0.60 (primary)

lane_area_ratio ≥ 0.50

0.01 ≤ green_full ≤ 0.35

mean |sym| small (≈ 0.00–0.02)

Recommended baselines
| z (m) | pitch (rad) | |sym| (typ.) | confidence | Notes |
|---:|---:|---:|---:|---|
| 0.32 | 0.60 | ~0.003 | ~0.66 (level=2) | Default |
| 0.40 | 0.40 | ~0.001 | ~0.60–0.65 | Also good |


Troubleshooting

ROI not found (no ROI workable) at start offsets
The node now keeps prior-frame tracking and a local re-search near last edges. If you start severely yawed/offset so only one side is visible, the one-side bootstrap kicks in:

If only left crop found → steer right; only right found → steer left, until both edges are visible.

This uses half-image green projections as a fallback heuristic.
If you still see failures, lower roi_lane_area_min (e.g. 0.12), increase search_radius_px (e.g. 160), and allow a more bottom-heavy ROI candidate (e.g. add [0.00, 0.60] to roi_band_defs).

Gradual bias / won’t re-center
Increase k_lat slightly (0.02→0.03), add sym_deadband (already 0.01), and check that edge_max_step_px isn’t too small (can be 50). Ensure omega_limit is sufficient (≥1.5).

Flapping ROI
Raise roi_hold_frames (e.g. 8–10) or increase roi_switch_delta (e.g. 0.08).

Config YAML load error
Ensure YAML syntax is valid (sequences must be values, not keys). Paths are loaded by launch from
$(ros2 pkg prefix row_follower)/share/row_follower/config/*.yaml.

License

MIT