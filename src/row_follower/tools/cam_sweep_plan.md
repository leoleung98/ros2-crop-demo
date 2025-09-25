Camera Pose Auto-Sweep (Phase 1)
Grid

z ∈ {0.25, 0.32, 0.40} (m)

pitch ∈ {0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.57} (rad)

image: 640×480, horizontal_fov = 1.047 rad, fps ≈ 30

Metrics (each run 10–15s)

Core (lane band @ y*):

/perception/green_ratio_lane

/perception/ground_ratio_lane (= 1 − green_lane)

|/perception/crop_symmetry| (absolute value)

Context / confidence:

/perception/green_ratio_full

/perception/lane_area_ratio

/perception/confidence ∈ [0,1] and /perception/conf_level ∈ {0,1,2}

/perception/roi_band_idx (0: lower-heavy band, 1: upper-heavy, 2: full/upper, 3: top-only — see code)

Note about this sim
Textured “crop” strips often yield very small green_ratio_lane (≈ 0.000–0.01) even when the view is excellent. We therefore use confidence (which combines lane green, full-image green, lane area, and geometry) as the primary operating criterion, and treat green_ratio_full + lane_area_ratio as sanity checks.

Table (measured means)
z (m)	pitch (rad)	mean green_lane	mean ground_lane	mean |sym|	Notes
0.25	0.0	0.0003	0.9997	0.0299	
0.25	0.2	0.0359	0.9641	0.0019	
0.25	0.4	0.0004	0.9996	0.0189	
0.25	0.6	0.0003	0.9997	0.0018	
0.25	0.8	0.0000	0.0000	0.0000	ROI fails / invalid
0.25	1.0	0.0000	0.0000	0.0000	ROI fails / invalid
0.25	1.2	0.0000	0.0000	0.0000	ROI fails / invalid
0.25	1.4	0.0000	0.0000	0.0000	ROI fails / invalid
0.25	1.57	0.0000	0.0000	0.0000	ROI fails / invalid

| 0.32 | 0.0 | 0.0003 | 0.9997 | 0.0016 | |
| 0.32 | 0.2 | 0.0003 | 0.9997 | 0.0105 | |
| 0.32 | 0.4 | 0.0004 | 0.9996 | 0.0086 | |
| 0.32 | 0.6 | 0.0003 | 0.9997 | 0.0033 | baseline candidate |
| 0.32 | 0.8 | 0.0000 | 0.0000 | 1.2313 | ROI fails / invalid |
| 0.32 | 1.0 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |
| 0.32 | 1.2 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |
| 0.32 | 1.4 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |
| 0.32 | 1.57 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |

| 0.40 | 0.0 | 0.0078 | 0.9922 | 0.0020 | |
| 0.40 | 0.2 | 0.0003 | 0.9997 | 0.0029 | |
| 0.40 | 0.4 | 0.0004 | 0.9996 | 0.0013 | alt candidate |
| 0.40 | 0.6 | 0.0003 | 0.9997 | 0.0019 | |
| 0.40 | 0.8 | 0.0002 | 0.9998 | 0.0060 | borderline |
| 0.40 | 1.0 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |
| 0.40 | 1.2 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |
| 0.40 | 1.4 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |
| 0.40 | 1.57 | 0.0000 | 0.0000 | 0.0000 | ROI fails / invalid |

Pattern

pitch 0–0.6: stable — lane band clear (green_lane ≈ 0, ground_lane ≈ 1), |sym| small.

pitch ≥ 0.8: invalid — ROI not found / poor geometry; avoid steep pitch.

Baseline summary (measured)

Chosen default pose: z = 0.32 m, pitch = 0.60 rad

Typical steady-state (example you recorded):

green_lane ≈ 0.000–0.001, ground_lane ≈ 0.999

green_full ≈ 0.03, lane_area_ratio ≈ 0.92

confidence ≈ 0.66, conf_level = 2

roi_band_idx = 1 (upper-heavy / full-height band)

Rationale: robust ROI lock, high lane coverage, small |sym|, confidence ≥ 0.6.

Selection rule (what “good” looks like here)

Because green_lane is tiny in this world, we accept poses with:

confidence ≥ 0.60 (primary gate)

lane_area_ratio ≥ 0.50 (lane polygon covers enough of the image)

0.01 ≤ green_full ≤ 0.35 (scene contains some crop; not all ground, not all crop)

mean |sym| small (≈ 0.00–0.02 at baseline)

If you run a different map/texture and green_lane becomes meaningful (≥ 0.15), then revert to the original rule “green_lane ≥ 0.30 & ground_lane ≥ 0.20 with small |sym|”; until then, use confidence.

Shortlist (recommended poses)
z (m)	pitch (rad)	mean |sym|	confidence (typ.)	Notes
0.32	0.60	~0.003	~0.66 (level=2)	Default: stable, lane band clean, ROI idx ≈ 1
0.40	0.40	~0.001	~0.60–0.65	Also good; slightly higher z tolerated

(Confidence values are from your recent runs; fill in more as you sweep.)

Procedure

Edit <link name="camera_link"> <pose> in world/crops.world to set a candidate (z, pitch).

Rebuild & launch:

colcon build && source install/setup.bash
ros2 launch row_follower row_follow_launch.py


Record ~10–15 s of topics (3 terminals or one bag):

# CSV quick way
ros2 topic echo /perception/green_ratio_lane   --csv > green.csv
ros2 topic echo /perception/ground_ratio_lane  --csv > ground.csv
ros2 topic echo /perception/crop_symmetry      --csv > sym.csv
ros2 topic echo /perception/green_ratio_full   --csv > green_full.csv
ros2 topic echo /perception/lane_area_ratio    --csv > lane_area.csv
ros2 topic echo /perception/confidence         --csv > conf.csv
ros2 topic echo /perception/roi_band_idx       --csv > roi.csv


Compute means:

# mean green / ground / |sym|
awk -F, 'NR>1{s+=$2;n++} END{print s/n}' green.csv
awk -F, 'NR>1{s+=$2;n++} END{print s/n}' ground.csv
awk -F, 'NR>1{a=$2; if(a<0) a=-a; s+=a; n++} END{print s/n}' sym.csv

# mean green_full / lane_area / confidence
awk -F, 'NR>1{s+=$2;n++} END{print s/n}' green_full.csv
awk -F, 'NR>1{s+=$2;n++} END{print s/n}' lane_area.csv
awk -F, 'NR>1{s+=$2;n++} END{print s/n}' conf.csv

# most frequent ROI band index (mode)
awk -F, 'NR>1{c[$2]++} END{m=0;for (k in c) if(c[k]>m){m=c[k];w=k} print w+0}' roi.csv


Fill the table / shortlist; pick the default pose meeting the confidence rule above.

Write the chosen (z, pitch) back to crops.world and grab two screenshots:

rqt (/camera/image_raw) and (/perception/mask) at steady state.

Commit & tag:

git add .
git commit -m "tools: camera sweep results + Phase1 baseline"
git tag v0.2-camera-autoadapt
git push --tags