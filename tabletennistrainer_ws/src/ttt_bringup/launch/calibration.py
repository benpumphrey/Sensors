# ── TTT System Parameters ─────────────────────────────────────────────────────
# Single source of truth for all tunable values.
# Imported by jetsonA.launch.py, jetsonB.launch.py, and marty_gui.py.
# Edit here and restart to apply changes everywhere.

PARAMS = {

    # ── Camera ────────────────────────────────────────────────────────────────
    'exposure':         7000,
    'analogue_gain':    1200,
    'width':             640,
    'height':            400,
    'fps':               240,

    # ── Ball Detection (vision_node) ──────────────────────────────────────────
    'min_area':             1,
    'max_area':          2000,
    'motion_threshold':     5,
    'min_contrast':         14,
    'dilate_iters':         1,
    'edge_margin':         10,

    # ── Table ROI (pixels) — empty = auto-detect from dark surface on startup ─
    # Set by GUI calibration (/api/set_roi) or edit manually.
    # Format: [x0,y0, x1,y1, x2,y2, x3,y3]  (top-left → top-right → bottom-right → bottom-left)
    'table_roi_left':  [169, 177, 378, 123, 516, 198, 216, 326],
    'table_roi_right': [290, 41, 510, 87, 464, 258, 143, 132],

    # ── Stereo Triangulation (stereo_node) ────────────────────────────────────
    # Cameras are mounted on the side rails, midway between MARTY's end and the net.
    # Baseline = distance between the two cameras across the table width.
    'fx':              448.0,   # focal length x (pixels)
    'fy':              448.0,   # focal length y (pixels)
    'cx':              320.0,   # principal point x (pixels) — half image width
    'cy':              200.0,   # principal point y (pixels) — half image height
    'baseline_m':      1.397,   # distance between cameras (m) — measured 4ft 7in
    'max_sync_age_ms':    50,   # max timestamp gap between left/right detections (ms)
    'pan_left_deg':     17.1,
    'net_dist_z': 0.56,
    'height_right': 0.34,
    'height_left': 0.58,
    'tilt_right_deg': 27.2,
    'tilt_left_deg': 21.7,   # left camera pan inward toward table (degrees)
    'pan_right_deg':    17.1,
    'roll_left_deg':    -14.0,
    'roll_right_deg':   13.8,   # right camera pan inward toward table (degrees)

    # ── Trajectory Prediction (trajectory_node) ───────────────────────────────
    'lookahead_ms':      100,   # how far ahead to predict (ms)
    'min_samples':         3,   # min detections before predicting
    'max_samples':        12,   # rolling window size
    'gravity':          9.81,   # m/s²
    'camera_tilt_deg':  44.0,   # camera tilt downward from horizontal (degrees)
    'table_y':           1.23,   # Y coordinate of table surface in camera frame (tune after mount)
    'restitution':      0.85,   # energy retained after bounce (0–1)
    'net_z':           0.5207,  # Z of net in camera frame; negative = gate disabled

}
