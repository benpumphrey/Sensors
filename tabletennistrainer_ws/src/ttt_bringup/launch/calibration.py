# ── TTT System Parameters ─────────────────────────────────────────────────────
# Single source of truth for all tunable values.
# Imported by jetsonA.launch.py, jetsonB.launch.py, and marty_gui.py.
# Edit here and restart to apply changes everywhere.

PARAMS = {

    # ── Camera Hardware ───────────────────────────────────────────────────────
    'exposure':          10000,   # Increase for more light (Warning: Too high will drop FPS below 240)
    'analogue_gain':      1600,   # Increase for more digital brightness without dropping FPS
    'width':               640,
    'height':              400,
    'fps':                 240,

    # ── Ball Detection (vision_node) ──────────────────────────────────────────
    'min_area':                4,
    'max_area':             300,
    'motion_threshold':      10,   # Increased to bias towards distinct motion and ignore edge flicker
    'min_contrast':         65,
    'dilate_iters':          1,
    'edge_margin':          10,
    'table_roi_left':     [103, 154, 336, 42, 531, 137, 237, 381],
    'table_roi_right':    [313, 25, 545, 98, 446, 345, 132, 132],

    # ── Stereo Camera Intrinsic Lenses ────────────────────────────────────────
    'fx':                448.0,
    'fy':                448.0,
    'cx':                320.0,
    'cy':                200.0,
    
    # ── Stereo Alignment & Origin Shift (stereo_node) ─────────────────────────
    'baseline_m':        1.397,   # Distance between cameras
    'max_sync_age_ms':      15,   
    'net_dist_z':         0.61,   # Z-distance from cameras to the physical net
    'height_left':        0.7,   # Physical height of Left Cam
    'height_right':       0.64,   # Physical height of Right Cam
    'pan_left_deg':       29.0,
    'pan_right_deg':      28.8,
    'tilt_left_deg':      33.4,
    'tilt_right_deg':     35.6,
    'roll_left_deg':     -17.1,
    'roll_right_deg':      7.9,
    'limit_x_m':           1.5,   # 3D bounding box (width max +/-)
    'limit_y_top_m':       2.0,   # 3D bounding box (height max)
    'limit_y_bottom_m':   -0.2,   # 3D bounding box (height min)
    'limit_z_m':           2.5,   # 3D bounding box (depth max +/-)

    # ── Robust Trajectory Prediction (trajectory_node) ────────────────────────
    'lookahead_ms':        150,   # How far AFTER the bounce to intercept the ball (ms)
    'min_samples':           5,   # Min detections before predicting
    'max_samples':          10,   # Rolling window size
    'gravity':            9.81,   # m/s^2
    'restitution':        0.85,   # Energy retained after bounce (0–1)
    'min_incoming_speed':  0.5,   # MIN SPEED: Ignore shots slower than 0.5 m/s
    'net_margin_z':       -0.2,   # ORIGIN GATE: Shot must have crossed Z > -0.2m
    'max_track_z':         1.5,  # END OF TABLE: Ignore noise beyond this Z depth
    'max_velocity':        25.0,  # SPEED LIMIT: Ignore tracking jumps > 25 m/s (~56mph)
}