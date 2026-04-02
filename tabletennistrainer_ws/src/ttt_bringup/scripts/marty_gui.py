import sys, cv2, numpy as np, subprocess, os, signal, time, threading, logging, json, math
from collections import deque
from flask import Flask, Response, render_template_string, request

# --- 1. DDS CONFIG ---
cyc_b = """<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><Interfaces><NetworkInterface address="192.168.1.20"/></Interfaces></General>
  </Domain>
</CycloneDDS>"""
with open('/tmp/cyc_b.xml', 'w') as f: f.write(cyc_b)

for k in ['CYCLONEDDS_URI']:
    if k in os.environ: del os.environ[k]
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = 'file:///tmp/cyc_b.xml'

# --- 2. SOURCE WORKSPACE ---
_scripts = ['/opt/ros/humble/setup.bash', '/home/capstone-nano2/Sensors/tabletennistrainer_ws/install/setup.bash']
for _s in _scripts:
    for _l in subprocess.run(['bash', '-c', f'source {_s} && env'], capture_output=True, text=True).stdout.splitlines():
        if '=' in _l: os.environ.setdefault(_l.split('=', 1)[0], _l.split('=', 1)[1])

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped
from ttt_msgs.msg import BallDetection

import sys as _sys, re as _re
_sys.path.insert(0, '/home/capstone-nano2/Sensors/tabletennistrainer_ws/src/ttt_bringup/launch')
from calibration import PARAMS as CAL

_CAL_PATH = '/home/capstone-nano2/Sensors/tabletennistrainer_ws/src/ttt_bringup/launch/calibration.py'

# --- MERGED FROM DOC 6: unified save helper replaces _save_roi_to_cal + _save_angles_to_cal ---
def _save_all_to_cal(data):
    """Write any subset of PARAMS keys back to calibration.py and update the live CAL dict."""
    try:
        with open(_CAL_PATH, 'r') as f: src = f.read()
        for k, v in data.items():
            if f"'{k}'" in src:
                src = _re.sub(rf"('{k}':\s*)[0-9.\-]+", lambda m, val=v: m.group(1) + str(val), src)
        with open(_CAL_PATH, 'w') as f: f.write(src)
        CAL.update(data)
    except Exception as e: print(f"Save Error: {e}")

def _save_roi_to_cal(side, pts):
    """Save table ROI pixel coords for one side back to calibration.py."""
    key = f'table_roi_{side}'
    val = repr(list(pts))
    try:
        with open(_CAL_PATH, 'r') as f: src = f.read()
        new_src = _re.sub(rf"('{key}':\s*)\[[^\]]*\]", lambda m: m.group(1) + val, src)
        with open(_CAL_PATH, 'w') as f: f.write(new_src)
        CAL[key] = list(pts)
    except Exception as e: pass

# --- HTML DASHBOARD ---
_HTML_PAGE = """<!DOCTYPE html>
<html>
<head>
  <title>M.A.R.T.Y. Control Center</title>
  <style>
    body{background:#121212;color:#e0e0e0;font-family:Consolas,monospace;margin:0;padding:20px;}
    h1{text-align:center;color:#00FF00;margin:0 0 12px;}
    .feeds{display:flex;gap:16px;justify-content:center;flex-wrap:nowrap;width:100%;}
    .feed{text-align:center;flex:1;}
    .feed h3{color:#aaa;margin:4px 0;}
    .feed img{border:2px solid #444;width:100%;max-width:48vw;height:auto;}
    .stats{background:#1e1e1e;border-radius:10px;border:1px solid #333;padding:16px;margin:16px auto;max-width:820px;}
    .hdr{color:#aaa;text-align:center;font-size:11px;margin:8px 0 4px;}
    .row{display:flex;justify-content:space-around;margin:6px 0;}
    .lbl{color:#aaa;font-size:11px;text-align:center;}
    .val{font-size:22px;font-weight:bold;text-align:center;}
    .sval{font-size:16px;text-align:center;}
    .x{color:#ff5555}.y{color:#55ff55}.z{color:#5555ff}
    .px{color:#ff9999}.py{color:#99ff99}.pz{color:#9999ff}
    .land{color:#ffcc00}
    .topics-panel{background:#1e1e1e;border-radius:10px;border:1px solid #333;padding:16px;margin:16px auto;max-width:1100px;overflow-x:auto;}
    .tp-table{width:100%;border-collapse:collapse;font-size:12px;}
    .tp-table th{color:#666;text-align:left;padding:4px 12px;border-bottom:1px solid #333;font-weight:normal;letter-spacing:1px;}
    .tp-table td{padding:5px 12px;border-bottom:1px solid #222;white-space:nowrap;}
    .tp-name{color:#ccc;font-family:Consolas,monospace;}
    .tp-type{color:#666;}
    .tp-age{font-weight:bold;min-width:60px;}
    .tp-vals{color:#aaa;font-family:Consolas,monospace;white-space:pre;}
    .fresh{color:#00ff00;}.warn{color:#ffaa00;}.stale{color:#ff4444;}
    .btn{background:#222;border:1px solid #0f0;color:#0f0;padding:6px 18px;font-family:Consolas,monospace;cursor:pointer;font-size:13px;margin:0 5px;border-radius:4px;}
    .btn:hover{background:#333;}
    .btn-align{border-color:#0ff;color:#0ff;}
    .btn-vision{border-color:#ff55ff;color:#ff55ff;}
    .btn-roi{border-color:#ffaa00;color:#ffaa00;}
  </style>
</head>
<body>
  <h1>M.A.R.T.Y. Control Center</h1>
  <div style="text-align:center;margin-bottom:8px;">
    <button class="btn" onclick="toggleCal()" id="cal-btn">&#9654; SET ROI</button>
    <button class="btn btn-align" onclick="toggleAlign()" id="align-btn">TARGET OVERLAY: OFF</button>
    <button class="btn btn-vision" onclick="toggleVisionMode()" id="vision-btn">TRACKING: MOTION (PLAY)</button>
    <button class="btn btn-roi" onclick="toggleRoiMask()" id="roi-toggle-btn">ROI: VISIBLE</button>
  </div>

  <div id="cal-panel" style="display:none;background:#1a1a1a;border:1px solid #0f0;border-radius:8px;padding:14px;margin:0 auto 10px;max-width:1200px;font-size:12px;">
    <div style="color:#0f0;font-weight:bold;margin-bottom:6px;">Click 4 table corners on each camera image below.</div>
    <div style="display:flex;gap:20px;flex-wrap:wrap;">
      <div style="flex:1;min-width:280px;">
        <div style="color:#aaa;margin-bottom:4px;">LEFT (Jetson A) <span id="lcnt" style="color:#ff5">(0/4)</span>
          <button onclick="resetCal('left')" style="background:#333;color:#f55;border:1px solid #f55;padding:2px 8px;cursor:pointer;font-size:11px;margin-left:8px;">Reset</button>
          <span id="lapply-status" style="margin-left:10px;font-size:12px;"></span></div>
        <div id="lpts" style="color:#ccc;line-height:1.8;min-height:48px;font-size:11px;"></div>
      </div>
      <div style="flex:1;min-width:280px;">
        <div style="color:#aaa;margin-bottom:4px;">RIGHT (Jetson B) <span id="rcnt" style="color:#ff5">(0/4)</span>
          <button onclick="resetCal('right')" style="background:#333;color:#f55;border:1px solid #f55;padding:2px 8px;cursor:pointer;font-size:11px;margin-left:8px;">Reset</button>
          <span id="rapply-status" style="margin-left:10px;font-size:12px;"></span></div>
        <div id="rpts" style="color:#ccc;line-height:1.8;min-height:48px;font-size:11px;"></div>
      </div>
    </div>
    <div style="margin-top:10px;border-top:1px solid #333;padding-top:8px;">
      <span style="color:#aaa;font-size:11px;">Net Z — place ball at net: </span>
      <button onclick="captureNetZ()" style="background:#222;color:#55f;border:1px solid #55f;padding:3px 10px;cursor:pointer;font-size:11px;font-family:Consolas,monospace;">Capture Net Z</button>
      <span id="net-z-val" style="color:#0ff;margin-left:10px;font-family:Consolas,monospace;font-size:11px;"></span>
    </div>
  </div>

  <div id="align-panel" style="display:none;background:#1a1a1a;border:1px solid #0ff;border-radius:8px;padding:16px;margin:12px auto 10px;max-width:1200px;text-align:center;">
    <div style="display:flex;gap:24px;justify-content:center;align-items:center;flex-wrap:wrap;margin-bottom:12px;">
      <div>
        <span style="color:#ff55ff;font-size:11px;font-weight:bold;">Dist to Net (Z): </span>
        <input type="range" id="net-slider" min="0.0" max="1.5" step="0.01" value="0.52" oninput="onAngleSlider()" style="width:120px;vertical-align:middle;">
        <span id="net-val" style="color:#ff55ff;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.52m</span>
      </div>
      <div>
        <span style="color:#fff;font-size:11px;font-weight:bold;">Min Contrast: </span>
        <input type="range" id="contrast-slider" min="5" max="150" step="1" value="25" oninput="onAngleSlider()" style="width:120px;vertical-align:middle;">
        <span id="contrast-val" style="color:#fff;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:35px;">25</span>
      </div>
      <button onclick="applyAngles()" style="background:#222;color:#0ff;border:1px solid #0ff;padding:6px 16px;cursor:pointer;font-family:Consolas,monospace;border-radius:4px;font-weight:bold;">Apply to C++</button>
      <span id="angle-status" style="font-size:12px;margin-left:8px;"></span>
    </div>

    <div style="display:flex;gap:40px;justify-content:center;flex-wrap:wrap;">
      <div style="border:1px solid #333; padding:10px; border-radius:6px; background:#111; text-align:right;">
        <div style="color:#00ffaa;font-size:12px;font-weight:bold;margin-bottom:8px;text-align:center;">LEFT CAM</div>
        <div>
          <span style="color:#aaa;font-size:10px;">Height (Y): </span>
          <input type="range" id="height-l-slider" min="0.0" max="1.5" step="0.01" value="0.889" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="height-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.89m</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Tilt (Pitch): </span>
          <input type="range" id="tilt-l-slider" min="0" max="60" step="0.1" value="45.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="tilt-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">45.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Pan: </span>
          <input type="range" id="pan-l-slider" min="0" max="45" step="0.1" value="15.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="pan-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">15.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Roll: </span>
          <input type="range" id="roll-l-slider" min="-30" max="30" step="0.1" value="0.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="roll-l-val" style="color:#00ffaa;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.0</span>
        </div>
      </div>

      <div style="border:1px solid #333; padding:10px; border-radius:6px; background:#111; text-align:right;">
        <div style="color:#ffaa00;font-size:12px;font-weight:bold;margin-bottom:8px;text-align:center;">RIGHT CAM</div>
        <div>
          <span style="color:#aaa;font-size:10px;">Height (Y): </span>
          <input type="range" id="height-r-slider" min="0.0" max="1.5" step="0.01" value="0.889" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="height-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.89m</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Tilt (Pitch): </span>
          <input type="range" id="tilt-r-slider" min="0" max="60" step="0.1" value="45.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="tilt-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">45.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Pan: </span>
          <input type="range" id="pan-r-slider" min="0" max="45" step="0.1" value="15.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="pan-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">15.0</span>
        </div>
        <div>
          <span style="color:#aaa;font-size:10px;">Roll: </span>
          <input type="range" id="roll-r-slider" min="-30" max="30" step="0.1" value="0.0" oninput="onAngleSlider()" style="width:100px;vertical-align:middle;">
          <span id="roll-r-val" style="color:#ffaa00;font-family:Consolas,monospace;font-weight:bold;display:inline-block;width:45px;">0.0</span>
        </div>
      </div>
    </div>
  </div>

  <div class="feeds">
    <div class="feed"><h3>LEFT CAMERA (Jetson A)</h3><img id="img-left" src="/stream/left" onclick="imgClick(event,'left')" style="cursor:crosshair;"></div>
    <div class="feed"><h3>RIGHT CAMERA (Jetson B)</h3><img id="img-right" src="/stream/right" onclick="imgClick(event,'right')" style="cursor:crosshair;"></div>
  </div>

  <div style="display:flex;gap:12px;justify-content:center;flex-wrap:wrap;margin:12px auto;max-width:1100px;">
    <div class="stats" style="flex:1;min-width:320px;">
      <div class="hdr">CURRENT POSITION (m)</div>
      <div class="row">
        <div><div class="lbl">X</div><div class="val x" id="x">--</div></div>
        <div><div class="lbl">Y</div><div class="val y" id="y">--</div></div>
        <div><div class="lbl">Z</div><div class="val z" id="z">--</div></div>
      </div>
      <div class="hdr" style="margin-top:12px;">PREDICTED +250ms (m)</div>
      <div class="row">
        <div><div class="lbl">pX</div><div class="sval px" id="px">--</div></div>
        <div><div class="lbl">pY</div><div class="sval py" id="py">--</div></div>
        <div><div class="lbl">pZ</div><div class="sval pz" id="pz">--</div></div>
      </div>
      <div class="row" style="margin-top:12px;">
        <div class="sval land" id="land">Landing: --</div>
      </div>
    </div>

    <div class="stats" style="min-width:280px;max-width:380px;">
      <div class="hdr">COORDINATE FRAME</div>
      <div style="font-size:10px;color:#666;text-align:center;margin-bottom:2px;">TOP VIEW</div>
      <svg width="100%" viewBox="0 0 260 150" style="display:block;margin:0 auto 6px;">
        <defs>
          <marker id="arrowZ" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0,0 L0,6 L6,3 z" fill="#5555ff"/></marker>
          <marker id="arrowX" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto"><path d="M0,0 L0,6 L6,3 z" fill="#ff5555"/></marker>
        </defs>
        <rect x="20" y="30" width="220" height="90" rx="3" fill="#1a2a1a" stroke="#333" stroke-width="1.5"/>
        <line x1="130" y1="30" x2="130" y2="120" stroke="#555" stroke-width="1.5" stroke-dasharray="4,3"/>
        <text x="133" y="80" fill="#666" font-size="8" font-family="Consolas">NET</text>
        <text x="75"  y="24" fill="#aaa" font-size="8" font-family="Consolas" text-anchor="middle">MARTY</text>
        <text x="195" y="24" fill="#aaa" font-size="8" font-family="Consolas" text-anchor="middle">OPPONENT</text>
        <rect x="70" y="22" width="10" height="8" rx="1" fill="#1a1a1a" stroke="#0f0" stroke-width="1.2"/>
        <text x="75" y="19" fill="#0f0" font-size="7" font-family="Consolas" text-anchor="middle">CAM L</text>
        <rect x="70" y="120" width="10" height="8" rx="1" fill="#1a1a1a" stroke="#0f0" stroke-width="1.2"/>
        <text x="75" y="138" fill="#0f0" font-size="7" font-family="Consolas" text-anchor="middle">CAM R</text>
        <line x1="75" y1="75" x2="235" y2="75" stroke="#5555ff" stroke-width="1.5" marker-end="url(#arrowZ)"/>
        <text x="240" y="79" fill="#5555ff" font-size="11" font-family="Consolas" font-weight="bold">Z</text>
        <line x1="75" y1="115" x2="75" y2="36" stroke="#ff5555" stroke-width="1.5" marker-end="url(#arrowX)"/>
        <text x="67" y="34" fill="#ff5555" font-size="11" font-family="Consolas" font-weight="bold">X</text>
        <line id="ray-l" x1="75" y1="26" x2="75" y2="26" stroke="#00ffaa" stroke-width="1.5" stroke-dasharray="5,3" opacity="0"/>
        <line id="ray-r" x1="75" y1="124" x2="75" y2="124" stroke="#ffaa00" stroke-width="1.5" stroke-dasharray="5,3" opacity="0"/>
        <circle id="ball-td" cx="0" cy="0" r="4" fill="#ffff00" stroke="#fff" stroke-width="1" opacity="0"/>
      </svg>
    </div>
  </div>

  <div class="topics-panel">
    <div class="hdr">ROS TOPICS</div>
    <table class="tp-table">
      <thead><tr><th>TOPIC</th><th>TYPE</th><th>AGE</th><th>VALUES</th></tr></thead>
      <tbody id="topics-tbody">
        <tr><td colspan="4" style="color:#444;text-align:center;padding:12px;">waiting for data...</td></tr>
      </tbody>
    </table>
  </div>

  <script>
    var _panL = 15.0, _panR = 15.0, _tiltL = 45.0, _tiltR = 45.0, _hl = 0.889, _hr = 0.889, _rollL = 0.0, _rollR = 0.0, _net = 0.52;
    var _fx = 448.2, _fy = 448.2, _cx = 320.0, _cy = 200.0, _baseline = 1.5;
    var _SVG_Z_SCALE = 80, _SVG_X_SCALE = 65;
    var _CAM_L = {x:75, y:26}, _CAM_R = {x:75, y:124};
    var calOpen = false, alignOpen = false, staticMode = false;
    var calPts = {left:[], right:[]};
    var CAM_W = 640, CAM_H = 400;
    var CORNER_LABELS = ['top-left','top-right','bottom-right','bottom-left'];

    function drawRaysSvg(d) {
      var rayL = document.getElementById('ray-l');
      var rayR = document.getElementById('ray-r');
      var ballDot = document.getElementById('ball-td');

      if (d.det_l_x !== null && d.det_l_x !== undefined) {
        var u = (d.det_l_x - _cx) / _fx;
        var panL = _panL * Math.PI / 180;
        var dlx = u * Math.cos(panL) + Math.sin(panL);
        var dlz = -u * Math.sin(panL) + Math.cos(panL);
        var sx = dlz * _SVG_Z_SCALE, sy = dlx * _SVG_X_SCALE;
        var t = 220 / Math.sqrt(sx*sx + sy*sy + 1e-9);
        rayL.setAttribute('x1', _CAM_L.x); rayL.setAttribute('y1', _CAM_L.y);
        rayL.setAttribute('x2', _CAM_L.x + sx*t); rayL.setAttribute('y2', _CAM_L.y + sy*t);
        rayL.setAttribute('opacity', '0.85');
      } else { rayL.setAttribute('opacity', '0'); }

      if (d.det_r_x !== null && d.det_r_x !== undefined) {
        var ur = (d.det_r_x - _cx) / _fx;
        var panR = -_panR * Math.PI / 180;
        var drx = ur * Math.cos(panR) + Math.sin(panR);
        var drz = -ur * Math.sin(panR) + Math.cos(panR);
        var sx = drz * _SVG_Z_SCALE, sy = drx * _SVG_X_SCALE;
        var t = 220 / Math.sqrt(sx*sx + sy*sy + 1e-9);
        rayR.setAttribute('x1', _CAM_R.x); rayR.setAttribute('y1', _CAM_R.y);
        rayR.setAttribute('x2', _CAM_R.x + sx*t); rayR.setAttribute('y2', _CAM_R.y + sy*t);
        rayR.setAttribute('opacity', '0.85');
      } else { rayR.setAttribute('opacity', '0'); }

      if (d.x !== null && d.x !== undefined && d.z !== null && d.z !== undefined) {
        ballDot.setAttribute('cx', 75 + d.z * _SVG_Z_SCALE);
        ballDot.setAttribute('cy', 26 + d.x * _SVG_X_SCALE);
        ballDot.setAttribute('opacity', '0.9');
      } else { ballDot.setAttribute('opacity', '0'); }
    }

    function poll(){
      fetch('/api/stats').then(r=>r.json()).then(d=>{
        document.getElementById('x').textContent=d.x!==null?d.x.toFixed(2):'--';
        document.getElementById('y').textContent=d.y!==null?d.y.toFixed(2):'--';
        document.getElementById('z').textContent=d.z!==null?d.z.toFixed(2):'--';
        document.getElementById('px').textContent=d.px!==null?d.px.toFixed(2):'--';
        document.getElementById('py').textContent=d.py!==null?d.py.toFixed(2):'--';
        document.getElementById('pz').textContent=d.pz!==null?d.pz.toFixed(2):'--';
        document.getElementById('land').textContent=(d.land_x!==null&&d.land_z!==null)?'Landing X: '+d.land_x.toFixed(2)+' Z: '+d.land_z.toFixed(2):'Landing: --';
        drawRaysSvg(d);
      }).catch(()=>{});
      setTimeout(poll,100);
    }
    poll();

    function pollTopics(){
      fetch('/api/topics').then(r=>r.json()).then(d=>{
        var topics = Object.keys(d).sort();
        var rows = '';
        if(topics.length === 0){
          rows = '<tr><td colspan="4" style="color:#444;text-align:center;padding:12px;">waiting for data...</td></tr>';
        } else {
          topics.forEach(function(name){
            var t = d[name];
            var age = t.age_ms;
            var ageStr = age === null ? 'never' : age < 1000 ? age+'ms' : (age/1000).toFixed(1)+'s';
            var ageCls = age === null ? 'stale' : age < 150 ? 'fresh' : age < 1000 ? 'warn' : 'stale';
            var vals = Object.entries(t.values).map(function(e){ return e[0]+': '+e[1]; }).join('    ');
            rows += '<tr><td class="tp-name">'+name+'</td><td class="tp-type">'+t.type+'</td>'
                  + '<td class="tp-age '+ageCls+'">'+ageStr+'</td><td class="tp-vals">'+vals+'</td></tr>';
          });
        }
        document.getElementById('topics-tbody').innerHTML = rows;
      }).catch(function(){});
      setTimeout(pollTopics, 250);
    }
    pollTopics();

    fetch('/api/config').then(r=>r.json()).then(cfg=>{
      _panL = cfg.pan_left_deg || 15.0;
      _panR = cfg.pan_right_deg || 15.0;
      _tiltL = cfg.tilt_left_deg || 45.0;
      _tiltR = cfg.tilt_right_deg || 45.0;
      _hl = cfg.height_left || 0.889;
      _hr = cfg.height_right || 0.889;
      _rollL = cfg.roll_left_deg || 0.0;
      _rollR = cfg.roll_right_deg || 0.0;
      _net = cfg.net_dist_z || 0.52;
      _baseline = cfg.baseline_m || 1.525;
      _fx = cfg.fx || 448.2;
      _fy = cfg.fy || 448.2;
      _cx = cfg.cx || 320.0;
      _cy = cfg.cy || 200.0;

      document.getElementById('pan-l-slider').value = _panL;
      document.getElementById('pan-r-slider').value = _panR;
      document.getElementById('tilt-l-slider').value = _tiltL;
      document.getElementById('tilt-r-slider').value = _tiltR;
      document.getElementById('height-l-slider').value = _hl;
      document.getElementById('height-r-slider').value = _hr;
      document.getElementById('roll-l-slider').value = _rollL;
      document.getElementById('roll-r-slider').value = _rollR;
      document.getElementById('net-slider').value = _net;
      document.getElementById('contrast-slider').value = cfg.min_contrast || 25;
      onAngleSlider();
    }).catch(()=>{});

    function toggleCal(){
      calOpen = !calOpen;
      document.getElementById('cal-panel').style.display = calOpen ? 'block' : 'none';
      document.getElementById('cal-btn').innerHTML = calOpen ? '&#9660; SET ROI' : '&#9654; SET ROI';
    }

    function toggleAlign(){
      fetch('/api/toggle_align', {method:'POST'});
      alignOpen = !alignOpen;
      var btn = document.getElementById('align-btn');
      btn.textContent = alignOpen ? 'TARGET OVERLAY: ON' : 'TARGET OVERLAY: OFF';
      btn.style.background = alignOpen ? '#003333' : '#222';
      document.getElementById('align-panel').style.display = alignOpen ? 'block' : 'none';
    }

    function toggleVisionMode(){
      staticMode = !staticMode;
      var btn = document.getElementById('vision-btn');
      btn.textContent = staticMode ? 'TRACKING: STATIC (CALIBRATION)' : 'TRACKING: MOTION (PLAY)';
      btn.style.background = staticMode ? '#330033' : '#222';
      fetch('/api/set_vision_mode', {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({static_mode: staticMode})});
    }

    function toggleRoiMask(){
      fetch('/api/toggle_roi', {method:'POST'}).then(r=>r.json()).then(d=>{
        var btn = document.getElementById('roi-toggle-btn');
        btn.textContent = d.state ? 'ROI: VISIBLE' : 'ROI: HIDDEN';
        btn.style.background = d.state ? '#222' : '#332200';
      });
    }

    function imgClick(e, side){
      if(!calOpen) return;
      var pts = calPts[side];
      if(pts.length >= 4) return;
      var img = e.currentTarget;
      var rect = img.getBoundingClientRect();
      pts.push([Math.round((e.clientX - rect.left) * CAM_W / rect.width), Math.round((e.clientY - rect.top) * CAM_H / rect.height)]);
      updateCal(side);
    }
    function resetCal(side){ calPts[side] = []; updateCal(side); }
    function updateCal(side){
      var pts = calPts[side];
      var pre = side === 'left' ? 'l' : 'r';
      document.getElementById(pre+'cnt').textContent = '('+pts.length+'/4)';
      var phtml = '';
      pts.forEach(function(p,i){ phtml += CORNER_LABELS[i]+': ('+p[0]+', '+p[1]+')<br>'; });
      document.getElementById(pre+'pts').innerHTML = phtml;
      if(pts.length === 4){ applyRoi(side); }
    }

    function applyRoi(side){
      var pts = calPts[side];
      var flat = pts.reduce(function(a,p){return a.concat(p);}, []);
      var body = {}; body[side] = flat;
      var st = document.getElementById((side==='left'?'l':'r')+'apply-status');
      st.style.color='#ff0'; st.textContent='applying...';
      fetch('/api/set_roi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
        .then(function(r){return r.json();}).then(function(res){
          if(res[side] && res[side].ok){ st.style.color='#0f0'; st.textContent='\u2713 applied'; }
          else { st.style.color='#f55'; st.textContent='\u2717 '+(res[side]?res[side].msg:'failed'); }
        }).catch(function(){ st.style.color='#f55'; st.textContent='\u2717 error'; });
    }

    function captureNetZ(){
      var el = document.getElementById('net-z-val');
      el.innerHTML = 'setting...';
      fetch('/api/stats').then(r=>r.json()).then(d=>{
        var z = d.z;
        fetch('/api/set_net_z', {
          method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({z: z})
        }).then(r=>r.json()).then(res=>{
          if(res.ok){
            el.innerHTML = '&#10003; net_z set to <code>'+z.toFixed(3)+'</code>';
            el.style.color='#0f0';
          } else {
            el.innerHTML = '&#10007; Failed';
            el.style.color='#f55';
          }
        });
      });
    }

    function onAngleSlider(){
      _panL = parseFloat(document.getElementById('pan-l-slider').value);
      _panR = parseFloat(document.getElementById('pan-r-slider').value);
      _tiltL = parseFloat(document.getElementById('tilt-l-slider').value);
      _tiltR = parseFloat(document.getElementById('tilt-r-slider').value);
      _hl = parseFloat(document.getElementById('height-l-slider').value);
      _hr = parseFloat(document.getElementById('height-r-slider').value);
      _rollL = parseFloat(document.getElementById('roll-l-slider').value);
      _rollR = parseFloat(document.getElementById('roll-r-slider').value);
      _net = parseFloat(document.getElementById('net-slider').value);
      _contrast = parseInt(document.getElementById('contrast-slider').value);

      document.getElementById('pan-l-val').textContent = _panL.toFixed(1);
      document.getElementById('pan-r-val').textContent = _panR.toFixed(1);
      document.getElementById('tilt-l-val').textContent = _tiltL.toFixed(1);
      document.getElementById('tilt-r-val').textContent = _tiltR.toFixed(1);
      document.getElementById('height-l-val').textContent = _hl.toFixed(2) + 'm';
      document.getElementById('height-r-val').textContent = _hr.toFixed(2) + 'm';
      document.getElementById('roll-l-val').textContent = _rollL.toFixed(1);
      document.getElementById('roll-r-val').textContent = _rollR.toFixed(1);
      document.getElementById('net-val').textContent = _net.toFixed(2) + 'm';
      document.getElementById('contrast-val').textContent = _contrast;

      fetch('/api/preview_angles', {method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({pl:_panL, pr:_panR, tl:_tiltL, tr:_tiltR, hl:_hl, hr:_hr, rl:_rollL, rr:_rollR, nd:_net})});
    }

    function applyAngles(){
      var st = document.getElementById('angle-status');
      st.style.color='#ff0'; st.textContent='applying to C++...';
      _contrast = parseInt(document.getElementById('contrast-slider').value);
      fetch('/api/set_angles',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({pl:_panL, pr:_panR, tl:_tiltL, tr:_tiltR, hl:_hl, hr:_hr, rl:_rollL, rr:_rollR, nd:_net, contrast:_contrast})})
        .then(r=>r.json()).then(res=>{
          if(res.ok){ st.style.color='#0f0'; st.textContent='\u2713 C++ updated!'; }
          else { st.style.color='#f55'; st.textContent='\u2717 '+(res.msg||'error'); }
        }).catch(()=>{ st.style.color='#f55'; st.textContent='\u2717 error'; });
    }
  </script>
</body>
</html>"""

# --- DRAWING FUNCTIONS ---
CAM_FX, CAM_FY, CAM_CX, CAM_CY = 448.2, 448.2, 320.0, 200.0  # fy=fx for OV9281 square pixels

def project_3d(x, y, z):
    if z <= 0.05: return None
    return (int(CAM_FX * x / z + CAM_CX), int(CAM_FY * y / z + CAM_CY))

def _dist3d(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

# MERGED FROM DOC 6: table_to_cam now takes (tx, ty, tz) so the relative vertical
# vector dy = ty - cam_y is correct. Previously dy was always set to ws.hl/ws.hr
# regardless of the 3D point's Y, which was wrong for anything not at camera height.
def draw_alignment_overlay(frame, side, ws):
    h, w = frame.shape[:2]

    def table_to_cam(tx, ty, tz):
        # World frame: tx=0 is table centre, ty=0 is table surface, tz=0 is net
        cam_x = -(ws.baseline / 2.0) if side == 'left' else (ws.baseline / 2.0)
        cam_y = ws.hl if side == 'left' else ws.hr   # camera height above table surface
        cam_z = -ws.net_dist                          # camera Z offset behind net

        # Vector from camera to world point
        dx = tx - cam_x
        dy = ty - cam_y   # KEY FIX: relative to camera height, not absolute height
        dz = tz - cam_z

        p = ws.pl if side == 'left' else -ws.pr
        t = ws.tl if side == 'left' else ws.tr
        r = ws.rl if side == 'left' else ws.rr

        # 1. Pan (Y-axis rotation)
        x1 = dx * math.cos(p) - dz * math.sin(p)
        z1 = dx * math.sin(p) + dz * math.cos(p)

        # 2. Tilt (X-axis rotation)
        y2 = dy * math.cos(t) - z1 * math.sin(t)
        z2 = dy * math.sin(t) + z1 * math.cos(t)

        # 3. Roll (Z-axis rotation)
        cx = x1 * math.cos(r) - y2 * math.sin(r)
        cy = x1 * math.sin(r) + y2 * math.cos(r)

        return cx, cy, z2

    half_w = 0.7625  # half table width (1.525m standard)
    depth  = 1.37    # half table length from net to baseline

    # Table surface points: ty=0 (table surface level)
    pts_3d = {
        'net_l':      table_to_cam(-half_w, 0, 0),
        'net_r':      table_to_cam( half_w, 0, 0),
        'base_l':     table_to_cam(-half_w, 0, depth),
        'base_r':     table_to_cam( half_w, 0, depth),
        'center_net': table_to_cam(0,       0, 0),
        'center_base':table_to_cam(0,       0, depth),
    }

    p = {k: project_3d(*v) for k, v in pts_3d.items()}

    def in_bounds(pt): return pt and -1000 <= pt[0] <= w+1000 and -1000 <= pt[1] <= h+1000
    def midpt(a, b):
        if a and b: return ((a[0]+b[0])//2, (a[1]+b[1])//2)

    # Net line (pink)
    if p['net_l'] and p['net_r'] and in_bounds(p['net_l']) and in_bounds(p['net_r']):
        cv2.line(frame, p['net_l'], p['net_r'], (255, 50, 255), 2)
        mid = midpt(p['net_l'], p['net_r'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['net_l'], pts_3d['net_r']):.2f}m",
                        (mid[0]-25, mid[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 50, 255), 1, cv2.LINE_AA)

    # Opponent baseline (yellow)
    if p['base_l'] and p['base_r'] and in_bounds(p['base_l']) and in_bounds(p['base_r']):
        cv2.line(frame, p['base_l'], p['base_r'], (255, 255, 50), 2)
        mid = midpt(p['base_l'], p['base_r'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['base_l'], pts_3d['base_r']):.2f}m",
                        (mid[0]-25, mid[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    # Sidelines (yellow)
    if p['net_l'] and p['base_l'] and in_bounds(p['net_l']) and in_bounds(p['base_l']):
        cv2.line(frame, p['net_l'], p['base_l'], (255, 255, 50), 2)
        mid = midpt(p['net_l'], p['base_l'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['net_l'], pts_3d['base_l']):.2f}m",
                        (mid[0]+5, mid[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 100), 1, cv2.LINE_AA)
    if p['net_r'] and p['base_r'] and in_bounds(p['net_r']) and in_bounds(p['base_r']):
        cv2.line(frame, p['net_r'], p['base_r'], (255, 255, 50), 2)
        mid = midpt(p['net_r'], p['base_r'])
        if mid:
            cv2.putText(frame, f"{_dist3d(pts_3d['net_r'], pts_3d['base_r']):.2f}m",
                        (mid[0]+5, mid[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 100), 1, cv2.LINE_AA)

    # Centre line (cyan)
    if p['center_net'] and p['center_base'] and in_bounds(p['center_net']) and in_bounds(p['center_base']):
        cv2.line(frame, p['center_net'], p['center_base'], (0, 255, 255), 1, cv2.LINE_AA)

    # Net centre cross + label
    if p['center_net'] and in_bounds(p['center_net']):
        cv2.drawMarker(frame, p['center_net'], (255, 50, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, "NET CENTER", (p['center_net'][0]+10, p['center_net'][1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 50, 255), 2, cv2.LINE_AA)

    # Baseline centre cross + label
    if p['center_base'] and in_bounds(p['center_base']):
        cv2.drawMarker(frame, p['center_base'], (255, 255, 50), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(frame, "OPPONENT BASELINE", (p['center_base'][0]+10, p['center_base'][1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 50), 2, cv2.LINE_AA)


def draw_trajectory_overlay(frame, trail, predicted, landing):
    h, w = frame.shape[:2]
    def in_bounds(pt): return pt and 0 <= pt[0] < w and 0 <= pt[1] < h

    proj = [project_3d(x, y, z) for x, y, z in trail]
    for i, pt in enumerate(proj):
        if not in_bounds(pt): continue
        alpha = (i + 1) / len(trail)
        cv2.circle(frame, pt, max(2, int(4 * alpha)), (int(255 * alpha), int(180 * alpha), 0), -1)
        if i > 0 and in_bounds(proj[i-1]):
            cv2.line(frame, proj[i-1], pt, (int(255 * alpha), int(180 * alpha), 0), 1)

    if predicted:
        pt = project_3d(*predicted)
        if in_bounds(pt):
            cv2.circle(frame, pt, 10, (0, 140, 255), 2)
            cv2.line(frame, (pt[0]-14, pt[1]), (pt[0]+14, pt[1]), (0, 140, 255), 2)
            cv2.line(frame, (pt[0], pt[1]-14), (pt[0], pt[1]+14), (0, 140, 255), 2)

    if landing:
        lpt = project_3d(landing[0], predicted[1] if predicted else 0.0, landing[1])
        if in_bounds(lpt):
            cv2.line(frame, (lpt[0]-8, lpt[1]-8), (lpt[0]+8, lpt[1]+8), (0, 255, 255), 2)
            cv2.line(frame, (lpt[0]+8, lpt[1]-8), (lpt[0]-8, lpt[1]+8), (0, 255, 255), 2)

def draw_axis_indicator(frame):
    h, w = frame.shape[:2]
    origin, length = (55, h - 55), 40
    cv2.rectangle(frame, (origin[0]-15, origin[1]-length-20), (origin[0]+length+20, origin[1]+15), (0,0,0), -1)
    for (dx, dy), color, lbl in [((length,0),(0,0,255),'X'), ((0,-length),(0,255,0),'Y'), ((length//2,-length//2),(255,0,0),'Z')]:
        tip = (origin[0] + dx, origin[1] + dy)
        cv2.arrowedLine(frame, origin, tip, color, 2, tipLength=0.3)
        cv2.putText(frame, lbl, (tip[0]+4, tip[1]+4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

# --- WEB SERVER ---
class WebStreamer:
    def __init__(self):
        self._frames = {'left': None, 'right': None}
        self._stats = {'x':None, 'y':None, 'z':None, 'px':None, 'py':None, 'pz':None, 'land_x':None, 'land_z':None,
                       'det_l_x':None, 'det_l_y':None, 'det_r_x':None, 'det_r_y':None}
        self._topic_data = {}

        self.show_align = False
        self.show_roi_mask = True

        self.baseline = CAL.get('baseline_m', 1.525)
        self.pl = math.radians(CAL.get('pan_left_deg', 15.0))
        self.pr = math.radians(CAL.get('pan_right_deg', 15.0))
        self.tl = math.radians(CAL.get('tilt_left_deg', 45.0))
        self.tr = math.radians(CAL.get('tilt_right_deg', 45.0))
        self.hl = CAL.get('height_left', 0.889)
        self.hr = CAL.get('height_right', 0.889)
        self.rl = math.radians(CAL.get('roll_left_deg', 0.0))
        self.rr = math.radians(CAL.get('roll_right_deg', 0.0))
        self.net_dist = CAL.get('net_dist_z', 0.52)

        self._app = Flask(__name__)
        logging.getLogger('werkzeug').setLevel(logging.ERROR)

        @self._app.route('/')
        def index(): return render_template_string(_HTML_PAGE)

        @self._app.route('/stream/<side>')
        def stream(side):
            def gen():
                while True:
                    if self._frames.get(side): yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + self._frames[side] + b'\r\n')
                    time.sleep(0.008)
            return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self._app.route('/api/stats')
        def stats(): return Response(json.dumps(self._stats), mimetype='application/json')

        @self._app.route('/api/topics')
        def topics():
            now = time.time()
            out = {}
            for name, d in list(self._topic_data.items()):
                age_ms = int((now - d['last_t']) * 1000) if d.get('last_t') else None
                out[name] = {'type': d['type'], 'age_ms': age_ms, 'values': d['values']}
            return Response(json.dumps(out), mimetype='application/json')

        @self._app.route('/api/toggle_align', methods=['POST'])
        def toggle_align():
            self.show_align = not self.show_align
            return Response(json.dumps({'ok':True}), mimetype='application/json')

        @self._app.route('/api/toggle_roi', methods=['POST'])
        def toggle_roi():
            self.show_roi_mask = not self.show_roi_mask
            return Response(json.dumps({'ok': True, 'state': self.show_roi_mask}), mimetype='application/json')

        @self._app.route('/api/set_vision_mode', methods=['POST'])
        def set_vision_mode():
            static_mode = request.get_json().get('static_mode', False)
            for node in ['/ball_detector_left', '/ball_detector_right']:
                try: subprocess.run(['ros2', 'param', 'set', node, 'static_mode', str(static_mode).lower()], timeout=3)
                except: pass
            return Response(json.dumps({'ok':True}), mimetype='application/json')

        @self._app.route('/api/set_roi', methods=['POST'])
        def set_roi():
            data = request.get_json()
            nodes = {'left': '/ball_detector_left', 'right': '/ball_detector_right'}
            results = {}
            for side, node in nodes.items():
                pts = data.get(side)
                if not pts or len(pts) != 8: continue
                val = '[' + ','.join(str(v) for v in pts) + ']'
                try:
                    r = subprocess.run(['ros2', 'param', 'set', node, 'table_roi', val], capture_output=True, text=True, timeout=8)
                    results[side] = {'ok': r.returncode == 0, 'msg': r.stdout.strip() or r.stderr.strip() or 'ok'}
                except subprocess.TimeoutExpired:
                    results[side] = {'ok': False, 'msg': f'timeout — is {node} running?'}
                except Exception as e:
                    results[side] = {'ok': False, 'msg': str(e)}
                else:
                    if results.get(side, {}).get('ok'): _save_roi_to_cal(side, pts)
            return Response(json.dumps(results), mimetype='application/json')

        @self._app.route('/api/set_net_z', methods=['POST'])
        def set_net_z():
            z = float(request.get_json().get('z', 0.0))
            result = subprocess.run(['ros2', 'param', 'set', '/trajectory_node', 'net_z', str(z)], capture_output=True, text=True, timeout=5)
            ok = result.returncode == 0
            return Response(json.dumps({'ok': ok, 'z': z, 'msg': result.stdout.strip() or result.stderr.strip()}), mimetype='application/json')

        @self._app.route('/api/config')
        def config():
            # MERGED: also expose intrinsics so JS ray-drawing uses real values
            return Response(json.dumps({
                'pan_left_deg':   CAL.get('pan_left_deg',  15.0),
                'pan_right_deg':  CAL.get('pan_right_deg', 15.0),
                'tilt_left_deg':  CAL.get('tilt_left_deg', 45.0),
                'tilt_right_deg': CAL.get('tilt_right_deg', 45.0),
                'height_left':    CAL.get('height_left', 0.889),
                'height_right':   CAL.get('height_right', 0.889),
                'roll_left_deg':  CAL.get('roll_left_deg', 0.0),
                'roll_right_deg': CAL.get('roll_right_deg', 0.0),
                'net_dist_z':     CAL.get('net_dist_z', 0.52),
                'min_contrast':   CAL.get('min_contrast', 25),
                'baseline_m':     CAL.get('baseline_m', 1.525),
                'fx':             CAL.get('fx', 448.2),
                'fy':             CAL.get('fy', 448.2),
                'cx':             CAL.get('cx', 320.0),
                'cy':             CAL.get('cy', 200.0),
            }), mimetype='application/json')

        @self._app.route('/api/preview_angles', methods=['POST'])
        def preview_angles():
            data = request.get_json()
            self.pl = math.radians(float(data.get('pl', 15.0)))
            self.pr = math.radians(float(data.get('pr', 15.0)))
            self.tl = math.radians(float(data.get('tl', 45.0)))
            self.tr = math.radians(float(data.get('tr', 45.0)))
            self.hl = float(data.get('hl', 0.889))
            self.hr = float(data.get('hr', 0.889))
            self.rl = math.radians(float(data.get('rl', 0.0)))
            self.rr = math.radians(float(data.get('rr', 0.0)))
            self.net_dist = float(data.get('nd', 0.52))
            return Response(json.dumps({'ok':True}), mimetype='application/json')

        @self._app.route('/api/set_angles', methods=['POST'])
        def set_angles():
            data = request.get_json()
            pl = float(data.get('pl', CAL.get('pan_left_deg', 15.0)))
            pr = float(data.get('pr', CAL.get('pan_right_deg', 15.0)))
            tl = float(data.get('tl', CAL.get('tilt_left_deg', 45.0)))
            tr = float(data.get('tr', CAL.get('tilt_right_deg', 45.0)))
            hl = float(data.get('hl', CAL.get('height_left', 0.889)))
            hr = float(data.get('hr', CAL.get('height_right', 0.889)))
            rl = float(data.get('rl', CAL.get('roll_left_deg', 0.0)))
            rr = float(data.get('rr', CAL.get('roll_right_deg', 0.0)))
            nd = float(data.get('nd', CAL.get('net_dist_z', 0.52)))
            contrast = int(data.get('contrast', CAL.get('min_contrast', 25)))

            msgs = []
            ok = True

            # Angle params to stereo_node
            for param, val in [('pan_left_deg', pl), ('pan_right_deg', pr),
                               ('tilt_left_deg', tl), ('tilt_right_deg', tr),
                               ('roll_left_deg', rl), ('roll_right_deg', rr)]:
                try:
                    r = subprocess.run(['ros2', 'param', 'set', '/stereo_node', param, str(val)],
                                       capture_output=True, text=True, timeout=3)
                    if r.returncode != 0: ok = False; msgs.append(r.stderr.strip())
                except: ok = False; msgs.append(f'timeout {param}')

            # MERGED FROM DOC 6: also push height and net distance to stereo_node
            avg_height = (hl + hr) / 2.0
            for param, val in [('height_m', avg_height), ('net_dist_m', nd)]:
                try:
                    subprocess.run(['ros2', 'param', 'set', '/stereo_node', param, str(val)], timeout=3)
                except: pass

            # Contrast to vision nodes
            for node in ['/ball_detector_left', '/ball_detector_right']:
                try: subprocess.run(['ros2', 'param', 'set', node, 'min_contrast', str(contrast)], timeout=3)
                except: pass

            # Tilt and height to trajectory node
            avg_tilt = (tl + tr) / 2.0
            try: subprocess.run(['ros2', 'param', 'set', '/trajectory_node', 'camera_tilt_deg', str(avg_tilt)], timeout=3)
            except: pass
            try: subprocess.run(['ros2', 'param', 'set', '/trajectory_node', 'table_y', str(avg_height)], timeout=3)
            except: pass

            # MERGED: use unified _save_all_to_cal
            if ok:
                _save_all_to_cal({
                    'pan_left_deg': pl, 'pan_right_deg': pr,
                    'tilt_left_deg': tl, 'tilt_right_deg': tr,
                    'height_left': hl, 'height_right': hr,
                    'roll_left_deg': rl, 'roll_right_deg': rr,
                    'net_dist_z': nd, 'min_contrast': contrast,
                })
            return Response(json.dumps({'ok': ok, 'msg': '; '.join(msgs) or 'ok'}), mimetype='application/json')

    def push_frame(self, f, s):
        _, j = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 70])
        self._frames[s] = j.tobytes()

    def start(self): threading.Thread(target=lambda: self._app.run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()

class SystemLauncher(threading.Thread):
    def run(self):
        print("🔄 Syncing and building workspace on Jetson A... (this may take a moment)")
        subprocess.run("bash /home/capstone-nano2/Sensors/tabletennistrainer_ws/sync_and_build.sh", shell=True)
        print("✅ Workspace synced! Launching ROS 2 nodes...")

        _roi_b = (' -p table_roi:="[' + ','.join(str(v) for v in CAL['table_roi_right']) + ']"') if CAL['table_roi_right'] else ''
        cmd_b = (
            "export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyc_b.xml\n"
            "source /opt/ros/humble/setup.bash && source /home/capstone-nano2/Sensors/tabletennistrainer_ws/install/setup.bash\n"
            "ros2 launch ttt_bringup camera_right.launch.py &\n"
            f"ros2 run ttt_vision vision_node --ros-args -r __node:=ball_detector_right -p camera_id:=right -p min_area:={CAL['min_area']} -p max_area:={CAL['max_area']} -p motion_threshold:={CAL['motion_threshold']} -p min_contrast:={CAL.get('min_contrast', 25)} -p dilate_iters:={CAL['dilate_iters']} -p edge_margin:={CAL['edge_margin']} {_roi_b} &\n"
            "wait"
        )
        with open('/tmp/lb.sh', 'w') as f: f.write(cmd_b)
        subprocess.Popen("bash /tmp/lb.sh", shell=True)

        _roi_a = (' -p table_roi:="[' + ','.join(str(v) for v in CAL['table_roi_left']) + ']"') if CAL['table_roi_left'] else ''
        cmd_a = (
            "echo \"<?xml version='1.0' encoding='UTF-8' ?><CycloneDDS xmlns='https://cdds.io/config'><Domain id='any'><General><Interfaces><NetworkInterface address='192.168.1.10'/></Interfaces></General></Domain></CycloneDDS>\" > /tmp/cyc_a.xml\n"
            "export ROS_DOMAIN_ID=42; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; export CYCLONEDDS_URI=file:///tmp/cyc_a.xml\n"
            f"v4l2-ctl -d /dev/video0 -c auto_exposure=1 -c exposure={CAL['exposure']} -c analogue_gain={CAL['analogue_gain']}\n"
            "source /opt/ros/humble/setup.bash && source /home/capstone-nano1/Sensors/tabletennistrainer_ws/install/setup.bash\n"
            "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 robot_base root --ros-args -r __node:=robot_base_to_root &\n"
            "ros2 run ttt_calibration tf_broadcaster_node --ros-args --params-file /home/capstone-nano1/Sensors/tabletennistrainer_ws/src/ttt_calibration/config/stereo_extrinsic.yaml &\n"
            "ros2 launch ttt_bringup camera_left.launch.py &\n"
            f"ros2 run ttt_vision vision_node --ros-args -r __node:=ball_detector_left -p camera_id:=left -p min_area:={CAL['min_area']} -p max_area:={CAL['max_area']} -p motion_threshold:={CAL['motion_threshold']} -p min_contrast:={CAL.get('min_contrast', 25)} -p dilate_iters:={CAL['dilate_iters']} -p edge_margin:={CAL['edge_margin']} {_roi_a} &\n"
            f"ros2 run ttt_stereo stereo_node --ros-args -p fx:={CAL['fx']} -p fy:={CAL['fy']} -p cx:={CAL['cx']} -p cy:={CAL['cy']} -p baseline_m:={CAL.get('baseline_m', 1.525)} -p max_sync_age_ms:={CAL['max_sync_age_ms']} -p pan_left_deg:={CAL.get('pan_left_deg', 15.0)} -p pan_right_deg:={CAL.get('pan_right_deg', 15.0)} -p tilt_left_deg:={CAL.get('tilt_left_deg', 45.0)} -p tilt_right_deg:={CAL.get('tilt_right_deg', 45.0)} -p roll_left_deg:={CAL.get('roll_left_deg', 0.0)} -p roll_right_deg:={CAL.get('roll_right_deg', 0.0)} &\n"
            f"ros2 run ttt_trajectory trajectory_node --ros-args -p lookahead_ms:={CAL['lookahead_ms']} -p min_samples:={CAL['min_samples']} -p max_samples:={CAL['max_samples']} -p gravity:={CAL['gravity']} -p camera_tilt_deg:={(CAL.get('tilt_left_deg', 45.0) + CAL.get('tilt_right_deg', 45.0)) / 2.0} -p table_y:={(CAL.get('height_left', 0.889) + CAL.get('height_right', 0.889)) / 2.0} -p restitution:={CAL['restitution']} -p net_z:={CAL['net_z']} &\n"
            "wait\n"
        )
        with open('/tmp/la.sh', 'w') as f: f.write(cmd_a)
        subprocess.run("scp /tmp/la.sh capstone-nano1@192.168.1.10:/tmp/la.sh", shell=True, stderr=subprocess.DEVNULL)
        subprocess.Popen("ssh -tt capstone-nano1@192.168.1.10 'bash /tmp/la.sh'", shell=True)

class RoiPoller(threading.Thread):
    def __init__(self):
        super().__init__(); self.daemon = True
        self.rois = {'left': [], 'right': []}

    def run(self):
        import re
        nodes = {'left': '/ball_detector_left', 'right': '/ball_detector_right'}
        while True:
            for side, node in nodes.items():
                try:
                    result = subprocess.run(['ros2', 'param', 'get', node, 'table_roi'], capture_output=True, text=True, timeout=3)
                    m = re.search(r'\[([0-9,\s]+)\]', result.stdout)
                    if m:
                        vals = list(map(int, m.group(1).split(',')))
                        if len(vals) >= 8: self.rois[side] = [(vals[i*2], vals[i*2+1]) for i in range(len(vals)//2)]
                except Exception: pass
            time.sleep(5)

_roi_poller = RoiPoller()

class ROSWorker(threading.Thread):
    def __init__(self, ws):
        super().__init__(); self.daemon = True; self.ws = ws
        self.trail = deque(maxlen=25); self.pred = None; self.land = None; self.dets = {'left':None, 'right':None}
        self._cam_count = {'left': 0, 'right': 0}; self._cam_t0 = {}; self._cam_fps = {'left': 0.0, 'right': 0.0}

    def _rec(self, topic, typ, values):
        self.ws._topic_data[topic] = {'type': typ, 'last_t': time.time(), 'values': values}

    def run(self):
        rclpy.init()
        self.n = Node('marty_dashboard')
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.n.create_subscription(CompressedImage, '/camera/left/compressed', lambda m: self.pf(m, 'left'), q)
        self.n.create_subscription(CompressedImage, '/camera/right/compressed', lambda m: self.pf(m, 'right'), q)
        self.n.create_subscription(PointStamped, '/ball_position_3d', self.cb_3d, q)
        self.n.create_subscription(PointStamped, '/ball_trajectory/predicted', self.cb_pred, q)
        self.n.create_subscription(PointStamped, '/ball_trajectory/landing', self.cb_land, q)
        self.n.create_subscription(BallDetection, '/ball_detection/left',
            lambda m: (self.dets.update({'left': m}),
                       self.ws._stats.update({'det_l_x': m.x if m.x >= 0 else None, 'det_l_y': m.y if m.x >= 0 else None}),
                       self._rec('/ball_detection/left', 'BallDetection', {'x': round(m.x, 1), 'y': round(m.y, 1), 'radius': round(getattr(m, 'radius', 0.0), 1), 'conf': round(getattr(m, 'confidence', 0.0), 3)})), q)
        self.n.create_subscription(BallDetection, '/ball_detection/right',
            lambda m: (self.dets.update({'right': m}),
                       self.ws._stats.update({'det_r_x': m.x if m.x >= 0 else None, 'det_r_y': m.y if m.x >= 0 else None}),
                       self._rec('/ball_detection/right', 'BallDetection', {'x': round(m.x, 1), 'y': round(m.y, 1), 'radius': round(getattr(m, 'radius', 0.0), 1), 'conf': round(getattr(m, 'confidence', 0.0), 3)})), q)
        rclpy.spin(self.n)

    def cb_3d(self, m):
        self.trail.append((m.point.x, m.point.y, m.point.z))
        self.ws._stats.update({'x': m.point.x, 'y': m.point.y, 'z': m.point.z})
        self._rec('/ball_position_3d', 'PointStamped', {'x': round(m.point.x, 3), 'y': round(m.point.y, 3), 'z': round(m.point.z, 3)})

    def cb_pred(self, m):
        self.pred = (m.point.x, m.point.y, m.point.z)
        self.ws._stats.update({'px': m.point.x, 'py': m.point.y, 'pz': m.point.z})
        self._rec('/ball_trajectory/predicted', 'PointStamped', {'x': round(m.point.x, 3), 'y': round(m.point.y, 3), 'z': round(m.point.z, 3)})

    def cb_land(self, m):
        self.land = (m.point.x, m.point.z)
        self.ws._stats.update({'land_x': m.point.x, 'land_z': m.point.z})
        self._rec('/ball_trajectory/landing', 'PointStamped', {'x': round(m.point.x, 3), 'z': round(m.point.z, 3)})

    def pf(self, msg, side):
        now = time.time()
        self._cam_count[side] += 1
        if side not in self._cam_t0: self._cam_t0[side] = now
        elif now - self._cam_t0[side] >= 1.0:
            self._cam_fps[side] = self._cam_count[side] / (now - self._cam_t0[side])
            self._cam_count[side] = 0
            self._cam_t0[side] = now
        self._rec(f'/camera/{side}/compressed', 'CompressedImage', {'fps': round(self._cam_fps[side], 1)})

        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)
        if frame is not None:
            if self.ws.show_align: draw_alignment_overlay(frame, side, self.ws)

            if self.ws.show_roi_mask:
                roi = _roi_poller.rois.get(side, [])
                if roi:
                    pts = np.array(roi, dtype=np.int32)
                    cv2.polylines(frame, [pts], True, (0, 200, 0), 2)

            det = self.dets.get(side)
            if det and getattr(det, 'confidence', 1) > 0:
                cx, cy = int(det.x), int(det.y)
                draw_r = max(int(getattr(det, 'radius', 0.0)) * 2, 20)
                cv2.circle(frame, (cx, cy), draw_r, (0, 0, 255), 3)
                h, w = frame.shape[:2]
                bx, by = max(0, min(cx, w - 1)), max(0, min(cy, h - 1))
                brightness = int(frame[by, bx, 0])
                label = f"r={getattr(det, 'radius', 0.0):.1f} circ={getattr(det, 'confidence', 0.0):.2f} bright={brightness}"
                label_y = max(cy - draw_r - 8, 14)
                cv2.putText(frame, label, (cx - 80, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

                sx, sy, sz = self.ws._stats.get('x'), self.ws._stats.get('y'), self.ws._stats.get('z')
                if sx is not None and sy is not None and sz is not None:
                    cv2.putText(frame, f"X:{sx:+.2f} Y:{sy:+.2f} Z:{sz:.2f}m", (cx - 80, label_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 255, 100), 1, cv2.LINE_AA)

            if side == 'left': draw_trajectory_overlay(frame, self.trail, self.pred, self.land)
            draw_axis_indicator(frame)
            self.ws.push_frame(frame, side)

def _cleanup():
    subprocess.run("pkill -f 'ros2 run'", shell=True, stderr=subprocess.DEVNULL)
    subprocess.run("ssh capstone-nano1@192.168.1.10 \"pkill -f 'ros2 run'\"", shell=True, stderr=subprocess.DEVNULL)

if __name__ == "__main__":
    import atexit; atexit.register(_cleanup)
    signal.signal(signal.SIGTERM, lambda *_: (_cleanup(), sys.exit(0)))
    _cleanup()

    streamer = WebStreamer()
    streamer.start()
    SystemLauncher().start()
    _roi_poller.start()
    ROSWorker(streamer).start()

    print("=======================================\n✅ FULL VISION & TELEMETRY ONLINE\nVIEW AT: http://192.168.55.1:5000\n=======================================")
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        _cleanup()
        sys.exit(0)