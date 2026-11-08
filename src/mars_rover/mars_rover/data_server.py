#!/usr/bin/env python3
"""Data server for the Mars rover.

Subscribes to every sensor/state topic and serves it over HTTP (stdlib only, no extra packages):

  http://localhost:8080/                 live dashboard (values, camera, SLAM map, drive buttons)
  /api/all                               everything in one JSON
  /api/scan  (?full=1 for all ranges)    lidar summary: closest, front/left/right/rear distance
  /api/imu                               orientation (roll/pitch/yaw), gyro, accelerometer
  /api/odom                              raw wheel odometry
  /api/fused_odom                        EKF output (wheel odom + IMU gyro)
  /api/camera                            camera metadata
  /api/map                               SLAM map metadata (size, explored area)
  /api/cmd_vel                           last velocity command sent to the rover
  /api/drive_mode                        auto | manual | stop
  /api/obstacle                          obstacle_avoider status (state, distances, tilt...)
  /api/rates                             measured wall-clock Hz and age of every topic
  /camera.png   /map.png                 images
  /api/mode?set=auto|manual|stop         switch the drive mode (403 if allow_control:=false)
  /api/drive?lin=0.3&ang=0.0             manual drive command (only acts in manual mode)

A CSV log is written to ~/mars_rover_logs/ (2 rows per second).
"""
import csv
import json
import math
import os
import struct
import threading
import time
import webbrowser
import zlib
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String


# ----------------------------------------------------------------- helpers
def num(v, nd=3):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return None
    return round(v, nd) if math.isfinite(v) else None


def quat_rpy_deg(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def png_bytes(arr):
    """Encode a uint8 HxW (grey) or HxWx3 (RGB) array as PNG without any imaging library."""
    arr = np.ascontiguousarray(arr, dtype=np.uint8)
    h, w = arr.shape[:2]
    colour_type = 0 if arr.ndim == 2 else 2
    raw = b''.join(b'\x00' + arr[i].tobytes() for i in range(h))

    def chunk(tag, data):
        c = struct.pack('>I', len(data)) + tag + data
        return c + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)

    return (b'\x89PNG\r\n\x1a\n'
            + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, colour_type, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 3))
            + chunk(b'IEND', b''))


DASHBOARD = """<!doctype html><html><head><meta charset="utf-8"><title>Mars rover control and data</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;background:#1b1210;color:#f0dccf;margin:0;padding:12px}
#top{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin-bottom:12px}
h1{font-size:20px;margin:0 8px 0 0}
.badge{padding:5px 10px;border-radius:14px;background:#3a2820;font-size:13px}
.auto{background:#1f6b2f}.manual{background:#9a5a12}.stop{background:#a01818}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}
.card{background:#2a1c17;border:1px solid #5a3a2c;border-radius:8px;padding:10px}
.card h2{font-size:14px;margin:0 0 8px;color:#ffb27a}
pre{margin:0;font-size:12px;white-space:pre-wrap;max-height:260px;overflow:auto}
img,canvas{max-width:100%;border-radius:6px;background:#000;display:block;margin:auto}
button{background:#7a3b1d;color:#fff;border:0;border-radius:6px;padding:10px 14px;font-size:15px;cursor:pointer;touch-action:none;user-select:none}
button:active{background:#c45a2a}
button.big{background:#a01818;font-weight:bold;padding:10px 22px}
.sel{outline:3px solid #7ff07f}
.dpad{display:grid;grid-template-columns:repeat(3,64px);grid-auto-rows:56px;gap:6px;justify-content:center;margin:10px 0}
.dpad button{padding:0;font-size:20px}
label{display:block;font-size:13px;margin:6px 0}
input[type=range]{width:100%}
.hint{font-size:12px;color:#c9a88f;margin-top:6px;line-height:1.5}
#toast{font-size:12px;color:#7ff07f;min-height:16px}
</style></head><body>
<div id="top">
  <h1>Mars rover</h1>
  <span id="mode" class="badge">mode: ?</span>
  <span id="state" class="badge">avoider: ?</span>
  <span id="conn" class="badge">connecting...</span>
  <button class="big" onclick="setMode('stop')">STOP (space)</button>
</div>
<div class="grid">
  <div class="card"><h2>Drive control</h2>
    <div>
      <button id="b-auto" onclick="setMode('auto')">AUTO (1)</button>
      <button id="b-manual" onclick="setMode('manual')">MANUAL (2)</button>
      <button id="b-stop" onclick="setMode('stop')">STOP</button>
    </div>
    <label>Speed: <b id="sv">0.30</b> m/s <input id="sp" type="range" min="0.1" max="0.6" step="0.05" value="0.3" oninput="speed=+this.value;sv.textContent=speed.toFixed(2)" onchange="this.blur()"></label>
    <label>Turn rate: <b id="tv">0.80</b> rad/s <input id="tr" type="range" min="0.2" max="1.5" step="0.1" value="0.8" oninput="turn=+this.value;tv.textContent=turn.toFixed(2)" onchange="this.blur()"></label>
    <div class="dpad">
      <button data-f="1" data-a="1">&#8598;</button><button data-f="1" data-a="0">&#9650;</button><button data-f="1" data-a="-1">&#8599;</button>
      <button data-f="0" data-a="1">&#9664;</button><button onclick="setMode('stop')">&#9632;</button><button data-f="0" data-a="-1">&#9654;</button>
      <button data-f="-1" data-a="1">&#8601;</button><button data-f="-1" data-a="0">&#9660;</button><button data-f="-1" data-a="-1">&#8600;</button>
    </div>
    <div class="hint">Hold a button, or use the keyboard: W/A/S/D or arrow keys to drive (hold), Space = STOP, 1 = AUTO, 2 = MANUAL.
    Pressing a drive key while in AUTO switches to MANUAL for you.</div>
    <div id="toast"></div>
  </div>
  <div class="card"><h2>Camera</h2><img id="cam" src="/camera.png"></div>
  <div class="card"><h2>SLAM map (builds as the rover moves)</h2><img id="map" src="/map.png"></div>
  <div class="card"><h2>Lidar radar (top view, forward = up)</h2><canvas id="radar" width="360" height="360"></canvas></div>
  <div class="card"><h2>Obstacle avoider</h2><pre id="obstacle"></pre></div>
  <div class="card"><h2>Lidar summary</h2><pre id="scan"></pre></div>
  <div class="card"><h2>IMU</h2><pre id="imu"></pre></div>
  <div class="card"><h2>Fused odometry (EKF)</h2><pre id="fused_odom"></pre></div>
  <div class="card"><h2>Raw wheel odometry</h2><pre id="odom"></pre></div>
  <div class="card"><h2>Velocity command</h2><pre id="cmd_vel"></pre></div>
  <div class="card"><h2>Map info</h2><pre id="map_info"></pre></div>
  <div class="card"><h2>Topic rates (wall-clock Hz)</h2><pre id="rates"></pre></div>
</div>
<script>
let BASE = '';
let speed = 0.3, turn = 0.8, mode = '?', driveTimer = null, keys = new Set();
const $ = id => document.getElementById(id);
function toast(t){ $('toast').textContent = t; }


function setMode(m){
  fetch(BASE + '/api/mode?set=' + m).then(r => {
    if (r.status === 403){ toast('control is disabled on this server (read-only)'); return; }
    mode = m; showMode(); toast('mode set to ' + m);
  }).catch(() => toast('server not reachable'));
}
function showMode(){
  $('mode').textContent = 'mode: ' + mode;
  $('mode').className = 'badge ' + (['auto','manual','stop'].includes(mode) ? mode : '');
  for (const m of ['auto','manual','stop']) $('b-' + m).classList.toggle('sel', m === mode);
}
function sendDrive(f, a){ fetch(BASE + '/api/drive?lin=' + (f * speed) + '&ang=' + (a * turn)).catch(() => {}); }
function setDrive(f, a){
  if (driveTimer){ clearInterval(driveTimer); driveTimer = null; }
  if (!f && !a){ sendDrive(0, 0); return; }
  if (mode !== 'manual') setMode('manual');
  sendDrive(f, a);
  driveTimer = setInterval(() => sendDrive(f, a), 150);
}
// on-screen buttons
document.querySelectorAll('.dpad button[data-f]').forEach(b => {
  const f = +b.dataset.f, a = +b.dataset.a;
  b.addEventListener('pointerdown', e => { e.preventDefault(); setDrive(f, a); });
  for (const ev of ['pointerup','pointerleave','pointercancel']) b.addEventListener(ev, () => setDrive(0, 0));
});
// keyboard
function keyDrive(){
  const f = (keys.has('w') || keys.has('arrowup') ? 1 : 0) - (keys.has('s') || keys.has('arrowdown') ? 1 : 0);
  const a = (keys.has('a') || keys.has('arrowleft') ? 1 : 0) - (keys.has('d') || keys.has('arrowright') ? 1 : 0);
  setDrive(f, a);
}
const DRIVE_KEYS = ['w','a','s','d','arrowup','arrowdown','arrowleft','arrowright'];
addEventListener('keydown', e => {
  const k = e.key.toLowerCase();
  if (k === ' ' ){ e.preventDefault(); keys.clear(); if (driveTimer){ clearInterval(driveTimer); driveTimer = null; } setMode('stop'); return; }
  if (k === '1'){ keys.clear(); setDrive(0, 0); setMode('auto'); return; }
  if (k === '2'){ setMode('manual'); return; }
  if (DRIVE_KEYS.includes(k)){ e.preventDefault(); if (!keys.has(k)){ keys.add(k); keyDrive(); } }
});
addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (DRIVE_KEYS.includes(k)){ keys.delete(k); keyDrive(); }
});
addEventListener('blur', () => { keys.clear(); setDrive(0, 0); });

// lidar radar
function drawRadar(ranges){
  const c = $('radar'), g = c.getContext('2d'), S = c.width, cx = S / 2, cy = S / 2, R = S / 2 - 12, MAXR = 12;
  g.clearRect(0, 0, S, S); g.fillStyle = '#0b0807'; g.fillRect(0, 0, S, S);
  g.strokeStyle = '#4a3a30'; g.fillStyle = '#8a7568'; g.font = '10px sans-serif';
  for (const r of [2, 4, 6, 8, 10, 12]){ g.beginPath(); g.arc(cx, cy, r / MAXR * R, 0, 7); g.stroke(); g.fillText(r + 'm', cx + 2, cy - r / MAXR * R + 10); }
  g.beginPath(); g.moveTo(cx, cy - R); g.lineTo(cx, cy + R); g.moveTo(cx - R, cy); g.lineTo(cx + R, cy); g.stroke();
  if (ranges){
    const n = ranges.length;
    for (let i = 0; i < n; i++){
      const r = ranges[i]; if (r === null || r === undefined) continue;
      const ang = -Math.PI + i * 2 * Math.PI / n;
      const x = cx - r * Math.sin(ang) / MAXR * R, y = cy - r * Math.cos(ang) / MAXR * R;
      g.fillStyle = r < 1 ? '#ff4040' : (r < 2 ? '#ffa030' : '#50e070');
      g.fillRect(x - 1.5, y - 1.5, 3, 3);
    }
  }
  g.fillStyle = '#ffb27a'; g.beginPath(); g.moveTo(cx, cy - 10); g.lineTo(cx - 6, cy + 7); g.lineTo(cx + 6, cy + 7); g.closePath(); g.fill();
}
drawRadar(null);

async function tick(){
  try {
    const d = await (await fetch(BASE + '/api/all?full=1')).json();
    $('conn').textContent = 'connected'; $('conn').className = 'badge auto';
    if (d.drive_mode && d.drive_mode !== 'unknown'){ mode = d.drive_mode; showMode(); }
    const hide = (k, v) => (k === 'ranges' || k === 'ranges_downsampled') ? undefined : v;
    for (const k of ['obstacle','scan','imu','fused_odom','odom','cmd_vel','rates']){
      $(k).textContent = d[k] ? JSON.stringify(d[k], hide, 1) : 'waiting for data...';
    }
    $('map_info').textContent = d.map ? JSON.stringify(d.map, null, 1) : 'waiting for /map...';
    $('state').textContent = 'avoider: ' + (d.obstacle ? d.obstacle.state : '?');
    drawRadar(d.scan ? d.scan.ranges : null);
  } catch (e) { $('conn').textContent = 'no connection'; $('conn').className = 'badge stop'; }
}
function refreshImages(){ const t = Date.now(); $('cam').src = BASE + '/camera.png?' + t; $('map').src = BASE + '/map.png?' + t; }
setInterval(tick, 500); setInterval(refreshImages, 1000); tick(); refreshImages();
</script></body></html>"""


# ------------------------------------------------------------------- node
class DataServer(Node):
    def __init__(self):
        super().__init__('data_server')
        for k, v in {'host': '127.0.0.1', 'port': 8080, 'log_csv': True,
                     'log_dir': os.path.expanduser('~/mars_rover_logs'),
                     'max_manual_speed': 0.6, 'open_browser': True,
                     'allow_control': True}.items():
            self.declare_parameter(k, v)

        self.data = {}                 # name -> latest dict
        self.counts, self.prev_counts, self.hz, self.seen = {}, {}, {}, {}
        self.cam = None               # latest camera frame (numpy)
        self.map = None               # latest map (numpy int8)
        self.drive_mode = 'unknown'
        self.web_cmd, self.web_until, self.web_active = (0.0, 0.0), 0.0, False
        self.lock = threading.Lock()

        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self.on_odom('odom', m), 10)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.on_odom('fused_odom', m), 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(String, '/drive_mode_status', self.on_mode, 10)
        self.create_subscription(String, '/obstacle_status', self.on_obstacle, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))

        self.mode_pub = self.create_publisher(String, '/drive_mode', 10)
        self.manual_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)
        self.create_timer(0.1, self.web_drive_tick)
        self.create_timer(2.0, self.update_rates)

        self.csv_file = self.csv = None
        if self.get_parameter('log_csv').value:
            self.open_log()
            self.create_timer(0.5, self.log_row)

        self.start_http()

    # ------------------------------------------------------------ bookkeeping
    def tick(self, name):
        self.counts[name] = self.counts.get(name, 0) + 1
        self.seen[name] = time.time()

    def update_rates(self):
        for name, c in self.counts.items():
            self.hz[name] = round((c - self.prev_counts.get(name, 0)) / 2.0, 1)
            self.prev_counts[name] = c

    def rates(self):
        now = time.time()
        return {n: {'hz': self.hz.get(n, 0.0), 'age_s': num(now - t, 1), 'messages': self.counts[n]}
                for n, t in self.seen.items()}

    # --------------------------------------------------------------- callbacks
    def on_scan(self, msg):
        self.tick('scan')
        r = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min)

        def sector(lo, hi):
            m = ok & (ang >= math.radians(lo)) & (ang <= math.radians(hi))
            return num(r[m].min(), 2) if m.any() else None

        closest = closest_ang = None
        if ok.any():
            i = int(np.argmin(np.where(ok, r, np.inf)))
            closest, closest_ang = num(r[i], 2), num(math.degrees(ang[i]), 1)
        rear_m = ok & ((ang >= math.radians(160)) | (ang <= -math.radians(160)))
        self.data['scan'] = {
            'frame_id': msg.header.frame_id, 'samples': int(r.size),
            'range_min_m': num(msg.range_min, 2), 'range_max_m': num(msg.range_max, 2),
            'closest_m': closest, 'closest_angle_deg': closest_ang,
            'front_m': sector(-20, 20), 'left_m': sector(20, 100), 'right_m': sector(-100, -20),
            'rear_m': num(r[rear_m].min(), 2) if rear_m.any() else None,
            'ranges_downsampled': [num(v, 2) for v in r[::5]],
            '_full': [num(v, 2) for v in r],
        }

    def on_imu(self, msg):
        self.tick('imu')
        q = msg.orientation
        roll, pitch, yaw = quat_rpy_deg(q.x, q.y, q.z, q.w)
        self.data['imu'] = {
            'frame_id': msg.header.frame_id,
            'roll_deg': num(roll, 2), 'pitch_deg': num(pitch, 2), 'yaw_deg': num(yaw, 1),
            'gyro_rad_s': {'x': num(msg.angular_velocity.x, 4), 'y': num(msg.angular_velocity.y, 4),
                           'z': num(msg.angular_velocity.z, 4)},
            'accel_m_s2': {'x': num(msg.linear_acceleration.x, 3), 'y': num(msg.linear_acceleration.y, 3),
                           'z': num(msg.linear_acceleration.z, 3)},
        }

    def on_odom(self, name, msg):
        self.tick(name)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        _, _, yaw = quat_rpy_deg(q.x, q.y, q.z, q.w)
        self.data[name] = {
            'frame_id': msg.header.frame_id, 'child_frame_id': msg.child_frame_id,
            'x_m': num(p.x, 3), 'y_m': num(p.y, 3), 'yaw_deg': num(yaw, 1),
            'linear_speed_m_s': num(msg.twist.twist.linear.x, 3),
            'yaw_rate_rad_s': num(msg.twist.twist.angular.z, 3),
        }

    def on_image(self, msg):
        self.tick('camera')
        if msg.encoding not in ('rgb8', 'bgr8') or msg.width == 0:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            arr = arr[..., ::-1]
        if self.counts['camera'] % 3 == 1 or self.cam is None:     # keep every 3rd frame
            self.cam = arr[::2, ::2].copy()
        self.data['camera'] = {'frame_id': msg.header.frame_id, 'width': msg.width,
                               'height': msg.height, 'encoding': msg.encoding}

    def on_map(self, msg):
        self.tick('map')
        h, w = msg.info.height, msg.info.width
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        self.map = grid
        res = msg.info.resolution
        free, occ = int((grid == 0).sum()), int((grid > 50).sum())
        self.data['map'] = {
            'width_cells': w, 'height_cells': h, 'resolution_m': num(res, 3),
            'size_m': [num(w * res, 1), num(h * res, 1)],
            'origin_xy_m': [num(msg.info.origin.position.x, 2), num(msg.info.origin.position.y, 2)],
            'free_cells': free, 'occupied_cells': occ,
            'explored_area_m2': num((free + occ) * res * res, 1),
        }

    def on_cmd(self, msg):
        self.tick('cmd_vel')
        self.data['cmd_vel'] = {'linear_m_s': num(msg.linear.x, 3), 'angular_rad_s': num(msg.angular.z, 3)}

    def on_mode(self, msg):
        self.tick('drive_mode')
        self.drive_mode = msg.data

    def on_obstacle(self, msg):
        self.tick('obstacle')
        try:
            self.data['obstacle'] = json.loads(msg.data)
        except ValueError:
            pass

    # ---------------------------------------------------------- web control
    def set_mode(self, mode):
        if mode not in ('auto', 'manual', 'stop'):
            return False
        self.mode_pub.publish(String(data=mode))
        return True

    def web_drive(self, lin, ang):
        lim = float(self.get_parameter('max_manual_speed').value)
        self.web_cmd = (max(-lim, min(lim, lin)), max(-1.5, min(1.5, ang)))
        self.web_until = time.time() + 0.6

    def web_drive_tick(self):
        if time.time() < self.web_until:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = self.web_cmd
            self.manual_pub.publish(cmd)
            self.web_active = True
        elif self.web_active:
            self.manual_pub.publish(Twist())
            self.web_active = False

    # ------------------------------------------------------------- snapshots
    def snapshot(self, full=False):
        out = {}
        for k, v in self.data.items():
            d = {kk: vv for kk, vv in v.items() if kk != '_full'} if isinstance(v, dict) else v
            if full and k == 'scan':
                d['ranges'] = v['_full']
            out[k] = d
        out['drive_mode'] = self.drive_mode
        out['rates'] = self.rates()
        out['time'] = datetime.now().isoformat(timespec='seconds')
        return out

    def map_png(self):
        if self.map is None:
            return None
        g = self.map
        img = np.full(g.shape, 205, dtype=np.uint8)      # unknown = grey
        img[g == 0] = 254                                # free = white
        img[g > 50] = 0                                  # occupied = black
        img = np.flipud(img)                             # map y points up
        f = max(1, 420 // max(img.shape))
        if f > 1:
            img = np.repeat(np.repeat(img, f, axis=0), f, axis=1)
        step = max(1, max(img.shape) // 900)
        return png_bytes(img[::step, ::step])

    def camera_png(self):
        return None if self.cam is None else png_bytes(self.cam)

    # ----------------------------------------------------------------- CSV
    def open_log(self):
        try:
            d = self.get_parameter('log_dir').value
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, datetime.now().strftime('rover_%Y%m%d_%H%M%S.csv'))
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(['time_s', 'mode', 'state', 'x', 'y', 'yaw_deg', 'speed', 'yaw_rate',
                               'roll_deg', 'pitch_deg', 'front_m', 'left_m', 'right_m', 'closest_m',
                               'cam_rock_fraction', 'map_explored_m2'])
            self.get_logger().info(f'logging data to {path}')
        except OSError as e:
            self.get_logger().warn(f'cannot open log file: {e}')
            self.csv = None

    def log_row(self):
        if self.csv is None or 'fused_odom' not in self.data and 'odom' not in self.data:
            return
        od = self.data.get('fused_odom') or self.data.get('odom') or {}
        imu, sc = self.data.get('imu', {}), self.data.get('scan', {})
        ob, mp = self.data.get('obstacle', {}), self.data.get('map', {})
        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv.writerow([round(t, 2), self.drive_mode, ob.get('state'),
                           od.get('x_m'), od.get('y_m'), od.get('yaw_deg'),
                           od.get('linear_speed_m_s'), od.get('yaw_rate_rad_s'),
                           imu.get('roll_deg'), imu.get('pitch_deg'),
                           sc.get('front_m'), sc.get('left_m'), sc.get('right_m'), sc.get('closest_m'),
                           ob.get('cam_rock_fraction'), mp.get('explored_area_m2')])
        self.csv_file.flush()

    # ---------------------------------------------------------------- HTTP
    def start_http(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def reply(self, code, body, ctype):
                if isinstance(body, str):
                    body = body.encode()
                self.send_response(code)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def js(self, obj, code=200):
                self.reply(code, json.dumps(obj), 'application/json')

            def do_OPTIONS(self):
                self.send_response(204)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', '*')
                self.send_header('Access-Control-Allow-Private-Network', 'true')
                self.send_header('Content-Length', '0')
                self.end_headers()

            def do_GET(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                p = u.path.rstrip('/') or '/'
                try:
                    if p == '/':
                        return self.reply(200, DASHBOARD, 'text/html; charset=utf-8')
                    if p == '/camera.png' or p == '/map.png':
                        png = node.camera_png() if p == '/camera.png' else node.map_png()
                        if png is None:
                            return self.reply(404, 'no data yet', 'text/plain')
                        return self.reply(200, png, 'image/png')
                    if p in ('/api/mode', '/api/drive') and not node.get_parameter('allow_control').value:
                        return self.js({'error': 'control is disabled on this server (read-only)'}, 403)
                    if p == '/api/mode':
                        ok = node.set_mode(q.get('set', [''])[0].lower())
                        return self.js({'ok': ok, 'requested': q.get('set', [''])[0]}, 200 if ok else 400)
                    if p == '/api/drive':
                        node.web_drive(float(q.get('lin', ['0'])[0]), float(q.get('ang', ['0'])[0]))
                        return self.js({'ok': True, 'note': 'only acts while drive mode is manual'})
                    if p.startswith('/api/'):
                        snap = node.snapshot(full=q.get('full', ['0'])[0] == '1')
                        key = p[5:]
                        if key == 'all':
                            return self.js(snap)
                        if key in snap:
                            return self.js(snap[key])
                        return self.js({'error': f'no data for "{key}" yet', 'available': list(snap)}, 404)
                    return self.reply(404, 'not found', 'text/plain')
                except ValueError:
                    return self.js({'error': 'bad number in query'}, 400)

        host, port = self.get_parameter('host').value, int(self.get_parameter('port').value)
        try:
            self.httpd = ThreadingHTTPServer((host, port), Handler)
        except OSError as e:
            self.get_logger().error(f'cannot start web server on {host}:{port}: {e}')
            self.httpd = None
            return
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        url = f'http://{"localhost" if host in ("127.0.0.1", "0.0.0.0") else host}:{port}/'
        self.get_logger().info(f'DATA SERVER RUNNING -> open {url} in a browser')
        if self.get_parameter('open_browser').value:
            threading.Timer(2.0, lambda: webbrowser.open(url)).start()

    def shutdown_http(self):
        if getattr(self, 'httpd', None):
            self.httpd.shutdown()
        if self.csv_file:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = DataServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_http()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
