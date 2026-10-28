import cv2
import numpy as np
from pyzbar.pyzbar import decode
import serial
import threading
import time
import json
import os
from collections import deque
from flask import Flask, Response, request, jsonify, render_template_string

app = Flask(__name__)

# --- 1. CONFIGURATION & CALIBRATION ---
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200
MAP_FILE = 'hallway_map.json'

# Visual Servoing / Auto-Centering Calibration (for 640x480 frame)
FRAME_W = 640
FRAME_H = 480
CENTER_X = 320
TARGET_Y = 360        # Y-pixel coordinate where QR is right in front of bumper
TOLERANCE_X = 65      # Horizontal pixel tolerance for centering
TOLERANCE_Y = 45      # Vertical pixel tolerance for stopping
AXLE_CREEP_SEC = 0.45 # Time to creep forward so wheels sit over the QR code before turning

COMPASS = {"N": 0, "E": 90, "S": 180, "W": 270}

# --- 2. GLOBAL ROBOT STATE ---
state = {
    "current_node": "QR_START",
    "heading": "N",          # "N", "E", "S", "W"
    "status": "IDLE",        # IDLE, NAVIGATING, CENTERING, TURNING, OBSTACLE_PAUSED, DELIVERING
    "target_node": None,
    "planned_route": [],
    "last_scanned_qr": None,
    "obstacle_paused": False,
    "distance_cm": 999,
    "camera_ok": False,
    "camera_stats": "Not initialized",
    "turn_ms_90": 550
}

# Load or create initial map database
if os.path.exists(MAP_FILE):
    with open(MAP_FILE, 'r') as f:
        map_db = json.load(f)
else:
    map_db = {
        "QR_START": {"name": "Start Station", "exits": {"N": "QR_101", "E": "", "S": "", "W": ""}},
        "QR_101":   {"name": "Room 101",      "exits": {"N": "QR_102", "E": "", "S": "QR_START", "W": ""}},
        "QR_102":   {"name": "Room 102",      "exits": {"N": "",       "E": "", "S": "QR_101",   "W": ""}}
    }
    with open(MAP_FILE, 'w') as f:
        json.dump(map_db, f, indent=2)

def save_map():
    with open(MAP_FILE, 'w') as f:
        json.dump(map_db, f, indent=2)

# --- 3. SERIAL CONNECTION TO ESP32 ---
esp = None
serial_lock = threading.Lock()

def init_serial():
    global esp
    for port in [SERIAL_PORT, '/dev/ttyAMA0', '/dev/ttyS0']:
        try:
            esp = serial.Serial(port, BAUD_RATE, timeout=0.5)
            time.sleep(0.5)
            esp.reset_input_buffer()
            print(f"[SERIAL] Connected on {port}")
            return
        except Exception as e:
            print(f"[SERIAL] Failed {port}: {e}")

def send_cmd(cmd):
    if esp and esp.is_open:
        with serial_lock:
            try:
                esp.write(f"{cmd}\n".encode('utf-8'))
            except Exception as e:
                print(f"[SERIAL WRITE ERR]: {e}")

def serial_listener():
    while True:
        if esp and esp.is_open:
            try:
                if esp.in_waiting > 0:
                    line = esp.readline().decode('utf-8', errors='ignore').strip()
                    if line == "OBSTACLE_STOP":
                        state["obstacle_paused"] = True
                        state["status"] = "OBSTACLE_PAUSED (<15cm)"
                    elif line == "OBSTACLE_CLEAR":
                        state["obstacle_paused"] = False
                        state["status"] = "NAVIGATING (Resumed)"
                    elif line.startswith("DIST:"):
                        try:
                            state["distance_cm"] = int(line.split(":")[1])
                        except ValueError:
                            pass
            except Exception:
                pass
        time.sleep(0.03)

# --- 4. TROUBLESHOOTED CAMERA & QR ENGINE ---
cap = None
latest_annotated_frame = None
latest_qr_detection = None  # Stores {"id": str, "cx": int, "cy": int, "pts": list, "ts": float}
cam_lock = threading.Lock()

def open_camera():
    global cap
    if cap is not None:
        cap.release()
    # Try V4L2 indices 0 and 1 (prevents Pi Zero USB camera hangs)
    for idx in [0, 1]:
        c = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not c.isOpened():
            c = cv2.VideoCapture(idx)
        if c.isOpened():
            c.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
            c.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            c.set(cv2.CAP_PROP_FPS, 15)
            c.set(cv2.CAP_PROP_BUFFERSIZE, 1) # Prevents old frame lag!
            cap = c
            state["camera_ok"] = True
            print(f"[CAMERA] Opened index {idx}")
            return True
    state["camera_ok"] = False
    return False

def decode_qr_robust(frame):
    """
    Uses grayscale + CLAHE contrast boost + Adaptive Threshold fallback
    to handle dim indoor lighting on the 0.3MP webcam.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mean_lum = int(np.mean(gray))
    state["camera_stats"] = f"Res: {frame.shape[1]}x{frame.shape[0]} | Brightness: {mean_lum}/255"

    # Pass 1: Raw grayscale
    codes = decode(gray)
    if not codes:
        # Pass 2: CLAHE contrast enhancement (fixes dark/washed-out QR codes)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        codes = decode(enhanced)
    if not codes:
        # Pass 3: OpenCV QRCodeDetector fallback
        detector = cv2.QRCodeDetector()
        retval, decoded_info, points, _ = detector.detectAndDecodeMulti(gray)
        if retval and points is not None:
            for info, pts in zip(decoded_info, points):
                if info:
                    pts_int = pts.astype(int)
                    cx = int(np.mean(pts_int[:, 0]))
                    cy = int(np.mean(pts_int[:, 1]))
                    return {"id": info.strip(), "cx": cx, "cy": cy, "pts": pts_int}

    if codes:
        c = codes[0]
        qr_id = c.data.decode('utf-8', errors='ignore').strip()
        rect = c.rect
        cx = rect.left + rect.width // 2
        cy = rect.top + rect.height // 2
        pts = np.array([[p.x, p.y] for p in c.polygon], dtype=np.int32)
        return {"id": qr_id, "cx": cx, "cy": cy, "pts": pts}

    return None

def vision_worker():
    global latest_annotated_frame, latest_qr_detection
    open_camera()
    while True:
        if cap is None or not cap.isOpened():
            state["camera_ok"] = False
            open_camera()
            time.sleep(1.0)
            continue

        ret, frame = cap.read()
        if not ret or frame is None:
            state["camera_ok"] = False
            time.sleep(0.2)
            continue

        state["camera_ok"] = True
        qr_info = decode_qr_robust(frame)

        # Draw alignment HUD overlay on frame for web inspection
        vis = frame.copy()
        # Target centering box & crosshairs
        cv2.line(vis, (CENTER_X, 0), (CENTER_X, FRAME_H), (180, 180, 0), 1)
        cv2.line(vis, (0, TARGET_Y), (FRAME_W, TARGET_Y), (180, 180, 0), 1)
        cv2.rectangle(vis,
                      (CENTER_X - TOLERANCE_X, TARGET_Y - TOLERANCE_Y),
                      (CENTER_X + TOLERANCE_X, TARGET_Y + TOLERANCE_Y),
                      (255, 200, 0), 1)

        if qr_info:
            qr_id = qr_info["id"]
            qr_info["ts"] = time.time()
            latest_qr_detection = qr_info
            state["last_scanned_qr"] = qr_id

            # Auto-register newly discovered QR code into map_db so user can set directions!
            if qr_id not in map_db:
                map_db[qr_id] = {
                    "name": f"Node {qr_id}",
                    "exits": {"N": "", "E": "", "S": "", "W": ""}
                }
                save_map()

            # Draw green bounding polygon & center point
            if len(qr_info["pts"]) >= 4:
                cv2.polylines(vis, [qr_info["pts"]], True, (0, 255, 0), 3)
            cv2.circle(vis, (qr_info["cx"], qr_info["cy"]), 6, (0, 0, 255), -1)
            cv2.putText(vis, f"{qr_id} ({qr_info['cx']},{qr_info['cy']})",
                        (qr_info["cx"] - 40, max(25, qr_info["cy"] - 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        with cam_lock:
            latest_annotated_frame = vis

        time.sleep(0.04)

# --- 5. BFS PATHFINDING & TURN LOGIC ---
def compute_turn(current_dir, target_dir):
    curr_deg = COMPASS[current_dir]
    targ_deg = COMPASS[target_dir]
    diff = (targ_deg - curr_deg) % 360
    if diff == 180:
        return 180
    elif diff == 90:
        return 90
    elif diff == 270:
        return -90
    return 0

def bfs_shortest_path(start_node, goal_node):
    if start_node == goal_node:
        return []
    queue = deque([(start_node, [])])
    visited = {start_node}

    while queue:
        curr, path = queue.popleft()
        if curr not in map_db:
            continue
        for direction, neighbor in map_db[curr].get("exits", {}).items():
            if not neighbor or neighbor not in map_db:
                continue
            new_path = path + [{"from": curr, "to": neighbor, "dir": direction}]
            if neighbor == goal_node:
                return new_path
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, new_path))
    return None

# --- 6. AUTO-CENTERING & NAVIGATION CONTROLLER ---
nav_abort = False

def auto_center_on_qr():
    """
    Uses micro-pulses (PULSE_L, PULSE_R, PULSE_F, PULSE_B) to center the QR
    in the target zone before the axle creeps over the junction.
    """
    state["status"] = "CENTERING ON QR"
    send_cmd("S")
    time.sleep(0.15)

    for _ in range(18):  # Max 18 micro-adjustments
        if nav_abort:
            return
        while state["obstacle_paused"]:
            time.sleep(0.1)

        det = latest_qr_detection
        if not det or (time.time() - det["ts"] > 0.6):
            break

        dx = det["cx"] - CENTER_X
        dy = det["cy"] - TARGET_Y

        # 1. Fix Horizontal Offset first
        if abs(dx) > TOLERANCE_X:
            if dx > 0:
                send_cmd("PULSE_R")
            else:
                send_cmd("PULSE_L")
            time.sleep(0.25)
            continue

        # 2. Fix Vertical Distance Offset
        if abs(dy) > TOLERANCE_Y:
            if dy < 0:
                send_cmd("PULSE_F") # QR is too high in frame -> nudge forward
            else:
                send_cmd("PULSE_B") # QR is too low in frame -> nudge backward
            time.sleep(0.25)
            continue

        # Centered inside target box!
        break

    # Creep forward slightly so wheel axle sits directly over the QR code for turning
    send_cmd("F")
    time.sleep(AXLE_CREEP_SEC)
    send_cmd("S")
    time.sleep(0.2)

def execute_route_worker(route, target_node):
    global nav_abort
    nav_abort = False
    state["planned_route"] = route

    for step in route:
        if nav_abort:
            send_cmd("S")
            state["status"] = "IDLE (Aborted)"
            return

        required_dir = step["dir"]
        next_node = step["to"]

        # 1. Wait if obstacle is currently blocking
        while state["obstacle_paused"] and not nav_abort:
            time.sleep(0.1)

        # 2. Turn toward the required compass exit
        turn_angle = compute_turn(state["heading"], required_dir)
        if turn_angle != 0:
            state["status"] = f"TURNING {turn_angle}° TO {required_dir}"
            send_cmd(f"T:{turn_angle}")
            # Wait for turn to finish
            wait_time = (abs(turn_angle) / 90.0) * (state["turn_ms_90"] / 1000.0) + 0.4
            time.sleep(wait_time)
        state["heading"] = required_dir

        # 3. Drive forward until the next QR code is detected
        state["status"] = f"DRIVING {required_dir} -> {next_node}"
        send_cmd("F")

        arrived = False
        while not arrived and not nav_abort:
            # If paused by ultrasonic sensor (<15cm), wait indefinitely until obstacle clears!
            if state["obstacle_paused"]:
                time.sleep(0.1)
                continue

            det = latest_qr_detection
            if det and (time.time() - det["ts"] < 0.4):
                # If we see the expected next QR (or any new junction QR other than the one we just left)
                if det["id"] == next_node or det["id"] != state["current_node"]:
                    send_cmd("S")
                    state["current_node"] = det["id"]
                    auto_center_on_qr()
                    arrived = True
                    break
            time.sleep(0.04)

    if not nav_abort:
        # Target Reached! Open servo for 20 seconds
        send_cmd("S")
        state["status"] = f"ARRIVED AT {target_node}! OPENING LID (20s)"
        send_cmd("OPEN")
        for remaining in range(20, 0, -1):
            if nav_abort:
                break
            state["status"] = f"DELIVERING AT {target_node} (Closing in {remaining}s)"
            time.sleep(1.0)
        send_cmd("CLOSE")
        state["status"] = "IDLE (Delivery Complete)"
        state["target_node"] = None
        state["planned_route"] = []

# --- 7. WEB INTERFACE & API ROUTES ---
HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Autonomous Delivery Bot Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; background: #121826; color: #f3f4f6; margin: 0; padding: 15px; }
    h2, h3 { margin-top: 0; color: #38bdf8; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(340px, 1fr)); gap: 15px; }
    .card { background: #1f2937; padding: 15px; border-radius: 10px; border: 1px solid #374151; }
    .badge { display: inline-block; padding: 5px 10px; border-radius: 6px; font-weight: bold; background: #0284c7; }
    .btn { padding: 9px 14px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; color: white; background: #2563eb; margin: 3px; }
    .btn:hover { opacity: 0.9; }
    .btn-red { background: #dc2626; }
    .btn-green { background: #16a34a; }
    .btn-amber { background: #d97706; }
    input, select { padding: 7px; border-radius: 5px; border: 1px solid #4b5563; background: #111827; color: white; margin: 3px 0; }
    table { width: 100%; border-collapse: collapse; margin-top: 8px; font-size: 13px; }
    th, td { border: 1px solid #374151; padding: 6px; text-align: center; }
    th { background: #111827; }
    img.cam-box { width: 100%; max-width: 480px; border-radius: 8px; border: 2px solid #38bdf8; background: #000; }
  </style>
</head>
<body>
  <h2>🤖 Delivery Robot Dashboard (BFS + Auto-Center + Map Builder)</h2>
  <div class="grid">
    <!-- 1. STATUS & MANUAL POSE OVERRIDE -->
    <div class="card">
      <h3>1. Live Status & Manual Pose Override</h3>
      <p><b>State:</b> <span id="st_status" class="badge">LOADING...</span></p>
      <p><b>Current Place (QR):</b> <span id="st_node">-</span> | <b>Direction:</b> <span id="st_head">-</span></p>
      <p><b>Last Scanned QR:</b> <span id="st_qr" style="color:#4ade80;font-weight:bold;">None</span>
         <button class="btn btn-amber" onclick="useScannedAsCurrent()">Set Scanned QR as Current Place</button>
      </p>
      <hr style="border-color:#374151">
      <h4>Manual Change Place & Direction</h4>
      <label>Current Place:</label>
      <select id="manual_node"></select>
      <label>Direction:</label>
      <select id="manual_dir">
        <option value="N">North (N - 0°)</option>
        <option value="E">East (E - 90°)</option>
        <option value="S">South (S - 180°)</option>
        <option value="W">West (W - 270°)</option>
      </select>
      <button class="btn btn-green" onclick="setManualPose()">Update Bot Pose</button>
    </div>

    <!-- 2. CAMERA TEST & LIVE STREAM -->
    <div class="card">
      <h3>2. Camera Troubleshooting & Frame Tester</h3>
      <p id="cam_stats" style="font-size:13px;color:#9ca3af;">Checking camera...</p>
      <div style="margin-bottom:8px;">
        <button class="btn btn-green" onclick="captureTestFrame()">📸 Test Camera (Grab Current Frame)</button>
        <button class="btn" onclick="toggleLiveStream()">🎥 Toggle Live MJPEG Stream</button>
        <button class="btn btn-amber" onclick="resetCamera()">🔄 Reset Camera USB</button>
      </div>
      <img id="cam_view" class="cam-box" src="/api/snapshot" alt="Camera Frame">
    </div>

    <!-- 3. BFS TARGET NAVIGATION & MANUAL JOG -->
    <div class="card">
      <h3>3. BFS Auto-Delivery & Manual Controls</h3>
      <label>Select Target Destination:</label>
      <select id="target_select"></select>
      <button class="btn btn-green" onclick="startBfsDelivery()">🚀 Start BFS Delivery</button>
      <button class="btn btn-red" onclick="stopBot()">🛑 EMERGENCY STOP / ABORT</button>
      <p id="route_display" style="font-size:13px;color:#fbbf24;"></p>
      <hr style="border-color:#374151">
      <h4>Manual Hardware Test</h4>
      <div>
        <button class="btn" onclick="hwCmd('F')">⬆ Forward</button>
        <button class="btn" onclick="hwCmd('B')">⬇ Back</button>
        <button class="btn" onclick="hwCmd('T:-90')">↺ Left 90°</button>
        <button class="btn" onclick="hwCmd('T:90')">↻ Right 90°</button>
        <button class="btn btn-red" onclick="hwCmd('S')">⏹ Stop</button>
      </div>
      <div style="margin-top:6px;">
        <button class="btn btn-amber" onclick="hwCmd('OPEN')">🔓 Open Lid</button>
        <button class="btn btn-amber" onclick="hwCmd('CLOSE')">🔒 Close Lid</button>
        <button class="btn" onclick="hwCmd('BEEP')">🔊 Beep</button>
        <button class="btn" onclick="triggerCenter()">🎯 Test Auto-Center on QR</button>
      </div>
      <div style="margin-top:8px;">
        <label>90° Turn Timing (ms):</label>
        <input type="number" id="turn_ms" value="550" style="width:75px;">
        <button class="btn" onclick="updateTurnMs()">Save Turn Radius</button>
      </div>
    </div>

    <!-- 4. QR MAP & DIRECTION BUILDER -->
    <div class="card">
      <h3>4. QR Junction Map (Edit Directions Anytime)</h3>
      <p style="font-size:12px;color:#9ca3af;">Scan any QR code to auto-add it below, or type a new QR ID manually. Set which QR lies to the North, East, South, or West of each node.</p>
      <div>
        <input type="text" id="new_qr_id" placeholder="QR ID (e.g. QR_103)" style="width:120px;">
        <input type="text" id="new_qr_name" placeholder="Label (e.g. Room 103)" style="width:130px;">
        <button class="btn btn-green" onclick="addOrUpdateNode()">+ Add QR Node</button>
      </div>
      <div style="overflow-x:auto;">
        <table id="map_table">
          <thead>
            <tr><th>QR ID</th><th>Name</th><th>N (0°)</th><th>E (90°)</th><th>S (180°)</th><th>W (270°)</th><th>Save</th></tr>
          </thead>
          <tbody></tbody>
        </table>
      </div>
    </div>
  </div>

<script>
let knownNodesKey = "";
let liveMode = false;

async function fetchStatus() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();
    document.getElementById('st_status').innerText = data.state.status;
    document.getElementById('st_node').innerText = data.state.current_node;
    document.getElementById('st_head').innerText = data.state.heading;
    document.getElementById('st_qr').innerText = data.state.last_scanned_qr || "None";
    document.getElementById('cam_stats').innerText = (data.state.camera_ok ? "✅ Camera OK | " : "❌ Camera Error | ") + data.state.camera_stats;

    if (data.state.planned_route && data.state.planned_route.length > 0) {
      const steps = data.state.planned_route.map(s => `${s.from} --(${s.dir})--> ${s.to}`).join("  ➔  ");
      document.getElementById('route_display').innerText = "Active Route: " + steps;
    } else {
      document.getElementById('route_display').innerText = "";
    }

    const nodesKey = JSON.stringify(data.map);
    if (nodesKey !== knownNodesKey) {
      knownNodesKey = nodesKey;
      renderMapControls(data.map, data.state.current_node);
    }
  } catch (e) {}
}

function renderMapControls(mapObj, currentNode) {
  const mNode = document.getElementById('manual_node');
  const tSel = document.getElementById('target_select');
  const tbody = document.querySelector('#map_table tbody');
  mNode.innerHTML = "";
  tSel.innerHTML = "";
  tbody.innerHTML = "";

  for (const [id, info] of Object.entries(mapObj)) {
    mNode.innerHTML += `<option value="${id}" ${id===currentNode?'selected':''}>${id} (${info.name})</option>`;
    tSel.innerHTML += `<option value="${id}">${id} - ${info.name}</option>`;

    const ex = info.exits || {};
    tbody.innerHTML += `
      <tr>
        <td><b>${id}</b></td>
        <td><input id="nm_${id}" value="${info.name||''}" style="width:85px"></td>
        <td><input id="n_${id}" value="${ex.N||''}" style="width:65px" placeholder="QR_..."></td>
        <td><input id="e_${id}" value="${ex.E||''}" style="width:65px" placeholder="QR_..."></td>
        <td><input id="s_${id}" value="${ex.S||''}" style="width:65px" placeholder="QR_..."></td>
        <td><input id="w_${id}" value="${ex.W||''}" style="width:65px" placeholder="QR_..."></td>
        <td><button class="btn btn-green" onclick="saveRow('${id}')">💾</button></td>
      </tr>`;
  }
}

async function saveRow(id) {
  const payload = {
    id: id,
    name: document.getElementById('nm_'+id).value.trim(),
    exits: {
      N: document.getElementById('n_'+id).value.trim(),
      E: document.getElementById('e_'+id).value.trim(),
      S: document.getElementById('s_'+id).value.trim(),
      W: document.getElementById('w_'+id).value.trim()
    }
  };
  await fetch('/api/update_node', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(payload)});
  fetchStatus();
}

async function addOrUpdateNode() {
  const id = document.getElementById('new_qr_id').value.trim();
  const name = document.getElementById('new_qr_name').value.trim() || id;
  if (!id) return alert("Enter a QR ID");
  await fetch('/api/update_node', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({id, name, exits:{N:"",E:"",S:"",W:""}})});
  document.getElementById('new_qr_id').value = "";
  document.getElementById('new_qr_name').value = "";
  fetchStatus();
}

async function setManualPose() {
  const node = document.getElementById('manual_node').value;
  const heading = document.getElementById('manual_dir').value;
  await fetch('/api/set_pose', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({node, heading})});
  fetchStatus();
}

async function useScannedAsCurrent() {
  const qr = document.getElementById('st_qr').innerText;
  if (!qr || qr === "None") return alert("No QR scanned yet!");
  const heading = document.getElementById('manual_dir').value;
  await fetch('/api/set_pose', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({node: qr, heading})});
  fetchStatus();
}

function captureTestFrame() {
  liveMode = false;
  document.getElementById('cam_view').src = "/api/snapshot?t=" + new Date().getTime();
}

function toggleLiveStream() {
  liveMode = !liveMode;
  document.getElementById('cam_view').src = liveMode ? "/api/video_feed" : ("/api/snapshot?t=" + new Date().getTime());
}

async function resetCamera() {
  await fetch('/api/reset_camera', {method:'POST'});
  setTimeout(captureTestFrame, 800);
}

async function startBfsDelivery() {
  const target = document.getElementById('target_select').value;
  const res = await fetch('/api/navigate', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({target})});
  const data = await res.json();
  if (!data.ok) alert(data.msg);
  fetchStatus();
}

async function stopBot() {
  await fetch('/api/stop', {method:'POST'});
  fetchStatus();
}

async function hwCmd(cmd) {
  await fetch('/api/hw', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({cmd})});
}

async function triggerCenter() {
  await fetch('/api/center', {method:'POST'});
}

async function updateTurnMs() {
  const ms = parseInt(document.getElementById('turn_ms').value) || 550;
  await fetch('/api/set_turn_ms', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify({ms})});
}

setInterval(fetchStatus, 700);
fetchStatus();
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/api/status')
def api_status():
    return jsonify({"state": state, "map": map_db})

@app.route('/api/snapshot')
def api_snapshot():
    """Returns a single current JPEG frame for the 'Test Camera' button."""
    with cam_lock:
        if latest_annotated_frame is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "No Camera Frame Available", (120, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            _, jpg = cv2.imencode('.jpg', blank)
        else:
            _, jpg = cv2.imencode('.jpg', latest_annotated_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    return Response(jpg.tobytes(), mimetype='image/jpeg')

def gen_mjpeg():
    while True:
        with cam_lock:
            frame = latest_annotated_frame
        if frame is not None:
            _, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpg.tobytes() + b'\r\n')
        time.sleep(0.08)

@app.route('/api/video_feed')
def api_video_feed():
    return Response(gen_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/reset_camera', methods=['POST'])
def api_reset_camera():
    open_camera()
    return jsonify({"ok": True})

@app.route('/api/update_node', methods=['POST'])
def api_update_node():
    data = request.json
    qr_id = data.get("id", "").strip()
    if qr_id:
        map_db[qr_id] = {
            "name": data.get("name", qr_id),
            "exits": data.get("exits", {"N": "", "E": "", "S": "", "W": ""})
        }
        save_map()
    return jsonify({"ok": True})

@app.route('/api/set_pose', methods=['POST'])
def api_set_pose():
    data = request.json
    if "node" in data and data["node"]:
        state["current_node"] = data["node"]
    if "heading" in data and data["heading"] in COMPASS:
        state["heading"] = data["heading"]
    return jsonify({"ok": True})

@app.route('/api/hw', methods=['POST'])
def api_hw():
    cmd = request.json.get("cmd", "")
    send_cmd(cmd)
    return jsonify({"ok": True})

@app.route('/api/center', methods=['POST'])
def api_center():
    threading.Thread(target=auto_center_on_qr, daemon=True).start()
    return jsonify({"ok": True})

@app.route('/api/set_turn_ms', methods=['POST'])
def api_set_turn_ms():
    ms = int(request.json.get("ms", 550))
    state["turn_ms_90"] = ms
    send_cmd(f"SET_TURN_MS:{ms}")
    return jsonify({"ok": True})

@app.route('/api/stop', methods=['POST'])
def api_stop():
    global nav_abort
    nav_abort = True
    send_cmd("S")
    state["status"] = "IDLE (Stopped)"
    return jsonify({"ok": True})

@app.route('/api/navigate', methods=['POST'])
def api_navigate():
    target = request.json.get("target")
    if not target or target not in map_db:
        return jsonify({"ok": False, "msg": "Invalid target QR selected."})

    route = bfs_shortest_path(state["current_node"], target)
    if route is None:
        return jsonify({"ok": False, "msg": f"No BFS path found from {state['current_node']} to {target}! Check your N/E/S/W exits."})

    state["target_node"] = target
    threading.Thread(target=execute_route_worker, args=(route, target), daemon=True).start()
    return jsonify({"ok": True, "route": route})

if __name__ == '__main__':
    init_serial()
    threading.Thread(target=serial_listener, daemon=True).start()
    threading.Thread(target=vision_worker, daemon=True).start()
    print("[SERVER] Starting Web UI on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)