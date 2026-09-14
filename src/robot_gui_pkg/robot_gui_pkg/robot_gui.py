"""
ABU Robocon 2026 — Meihua Forest Planner (Tkinter + ROS2)
A* Pathfinding untuk R2 Autonomous Navigation
─────────────────────────────────────────────
Jalankan TANPA ROS2 (GUI only):
    python meihua_tkinter.py

Jalankan DENGAN ROS2:
    python meihua_tkinter.py --ros

Logika Start:
  • R2 selalu START di grid (0,1) — entry point Meihua Forest
  • Robot MENUNGGU sinyal bahwa ia sudah naik ke grid (0,1)
  • Setelah konfirmasi posisi → A* dijalankan & path dikirim ke ROS2

ROS2 Topics:
  Publish:
    /meihua/path_steps   (std_msgs/String)  — Full path JSON
    /meihua/next_step    (std_msgs/String)  — Step berikutnya
    /meihua/status       (std_msgs/String)  — Status robot
  Subscribe:
    /meihua/r2_position  (std_msgs/String)  — {"row":0,"col":1}
    /meihua/r2_arrived   (std_msgs/String)  — Konfirmasi di entry
"""

import tkinter as tk
from tkinter import messagebox, scrolledtext
from collections import defaultdict
import heapq
import threading
import json
import sys
import time

# ─── ROS2 optional import ────────────────────────────────────────────────────
USE_ROS = "--no-ros" not in sys.argv
ros_ok  = False

if USE_ROS:
    try:
        import rclpy
        import rclpy.signals
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_msgs.msg import Bool
        from rclpy.qos import QoSProfile, DurabilityPolicy
        try:
            from mehua_pkg_msgs.msg import KFSDetectionArray
            kfs_msg_ok = True
        except ImportError:
            kfs_msg_ok = False
            print("[WARN] mehua_pkg_msgs tidak ditemukan, KFS detection dinonaktifkan")
        ros_ok = True
    except ImportError:
        print("[WARN] rclpy tidak ditemukan. Jalankan: pip install rclpy")
        ros_ok = False

# ─── Konfigurasi Grid ────────────────────────────────────────────────────────
GRID_HEIGHTS = [
    [400, 200, 400],
    [200, 400, 600],
    [400, 600, 400],
    [200, 400, 200],
]
ROWS = 4
COLS = 3

# R2 selalu masuk Meihua Forest dari titik ini (row=0, col=1, height=200mm)
R2_ENTRY = (0, 1)

CELL_W = 100
CELL_H = 82
PAD    = 9

# ─── Warna ───────────────────────────────────────────────────────────────────
BG       = "#0d1117"
PANEL_BG = "#0f1923"
BORDER   = "#1e3a2a"
ACCENT   = "#4ade80"

HEIGHT_COLOR = {
    200: {"bg": "#1a3a1a", "border": "#2d6e2d", "label": "#4ade80"},
    400: {"bg": "#1a2a3a", "border": "#2d5a8e", "label": "#60a5fa"},
    600: {"bg": "#2a1a3a", "border": "#6e2d8e", "label": "#c084fc"},
}
KFS_COLOR = {
    "R1":    {"bg": "#7c2d12", "border": "#f97316", "text": "#fed7aa"},
    "R2":    {"bg": "#1e3a5f", "border": "#3b82f6", "text": "#bfdbfe"},
    "FAKE":  {"bg": "#3b1f1f", "border": "#ef4444", "text": "#fecaca"},
    "EMPTY": {"bg": "",        "border": "",         "text": ""},
}

STATUS_WAITING = "WAITING"
STATUS_READY   = "READY"
STATUS_RUNNING = "RUNNING"
STATUS_DONE    = "DONE"
STATUS_ERROR   = "ERROR"
STATUS_COLOR   = {
    STATUS_WAITING: "#f59e0b",
    STATUS_READY:   "#4ade80",
    STATUS_RUNNING: "#3b82f6",
    STATUS_DONE:    "#a855f7",
    STATUS_ERROR:   "#ef4444",
}

# ─── A* ──────────────────────────────────────────────────────────────────────
def heuristic(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def get_neighbors(r, c):
    result = []
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < ROWS and 0 <= nc < COLS:
            if abs(GRID_HEIGHTS[r][c] - GRID_HEIGHTS[nr][nc]) <= 200:
                result.append((nr, nc))
    return result

def astar(start, goal, blocked):
    blocked_set = set(blocked)
    open_heap   = [(0, start)]
    came_from   = {}
    g_score     = defaultdict(lambda: float('inf'))
    g_score[start] = 0
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return list(reversed(path))
        for nb in get_neighbors(*current):
            if nb in blocked_set:
                continue
            tg = g_score[current] + 1
            if tg < g_score[nb]:
                came_from[nb] = current
                g_score[nb]   = tg
                heapq.heappush(open_heap, (tg + heuristic(nb, goal), nb))
    return None

def direction(a, b):
    dr, dc = b[0]-a[0], b[1]-a[1]
    return {(-1,0):"NORTH",(1,0):"SOUTH",(0,1):"EAST",(0,-1):"WEST"}.get((dr,dc),"STAY")

# ─── ROS2 Node ───────────────────────────────────────────────────────────────
class MeihuaRosNode:
    def __init__(self, on_position_cb, on_wp_reached):
        print("MeihuaRosNode constructor TERPANGGIL")
        self.node             = None
        self.pub_path         = None
        self.pub_next         = None
        self.pub_status       = None
        self.pub_robot_start  = None
        self.pub_retry_stage2 = None
        self.pub_retry_stage3 = None
        self.on_position      = on_position_cb
        self.on_wp_reached    = on_wp_reached
        # FIX: inisialisasi di sini supaya tidak AttributeError
        # jika ROS message masuk sebelum MeihuaApp selesai __init__
        self.on_wp_reached_safe = None
        self.on_kfs_detection   = None
        self._spin_thread       = None
        self._active            = False

    def start(self):
        print("start() dipanggil")

        if not ros_ok:
            print("ros_ok = False")

            return False
        try:
            print("rclpy.init()")
            rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
            # rclpy.init()

            print("create_node()")

            self.node       = rclpy.create_node("meihua_forest_planner")
            self.pub_path   = self.node.create_publisher(String, "/meihua/path_steps", 10)
            self.pub_next   = self.node.create_publisher(String, "/meihua/next_step",  10)
            self.pub_status = self.node.create_publisher(String, "/meihua/status",     10)
            self.pub_entry  = self.node.create_publisher(String, "/meihua/entry_point", 10)
            qos_tl = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_robot_start  = self.node.create_publisher(Bool, "/robot_start",    qos_tl)
            self.pub_retry_stage2 = self.node.create_publisher(Bool, "/button_stage2",  10)
            self.pub_retry_stage3 = self.node.create_publisher(Bool, "/button_stage3",  10)
            self.node.create_subscription(String, "/meihua/r2_position",self._cb_position, 10)
            self.node.create_subscription(Bool, "/meihua/r2_arrived",self._cb_arrived, 10)
            self.node.create_subscription(Bool, "/meihua/wp_reached", self._cb_wp_reached, 10)
            # if kfs_msg_ok:
            #     self.node.create_subscription(
            #         KFSDetectionArray, "/kfs_detections", self._cb_kfs_detection, 10)

            self._active      = True
            # self._spin_thread = threading.Thread(target=self._spin, daemon=True)
            # self._spin_thread.start()

            print("start() sukses")

            return True
        except Exception as e:
            print(f"[ROS] Error: {e}")
            return False

    def start_spin(self):
        # Tidak dipakai lagi — spin sekarang via root.after() di MeihuaApp
        pass

    def _spin(self):
        # Tidak dipakai lagi — spin sekarang via root.after() di MeihuaApp
        pass

    def _cb_position(self, msg):
        try:
            d = json.loads(msg.data)
            self.on_position(d.get("row",-1), d.get("col",-1), source="topic")
        except Exception:
            pass

    def _cb_arrived(self, msg):
        if msg.data:
            cb = getattr(self, "on_arrived", None)
            if cb:
                cb()
            else:
                self.on_position(R2_ENTRY[0], R2_ENTRY[1], source="arrived")
            print("[ROS] R2 confirmed arrived at entry point via /meihua/r2_arrived")

    def _cb_wp_reached(self, msg):
        if msg.data:
            print("[ROS] Waypoint reached, continue to next step")
            # FIX: pakai getattr supaya aman walau on_wp_reached_safe belum di-set
            cb = getattr(self, 'on_wp_reached_safe', None)
            if cb:
                cb()


    def publish_path(self, path, goal):
        if not (ros_ok and self._active):
            return
        steps = [
            {"step": i, "row": r, "col": c,
             "height": GRID_HEIGHTS[r][c],
             "direction": direction(path[i-1],(r,c)) if i>0 else "START"}
            for i,(r,c) in enumerate(path)
        ]
        payload = json.dumps({
            "total_steps": len(steps),
            "entry_point": list(self.r2_entry),
            "goal": list(goal) if goal else None,
            "steps": steps
        })
        msg = String(); msg.data = payload
        self.pub_path.publish(msg)

    def publish_next_step(self, step_idx, path):
        if not (ros_ok and self._active) or step_idx >= len(path):
            return
        r, c = path[step_idx]
        prev = path[step_idx-1] if step_idx > 0 else (r,c)
        msg  = String()
        msg.data = json.dumps({
            "step": step_idx, "row": r, "col": c,
            "height": GRID_HEIGHTS[r][c],
            "direction": direction(prev,(r,c)),
            "remaining": len(path)-step_idx-1
        })
        self.pub_next.publish(msg)

    def publish_status(self, status, extra=""):
        if not (ros_ok and self._active):
            return
        msg = String()
        msg.data = json.dumps({"status": status, "info": extra,
                               "timestamp": time.time()})
        self.pub_status.publish(msg)

    def publish_robot_start(self):
        if not (ros_ok and self._active and self.pub_robot_start):
            return
        msg = Bool(); msg.data = True
        self.pub_robot_start.publish(msg)
        print("robot start")

    def publish_retry_stage2(self):
        if not (ros_ok and self._active and self.pub_retry_stage2):
            return
        msg = Bool(); msg.data = True
        self.pub_retry_stage2.publish(msg)
        print("retry stage 2")

    def publish_retry_stage3(self):
        if not (ros_ok and self._active and self.pub_retry_stage3):
            return
        msg = Bool(); msg.data = True
        self.pub_retry_stage3.publish(msg)
        print("retry stage 3")

    def publish_entry_point(self, col):
        if not(ros_ok and self._active and self.pub_entry):
            return
        
        msg = String()

        msg.data = json.dumps({"col" : col})
        self.pub_entry.publish(msg)
        print(f"[ROS] Target Entry Stage 2 dikirim: kolom {col}")

    def stop(self):
        self._active = False
        if ros_ok and self.node:
            self.node.destroy_node()
            try: rclpy.shutdown()
            except: pass

# ─── Aplikasi Tkinter ────────────────────────────────────────────────────────
class MeihuaApp:
    def __init__(self, root):
        self.root = root
        print("1. Masuk __init__")

        self.root.title(" Meihua Forest Planner")
        self.root.configure(bg=BG)
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # State
        self.kfs_grid     = [["EMPTY"] * COLS for _ in range(ROWS)]
        self.r2_entry     = R2_ENTRY           # Entry point dipilih dari UI
        self.r2_start     = self.r2_entry
        self.r2_goal      = None
        self.path         = []
        self.path_step    = 0
        self.animating    = False
        self.mode         = "place"
        self.tool         = "R2"
        self.log_msgs     = []
        self.robot_pos    = None
        self.robot_status = STATUS_WAITING
        self.r2_at_entry  = False          # True ketika R2 sudah di (0,1)



        self.ros_mode   = tk.StringVar(value="full_path")
        self._spin_delay_val = 500
        self._topic_val      = "/meihua/path_steps"

        # ROS2
        print("2. Membuat ROS object")

        self.ros = MeihuaRosNode(on_position_cb=self._on_robot_position,
                                 on_wp_reached=self._send_next_step)

        print("3. Sebelum start()")

        self.ros_connected = self.ros.start() if USE_ROS else False
        print("4. Setelah start()", self.ros_connected)

        print("5. Sebelum build_ui")

        self._build_ui()
        print("6. Setelah build_ui")

        # Set semua callback dulu sebelum spin mulai
        self.ros.on_wp_reached_safe  = lambda: self.root.after(0, self._send_next_step)
        self.ros.on_kfs_detection    = lambda det: self._log(
            f"🔍 KFS detected: {det.label} conf={det.confidence:.2f}")
        # Callback arrived — pakai entry dinamis dari app
        self.ros.on_arrived = lambda: self.root.after(
            0, lambda: self._on_robot_position(self.r2_entry[0], self.r2_entry[1], source="arrived"))

        # FIX: Pakai pola sama seperti gui_kfs.py — spin via root.after(), TANPA thread
        # Tidak ada race condition karena berjalan di dalam Tkinter event loop
        if self.ros_connected:
            self.root.after(100, self._ros_spin_loop)


        self._log(f"USE_ROS = {USE_ROS}")
        self._log(f"ros_ok = {ros_ok}")
        self._log(f"ROS connected = {self.ros_connected}")

        # Initial log
        self._log("🌸 Meihua Forest Planner siap!")
        self._log(f"📍 Entry Point R2: grid {self.r2_entry} → H:{GRID_HEIGHTS[self.r2_entry[0]][self.r2_entry[1]]}mm")
        self._log(f"⏳ Menunggu R2 menuju grid {self.r2_entry}...")
        if USE_ROS:
            st = "🟢 ROS2 terhubung!" if self.ros_connected else "🔴 ROS2 gagal connect"
            self._log(st)
        else:
            self._log("ℹ️  Mode: GUI Only (tambah --ros untuk ROS2)")

    # ── Build UI ─────────────────────────────────────────────────────────────
    def _build_ui(self):
        # Header
        hdr = tk.Frame(self.root, bg=BG)
        hdr.pack(fill="x", pady=(8,3))
        # tk.Label(hdr, text="ABU ROBOCON 2026",
        #          bg=BG, fg=ACCENT, font=("Courier",8,"bold")).pack()
        tk.Label(hdr, text="MEIHUA FOREST PLANNER (BLUE SIDE)",
                 bg=BG, fg="#60a5fa", font=("Courier",13,"bold")).pack()
        tk.Label(hdr, text=" ",
                 bg=BG, fg="#64748b", font=("Courier",9)).pack()

        # ── Status Bar ───────────────────────────────────────────────────
        sb = tk.Frame(self.root, bg=PANEL_BG,
                      highlightbackground=BORDER, highlightthickness=1)
        sb.pack(fill="x", padx=10, pady=(3,0))

        self.lbl_robot_status = tk.Label(sb, text=f"● {STATUS_WAITING}",
            bg=PANEL_BG, fg=STATUS_COLOR[STATUS_WAITING],
            font=("Courier",9,"bold"))
        self.lbl_robot_status.pack(side="left", padx=12, pady=4)

        self.lbl_robot_pos = tk.Label(sb, text="R2 Pos: —",
            bg=PANEL_BG, fg="#94a3b8", font=("Courier",8))
        self.lbl_robot_pos.pack(side="left", padx=8)

        self.lbl_entry_status = tk.Label(sb,
            text=f"Entry {self.r2_entry}: ✗ Belum",
            bg=PANEL_BG, fg="#f59e0b", font=("Courier",8))
        self.lbl_entry_status.pack(side="left", padx=8)

        self.lbl_ros = tk.Label(sb,
            text="ROS2: " + ("🟢 ON" if self.ros_connected else "⚫ OFF"),
            bg=PANEL_BG, fg="#4ade80" if self.ros_connected else "#64748b",
            font=("Courier",8))
        self.lbl_ros.pack(side="right", padx=10)

        # ── Main 3-column layout ─────────────────────────────────────────
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=10, pady=6)

        left = tk.Frame(main, bg=BG, width=190)
        left.pack(side="left", fill="y", padx=(0,8))
        left.pack_propagate(False)
        self._build_left(left)

        center = tk.Frame(main, bg=BG)
        center.pack(side="left", fill="both", expand=True)
        self._build_canvas(center)

        right = tk.Frame(main, bg=BG, width=270)
        right.pack(side="left", fill="y", padx=(8,0))
        right.pack_propagate(False)
        self._build_right(right)

    def _sec(self, parent, title, fg=ACCENT):
        f = tk.Frame(parent, bg=PANEL_BG,
                     highlightbackground=BORDER, highlightthickness=1)
        f.pack(fill="x", pady=(0,8))
        tk.Label(f, text=title, bg=PANEL_BG, fg=fg,
                 font=("Courier",8,"bold")).pack(anchor="w", padx=10, pady=(7,3))
        return f

    def _btn(self, parent, text, color, cmd, small=False):
        b = tk.Button(parent, text=text, bg=PANEL_BG, fg=color,
                      activebackground="#1a2a1a", activeforeground=color,
                      font=("Courier", 9 if small else 10),
                      bd=0, cursor="hand2", relief="flat",
                      anchor="w", padx=10, pady=4 if small else 5,
                      highlightbackground=BORDER, highlightthickness=1,
                      command=cmd)
        b.pack(fill="x", padx=8, pady=2)
        return b
    

    def _build_left(self, parent):
        # KFS Tools
        sec = self._sec(parent, "🎴  KFS PLACEMENT TOOL")

        for lbl, tool, col in [
            ("🟠  R1 KFS",   "R1",   "#f97316"),
            ("🔵  R2 KFS",   "R2",   "#3b82f6"),
            ("💀  FAKE KFS", "FAKE", "#ef4444"),
            ("🗑   Erase",    "ERASE","#64748b"),
        ]:
            self._btn(sec, lbl, col, lambda t=tool: self._set_tool(t))

        # R2 Entry — bisa pilih kolom
        sec2 = self._sec(parent, "🤖  R2 — PILIH ENTRY POINT")

        entry_row = tk.Frame(sec2, bg=PANEL_BG)
        entry_row.pack(fill="x", padx=8, pady=(2,4))
        self.entry_var = tk.IntVar(value=1)  # default kolom 1
        self.entry_btns = {}
        for col in [0, 1, 2]:
            h = GRID_HEIGHTS[0][col]
            btn = tk.Radiobutton(
                entry_row,
                text=f"[0,{col}]\n{h}mm",
                variable=self.entry_var,
                value=col,
                command=self._on_entry_changed, 
                bg=PANEL_BG, fg="#a855f7",
                selectcolor="#1e1a2e",
                activebackground=PANEL_BG,
                activeforeground="#a855f7",
                font=("Courier", 8, "bold"),
                indicatoron=0,
                width=6, pady=4,
                relief="flat", bd=1,
                highlightbackground=BORDER,
                highlightthickness=1,
            )
            btn.pack(side="left", padx=2, expand=True)
            self.entry_btns[col] = btn

        self.lbl_entry_selected = tk.Label(
            sec2,
            text=f"  Entry: [0,1]  H:{GRID_HEIGHTS[0][1]}mm",
            bg=PANEL_BG, fg="#a855f7", font=("Courier",9,"bold"))
        self.lbl_entry_selected.pack(anchor="w", padx=6, pady=(0,2))

        self._btn(sec2, "🎯  Set R2 Goal", "#06b6d4", lambda: self._set_mode("goal"))
        self.lbl_goal = tk.Label(sec2, text="Goal: —", bg=PANEL_BG, fg="#06b6d4",
                                  font=("Courier",9))
        self.lbl_goal.pack(anchor="w", padx=10, pady=2)

        # Simulasi tombol
        self._btn(sec2, "✅  [SIM] R2 Sudah di Entry", "#4ade80",
                  self._sim_r2_arrived, small=True)

        # # Robot Control (dari gui_kfs)
        # sec_ctrl = self._sec(parent, "🚀  ROBOT CONTROL")
        # self._btn(sec_ctrl, "▶  START",         "#4ade80", self._send_robot_start)
        # self._btn(sec_ctrl, "↺  RETRY STAGE 2", "#f59e0b", self._send_retry_stage2, small=True)
        # self._btn(sec_ctrl, "↺  RETRY STAGE 3", "#f59e0b", self._send_retry_stage3, small=True)

        # # Pathfinding
        # sec3 = self._sec(parent, "⚡  PATHFINDING & ROS2 SEND")
        # # self._btn(sec3, "🔍  Run A* Algorithm",   "#10b981", self._run_astar)
        # # self._btn(sec3, "▶   Animate + Send",      "#a855f7", self._animate)
        # # self._btn(sec3, "🔄  Reset All",            "#ef4444", self._reset)
        # self._btn(sec3, "📡  Send Full Path",      "#3b82f6", self._send_path_ros)
        # self._btn(sec3, "⏭   Next Step (Manual)",  "#06b6d4", self._send_next_step)
        # self._btn(sec3, "⏹   Stop",                "#f59e0b", self._stop)

        # # Mode indicator
        self.lbl_mode = tk.Label(parent, text="MODE: PLACE KFS",
                                  bg=BG, fg=ACCENT, font=("Courier",9,"bold"))
        self.lbl_mode.pack(pady=5)

        # Height Legend
        # sec4 = self._sec(parent, "📊  HEIGHT LEGEND")
        # for h, hc in HEIGHT_COLOR.items():
        #     row = tk.Frame(sec4, bg=PANEL_BG)
        #     row.pack(fill="x", padx=10, pady=2)
        #     tk.Canvas(row, width=14, height=14, bg=hc["bg"],
        #               highlightbackground=hc["border"],
        #               highlightthickness=2).pack(side="left", padx=(0,6))
        #     tk.Label(row, text=f"{h}mm", bg=PANEL_BG, fg=hc["label"],
        #              font=("Courier",10)).pack(side="left")
        # tk.Label(sec4, text="  Max climb: ±200mm", bg=PANEL_BG, fg="#64748b",
        #          font=("Courier",8)).pack(anchor="w", padx=6, pady=(2,8))

    def _build_canvas(self, parent):
        self.lbl_hint = tk.Label(parent,
            text="← Klik sel untuk menempatkan KFS",
            bg=BG, fg="#64748b", font=("Courier",9))
        self.lbl_hint.pack(pady=(0,4))

        cw = COLS*(CELL_W+PAD)+PAD
        ch = ROWS*(CELL_H+PAD)+PAD
        self.canvas = tk.Canvas(parent, width=cw, height=ch,
                                bg=PANEL_BG,
                                highlightbackground=BORDER, highlightthickness=1,
                                cursor="hand2")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self._on_canvas_click)

        self.lbl_path_info = tk.Label(parent, text="",
                                       bg=BG, fg=ACCENT, font=("Courier",10))
        self.lbl_path_info.pack(pady=4)
        self._draw_grid()

    def _build_right(self, parent):
        # ROS2 Config
        # sec_ros = self._sec(parent, "📡  ROS2 CONFIG", fg="#36393f")

        # r1 = tk.Frame(sec_ros, bg=PANEL_BG)
        # r1.pack(fill="x", padx=8, pady=2)
        # tk.Label(r1, text="Topic:", bg=PANEL_BG, fg="#94a3b8",
        #          font=("Courier",8)).pack(side="left")
        # self.entry_topic = tk.Entry(r1, bg="#0d1117", fg="#60a5fa",
        #                             font=("Courier",9), bd=0,
        #                             insertbackground="#60a5fa", width=22)
        # self.entry_topic.insert(0, "/meihua/path_steps")
        # self.entry_topic.pack(side="left", padx=4)

        # r2 = tk.Frame(sec_ros, bg=PANEL_BG)
        # r2.pack(fill="x", padx=8, pady=2)
        # tk.Label(r2, text="Mode:", bg=PANEL_BG, fg="#94a3b8",
        #          font=("Courier",8)).pack(side="left")
        # self.ros_mode = tk.StringVar(value="full_path")
        # for val, lbl in [("full_path","Full Path"),("step_by_step","Step-by-Step")]:
        #     tk.Radiobutton(r2, text=lbl, variable=self.ros_mode, value=val,
        #                    bg=PANEL_BG, fg="#94a3b8", selectcolor="#1e3a2a",
        #                    activebackground=PANEL_BG,
        #                    font=("Courier",8)).pack(side="left", padx=4)

        # r3 = tk.Frame(sec_ros, bg=PANEL_BG)
        # r3.pack(fill="x", padx=8, pady=(2,6))
        # tk.Label(r3, text="Step Delay(ms):", bg=PANEL_BG, fg="#94a3b8",
        #          font=("Courier",8)).pack(side="left")
        # self.spin_delay = tk.Spinbox(r3, from_=100, to=5000, increment=100,
        #                              width=6, bg="#0d1117", fg="#60a5fa",
        #                              font=("Courier",9), bd=0)
        # self.spin_delay.delete(0,"end"); self.spin_delay.insert(0,"500")
        # self.spin_delay.pack(side="left", padx=4)


        # Robot Control (dari gui_kfs)
        sec_ctrl = self._sec(parent, "🚀  ROBOT CONTROL")
        self._btn(sec_ctrl, "▶  START",         "#4ade80", self._send_robot_start)
        self._btn(sec_ctrl, "↺  RETRY STAGE 2", "#f59e0b", self._send_retry_stage2, small=True)
        self._btn(sec_ctrl, "↺  RETRY STAGE 3", "#f59e0b", self._send_retry_stage3, small=True)

        # JSON Preview
        # sec_json = self._sec(parent, "📋  PATH JSON → ROS2", fg="#3b82f6")
        # self.json_box = scrolledtext.ScrolledText(
        #     sec_json, bg="#060d14", fg="#60a5fa",
        #     font=("Courier",8), height=7, bd=0,
        #     state="disabled", wrap="none")
        # self.json_box.pack(fill="x", padx=8, pady=(0,8))

        # Pathfinding
        sec3 = self._sec(parent, "⚡  PATHFINDING & ROS2 SEND")
        self._btn(sec3, "🔍  Run A* Algorithm",   "#10b981", self._run_astar)
        self._btn(sec3, "▶   Animate + Send",      "#a855f7", self._animate)
        self._btn(sec3, "🔄  Reset All",            "#ef4444", self._reset)
        # self._btn(sec3, "📡  Send Full Path",      "#3b82f6", self._send_path_ros)
        # self._btn(sec3, "⏭   Next Step (Manual)",  "#06b6d4", self._send_next_step)
        # self._btn(sec3, "⏹   Stop",                "#f59e0b", self._stop)


        # Event Log
        sec_log = self._sec(parent, "📋  EVENT LOG")
        self.log_box = scrolledtext.ScrolledText(
            sec_log, bg="#060d14", fg="#e2e8f0",
            font=("Courier",8), height=5, bd=0,
            state="disabled", wrap="word")
        self.log_box.pack(fill="x", padx=8, pady=(0,4))

        # Path Steps
        sec_path = self._sec(parent, "🗺  PATH STEPS")
        self.path_box = scrolledtext.ScrolledText(
            sec_path, bg="#060d14", fg="#e2e8f0",
            font=("Courier",8), height=6, bd=0,
            state="disabled", wrap="none")
        self.path_box.pack(fill="x", padx=8, pady=(0,8))

    # ── Drawing ──────────────────────────────────────────────────────────────
    def _draw_grid(self):
        self.canvas.delete("all")
        for r in range(ROWS):
            for c in range(COLS):
                self._draw_cell(r, c)

    def _draw_cell(self, r, c):
        x    = PAD + c*(CELL_W+PAD)
        y    = PAD + r*(CELL_H+PAD)
        h    = GRID_HEIGHTS[r][c]
        hc   = HEIGHT_COLOR[h]
        kfs  = self.kfs_grid[r][c]

        on_path  = any(p==(r,c) for p in self.path)
        p_idx    = next((i for i,p in enumerate(self.path) if p==(r,c)), -1)
        is_curr  = (self.animating and self.path and
                    self.path[min(self.path_step,len(self.path)-1)]==(r,c))
        is_entry = (r,c) == self.r2_entry
        is_goal  = (r,c) == self.r2_goal
        in_path  = on_path and p_idx <= self.path_step
        is_rpos  = self.robot_pos == (r,c)

        # Background
        if is_curr:   bg_col = "#1a3a0a"
        elif in_path: bg_col = "#0f2a0f"
        else:         bg_col = hc["bg"]

        # Border
        if is_rpos and not self.animating:
            bd_col = "#4ade80"
        elif is_curr:
            bd_col = "#4ade80"
        elif is_entry:
            bd_col = "#4ade80" if self.r2_at_entry else "#f59e0b"
        elif is_goal:
            bd_col = "#06b6d4"
        elif in_path:
            bd_col = "#2d6e2d"
        else:
            bd_col = hc["border"]

        self.canvas.create_rectangle(x, y, x+CELL_W, y+CELL_H,
            fill=bg_col, outline=bd_col, width=2)

        # Height label (top-right)
        self.canvas.create_text(x+CELL_W-5, y+7, text=f"{h}mm",
            anchor="ne", fill=hc["label"], font=("Courier",8,"bold"))

        # Coord (top-left)
        self.canvas.create_text(x+5, y+7, text=f"{r},{c}",
            anchor="nw", fill="#374151", font=("Courier",7))

        # Path step number (bottom-right)
        if in_path:
            self.canvas.create_text(x+CELL_W-5, y+CELL_H-7,
                text=f"#{p_idx}", anchor="se", fill="#4ade80",
                font=("Courier",8,"bold"))

        cx = x + CELL_W//2
        cy = y + CELL_H//2

        # Center content
        if is_rpos and not self.animating:
            self.canvas.create_text(cx, cy-8,  text="🤖", font=("",18))
            self.canvas.create_text(cx, cy+14, text="R2 LIVE",
                fill="#4ade80", font=("Courier",7,"bold"))

        elif is_curr:
            self.canvas.create_text(cx, cy, text="🤖", font=("",22))

        elif is_entry:
            col_e = "#4ade80" if self.r2_at_entry else "#f59e0b"
            mark  = "ENTRY ✓" if self.r2_at_entry else "ENTRY ⏳"
            self.canvas.create_rectangle(cx-32, cy-13, cx+32, cy+13,
                fill="#0f1923", outline=col_e, width=1)
            self.canvas.create_text(cx, cy, text=mark,
                fill=col_e, font=("Courier",8,"bold"))

        elif is_goal:
            self.canvas.create_text(cx, cy,    text="🎯", font=("",18))
            self.canvas.create_text(cx, cy+20, text="GOAL",
                fill="#06b6d4", font=("Courier",7,"bold"))

        elif kfs != "EMPTY":
            kc = KFS_COLOR[kfs]
            self.canvas.create_rectangle(cx-29, cy-13, cx+29, cy+13,
                fill=kc["bg"], outline=kc["border"], width=2)
            self.canvas.create_text(cx, cy,
                text="⚠ FAKE" if kfs=="FAKE" else f"{kfs} KFS",
                fill=kc["text"], font=("Courier",9,"bold"))

        elif in_path and not is_goal:
            self.canvas.create_text(cx, cy, text="◆",
                fill="#4ade80", font=("Courier",14))

    # ── Events ───────────────────────────────────────────────────────────────
    def _on_canvas_click(self, event):
        c = (event.x - PAD) // (CELL_W + PAD)
        r = (event.y - PAD) // (CELL_H + PAD)
        if not (0 <= r < ROWS and 0 <= c < COLS):
            return
        if self.mode == "goal":
            if self.kfs_grid[r][c] == "FAKE":
                messagebox.showwarning("⚠️", "Tidak bisa set goal di FAKE KFS!")
                return
            self.r2_goal = (r, c)
            self.lbl_goal.config(text=f"Goal: [{r},{c}] {GRID_HEIGHTS[r][c]}mm")
            self._log(f"🎯 Goal: [{r},{c}] H:{GRID_HEIGHTS[r][c]}mm")
            self._set_mode("place")
            self.path = []
        else:
            nxt = "EMPTY" if self.tool == "ERASE" else self.tool
            self.kfs_grid[r][c] = nxt
            self._log(f"📍 [{r},{c}] → {nxt}")
        self._draw_grid()

    def _set_tool(self, tool):
        self.tool = tool
        self._set_mode("place")

    def _set_mode(self, mode):
        self.mode = mode
        self.lbl_mode.config(text={
            "place": f"MODE: PLACE  [{self.tool}]",
            "goal":  "MODE: KLIK → SET GOAL R2",
        }.get(mode, mode))
        self.lbl_hint.config(text={
            "place": "← Klik sel untuk menempatkan KFS",
            "goal":  "← Klik sel untuk set posisi GOAL R2",
        }.get(mode, ""))

    # ── Robot position callbacks ──────────────────────────────────────────────
    def _on_robot_position(self, r, c, source="topic"):
        """Thread-safe callback dari ROS2."""
        self.root.after(0, lambda: self._handle_position(r, c, source))

    def _handle_position(self, r, c, source):
        self.robot_pos = (r, c)
        self.lbl_robot_pos.config(
            text=f"R2 Pos: [{r},{c}] {GRID_HEIGHTS[r][c]}mm")
        self._log(f"📡 R2 posisi: [{r},{c}] via {source}")

        # Cek apakah sudah di entry point
        if (r, c) == self.r2_entry and not self.r2_at_entry:
            self.r2_at_entry = True
            self._set_status(STATUS_READY)
            self.lbl_entry_status.config(
                text=f"Entry {self.r2_entry}: ✓ SIAP", fg="#4ade80")
            self._log(f"✅ R2 di entry {self.r2_entry}! Algoritma siap.")
            self.ros.publish_status(STATUS_READY, f"R2 at {self.r2_entry}")
            # Auto-run A* jika goal sudah diset
            if self.r2_goal:
                self._log("🚀 Auto-run A* (goal sudah ada)...")
                self.root.after(400, self._run_astar)

        self._draw_grid()

    def _on_entry_changed(self):
        col = self.entry_var.get()
        self.r2_entry = (0, col)
        self.r2_start = self.r2_entry
        h = GRID_HEIGHTS[0][col]
        self.lbl_entry_selected.config(text=f"  Entry: [0,{col}]  H:{h}mm")
        self.r2_at_entry  = False
        self.path = []
        self.path_step = 0
        self.lbl_entry_status.config(
            text=f"Entry [0,{col}]: ✗ Belum", fg="#f59e0b")
        self._set_status(STATUS_WAITING)
        self._log(f"📍 Entry point diubah → [0,{col}] H:{h}mm")
        self._draw_grid()


        if self.ros_connected:
            self.ros.publish_entry_point(col)

    def _sim_r2_arrived(self):
        """Simulasi: tekan ini jika tidak ada robot fisik."""
        self._on_robot_position(self.r2_entry[0], self.r2_entry[1], source="SIMULATION")

    def _set_status(self, status, extra=""):
        self.robot_status = status
        col = STATUS_COLOR.get(status, "#64748b")
        self.lbl_robot_status.config(text=f"● {status}", fg=col)
        self.ros.publish_status(status, extra)

    # ── A* ───────────────────────────────────────────────────────────────────
    def _run_astar(self):
        if not self.r2_goal:
            messagebox.showwarning("⚠️", "Set Goal dulu!")
            return
        if not self.r2_at_entry:
            messagebox.showwarning("⏳ Menunggu R2",
                f"R2 belum ada di entry point {self.r2_entry}!\n\n"
                "Tekan [SIM] R2 Sudah di Entry untuk simulasi,\n"
                "atau tunggu sinyal dari robot via:\n"
                "  ros2 topic pub /meihua/r2_arrived std_msgs/String '{}'")
            return

        blocked = [(r,c) for r in range(ROWS) for c in range(COLS)
                   if self.kfs_grid[r][c] == "FAKE"]
        result = astar(self.r2_start, self.r2_goal, blocked)

        if result:
            self.path = result
            self.path_step = 0
            self._set_status(STATUS_RUNNING)
            self._log(f"✅ A* selesai! {len(result)} langkah")
            self._update_path_box()
            # self._update_json_preview()
            self._update_path_info()
            self._log("📡 Auto-kirim step pertama ke robot...")
            self.root.after(200, self._send_next_step)
        else:
            self.path = []
            self._set_status(STATUS_ERROR)
            self._log("❌ Tidak ada jalur valid!")
            messagebox.showerror("Tidak Ada Jalur",
                "A* tidak menemukan jalur!\n"
                "• Cek perbedaan ketinggian (max ±200mm)\n"
                "• Cek posisi FAKE KFS\n"
                "• Pastikan goal bisa dicapai dari entry (0,1)")
        self._draw_grid()

    # ── ROS2 Send ─────────────────────────────────────────────────────────────
    def _send_path_ros(self):
        if not self.path:
            messagebox.showinfo("Info", "Jalankan A* dulu!")
            return
        if not self.r2_at_entry:
            messagebox.showwarning("⏳", "R2 belum di entry point!")
            return
        self.ros.publish_path(self.path, self.r2_goal)
        self._log(f"📡 Full path dikirim → /meihua/path_steps")
        self._log(f"   {len(self.path)} steps, mode: {self.ros_mode.get()}")
        if not ros_ok:
            self._log("   (ROS2 OFF — lihat JSON Preview)")

    # def _send_next_step(self):
    #     if not self.path:
    #         messagebox.showinfo("Info", "Jalankan A* dulu!")
    #         return
        
    #     if self.path_step >= len(self.path):
    #         self._log("✅ Semua step sudah dikirim!")
    #         return
        
    #     if self.path_step == 0:
    #         self._log("Skip step #0 (entry point, robot sudah di sini)")
    #         self.path_step = 1
    #         if self.path_step >= len(self.path):
    #             self._log("✅ Hanya 1 step, sudah di goal!")
    #             self._set_status(STATUS_DONE)
    #             return
            
    #     r, c = self.path[self.path_step]
    #     self.ros.publish_next_step(self.path_step, self.path)
    #     self._log(f"📡 Step #{self.path_step}: [{r},{c}] {GRID_HEIGHTS[r][c]}mm")
    #     if self.path_step == len(self.path)-1:
    #         self._set_status(STATUS_DONE)
    #         self._log("🏁 Semua step dikirim!")
    #     else:
    #         self.path_step += 1
    #         self._update_path_info()
    #         self._draw_grid()

    def _send_next_step(self):
            
        if not self.path:
            messagebox.showinfo("Info", "Jalankan A* dulu!")
            return
 
        # Saat dipanggil pertama kali (dari _run_astar), path_step = 0 (entry point).
        # Kita skip step 0 karena robot sudah di sana, langsung kirim step 1.
        # Saat dipanggil dari wp_reached, path_step sudah menunjuk step yang BARU SAJA
        # dikirim — increment dulu, baru kirim step berikutnya.
        if self.path_step == 0:
            # Pertama kali: skip entry point, mulai dari step 1
            self.path_step = 1
            self._log("⏭ Skip step #0 (entry point, robot sudah di sini)")
        else:
            # Dipanggil setelah wp_reached: robot sudah tiba di path_step,
            # increment ke step berikutnya sekarang
            self.path_step += 1
 
        if self.path_step >= len(self.path):
            self._log("✅ Semua step sudah dikirim! Kirim mundur exit stage2...")

            last_r, last_c = self.path[-1]
            last_h = GRID_HEIGHTS[last_r][last_c]

            import json
            from std_msgs.msg import String
            exit_payload = json.dumps({
                "step" : len(self.path),
                "row" : last_r + 1,
                "col" : last_c,
                "height" : last_h - 200,
                "direction" : "DOWN"
            })

            msg = String()
            msg.data = exit_payload
            self.ros.pub_next.publish(msg)
            self._log("📡 EXIT mundur dikirim → keluar stage2")
            self._log("✅ Semua step sudah dikirim!")
            self._set_status(STATUS_DONE)
            return
 
        r, c = self.path[self.path_step]
        self.ros.publish_next_step(self.path_step, self.path)
        self._log(f"📡 Step #{self.path_step}/{len(self.path)-1}: [{r},{c}] {GRID_HEIGHTS[r][c]}mm")
        self._update_path_info()
        self._draw_grid()
        if self.path_step == len(self.path)-1:
            self._log("🏁 Step terakhir dikirim! Menunggu konfirmasi robot...")

    # ── Animate ──────────────────────────────────────────────────────────────
    def _animate(self):
        if not self.path:
            messagebox.showinfo("Info", "Jalankan A* dulu!")
            return
        if not self.r2_at_entry:
            messagebox.showwarning("⏳", "R2 belum di entry point!")
            return
        self.path_step = 0
        self.animating = True
        self._draw_grid()
        self._anim_step()

    def _anim_step(self):
        if not self.animating:
            return
        self._draw_grid()
        self._update_path_info()
        # Kirim step-by-step jika mode tersebut aktif
        if self.ros_mode.get() == "step_by_step":
            self.ros.publish_next_step(self.path_step, self.path)

        if self.path_step < len(self.path)-1:
            self.path_step += 1
            delay = self._spin_delay_val
            self.root.after(delay, self._anim_step)
        else:
            self.animating = False
            self._set_status(STATUS_DONE)
            self._log("🏁 R2 mencapai goal!")

    def _stop(self):
        self.animating = False
        self._log("⏹ Dihentikan")

    def _reset(self):
        self.kfs_grid     = [["EMPTY"] * COLS for _ in range(ROWS)]
        self.r2_start     = self.r2_entry    # ikut entry yang dipilih user
        self.r2_goal      = None
        self.path         = []
        self.path_step    = 0
        self.animating    = False
        self.robot_pos    = None
        self.r2_at_entry  = False
        self._set_status(STATUS_WAITING)
        self.lbl_goal.config(text="Goal: —")
        self.lbl_path_info.config(text="")
        self.lbl_robot_pos.config(text="R2 Pos: —")
        self.lbl_entry_status.config(
            text=f"Entry {self.r2_entry}: ✗ Belum", fg="#f59e0b")
        self._set_mode("place")
        self._draw_grid()
        self._log("🔄 Reset selesai")
        self._update_path_box()
        # self._update_json_preview()

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _log(self, msg):
        self.log_msgs.insert(0, msg)
        self.log_msgs = self.log_msgs[:30]
        self.log_box.config(state="normal")
        self.log_box.delete("1.0","end")
        for m in self.log_msgs:
            self.log_box.insert("end", m+"\n")
        self.log_box.config(state="disabled")

    def _update_path_box(self):
        self.path_box.config(state="normal")
        self.path_box.delete("1.0","end")
        for i,(r,c) in enumerate(self.path):
            icon = "🟣" if i==0 else "🎯" if i==len(self.path)-1 else "◆"
            dirn = direction(self.path[i-1],(r,c)) if i>0 else "START"
            h    = GRID_HEIGHTS[r][c]
            self.path_box.insert("end",
                f"{icon} #{i:2d}  [{r},{c}]  {h}mm  {dirn}\n")
        self.path_box.config(state="disabled")

    def _update_json_preview(self):
        self.json_box.config(state="normal")
        self.json_box.delete("1.0","end")
        if self.path:
            steps = [
                {"step":i, "row":r, "col":c,
                 "height":GRID_HEIGHTS[r][c],
                 "direction": direction(self.path[i-1],(r,c)) if i>0 else "START"}
                for i,(r,c) in enumerate(self.path)
            ]
            payload = {
                "total_steps": len(steps),
                "entry_point": list(self.r2_entry),
                "goal": list(self.r2_goal) if self.r2_goal else None,
                "steps": steps
            }
            self.json_box.insert("end", json.dumps(payload, indent=2))
        self.json_box.config(state="disabled")

    def _update_path_info(self):
        if not self.path:
            self.lbl_path_info.config(text="")
            return
        r, c = self.path[min(self.path_step, len(self.path)-1)]
        self.lbl_path_info.config(
            text=f"Step {self.path_step}/{len(self.path)-1}  |  "
                 f"[{r},{c}]  |  {GRID_HEIGHTS[r][c]}mm")

    def _send_robot_start(self):
        self.ros.publish_robot_start()
        self._log("🚀 START dikirim → /robot_start")

    def _send_retry_stage2(self):
        self.ros.publish_retry_stage2()
        self._log("↺ RETRY STAGE 2 dikirim → /button_stage2")

    def _send_retry_stage3(self):
        self.ros.publish_retry_stage3()
        self._log("↺ RETRY STAGE 3 dikirim → /button_stage3")

    def _ros_spin_loop(self):
        """Spin ROS sekali tiap 50ms via Tkinter after() — sama persis pola gui_kfs.py.
        Tidak pakai thread, jadi tidak ada race condition sama sekali."""
        if self.ros._active and self.ros.node:
            try:
                rclpy.spin_once(self.ros.node, timeout_sec=0)
            except Exception:
                pass
        self.root.after(50, self._ros_spin_loop)

    def _on_close(self):
        self.animating = False
        self.ros.stop()
        self.root.destroy()


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    app  = MeihuaApp(root)
    root.mainloop()
    

if __name__ == "__main__":
    main()