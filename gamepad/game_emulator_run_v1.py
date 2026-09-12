import copy
import math
import os
import socket
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

import numpy as np

# -----------------------------------------------------------------------------
# Networking (send UDP packets to the MJLab play environment)
# -----------------------------------------------------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ADDR = (
    os.environ.get("BALLET_ROBOT_IP", "127.0.0.1"),
    int(os.environ.get("BALLET_UDP_PORT", "55001")),
)  # same v1 wire protocol; only destination is configurable for deployment

# -----------------------------------------------------------------------------
# GUI constants
# -----------------------------------------------------------------------------
NUM_AXES = 29
STREAM_RATE_HZ = 50.0
STREAM_PERIOD_S = 1.0 / STREAM_RATE_HZ
STREAM_INTERVAL_MS = round(STREAM_PERIOD_S * 1000.0)
GUI_REFRESH_PERIOD_S = 1.0 / 20.0
STATUS_INTERVAL_S = 1.0

SLIDER_MIN, SLIDER_MAX = -1.0, 1.0
EXTRA_SPEED_MIN, EXTRA_SPEED_MAX = -1.0, 1.0  # v_x ∈ [-1,1]
EXTRA_LR_MIN, EXTRA_LR_MAX = -1.0, 1.0  # v_y
EXTRA_ANGLE_MIN, EXTRA_ANGLE_MAX = -1.0, 1.0  # ω_z ∈ [−1,1]

# Canonical G1-29DoF joint order (matches the compiled MJCF joint order --
# see wbc_ballet.robots.g1.constants / tests/test_g1_asset.py -- the SAME
# order used by joint_pos/joint_vel/axis_actual_normalized observations and
# by the action space). Axis i here means the same joint as axis i
# everywhere else in the stack.
#
# NOTE (fixed during the 23->29 DOF port): the previous 23-axis list here
# used Orbit's own axis order (interleaved left/right, legs mixed with
# arms), which did NOT match the order the trained policy actually used
# (mjlab resolves an unfiltered SceneEntityCfg's joints in natural MJCF
# declaration order, not Orbit's order). That mismatch meant this tool's
# slider labels did not correspond to the axes they actually drove. This
# list uses the correct (natural MJCF) order instead.
JOINT_NAMES = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# --- нейтральная поза (нормализованная в [-1,1]) ---
# Exact normalization of robots/g1/constants.py:HOME_KEYFRAME through the
# joint ranges in robots/g1/xmls/g1.xml, in the canonical JOINT_NAMES order.
INIT_BASELINE = np.array(
    [
        -0.101487848,
        -0.700002865,
        0.0,
        -0.738956350,
        -0.036475753,
        0.0,
        -0.101487848,
        0.700002865,
        0.0,
        -0.738956350,
        -0.036475753,
        0.0,
        0.0,
        0.0,
        0.0,
        0.194249601,
        -0.078990546,
        0.0,
        0.220524573,
        0.0,
        0.0,
        0.0,
        0.194249601,
        0.078990546,
        0.0,
        0.220524573,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)

assert INIT_BASELINE.shape == (NUM_AXES,)

# ================================== UTIL ===================================


def smoothstep(t: np.ndarray) -> np.ndarray:
    """3t^2 - 2t^3, монотонная S-кривая, нулевые производные на концах."""
    return t * t * (3.0 - 2.0 * t)


def resample_track(y_old: np.ndarray, new_T: int) -> np.ndarray:
    """Линейный ресэмпл в новый размер."""
    T_old = len(y_old)
    if T_old == new_T:
        return y_old.copy()
    x_old = np.linspace(0.0, 1.0, T_old, dtype=np.float32)
    x_new = np.linspace(0.0, 1.0, new_T, dtype=np.float32)
    return np.interp(x_new, x_old, y_old).astype(np.float32)


def pattern_frame_index(elapsed_s: float, duration_s: float, total_frames: int) -> int:
    """Map monotonic playback time to a trajectory frame.

    Playback is time-based, so a delayed Tk callback skips stale frames instead
    of stretching a five-second pattern into minutes.
    """
    if total_frames < 1:
        raise ValueError("total_frames must be positive")
    if duration_s <= 0.0 or total_frames == 1:
        return total_frames - 1
    progress = float(np.clip(elapsed_s / duration_s, 0.0, 1.0))
    return min(int(progress * (total_frames - 1)), total_frames - 1)


# =============================== PATTERN EDITOR ==============================


class PatternEditor(tk.Toplevel):
    """
    Редактор паттернов: 29 полос-канвасов, редактирование кликом/перетаскиванием.
    Между пинами строится монотонный плавный переход (smoothstep).
    Начало/конец — baseline. Состояние сохраняется между открытиями окна.
    """

    CANVAS_W = 1000
    ROW_H = 100
    PAD_Y = 8

    def __init__(
        self,
        parent,
        *,
        baseline_vec,
        on_save_callback,
        initial_tracks=None,  # np.ndarray [T,29] или None
        initial_pins=None,  # list[set] или None
        initial_duration_s=None,  # float или None
    ):
        super().__init__(parent)
        self.title("Pattern Editor")
        self.transient(parent)
        self.grab_set()

        self.parent = parent
        self.on_save_callback = on_save_callback

        # baseline (29,)
        self.baseline = np.asarray(baseline_vec, dtype=np.float32)

        # state
        if initial_tracks is not None:
            self.tracks = initial_tracks.astype(np.float32).copy()  # [T,29]
            self.T = self.tracks.shape[0]
            self.duration_s = tk.DoubleVar(
                value=float(
                    initial_duration_s
                    if initial_duration_s is not None
                    else (self.T - 1) * STREAM_PERIOD_S
                )
            )
            if initial_pins is not None:
                self.pins = [set(p) for p in initial_pins]
                for a in range(NUM_AXES):
                    self.pins[a].add(0)
                    self.pins[a].add(self.T - 1)
            else:
                self.pins = [{0, self.T - 1} for _ in range(NUM_AXES)]
        else:
            self.duration_s = tk.DoubleVar(value=5.0)
            # Include both t=0 and t=duration endpoints.
            self.T = max(3, round(self.duration_s.get() * STREAM_RATE_HZ) + 1)
            self.tracks = np.tile(self.baseline[None, :], (self.T, 1)).astype(
                np.float32
            )
            self.pins = [{0, self.T - 1} for _ in range(NUM_AXES)]

        self._build_ui()
        self._redraw_all()

    # -------------------- UI --------------------
    def _build_ui(self):
        top = ttk.Frame(self)
        top.pack(fill="x", padx=10, pady=8)

        ttk.Label(top, text="Duration (s):").pack(side="left")
        ttk.Entry(top, textvariable=self.duration_s, width=6).pack(
            side="left", padx=(4, 12)
        )
        ttk.Button(top, text="Apply", command=self._apply_duration).pack(side="left")

        ttk.Button(top, text="Reset Axis", command=self._reset_selected).pack(
            side="left", padx=(10, 0)
        )
        ttk.Button(top, text="Reset All", command=self._reset_all).pack(
            side="left", padx=(6, 0)
        )

        ttk.Button(top, text="Save & Play", command=self._save_and_play).pack(
            side="right"
        )

        # scroll area with 29 canvases
        outer = ttk.Frame(self)
        outer.pack(fill="both", expand=True, padx=10, pady=(4, 10))

        self.scroll = tk.Canvas(
            outer,
            highlightthickness=0,
            height=NUM_AXES * (self.ROW_H + self.PAD_Y) // 2,
        )
        self.scroll.pack(side="left", fill="both", expand=True)

        vs = ttk.Scrollbar(outer, orient="vertical", command=self.scroll.yview)
        vs.pack(side="right", fill="y")
        self.scroll.configure(yscrollcommand=vs.set)

        self.rows_holder = ttk.Frame(self.scroll)
        self.scroll.create_window((0, 0), window=self.rows_holder, anchor="nw")
        self.rows_holder.bind(
            "<Configure>",
            lambda e: self.scroll.configure(scrollregion=self.scroll.bbox("all")),
        )

        # per-axis canvases
        self.canvases = []
        self.active_axis = 0
        for a in range(NUM_AXES):
            row = ttk.Frame(self.rows_holder)
            row.pack(fill="x", pady=(0, self.PAD_Y))

            ttk.Label(row, text=JOINT_NAMES[a], width=28).pack(side="left", padx=(0, 8))
            c = tk.Canvas(
                row,
                width=self.CANVAS_W,
                height=self.ROW_H,
                bg="#0e0e10",
                highlightthickness=2,
            )
            c.pack(side="left", fill="x", expand=True)
            c.configure(highlightbackground="#2a2a2d", highlightcolor="#4ea1ff")
            c.bind("<Enter>", lambda e, ax=a: self._set_active_axis(ax))
            c.bind("<Button-1>", lambda e, ax=a: self._on_click_drag(ax, e))
            c.bind("<B1-Motion>", lambda e, ax=a: self._on_click_drag(ax, e))
            c.bind("<ButtonRelease-1>", lambda e, ax=a: self._on_release(ax, e))
            self.canvases.append(c)

    # -------------------- helpers --------------------
    def _set_active_axis(self, a):
        self.active_axis = a

    def _apply_duration(self):
        new_d = max(0.1, float(self.duration_s.get()))
        self.duration_s.set(new_d)
        new_T = max(3, round(new_d * STREAM_RATE_HZ) + 1)
        if new_T == self.T:
            return
        old_T = self.T
        self.tracks = np.stack(
            [resample_track(self.tracks[:, a], new_T) for a in range(NUM_AXES)], axis=1
        )
        new_pins = []
        for a in range(NUM_AXES):
            scaled = {round(p / (old_T - 1) * (new_T - 1)) for p in self.pins[a]}
            scaled.add(0)
            scaled.add(new_T - 1)
            new_pins.append(scaled)
        self.pins = new_pins
        self.T = new_T
        self._redraw_all()

    # координаты
    def _x_to_i(self, x):
        x = np.clip(x, 0, self.CANVAS_W - 1)
        i = round(x / (self.CANVAS_W - 1) * (self.T - 1))
        return int(np.clip(i, 0, self.T - 1))

    def _y_to_val(self, y):
        y = np.clip(y, 0, self.ROW_H - 1)
        v = 1.0 - 2.0 * (y / (self.ROW_H - 1))
        return float(np.clip(v, -1.0, 1.0))

    def _val_to_y(self, v):
        v = float(np.clip(v, -1.0, 1.0))
        y = (1.0 - v) * 0.5 * (self.ROW_H - 1)
        return y

    # монотонный сегмент между двумя пинами
    def _fill_segment_monotone(self, axis, i0, i1):
        if i1 <= i0:
            return
        y0 = self.tracks[i0, axis]
        y1 = self.tracks[i1, axis]
        n = i1 - i0
        xs = np.arange(0, n + 1, dtype=np.float32) / float(n)
        s = smoothstep(xs)  # монотонно
        self.tracks[i0 : i1 + 1, axis] = y0 + (y1 - y0) * s

    def _rebuild_axis_from_pins(self, axis):
        self.pins[axis].add(0)
        self.pins[axis].add(self.T - 1)
        self.tracks[0, axis] = self.baseline[axis]
        self.tracks[-1, axis] = self.baseline[axis]
        pins_sorted = sorted(self.pins[axis])
        for k in range(len(pins_sorted) - 1):
            i0, i1 = pins_sorted[k], pins_sorted[k + 1]
            self._fill_segment_monotone(axis, i0, i1)

    # отрисовка
    def _draw_axis(self, a):
        c = self.canvases[a]
        c.delete("all")
        y0 = self._val_to_y(0.0)
        c.create_line(0, y0, self.CANVAS_W, y0, fill="#2a2a2d")
        yb = self._val_to_y(self.baseline[a])
        c.create_line(0, yb, self.CANVAS_W, yb, fill="#2d6cdf", dash=(4, 3))
        stride = max(1, self.T // 1000)
        pts = []
        for i in range(0, self.T, stride):
            x = i / (self.T - 1) * (self.CANVAS_W - 1)
            y = self._val_to_y(self.tracks[i, a])
            pts.extend([x, y])
        c.create_line(*pts, fill="#e5e5e5", width=2, smooth=True, splinesteps=12)
        for i in self.pins[a]:
            x = i / (self.T - 1) * (self.CANVAS_W - 1)
            y = self._val_to_y(self.tracks[i, a])
            r = 3
            c.create_oval(x - r, y - r, x + r, y + r, fill="#ffffff", outline="")
        c.configure(
            highlightbackground=("#4ea1ff" if a == self.active_axis else "#2a2a2d")
        )

    def _redraw_all(self):
        for a in range(NUM_AXES):
            self._draw_axis(a)

    # события
    def _on_click_drag(self, axis, event):
        self.active_axis = axis
        i = self._x_to_i(event.x)
        v = self._y_to_val(event.y)
        self.tracks[i, axis] = v
        self.pins[axis].add(i)
        self.pins[axis].add(0)
        self.pins[axis].add(self.T - 1)
        self.tracks[0, axis] = self.baseline[axis]
        self.tracks[-1, axis] = self.baseline[axis]
        self._rebuild_axis_from_pins(axis)
        self._draw_axis(axis)

    def _on_release(self, axis, _event):
        pass

    # reset/save
    def _reset_selected(self):
        a = self.active_axis
        self.tracks[:, a] = self.baseline[a]
        self.pins[a] = {0, self.T - 1}
        self._draw_axis(a)

    def _reset_all(self):
        for a in range(NUM_AXES):
            self.tracks[:, a] = self.baseline[a]
            self.pins[a] = {0, self.T - 1}
        self._redraw_all()

    def _save_and_play(self):
        traj = self.tracks.copy()
        traj[0, :] = self.baseline
        traj[-1, :] = self.baseline
        self.on_save_callback(
            traj,
            float(self.duration_s.get()),
            copy.deepcopy(self.pins),
            self.baseline.copy(),
        )
        self.destroy()


# ================================ MAIN APP ===================================


class AxisControlApp(tk.Tk):
    """Отправляет 61-элементный вектор с целевой частотой STREAM_RATE_HZ.

    Воспроизведение привязано к монотонному времени, а не к количеству
    вызовов Tk ``after``. Поэтому задержка GUI не растягивает паттерн.
    """

    def __init__(self):
        super().__init__()
        self.title("Axis Control (29 DOF + XY speed + yaw)")
        self.geometry("1200x980")
        self.tk.call("tk", "scaling", 2.0)

        default_font = tkfont.nametofont("TkDefaultFont")
        default_font.configure(size=16)
        self.option_add("*Font", default_font)

        # baseline для всего приложения — из INIT_BASELINE
        self.global_baseline = INIT_BASELINE.copy()

        # live vars
        self._make_vars()

        # pattern playback state
        self.pattern_active = False
        self.pattern_targets = None  # np.ndarray [T, 29]
        self.pattern_masks = None  # np.ndarray [T, 29]
        self.pattern_index = 0
        self.pattern_total = 0
        self.pattern_duration_s = 0.0
        self.pattern_started_at = None
        self._next_gui_refresh_at = 0.0

        # Absolute-deadline scheduler avoids accumulating callback execution
        # time in the UDP period. Slow callbacks skip deadlines, not pattern time.
        now = time.perf_counter()
        self._next_send_deadline = now
        self._status_started_at = now
        self._status_packet_count = 0

        # editor persistent state между открытиями
        self.editor_tracks = None  # np.ndarray [T,29]
        self.editor_pins = None  # list[set] per axis
        self.editor_duration_s = None  # float

        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(0, self._stream_vector)

    # -------------------------- model vars --------------------------
    def _make_vars(self):
        self.switch_vars = [tk.IntVar(value=0) for _ in range(NUM_AXES)]
        # слайдеры стартуют в нейтральной позе
        self.slider_vars = [
            tk.DoubleVar(value=float(self.global_baseline[i])) for i in range(NUM_AXES)
        ]
        self.speed_x_var = tk.DoubleVar(value=0.0)
        self.speed_y_var = tk.DoubleVar(value=0.0)
        self.angle_z_var = tk.DoubleVar(value=0.0)

    # -------------------------- UI --------------------------
    def _build_ui(self):
        header_font = ("Helvetica", 20, "bold")
        ttk.Label(self, text="Activate axis + set value", font=header_font).grid(
            row=0, column=0, columnspan=4, pady=(4, 12)
        )

        # pattern controls
        ctrl = ttk.Frame(self)
        ctrl.grid(row=1, column=0, columnspan=4, sticky="we", padx=6, pady=(0, 8))
        ttk.Button(
            ctrl, text="Open Pattern Editor", command=self._open_pattern_editor
        ).pack(side="left")
        self.play_label = ttk.Label(ctrl, text="(idle)")
        self.play_label.pack(side="left", padx=12)

        # grid of axes
        for i in range(NUM_AXES):
            row = i + 2
            ttk.Checkbutton(self, variable=self.switch_vars[i]).grid(
                row=row, column=0, sticky="w"
            )
            ttk.Label(self, text=JOINT_NAMES[i]).grid(row=row, column=1, sticky="w")
            ttk.Scale(
                self,
                from_=SLIDER_MIN,
                to=SLIDER_MAX,
                orient="horizontal",
                length=600,
                variable=self.slider_vars[i],
            ).grid(row=row, column=2, padx=10, pady=2, sticky="we")

        self.columnconfigure(2, weight=1)

        base = NUM_AXES + 3
        ttk.Label(self, text="Global motion", font=header_font).grid(
            row=base, column=0, columnspan=4, pady=(24, 8)
        )

        self._add_extra_slider(
            base + 1,
            "Speed X (−1‥1)",
            self.speed_x_var,
            EXTRA_SPEED_MIN,
            EXTRA_SPEED_MAX,
        )
        self._add_extra_slider(
            base + 2, "Speed Y (−1‥1)", self.speed_y_var, EXTRA_LR_MIN, EXTRA_LR_MAX
        )
        self._add_extra_slider(
            base + 3, "Yaw Z (−1‥1)", self.angle_z_var, EXTRA_ANGLE_MIN, EXTRA_ANGLE_MAX
        )

        # playback progress bar
        self.pb = ttk.Progressbar(
            self, orient="horizontal", mode="determinate", length=600
        )
        self.pb.grid(row=base + 4, column=0, columnspan=4, pady=(16, 8))

        ttk.Label(
            self,
            text=(
                f"UDP target: {STREAM_RATE_HZ:.0f} Hz ({STREAM_INTERVAL_MS} ms) "
                f"to {ADDR[0]}:{ADDR[1]}"
            ),
        ).grid(row=base + 5, column=0, columnspan=4, pady=(0, 6))

    def _add_extra_slider(self, row, text, var, vmin, vmax):
        ttk.Label(self, text=text).grid(row=row, column=1, sticky="w")
        ttk.Scale(
            self, from_=vmin, to=vmax, orient="horizontal", length=600, variable=var
        ).grid(row=row, column=2, padx=10, pady=2, sticky="we")

    # -------------------------- pattern editor --------------------------
    def _open_pattern_editor(self):
        # baseline редактора = глобальная нейтральная поза
        baseline = self.global_baseline.copy()

        def on_save(traj_T29, duration_s, pins, baseline_out):
            """Принимаем траекторию и сохраняем состояние редактора."""
            total = traj_T29.shape[0]
            self.pattern_targets = traj_T29.astype(np.float32)
            # маска: 1, где отличается от baseline
            eps = 1e-6
            self.pattern_masks = (
                np.abs(self.pattern_targets - baseline[None, :]) > eps
            ).astype(np.float32)
            self.pattern_total = int(total)
            self.pattern_index = 0
            self.pattern_duration_s = max(float(duration_s), 1.0e-6)
            self.pattern_started_at = time.perf_counter()
            self._next_gui_refresh_at = self.pattern_started_at
            self.pattern_active = True
            self.pb.configure(maximum=self.pattern_duration_s, value=0.0)
            self.play_label.configure(text=f"PLAY {duration_s:.2f}s ({total} frames)")

            # запомним состояние редактора для следующего открытия
            self.editor_tracks = traj_T29.copy()
            self.editor_pins = pins
            self.editor_duration_s = float(duration_s)
            self.global_baseline = baseline_out.copy()  # на случай будущей смены

        PatternEditor(
            self,
            baseline_vec=baseline,
            on_save_callback=on_save,
            initial_tracks=self.editor_tracks,
            initial_pins=self.editor_pins,
            initial_duration_s=self.editor_duration_s,
        )

    # -------------------------- packet construction --------------------------
    def _collect_live_vector(self):
        targets = np.asarray([v.get() for v in self.slider_vars], dtype=np.float32)
        mask = np.asarray([float(v.get()) for v in self.switch_vars], dtype=np.float32)
        extra = np.asarray(
            [self.speed_x_var.get(), self.speed_y_var.get(), self.angle_z_var.get()],
            dtype=np.float32,
        )
        # Canonical UDP representation: an inactive axis always carries target 0.
        visible_targets = np.where(mask >= 0.5, targets, 0.0)
        return np.concatenate([visible_targets, mask, extra]).astype(np.float32)

    def _collect_pattern_vector(self, frame_index, *, sync_ui):
        t = frame_index
        targets = self.pattern_targets[t]  # (29,)
        mask = self.pattern_masks[t]  # (29,)
        # Tk variable updates are visual only, so throttle them independently
        # from the 50 Hz UDP stream.
        if sync_ui:
            for i in range(NUM_AXES):
                self.slider_vars[i].set(float(targets[i]))
                self.switch_vars[i].set(int(mask[i] > 0.5))

        extra = np.array(
            [self.speed_x_var.get(), self.speed_y_var.get(), self.angle_z_var.get()],
            dtype=np.float32,
        )
        # Keep raw pattern values in the UI, but never transmit an inactive target.
        visible_targets = np.where(mask >= 0.5, targets, 0.0)
        vec = np.concatenate([visible_targets, mask, extra]).astype(np.float32)
        return vec

    # -------------------------- main loop --------------------------
    def _stream_vector(self):
        now = time.perf_counter()
        try:
            if self.pattern_active and self.pattern_targets is not None:
                elapsed_s = now - self.pattern_started_at
                self.pattern_index = pattern_frame_index(
                    elapsed_s,
                    self.pattern_duration_s,
                    self.pattern_total,
                )
                sync_ui = (
                    now >= self._next_gui_refresh_at
                    or elapsed_s >= self.pattern_duration_s
                )
                vec = self._collect_pattern_vector(self.pattern_index, sync_ui=sync_ui)
                sock.sendto(vec.tobytes(), ADDR)
                if sync_ui:
                    shown_s = min(elapsed_s, self.pattern_duration_s)
                    self.pb["value"] = shown_s
                    self.play_label.configure(
                        text=(
                            f"PLAY {shown_s:.2f}/{self.pattern_duration_s:.2f}s "
                            f"frame {self.pattern_index + 1}/{self.pattern_total}"
                        )
                    )
                    self._next_gui_refresh_at = now + GUI_REFRESH_PERIOD_S
                if elapsed_s >= self.pattern_duration_s:
                    self.pattern_active = False
                    self.play_label.configure(text="(idle)")
                    self.pb["value"] = self.pattern_duration_s
            else:
                vec = self._collect_live_vector()
                sock.sendto(vec.tobytes(), ADDR)
            self._log_stream_status(now)
        except OSError as ex:
            print(f"[UDP] send failed: {ex}")
        self._schedule_next_stream()

    def _log_stream_status(self, now):
        """Print one compact frequency/progress line instead of 50 vectors/s."""
        self._status_packet_count += 1
        elapsed_s = now - self._status_started_at
        if elapsed_s < STATUS_INTERVAL_S:
            return
        actual_hz = self._status_packet_count / elapsed_s
        if self.pattern_active and self.pattern_started_at is not None:
            pattern_elapsed_s = min(
                now - self.pattern_started_at, self.pattern_duration_s
            )
            state = f"pattern={pattern_elapsed_s:.2f}/{self.pattern_duration_s:.2f}s"
        else:
            state = "idle"
        print(f"[UDP] send_rate={actual_hz:.1f} Hz  {state}")
        self._status_started_at = now
        self._status_packet_count = 0

    def _schedule_next_stream(self):
        """Schedule against an absolute deadline so callback time does not drift."""
        self._next_send_deadline += STREAM_PERIOD_S
        now = time.perf_counter()
        if self._next_send_deadline <= now:
            missed = math.floor((now - self._next_send_deadline) / STREAM_PERIOD_S) + 1
            self._next_send_deadline += missed * STREAM_PERIOD_S
        delay_ms = max(1, math.ceil((self._next_send_deadline - now) * 1000.0))
        self.after(delay_ms, self._stream_vector)

    def _on_close(self):
        sock.close()
        self.destroy()


# ----------------------------------------------------------------------
def run_gui() -> None:
    AxisControlApp().mainloop()


if __name__ == "__main__":
    run_gui()
