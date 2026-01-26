# ArmSimulationView.py
# A drop-in 3D simulator/preview widget for a serial robotic arm (FK-based)
#
# Dependencies: numpy, matplotlib
# - pip install numpy matplotlib
#
# Embeds a Matplotlib 3D axis into a (custom)Tkinter frame and draws a link-by-link
# visualization from DH parameters + joint angles. Also renders a translucent target
# pose (for XYZ/RPY control preview) and can animate trajectories using Tk's event loop.
#
# Example usage (inside your ValueDisplay or any CTkFrame):
#     from ArmSimulationView import ArmSimulationView, DH
#     dh = [
#         DH(a=0.0, alpha=np.deg2rad(90), d=0.10, theta_offset=0.0),
#         DH(a=0.25, alpha=0.0,         d=0.0,  theta_offset=0.0),
#         DH(a=0.20, alpha=0.0,         d=0.0,  theta_offset=0.0),
#         DH(a=0.0,  alpha=np.deg2rad(90), d=0.08, theta_offset=0.0),
#         DH(a=0.0,  alpha=-np.deg2rad(90), d=0.0, theta_offset=0.0),
#         DH(a=0.0,  alpha=0.0,         d=0.10, theta_offset=0.0),
#     ]
#     sim = ArmSimulationView(parent=self.visual_frame, dh_params=dh, dof=6)
#     sim.pack(fill="both", expand=True)
#     sim.set_joint_angles([0, 0, 0, 0, 0, 0])
#     # Preview an XYZ+RPY target (without IK):
#     sim.set_target_pose_xyzrpy(x=0.35, y=0.0, z=0.25, roll=0, pitch=np.deg2rad(90), yaw=0)
#     # Animate a small trajectory
#     import numpy as np
#     path = [np.deg2rad([0, 20*t, 10*np.sin(t), 0, 0, 0]) for t in np.linspace(0, 1, 60)]
#     sim.animate_trajectory(path, interval_ms=30)

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

import tkinter as tk

from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import customtkinter as ctk

    BaseFrame = ctk.CTkFrame
except Exception:
    BaseFrame = tk.Frame  # fallback to standard Tk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


@dataclass
class DH:
    """Denavit–Hartenberg parameters for one joint (standard DH)."""
    a: float
    alpha: float  # radians
    d: float
    theta_offset: float = 0.0  # radians


def _rotz(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)


def _rotx(alpha: float) -> np.ndarray:
    c, s = np.cos(alpha), np.sin(alpha)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]], dtype=float)


def _transx(a: float) -> np.ndarray:
    T = np.eye(4)
    T[0, 3] = a
    return T


def _transz(d: float) -> np.ndarray:
    T = np.eye(4)
    T[2, 3] = d
    return T


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Z-Y-X (yaw-pitch-roll) intrinsic convention -> rotation matrix."""
    Rz = _rotz(yaw)
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch), 0],
                   [0, 1, 0, 0],
                   [-np.sin(pitch), 0, np.cos(pitch), 0],
                   [0, 0, 0, 1]], dtype=float)
    Rx = _rotx(roll)
    return Rz @ Ry @ Rx


class ArmSimulationView(BaseFrame):

    """
    A
    3
    D
    arm
    simulator / preview
    widget.

    - Displays
    a
    serial
    robot
    using
    standard
    DH
    parameters.
    - Accepts
    joint
    angles( in radians) and renders
    the
    chain and coordinate
    frames.
    - Shows
    an
    optional
    translucent
    target
    pose
    for XYZ + RPY preview.
        - Can
        animate
        a
        sequence
        of
        joint
        angle
        waypoints
        using
        Tk
        's event loop.

    Public
    API:
    - set_joint_angles(joint_angles: Sequence[float], redraw = True)
    - set_target_pose_xyzrpy(x, y, z, roll, pitch, yaw)
    - clear_target()
    - animate_trajectory(angles_path: Sequence[Sequence[float]], interval_ms = 30)
    - set_workspace(limits: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]])
    """

    def __init__(self,
                 parent,
                 dh_params: Sequence[DH],
                 dof: Optional[int] = None,
                 base_T: Optional[np.ndarray] = None,
                 bg="#0f0f10",
                 grid=True,
                 show_frames=True,
                 link_radius: float = 0.01,
                 # --- Optional joint calibration mapping (applied to inputs before FK) ---
                 # Each joint i is mapped as: q_mapped = clamp(min_i, max_i, offset_i + scale_i * q_input)
                 # Defaults implement your described behavior: input 0..180° -> physical 30..150°.
                 joint_offset_deg: Optional[Sequence[float]] = None,
                 joint_scale: Optional[Sequence[float]] = None,
                 joint_min_deg: Optional[Sequence[float]] = None,
                 joint_max_deg: Optional[Sequence[float]] = None,
                 **kwargs):
        super().__init__(parent, **kwargs)
        # customtkinter frames use `fg_color`, Tk frames use `bg`
        try:
            self.configure(fg_color=bg)
        except Exception:
            try:
                self.configure(bg=bg)
            except Exception:
                pass

        self.dh: List[DH] = list(dh_params)
        self.dof = dof or len(self.dh)

        # --- Gripper visualization controls ---
        self.gripper_roll = 0.0  # radians (rotation about tool Z)
        self.gripper_open = 0.5  # 0..1 (0=closed, 1=open)

        # Geometry (meters) – tune to your CAD later
        self.gripper_finger_len = 0.05
        self.gripper_finger_base = 0.015
        self.gripper_max_open = 0.04  # full jaw gap (meters)

        # --- Joint mapping defaults (your: 0..180 maps to 30..150, min=30) ---
        # If you pass custom arrays, they must match dof.
        default_offset = [120] * self.dof
        default_scale = [120.0 / 180.0] * self.dof  # 2/3
        default_min = [0.0] * self.dof
        default_max = [180.0] * self.dof

        off = default_offset if joint_offset_deg is None else list(joint_offset_deg)
        scl = default_scale if joint_scale is None else list(joint_scale)
        jmin = default_min if joint_min_deg is None else list(joint_min_deg)
        jmax = default_max if joint_max_deg is None else list(joint_max_deg)

        if not (len(off) == len(scl) == len(jmin) == len(jmax) == self.dof):
            raise ValueError("Joint mapping arrays must all match DOF")

        self._joint_offset_rad = np.deg2rad(np.array(off, dtype=float))
        self._joint_scale = np.array(scl, dtype=float)
        self._joint_min_rad = np.deg2rad(np.array(jmin, dtype=float))
        self._joint_max_rad = np.deg2rad(np.array(jmax, dtype=float))

        self.base_T = np.eye(4) if base_T is None else base_T.copy()

        self.grid_on = grid
        self.show_frames = show_frames
        self.link_radius = link_radius

        self.joint_angles = np.zeros(self.dof, dtype=float)  # radians (arm joints only)

        # --- Tool / gripper visualization (orientation handled AFTER arm FK) ---
        # tool_offset is expressed in the flange frame (meters).
        self.tool_offset = np.array([0.0, 0.0, 0.0], dtype=float)
        # tool_rpy are intrinsic Z-Y-X (yaw-pitch-roll) in radians.
        self.tool_rpy = np.array([0.0, 0.0, 0.0], dtype=float)
        self._tool_artists: List = []
        self._real_artists: List = []
        self._preview_artists: List = []

        self._trajectory: List[np.ndarray] = []
        self._anim_index = 0
        self._anim_running = False

        # Matplotlib Figure embedded in Tk
        self.fig = Figure(figsize=(9, 8), dpi=100)


        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # Default workspace limits (meters)
        self.xlim = (-0.3, 0.3)
        self.ylim = (-0.3, 0.3)
        self.zlim = (0.0, 0.45)

        # Per-link visual radii (meters)
        # Index corresponds to link i -> between joint i and i+1
        self.link_radii = [
            self.link_radius * 5.0,  # base column
            self.link_radius * 1.5,  # shoulder / second link
            self.link_radius * 1.0,  # rest
            self.link_radius * 1.0,
            self.link_radius * 1.0,
        ]

        # --- Preview arm state (ghost) ---
        self.preview_angles = None  # type: Optional[np.ndarray]
        self.preview_enabled = True

        # Colors/opacity (you can tune)
        self.real_alpha = 0.95
        self.preview_alpha = 0.70

        # Use explicit RGBA so it is stable across matplotlib defaults
        self.real_color = (0.20, 0.80, 0.35, self.real_alpha)  # green-ish
        self.preview_color = (0.95, 0.15, 0.15, self.preview_alpha)  # red

        # Visual handles
        self._link_lines: List = []
        self._joint_markers: List = []
        self._frames_quivers: List = []
        self._target_artists: List = []

        self._style_axes()
        self._redraw_all()

    def set_preview_joint_angles(self, joint_angles: Sequence[float], redraw: bool = True):
        arr = np.array(joint_angles, dtype=float)
        if arr.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} joint angles, got {arr.shape[0]}")
        self.preview_angles = arr
        if redraw:
            self._redraw_all()

    def _clear_tool_artists(self):
        for a in self._tool_artists:
            try:
                a.remove()
            except Exception:
                pass
        self._tool_artists.clear()

    def clear_preview(self, redraw: bool = True):
        self.preview_angles = None
        if redraw:
            self._redraw_all()

    def _unit_vector(self, v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else v

    def _rotation_from_z(self, direction: np.ndarray) -> np.ndarray:
        """
        Returns a 3x3 rotation matrix R such that R @ [0,0,1] = direction (unit).
        """
        z = np.array([0.0, 0.0, 1.0])
        d = self._unit_vector(direction)

        # If already aligned
        if np.allclose(d, z):
            return np.eye(3)
        if np.allclose(d, -z):
            # 180 deg rotation around x (arbitrary orthogonal axis)
            return np.array([[1, 0, 0],
                             [0, -1, 0],
                             [0, 0, -1]], dtype=float)

        v = np.cross(z, d)
        s = np.linalg.norm(v)
        c = np.dot(z, d)

        vx = np.array([[0, -v[2], v[1]],
                       [v[2], 0, -v[0]],
                       [-v[1], v[0], 0]], dtype=float)

        # Rodrigues' rotation formula
        R = np.eye(3) + vx + (vx @ vx) * ((1 - c) / (s * s))
        return R

    def _cylinder_mesh(self, p0: np.ndarray, p1: np.ndarray, radius: float = 0.01, n_sides: int = 16):
        """
        Build a closed cylinder (with caps) between p0 and p1.
        Returns a list of polygon faces suitable for Poly3DCollection.
        """
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        axis = p1 - p0
        length = np.linalg.norm(axis)
        if length < 1e-9:
            return []

        R = self._rotation_from_z(axis)

        angles = np.linspace(0, 2 * np.pi, n_sides, endpoint=False)
        circle = np.stack([
            np.cos(angles) * radius,
            np.sin(angles) * radius,
            np.zeros_like(angles)
        ], axis=1)

        bottom = circle
        top = circle + np.array([0.0, 0.0, length])

        bottom_w = (R @ bottom.T).T + p0
        top_w = (R @ top.T).T + p0

        faces = []

        # --- Side faces ---
        for i in range(n_sides):
            j = (i + 1) % n_sides
            faces.append([
                bottom_w[i],
                bottom_w[j],
                top_w[j],
                top_w[i],
            ])

        # --- Bottom cap ---
        center_bottom = p0
        for i in range(n_sides):
            j = (i + 1) % n_sides
            faces.append([
                center_bottom,
                bottom_w[j],
                bottom_w[i],
            ])

        # --- Top cap ---
        center_top = p1
        for i in range(n_sides):
            j = (i + 1) % n_sides
            faces.append([
                center_top,
                top_w[i],
                top_w[j],
            ])

        return faces

    def set_gripper_roll(self, roll_rad: float, redraw: bool = True):
        self.gripper_roll = float(roll_rad)
        if redraw:
            self._redraw_all()

    def set_gripper_open(self, open01: float, redraw: bool = True):
        self.gripper_open = float(np.clip(open01, 0.0, 1.0))
        if redraw:
            self._redraw_all()

    def _trans(self, x, y, z):
        T = np.eye(4)
        T[:3, 3] = [x, y, z]
        return T

    def _draw_gripper(self, T_tcp: np.ndarray):
        """
        Draw a simple 2-finger gripper at TCP.
        Finger motion: opens/closes symmetrically along TCP Y.
        Finger direction: along TCP X (forward).
        Roll: rotates around TCP Z.
        """
        # Clear old tool artists was already done in your redraw; append into self._tool_artists

        # Apply gripper roll about TCP Z
        T_roll = _rotz(self.gripper_roll)
        T_g = T_tcp @ T_roll

        jaw_gap = self.gripper_max_open * self.gripper_open
        half_gap = jaw_gap / 2.0

        L = self.gripper_finger_len
        base = self.gripper_finger_base

        # Two fingers start a bit forward of TCP origin
        # Finger centerlines in gripper frame:
        p0 = np.array([base, 0, 0, 1.0])
        p1 = np.array([base + L, 0, 0, 1.0])

        # Top finger is +Y, bottom is -Y in gripper frame
        for sign in (+1, -1):
            T_f = T_g @ self._trans(0.0, sign * half_gap, 0.0)
            a = (T_f @ p0)[:3]
            b = (T_f @ p1)[:3]
            ln, = self.ax.plot([a[0], b[0]], [a[1], b[1]], [a[2], b[2]], linewidth=4)
            self._tool_artists.append(ln)

        # Optional: small “palm” crossbar
        left = (T_g @ np.array([base, +half_gap, 0, 1.0]))[:3]
        right = (T_g @ np.array([base, -half_gap, 0, 1.0]))[:3]
        palm, = self.ax.plot([left[0], right[0]], [left[1], right[1]], [left[2], right[2]], linewidth=3, alpha=0.9)
        self._tool_artists.append(palm)

    # ----------------- Public API -----------------
    def set_workspace(self, limits: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]):
        self.xlim, self.ylim, self.zlim = limits
        self._style_axes()
        self._redraw_all()

    def set_joint_angles(self, joint_angles: Sequence[float], redraw: bool = True):
        arr = np.array(joint_angles, dtype=float)
        if arr.shape[0] != self.dof:
            raise ValueError(f"Expected {self.dof} joint angles, got {arr.shape[0]}")
        self.joint_angles = arr
        if redraw:
            self._redraw_all()

    def set_target_pose_xyzrpy(self, x: float, y: float, z: float,
                               roll: float, pitch: float, yaw: float):
        T = np.eye(4)
        T[:3, :3] = (rpy_to_rot(roll, pitch, yaw))[:3, :3]
        T[:3, 3] = [x, y, z]
        self._draw_target_pose(T)
        self.canvas.draw_idle()

    def set_tool_offset(self, x: float, y: float, z: float, redraw: bool = True):
        """
    Set
    fixed
    TCP
    offset
    relative
    to
    the
    flange
    frame(meters).
    """
        self.tool_offset = np.array([x, y, z], dtype=float)
        if redraw:
            self._redraw_all()

    def set_tool_rpy(self, roll: float, pitch: float, yaw: float, redraw: bool = True):
        """
    Set
    tool
    orientation
    relative
    to
    the
    flange(radians).
    """
        self.tool_rpy = np.array([roll, pitch, yaw], dtype=float)
        if redraw:
            self._redraw_all()

    def clear_target(self):
        for a in self._target_artists:
            try:
                a.remove()
            except Exception:
                pass
        self._target_artists.clear()
        self.canvas.draw_idle()

    def animate_trajectory(self, angles_path: Sequence[Sequence[float]], interval_ms: int = 30):
        """
    Animate
    a
    list
    of
    joint
    configurations( in radians)."""
        if not angles_path:
            return
        self._trajectory = [np.array(a, dtype=float) for a in angles_path]
        self._anim_index = 0
        self._anim_running = True
        self._step_animation(interval_ms)

    def stop_animation(self):
        self._anim_running = False

    # ----------------- Internals -----------------
    def _style_axes(self):
        self.ax.cla()
        self.ax.set_xlim(*self.xlim)
        self.ax.set_ylim(*self.ylim)
        self.ax.set_zlim(*self.zlim)
        self.ax.set_box_aspect((self.xlim[1]-self.xlim[0],
                                self.ylim[1]-self.ylim[0],
                                self.zlim[1]-self.zlim[0]))
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.view_init(elev=25, azim=-60)
        if not self.grid_on:
            self.ax.grid(False)
        self.ax.set_facecolor((14/255, 14/255, 16/255))
        self.fig.patch.set_facecolor((14/255, 14/255, 16/255))

    def _map_joint_angles(self, q_in: np.ndarray) -> np.ndarray:
        q = q_in.copy()
        q[1:] = np.clip(q[1:], self._joint_min_rad[1:], self._joint_max_rad[1:])
        return q

    def _fk_chain(self, q: np.ndarray) -> List[np.ndarray]:
        """
    Compute
    forward - kinematics
    transforms
    for each joint frame and the end-effector.
    Returns
    a
    list[T0, T1, ..., Tn]
    where
    T0 is base_T and Tn is EE.
    """
        q_m = self._map_joint_angles(q)
        Ts = [self.base_T.copy()]
        T = self.base_T.copy()
        for i, (dh, qi) in enumerate(zip(self.dh, q_m)):
            # Standard DH: T_i = RotZ(theta_i+offset) * TransZ(d) * TransX(a) * RotX(alpha)
            T_i = _rotz(qi + dh.theta_offset) @ _transz(dh.d) @ _transx(dh.a) @ _rotx(dh.alpha)
            T = T @ T_i
            Ts.append(T)
        return Ts

    def _draw_frames(self, Ts: List[np.ndarray], scale: float = 0.06):
        # Clear old frames
        for quiv in self._frames_quivers:
            try:
                quiv.remove()
            except Exception:
                pass
        self._frames_quivers.clear()

        for T in Ts:
            o = T[:3, 3]
            x = T[:3, 0] * scale
            y = T[:3, 1] * scale
            z = T[:3, 2] * scale
            # Draw small axes at each frame origin
            qx = self.ax.quiver(o[0], o[1], o[2], x[0], x[1], x[2])
            qy = self.ax.quiver(o[0], o[1], o[2], y[0], y[1], y[2])
            qz = self.ax.quiver(o[0], o[1], o[2], z[0], z[1], z[2])
            self._frames_quivers.extend([qx, qy, qz])

    def _draw_chain(self, points: np.ndarray, rgba, joint_s=70, ee_s=90):
        """
        Draw one arm chain (links as cylinders + joints as scatter).
        Returns list of artists created.
        """
        artists = []

        n_sides = 14
        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]

            radius = (
                self.link_radii[i]
                if i < len(self.link_radii)
                else self.link_radius
            )

            faces = self._cylinder_mesh(p0, p1, radius=radius, n_sides=n_sides)
            if not faces:
                continue

            poly = Poly3DCollection(faces, alpha=rgba[3])
            poly.set_facecolor(rgba)
            poly.set_edgecolor((0, 0, 0, 0.20))
            self.ax.add_collection3d(poly)
            artists.append(poly)

        xs, ys, zs = points[:, 0], points[:, 1], points[:, 2]
        joint = self.ax.scatter(xs[:-1], ys[:-1], zs[:-1], s=joint_s, alpha=rgba[3])
        ee = self.ax.scatter([xs[-1]], [ys[-1]], [zs[-1]], s=ee_s, alpha=rgba[3])
        artists.extend([joint, ee])

        return artists

    def _clear_artists(self, artists_list: List):
        for a in artists_list:
            try:
                a.remove()
            except Exception:
                pass
        artists_list.clear()

    def _tool_T(self) -> np.ndarray:
        """
        Tool
        transform
        from flange to
        TCP.

        Convention
        used
        here:
        T_tool = Trans(tool_offset in flange
        frame) *Rot(tool_rpy)

        This
        makes
        the
        offset
        easy
        to
        measure in CAD
        from the flange

        origin.
        If
        you
        prefer
        the
        offset in the
        tool
        's rotated frame, swap the order.
        """
        T = np.eye(4)
        T[:3, 3] = self.tool_offset
        R = rpy_to_rot(self.tool_rpy[0], self.tool_rpy[1], self.tool_rpy[2])
        return T @ R

    def _draw_target_pose(self, T: np.ndarray, size: float = 0.07):
        self.clear_target()
        o = T[:3, 3]
        x = T[:3, 0] * size
        y = T[:3, 1] * size
        z = T[:3, 2] * size
        # Coordinate triad for the target
        qx = self.ax.quiver(o[0], o[1], o[2], x[0], x[1], x[2], alpha=0.6)
        qy = self.ax.quiver(o[0], o[1], o[2], y[0], y[1], y[2], alpha=0.6)
        qz = self.ax.quiver(o[0], o[1], o[2], z[0], z[1], z[2], alpha=0.6)
        # Small sphere-ish marker for target origin (approx via scatter)
        marker = self.ax.scatter([o[0]], [o[1]], [o[2]], s=60, alpha=0.6)
        self._target_artists.extend([qx, qy, qz, marker])

    def _redraw_all(self):
        self._style_axes()

        # Clear old drawings
        self._clear_artists(self._real_artists)
        self._clear_artists(self._preview_artists)
        self._clear_tool_artists()  # <<< ADD THIS

        # --- Real arm ---
        Ts_real = self._fk_chain(self.joint_angles)
        pts_real = np.array([T[:3, 3] for T in Ts_real])
        self._real_artists.extend(self._draw_chain(pts_real, self.real_color))

        if self.show_frames:
            self._draw_frames(Ts_real)

        # Tool/TCP for REAL arm only (keeps UI sane)
        T_flange = Ts_real[-1]
        T_tcp = T_flange @ self._tool_T()

        p_f = T_flange[:3, 3]
        p_t = T_tcp[:3, 3]

        tool_line, = self.ax.plot([p_f[0], p_t[0]], [p_f[1], p_t[1]], [p_f[2], p_t[2]], linewidth=2)
        tcp_marker = self.ax.scatter([p_t[0]], [p_t[1]], [p_t[2]], s=40)
        self._tool_artists.extend([tool_line, tcp_marker])

        if self.show_frames:
            size = 0.06
            o = p_t
            x = T_tcp[:3, 0] * size
            y = T_tcp[:3, 1] * size
            z = T_tcp[:3, 2] * size
            qx = self.ax.quiver(o[0], o[1], o[2], x[0], x[1], x[2], alpha=0.8)
            qy = self.ax.quiver(o[0], o[1], o[2], y[0], y[1], y[2], alpha=0.8)
            qz = self.ax.quiver(o[0], o[1], o[2], z[0], z[1], z[2], alpha=0.8)
            self._tool_artists.extend([qx, qy, qz])

        # Draw gripper for REAL arm only (optional; keep consistent)
        self._draw_gripper(T_tcp)

        # --- Preview arm (ghost), drawn AFTER so it overlays nicely ---
        if self.preview_angles is not None and self.preview_enabled:
            Ts_prev = self._fk_chain(self.preview_angles)
            pts_prev = np.array([T[:3, 3] for T in Ts_prev])
            self._preview_artists.extend(self._draw_chain(pts_prev, self.preview_color, joint_s=55, ee_s=70))

        self.canvas.draw_idle()

    def _step_animation(self, interval_ms: int):
        if not self._anim_running:
            return
        if self._anim_index >= len(self._trajectory):
            self._anim_running = False
            return
        self.set_joint_angles(self._trajectory[self._anim_index], redraw=True)
        self._anim_index += 1
        # Schedule next frame
        self.after(interval_ms, lambda: self._step_animation(interval_ms))
