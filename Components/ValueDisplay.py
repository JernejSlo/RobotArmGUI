import customtkinter

from Components.PositionControlPanel import PositionControlPanel
from Utils.color_theme import COLORS

import cv2
from PIL import Image, ImageTk

from Components.MovementSimulation import ArmSimulationView, DH
import numpy as np


class ValueDisplay(customtkinter.CTkFrame):
    """Robot guidance control area with visual display and sliders + teach/save UI."""

    def __init__(self, parent, running,db=None):
        super().__init__(parent, fg_color=COLORS["backgroundLight"])

        # 2-column layout: left (visual + joint controls), right (position control placeholder)
        self.grid(row=1, column=1, padx=(20, 20), pady=(10, 80), sticky="nsew")
        self.grid_columnconfigure(0, weight=3)  # left
        self.grid_columnconfigure(1, weight=1)  # right
        self.grid_rowconfigure(0, weight=1)

        # =========================
        # LEFT SIDE (visual + sliders+values + save)
        # =========================
        self.left_panel = customtkinter.CTkFrame(self, fg_color="transparent")
        self.left_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.left_panel.grid_rowconfigure(0, weight=3)
        self.left_panel.grid_rowconfigure(1, weight=2)
        self.left_panel.grid_columnconfigure(0, weight=1)

        # --- Visualization ---
        self.visual_frame = customtkinter.CTkFrame(self.left_panel, fg_color="#303030")
        self.visual_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=(10, 5))
        self.visual_frame.grid_propagate(False)

        # --- Controls block (sliders + values + save row) ---
        self.controls_frame = customtkinter.CTkFrame(self.left_panel, fg_color="transparent")
        self.controls_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=(5, 10))
        self.controls_frame.grid_columnconfigure(0, weight=3)  # sliders
        self.controls_frame.grid_columnconfigure(1, weight=1)  # values
        self.controls_frame.grid_rowconfigure(0, weight=1)     # sliders/values area
        self.controls_frame.grid_rowconfigure(1, weight=0)     # save/send row

        # --- Sliders container (left) ---
        self.slider_frame = customtkinter.CTkFrame(self.controls_frame, fg_color="transparent")
        self.slider_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 12), pady=(0, 0))
        self.slider_frame.grid_columnconfigure(0, weight=1)

        # --- Values container (right) ---
        self.values_frame = customtkinter.CTkFrame(self.controls_frame, fg_color="transparent")
        self.values_frame.grid(row=0, column=1, sticky="nsew", padx=(0, 0), pady=(0, 0))
        self.values_frame.grid_columnconfigure(0, weight=1)

        # --- Bottom row: text input + Save + Send ---
        self.actions_frame = customtkinter.CTkFrame(self.controls_frame, fg_color="transparent")
        self.actions_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        self.actions_frame.grid_columnconfigure(0, weight=1)  # entry grows
        self.actions_frame.grid_columnconfigure(1, weight=0)
        self.actions_frame.grid_columnconfigure(2, weight=0)

        self.pose_name_var = customtkinter.StringVar(value="")
        self.pose_name_entry = customtkinter.CTkEntry(
            self.actions_frame,
            textvariable=self.pose_name_var,
            placeholder_text="Pose name (e.g., Home, Pickup, Drop)"
        )
        self.pose_name_entry.grid(row=0, column=0, sticky="ew", padx=(0, 10))

        self.save_button = customtkinter.CTkButton(
            self.actions_frame,
            text="Save Pose",
            state="disabled"  # enabled when a callback is set
        )
        self.save_button.grid(row=0, column=1, sticky="ew", padx=(0, 10))

        # Send-to-robot button (you already had this; now it lives here)
        self.send_button = customtkinter.CTkButton(
            self.actions_frame,
            text="Send to Robot",
            state="disabled"
        )
        self.send_button.grid(row=0, column=2, sticky="ew")

        # bookkeeping for dynamically created sliders/labels
        self.mode = ""
        self.sliders = []
        self.labels = []
        self.slider_vars = []

        # Value labels (one per slider). Created/updated by create_value_labels().
        self.value_labels = []

        # ===== Simulator =====
        self.dh = [
            DH(a=0.0, alpha=np.deg2rad(90), d=0.0725, theta_offset=0.0),
            DH(a=0.185, alpha=np.deg2rad(0), d=0.0, theta_offset=np.deg2rad(-20)),
            DH(a=0.1175, alpha=np.deg2rad(180), d=0.0, theta_offset=np.deg2rad(-120)),
            DH(a=0.08, alpha=np.deg2rad(0), d=0.0, theta_offset=np.deg2rad(-60)),
        ]
        self.sim = ArmSimulationView(parent=self.visual_frame, dh_params=self.dh, dof=6, show_frames=True)
        self.sim.pack(fill="both", expand=True)
        self.sim.set_joint_angles([0.0, 130.0, 30.0, 90.0, 60.0, 60.0], redraw=True)

        # =========================
        # RIGHT SIDE
        # =========================
        self.position_panel = PositionControlPanel(
            self,
            db=db,  # pass the RobotPositionsDB from your app
            on_preview=None,
            on_home=None,
            on_set_pose=None,
        )
        self.position_panel.grid(row=0, column=1, sticky="nsew", padx=(10, 0), pady=10)

        # Camera stuff (unchanged)
        self.camera_active = False

    # -----------------------------
    # Wiring callbacks (App provides these)
    # -----------------------------
    def set_send_callback(self, callback):
        """Attach callback to Send button and enable it when pending values exist."""
        self.send_button.configure(command=callback)
        # leave disabled until sliders move; App will enable it

    def set_save_callback(self, callback):
        """
        callback signature recommended: callback(pose_name: str)
        """
        self.save_button.configure(command=callback, state="normal")

    def get_pose_name(self) -> str:
        return self.pose_name_var.get().strip()

    # -----------------------------
    # Value labels next to sliders
    # -----------------------------
    def create_value_labels(self, names):
        """Create the numeric labels aligned with the slider list."""
        # clear old
        for w in self.values_frame.winfo_children():
            w.destroy()
        self.value_labels = []

        for i, name in enumerate(names):
            row = customtkinter.CTkFrame(self.values_frame, fg_color="transparent")
            row.grid(row=i, column=0, sticky="ew", pady=(6, 6))
            row.grid_columnconfigure(0, weight=1)

            lbl = customtkinter.CTkLabel(row, text="0.00", anchor="e")
            lbl.grid(row=0, column=0, sticky="e")

            self.value_labels.append(lbl)

    def update_value_labels(self, values):
        """Update numeric labels to reflect current slider values."""
        for i, v in enumerate(values):
            if i < len(self.value_labels):
                self.value_labels[i].configure(text=f"{v:.2f}")

    # -----------------------------
    # Existing camera methods (unchanged)
    # -----------------------------
    def show_camera_feed(self):
        self.cap = cv2.VideoCapture(0)
        self.camera_active = True

        self.camera_label = customtkinter.CTkLabel(self.slider_frame, text="")
        self.camera_label.pack()

        def update_frame():
            if not self.camera_active:
                return

            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                img = img.resize((300, 200))
                photo = ImageTk.PhotoImage(image=img)
                self.camera_label.configure(image=photo, text="")
                self.camera_label.image = photo

            self.after(200, update_frame)

        update_frame()

    def stop_camera_feed(self):
        self.camera_active = False
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
