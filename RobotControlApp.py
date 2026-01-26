import tkinter as tk
import customtkinter
import random
import threading

import numpy as np
import serial.tools.list_ports
from Components.RobotSettings import RobotSettings
from Components.Graph import GraphComponent
from Components.OutputTerminal import TerminalOutput
from Components.Sidebar import Sidebar
from Components.UpperPanel import UpperPanel
from Components.ValueDisplay import ValueDisplay
from Components.BottomTabBar import BottomTabBar

from RobotArm import RobotArm
from Utils.DatabaseManager import RobotPositionsDB
from Utils.GenerationAndDisplayUtils import GenerationAndDisplayUtils
from Utils.color_theme import COLORS

import screeninfo

customtkinter.set_appearance_mode("Dark")
customtkinter.set_default_color_theme("blue")

class RobotControlApp(customtkinter.CTk,GenerationAndDisplayUtils):
    def __init__(self):
        self.db = RobotPositionsDB(db_path="data/robot_positions.sqlite")
        self.selected_com_port = None

        self.home_pos = [0.0, 160.0, 60.0, 120.0, 90.0, 90.0]

        self.selected_mode = ""
        super().__init__()
        self.configure(fg_color=COLORS["backgroundLight"], bg_color=COLORS["backgroundLight"])

        self.default_color = COLORS["backgroundLight"]
        self.active_color = COLORS["backgroundDark"]
        self.hover_color = COLORS["hover"]
        self.text_color = COLORS["lg_text"]

        self.title("Calibration App.py")

        screen = screeninfo.get_monitors()[0]
        #width, height = screen.width, screen.height
        #self.geometry(f"{width}x{height-300}+0+0")
        self.state('zoomed')
        self.update()
        self.running = False
        self.graph_enabled = False

        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=2)
        self.grid_rowconfigure(2, weight=1)

        # Paned container for resizable layout
        self.paned = tk.PanedWindow(self, orient=tk.HORIZONTAL, sashwidth=6, bg=self.default_color, showhandle=True)
        self.paned.grid(row=0, column=0, rowspan=3, columnspan=2, sticky="nsew")

        # === Sidebar container (inside paned window) ===
        self.sidebar_container = customtkinter.CTkFrame(self.paned, corner_radius=0)
        self.sidebar = Sidebar(self.sidebar_container, self.update_title, self.change_scaling,
                               self.show_settings)
        self.sidebar.pack(fill="both", expand=True)
        self.paned.add(self.sidebar_container, minsize=140)

        # === Main Frame Split Vertically ===

        self.main_frame = customtkinter.CTkFrame(self.paned, fg_color=self.default_color,bg_color=self.default_color)
        self.paned.add(self.main_frame)

        # === Create a vertical PanedWindow inside main_frame ===
        self.vertical_pane = tk.PanedWindow(self.main_frame, orient=tk.VERTICAL, sashwidth=6, bg=self.default_color,
                                            showhandle=True,background=self.default_color)
        self.vertical_pane.pack(fill="both", expand=True)

        # === Upper and lower panels ===

        self.upper_panel = UpperPanel(self.vertical_pane,self.running, self.db)
        # Wire right panel button callbacks
        self.upper_panel.value_display.position_panel.on_set_pose = self.set_pose_from_db

        self.lower_panel = customtkinter.CTkFrame(
            self.vertical_pane,
            fg_color="transparent",
            height=200
        )

        self.vertical_pane.add(self.upper_panel)
        self.vertical_pane.add(self.lower_panel)
        # After the window is drawn, force a smaller terminal pane
        self.after(100, self._set_default_vertical_split)

        self.main_frame.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(0, weight=1)
        self.paned.add(self.main_frame, minsize=400)

        # === Inside Lower Panel (Graph + Terminal toggle) ===
        self.graph = GraphComponent(self.lower_panel,self.selected_mode)
        self.terminal = TerminalOutput(self.lower_panel)

        self.graph.pack_forget()
        self.terminal.grid(row=0, column=0, sticky="nsew")

        self.lower_panel.grid_rowconfigure(0, weight=1)
        self.lower_panel.grid_columnconfigure(0, weight=1)

        # === Bottom Tab Bar ===
        self.bottom_bar = BottomTabBar(self, self.show_terminal, self.show_graph)
        self.bottom_bar.grid(row=3, column=0, columnspan=2, sticky="ew")

        self.protocol("WM_DELETE_WINDOW", self.on_closing)



        self.robot_settings = RobotSettings(self, self.hide_settings)
        self.robot_settings.grid(row=0, column=1, rowspan=3, padx=20, pady=20, sticky="nsew")
        self.robot_settings.grid_remove()


        self.graph.grid_remove()

        self.selected_mode = ""
        self.show_terminal()

        self.pause_event = threading.Event()
        self.prompt_shown = False

        self.upper_panel.content_box.grid_remove()
        self.upper_panel.content_box.grid_remove()


        self.pending_mode = None  # "Angles" or "Simple"
        self.pending_values = None  # list[float]

        self.robot_arm = RobotArm(terminal=self.terminal)

        # Wire buttons (ValueDisplay owns the UI elements)
        self.upper_panel.value_display.set_send_callback(self.send_pending_to_robot)
        self.upper_panel.value_display.set_save_callback(self.save_current_pose)

        # Wire right-side panel callbacks
        self._wire_position_panel_callbacks()
        #self.show_com_port_popup()

    def _wire_position_panel_callbacks(self):
        panel = getattr(self.upper_panel.value_display, "position_panel", None)
        if panel is None:
            return

        panel.on_preview = self.preview_pose_from_db
        panel.on_set_pose = self.set_pose_from_db
        panel.on_home = self.home_pose

    def home_pose(self):
        pose = self.db.get_position_by_id(1)
        if pose is None:
            self.terminal.log("Home pose (id=1) not found in database.")
            return

        self.set_pose_from_db(pose)
        self.terminal.log("Loaded Home pose into sliders.")

    def preview_pose_from_db(self, pose):
        # Ensure Angles mode UI exists (so sim is in correct context)
        if self.selected_mode != "Angles":
            self.update_title("Angles")

        values = [
            float(pose.j_base),
            float(pose.j_bottom),
            float(pose.j_2),
            float(pose.j_3),
            float(pose.j_4),
            float(pose.j_gripper),
        ]

        vdisp = self.upper_panel.value_display

        # Preview affects ONLY preview ghost (no slider changes required)
        try:
            import numpy as np
            vdisp.sim.set_preview_joint_angles(np.deg2rad(values), redraw=True)
            self.terminal.log(f"Previewing pose: {pose.name} (id={pose.id})")
        except Exception:
            self.terminal.log("Preview failed (sim not ready).")

    def set_pose_from_db(self, pose):
        """
        Called when 'Set Pose' is clicked in the PositionControlPanel.
        Sets sliders to the pose joint values (index-aligned).
        Assumes Angles mode sliders order:
          [J_BASE, J_BOTTOM, J_2, J_3, J_4, J_GRIPPER]
        """
        # 1) Ensure we are in Angles mode (otherwise sliders might be XYZ etc.)
        if self.selected_mode != "Angles":
            # Switch UI mode first
            self.update_title("Angles")  # triggers set_mode("Angles")

        values = [
            float(pose.j_base),
            float(pose.j_bottom),
            float(pose.j_2),
            float(pose.j_3),
            float(pose.j_4),
            float(pose.j_gripper),
        ]

        # 2) Set slider vars (this updates slider visuals)
        vdisp = self.upper_panel.value_display

        # Safety: ensure sliders exist and count matches
        if not hasattr(vdisp, "slider_vars") or len(vdisp.slider_vars) < len(values):
            self.terminal.log("Sliders not ready; cannot set pose.")
            return

        for i, val in enumerate(values):
            vdisp.slider_vars[i].set(val)

        # 3) Update pending state so Send works
        self.pending_mode = "Angles"
        self.pending_values = values

        # 4) Enable Send button
        if hasattr(vdisp, "send_button") and vdisp.send_button is not None:
            try:
                vdisp.send_button.configure(state="normal")
            except Exception:
                pass

        # 5) Update value labels (if you implemented them)
        if hasattr(vdisp, "update_value_labels"):
            try:
                vdisp.update_value_labels(values)
            except Exception:
                pass

        # 6) Update simulator preview (optional but recommended)
        try:
            import numpy as np
            if hasattr(vdisp, "sim") and vdisp.sim is not None:
                vdisp.sim.set_joint_angles(np.deg2rad(values), redraw=True)
        except Exception:
            pass

        self.terminal.log(f"Loaded pose into sliders: {pose.name} (id={pose.id})")

        # Clear preview after setting real pose
        try:
            self.upper_panel.value_display.sim.clear_preview(redraw=True)
        except Exception:
            pass

    def _set_default_vertical_split(self):
        try:
            total_h = self.vertical_pane.winfo_height()
            if total_h <= 1:
                return

            terminal_h = 500  # desired default terminal height in px
            # sash position is from top: set it so lower pane is ~terminal_h
            self.vertical_pane.sash_place(0, 0, max(100, total_h - terminal_h))
        except Exception:
            pass

    def save_current_pose(self):
        name = self.upper_panel.value_display.get_pose_name()
        if not name:
            self.terminal.log("Pose name is empty.")
            return

        if self.pending_mode != "Angles" or not self.pending_values:
            self.terminal.log("Switch to Angles mode and move sliders first.")
            return

        pose_id = self.db.add_position(name=name, joints=self.pending_values[:6])
        self.terminal.log(f"Saved pose '{name}' with id={pose_id}")

    def send_command(self, cmd: str):
        if self.robot_arm.ser and self.robot_arm.ser.is_open:
            try:
                self.robot_arm.ser.write((cmd + '\n').encode())
                print(f"Sent: {cmd}")
            except Exception as e:
                print(f"Error sending command: {e}")

    def send_pending_to_robot(self):
        """Send the most recently selected slider values exactly once."""
        if not self.pending_mode or self.pending_values is None:
            self.terminal.log("Nothing to send yet.")
            return

        mode = self.pending_mode
        values = self.pending_values

        mode_prefix = f"MODE:{mode}"

        if mode == "Simple":
            keys = ["X", "Y", "Z", "Roll", "Pitch", "Yaw"]
        elif mode == "Angles":
            keys = [f"Joint{i + 1}" for i in range(6)]
        else:
            self.terminal.log(f"Unknown pending mode: {mode}")
            return

        payload = " ".join(f"{k}={v:.2f}" for k, v in zip(keys, values))
        full_command = f"{mode_prefix} {payload}"

        self.send_command(full_command)
        self.terminal.log(f"Sent command: {full_command}")

    def set_mode(self, mode):
        """ Switch between Cartesian, Joint, and Continuous (AI-guided) control """
        self.mode = mode
        print(mode)
        # Always stop camera if switching mode
        if hasattr(self.upper_panel.value_display, "stop_camera_feed"):
            self.upper_panel.value_display.stop_camera_feed()
        # Clear slider frame
        for widget in self.upper_panel.value_display.slider_frame.winfo_children():
            widget.destroy()
        # Recreate Send button after sliders
        """self.upper_panel.value_display.create_send_button(
            self.send_pending_to_robot
        )"""

        self.upper_panel.value_display.sliders = []
        self.upper_panel.value_display.labels = []
        self.upper_panel.value_display.slider_vars = []

        if mode == "Simple":
            names = ["X", "Y", "Z", "Attachment Rotation (Yaw)", "Attachment Roll (Roll)"]
            ranges = [
                (-250, 250),  # X (mm)  adjust
                (-250, 250),  # Y (mm)
                (0, 400),  # Z (mm)
                (0, 180),  # Base Heading (deg)
                (0, 180),  # Tool Roll (deg)
            ]
        elif mode == "Angles":
            names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Gripper Roll", "Gripper Open"]
            ranges = [(30, 180)] * 6
            ranges[0] = (0, 360)
            ranges[-1] = (70, 100)

            self.upper_panel.value_display.create_value_labels(names)

        elif mode == "Continuous":
            self.upper_panel.value_display.show_camera_feed()
            return
        else:
            return

        def slider_moved(_=None):
            # Update internal state only
            values = self.get_slider_values()
            self.pending_mode = mode
            self.pending_values = values

            # Optional: live preview only (no serial)
            if hasattr(self.upper_panel, "value_display") and hasattr(self.upper_panel.value_display, "sim"):
                try:
                    import numpy as np
                    if mode == "Angles":
                        self.upper_panel.value_display.sim.set_joint_angles(np.deg2rad(values))
                        self.upper_panel.value_display.update_value_labels(values)
                    elif mode == "Simple":
                        # optional: show target pose only (no IK yet)
                        x, y, z, roll, pitch, yaw = values
                        self.upper_panel.value_display.sim.set_target_pose_xyzrpy(
                            x=x / 1000.0, y=y / 1000.0, z=z / 1000.0,  # if your sliders are mm; remove if meters
                            roll=np.deg2rad(roll), pitch=np.deg2rad(pitch), yaw=np.deg2rad(yaw)
                        )
                    # Enable send button
                    btn = self.upper_panel.value_display.send_button
                    if btn.cget("state") == "disabled":
                        btn.configure(state="normal")

                except Exception as e:
                    # Keep UI resilient; do not crash on preview
                    pass

        def on_slider_release(event=None):
            # trigger once after the user lets go
            values = self.get_slider_values()  # current slider values
            # e.g., start an animation/plan from current -> target
            if hasattr(self.upper_panel.value_display, "sim"):
                # example: animate tiny easing toward current joint target
                import numpy as np
                q_target = np.deg2rad(values)  # if Angles mode
                q_now = self.upper_panel.value_display.sim.joint_angles
                frames = 45
                path = [q_now + (q_target - q_now) * t
                        for t in np.linspace(0, 1, frames)]
                self.upper_panel.value_display.sim.animate_trajectory(path, interval_ms=16)
            # or: self.send_command(full_command) just once here

        for i, (name, (min_val, max_val)) in enumerate(zip(names, ranges)):
            label = customtkinter.CTkLabel(self.upper_panel.value_display.slider_frame, text=name,
                                           font=customtkinter.CTkFont(size=14))
            label.grid(row=i * 2, column=0, sticky="w", pady=(5, 0))

            slider_var = customtkinter.DoubleVar()
            slider_var.set(self.home_pos[i])
            slider = customtkinter.CTkSlider(
                self.upper_panel.value_display.slider_frame,
                from_=min_val,
                to=max_val,
                variable=slider_var,
                number_of_steps=100,
                command=slider_moved  # <== Callback when moved
            )
            slider.grid(row=i * 2 + 1, column=0, sticky="ew", pady=(0, 10))
            slider.bind("<ButtonRelease-1>", lambda e: on_slider_release(e))
            self.upper_panel.value_display.labels.append(label)
            self.upper_panel.value_display.sliders.append(slider)
            self.upper_panel.value_display.slider_vars.append(slider_var)


    def get_slider_values(self):
        return [v.get() for v in self.upper_panel.value_display.slider_vars]


    def show_terminal(self):
        self.graph.pack_forget()
        self.terminal.pack(fill="both", expand=True)
        self.bottom_bar.highlight_terminal()

    def show_graph(self):
        if self.selected_mode in ["ACV","DCV"]:
            self.terminal.pack_forget()
            self.graph.pack(fill="both", expand=True)
            self.bottom_bar.highlight_graph()

            self.graph_enabled = True

    def hide_settings(self):
        self.robot_settings.grid_remove()
        self.show_all_pages()

    def on_closing(self):
        """ Cleanup when closing the app """
        self.destroy()

    def hide_all_pages(self):
        """ Switch to Database Overview screen """

        # Hide calibration components
        self.bottom_bar.grid_remove()
        self.paned.grid_remove()
    def show_all_pages(self):
        """ Switch to Database Overview screen """

        # Hide calibration components
        self.bottom_bar.grid()
        self.paned.grid()

    def show_settings(self):
        self.hide_all_pages()
        self.robot_settings.grid()

    def update_title(self, mode):

        if not self.running:
            self.selected_mode = mode

            # Show the ValueDisplay when a mode is selected
            if not self.upper_panel.value_display.winfo_ismapped():
                self.upper_panel.content_box.grid(row=0, column=0)

            # Enable graph only for certain modes
            self.show_terminal()


            self.set_mode(mode)


    def change_scaling(self, new_scaling):
        customtkinter.set_widget_scaling(int(new_scaling.replace("%", "")) / 100)

    def show_com_port_popup(self, message="Select COM port for robot communication:"):
        self.update_idletasks()
        m_x = self.winfo_width() / 2
        m_y = self.winfo_height() / 2

        popup = customtkinter.CTkToplevel(self)
        popup.title("Select COM Port")
        popup.geometry(f"700x250+{int(m_x - 350)}+{int(m_y - 125)}")
        popup.transient(self)
        popup.grab_set()

        label = customtkinter.CTkLabel(popup, text=message, text_color="white")
        label.pack(pady=(20, 10))

        import serial.tools.list_ports
        ports = [port.device for port in serial.tools.list_ports.comports()]

        com_var = customtkinter.StringVar(value=ports[0] if ports else "")
        dropdown = customtkinter.CTkOptionMenu(popup, values=ports, variable=com_var)
        dropdown.pack(pady=10)

        def confirm():
            selected = com_var.get().strip()
            if selected:
                self.selected_com_port = selected
                popup.destroy()
            else:
                dropdown.configure(fg_color="red")

        btn_confirm = customtkinter.CTkButton(popup, text="Confirm", command=confirm)
        btn_confirm.pack(pady=10)

        # Optional "Use Default" fallback (e.g., COM6)
        def use_default():
            self.selected_com_port = "COM6"
            popup.destroy()

        btn_default = customtkinter.CTkButton(popup, text="Use Default (COM6)", command=use_default)
        btn_default.pack(pady=(0, 10))


if __name__ == "__main__":
    app = RobotControlApp()
    app.mainloop()
