# Components/PositionControlPanel.py
# UI panel: search saved poses (SQLite), selectable list, details view, and action buttons.

from __future__ import annotations

import customtkinter
import customtkinter as ctk
from typing import Callable, List, Optional

from Utils.color_theme import COLORS
from Utils.DatabaseManager import RobotPositionsDB, RobotPosition


class PositionControlPanel(ctk.CTkFrame):
    """
    Right-side panel for pose search + selection + basic actions.

    Contains:
      - Search bar (partial name match)
      - Scrollable selectable list of results
      - Details area: pose name + joint value labels
      - Buttons: Preview, Reset, Set Pose (callbacks are placeholders)

    Callbacks (optional):
      - on_preview(pose: RobotPosition)
      - on_reset()
      - on_set_pose(pose: RobotPosition)
    """

    def __init__(
        self,
        parent,
        db: RobotPositionsDB,
        *,
        on_preview: Optional[Callable[[RobotPosition], None]] = None,
        on_home: Optional[Callable[[RobotPosition], None]] = None,
        on_set_pose: Optional[Callable[[RobotPosition], None]] = None,
        on_add_to_sequence: Optional[Callable[[RobotPosition], None]] = None,
        width: int = 320,
        **kwargs
    ):
        super().__init__(parent, fg_color=COLORS["backgroundDark"], width=width, **kwargs)

        self.db = db
        self.on_preview = on_preview
        self.on_home = on_home
        self.on_set_pose = on_set_pose
        self.on_add_to_sequence = None  # callable(pose_or_wait_dict)

        self._results: List[RobotPosition] = []
        self._selected: Optional[RobotPosition] = None

        # Layout
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=1)  # results list expands

        # Title
        self.title = ctk.CTkLabel(
            self,
            text="Position control panel",
            font=ctk.CTkFont(size=16, weight="bold"),
        )
        self.title.grid(row=0, column=0, sticky="nw", padx=12, pady=(12, 8))

        # Search row
        self.search_row = ctk.CTkFrame(self, fg_color="transparent")
        self.search_row.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 10))
        self.search_row.grid_columnconfigure(0, weight=1)

        self.search_var = ctk.StringVar(value="")
        self.search_entry = ctk.CTkEntry(
            self.search_row,
            textvariable=self.search_var,
            placeholder_text="Search poses...",
        )
        self.search_entry.grid(row=0, column=0, sticky="ew")
        self.search_entry.bind("<KeyRelease>", self._on_search_changed)
        self.search_entry.bind("<Return>", self._on_search_enter)

        # Results list (scrollable)
        self.results_frame = ctk.CTkScrollableFrame(self, fg_color="transparent")
        self.results_frame.grid(row=2, column=0, sticky="nsew", padx=12, pady=(0, 10))
        self.results_frame.grid_columnconfigure(0, weight=1)

        # Details panel
        self.details = ctk.CTkFrame(self, fg_color="transparent")
        self.details.grid(row=3, column=0, sticky="ew", padx=12, pady=(0, 10))
        self.details.grid_columnconfigure(0, weight=1)
        self.details.grid_columnconfigure(1, weight=0)

        self.selected_name_lbl = ctk.CTkLabel(
            self.details,
            text="Selected: —",
            anchor="w",
            font=ctk.CTkFont(size=14, weight="bold"),
        )
        self.selected_name_lbl.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 8))

        # Joint labels
        self._joint_names = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Gripper Roll", "Gripper Open"]
        self._joint_value_labels: List[ctk.CTkLabel] = []

        for i, jn in enumerate(self._joint_names):
            name_lbl = ctk.CTkLabel(self.details, text=f"{jn}:", anchor="w")
            name_lbl.grid(row=i + 1, column=0, sticky="w", pady=1)

            val_lbl = ctk.CTkLabel(self.details, text="—", anchor="e")
            val_lbl.grid(row=i + 1, column=1, sticky="e", pady=1)
            self._joint_value_labels.append(val_lbl)

        # Wait input row (above buttons)
        self.wait_row = ctk.CTkFrame(self, fg_color="transparent")
        self.wait_row.grid(row=4, column=0, sticky="ew", padx=12, pady=(0, 8))
        self.wait_row.grid_columnconfigure(0, weight=1)

        self.wait_seconds_var = ctk.StringVar(value="1.0")
        self.wait_entry = ctk.CTkEntry(
            self.wait_row,
            textvariable=self.wait_seconds_var,
            placeholder_text="Wait seconds (e.g., 0.5)"
        )
        self.wait_entry.grid(row=0, column=0, sticky="ew")

        # Buttons row
        self.buttons = ctk.CTkFrame(self, fg_color="transparent")
        self.buttons.grid(row=5, column=0, sticky="ew", padx=12, pady=(0, 12))
        for c in range(4):
            self.buttons.grid_columnconfigure(c, weight=1)

        self.btn_preview = ctk.CTkButton(self.buttons, text="Preview", command=self._preview_clicked, state="disabled")
        self.btn_preview.grid(row=0, column=0, sticky="ew", padx=(0, 8))

        # Rename Reset -> Home (text only, keep handler name if you want)
        self.btn_home = ctk.CTkButton(self.buttons, text="Home", command=self._home_clicked)
        self.btn_home.grid(row=0, column=1, sticky="ew", padx=(0, 8))

        self.btn_set_pose = ctk.CTkButton(self.buttons, text="Set Pose", command=self._set_pose_clicked,
                                          state="disabled")
        self.btn_set_pose.grid(row=0, column=2, sticky="ew", padx=(0, 8))

        self.btn_add_seq = ctk.CTkButton(self.buttons, text="Add to sequence", command=self._add_to_sequence_clicked,
                                         state="disabled")
        self.btn_add_seq.grid(row=0, column=3, sticky="ew")

        # Initial load
        self.refresh_results("")



    # ---------------- Public API ----------------
    def refresh_results(self, query: str):
        """Fetch from DB and rebuild list."""
        try:
            self._results = self.db.search_positions_by_name(query)
        except Exception:
            self._results = []

        self._rebuild_results_list()

        # If current selection is not in results, clear selection
        if self._selected and all(p.id != self._selected.id for p in self._results):
            self.clear_selection()

    def clear_selection(self):
        self._selected = None
        self.selected_name_lbl.configure(text="Selected: —")
        for v in self._joint_value_labels:
            v.configure(text="—")
        self.btn_preview.configure(state="disabled")
        self.btn_set_pose.configure(state="disabled")
        self.btn_add_seq.configure(state="disabled")


    def get_selected_pose(self) -> Optional[RobotPosition]:
        return self._selected

    # ---------------- Internal ----------------
    def _on_search_changed(self, _event=None):
        self.refresh_results(self.search_var.get().strip())

    def _on_search_enter(self, _event=None):
        self.refresh_results(self.search_var.get().strip())

    def _rebuild_results_list(self):
        # clear
        for w in self.results_frame.winfo_children():
            w.destroy()

        if not self._results:
            empty = ctk.CTkLabel(self.results_frame, text="No saved positions found.", anchor="w")
            empty.grid(row=0, column=0, sticky="ew", pady=6)
            return

        for idx, pose in enumerate(self._results):
            btn = ctk.CTkButton(
                self.results_frame,
                text=f"{pose.name}  (id={pose.id})",
                anchor="w",
                fg_color=COLORS.get("backgroundLight", "gray20"),
                hover_color=COLORS.get("hover", "gray30"),
                command=lambda p=pose: self._select_pose(p),
            )
            btn.grid(row=idx, column=0, sticky="ew", pady=4)

    def _select_pose(self, pose: RobotPosition):
        self._selected = pose
        self.selected_name_lbl.configure(text=f"Selected: {pose.name}  (id={pose.id})")

        values = [pose.j_base, pose.j_bottom, pose.j_2, pose.j_3, pose.j_4, pose.j_gripper]
        for lbl, v in zip(self._joint_value_labels, values):
            lbl.configure(text=f"{v:.2f}")

        self.btn_preview.configure(state="normal")
        self.btn_set_pose.configure(state="normal")
        self.btn_add_seq.configure(state="normal")

    # ---------------- Button handlers (placeholders) ----------------
    def _preview_clicked(self):
        if self._selected and self.on_preview:
            self.on_preview(self._selected)
    def _set_pose_clicked(self):
        if self._selected and self.on_set_pose:
            print(self._selected)
            self.on_set_pose(self._selected)

    def _home_clicked(self):
        if getattr(self, "on_home", None):
            self.on_home()

    def _add_to_sequence_clicked(self):
        """
        Adds either:
          - selected pose block
          - OR a wait block if no pose selected? (MVP: require pose selected for pose blocks)
        Here: if pose selected -> add pose block; also supports adding wait by typing seconds and pressing add (optional).
        """
        if self.on_add_to_sequence is None:
            return

        # If a pose is selected, add that pose as a block
        if self._selected is not None:
            pose = self._selected
            self.on_add_to_sequence({
                "type": "pose",
                "id": pose.id,
                "name": pose.name,
                "joints": [pose.j_base, pose.j_bottom, pose.j_2, pose.j_3, pose.j_4, pose.j_gripper],
            })
            return

        # Otherwise add wait (not typical, but supported)
        sec = self._parse_wait_seconds()
        self.on_add_to_sequence({"type": "wait", "seconds": sec})

    def _parse_wait_seconds(self) -> float:
        try:
            return float(self.wait_seconds_var.get().strip())
        except Exception:
            return 1.0

