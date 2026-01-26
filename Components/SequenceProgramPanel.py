# Components/SequenceProgramPanel.py

from __future__ import annotations

import customtkinter as ctk
from dataclasses import dataclass
from typing import Callable, List, Optional, Union, Tuple

from Utils.color_theme import COLORS


@dataclass
class PoseBlock:
    pose_id: int
    pose_name: str
    joints: List[float]  # [J0..J5] in degrees


@dataclass
class WaitBlock:
    seconds: float


SequenceBlock = Union[PoseBlock, WaitBlock]


class SequenceProgramPanel(ctk.CTkFrame):
    """
    Sequence programmer with:
      - sequence dropdown (select which sequence you're editing)
      - new sequence creation
      - run/pause/stop
      - block list + edit operations

    External responsibilities:
      - loading a sequence into the panel (set_sequence_blocks)
      - saving blocks when changed (on_sequence_changed)
    """

    def __init__(self, parent, width: int = 360, **kwargs):
        super().__init__(parent, fg_color=COLORS["backgroundDark"], width=width, **kwargs)

        # External callbacks
        self.on_run: Optional[Callable[[List[SequenceBlock]], None]] = None
        self.on_pause: Optional[Callable[[], None]] = None
        self.on_stop: Optional[Callable[[], None]] = None
        self.on_simulate = Optional[Callable[[List[SequenceBlock]], None]]

        # NEW: sequence selection callbacks
        self.on_sequence_selected: Optional[Callable[[int], None]] = None
        self.on_sequence_new: Optional[Callable[[str], int]] = None
        self.on_sequence_changed: Optional[Callable[[int, List[SequenceBlock]], None]] = None


        self._sequence_id: Optional[int] = None
        self._sequence_choices: List[Tuple[int, str]] = []  # [(id,name)]

        self._blocks: List[SequenceBlock] = []
        self._selected_index: Optional[int] = None

        self._is_running = False
        self._is_paused = False

        # Layout
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(5, weight=1)  # list grows

        # Title
        self.title = ctk.CTkLabel(self, text="Sequence programmer", font=ctk.CTkFont(size=16, weight="bold"))
        self.title.grid(row=0, column=0, sticky="nw", padx=12, pady=(12, 6))

        # --- Sequence selector row (dropdown + New) ---
        self.seq_row = ctk.CTkFrame(self, fg_color="transparent")
        self.seq_row.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 10))
        self.seq_row.grid_columnconfigure(0, weight=1)
        self.seq_row.grid_columnconfigure(1, weight=0)

        self.seq_var = ctk.StringVar(value="(no sequences)")
        self.seq_dropdown = ctk.CTkOptionMenu(
            self.seq_row,
            values=["(no sequences)"],
            variable=self.seq_var,
            command=self._on_dropdown_changed,
        )
        self.seq_dropdown.grid(row=0, column=0, sticky="ew", padx=(0, 8))

        self.btn_new_seq = ctk.CTkButton(self.seq_row, text="New", command=self._new_sequence_clicked, width=70)
        self.btn_new_seq.grid(row=0, column=1, sticky="e")

        # Hint
        self.hint = ctk.CTkLabel(self, text="Blocks execute top-to-bottom.", anchor="w")
        self.hint.grid(row=2, column=0, sticky="ew", padx=12, pady=(0, 10))

        # --- Run Controls ---
        self.run_controls = ctk.CTkFrame(self, fg_color="transparent")
        self.run_controls.grid(row=3, column=0, sticky="ew", padx=12, pady=(0, 10))
        # in __init__ after creating self.run_controls
        for c in range(4):
            self.run_controls.grid_columnconfigure(c, weight=1)

        self.btn_run = ctk.CTkButton(self.run_controls, text="Run", command=self._run_clicked, state="disabled")
        self.btn_run.grid(row=0, column=0, sticky="ew", padx=(0, 8))

        self.btn_pause = ctk.CTkButton(self.run_controls, text="Pause", command=self._pause_clicked, state="disabled")
        self.btn_pause.grid(row=0, column=1, sticky="ew", padx=(0, 8))

        self.btn_stop = ctk.CTkButton(self.run_controls, text="Stop", command=self._stop_clicked, state="disabled")
        self.btn_stop.grid(row=0, column=2, sticky="ew", padx=(0, 8))

        self.btn_simulate = ctk.CTkButton(self.run_controls, text="Simulate", command=self._simulate_clicked, state="disabled")
        self.btn_simulate.grid(row=0, column=3, sticky="ew")


        # Scrollable list
        self.list_frame = ctk.CTkScrollableFrame(self, fg_color="transparent")
        self.list_frame.grid(row=5, column=0, sticky="nsew", padx=12, pady=(0, 10))
        self.list_frame.grid_columnconfigure(0, weight=1)

        # Edit controls row
        self.controls = ctk.CTkFrame(self, fg_color="transparent")
        self.controls.grid(row=6, column=0, sticky="ew", padx=12, pady=(0, 12))
        for c in range(4):
            self.controls.grid_columnconfigure(c, weight=1)

        self.btn_up = ctk.CTkButton(self.controls, text="Up", command=self._move_up, state="disabled")
        self.btn_up.grid(row=0, column=0, sticky="ew", padx=(0, 8))

        self.btn_down = ctk.CTkButton(self.controls, text="Down", command=self._move_down, state="disabled")
        self.btn_down.grid(row=0, column=1, sticky="ew", padx=(0, 8))

        self.btn_delete = ctk.CTkButton(self.controls, text="Delete", command=self._delete_selected, state="disabled")
        self.btn_delete.grid(row=0, column=2, sticky="ew", padx=(0, 8))

        self.btn_clear = ctk.CTkButton(self.controls, text="Clear", command=self.clear_sequence)
        self.btn_clear.grid(row=0, column=3, sticky="ew")

        self._rebuild()

    # ---------------- Sequence selector API ----------------
    def set_sequences(self, sequences: List[Tuple[int, str]], select_id: Optional[int] = None):
        """
        sequences: [(id, name), ...]
        select_id: sequence to select (optional)
        """
        self._sequence_choices = list(sequences)

        if not self._sequence_choices:
            self.seq_dropdown.configure(values=["(no sequences)"])
            self.seq_var.set("(no sequences)")
            self._sequence_id = None
            self._rebuild()
            return

        display = [f"{sid}: {name}" for sid, name in self._sequence_choices]
        self.seq_dropdown.configure(values=display)

        if select_id is None:
            select_id = self._sequence_choices[0][0]

        # Find matching display string
        sel_text = None
        for sid, name in self._sequence_choices:
            if sid == select_id:
                sel_text = f"{sid}: {name}"
                self._sequence_id = sid
                break

        if sel_text is None:
            sid, name = self._sequence_choices[0]
            self._sequence_id = sid
            sel_text = f"{sid}: {name}"

        self.seq_var.set(sel_text)
        self._rebuild()

    def get_selected_sequence_id(self) -> Optional[int]:
        return self._sequence_id

    def set_sequence_blocks(self, sequence_id: int, blocks: List[SequenceBlock]):
        self._sequence_id = sequence_id
        self._blocks = list(blocks)
        self._selected_index = None
        self._rebuild()

    # ---------------- Blocks API ----------------
    def add_pose_block(self, pose_id: int, pose_name: str, joints_deg: List[float]):
        self._blocks.append(PoseBlock(pose_id=pose_id, pose_name=pose_name, joints=list(joints_deg)))
        self._selected_index = len(self._blocks) - 1
        self._rebuild()
        self._notify_changed()

    def add_wait_block(self, seconds: float):
        sec = float(seconds)
        if sec < 0:
            sec = 0.0
        self._blocks.append(WaitBlock(seconds=sec))
        self._selected_index = len(self._blocks) - 1
        self._rebuild()
        self._notify_changed()

    def get_sequence(self) -> List[SequenceBlock]:
        return list(self._blocks)

    def clear_sequence(self):
        if self._is_running:
            return
        self._blocks.clear()
        self._selected_index = None
        self._rebuild()
        self._notify_changed()

    # ---------------- Run-state API ----------------
    def set_running_state(self, running: bool, paused: bool = False):
        self._is_running = bool(running)
        self._is_paused = bool(paused)
        self._update_run_controls_state()

    # ---------------- Callbacks from UI ----------------
    def _on_dropdown_changed(self, selection: str):
        if not selection or selection.startswith("(no sequences)"):
            return
        try:
            sid = int(selection.split(":")[0].strip())
        except Exception:
            return

        self._sequence_id = sid
        if self.on_sequence_selected:
            self.on_sequence_selected(sid)

    def _new_sequence_clicked(self):
        # MVP: simple popup asking for a name
        win = ctk.CTkToplevel(self)
        win.title("New sequence")
        win.geometry("360x160")
        win.grab_set()

        lbl = ctk.CTkLabel(win, text="Sequence name:")
        lbl.pack(padx=12, pady=(16, 6), anchor="w")

        entry = ctk.CTkEntry(win)
        entry.pack(padx=12, pady=(0, 12), fill="x")
        entry.focus_set()

        def create():
            name = entry.get().strip() or "Unnamed"
            if not self.on_sequence_new:
                win.destroy()
                return
            sid = int(self.on_sequence_new(name))
            win.destroy()
            # the app will call set_sequences + set_sequence_blocks after creation

        btn = ctk.CTkButton(win, text="Create", command=create)
        btn.pack(padx=12, pady=(0, 12), fill="x")

    def _run_clicked(self):
        if not self._blocks:
            return
        if self.on_run:
            self.set_running_state(True, paused=False)
            self.on_run(self.get_sequence())

    def _simulate_clicked(self):
        if not self._blocks:
            return
        if self.on_simulate:
            self.on_simulate(self.get_sequence())

    def _pause_clicked(self):
        if not self._is_running:
            return
        self._is_paused = not self._is_paused
        self._update_run_controls_state()
        if self.on_pause:
            self.on_pause()

    def _stop_clicked(self):
        if not self._is_running:
            return
        self.set_running_state(False, paused=False)
        if self.on_stop:
            self.on_stop()

    # ---------------- Internals ----------------
    def _notify_changed(self):
        sid = self._sequence_id
        if sid is None:
            return
        if self.on_sequence_changed:
            self.on_sequence_changed(sid, self.get_sequence())

    def _update_run_controls_state(self):
        has_blocks = len(self._blocks) > 0
        has_seq = self._sequence_id is not None

        if not self._is_running:
            can = (has_seq and has_blocks)
            self.btn_run.configure(state="normal" if can else "disabled")
            self.btn_simulate.configure(state="normal" if can else "disabled")
            self.btn_run.configure(state="normal" if (has_seq and has_blocks) else "disabled")
            self.btn_pause.configure(state="disabled", text="Pause")
            self.btn_stop.configure(state="disabled")
            self.btn_clear.configure(state="normal")
        else:
            self.btn_run.configure(state="disabled")
            self.btn_stop.configure(state="normal")
            self.btn_pause.configure(state="normal", text="Resume" if self._is_paused else "Pause")
            self.btn_clear.configure(state="disabled")
            self.btn_up.configure(state="disabled")
            self.btn_down.configure(state="disabled")
            self.btn_delete.configure(state="disabled")
            self.btn_simulate.configure(state="disabled")

    def _set_selected(self, idx: int):
        if self._is_running:
            return
        if idx < 0 or idx >= len(self._blocks):
            self._selected_index = None
        else:
            self._selected_index = idx
        self._rebuild()

    def _rebuild(self):
        for w in self.list_frame.winfo_children():
            w.destroy()

        if not self._blocks:
            empty = ctk.CTkLabel(self.list_frame, text="No blocks yet. Add poses or Wait blocks.", anchor="w")
            empty.grid(row=0, column=0, sticky="ew", pady=6)
            self._update_controls_state()
            self._update_run_controls_state()
            return

        for i, block in enumerate(self._blocks):
            is_sel = (self._selected_index == i)

            card = ctk.CTkFrame(
                self.list_frame,
                fg_color=COLORS.get("backgroundLight", "gray20") if is_sel else "transparent",
                corner_radius=10,
            )
            card.grid(row=i, column=0, sticky="ew", pady=6)
            card.grid_columnconfigure(0, weight=1)

            header = ctk.CTkFrame(card, fg_color="transparent")
            header.grid(row=0, column=0, sticky="ew", padx=10, pady=(8, 2))
            header.grid_columnconfigure(1, weight=1)

            num = ctk.CTkLabel(header, text=f"{i+1}.", width=28, anchor="w", font=ctk.CTkFont(weight="bold"))
            num.grid(row=0, column=0, sticky="w")

            if isinstance(block, PoseBlock):
                title = f"POSE  |  {block.pose_name} (id={block.pose_id})"
                detail = "J: " + ", ".join(f"{v:.1f}" for v in block.joints)
            else:
                title = "WAIT"
                detail = f"{block.seconds:.2f} s"

            t = ctk.CTkLabel(header, text=title, anchor="w", font=ctk.CTkFont(weight="bold"))
            t.grid(row=0, column=1, sticky="w")

            d = ctk.CTkLabel(card, text=detail, anchor="w")
            d.grid(row=1, column=0, sticky="ew", padx=38, pady=(0, 8))

            def make_click(j=i):
                return lambda _e=None: self._set_selected(j)

            card.bind("<Button-1>", make_click(i))
            header.bind("<Button-1>", make_click(i))
            t.bind("<Button-1>", make_click(i))
            d.bind("<Button-1>", make_click(i))
            num.bind("<Button-1>", make_click(i))

        self._update_controls_state()
        self._update_run_controls_state()

    def _update_controls_state(self):
        if self._is_running:
            return
        has_sel = self._selected_index is not None
        self.btn_delete.configure(state="normal" if has_sel else "disabled")
        self.btn_up.configure(state="normal" if (has_sel and self._selected_index > 0) else "disabled")
        self.btn_down.configure(state="normal" if (has_sel and self._selected_index < len(self._blocks) - 1) else "disabled")

    def _delete_selected(self):
        if self._is_running or self._selected_index is None:
            return
        del self._blocks[self._selected_index]
        self._selected_index = None if not self._blocks else min(self._selected_index, len(self._blocks) - 1)
        self._rebuild()
        self._notify_changed()

    def _move_up(self):
        i = self._selected_index
        if self._is_running or i is None or i <= 0:
            return
        self._blocks[i - 1], self._blocks[i] = self._blocks[i], self._blocks[i - 1]
        self._selected_index = i - 1
        self._rebuild()
        self._notify_changed()

    def _move_down(self):
        i = self._selected_index
        if self._is_running or i is None or i >= len(self._blocks) - 1:
            return
        self._blocks[i + 1], self._blocks[i] = self._blocks[i], self._blocks[i + 1]
        self._selected_index = i + 1
        self._rebuild()
        self._notify_changed()
