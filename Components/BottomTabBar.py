import customtkinter
from Utils.color_theme import COLORS

class BottomTabBar(customtkinter.CTkFrame):
    def __init__(self, parent, terminal_callback, graph_callback):
        super().__init__(parent, height=30, bg_color=COLORS["backgroundLight"], fg_color=COLORS["backgroundLight"], corner_radius=0)
        self.grid(row=3, column=0, columnspan=3, sticky="ew")
        self.grid_columnconfigure(0, weight=1)

        # === Top Border ===
        self.top_border = customtkinter.CTkFrame(self, height=1, fg_color=COLORS["border"])
        #self.top_border.pack(side="top", fill="x")

        # === Button Styles ===
        self.default_color = COLORS["backgroundLight"]
        self.active_color = COLORS["backgroundDark"]
        self.hover_color = COLORS["hover"]
        self.text_color = COLORS["lg_text"]

        # left pad, doesn't work color wise with just padx
        self.side_border = customtkinter.CTkFrame(self, height=30,width=18, fg_color=COLORS["backgroundLight"])
        self.side_border.pack(side="left", fill="y", pady=0)

        # === Buttons ===
        self.terminal_btn = customtkinter.CTkButton(
            self,
            text="Terminal",
            command=terminal_callback,
            width=80,
            corner_radius=0,
            text_color="white",
            fg_color=self.active_color,
            hover_color=self.hover_color,
            anchor="center",
        )

        self.graph_btn = customtkinter.CTkButton(
            self,
            text="Graph",
            command=graph_callback,
            width=60,
            corner_radius=0,
            fg_color=self.default_color,
            text_color=self.text_color,
            hover_color=self.hover_color,
            anchor="center",
        )

        self.terminal_btn.pack(side="left", fill="y", pady=0)
        self.graph_btn.pack(side="left", fill="y", pady=0)

    def highlight_terminal(self):
        self.terminal_btn.configure(fg_color=self.active_color, text_color="white")
        self.graph_btn.configure(fg_color=self.default_color, text_color=self.text_color)

    def highlight_graph(self):
        self.graph_btn.configure(fg_color=self.active_color, text_color="white")
        self.terminal_btn.configure(fg_color=self.default_color, text_color=self.text_color)
