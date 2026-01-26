import customtkinter
from Utils.color_theme import COLORS


class RobotSettings(customtkinter.CTkFrame):
    """ Robot settings panel """

    def __init__(self, parent, callback):
        self.default_color = COLORS["backgroundLight"]
        self.active_color = COLORS["backgroundDark"]
        self.hover_color = COLORS["hover"]
        self.text_color = COLORS["lg_text"]

        super().__init__(parent, fg_color=self.default_color, bg_color=self.default_color)
        self.grid(row=0, column=1, rowspan=3, padx=20, pady=20, sticky="nsew")

        self.callback = callback

        self.grid_rowconfigure(2, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # === Back Button ===
        self.back_button = customtkinter.CTkButton(
            self, text="‹ Back", width=80, command=self.callback
        )
        self.back_button.grid(row=0, column=0, sticky="w", padx=5, pady=(5, 10))

        # === Settings content placeholder ===
        placeholder_label = customtkinter.CTkLabel(
            self, text="Settings will be available here...",
            font=customtkinter.CTkFont(size=14, weight="normal"),
            text_color=self.text_color
        )
        placeholder_label.grid(row=1, column=0, sticky="nw", padx=10, pady=10)
