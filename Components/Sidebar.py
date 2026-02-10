import customtkinter
from Utils.color_theme import COLORS


class Sidebar(customtkinter.CTkFrame):
    def __init__(self, parent, mode_callback, scaling_callback, settings_callback, reconnect_callback):
        self.reconnect_callback = reconnect_callback

        self.default_color = COLORS["backgroundLight"]
        self.active_button_color = "#103857"  # Slightly darker than default blue
        self.hover_color = COLORS["hover"]
        self.text_color = COLORS["lg_text"]
        self.button_bg = COLORS["button_bg"]

        super().__init__(parent, width=140, corner_radius=0, fg_color=self.default_color, bg_color=self.default_color)
        self.grid(row=0, column=0, rowspan=3, sticky="nsew")

        self.mode_callback = mode_callback
        self.mode_buttons = {}
        self.active_button = None

        # Title Label
        self.title_label = customtkinter.CTkLabel(self, text="Robot Control", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.title_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        # Control Mode Buttons
        button_labels = ["Simple", "Angles", "Continuous"]
        for i, label in enumerate(button_labels):
            btn = customtkinter.CTkButton(
                self,
                text=label,
                command=lambda l=label: self.set_active_mode(l),
                hover_color=self.hover_color,
            )
            btn.grid(row=i + 1, column=0, padx=20, pady=10)
            self.mode_buttons[label] = btn

        row_index = len(button_labels) + 1


        # Settings Section
        self.settings_label = customtkinter.CTkLabel(self, text="Robot Settings:", anchor="w")
        self.settings_label.grid(row=row_index, column=0, padx=20, pady=(10, 0))
        row_index += 1

        self.settings_button = customtkinter.CTkButton(self, text="Settings", command=settings_callback)
        self.settings_button.grid(row=row_index, column=0, padx=20, pady=(10, 10))
        row_index += 1

        self.reconnect_button = customtkinter.CTkButton(
            self,
            text="Reconnect",
            command=self.reconnect_callback,
            fg_color="#2b5a2b",
            hover_color="#3a7a3a",
        )
        self.reconnect_button.grid(row=row_index, column=0, padx=20, pady=(5, 10))
        row_index += 1

        # Defaults
        self.selected_mode = ""

    def set_active_mode(self, mode):
        # Reset previous button to theme default
        if self.active_button:
            self.active_button.configure(fg_color=self.button_bg)

        # Highlight new one
        self.selected_mode = mode
        self.active_button = self.mode_buttons[mode]
        self.active_button.configure(fg_color=self.active_button_color)

        # Trigger external mode update
        self.mode_callback(mode)
