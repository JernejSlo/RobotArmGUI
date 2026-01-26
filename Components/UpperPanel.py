import customtkinter

from Components.ValueDisplay import ValueDisplay

class UpperPanel(customtkinter.CTkFrame):
    def __init__(self, parent, running, db=None):
        super().__init__(parent, fg_color="transparent")
        # Full panel stretches
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Create center container
        self.center_container = customtkinter.CTkFrame(self, fg_color="transparent")
        self.center_container.grid(row=0, column=0, pady=(40, 0))  # Push down by 40 pixels


        # Trick: Use an outer padding container to center inner vertically + horizontally
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # Inner contents centered in center_container
        self.center_container.grid_rowconfigure(0, weight=1)
        self.center_container.grid_columnconfigure(0, weight=1)

        # === Frame that holds just the components ===
        self.content_box = customtkinter.CTkFrame(self.center_container, fg_color="transparent")
        self.content_box.grid(row=0, column=0)

        self.value_display = ValueDisplay(self.content_box, running, db)

        self.value_display.grid(row=0, column=0, pady=(0, 10)) # remove to not show at start

