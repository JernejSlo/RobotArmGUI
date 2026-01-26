import tkinter
import traceback

import customtkinter
import numpy as np
from colorama import Fore, Style
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from Utils.color_theme import COLORS


class GraphComponent(customtkinter.CTkFrame):
    """ Real-time updating graph component (Uses CustomTkinter default background) """

    def __init__(self, parent,mode):
        self.default_color = COLORS["backgroundLight"]
        self.active_color = COLORS["backgroundDark"]
        self.hover_color = COLORS["hover"]
        self.text_color = COLORS["lg_text"]
        self.selected_mode = mode
        super().__init__(parent, fg_color=self.default_color)

        self.grid(row=0, column=0, padx=(20, 20), pady=(10, 10), sticky="nsew")
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.default_bg_color = self.default_color
        plt.style.use("dark_background")

        # === Graph Frame ===
        self.graph_frame = customtkinter.CTkFrame(self, fg_color=self.default_color)
        self.graph_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.graph_frame.grid_rowconfigure(0, weight=1)
        self.graph_frame.grid_rowconfigure(1, weight=0)
        self.graph_frame.grid_columnconfigure(0, weight=1)

        # === Graph ===
        self.figure, self.ax = plt.subplots()
        self.figure.patch.set_facecolor(self.default_bg_color)
        self.ax.set_facecolor(self.default_bg_color)

        self.canvas = FigureCanvasTkAgg(self.figure, master=self.graph_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # === Controls Below the Graph ===
        self.bottom_control_frame = customtkinter.CTkFrame(self.graph_frame, fg_color=self.default_color)
        self.bottom_control_frame.grid(row=1, column=0, pady=(5, 0))

        shared_btn_width = 40
        shared_btn_height = 26

        # Default: only one value
        self.actual_values = []
        self.time_values = []
        self.data_sets = [self.actual_values]
        self.labels = ["Measured values"]
        self.selected_index = 0

        self.max_points = 10

        self.value_data_labels = []

        # If multiple data types are added, show controls. For now, skip them
        if len(self.labels) > 1:
            self.left_button = customtkinter.CTkButton(
                self.bottom_control_frame, text="◀", width=shared_btn_width,
                height=shared_btn_height, command=self.switch_left)
            self.left_button.grid(row=0, column=0, padx=5)

            self.display_label = customtkinter.CTkLabel(
                self.bottom_control_frame, text=self.labels[self.selected_index],
                font=customtkinter.CTkFont(size=14))
            self.display_label.grid(row=0, column=1, padx=5)

            self.right_button = customtkinter.CTkButton(
                self.bottom_control_frame, text="▶", width=shared_btn_width,
                height=shared_btn_height, command=self.switch_right)
            self.right_button.grid(row=0, column=2, padx=5)

            spacer_col = 3
        else:
            spacer_col = 0


        # === Value Label (always shown) ===
        self.value_label = customtkinter.CTkLabel(
            self.bottom_control_frame, text="", font=customtkinter.CTkFont(size=14))
        self.value_label.grid(row=1, column=0, columnspan=7, pady=(5, 5))
        self.value_data_labels = [self.value_label]

        # Start graph animation
        self.ani = FuncAnimation(self.figure, self.update_graph, interval=1000)

    def update_data(self, values):
        print("Showing for",values)
        print("Currently", self.data_sets)
        """ Store new data points and update the graph """
        self.time_values.append(values[0]["Step"])
        for i, val in enumerate(values):
            self.data_sets[i].append(val["Value"])
            raw_val = val['Value']
            """value_str = raw_val if isinstance(raw_val, str) else f"{float(raw_val):.2f}"
            if i < len(self.value_data_labels):
                self.value_data_labels[i].configure(text=f"{value_str} {val['Label']}")"""


    def update_mode(self,mode):
        self.clear_graph()
        self.selected_mode = mode

    def clear_graph(self):
        """Clear all graph data and reset the display."""
        # Clear stored data
        self.time_values = []
        self.actual_values = []
        self.data_sets = [self.actual_values]

        # Clear plot
        self.ax.clear()

        # Optionally reset axis labels
        self.ax.set_xlabel("")
        self.ax.set_ylabel("")

        # Redraw the empty canvas
        self.canvas.draw()

    def update_graph(self, frame=None):
        """ Update the graph display with points and a fitted or fixed line depending on mode """
        self.ax.clear()
        data = self.data_sets[self.selected_index]
        time = self.time_values

        if self.max_points is not None:
            data = data[-self.max_points:]
            time = time[-self.max_points:]

        # Plot points
        self.ax.scatter(time, data, label=self.labels[self.selected_index], color="cyan", s=20)

        # Mode-dependent line
        try:
            if self.selected_mode == "ACV":
                self.ax.set_xlabel("Set frequency [Hz]")
                self.ax.set_ylabel("Measured voltage [V]")

                self.ax.set_xlim(0, 110)  # shows x-axis from 0 to 50
                # Flat line at 10

                x_vals = np.arange(10,100)
                y_vals = np.full_like(x_vals, 10)
                self.ax.plot(x_vals, y_vals, color="orange", linestyle="--", linewidth=1.5, label="Reference at 10 V")

            elif self.selected_mode == "DCV":
                self.ax.set_xlabel("Set voltage [V]")
                self.ax.set_ylabel("Mesured voltage [V]")
                x_vals = np.arange(0, 10)  # Fallback to (1,1), (2,2), ..., (5,5)

                y_vals = x_vals

                self.ax.plot(x_vals, y_vals, color="lightgreen", linestyle="--", linewidth=1.5, label="Linear reference")

        except Exception as e:
            print("Graph line drawing failed:", e)
            print(Fore.RED + Style.BRIGHT + "Exception type: " + str(type(e)))
            print(Fore.YELLOW + Style.BRIGHT + "Exception message: " + str(e))
            print(Fore.CYAN + Style.BRIGHT + "Traceback:")
            traceback_lines = traceback.format_exception(type(e), e, e.__traceback__)
            for line in traceback_lines:
                print(Fore.CYAN + line, end='')


        self.ax.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5)
        self.ax.legend()
        self.canvas.draw()

