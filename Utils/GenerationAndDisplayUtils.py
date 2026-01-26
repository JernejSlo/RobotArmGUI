import random
import time
import traceback
from colorama import Fore, Style


class GenerationAndDisplayUtils():

    def __init__(self):
        pass

    def generate_values_no_machine(self):
        """Generate values one by one with display updating gradually."""

        if not self.running:
            return

        total_values = len(self.upper_panel.value_display.labels_values["references"])
        current_values = []
        difference_values = []
        std_values = []

        for index in range(total_values):
            if self.interrupt("Generation interrupted."):
                return

            try:
                unit = self.upper_panel.value_display.labels_values["units"][index]
                reference = self.upper_panel.value_display.labels_values["references"][index]
            except IndexError:
                unit = "mV"
                reference = 0

            if self.sidebar.selected_mode in ("DCV", "ACV"):
                unit = unit[:-1] + "V"
            elif self.sidebar.selected_mode in ("DCI", "ACI"):
                unit = unit[:-1] + "A"
            if index == 4 and self.selected_mode == "RES" and not self.prompt_shown:
                self.prompt_shown = True
                self.running = False
                self.after(100, self.show_pause_popup)
            new_value = round(random.uniform(0, 1000000000), 2) / 1000000
            difference = round(random.uniform(-1000, 1000), 2) / 1000
            std = round(random.uniform(0.01, 0.3), 3)

            current_values.append({"Value": new_value, "Label": unit})
            difference_values.append({"Value": difference, "Label": unit})
            std_values.append({"Value": std, "Label": unit})

            self.upper_panel.value_display.labels_values["diffMeas"][index] = difference

            self.terminal.log(
                f"Index {index} - Simulated {self.sidebar.selected_mode}: {new_value} {unit}, Δ = {difference} {unit}, σ = {std} "
                f"(ref: {reference} {unit})"
            )

            self.log_measurement(
                calibration_id=self.current_calibration_id,
                set_value=reference,
                calculated_value=new_value,
                ref_set_diff=difference,
                std=std,
                unit = unit,
                frequency=None
            )

            self.upper_panel.value_display.update_values(current_values, difference_values, std_values)
            self.update_idletasks()

            # Only simulate linear refs if mode is DCV or ACV
            if self.selected_mode in ["DCV", "ACV"]:
                simulated_refs = [i for i in range(1, 6)]
                simulated_meas = [round(r + random.uniform(-0.1, 0.1), 5) for r in simulated_refs]
                simulated_diffs = [round(m - r, 5) for m, r in zip(simulated_meas, simulated_refs)]
                simulated_stds = [round(random.uniform(0.001, 0.01), 5) for _ in simulated_refs]
                simulated_units = [unit] * len(simulated_refs)

                for i in range(len(simulated_refs)):
                    self.log_linear_refs(
                        calibration_id=self.current_calibration_id,
                        set_value=simulated_refs[i],
                        calculated_value=simulated_meas[i],
                        ref_set_diff=simulated_diffs[i],
                        std=simulated_stds[i],
                        unit=simulated_units[i]
                    )

            time.sleep(0.1)

        if self.interrupt("Generation interrupted."):
            return



        self.stop_action()

    def interrupt(self,message):
        if not self.running:
            self.terminal.log(message)
            return True

    def log_all(self):
        meas = self.measParameters
        print("ahsvkdjasdj,mahv,mnahvskj,\n\n")
        print(meas)
        for i in range(len(meas["measurements"])):
            new_value = meas["measurements"][i]
            stdVar = meas["stdVars"][i]
            diffMeas = meas["diffMeas"][i]
            ref = meas["references"][i]
            unit = meas["units"][i]
            if meas["dirType"] != ":AC":
                freq = "/"
            else:
                freq = meas["frequencies"][i]

            self.log_measurement(
                calibration_id=self.current_calibration_id,
                set_value=ref,
                calculated_value=new_value,
                ref_set_diff=diffMeas,
                std=stdVar,
                frequency=freq,
                unit=unit
            )
        try:
            for i in range(len(meas["linearRefs"])):
                if self.selected_mode in ["DCV","ACV"]:
                    calibration_id = self.current_calibration_id
                    set_value = meas["linearRefs"][i]
                    calculated_value = meas["linearMeas"][i]
                    ref_set_diff = meas["diffLinearMeas"][i]
                    std = meas["linearStdVars"][i]
                    unit = meas["linearUnits"][i]
                    self.log_linear_refs(
                        calibration_id, set_value, calculated_value, ref_set_diff, std, unit
                    )
        except Exception as e:
            print("Couldn't log linearity")
            print(e)
    def get_calibration_values(self):
        """Run calibration once, log and update values. Falls back to fake values on error."""
        try:
            self.calibrate()  # Should generate all values in one go

            # Simulate graph update (fake example)
            self.log_all()
        except Exception as e:
            print("Here")
            if not self.skip_fake_version:
                print(Fore.RED + Style.BRIGHT + "Exception type: " + str(type(e)))
                print(Fore.YELLOW + Style.BRIGHT + "Exception message: " + str(e))
                print(Fore.CYAN + Style.BRIGHT + "Traceback:")
                traceback_lines = traceback.format_exception(type(e), e, e.__traceback__)
                for line in traceback_lines:
                    print(Fore.CYAN + line, end='')

                print("Returning to fake version.")
                self.get_calibration_values = self.generate_values_no_machine
                self.generate_values_no_machine()
            else:
                raise e
            return




        self.running = False
