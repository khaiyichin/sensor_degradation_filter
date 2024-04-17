import numpy as np
import json
import os
import matplotlib.pyplot as plt

class StaticDegradationJsonData:

    def __init__(self, data_folder: str, silent=True):
        self.first_pass = True
        self.silent = silent
        self.num_correct_robots = []
        self.num_flawed_robots = []

        self.load_data(data_folder)

    def populate_common_data(self, json_dict):
        self.num_trials = json_dict["num_trials"]
        self.num_robots = json_dict["num_robots"]
        self.num_steps = json_dict["num_steps"]
        self.sensor_filter_period = json_dict["sensor_filter_period"]
        self.comms_period = json_dict["comms_period"]
        self.tfr = json_dict["tfr"]
        self.flawed_sensor_acc_b = json_dict["flawed_sensor_acc_b"]
        self.flawed_sensor_acc_w = json_dict["flawed_sensor_acc_w"]
        self.correct_sensor_acc_b = json_dict["correct_sensor_acc_b"]
        self.correct_sensor_acc_w = json_dict["correct_sensor_acc_w"]
        self.fully_connected = json_dict["fully_connected"]
        self.correct_robot_filter = json_dict["correct_robot_filter"]

    def print_common_data(self):
        print("num_trials:", self.num_trials)
        print("num_robots:", self.num_robots)
        print("num_steps:", self.num_steps)
        print("sensor_filter_period:", self.sensor_filter_period)
        print("comms_period:", self.comms_period)
        print("tfr:", self.tfr)
        print("flawed_sensor_acc_b:", self.flawed_sensor_acc_b)
        print("flawed_sensor_acc_w:", self.flawed_sensor_acc_w)
        print("correct_sensor_acc_b:", self.correct_sensor_acc_b)
        print("correct_sensor_acc_w:", self.correct_sensor_acc_w)
        print("fully_connected:", self.fully_connected)
        print("correct_robot_filter:", self.correct_robot_filter)

    def load_data(self, folder_path):
        """Load the JSON data from a given folder.

        Args:
            data_folder: Path to the folder containing all the JSON data files.
        """
        self.data = {}

        # Receive folder path
        for root, _, files in os.walk(folder_path):
            if not self.silent: print("In folder:", root)

            for f in files:

                # Load the JSON file
                with open(os.path.join(root, f), "r") as file:
                    if not self.silent: print("Loading", f)
                    json_dict = json.load(file)

                # Initialize data
                if self.first_pass or json_dict["num_flawed_robots"] not in self.data.keys():

                    # Populate common data
                    self.populate_common_data(json_dict)

                    # Set up data structure
                    self.num_correct_robots.append(json_dict["num_correct_robots"])
                    self.num_flawed_robots.append(json_dict["num_flawed_robots"])

                    self.data[json_dict["num_flawed_robots"]] = np.empty(
                        (
                            self.num_trials,
                            self.num_robots,
                            self.num_steps + 1,
                            2 # informed estimate and sensor estimate of robot
                        )
                    )

                    self.first_pass = False

                # Decode json file into data
                self.data[json_dict["num_flawed_robots"]][json_dict["trial_ind"]] = \
                    self.decode_data_str(json_dict["data_str"])

        if not self.data: # no data populated
            raise Exception("No data populated, please check provided arguments.")

    def decode_data_str(self, data_str_vec):
        """Decodes the array of data string.

        Args:
            data_str_vec: List of lists of data string for all robots (across all time steps) in a single trial

        Returns:
            Numpy array of decisions for the robots across all time steps (num_agents x num_timesteps x 1)
        """
        # data_str_vec is a num_agents x num_steps list of lists
        return np.asarray(
            [
                [
                    [float(val) for val in elem.split(",")[2:4]] for elem in row
                ] for row in data_str_vec
            ]
        ) # row = [local_est, social_est, informed_est, sensor_b_est, sensor_w_est]