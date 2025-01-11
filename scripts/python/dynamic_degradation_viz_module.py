import numpy as np
import json
import os
import pandas as pd
import scripts.python.static_degradation_viz_module as sdvm

INF_EST_DF_PREFIX = "inf_est_"
SENSOR_ACC_DF_PREFIX = "sensor_acc_"

class DynamicDegradationJsonData(sdvm.StaticDegradationJsonData):
    def populate_common_data(self, json_dict):
        super().populate_common_data(json_dict)
        self.density = np.round(json_dict["density"], 3)
        self.dynamic_degradation = json_dict["dynamic_degradation"]
        self.true_drift_coeff = json_dict["true_drift_coeff"]
        self.true_diffusion_coeff = json_dict["true_diffusion_coeff"]
        self.lowest_degraded_acc_lvl = json_dict["lowest_degraded_acc_lvl"]
        self.obs_queue_size = json_dict["obs_queue_size"]

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

                if not f.endswith(".json"): continue # only parse JSON files

                # Load the JSON file
                with open(os.path.join(root, f), "r") as file:
                    if not self.silent: print("Loading", f)
                    json_dict = json.load(file)

                # Initialize data
                if self.first_pass or json_dict["num_flawed_robots"] not in self.data.keys():

                    # Populate common data
                    self.populate_common_data(json_dict)

                    # Set up data structure
                    self.num_flawed_robots.append(json_dict["num_flawed_robots"])
                    try: # DEV NOTE: temporary fix due to dynamic_topo not having this key; in the future this `num_correct_robots` key will be removed because it's inferred
                        self.num_correct_robots.append(json_dict["num_correct_robots"])
                    except Exception as e:
                        self.num_correct_robots.append(self.num_robots - self.num_flawed_robots[-1])

                    self.data[json_dict["num_flawed_robots"]] = np.empty(
                        (
                            self.num_trials,
                            self.num_robots,
                            self.num_steps + 1,
                            4 # informed estimate, assumed sensor accuracy, and true sensor accuracy, and weighted average informed estimate of robot
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
            Numpy array of (informed_estimate, sensor_estimate) for all robots across all time steps (dim = num_robots * num_timesteps * 2)
        """
        # data_str_vec is a num_agents x num_steps list of lists
        return np.asarray(
            [
                [
                    [
                        float(split_elem[ind]) for ind in (7, 8, 10, 12) # extract 3 values (informed est, assumed acc, true acc, weighted avg informed est)
                    ] for elem in row for split_elem in [elem.split(",")]
                ] for row in data_str_vec
            ]
        ) # row = [rng_seed, n, t, local_est, local_conf, social_est, social_conf, informed_est, sensor_b_est, sensor_w_est, sensor_b_true, sensor_w_true, weighted_avg_informed_est]

################################################################################
################################################################################



################################################################################
# Alias for the StaticDegradationJsonDataSpecific
################################################################################

DynamicDegradationJsonDataSpecific = sdvm.StaticDegradationJsonDataSpecific

################################################################################
################################################################################



################################################################################
# Extract dynamic degradation data into DataFrames
################################################################################

# Extract the dynamic degradation data into Pandas DataFrames
def extract_dynamic_degradation_data_to_dfs(
    json_data_obj: DynamicDegradationJsonData,
    inf_est_df: pd.DataFrame = pd.DataFrame(),
    sensor_acc_df: pd.DataFrame = pd.DataFrame()
):
    """Extract the dynamic degradation experiment data into two Pandas DataFrame.

    The first DataFrame contains (besides common values) the following columns for each robot:
        - informed estimate (x_*), and
        - weighted informed estimate (x_prime_*).

    The second DataFrame contains (besides common values) the following columns for each robot:
        - assumed_acc (b_hat_*), and
        - true_acc (b_*).

    The * is a placeholder for the robot index. The returned DataFrame has `num_steps * num_trials` rows appended to whatever input DataFrame provided.

    Args:
        json_data_obj: DynamicDegradationJsonData that contains all the data intended for processing.
        inf_est_df: pd.DataFrame containing the informed estimates and the weighted informed estimates to concatenate to.
        sensor_acc_df: pd.DataFrame containing the assumed and true sensor accuracies to concatenate to.

    Returns:
        Two pandas.DataFrames, each with the following columns:
            "trial_ind",
            "step_ind",
            "sim_type",
            "method",
            "num_trials",
            "num_robots",
            "num_steps",
            "sensor_filter_period",
            "comms_period",
            "tfr",
            "flawed_sensor_acc_b",
            "flawed_sensor_acc_w",
            "correct_sensor_acc_b",
            "correct_sensor_acc_w",
            "correct_robot_filter",
            "dynamic_degradation"
            "true_drift_coeff",
            "true_diffusion_coeff",
            "lowest_degraded_acc_lvl",
            "obs_queue_size",
            "comms_range",
            "meas_period",
            "speed",
            "density",
            "filter_specific_params",
            "num_flawed_robots",
            "DATA_A_0",
            "DATA_A_1",
            ...,
            "DATA_A_{num_robots-1}",
            "DATA_B_0",
            "DATA_B_1",
            ...,
            "DATA_B_{num_robots-1}",
        where `DATA_A` is `x_*` (inf_est_df) / `b_hat_*` (sensor_acc_df) and `DATA_B` is `x_prime_*` (inf_est_df) / `b_*` (sensor_acc_df).
    """
    # json_data_obj.data is a dict with key=num_flawed_robots and value=np.ndarray with dim = (num_trials, num_robots, num_steps+1, 4)

    reshape_ndarr = lambda curves_ndarr : curves_ndarr.transpose(0, 2, 1).reshape(-1, json_data_obj.num_robots) # function to reshape the time-series + trials curves

    # Iterate through each case of # of flawed robots
    for n, data_ndarr_per_num_flawed_robot in json_data_obj.data.items():

        # Fill up common data
        common_data = {
            "sim_type": np.repeat(json_data_obj.sim_type, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "method": np.repeat(json_data_obj.method, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "num_trials": np.repeat(json_data_obj.num_trials, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "num_robots": np.repeat(json_data_obj.num_robots, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "num_steps": np.repeat(json_data_obj.num_steps, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "sensor_filter_period": np.repeat(json_data_obj.sensor_filter_period, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "comms_period": np.repeat(json_data_obj.comms_period, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "tfr": np.repeat(json_data_obj.tfr, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "flawed_sensor_acc_b": np.repeat(json_data_obj.flawed_sensor_acc_b, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "flawed_sensor_acc_w": np.repeat(json_data_obj.flawed_sensor_acc_w, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "correct_sensor_acc_b": np.repeat(json_data_obj.correct_sensor_acc_b, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "correct_sensor_acc_w": np.repeat(json_data_obj.correct_sensor_acc_w, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "dynamic_degradation": np.repeat(json_data_obj.dynamic_degradation, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "true_drift_coeff": np.repeat(json_data_obj.true_drift_coeff, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "true_diffusion_coeff": np.repeat(json_data_obj.true_diffusion_coeff, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "lowest_degraded_acc_lvl": np.repeat(json_data_obj.lowest_degraded_acc_lvl, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "obs_queue_size": np.repeat(json_data_obj.obs_queue_size, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "comms_range": np.repeat(json_data_obj.comms_range, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "meas_period": np.repeat(json_data_obj.meas_period, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "speed": np.repeat(json_data_obj.speed, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "density": np.repeat(json_data_obj.density, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "correct_robot_filter": np.repeat(json_data_obj.correct_robot_filter, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "filter_specific_params": np.repeat(json_data_obj.filter_specific_params, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
            "num_flawed_robots": np.repeat(n, (json_data_obj.num_trials * (json_data_obj.num_steps+1))),
        }

        # Extract the data into numpy array
        reg_inf_est_curves_ndarr = data_ndarr_per_num_flawed_robot[..., 0]
        assumed_acc_curves_ndarr = data_ndarr_per_num_flawed_robot[..., 1]
        true_acc_curves_ndarr = data_ndarr_per_num_flawed_robot[..., 2]
        weighted_avg_inf_est_curves_ndarr = data_ndarr_per_num_flawed_robot[..., 3]

        # Reshape the data: first flatten along trials and steps, keeping agents as columns
        inf_est_reshaped = reshape_ndarr(reg_inf_est_curves_ndarr)
        assumed_acc_reshaped = reshape_ndarr(assumed_acc_curves_ndarr)
        true_acc_reshaped = reshape_ndarr(true_acc_curves_ndarr)
        weighted_avg_inf_est_reshaped = reshape_ndarr(weighted_avg_inf_est_curves_ndarr)

        # Create temporary dataframes
        temp_reg_inf_est_df = pd.DataFrame(inf_est_reshaped, columns=["x_{0}".format(i) for i in range(json_data_obj.num_robots)])
        temp_assumed_acc_df = pd.DataFrame(assumed_acc_reshaped, columns=["b_hat_{0}".format(i) for i in range(json_data_obj.num_robots)])
        temp_true_acc_df = pd.DataFrame(true_acc_reshaped, columns=["b_{0}".format(i) for i in range(json_data_obj.num_robots)])
        temp_weighted_avg_inf_est_df = pd.DataFrame(weighted_avg_inf_est_reshaped, columns=["x_prime_{0}".format(i) for i in range(json_data_obj.num_robots)])

        temp_inf_est_df = pd.concat([
            pd.DataFrame(common_data),
            temp_reg_inf_est_df,
            temp_weighted_avg_inf_est_df
        ], axis=1)

        temp_sensor_acc_df = pd.concat([
            pd.DataFrame(common_data),
            temp_assumed_acc_df,
            temp_true_acc_df,
        ], axis=1)

        # Generate trial and step indices
        trial_indices = np.repeat(np.arange(json_data_obj.num_trials), json_data_obj.num_steps + 1)
        step_indices = np.tile(np.arange(json_data_obj.num_steps + 1), json_data_obj.num_trials)

        # Add the trial and step indices
        temp_inf_est_df["trial_ind"] = trial_indices
        temp_inf_est_df["step_ind"] = step_indices
        temp_inf_est_df = temp_inf_est_df[["trial_ind", "step_ind"] + [col for col in temp_inf_est_df.columns if col not in ['trial_ind', 'step_ind']]]

        temp_sensor_acc_df["trial_ind"] = trial_indices
        temp_sensor_acc_df["step_ind"] = step_indices
        temp_sensor_acc_df = temp_sensor_acc_df[["trial_ind", "step_ind"] + [col for col in temp_sensor_acc_df.columns if col not in ['trial_ind', 'step_ind']]]

        inf_est_df = pd.concat([inf_est_df, temp_inf_est_df], axis=0, ignore_index=True).copy(deep=True)
        sensor_acc_df = pd.concat([sensor_acc_df, temp_sensor_acc_df], axis=0, ignore_index=True).copy(deep=True)

    return (inf_est_df, sensor_acc_df)

################################################################################
################################################################################



################################################################################
# (Low-level) RMSD function given two arrays
################################################################################

def rmsd(arr_base, arr_target):
    """Compute the root mean squared deviation from a given target.

    Args:
        arr_base: array of base values
        arr_target: array of target values.

    Returns:
        An array of RMSD of the same size as the inputs
    """
    if (type(arr_target) != float and len(arr_base) != len(arr_target)):
        print("len(arr_base)={0}, len(arr_target)={1}".format(len(arr_base), len(arr_target)))
        raise RuntimeError("The two input arrays must be the same size or the target array should just be a floating point value.")

    return np.sqrt(
        np.mean(
            np.square(arr_base - arr_target)
        )
    )

################################################################################
################################################################################



################################################################################
# Compute the RMSD and store into a DataFrame
################################################################################

def compute_raw_rmsd(inf_est_df: pd.DataFrame, sensor_acc_df: pd.DataFrame):
    """Compute the RMSD for each time step.

    A single RMSD value is computed from all the robots' values at a particular time step.

    Args:
        inf_est_df: DataFrame containing the regular and weighted average informed estimates.
        sensor_acc_df: DataFrame containing the assumed and true sensor accuracies.

    Returns:
        A Pandas DataFrame containing only the common data and the 2 columns of RMSD values.
    """

    # Create a new DataFrame with the common values
    df = inf_est_df.loc[:, ~inf_est_df.columns.str.startswith("x")].copy(deep=True)

    # Get the columns desired
    x_prime_columns = [col for col in inf_est_df.columns if col.startswith("x_prime_")]
    b_hat_columns = [col for col in sensor_acc_df.columns if col.startswith("b_hat_")]
    b_columns = [col for col in sensor_acc_df.columns if col.startswith("b_") and not col.startswith("b_hat_")]

    # Compute the RMSD
    df["inf_est_rmsd"] = inf_est_df.apply(
        lambda row: rmsd(
            row[x_prime_columns].values,
            row["tfr"]
        ),
        axis=1
    )

    df["sensor_acc_rmsd"] = sensor_acc_df.apply(
        lambda row: rmsd(
            row[b_hat_columns].values,
            row[b_columns].values
        ),
        axis=1
    )

    return df