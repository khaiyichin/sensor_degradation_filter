import numpy as np
import pandas as pd
import json
import os
from joblib import Parallel, delayed
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.patches import Ellipse
import scipy.stats as stats
import multiprocessing

class StaticDegradationJsonData:
    """
    This class stores data for a given set of parameters for all trials and for all # of flawed robots. E.g., if an experiment
    is run for 25 trials with 1, 2, and 3 flawed robots (i.e., 3 * 25 = 75 .json files), a single instance of this class would
    store all 75 cases.
    """

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
        try:
            self.filter_specific_params = json_dict["filter_specific_params"]
        except Exception as e:
            self.filter_specific_params = None

    def print_common_data(self):
        print("num_trials:", self.num_trials)
        print("num_robots:", self.num_robots)
        print("num_correct_robots:", self.num_correct_robots)
        print("num_flawed_robots:", self.num_flawed_robots)
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
        print("filter_specific_params:", self.filter_specific_params)

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
            Numpy array of (informed_estimate, sensor_estimate) for all robots across all time steps (dim = num_robots * num_timesteps * 2)
        """
        # data_str_vec is a num_agents x num_steps list of lists
        return np.asarray(
            [
                [
                    [float(val) for val in elem.split(",")[2:4]] for elem in row
                ] for row in data_str_vec
            ]
        ) # row = [local_est, social_est, informed_est, sensor_b_est, sensor_w_est]

def process_convergence_accuracy(
    json_data_obj: StaticDegradationJsonData,
    df: pd.DataFrame = pd.DataFrame(),
    threshold = 0.01,
):
    """Compute the point in time when convergence is achieved.

    This computes the convergence timestep (# of observations) for the informed estimates.
    If the returned value is equal to the number of observations, that means convergence was not achieved.

    Args:
        json_data_obj: StaticDegradationJsonData that contains all the data intended for processing.
        df: pd.DataFrame to concatenate the processed data to.
        threshold: A float parametrizing the difference threshold.

    Returns:
        A pandas.DataFrame with the following columns:
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
            "fully_connected",
            "correct_robot_filter",
            "filter_specific_params",
            "num_flawed_robots",
            "conv_step_ind",
            "accuracies"
    """
    """
    Methodology in computing convergence:
    Anchor to one point and check all later values to see if difference exceeds threshold,
    repeat until the earliest anchor point reaches the end uninterrupted.
    """

    # json_data_obj.data is a dict with key=num_flawed_robots and value=np.ndarray with dim = (num_flawed_robots, num_trials, num_robots, num_steps+1, 2)

    # Iterate through each case of # of flawed robots
    for n, data_ndarr_per_num_flawed_robot in json_data_obj.data.items():

        inf_est_curves_ndarr = data_ndarr_per_num_flawed_robot[..., 0] # inf_est_curves_ndarr has dim = (num_trials, num_agents, num_steps+1)

        conv_ind_lst = detect_convergence(threshold, inf_est_curves_ndarr)

        # Compute accuracy
        acc_lst = compute_accuracy(json_data_obj.tfr, inf_est_curves_ndarr, conv_ind_lst)

        data = {
            "num_trials": [json_data_obj.num_trials],
            "num_robots": [json_data_obj.num_robots],
            "num_steps": [json_data_obj.num_steps],
            "sensor_filter_period": [json_data_obj.sensor_filter_period],
            "comms_period": [json_data_obj.comms_period],
            "tfr": [json_data_obj.tfr],
            "flawed_sensor_acc_b": [json_data_obj.flawed_sensor_acc_b],
            "flawed_sensor_acc_w": [json_data_obj.flawed_sensor_acc_w],
            "correct_sensor_acc_b": [json_data_obj.correct_sensor_acc_b],
            "correct_sensor_acc_w": [json_data_obj.correct_sensor_acc_w],
            "fully_connected": [json_data_obj.fully_connected],
            "correct_robot_filter": [json_data_obj.correct_robot_filter],
            "filter_specific_params": [json_data_obj.filter_specific_params],
            "num_flawed_robots": [n],
            "conv_step_ind": [conv_ind_lst],
            "accuracies": [acc_lst]
        }

        df = pd.concat([df, pd.DataFrame(data)], ignore_index=True)

    return df

def detect_convergence(
    conv_threshold,
    curves_ndarr: np.ndarray # dim = (num_trials, num_robots, num_steps+1)
) -> list:

    def parallel_find_convergence_point_trial(trial_ind, robot_curves_per_trial):
        ref_ind_lst = [len(robot_curves_per_trial[0])-1 for _ in range(len(robot_curves_per_trial))]

        # Iterate through each robot's curve
        for robot_ind, curve in enumerate(robot_curves_per_trial):

            # Iterate through each point in the curve
            for curve_ind, curve_val in enumerate(curve[:-1]):
                # Specify evaluation region, i.e., all points to the right of the current point
                evaluation_region = curve[curve_ind+1:]

                # Compute the upper and lower limits for the current point
                val_lower_limit = curve_val - conv_threshold
                val_upper_limit = curve_val + conv_threshold

                # Check if all points in the evaluation region falls within the upper and lower limit of the current point
                if np.min(evaluation_region) > val_lower_limit and np.max(evaluation_region) < val_upper_limit:
                    ref_ind_lst[robot_ind] = curve_ind
                    break

        return ref_ind_lst

    # Run each trial in parallel
    conv_ind_lst = Parallel(n_jobs=-1, verbose=0)(
        delayed(parallel_find_convergence_point_trial)(trial_ind, robot_curves_per_trial) for trial_ind, robot_curves_per_trial in enumerate(curves_ndarr)
    )

    return conv_ind_lst

def compute_accuracy(
    target_fill_ratio,
    curves_ndarr: np.ndarray, # dim = (num_trials, num_robots, num_steps+1)
    conv_ind_lst: list # dim = (num_trials, num_robots)
) -> list:
    """Compute the informed estimate accuracy with respect to the target fill ratio.

    The accuracy is computed at the timestep where convergence is found.

    Args:
        target_fill_ratio: The target fill ratio used in the simulation.
        curves_ndarr: A np.ndarray containing the informed estimates from start to end for each robot across all trials.
        conv_ind_lst: The list of convergence indices for all robots across all trials.

    Returns:
        A list of accuracies with the same shape as conv_ind_lst.
    """
    acc_lst = [None for _ in range(curves_ndarr.shape[0])]

    # Iterate through each trial to obtain the accuracy based on the provided convergence index
    for trial_ind, robot_curves_per_trial in enumerate(curves_ndarr):
        err = [
            abs(robot_curves_per_trial[robot_ind, conv_ind] - target_fill_ratio)
                    for robot_ind, conv_ind in enumerate(conv_ind_lst[trial_ind])
        ] # err is a list with num_robots elements

        acc_lst[trial_ind] = err

    return acc_lst # acc_lst has dim = (num_trials, num_robots)

def plot_scatter(df: pd.DataFrame, varying_metric_str: str, **kwargs):
    """Create a scatter plot based on the convergence and accuracy metrics.

    Args:
        df: pd.DataFrame object containing the data.
        varying_metric_str: Column in `df` to plot as separate cases.
    """

    # Process keyword arguments
    leg_title = varying_metric_str if "leg_title" not in kwargs else kwargs["leg_title"]

    fig = []
    ax = []

    if "ax" not in kwargs:
        fig_size = (8, 6)
        fig, ax = plt.subplots(
            tight_layout=True,
            figsize=fig_size,
            dpi=200
        )
    else:
        ax = kwargs["ax"]

    # Get performance metrics for all trials
    conv_min = np.inf
    conv_max = -np.inf
    acc_min = np.inf
    acc_max = -np.inf

    median_lst = []
    varying_metric_vals = list(set(df[varying_metric_str]))

    for metric_ind, val in enumerate(varying_metric_vals):

        df_subset = df.loc[df[varying_metric_str] == val]
        conv_lst = df_subset["conv_step_ind"] # dim = (num_trials, num_robots)
        acc_lst = df_subset["accuracies"] # dim = (num_trials, num_robots)

        # Flatten data
        conv_lst = [conv_per_rob for conv_per_trial in conv_lst for conv_per_rob in conv_per_trial]
        acc_lst = [acc_per_rob for acc_per_trial in acc_lst for acc_per_rob in acc_per_trial]

        # Compute minimum and maximum values
        conv_min = np.amin([conv_min, np.amin(conv_lst)])
        conv_max = np.amax([conv_max, np.amax(conv_lst)])
        acc_min = np.amin([acc_min, np.amin(acc_lst)])
        acc_max = np.amax([acc_max, np.amax(acc_lst)])

        # Calculate median
        median = (np.median(conv_lst), np.median(acc_lst))
        median_lst.append(median)

        # Compute interquartile range
        ellipse_dim = (stats.iqr(conv_lst), stats.iqr(acc_lst))
        ellipse_quantile_25 = (np.quantile(conv_lst, 0.25), np.quantile(acc_lst, 0.25))

        # Compute center coordinates for the ellipse
        ellipse_center = (ellipse_quantile_25[0] + ellipse_dim[0]/2, ellipse_quantile_25[1] + ellipse_dim[1]/2)

        # Plot interquartile ellipse
        ellipse = Ellipse(
            xy=ellipse_center,
            width=ellipse_dim[0],
            height=ellipse_dim[1],
            facecolor=plt.cm.winter(metric_ind/(len(varying_metric_vals)-1)),
            alpha=0.2,
        )

        ax.add_patch(ellipse)

    # Plot median points
    scatter_x, scatter_y = list(zip(*median_lst))

    ax.scatter(
        scatter_x,
        scatter_y,
        c=range(0, len(median_lst)),
        edgecolor="k",
        cmap="winter",
        vmin=0,
        vmax=len(varying_metric_vals) - 1,
        marker="o" if "marker" not in kwargs else kwargs["marker"],
        alpha=1.0,
        s=65
    )

    # Add legend
    legend_elements = []

    if "use_leg_patch" in kwargs and kwargs["use_leg_patch"]:
        legend_elements = [
            Patch(
                label="{0}".format(val),
                facecolor=plt.cm.winter(ind/(len(varying_metric_vals)-1))
            ) for ind, val in enumerate(varying_metric_vals)
        ]
    else:
        legend_elements = [
            mlines.Line2D(
                [], [],
                markersize=8,
                marker="o" if "marker" not in kwargs else kwargs["marker"],
                linestyle="",
                label="{0}".format(val),
                markeredgecolor="k",
                markerfacecolor=plt.cm.winter(ind/(len(varying_metric_vals)-1))
            ) for ind, val in enumerate(varying_metric_vals)
        ]

    ax.legend(handles=legend_elements, title=leg_title, loc="upper right")

    # Set limits
    ax.set_xlabel("Time Steps", fontsize=14)
    ax.set_ylabel("Absolute Error", fontsize=14)
    ax.set_xlim(-0.02 * df["num_steps"][0], 1.02 * df["num_steps"][0])
    ax.set_ylim(-0.05, 1.0)

    # Add grid
    ax.grid("on")

    print("Convergence timestep minimum: {0}, maximum: {1}".format(conv_min, conv_max))
    print("Accuracy error minimum: {0}, maximum: {1}".format(acc_min, acc_max))

    return fig, ax