import numpy as np
import pandas as pd
import json
import os
from joblib import Parallel, delayed
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from matplotlib.patches import Ellipse, Patch
import scipy.stats as stats
import plotly.express as px
import plotly.graph_objects as go
# from IPython.display import display, HTML

class StaticDegradationJsonData:
    """
    This class stores data for a given set of parameters for all trials and for all # of flawed robots. E.g., if an experiment
    is run for 25 trials with 1, 2, and 3 flawed robots (i.e., 3 * 25 = 75 .json files), a single instance of this class would
    store all 75 cases.
    """

    def __init__(self, data_folder: str, silent=True):
        self.first_pass = True
        self.silent = silent
        self.sim_type = ""
        self.num_correct_robots = []
        self.num_flawed_robots = []

        self.load_data(data_folder)

    def populate_common_data(self, json_dict):
        self.method = json_dict["method"]
        self.sim_type = json_dict["sim_type"]
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
        self.correct_robot_filter = json_dict["correct_robot_filter"]
        try:
            self.filter_specific_params = json_dict["filter_specific_params"]
        except Exception as e:
            self.filter_specific_params = None

        # Populate topology specific information
        if "static_topo" in self.sim_type:
            self.fully_connected = json_dict["fully_connected"]
            self.comms_range = None
            self.meas_period = None
            self.speed = None
            self.density = None
        elif "dynamic_topo" in self.sim_type:
            self.fully_connected = False
            self.comms_range = json_dict["comms_range"]
            self.meas_period = json_dict["meas_period"]
            self.speed = json_dict["speed"]
            self.density = np.round(json_dict["density"], 3)
        else:
            raise ValueError("Unknown sim_type detected: {0}".format(self.sim_type))

    def print_common_data(self):
        print("method", self.method)
        print("sim_type", self.sim_type)
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
        print("comms_range:", self.comms_range)
        print("meas_period:", self.meas_period)
        print("speed:", self.speed)
        print("density:", self.density)

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
                    [float(val) for val in elem.split(",")[7:9]] for elem in row
                ] for row in data_str_vec
            ]
        ) # row = [rng_seed, n, t, local_est, local_conf, social_est, social_conf, informed_est, sensor_b_est, sensor_w_est]

def process_convergence_accuracy(
    json_data_obj: StaticDegradationJsonData,
    df: pd.DataFrame = pd.DataFrame(),
    threshold = 0.01,
    perf_score_max = 100,
):
    """Finds the point in time when convergence is achieved and its corresponding accuracy, then computing a performance score.

    This finds the convergence timestep (# of observations) for the informed estimates.
    If the returned value is equal to the number of observations, that means convergence was not achieved.
    At the time of convergence, the absoulte error is computed as the accuracy.
    Finally, the convergence timestep and accuracy is combined to compute a performance score.

    Args:
        json_data_obj: StaticDegradationJsonData that contains all the data intended for processing.
        df: pd.DataFrame to concatenate the processed data to.
        threshold: A float parametrizing the difference threshold.
        perf_score_max: Maximum for the performance score. Minimum is 0.

    Returns:
        A pandas.DataFrame with the following columns:
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
            "fully_connected",
            "correct_robot_filter",
            "filter_specific_params",
            "num_flawed_robots",
            "conv_step_ind",
            "accuracies",
            "score"
    """

    # json_data_obj.data is a dict with key=num_flawed_robots and value=np.ndarray with dim = (num_flawed_robots, num_trials, num_robots, num_steps+1, 2)

    # Iterate through each case of # of flawed robots
    for n, data_ndarr_per_num_flawed_robot in json_data_obj.data.items():

        inf_est_curves_ndarr = data_ndarr_per_num_flawed_robot[..., 0] # inf_est_curves_ndarr has dim = (num_trials, num_agents, num_steps+1)

        # Detect convergence time
        conv_ind_lst = detect_convergence(threshold, inf_est_curves_ndarr)

        # Compute accuracy
        acc_lst = compute_accuracy(json_data_obj.tfr, inf_est_curves_ndarr, conv_ind_lst)

        # Get maximum values for convergence time step and accuracy
        max_conv_ind = np.max(conv_ind_lst)
        max_acc = np.max(acc_lst)

        # Compute performance score
        conv_ind_score_lst, acc_score_lst, aggregate_score_lst = compute_performance_score(
            conv_ind_lst,
            acc_lst,
            max_conv_ind,
            max_acc,
            perf_score_max
        )

        data = {
            "method": [json_data_obj.method],
            "sim_type": [json_data_obj.sim_type],
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
            "comms_range": [json_data_obj.comms_range],
            "meas_period": [json_data_obj.meas_period],
            "speed": [json_data_obj.speed],
            "density": [json_data_obj.density],
            "correct_robot_filter": [json_data_obj.correct_robot_filter],
            "filter_specific_params": [json_data_obj.filter_specific_params],
            "num_flawed_robots": [n],
            "conv_step_ind": [conv_ind_lst],
            "max_conv_step": [max_conv_ind],
            "accuracies": [acc_lst],
            "max_acc": [max_acc],
            "conv_scores": [conv_ind_score_lst],
            "acc_scores": [acc_score_lst],
            "agg_scores": [aggregate_score_lst]
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

def compute_performance_score(
    conv_ind_lst: list,
    accuracies_lst: list,
    conv_ind_max_possible_val: float,
    accuracies_max_possible_val: float,
    overall_scale = 100,
):
    """Compute performance scores for a tuple of convergence time and accuracy.

    The output score has a maximum of overall_scale and minimum of 0.

    Args:
        conv_ind_lst: List of convergence time indices (dim = num_trials x num_robots)
        accuracies_lst: List of accuracies (dim = num_trials x num_robots)
        conv_ind_max_possible_val: Maximum possible value for convergence time; used to scale the exponential function
        accuracies_max_possible: Maximum possible value for accuracy; used to scale the exponential function

    Returns:
        A tuple containing:
            a list of scaled individual robot convergence scores for all trials (dim = num_trials x num_robots)
            a list of scaled individual robot accuracy scores for all trials (dim = num_trials x num_robots)
            a list of aggregate scores (dim = num_trials)
    """

    # Equate NaNs to max_value (only applicable to accuracies) and convert to numpy ndarray
    accuracies_arr = np.nan_to_num(accuracies_lst, nan=accuracies_max_possible_val)
    conv_ind_arr = np.asarray(conv_ind_lst)

    # Define functions to compute individual scores and scale factors
    compute_indi_scores = lambda arr, max_val: overall_scale * (
        1.0 - np.divide(arr, max_val)
    )
    compute_scale_factor = lambda arr, max_val: np.exp(
        -stats.iqr(arr, axis=1, interpolation="midpoint") / max_val
    )

    # Compute individual scores
    individual_conv_ind_scores = compute_indi_scores(conv_ind_arr, conv_ind_max_possible_val) # dim = num_trials x num_robots
    individual_accuracy_scores = compute_indi_scores(accuracies_arr, accuracies_max_possible_val) # dim = num_trials x num_robots

    # Compute scale factors
    conv_ind_scale_factor = compute_scale_factor(conv_ind_arr, conv_ind_max_possible_val) # dim = num_trials
    accuracy_scale_factor = compute_scale_factor(accuracies_arr, accuracies_max_possible_val) # dim = num_trials

    # Compute the combined scores
    combined_score = 0.5 * (
        conv_ind_scale_factor * np.mean(individual_conv_ind_scores, axis=1) +
        accuracy_scale_factor * np.mean(individual_accuracy_scores, axis=1)
    ) # dim = num_trials

    return (
        (conv_ind_scale_factor.reshape(-1, 1) * individual_conv_ind_scores).tolist(),
        (accuracy_scale_factor.reshape(-1, 1) * individual_accuracy_scores).tolist(),
        combined_score.tolist()
    )

def process_decision(
    json_data_obj: StaticDegradationJsonData,
    num_bins = 10,
    step_inc = 5000
):
    """Process the data to obtain the swarm's fraction of correct decision.

    The columns that represent the time series decision fraction data would appear
    as follows:

    +----+---------------+-------------------+-------------------------|
    ...  | "trial_index" |  "decision_time"  |  "decision_fraction"    |
    +----+---------------+-------------------+-------------------------|
    ...  |  trial_ind_1  |  decision_time_1  |  decision_fraction_11   |
    ...  |  trial_ind_1  |  decision_time_2  |  decision_fraction_12   |
    ...  |  trial_ind_1  |  decision_time_3  |  decision_fraction_13   |
    ...  |  trial_ind_1  |  ...              |  decision_fraction_1... |
    ...  |  trial_ind_1  |  decision_time_n  |  decision_fraction_1n   |
    ...  |  trial_ind_2  |  decision_time_1  |  decision_fraction_21   |
    ...  |  trial_ind_2  |  decision_time_2  |  decision_fraction_21   |
    ...  |  trial_ind_2  |  decision_time_3  |  decision_fraction_21   |
    ...  |  trial_ind_2  |  ...              |  decision_fraction_2... |
    ...  |  trial_ind_2  |  decision_time_n  |  decision_fraction_2n   |
    ...  |  ...          |  ...              |  decision_fraction_...  |
    ...  |  trial_ind_m  |  decision_time_1  |  decision_fraction_m1   |
    ...  |  trial_ind_m  |  decision_time_2  |  decision_fraction_m2   |
    ...  |  trial_ind_m  |  decision_time_3  |  decision_fraction_m3   |
    ...  |  trial_ind_m  |  ...              |  decision_fraction_m... |
    ...  |  trial_ind_m  |  decision_time_n  |  decision_fraction_mn   |


    Args:
        json_data_obj: Object that contains experiment data
        num_bins: Number of bins to split the interval [0, 1] into so that estimates can be converted to decisions
        step_inc: Simulation step period to evaluate decisions
    Returns:
        Pandas DataFrame that contain decision data of all trials within json_data_obj
    """
    # json_data_obj.data is a dict with key=num_flawed_robots and value=np.ndarray with dim = (num_flawed_robots, num_trials, num_robots, num_steps+1, 2)

    # Calculate the timesteps at which the decision is obtained
    decision_time = list(range(step_inc, json_data_obj.num_steps+1, step_inc))

    decision_dfs = []

    # Iterate through each case of # of flawed robots
    def parallel_obtain_decision_fraction(n, data_ndarr_per_num_flawed_robot):

        inf_est_curves_ndarr = data_ndarr_per_num_flawed_robot[..., 0] # inf_est_curves_ndarr has dim = (num_trials, num_agents, num_steps+1)

        # Compute decision fractions
        decision_arr = compute_decision(inf_est_curves_ndarr, json_data_obj.tfr, num_bins, decision_time)

        # Create data dict
        data = {
            "method": np.repeat(json_data_obj.method, len(decision_time) * json_data_obj.num_trials),
            "sim_type": np.repeat(json_data_obj.sim_type, len(decision_time) * json_data_obj.num_trials),
            "num_trials": np.repeat(json_data_obj.num_trials, len(decision_time) * json_data_obj.num_trials),
            "num_robots": np.repeat(json_data_obj.num_robots, len(decision_time) * json_data_obj.num_trials),
            "num_steps": np.repeat(json_data_obj.num_steps, len(decision_time) * json_data_obj.num_trials),
            "comms_period": np.repeat(json_data_obj.comms_period, len(decision_time) * json_data_obj.num_trials),
            "fully_connected": np.repeat(json_data_obj.fully_connected, len(decision_time) * json_data_obj.num_trials),
            "comms_range": np.repeat(json_data_obj.comms_range, len(decision_time) * json_data_obj.num_trials),
            "meas_period": np.repeat(json_data_obj.meas_period, len(decision_time) * json_data_obj.num_trials),
            "speed": np.repeat(json_data_obj.speed, len(decision_time) * json_data_obj.num_trials),
            "density": np.repeat(json_data_obj.density, len(decision_time) * json_data_obj.num_trials),
            "correct_robot_filter": np.repeat(json_data_obj.correct_robot_filter, len(decision_time) * json_data_obj.num_trials),
            "filter_specific_params": np.repeat(json_data_obj.filter_specific_params, len(decision_time) * json_data_obj.num_trials),
            "sensor_filter_period": np.repeat(json_data_obj.sensor_filter_period, len(decision_time) * json_data_obj.num_trials),
            "tfr": np.repeat(json_data_obj.tfr, len(decision_time) * json_data_obj.num_trials),
            "flawed_sensor_acc_b": np.repeat(json_data_obj.flawed_sensor_acc_b, len(decision_time) * json_data_obj.num_trials),
            "flawed_sensor_acc_w": np.repeat(json_data_obj.flawed_sensor_acc_w, len(decision_time) * json_data_obj.num_trials),
            "correct_sensor_acc_b": np.repeat(json_data_obj.correct_sensor_acc_b, len(decision_time) * json_data_obj.num_trials),
            "correct_sensor_acc_w": np.repeat(json_data_obj.correct_sensor_acc_w, len(decision_time) * json_data_obj.num_trials),
            "density": np.repeat(json_data_obj.density, len(decision_time) * json_data_obj.num_trials),
            "num_flawed_robots": np.repeat(n, len(decision_time) * json_data_obj.num_trials),
            "trial_index": np.repeat(range(json_data_obj.num_trials), len(decision_time)),
            "decision_time": np.tile(decision_time, json_data_obj.num_trials),
            "decision_fraction": decision_arr
        }

        return pd.DataFrame(data)

    decision_dfs = Parallel(n_jobs=-1, verbose=0)(
        delayed(parallel_obtain_decision_fraction)(n, data_ndarr_per_num_flawed_robot) for n, data_ndarr_per_num_flawed_robot in json_data_obj.data.items()
    )

    return pd.concat(decision_dfs, axis=0, ignore_index=True)

def compute_decision(curves_ndarr, target_fill_ratio, num_bins, decision_time):

    # Create bins for the estimates
    bin_arr = np.linspace(0.0, 1.0, num_bins+1) # linspace gives the "checkpoints" for the bins, thus the bins are actually between each values

    # Define correct bin decision
    correct_bin = np.digitize(target_fill_ratio, bin_arr) # the output is between 1 to the number of bins, i.e., 1st bin, 2nd bin, etc.

    # Extract estimates
    estimates = curves_ndarr[:,:,decision_time]

    # Convert estimates to decisions
    decisions = np.digitize(estimates, bin_arr) # dim = (num_trials, num_robots, len(decision_time))

    # Count the fraction of correct decisions
    fractions = np.count_nonzero(decisions == correct_bin, axis=1) / curves_ndarr.shape[1] # dim = (num_trials, len(decision_time))

    return fractions.flatten() # dim = num_trials * len(decision_time)

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
    varying_metric_vals = []

    if varying_metric_str != "filter_specific_params":
        varying_metric_vals = sorted(list(set(df[varying_metric_str])))
    else:
        try:
            k = kwargs["filter_specific_params_key"]

            if k not in df[varying_metric_str].iloc[0]: raise KeyError("Incorrect \"filter_specific_params_key\".")
        except Exception:
            raise KeyError("You need to provide a \"filter_specific_params_key\" argument.")

        varying_metric_vals = set(
            row[k] for row in df[varying_metric_str]
        )

        varying_metric_vals = sorted(list(varying_metric_vals))

    for metric_ind, val in enumerate(varying_metric_vals):
        median_lst = []

        # Apply specific processing for filter_specific_params
        if varying_metric_str != "filter_specific_params":
            df_subset = df.loc[df[varying_metric_str] == val]
        else:
            df_subset = df.loc[
                df[varying_metric_str].apply(lambda x: kwargs["filter_specific_params_key"] in x.keys() and val in x.values())
            ] # search for rows containing the value based on the specific key

        df_subset.reset_index(drop=True)

        for exp_ind in range(len(df_subset)):
            conv_lst = df_subset["conv_step_ind"].iloc[exp_ind] # dim = (num_trials, num_robots)
            acc_lst = df_subset["accuracies"].iloc[exp_ind] # dim = (num_trials, num_robots)

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
                alpha=0.1,
                zorder=-1
            )

            ax.add_patch(ellipse)

        # Plot median points
        scatter_x, scatter_y = list(zip(*median_lst))

        ax.scatter(
            scatter_x,
            scatter_y,
            facecolor=plt.cm.winter(metric_ind/(len(varying_metric_vals)-1)),
            edgecolor="k",
            marker="o" if "marker" not in kwargs else kwargs["marker"],
            alpha=1.0,
            s=40
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
    if "xlim" in kwargs: ax.set_xlim(kwargs["xlim"])
    if "ylim" in kwargs: ax.set_ylim(kwargs["ylim"])

    # Add grid
    ax.grid("on")

    print("Convergence timestep minimum: {0}, maximum: {1}".format(conv_min, conv_max))
    print("Accuracy error minimum: {0}, maximum: {1}".format(acc_min, acc_max))

    return fig, ax

################################################################################
# Expand the lists in the aggregate score column into multiple rows
################################################################################

def expand_indi_scores(target_df, csv_filename=None):
    """Expand the individual accuracy and convergence scores into respective single-element rows and add accuracy difference column
    """

    dfs = [
        pd.DataFrame() for _ in range(len(target_df.index))
    ]
    params_only_dfs = target_df.drop([
        "conv_step_ind",
        "accuracies",
        "conv_scores",
        "acc_scores",
        "agg_scores"
    ], axis=1) # dataframe containing only parameter columns

    # Go through each row to expand the `indi_scores` column
    for ind, row in enumerate(target_df.itertuples()):
        current_conv_score_df = pd.DataFrame({"conv_scores": np.mean(row.conv_scores, axis=1)})
        current_acc_score_df = pd.DataFrame({"acc_scores": np.mean(row.acc_scores, axis=1)})

        # Check for NaNs (should never be true)
        if current_conv_score_df.isnull().values.any() or current_acc_score_df.isnull().values.any():
            print()
            raise ValueError("NaNs found in the {0}-th row!".format(ind))

        # Repeat the current parameters
        current_params_df = params_only_dfs.loc[[ind]]
        repeated_current_params_df = current_params_df.loc[
            current_params_df.index.repeat(row.num_trials)
        ].reset_index(drop=True)

        # Combine parameter rows with data rows
        dfs[ind] = pd.concat([repeated_current_params_df, current_conv_score_df, current_acc_score_df], axis=1)

    # Combine all into a single DataFrame
    final_df = pd.concat(dfs, axis=0, ignore_index=True)

    # Save a CSV copy of the new DataFrame
    if csv_filename: final_df.to_csv(csv_filename, index=False)

    return final_df

################################################################################
################################################################################



################################################################################
# Expand the lists in the aggregate score column into multiple rows
################################################################################

def expand_agg_scores(target_df, csv_filename=None):
    """Expand the `agg_scores` column into single-element rows and add accuracy difference column
    """

    dfs = [
        pd.DataFrame() for _ in range(len(target_df.index))
    ]
    params_only_dfs = target_df.drop([
        "conv_step_ind",
        "accuracies",
        "conv_scores",
        "acc_scores",
        "agg_scores"
    ], axis=1) # dataframe containing only parameter columns

    # Go through each row to expand the `agg_scores` column
    for ind, row in enumerate(target_df.itertuples()):
        current_score_df = pd.DataFrame({"agg_scores": row.agg_scores})

        # Check for NaNs (should never be true)
        if current_score_df.isnull().values.any():
            print("NaNs found in the {0}-th row!".format(ind))
            break

        # Repeat the current parameters
        current_params_df = params_only_dfs.loc[[ind]]
        repeated_current_params_df = current_params_df.loc[
            current_params_df.index.repeat(row.num_trials)
        ].reset_index(drop=True)

        # Combine parameter rows with data rows
        dfs[ind] = pd.concat([repeated_current_params_df, current_score_df], axis=1)

    # Combine all into a single DataFrame
    final_df = pd.concat(dfs, axis=0, ignore_index=True)

    # Save a CSV copy of the new DataFrame
    if csv_filename: final_df.to_csv(csv_filename, index=False)

    return final_df

################################################################################
################################################################################



################################################################################
# Add a column for the difference between actual and assumed accuracies
################################################################################

def compute_diff(target_df, csv_filename=None):
    """Create a column that records the difference between acutal and assumed accuracies
    """
    target_df["acc_diff"] = np.round(target_df["flawed_sensor_acc_b"] - target_df["correct_sensor_acc_b"], 3)

    # Save a CSV copy of the new DataFrame
    if csv_filename: target_df.to_csv(csv_filename, index=False)

    return target_df

################################################################################
################################################################################



################################################################################
# Add a column for the percentage of flawed robot quantity
################################################################################
def compute_flawed_percentage(target_df, csv_filename=None):

    num_flawed_robots_lst = target_df["num_flawed_robots"].unique()

    target_df["num_flawed_robots_pct"] = target_df["num_flawed_robots"].replace(
        {
            old_val: old_val * 100 // target_df["num_robots"].iat[0]
            for old_val in num_flawed_robots_lst
        }
    )

    # Save a CSV copy of the new DataFrame
    if csv_filename: target_df.to_csv(csv_filename, index=False)

    return target_df

################################################################################
################################################################################



################################################################################
# Create a boxplot figure
################################################################################

# # Workaround to enable LaTeX in Plotly figures using VSCode
# # see: https://github.com/microsoft/vscode-jupyter/issues/8131#issuecomment-1589961116
# plotly.offline.init_notebook_mode()
# display(HTML(
#     "<script type='text/javascript' async src='https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-MML-AM_SVG'></script>"
# ))

COLOR_BLIND_FRIENDLY_COLORS_DICT_HEX = {
    "black": "#000000",
    "orange": "#E69F00",
    "sky_blue": "#56B4E9",
    "green": "#009E73",
    "yellow": "#F0E442",
    "blue": "#0072B2",
    "vermillion": "#D55E00",
    "pink": "#CC79A7"
}

COLOR_BLIND_FRIENDLY_COLORS_DICT_RGBA = {
    "black": "rgba(0, 0, 0, 255)",
    "orange": "rgba(230, 159, 0, 255)",
    "sky_blue": "rgba(86, 180, 233, 255)",
    "green": "rgba(0, 158, 115, 255)",
    "yellow": "rgba(240, 228, 66, 255)",
    "blue": "rgba(0, 114, 178, 255)",
    "vermillion": "rgba(213, 94, 0, 255)",
    "pink": "rgba(204, 121, 167, 255)"
}

def plot_boxplot_plotly(
    target_df,
    y_key: str,
    x1_key: str,
    color_key: str,
    **kwargs
):
    """

    Args:
        target_df: Pandas DataFrame containing the data to plot.
        y_key: Key to the `df` column that represents the y-axis.
        x1_key: Key to the `df` column that represents the outer x-axis.
        color_key: Key to the `df` column for the boxes, to be each plotted as
            a different color within a single (x1-x2) combination.
        kwargs: Specific keyword arguments

    Returns:
        Handle to the created boxplot figure.
    """

    x2_key = None
    x2_labels = None
    x2_values = None
    if "x2_key" in kwargs: x2_key = kwargs["x2_key"]

    # Create copy of data
    df = pd.DataFrame(target_df, copy=True)

    # Get spacing values
    x1_spacing = 0.35 if "x1_spacing" not in kwargs else kwargs["x1_spacing"]
    x2_spacing = 0.15 if "x2_spacing" not in kwargs else kwargs["x2_spacing"]

    # Apply offset for x1 group (absolute)
    x1_values = sorted(df[x1_key].unique(), reverse=False if "x1_reverse" not in kwargs else kwargs["x1_reverse"])
    df["x1 offset"] = df[x1_key].replace(
        {
            k: np.round(ind*x1_spacing, 2)
            for ind, k in enumerate(x1_values)
        }
    )

    # Apply offset for x2 group (relative to x1 group)
    if x2_key:
        x2_placement = {}
        x2_values = sorted(df[x2_key].unique(), reverse=False if "x2_reverse" not in kwargs else kwargs["x2_reverse"])

        # Check if there are x2 values that need special treatment
        x2_special_val = np.nan # for special case

        if "x2_special_val" in kwargs:
            x2_special_val = x2_values.pop(x2_values.index(kwargs["x2_special_val"]))
            x2_placement.update({x2_special_val: 0.0})

        x2_placement.update({
            k: np.round(ind*x2_spacing - ((len(x2_values)-1) * x2_spacing/2), 2) # centers about the x1 label
            for ind, k in enumerate(x2_values)
        })

        df["x2 offset"] = df[x2_key].replace(x2_placement)

        # Create column for overall offset values
        df["x_axis_locations"] = df["x1 offset"] + df["x2 offset"]
    else:
        df["x_axis_locations"] = df["x1 offset"]

    fig = go.Figure()

    # Check for category orders if we need to order the color_key
    if "category_orders" in kwargs:
        for k, v in kwargs["category_orders"].items():
            df[k] = pd.Categorical(
                df[k], categories=v, ordered=True
            )

        # Sort the columns
        df.sort_values(by=list(kwargs["category_orders"].keys()), inplace=True)

    # Plot each trace, iterating by color
    for ind, trace_val in enumerate(df[color_key].unique()):
        current_df = df.loc[df[color_key] == trace_val]

        box_trace = go.Box(
            y=current_df[y_key].values,
            x=current_df["x_axis_locations"].values,
            name=trace_val,
            marker_color=None if "box_colors" not in kwargs else kwargs["box_colors"][ind],
            whiskerwidth=0.8,
            marker={
                "size": 15 if "marker_size" not in kwargs else kwargs["marker_size"]
            }
        )

        fig.add_trace(box_trace)

    # Modify IQR method
    fig.update_traces(quartilemethod="inclusive" if "quartilemethod" not in kwargs else kwargs["quartilemethod"])  # Or "inclusive"

    # Update axes labels
    x1_labels = [
        "{0}{1}{2}".format("" if "x1_label_prefix" not in kwargs else kwargs["x1_label_prefix"], v, "" if "x1_label_suffix" not in kwargs else kwargs["x1_label_suffix"])
        for v in x1_values
    ]
    if x2_key:
        x2_labels = [
            "{0}{1}{2}".format("" if "x2_label_prefix" not in kwargs else kwargs["x2_label_prefix"], v, "" if "x2_label_suffix" not in kwargs else kwargs["x2_label_suffix"])
            for v in x2_values
        ]

    # Update layout for better visibility
    fig.update_layout(
        boxmode="group",
        boxgroupgap=0.1 if "boxgroupgap" not in kwargs else kwargs["boxgroupgap"],
        xaxis_title_standoff=10,  # Adjust space between axis title and axis labels
        margin=dict(l=3, r=3, t=3 if ("show_title" in kwargs and kwargs["show_title"] == False) or "title" not in kwargs else 40, b=3),  # Adjust margins for better spacing
        yaxis=go.layout.YAxis(
            title=None if "y_title" not in kwargs or ("show_y" in kwargs and not kwargs["show_y"]) else kwargs["y_title"] ,
            mirror="ticks",
            showline=True,
            linewidth=1,
            linecolor="black",
            ticks="outside",
            tickprefix="",
            ticksuffix="",
            tickfont={"size": 25 if "main_tick_font_size" not in kwargs else kwargs["main_tick_font_size"]},
            range=None if "y_lim" not in kwargs else kwargs["y_lim"],
            minor={"showgrid": True},
            showticklabels=True if "show_y" not in kwargs else kwargs["show_y"],
            gridcolor="#bbbbbb",
            gridwidth=1,
            zerolinecolor="#bbbbbb",
            zerolinewidth=1,
            side="left" if "y_side" not in kwargs else kwargs["y_side"]
        ),
        xaxis=go.layout.XAxis(
            mirror=True,
            showline=True,
            linewidth=1,
            linecolor="black",
            tickvals=[
                df.loc[df[x1_key] == val, "x1 offset"].iat[0]
                for val in x1_values
            ],
            ticktext=x1_labels,
            ticks="outside",
            tickfont={"size": 25 if "main_tick_font_size" not in kwargs else kwargs["main_tick_font_size"]},
            ticklen=6,
            title="" if "x1_title" not in kwargs or ("show_x1" in kwargs and not kwargs["show_x1"]) else kwargs["x1_title"],
            titlefont={"size": 26},
            showticklabels=True if "show_x1" not in kwargs else kwargs["show_x1"],
            side="bottom" if "x1_side" not in kwargs else kwargs["x1_side"]
        ), # Explicit tick values and labels
        xaxis2=go.layout.XAxis(
            overlaying="x",
            ticks="inside",
            ticklabelposition="inside",
            tickcolor="rgba(0,0,0,0.0)",
            ticklen=5,
            tickvals=[
                df.loc[
                    (df[x1_key] == x1_val) & (df[x2_key] == x2_val),
                    "x_axis_locations"
                ].iat[0]
                for x1_val in x1_values for x2_val in x2_values if ((df[x1_key] == x1_val) & (df[x2_key] == x2_val)).any() # to prevent calling iat[0] when the dataframe cannot find the match of x1_val and x2_val
            ],
            ticktext=np.tile(x2_labels, len(x1_values)).tolist(),
            tickfont={"size": 24 if "x2_tick_font_size" not in kwargs else kwargs["x2_tick_font_size"]},
            side="bottom" if "x2_side" not in kwargs else kwargs["x2_side"]
        ) if x2_key else None,
        legend=go.layout.Legend(
            title="" if "legend_title" not in kwargs else kwargs["legend_title"],
            font={"size": 15},
            orientation="h",
            xref="container",
            yref="container",
            x=None if "legend_x" not in kwargs else kwargs["legend_x"],
            y=None if "legend_y" not in kwargs else kwargs["legend_y"]
        ),
        showlegend=True if "show_legend" not in kwargs else kwargs["show_legend"]
    )

    # Place an invisible trace on the inner x-axis, to avoid automatic hiding of the axis
    if x2_key: fig.add_trace(
        go.Bar(x=df["x_axis_locations"],y=np.zeros(len(df)), xaxis="x2", opacity=0.0, showlegend=False)
    )

    fig.show()

    if "output_path" in kwargs:
        fig.write_image(
            kwargs["output_path"],
            width=1300,
            height=650,
            scale=2.0
        )

    return fig