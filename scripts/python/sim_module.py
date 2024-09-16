import notebooks.single_robot_sim_module as srsm

import numpy as np
from scipy import stats
import json

class SimParam:

    def __init__(self, yaml_config):

        # Common parameters
        self.num_robots = int(yaml_config["numRobots"])
        self.comms_period = int(yaml_config["commsPeriod"])
        self.flawed_b = float(yaml_config["flawedSensorAccB"])
        self.flawed_w = float(yaml_config["flawedSensorAccW"])
        self.correct_b = float(yaml_config["correctSensorAccB"])
        self.correct_w = float(yaml_config["correctSensorAccW"])
        self.sensor_cov = float(yaml_config["flawedSensorCovariance"])
        self.tfr = float(yaml_config["targFillRatio"])
        self.sensor_filter_period = int(yaml_config["sensorFilterPeriod"])
        self.num_steps = int(yaml_config["numSteps"])
        self.num_trials = int(yaml_config["numTrials"])
        self.fully_connected = bool(yaml_config["fullyConnected"])
        self.method = str(yaml_config["method"])
        self.correct_robot_filter = bool(yaml_config["correctFilter"])
        if (yaml_config["filterSpecificParams"]):
            self.filter_specific_params = yaml_config["filterSpecificParams"]

        # Flawed robot range
        flawed_robots_min = int(yaml_config["flawedRobotRange"]["min"])
        flawed_robots_max = int(yaml_config["flawedRobotRange"]["max"])
        flawed_robots_steps = int(yaml_config["flawedRobotRange"]["incSteps"])
        self.flawed_robot_range = np.arange(
            flawed_robots_min,
            flawed_robots_max+1,
            flawed_robots_steps,
            dtype=np.int32
        ).tolist()

        self.create_folder_suffix()

    def create_folder_suffix(self):
        """Create suffix for folder name.
        """

        if len(self.flawed_robot_range) == 1:
            flawed_robot_inc = 0            
        else:
            flawed_robot_inc = self.flawed_robot_range[1] - self.flawed_robot_range[0]

        min_num_flawed_robots, max_num_flawed_robots = int(self.flawed_robot_range[0]), int(self.flawed_robot_range[-1])
        flawed_b, flawed_w = int(self.flawed_b * 1.0e3), int(self.flawed_w * 1.0e3)
        correct_b, correct_w = int(self.correct_b * 1.0e3), int(self.correct_w * 1.0e3)

        flawed_robot_suffix = "_flw" + str(min_num_flawed_robots) + "-" + str(flawed_robot_inc) + "-" + str(max_num_flawed_robots)
        flawed_sensor_acc_suffix = "_flwb" + str(flawed_b) + "_flww" + str(flawed_w)
        correct_sensor_acc_suffix = "_corb" + str(correct_b) + "_corw" + str(correct_w)
        comms_period_suffix = "_commsp" + str(self.comms_period)
        sensor_filter_period_suffix = "_filtp" + str(self.sensor_filter_period)
        correct_filter_suffix = "_corfilt" + str(int(self.correct_robot_filter))

        self.suffix = "_t" + str(self.num_trials) + "_s" + str(self.num_steps) + "_tfr" + str(int(self.tfr * 1.0e3)) + \
            flawed_robot_suffix + \
            flawed_sensor_acc_suffix + \
            correct_sensor_acc_suffix + \
            comms_period_suffix + \
            sensor_filter_period_suffix + \
            correct_filter_suffix

class MultiRobotSimStaticDegradation:

    def __init__(self, param: SimParam, trial_ind: int, num_flawed_robots: int):
        self.env = srsm.Environment(param.tfr)
        self.num_robots = param.num_robots
        self.num_correct_robots = self.num_robots - num_flawed_robots
        self.init_b = param.flawed_b
        self.init_w = param.flawed_w
        self.sensor_cov = param.sensor_cov
        self.act_b = param.correct_b
        self.act_w = param.correct_w
        self.tfr = param.tfr
        self.sensor_filter_period = param.sensor_filter_period
        self.comms_period = param.comms_period
        self.T = param.num_steps
        self.fully_connected = param.fully_connected
        self.method = param.method
        self.correct_robot_filter = param.correct_robot_filter

        if self.method == "BRAVO":
            self.filter_specific_params_lst = [float(param.filter_specific_params["type_1_err_prob"])]
        else:
            self.filter_specific_params_lst = [-1.0]

        self.num_flawed_robots = num_flawed_robots
        self.trial_ind = trial_ind

        self.data_dict = {
            "sim_type": "static_topo_static_degrad_1d",
            "num_robots": self.num_robots,
            "num_correct_robots": self.num_correct_robots,
            "num_flawed_robots": self.num_flawed_robots,
            "comms_period": self.comms_period,
            "num_trials": param.num_trials,
            "num_steps": self.T,
            "sensor_filter_period": param.sensor_filter_period,
            "tfr": self.tfr,
            "flawed_sensor_acc_b": self.init_b,
            "flawed_sensor_acc_w": self.init_w,
            "flawed_sensor_cov": self.sensor_cov,
            "correct_sensor_acc_b": self.act_b,
            "correct_sensor_acc_w": self.act_w,
            "fully_connected": self.fully_connected,
            "method": self.method,
            "trial_ind": self.trial_ind,
            "correct_robot_filter": self.correct_robot_filter,
            "filter_specific_params": self.filter_specific_params_lst,
            "data_str": []
        }

        self.robot_data_str = [["" for _ in range(self.T+1)] for _ in range(self.num_robots)]
        self.robot_local_estimates = np.zeros((self.num_robots, self.T+1))
        self.robot_social_estimates = np.zeros((self.num_robots, self.T+1))
        self.robot_informed_estimates = np.zeros((self.num_robots, self.T+1))
        self.robot_sensor_b_estimates = np.zeros((self.num_robots, self.T+1))
        self.robot_sensor_w_estimates = np.zeros((self.num_robots, self.T+1))

        # Initialize all the robots and their occurrences
        if not self.correct_robot_filter:
            self.correct_robots = [
                srsm.Robot(
                    1, # static degradation
                    0.0, # no variance
                    np.array([[self.act_b],[self.act_w]]),
                    np.array([[self.act_b],[self.act_w]]),
                    np.array([[self.sensor_cov],[self.sensor_cov]])
                ) for _ in range(self.num_correct_robots)
            ]
        else:
            self.correct_robots = [
                srsm.RobotStaticDegradation(
                    param.method,
                    np.array([[self.act_b],[self.act_w]]),
                    np.array([[self.act_b],[self.act_w]]),
                    np.array([[self.sensor_cov],[self.sensor_cov]]),
                    self.filter_specific_params_lst
                ) for _ in range(self.num_correct_robots)
            ]
        self.flawed_robots = [
            srsm.RobotStaticDegradation(
                param.method,
                np.array([[self.act_b],[self.act_w]]),
                np.array([[self.init_b],[self.init_w]]), # assuming that we know what it starts out very well
                np.array([[self.sensor_cov],[self.sensor_cov]]),
                self.filter_specific_params_lst
            ) for _ in range(self.num_flawed_robots)
        ]
        self.robots = self.correct_robots + self.flawed_robots # flawed robots are appended at the back

        # Simulate observation for all the robots
        self.occurrences = [self.env.get_occurrences(self.T) for _ in range(self.num_robots)]

        # Initialize random number generator for neighbor selection
        p = 0.1 # so that we get a right skew
        self.num_neighbor_selection = stats.binom(self.num_robots, p)

    def run(self):

        # Log initial data
        self.log_data(0)

        # Run simulation
        for t in range(1, self.T+1):

            self.step(t)

            # Run sensor filter
            if t % self.sensor_filter_period == 0:
                self.sensor_filter_step()

            self.log_data(t)

    def step(self, t):

        # Get the local estimates
        [robot.compute_local_estimate(occurrence=self.occurrences[ind][t-1]) for ind, robot in enumerate(self.robots)]
        local_est = [robot.get_local_estimate() for robot in self.robots]

        # Iterate through flawed robots to do social estimation
        if t % self.comms_period == 0:
            for ind, robot in enumerate(self.robots):

                # Select a random number of robots to communicate with from both the correct and flawed groups
                selected_quantity = self.num_robots if self.fully_connected else self.num_neighbor_selection.rvs(size=1)

                x_hat_arr = ()
                alpha_arr = ()

                if selected_quantity > 0:
                    indices = [i for i in range(self.num_robots) if i != ind] if self.fully_connected else np.random.randint(0, self.num_robots, selected_quantity)
                    x_hat_arr, alpha_arr = zip(*[est_conf_tuple for i, est_conf_tuple in enumerate(local_est) if i in indices])
                    robot.compute_social_estimate(x_hat_arr, alpha_arr)

        # Do informed estimation
        [robot.compute_informed_estimate() for robot in self.robots]

    def sensor_filter_step(self):

        start_ind = 0 if self.correct_robot_filter else self.num_correct_robots

        [self.robots[i].run_sensor_degradation_filter() for i in range(start_ind, self.num_robots)]

    def log_data(self, t):

        # Log sensor quality
        for ind, robot in enumerate(self.robots):
            est_vals = robot.get_est_sensor_quality()
            local_est, local_conf = robot.get_local_estimate()
            social_est, social_conf = robot.get_social_estimate()
            n, t = robot.get_observations()

            data_str = "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},".format(
                -1, # placholder for the rng seed; this is to mirror the data string in the dynamic topo experiment, but the
                    # seed in this case isn't useful for us because we can't reproduce the experiment with just the see alone
                n,
                t,
                np.round(local_est, 6),
                np.round(local_conf, 6),
                np.round(social_est, 6),
                np.round(social_conf, 6),
                np.round(robot.get_informed_estimate(), 6),
                np.round(est_vals.x[0,0], 6),
                np.round(est_vals.x[1,0], 6)
            )

            self.robot_data_str[ind][t] = data_str

    def save_data(self):

        self.data_dict["data_str"] = self.robot_data_str

        path = "flw" + str(self.num_flawed_robots) + "_t" + str(self.trial_ind) + ".json"

        with open(path, "w") as outfile:
            json.dump(self.data_dict, outfile, indent=4)