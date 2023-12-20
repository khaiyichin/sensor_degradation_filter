import pickle
from datetime import datetime
from .general_module import ParamCombo

# Define class to store the VI simulation data
class SimDataVI:

    def __init__(self, param_combo: ParamCombo, A, R):
        self.A = A
        self.R = R
        self.param_combo = param_combo
        self.data_pair = {}

    def store_data_pair(self, param: tuple, target_dist, surrogate_dist, result):

        if param not in self.param_combo.params_lst:
            raise RuntimeWarning("Storing data based on parameter that isn't not found in ParamCombo attribute.")
        elif param in self.data_pair.keys():
            raise RuntimeError("Parameter already exist for data to be stored.")

        self.data_pair[param] = (target_dist, surrogate_dist, result)

    def save(self, filepath=None, curr_time=None):
        # Get current time
        if curr_time is None:
            curr_time = datetime.now().strftime("%m%d%y_%H%M%S")

        if filepath:
            save_path = filepath + "_" + curr_time + ".sdv"
        else:
            save_path = "sim_data_VI_" + curr_time + ".sdv"

        with open(save_path, "wb") as fopen:
            pickle.dump(self, fopen, pickle.HIGHEST_PROTOCOL)

    def get_data_pair(self): return self.data_pair

    def get_param_combo(self): return self.param_combo
