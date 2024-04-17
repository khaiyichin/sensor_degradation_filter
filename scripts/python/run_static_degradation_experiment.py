import sim_module as sm
import yaml
import argparse
# from joblib import Parallel, delayed
from datetime import datetime
import timeit
import os
import sys

def parse_yaml_param_file(yaml_filepath):
    """Parse the YAML parameter file.
    """

    with open(yaml_filepath) as fopen:
        try:
            config = yaml.safe_load(fopen)
        except yaml.YAMLError as exception:
            print(exception)
    
        # Displayed processed arguments
        print("\n\t" + "="*15 + " Processed Parameters " + "="*15 + "\n")
        print("\t\t", end="")
        for line in yaml.dump(config, indent=4, default_flow_style=False):        
            if line == "\n":
                print(line, "\t\t",  end="")
            else:
                print(line, end="")
        print("\r", end="") # reset the cursor for print
        print("\n\t" + "="*13 + " End Processed Parameters " + "="*13  + "\n")

    param_obj = sm.SimParam(config)

    return param_obj

def create_data_folder():
    """Create a folder named data and use it as the working directory.
    """

    # Create the `data` folder if it doesn't exist
    try:
        os.mkdir("data")
        os.chdir("data")
        
    except FileExistsError:

        # Change to `data` directory
        try:
            os.chdir("data")
        except Exception:
            pass

def create_simulation_folder(suffix, curr_time):
    """Create a simulation folder within the `data` directory.
    """

    # Create data folder and change working directory
    create_data_folder()

    # Define folder name
    folder_name = curr_time + suffix

    # Create folder to store simulation data based on current datetime if doesn't exist (shouldn't exist, really)
    try:
        os.mkdir(folder_name)
        os.chdir(folder_name)

    except FileExistsError:
        print("folder {} already exists".format(folder_name))

        try:
            os.chdir(folder_name)
        except Exception:
            pass

def main():
    parser = argparse.ArgumentParser(description="Execute multi-agent simulation with static topologies and static degradation.")
    parser.add_argument("FILE", type=str, help="path to the parameter file relative to the current/execution directory")
    parser.add_argument("-p", action="store_true", help="flag to use cores to run simulations in parallel")
    args = parser.parse_args()

    # Obtain timestamp
    curr_time = datetime.now().strftime("%m%d%y_%H%M%S")

    # Parse simulation parameters
    param_obj = parse_yaml_param_file(args.FILE)

    # Create a folder to store simulation data
    create_simulation_folder(param_obj.suffix, curr_time)

    if args.p:
        raise NotImplementedError("Parallel mode not implemented yet.")
    else:

        for num_flawed_robots in param_obj.flawed_robot_range:
            print("\nRunning case with # flawed robots = " + str(num_flawed_robots))

            for trial in range(param_obj.num_trials):
                start = timeit.default_timer()
                print("\tRunning trial number = " + str(trial) + "... ", end="")
                sys.stdout.flush()

                s = sm.MultiRobotSimStaticDegradation(param_obj, trial, num_flawed_robots)

                s.run()

                # Write data to JSON file
                s.save_data()

                print("Done!\n")

                end = timeit.default_timer()

                print("\t-- Elapsed time:", end-start, "--\n")

if __name__ == "__main__":
    
    main()
    print("success")