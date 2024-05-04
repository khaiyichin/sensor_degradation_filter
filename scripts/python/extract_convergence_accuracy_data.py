#!/usr/bin/env python3

import scripts.python.static_degradation_viz_module as sdvm
import os
import pandas as pd
import argparse
import warnings

"""Saving numpy arrays in HDF files will give a performance warning, so ignoring it"""
warnings.simplefilter(action='ignore', category=pd.errors.PerformanceWarning)

def load_data(folder_path):
    """Load the JSON data from a given folder.

    Args:
        data_folder: Path to the folder containing all the JSON data files. The files can be several levels deep.
    """
    json_data_lst = []

    # Receive folder path
    for root, _, files in os.walk(folder_path):

        if not files: continue # no files detected (perhaps only folders)
        else:
            json_files = [filename.endswith(".json") for filename in files]
            if any(json_files):
                print("Creating a StaticDegradationJsonData object for {0} JSON files in folder={1}...".format(len(json_files), root), end=" ")
                json_data_lst.append(sdvm.StaticDegradationJsonData(root))
                print("Done!\n")
            else:
                continue

    return json_data_lst

def save_to_h5(json_data_lst, convergence_threshold, output_filename, data_key="df"):
    df = pd.DataFrame()

    for d in json_data_lst:
        df = sdvm.process_convergence_accuracy(d, df, convergence_threshold)

    df.to_hdf(output_filename, key=data_key)

    print("Stored data as {0} with key \"{1}\"".format(os.path.join(os.getcwd(), output_filename), data_key))

def main(args):
    json_data_lst = load_data(args.FOLDER)
    save_to_h5(json_data_lst, args.THRESH, args.output, args.key)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Process convergence and accuracy data from simulated experiment data to save into HDF file. The stored data is a Pandas DataFrame.")
    parser.add_argument("FOLDER", type=str, help="path to the folder containing all the JSON data files; the files can be several levels deep")
    parser.add_argument("THRESH", type=float, help="threshold to assess convergence point")
    parser.add_argument("--output", default="conv_acc_data.h5", type=str, help="output filename (default: conv_acc_data.h5)")
    parser.add_argument("--key", default="df", help="dictionary key when storing the Pandas DataFrame")

    args = parser.parse_args()

    main(args)
