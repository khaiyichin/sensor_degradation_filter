#!/usr/bin/env bash

# This script is modeled after the `sbatch_dynamic_topo_static_deg_run.sh` script and is meant to be run on a local workstation
# (not on an HPC). Here we run the two DELTA variants using the same random seed; each trial is run sequentially. The experiments
# are only run across one set of
#       `ground_sensor` parameters (`period_ticks`, `sensor_acc_*`, `true_deg_*_coeff`)
#       `sensor_degradation_filter` parameters (except `variant`, where both "bin" and "lap" are run)
#       `loop_functions` parameters (`num_trials`, `arena_tiles`, `target_fill_ratio`, `flawed_robots`, etc.)
#       swarm density (`arena` parameters)

# Stop execution after any error
set -e

# Trap errors and log the line number
trap "echo \"${LINENO}: ${BASH_COMMAND}\"; exit 1" ERR

########################################
#
# Useful variables
#
########################################

# ARGoS environment variables
# (Don't change this)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/research/sensor-degradation-filter/build_argos/src
export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:$HOME/research/sensor-degradation-filter/build_argos/src
export PATH=$PATH:$HOME/research/sensor-degradation-filter/build_argos/src

# Folder where you want your data to be stored
# (Adapt this to your needs)
DATADIR=$(pwd)

# Path to the LOCAL param_multi_robot_sim_1d_dynamic_degradation.argos (copied by the high-level script)
# (Adapt this to your needs)
TEMPLATE_ARGOSFILE=${DATADIR}/param_multi_robot_sim_1d_dynamic_degradation_DELTA.argos

########################################
#
# Job-related variables
#
########################################

# Parameters related to this job
# (To be changed by the high level script)
NUM_ROBOTS=15
CORRECT_FILTER=false            # whether non-flawed robots should run the filter (this only applies for *non-flawed* robots)
if [[ "${CORRECT_FILTER}" == "true" ]]; then CORFILT=1; else CORFILT=0; fi
NUM_FLAWED_ROBOTS=${NUM_ROBOTS} # set all robots to be flawed so that the filter will be run for all of them
DENSITY=0.1                     # desired swarm density
WALL_POSITION=7.64781           # computed from desired density
TRUE_DEG_DRIFT=-5e-5            # true sensor degradation drift coefficient (Wiener model)
TRUE_DEG_DIFFUSION=1e-3         # true sensor degradation drift coefficient (Wiener model)
METHOD=DELTA                    # filter type
OBS_Q_SIZE=200                  # observation queue size
PRED_DEG_MODEL_B=-5e-5          # state prediction model B
PRED_DEG_VAR_R=1e-6             # state prediction variance R
INIT_VAR=1e-2                   # initial variance for the filter (the initial mean will be the initial assumed sensor accuracy)

# Parameters related to this job
# (Fixed)
TRUE_ACC=(999 750)
TRUE_ACC_DEC=(0.999 0.75)
ASSUMED_ACC=(999 750) # when flawed robots num is set to 0 this isn't applied
ASSUMED_ACC_DEC=(0.999 0.75)
TFR=(550 950)
TFR_DEC=(0.55 0.95)
SEEDS=(
    290436
    181119
    38912
    154705
    155515
    608132
    132498
    866484
    266445
    189590
    770323
    395632
    353363
    276233
    416059
    568733
    409278
    671822
    625856
    469425
    576648
    91012
    326029
    810690
    553891
    45200
    905509
    540361
    345030
    452015
) # the randomly pre-generated list of seeds (so that all the experiments are the same across the two variants)
NUM_TICKS=20000
SPEED=14.14
COMMS_PERIOD=10
VERBOSITY=none
TICKS_PER_SECOND=10
NUM_TILES=130
WALL_THICKNESS=0.1
ARENA_LEN=16
DYNAMIC_DEGRADATION=true # dynamic sensor degradation
MEAS_PERIOD=10
FILTER_PERIOD=10
VARIANTS=("bin" "lap")
NUM_TRIALS=1                                            # must be 1 because the we want each trial to use a particular seed
UNPROCESSED_DATA_FILE="flw${NUM_FLAWED_ROBOTS}_t0.json" # used to search the output data file

########################################
#
# Actual job logic
#
########################################

ACTUAL_ARGOSFILE=temp.argos
cp ${TEMPLATE_ARGOSFILE} ${ACTUAL_ARGOSFILE}

# Configure ground sensor
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*period_ticks=\")[^\"]*/\1${MEAS_PERIOD}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*dynamic=\")[^\"]*/\1${DYNAMIC_DEGRADATION}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*true_deg_drift_coeff=\")[^\"]*/\1${TRUE_DEG_DRIFT}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*true_deg_diffusion_coeff=\")[^\"]*/\1${TRUE_DEG_DIFFUSION}/" ${ACTUAL_ARGOSFILE}

# Configure communication period
sed -i "s/<comms period_ticks=.*/<comms period_ticks=\"${COMMS_PERIOD}\" \/>/" ${ACTUAL_ARGOSFILE}

# Configure speed
sed -i -E "/<wheel_turning/,/\/>/ s/(max_speed=\")[^\"]*/\1${SPEED}/" ${ACTUAL_ARGOSFILE}

# Configure filter
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(method=\")[^\"]*/\1${METHOD}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(period_ticks=\")[^\"]*/\1${FILTER_PERIOD}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(observation_queue_size=\")[^\"]*/\1${OBS_Q_SIZE}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(pred_deg_model_B=\")[^\"]*/\1${PRED_DEG_MODEL_B}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(pred_deg_var_R=\")[^\"]*/\1${PRED_DEG_VAR_R}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(init_var=\")[^\"]*/\1${INIT_VAR}/" ${ACTUAL_ARGOSFILE}

# Configure loop functions
sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(num_trials value=\")[^\"]*/\1${NUM_TRIALS}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(arena_tiles tile_count_x=\")[^\"]*(\"[[:space:]]*tile_count_y=\")[^\"]*/\1${NUM_TILES}\2${NUM_TILES}/" ${ACTUAL_ARGOSFILE}
sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(verbosity level=\")[^\"]*/\1${VERBOSITY}/" ${ACTUAL_ARGOSFILE}

# Configure density (wall positions and robot starting locations)
sed -i "s/<arena size.*/<arena size=\"${ARENA_LEN}, ${ARENA_LEN}, 1\" center=\"0, 0, 0.5\">/" ${ACTUAL_ARGOSFILE} # arena size
sed -i "/<box id=\"wall_north\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                      # remove the line after "wall_north"
sed -i "/<box id=\"wall_south\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                      # remove the line after "wall_south"
sed -i "/<box id=\"wall_east\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                       # remove the line after "wall_east"
sed -i "/<box id=\"wall_west\".*/{n;d}" ${ACTUAL_ARGOSFILE}                                                       # remove the line after "wall_west"

sed -i "s/<box id=\"wall_north\".*/<box id=\"wall_north\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE} # north wall size
sed -i "s/<box id=\"wall_south\".*/<box id=\"wall_south\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE} # south wall size
sed -i "s/<box id=\"wall_east\".*/<box id=\"wall_east\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE}   # east wall size
sed -i "s/<box id=\"wall_west\".*/<box id=\"wall_west\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">/" ${ACTUAL_ARGOSFILE}   # west wall size

sed -i "s/<box id=\"wall_north\".*/<box id=\"wall_north\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">\n            <body position=\"0,${WALL_POSITION},0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}
sed -i "s/<box id=\"wall_south\".*/<box id=\"wall_south\" size=\"${ARENA_LEN},${WALL_THICKNESS},0.5\" movable=\"false\">\n            <body position=\"0,-${WALL_POSITION},0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}
sed -i "s/<box id=\"wall_east\".*/<box id=\"wall_east\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">\n            <body position=\"${WALL_POSITION},0,0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}
sed -i "s/<box id=\"wall_west\".*/<box id=\"wall_west\" size=\"${WALL_THICKNESS},${ARENA_LEN},0.5\" movable=\"false\">\n            <body position=\"-${WALL_POSITION},0,0\" orientation=\"0,0,0\" \/>/" ${ACTUAL_ARGOSFILE}

# Configure area of robot starting positions
sed -i "s/<position method=\"uniform\".*/<position method=\"uniform\" min=\"-${WALL_POSITION},-${WALL_POSITION},0\" max=\"${WALL_POSITION},${WALL_POSITION},0\" \/>/" ${ACTUAL_ARGOSFILE}

# Configure number of robots
sed -i "s/<entity.*/<entity quantity=\"${NUM_ROBOTS}\" max_trials=\"100\" base_num=\"0\">/" ${ACTUAL_ARGOSFILE}

{
    START_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION BEGIN ###################################"
    echo -e "################################# ${START_TIME} #################################\n"

    # Iterate over assumed sensor accuracies
    for ((i = 0; i < ${#ASSUMED_ACC[@]}; i++)); do

        # TODO: SED HERE
        sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(flawed_robots[[:space:]]*num=\")[^\"]*(\"[[:space:]]*acc_b=\")[^\"]*(\"[[:space:]]*acc_w=\")[^\"]*(\"[[:space:]]*activate_filter_for_all=\")[^\"]*/\1${NUM_FLAWED_ROBOTS}\2${ASSUMED_ACC_DEC[i]}\3${ASSUMED_ACC_DEC[i]}\4${CORRECT_FILTER}/" ${ACTUAL_ARGOSFILE}
        sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(init_mean=\")[^\"]*/\1${ASSUMED_ACC_DEC[i]}/" ${ACTUAL_ARGOSFILE}

        # Iterate over true sensor accuracies
        for ((j = 0; j < ${#TRUE_ACC[@]}; j++)); do
            sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*sensor_acc_b=\")[^\"]*/\1${TRUE_ACC_DEC[j]}/" ${ACTUAL_ARGOSFILE}
            sed -i -E "/<ground_sensor/,/\/>/ s/([[:space:]]*sensor_acc_w=\")[^\"]*/\1${TRUE_ACC_DEC[j]}/" ${ACTUAL_ARGOSFILE}

            # Iterate over target fill ratios
            for ((k = 0; k < ${#TFR[@]}; k++)); do
                sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(target_fill_ratio value=\")[^\"]*/\1${TFR_DEC[k]}/" ${ACTUAL_ARGOSFILE}

                # Current date and time
                CURR_DATETIME=$(date "+%m%d%y_%H%M%S")

                JSON_FOLDER=${CURR_DATETIME}_t${#SEEDS[@]}_s${NUM_TICKS}_tfr${TFR[k]}_flw${NUM_FLAWED_ROBOTS}_flwb${ASSUMED_ACC[i]}_corb${TRUE_ACC[j]}_drift${TRUE_DEG_DRIFT}_diff${TRUE_DEG_DIFFUSION}_modelb${PRED_DEG_MODEL_B}_modelr${PRED_DEG_VAR_R}_commsp${COMMSP}_filtp${FILTER_PERIOD}_corfilt${CORFILT}
                mkdir -p data/${JSON_FOLDER}

                # Configure output path
                sed -i -E "/<loop_functions/,/<\/loop_functions>/ s/(path folder=\")[^\"]*/\1data\/${JSON_FOLDER}/" ${ACTUAL_ARGOSFILE}

                # Define configuration file name
                PARAM_FILE=tfr${TFR[k]}_flw${NUM_FLAWED_ROBOTS}_flwb${ASSUMED_ACC[i]}_corb${TRUE_ACC[j]}.argos

                # Iterate over variants
                for ((l = 0; l < ${#VARIANTS[@]}; l++)); do
                    # Create directories for different variants
                    mkdir -p data/${JSON_FOLDER}/${VARIANTS[l]}_variant

                    # Modify variant type
                    sed -i -E "/<sensor_degradation_filter/,/<\/sensor_degradation_filter>/ s/(variant=\")[^\"]*/\1${VARIANTS[l]}/" ${ACTUAL_ARGOSFILE}

                    # Iterate over different seeds (each seed is for one trial)
                    for ((m = ${#SEEDS[@]} - 1; m >= 0; m--)); do
                        # Configure experiment length and seed
                        sed -i "s/<experiment.*/<experiment length=\"$((${NUM_TICKS} / ${TICKS_PER_SECOND}))\" ticks_per_second=\"${TICKS_PER_SECOND}\" random_seed=\"${SEEDS[m]}\" \/>/" ${ACTUAL_ARGOSFILE}

                        # Run the experiment
                        echo -n "Running trial index=${m} ($((${#SEEDS[@]}-${m}))/${#SEEDS[@]})... "
                        run_dynamic_topo_simulations -l /dev/null -c ${ACTUAL_ARGOSFILE}
                        echo "Done!"

                        # Modify the JSON data file so that `trial_ind` value reflects the correct trial index
                        sed -i -E "s/([[:space:]]*\"num_trials\"[[:space:]]*:[[:space:]]*)[0-9]+/\1${#SEEDS[@]}/" data/${JSON_FOLDER}/${UNPROCESSED_DATA_FILE}

                        # Rename and move the file
                        mv data/${JSON_FOLDER}/${UNPROCESSED_DATA_FILE} data/${JSON_FOLDER}/${VARIANTS[l]}_variant/flw${NUM_FLAWED_ROBOTS}_t${m}.json
                    done

                    echo "\n" # separate the stdout statements
                done

                # Create a copy of the modifed configuration file in the current folder
                cp ${ACTUAL_ARGOSFILE} data/${PARAM_FILE}
            done

        done

    done

    END_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION END ###################################"
    echo -e "################################# ${END_TIME} #################################\n"
}
