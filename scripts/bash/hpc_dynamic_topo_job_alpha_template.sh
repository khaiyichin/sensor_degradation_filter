#!/usr/bin/env bash

# Stop execution after any error
set -e

# Cleanup function to be executed upon exit, for any reason
function cleanup() {
    rm -rf $WORKDIR
}



########################################
#
# Useful variables
#
########################################

# Your user name
# (Don't change this)
MYUSER=$(whoami)

# Path of the local storage on a node
# Use this to avoid sending data streams over the network
# (Don't change this)
LOCALDIR=/local

# ARGoS environment variables
# (Don't change this)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/sensor-degradation-filter/build_argos/src
export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:$HOME/sensor-degradation-filter/build_argos/src
export PATH=$PATH:$HOME/sensor-degradation-filter/build_argos/src

# Folder where you want your data to be stored
# (Adapt this to your needs)
DATADIR=$1

# Path to the file param_multi_robot_sim_1d_dynamic_degradation.argos
# (Adapt this to your needs)
ARGOSFILE=~/sensor-degradation-filter/argos/param_multi_robot_sim_1d_dynamic_degradation.argos



########################################
#
# Job-related variables
#
########################################

# Parameters related to this job
CORRECT_FILTER=true
FLAWED_ROBOT_RANGE=(1 1 5) # array of 3 elements: (min increment max)
NUM_ROBOTS=20
NUM_TRIALS=30
NUM_TICKS=40000
TICKS_PER_SECOND=10
VERBOSITY=reduced
SPEED=14.14
OBS_PERIOD=1
FLAWED=(550 750 950)
CORRECT=(550 750 950)
TFR=(550 750 950)
FLAWED_DEC=(0.55 0.75 0.95)
CORRECT_DEC=(0.55 0.75 0.95)
TFR_DEC=(0.55 0.75 0.95)
NUM_TILES=1000
WALL_THICKNESS=0.1
ARENA_LEN=10

# Job id
# (Change this to reflect the above parameters)
if [[ "${CORRECT_FILTER}" == "true" ]]; then CORFILT=1; else CORFILT=0; fi
FLW_RANGE=${FLAWED_ROBOT_RANGE[0]}-${FLAWED_ROBOT_RANGE[1]}-${FLAWED_ROBOT_RANGE[2]}
THISJOB=dynamic_topo_static_deg_alpha_corfilt${CORFILT}_filtp${FILTER_PERIOD}

# Job working directory
# (Don't change this)
WORKDIR=${LOCALDIR}/${MYUSER}/${THISJOB}



########################################
#
# Job directory
#
########################################

# Create work dir from scratch, enter it
# (Don't change this)
rm -rf $WORKDIR && mkdir -p $WORKDIR && cd $WORKDIR

# Create data folder in work dir
mkdir data

# Make sure you cleanup upon exit
# (Don't change this)
trap cleanup EXIT SIGINT SIGTERM



########################################
#
# Actual job logic
#
########################################

# Configure experiment length
sed -i "s/<experiment.*/<experiment length=\"$(( ${NUM_TICKS} / ${TICKS_PER_SECOND} ))\" ticks_per_second=\"${TICKS_PER_SECOND}\" random_seed=\"0\" \/>/" ${ARGOSFILE}

# Configure communication period
sed -i "s/<comms period_ticks=.*/<comms period_ticks=\"${COMMS_PERIOD}\" \/>/" ${ARGOSFILE}

# Configure speed
sed -i "s/max_speed=.*/max_speed=\"${SPEED}\" \/>/" ${ARGOSFILE}

# Configure filter
sed -i "s/<static_degradation_filter.*/<static_degradation_filter sim=\"true\" method=\"ALPHA\" period_ticks=\"${FILTER_PERIOD}\" >/"

# Configure number of trials
sed -i "s/<num_trials.*/<num_trials value=\"${NUM_TRIALS}\" \/>/" ${ARGOSFILE}

# Configure number of arena tiles
sed -i "s/<arena_tiles.*/<arena_tiles tile_count_x=\"${NUM_TILES}\" tile_count_y=\"${NUM_TILES}\" \/>/" ${ARGOSFILE}

# Configure density
sed -i "s/<arena size.*/<arena size=\"${ARENA_LEN}, ${ARENA_LEN}, 1\" center=\"0,0,0.5\">/" ${ARGOSFILE} # arena size
sed -i "s/<box id=\"wall_north\".*/<box id=\"wall_north\" size=\"10,${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ARGOSFILE} # north wall size
sed -i "s/<box id=\"wall_south\".*/<box id=\"wall_south\" size=\"10,${WALL_THICKNESS},0.5\" movable=\"false\">/" ${ARGOSFILE} # south wall size
sed -i "s/<box id=\"wall_east\".*/<box id=\"wall_east\" size=\"${WALL_THICKNESS},10,0.5\" movable=\"false\">/" ${ARGOSFILE} # east wall size
sed -i "s/<box id=\"wall_west\".*/<box id=\"wall_west\" size=\"${WALL_THICKNESS},10,0.5\" movable=\"false\">/" ${ARGOSFILE} # west wall size

# Configure verbosity level
sed -i "s/<verbosity.*/<verbosity level=\"${VERBOSITY}\" \/>/" ${ARGOSFILE}

# Configure number of robots
sed -i "s/<entity.*/<entity quantity=\"${NUM_ROBOTS}\" max_trials=\"100\" base_num=\"0\">/" ${ARGOSFILE}

{
    START_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION BEGIN ###################################"
    echo -e "################################# ${START_TIME} #################################\n"

    # Iterate over flawed sensor accuracies
    for (( i = 0; i < ${#FLAWED[@]}; i++ ))
    do
        # Iterate over correct sensor accuracies
        for (( j = 0; j < ${#CORRECT[@]}; j++ ))
        do
            # Iterate over target fill ratios
            for (( k = 0; k < ${#TFR[@]}; k++ ))
            do
                # Configure target fill ratio
                sed -i "s/<target_fill_ratio.*/<target_fill_ratio value=\"${TFR_DEC[k]}\" \/>/" ${ARGOSFILE} # fill ratio range

                if (( ${FLAWED[i]} != ${CORRECT[j]} ))
                then
                    # Current date and time
                    CURR_DATETIME=$(date "+%m%d%Y_%H%M%S")

                    # Define JSON folder name for current iteration
                    FLWB=${FLAWED[i]}
                    FLWW=${FLAWED[i]}
                    CORB=${CORRECT[j]}
                    CORW=${CORRECT[j]}

                    JSON_FOLDER=${CURR_DATETIME}_t${NUM_TRIALS}_s${NUM_TICKS}_tfr${TFR[k]}_flw${FLW_RANGE}_flwb${FLWB}_flwc${FLWW}_corb${CORB}_corw${CORW}_commsp${COMMSP}_filtp${FILTP}_corfilt${CORFILT}

                    # Configure actual sensor accuracy
                    sed -i "s/<ground_sensor period_ticks=.*/<ground_sensor period_ticks=\"${OBS_PERIOD}\" sensor_acc_b=\"${CORRECT_DEC[j]}\" sensor_acc_w=\"${CORRECT_DEC[j]}\" \/>/" ${ARGOSFILE}

                    # Iterate over number of flawed robots
                    for l in $(seq ${FLAWED_ROBOT_RANGE[0]} ${FLAWED_ROBOT_RANGE[1]} ${FLAWED_ROBOT_RANGE[2]})
                    do
                        # Define configuration file name
                        PARAM_FILE=tfr${TFR[k]}_flw${l}_flwb${FLWB}_flwc${FLWW}_corb${CORB}_corw${CORW}.argos

                        # Configure output path
                        sed -i "s/<path folder=.*/<path folder=\"data/${JSON_FOLDER}\" \/>/" ${ARGOSFILE}

                        # Configure flawed sensor accuracy
                        sed -i "s/<flawed_robots.*/<flawed_robots num=\"${l}\" acc_b=\"${FLAWED_DEC[i]}\" acc_w=\"${FLAWED_DEC[i]}\" activate_filter_for_all=\"${CORRECT_FILTER}\" \/>/" ${ARGOSFILE}

                        # Create the modifed configuration file in the current folder
                        cp ${ARGOSFILE} ${PARAM_FILE}

                        # Run the experiment
                        run_dynamic_topo_simulations -c ${PARAM_FILE}
                    done
                fi
            done
        done
    done

    END_TIME=$(date +%m/%d/%Y-%H:%M:%S)
    echo -e "\n################################### EXECUTION END ###################################"
    echo -e "################################# ${END_TIME} #################################\n"
}

# Transfer everything into target data directory (including parameter files)
# (Adapt this to your data files)
cp -ar ./* $DATADIR