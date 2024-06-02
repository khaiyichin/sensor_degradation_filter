#!/bin/bash

# This script requires the `sbatch_static_topo_static_deg_run.sh` script to successfully execute static-topology-static-degradation experiments.
# It iterates over 3 parameters: filter period, comms period, and type 2 error probability (BRAVO filter), while
# the `sbatch_static_topo_static_deg_run.sh` script iterates over the different flawed_acc-correct_acc-TFR combinations
# within a single job.

# The script works by first modifying a template parameter file $TARGET_PARAM_FILE, then copying it to a target directory
# $TARGET_DIR before executing the `sbatch_static_topo_static_deg_run.sh` script in said directory. The copied
# parameter file serves as an intermediate template parameter file for `sbatch_static_topo_static_deg_run.sh` to modify
# and make copies.

NUM_TRIALS=30
NUM_STEPS=4.0e+4
METHOD="BRAVO"
CORRECT_FILTER=False
COMMS_PERIOD=(1 10 100)
FILTER_PERIOD=(1000 2000 4000)
TYPE2_ERR_PROB=(050 100 150 200 250)
TYPE2_ERR_PROB_DEC=(0.05 0.1 0.15 0.2 0.25)

module load slurm

# Copy param_multi_robot_sim_1d_static_degradation.yml to target directory
TOP_DIR=$(pwd)
TARGET_PARAM_FILE=param_multi_robot_sim_1d_static_degradation.yml
PYTHON_VENV_ACT_BIN=/home/kchin/sensor-degradation-filter/.venv/bin/activate

# Modify copied param file
sed -i "s/numTrials:.*/numTrials: ${NUM_TRIALS}/" ${TARGET_PARAM_FILE}
sed -i "s/method:.*/method: ${METHOD}/" ${TARGET_PARAM_FILE}
sed -i "s/correctFilter:.*/correctFilter: ${CORRECT_FILTER}/" ${TARGET_PARAM_FILE}
sed -i "s/numSteps:.*/numSteps: ${NUM_STEPS}/" ${TARGET_PARAM_FILE}

for (( i = 0; i < ${#COMMS_PERIOD[@]}; i++ ))
do
    for (( j = 0; j < ${#FILTER_PERIOD[@]}; j++ ))
    do
        for (( k = 0; k < ${#TYPE2_ERR_PROB[@]}; k++ ))
        do
            # Modify the parameters
            sed -i "s/commsPeriod:.*/commsPeriod: ${COMMS_PERIOD[i]}/" ${TARGET_PARAM_FILE}
            sed -i "s/sensorFilterPeriod:.*/sensorFilterPeriod: ${FILTER_PERIOD[j]}/" ${TARGET_PARAM_FILE}
            sed -i "s/  type2ErrProb:.*/  type2ErrProb: ${TYPE2_ERR_PROB_DEC[k]}/" ${TARGET_PARAM_FILE}

            # Copy param file
            TARGET_DIR=${TOP_DIR}/filtp${FILTER_PERIOD[j]}/commsp${COMMS_PERIOD[i]}/type2err${TYPE2_ERR_PROB[k]}
            mkdir -p ${TARGET_DIR}
	        cp ${TARGET_PARAM_FILE} ${TARGET_DIR}
	        cp sbatch_static_topo_static_deg_run.sh ${TARGET_DIR}
            pushd ${TARGET_DIR}
            mkdir -p data

            JOB_NAME=${METHOD}${CORRECT_FILTER}_filtp${FILTER_PERIOD[j]}_commsp${COMMS_PERIOD[i]}_type2err${TYPE2_ERR_PROB[k]}

            # Run the job
	        sbatch -N 1 -n 8 --mem=8G -p short -o "log_%x_%j.out" -e "log_%x_%j.err" -J ${JOB_NAME} -t 08:00:00 --mail-user=kchin@wpi.edu --mail-type=fail,end sbatch_static_topo_static_deg_run.sh
            popd
        done
    done
done