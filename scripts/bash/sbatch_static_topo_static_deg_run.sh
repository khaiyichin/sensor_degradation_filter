#!/bin/bash

# This script is meant to be executed by a high-level script as an `sbatch` command, and should be copied by said script
# to the intended working directory before execution. It iterates different flawed_acc-correct_acc-TFR combinations
# within a single job run.

# The script works by first modifying a template parameter file $LOCAL_PARAM_FILE (which has been partially filled by
# the aforementioned high-level script), then making a copy of it with a specific name $NEW_PARAM_FILE. Finally,
# the simulation is run in parallel mode.

FLAWED=(550 750 950)
FLAWED_DEC=(0.55 0.75 0.95)
CORRECT=(550 750 950)
CORRECT_DEC=(0.55 0.75 0.95)
TFR=(550 750 950)
TFR_DEC=(0.55 0.75 0.95)

LOCAL_PARAM_FILE=param_multi_robot_sim_1d_static_degradation.yml
PYTHON_VENV_ACT_BIN=/home/kchin/sensor-degradation-filter/.venv/bin/activate

# Activate the virtual environment
source ${PYTHON_VENV_ACT_BIN}

# Run every experiment
for (( i = 0; i < ${#FLAWED[@]}; i++ ))
do
    for (( j = 0; j < ${#CORRECT[@]}; j++ ))
    do
        for (( k = 0; k < ${#TFR[@]}; k++ ))
        do
            if (( ${FLAWED[i]} != ${CORRECT[j]} ))
            then
            
            	# Modify param values
                sed -i "s/flawedSensorAccB:.*/flawedSensorAccB: ${FLAWED_DEC[i]}/" ${LOCAL_PARAM_FILE}
                sed -i "s/flawedSensorAccW:.*/flawedSensorAccW: ${FLAWED_DEC[i]}/" ${LOCAL_PARAM_FILE}
                sed -i "s/correctSensorAccB:.*/correctSensorAccB: ${CORRECT_DEC[j]}/" ${LOCAL_PARAM_FILE}
                sed -i "s/correctSensorAccW:.*/correctSensorAccW: ${CORRECT_DEC[j]}/" ${LOCAL_PARAM_FILE}
                sed -i "s/targFillRatio:.*/targFillRatio: ${TFR_DEC[k]}/" ${LOCAL_PARAM_FILE}
    
                NEW_PARAM_FILE=flawed${FLAWED[i]}_correct${CORRECT[j]}_tfr${TFR[k]}.yml

		        # Create copy of param file with updated values
                cp ${LOCAL_PARAM_FILE} ${NEW_PARAM_FILE}

                # Run the job
                run_static_degradation_experiment.py ${NEW_PARAM_FILE} -p
            fi
        done
    done
done