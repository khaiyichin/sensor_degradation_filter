#!/usr/bin/env bash

JOB_TYPE=DELTA_rmsd

DENSITY=(1)
LDAL=(500 700)
PRED_DEG_MODEL_B=(-10e-6 -25e-6 -40e-6)

module load slurm

EXECUTATBLE_SCRIPT=compute_rmsd_df.py

# Activate virtual environment
PYTHON_VENV_ACT_BIN=/home/kchin/sensor-degradation-filter/.venv/bin/activate
source ${PYTHON_VENV_ACT_BIN}

for ((i = 0; i < ${#DENSITY[@]}; i++)); do
    for ((j = 0; j < ${#LDAL[@]}; j++)); do
        for ((k = 0; k < ${#PRED_DEG_MODEL_B[@]}; k++)); do
            NEW_WORKING_DIR=den${DENSITY[i]}/ldal${LDAL[j]}/modelb${PRED_DEG_MODEL_B[k]}
            pushd ${NEW_WORKING_DIR}

            JOB_NAME=${JOB_TYPE}_den${DENSITY[i]}_ldal${LDAL[j]}_modelb${PRED_DEG_MODEL_B[k]}

            # Run the job
            # (The memory usage can go higher, but the --mem flag asks for minimum amount, and there's no upper limit)
            sbatch -N 1 -n 16 --mem=64G -p short -o "log_%x_%j.out" -e "log_%x_%j.err" -J ${JOB_NAME} -t 04:00:00 --mail-user=kchin@wpi.edu --mail-type=fail,end --wrap "${EXECUTATBLE_SCRIPT} . --output ${JOB_NAME}.h5 --verbose"
            popd
        done
    done
done
