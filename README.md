# sensor_degradation_filter

## Requirements

- [GSL 2.5+](https://www.gnu.org/software/gsl/) - _can be installed using `apt install libgsl-dev` or built from source_

## unorganized notes
- the static topo static deg (STSD) experiment is run by using the `kheperaiv_static_deg_controller` in the argos file
- in simulation, the assumed accuracy of a robot (if it's supposed to have a different one from its actual accuracy) is set in the `<loop_functions>` node
    - justification for doing this (as opposed to setting it in the controller):
        - we can't control --- at least not easily --- which robots are flawed and which aren't if we assign the values using the controller
        - if we want to use the same controller for real robots, we can technically cross-compile specific robots to have flawed assumed accuracies, but using loop functions is way more streamlined
    - the actual accuracy of the robot is set in the `<controllers>` node because it applies to all the robots. (maybe we want to consider having a swarm with heterogenous sensor accuracies?)
- in the loop functions 

## how to run experiments
### static-topo-static-deg
1. remove `-p` if you don't want parallel; see `param_multi_robot_sim_1d_static_degradation.yml` for the format of `$PARAM_FILE`.
    ```
    (.venv) $ run_static_degradation_experiment.py $PARAM_FILE -p
    ```


### dynamic-topo-static-deg
1. build the loop functions, controller and dependent libraries (should be achieved with a single cmake/make)
2. use the script `run_dynamic_topo_simulations` instead of `argos3`.
    ```
    # in the build_argos/src folder
    $ ./run_dynamic_topo_simulations -c $PARAM_FILE
    ```
    where `$PARAM_FILE` is the `.argos` file (see `param_multi_robot_sim_1d_static_degradation.argos` for the format)


## how to run experiments on hpc

### static-topo_static-deg
1. use the `hpc_multi_job_static_topo_static_deg_run.sh` to run. This will use the `sbatch_static_topo_static_deg_run.sh` script under the hood.

### dynamic-topo-dynamic-deg