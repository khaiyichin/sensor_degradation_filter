# sensor_degradation_filter

## unorganized notes
- the static topo static deg (STSD) experiment is run by using the `kheperaiv_static_deg_controller` in the argos file
- in simulation, the assumed accuracy of a robot (if it's supposed to have a different one from its actual accuracy) is set in the `<loop_functions>` node
    - justification for doing this (as opposed to setting it in the controller):
        - we can't control --- at least not easily --- which robots are flawed and which aren't if we assign the values using the controller
        - if we want to use the same controller for real robots, we can technically cross-compile specific robots to have flawed assumed accuracies, but using loop functions is way more streamlined
    - the actual accuracy of the robot is set in the `<controllers>` node because it applies to all the robots. (maybe we want to consider having a swarm with heterogenous sensor accuracies?)
- in the loop functions 