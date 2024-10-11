# Dev notes
- The dynamic topology static deg (DTSD) experiment is run by using the `kheperaiv_static_deg_controller` and the `sensor_degradation_filter_loop_functions` libraries.
- In simulation, the assumed accuracy of a robot (if it's supposed to have a different one from its actual accuracy) is set in the `<loop_functions>` node. This is because:
    - we can't control --- at least not easily --- which robots are flawed and which aren't if we assign the values using the controller,
    - if we want to use the same controller for real robots, we can technically cross-compile specific robots to have flawed assumed accuracies, but using loop functions is way more streamlined
    - the actual accuracy of the robot is set in the `<controllers>` node because it applies to all the robots.