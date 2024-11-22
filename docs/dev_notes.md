# Dev notes
- The dynamic topology static deg (DTSD) experiment is run by using the `kheperaiv_diffusion_motion_controller` and the `sensor_degradation_filter_loop_functions` libraries.
- In simulation, the assumed accuracy of a robot (if it's supposed to have a different one from its actual accuracy) is set in the `<loop_functions>` node. This is because:
    - we can't control --- at least not easily --- which robots are flawed and which aren't if we assign the values using the controller,
    - if we want to use the same controller for real robots, we can technically cross-compile specific robots to have flawed assumed accuracies, but using loop functions is way more streamlined
    - the actual accuracy of the robot is set in the `<controllers>` node because it applies to all the robots.

- The `StaticDegradationAlpha` class refers to the static degradation filter without adaptive activation. The results from using this filter is available but not included in the paper.
- The `StaticDegradationBravo` class refers to the ASDF filter; it's simply `StaticDegradationAlpha` with the activation mechanism. The paper "_Adaptive Self-Calibration for Minimalistic Collective Perception using Imperfect Robot Swarms_" discusses the proposal of this filter.
- The `DynamicDegradationCharlie` class refers to the dynamic degradation filter that does posterior approximation using pure vanilla VI. What this means is that the distribution family is fixed and we optimize the ELBO for the distribution parameters.