# Feedback Control Simulation
This library includes functions to simulate attitude control. 

## Running the Simulation
See [here](https://slugsat.github.io/Mechanical/Docs/html/matlab_sim.html)
1. Open **twovector_control_simulation.m**.
2. If desired, change the settings on lines 13-35.
3. Run the file.

## Important Files
Feedback control simulation:
- twovector_control_simulation.m: main file for the feedback control simulation.
- BdotControllerTest.m: main file for the b-dot controller simulation.
- BdotController_light.m: light version of BdotControllerTest (No quiver graphs)

Feedback control functions:
- LargeErrorController.m : Feedback control for large errors
- StabilizationController.m: Feedback controller for small errors
