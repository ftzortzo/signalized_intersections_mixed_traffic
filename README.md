# Safe and Efficient Coexistence of Autonomous Vehicles with Human-Driven Traffic at Signalized Intersections

 ## 1 Running the Simulation
After downloading the files, the main script to run is Main_Trafficlight.m. When you execute this file, the intersection and traffic simulation will appear.
The simulation uses a predefined set of initial conditions that are loaded automatically. If you prefer to use new, randomly generated initial conditions, run the Initialize_Cars.m function and make sure to comment out the line in Main_Trafficlight.m that loads the old initial conditions.

## 2 Switching to Adaptive Traffic Lights
By default, the simulation uses fixed-time traffic lights.
To switch to adaptive (dynamic) traffic lights as described in the paper:
Uncomment the call to Calculate_Pressure in the main script.
In the Calculate_Pressure.m file, set the desired cycle_time and minimum green time (min_duration) (the may have some predefined values).
Make sure that the sum of the values in modeDurations (defined in the main script) matches the chosen cycle_time.
Lastly, comment out the block of code in Find_green_times.m that starts right after line 49, under the comment:
_% If you put adaptive signals, remove the following code_

## Questions 
If you face any difficulties or you want to commnent something on the code, feel free to reach us at ft253@cornell.edu
