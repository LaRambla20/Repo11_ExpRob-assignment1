# Repo11_ExpRob-assignment1
## SOFAR documentation
This github repository contains a ROS package and the corresponding modules documentation generated by the tool Sphinx starting from the comments in the code (link to visualize this documentation: https://larambla20.github.io/Repo11_ExpRob-assignment1/).
## Description
The first Experimental Robotics Laboratory assignment consists in simulating a robot deployed in a known-a-priori environment for surveillance purposes. It should interact with an ontology in order to identify locations that must be urgently visited and navigate to them. Locations' urgency is determined based on the difference between the last time that the robot has moved and the last time the location at issue has been visited. If no urgent locations are detected, the robot should prefer visiting corridors (room characterized by more than 1 connection with other locations), since starting from there it is easier for it to rapidly reach multiple locations. Once that a location has been reached, the robot should wait some seconds in order to simulate the room exploration.  
A battery management mechanism should be implemented as well. In particular whenever the battery level of the robot is low, the robot should navigate towards a specific location (charging room) and, once arrived, it should simulated the battery re-charging process for some seconds.
## Organization
In order to accomplish the above mentioned task, the ROS package named `exprob_first_assignment` contained in this repository has been implemented. In addition to that a folder named `docs` is present: it contains the nodes documentation generated by the tool `Sphinx` starting from the comments in the code.
The ROS package contains a launch file (`software_architecture.launch`) that launches the software architecture aimed at realising the desired robot behaviour. This architecture is composed of 4 python nodes, hereafter briefly described:
* `robot_states`: node that keeps track of the robot state (position and battery level). SPecifically, it defines two services to get and set the current robot pose, and a publisher to notify that the battery is low.
* `planner`: node that, given a desired location, retrieves the current robot position from the 'robot-states' node and returns a random plan as a set of via-points. In other words, it simulates the generation of a path towards a desired location.
* `controller`: node that, given a path as a set of via-points, simulates the movements to reach each via-point with a random delay. In addition to that, the node updates the current robot position stored in the 'robot-states' node every time that a via-point has supposedly been reached.
* `state_machine`: node that interacts with the other nodes of the software architecture in order to determine the desired behaviour of the robot. To this end, it implements a state machine composed of 6 states: BuildEnvironment, Reason, Charge, Navigate, NavigatetoCharge, Wait.
## How to run
Before running the notebook it is necessary to have both the simulation and the two external nodes up and running. To this end, follow the steps hereafter mentioned:
* open a terminal window and navigate to the `src` folder of your ROS workspace
* clone the repository https://github.com/LaRambla20/Repo08_RT2-assignment1_ROSpkg in the `src` folder of your ROS workspace
* build your ROS workspace with the command:
```bash
catkin_make
```
* from your ROS workspace folder, execute the following line to open the simulation environment:
```bash
roslaunch rt2_first_assignment final_assignment_sim.launch
```
* open another terminal window and navigate to your ROS workspace folder
* from your ROS workspace folder, execute the following line to run the two external nodes:
```bash
roslaunch rt2_first_assignment modalities_architecture.launch
```
As far as the notebook contained in this repository is concerned, the steps are instead the following:
* open a terminal window and navigate to a folder of your choosing
* clone this repository (https://github.com/LaRambla20/Repo07_RT2-assignment1) in the folder at issue
* from the folder, execute the following line to open Jupyter:
```bash
jupyter notebook --allow-root
```
* once Jupyter has been run in a browser window, click on the notebook named `rt2_assignment1` to open it
* run each block of the notebook
