# hallway_sim
During the 2019-2020 school year, the navigation sub-team of Lafayette's openDog senior design project created a **Gazebo simulation package** for developing and refining the navigation parameters and obstacle avoidance algorithm.  This repository contains all files related to generating a randomized hallway environment, launching a navigation simulation, controlling the robot, and evaluating the navigation results.  Included below are descriptions of each element.

**NOTE**: this package must be launched with the [**lidar_nav package**](https://github.com/fainorr/lidar_nav) also cloned in the catkin workspace (or in ROS development studio, the simulation_ws)!  This repository contains the URDF for the navigation robot with sensors.


## LAUNCH:
Any simulation in Gazebo requires a launch file, which creates an empty world, spawns URDF descriptions of robots or environments, and runs nodes to manipulate any ROS topics.

1. **hallway.launch**: this launch file runs a full hallway navigation simulation.  It generates a randomized hallway, spawns the robot, and controls it using the obstacle avoidance node **lidar_quad_node.py**.  All the results are echoed to three associated .txt files automatically created and stored in the eval directory for post-processing with **eval_sim.py**.

2. **room.launch**: this launch file spawns the basic four-sided **room.urdf** with the lidar_nav robot and three "people" who act with a random velocity.  The robot is controlled in the same manner as the hallway simulation, but the robot data is not echoed to a .txt file.

3. **elevator.launch**: this launch file loads the **elevator.urdf** and controls its door based on contact with the outer button.

4. **buttonpress.launch**: as part of the final integration step, this file inserts the **floatinghead** robot with arm attached and the elevator model to test the button press action.  The [**floatinghead repository**](https://github.com/dentinge/floatinghead_description), [**arm description**](https://github.com/MikeFischer1/arm_description), and [**depth camera library**](https://github.com/MikeFischer1/dcam_description) must all be cloned in the workspace.


## SCRIPTS:
This folder contains any necessary ROS nodes for running simulations, both in python (.py) or shell command (.sh) format.  It also contains the necessary functions that nodes rely on.

**robot control files**

1. **lidar_quad_node.py**: this is the primary ROS node for controlling the robot.  It subscribes to the LIDAR scan data and sends it to the **lidar_compare.py** python script.  This node is also where the navigation parameters are established.

2. **lidar_compare.py**: given the scan data, this script returns instructions for the robot as an "action" string and a "direction" string.  The "action" is either forward or turn and the "direction" is either left or right in terms of turning direction.  To determine the appropriate reaction, this script calls the methods outlined in **quad_analysis_methods.py**, which each process the scan in a unique way.  More details of each LIDAR analysis method with scripts to visualize and animate the results can be found in the [**opendog_IK repository**](https://github.com/fainorr/opendog_IK) in the LIDAR folder.

3. **gazebo_drive_node.py**: subscribing to the action and direction published by **lidar_quad_node.py**, this ROS node publishes linear and angular velocities to the robot in the /cmd_vel message.

4. **robot_sub_node.py**: as the simulation runs, this node subscribes to the robot x and y positions as well as its contact sensor and outputs the relevant data to a .txt file for post-processing.

5. **explore_FSM.py**: this node was constructed for the button press test.  It controls the robot by using the LIDAR to locate the elevator, and then it initiates the press action when the button is in range.


**person control files**

6. **person_control_node.py**: this node sends the linear and angular velocities for each "person" in the gazebo world; the number of publishers must match the number of people spawned.


**elevator control files**

7. **elevator_FSM.py**: this node is a finite state machine for the elevator, which opens and closes its door based on the state of the button contact sensor.  It publishes a desired door action to the **elevator_control.py** node.

8. **elevator_control.py**: with a requested action, this node finds the desired x, y, and z position of the door by calling the function outlined in **door_position.py** and publishes these values to the controller set up in the **elevator_control.yaml** file.


**shell nodes**

9. **gen_hallway.sh**: this is a shell node that runs the **prep_sim.py** function to generate a randomized hallway.

10. **gen_person.sh**: this is a shell node that runs the **person_gen.py** function to generate a new person model.

11. **timestamp.sh**: this shell script stores the start simulation time as a ROS parameter.  Any of the nodes that echo data to a .txt file uses this start_time in the name of the file to log simulations appropriately.


## EVAL:
After simulations are executed, any .txt files are saved in this folder for analysis using **eval_sim,py**.

1. **eval.py**: With all the files that end in "_hall.txt_" (each denoting one simulation), this script plots the specific hallway links with the robot path taken.  This plotted data can help to evaluate navigation success, and was specifically used to find the optimal navigation parameters for the simulated environment.  Running this script will generate a [**summary plot**](https://github.com/fainorr/hallway_sim/tree/master/images/simulation_plot.pdf) for each simulation set saved to the directory and save each plot as a pdf.


## URDF:
The environment description files (written in URDF format) and any python scripts used in generating the environment are included in this folder.

1. **prep_sim.py**: for each simulation, this script generates a [**randomized hallway environment**](https://github.com/fainorr/hallway_sim/tree/master/images/hallway.png) and stores its values in a .txt file for future reference in post-processing.  It calls upon **hallway_urdf_gen.py**, which specifies the environment and outputs the **random_hall.urdf**.

2. **hallway_urdf_gen.py**: the hallway itself is generated given specific hallway dimensions outlined in this script.  A hallway consists of straight hallway sections, each made up of a random number of “chunks."  Straight hallway sections intersect perpendicularly at intersection rooms to change the direction of the hallway.  The specific combination of these elements is outlined in the set_links function.  The elements themselves are actually developed in functions that the URDF generator calls:

    - **fxn_straight_hall.py**: given an orientation and the chunk specifications, this returns the links for a straight hallway.  This includes the walls of each hallway chunk and connectors to fill in gaps between chunks.

    - **fxn_hall_intersect.py**: this function returns the links for a room that changes the direction of the hallway, open on sides where it connects with a straight section and closed otherwise

3. **person_gen.py**: in the same way that the **hallway_urdf_gen.py** script creates a URDF for a randomized hallway, this script generates a randomized cylindrical "person" from a normal distribution and outputs its description to the **person.urdf** file.  To accomodate the possibility that a simulation might include more than one person, the URDF accepts an "index" specified in the launch file.

4. **elevator.urdf**: with a move-able door and buttons as contact sensors, this URDF specifies an elevator model for testing the integration of the arm with a moving robot.

5. **room.urdf**: this four-sided room URDF was created for initial navigation testing.
