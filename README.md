# hallway_sim
During the 2019-2020 school year, the navigation sub-team of Lafayette's openDog senior design project created a **Gazebo simulation package** for developing and refining the navigation parameters and obstacle avoidance algorithm.  This repository contains all files related to generating a randomized hallway environment, running a navigation simulation, and evaluating the simulation results.  Included below are descriptions of each element.

**NOTE**: this package must be launched with the [**lidar_nav**](https://github.com/fainorr/lidar_nav) package also cloned in the catkin workspace!  This repository contains the URDF for the navigation robot with sensors.

## URDF:
The environment description files (written in URDF format) and any python scripts used in generating the environment are included in this folder.

1. **prep_sim.py**: for each simulation, this script generates a [**randomized hallway environment**](https://github.com/fainorr/hallway_sim/tree/master/images/hallway.png) and stores its values in a txt file for future reference in post-processing.  It calls upon **hallway_urdf_gen.py**, which specifies the environment and outputs the **random_hall.urdf**.

2. **hallway_urdf_gen.py**: the hallway itself is generated given specific hallway dimensions outlined in this script.  A hallway consists of straight hallway sections, each made up of a random number of “chunks."  Straight hallway sections intersect perpendicularly at intersection rooms to change the direction of the hallway.  The specific combination of these elements is outlined in the set_links function.  The elements themselves are actually developed in functions that the URDF generator calls:

    - **fxn_straight_hall.py**: given an orientation and the chunk specifications, this returns the links for a straight hallway.  This includes the walls of each hallway chunk and connectors to fill in gaps between chunks.

    - **fxn_hall_intersect.py**: given the sides that should be open to a straight hallway, this returns the links for a room that changes the direction of the hallway.

3. **person_gen.py**: in the same way that the hallway_urdf_gen script creates a URDF for a randomized hallway, this script generates a randomized cylindrical "person" from a normal distribution and outputs its description to the **person.urdf** file.  To accomodate the possibility that a simulation might include more than one person, the urdf accepts an "index" specified in the launch file.

4. **elevator.urdf**: an elevator URDF

5. **room.urdf**: a room URDF
