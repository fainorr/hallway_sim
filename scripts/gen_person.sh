#!/bin/bash

cd ~/simulation_ws/src/hallway_sim/urdf
python person_gen.py $1

echo generated person!
