#!/usr/bin/env python

from lxml import etree
from random import *
from numpy import *
from math import *
from fxn_straight_hall import *

# --------------------------------------------
# RANDOMLY GENERATE HALLWAY WITH INTERSECTIONS
# --------------------------------------------

# to install lxml library, run in terminal:
# pip install lxml


# --- first hallway, in x direction ---

n_chunks = 10
chunk_length = [5.0, 1.0, 0.5]
chunk_width = [4.0, 0.5, 2.0]
wall_specs = [3.0, 0.2]

chunk_list, xyz_list, size_list, part_list, hall_length, hall_max_width = \
    straight_hall(n_chunks, chunk_length, chunk_width, wall_specs)



# --- base specs ---

base_xyz = '{} {} {}'.format(hall_length/2, 0, -wall_specs[1]/2.0)
base_size = '{} {} {}'.format(hall_max_width, hall_max_width+2.0*wall_specs[1], wall_specs[1])
base_inertia = ["50,0", "0.0", "0.0", "50.0", "0.0", "50.0"] # [ixx, ixy, ixz, iyy, iyz ,izz]


# ---------------
# WRITE URDF FILE
# ---------------

root = etree.Element('robot')
root.set("name","hallway")

# create base layer
base = etree.SubElement(root,"link")
base.set("name","base")
for tag in range(0,2):
    if tag == 0:
        tag_parent = etree.SubElement(base,"visual")
    else:
        tag_parent = etree.SubElement(base,"collision")
    origin = etree.SubElement(tag_parent,"origin")
    origin.set("xyz",base_xyz)
    origin.set("rpy",'0 0 0')
    geometry = etree.SubElement(tag_parent,"geometry")
    box = etree.SubElement(geometry,"box")
    box.set("size",base_size)
inertial = etree.SubElement(base,"inertial")
mass = etree.SubElement(inertial,"mass")
mass.set("value",base_inertia[0])
origin = etree.SubElement(inertial,"origin")
origin.set("xyz",base_xyz)
inertia = etree.SubElement(inertial,"inertia")
inertia.set("ixx",base_inertia[0])
inertia.set("ixy",base_inertia[1])
inertia.set("ixz",base_inertia[2])
inertia.set("iyy",base_inertia[3])
inertia.set("iyz",base_inertia[4])
inertia.set("izz",base_inertia[5])


# create walls for each chunk
n_links = len(xyz_list)
for n in range(0,n_links):
    link = etree.SubElement(root,"link")
    link.set("name","sec_{}_".format(chunk_list[n]) + part_list[n])

    for tag in range(0,2):
        if tag == 0:
            tag_parent = etree.SubElement(link,"visual")
        else:
            tag_parent = etree.SubElement(link,"collision")

        origin = etree.SubElement(tag_parent,"origin")
        origin.set("xyz",xyz_list[n])
        origin.set("rpy",'0 0 0')

        geometry = etree.SubElement(tag_parent,"geometry")

        box = etree.SubElement(geometry,"box")
        box.set("size",size_list[n])

    # define joints
    joint = etree.SubElement(root,"joint")
    joint.set("name","joint_sec_{}_".format(chunk_list[n]) + part_list[n])
    joint.set("type","fixed")

    parent = etree.SubElement(joint,"parent")
    parent.set("link","base")

    child = etree.SubElement(joint,"child")
    child.set("link","sec_{}_".format(chunk_list[n]) + part_list[n])

    origin = etree.SubElement(joint,"origin")
    origin.set("xyz",'0 0 0')
    origin.set("rpy",'0 0 0')


# make hallway static
gazebo_link = etree.SubElement(root,"gazebo")
static = etree.SubElement(gazebo_link,"static")
static.text = "true"


etree.ElementTree(root).write("random_hall.urdf", pretty_print=True)
