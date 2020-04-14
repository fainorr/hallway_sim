
#!/usr/bin/python

# RANDOMLY GENERATE STRAIGHT HALLWAY IN CHUNKS

# to install lxml library, run in terminal:
# pip install lxml

from lxml import etree
from random import *
from numpy import *
from math import *

chunks = 10

avg_length = 5.0
std_length = 1.0
min_length = 0.5

avg_width = 4.0
std_width = 0.5
min_width = 2.0

wall_height = 3.0
wall_thickness = 0.2


# DEFINE HALLWAY
# -----------------

# randomly define hallway specs for each chunk
length = zeros(chunks)
width = zeros(chunks)

for i in range(0,chunks):
    length[i] = round(gauss(avg_length,std_length),2)
    if length[i] < min_length:
        length[i] = min_length

    width[i] = round(gauss(avg_width,std_width),2)
    if width[i] < min_width:
        width[i] = min_width


# define necessary walls for each hallway chunk
chunk_list = []
xyz_list = []
size_list = []
part_list = []

for i in range(0,chunks):
    pos_x = round((length[i]/2.0) + sum(length[0:i]),3)
    pos_y = width[i]/2.0 + wall_thickness/2.0
    pos_z = wall_height/2.0

    size_x = length[i]
    size_y = wall_thickness
    size_z = wall_height

    for link in range(0,4):
        if link == 0: # left wall
            link_xyz = '{} {} {}'.format(pos_x, pos_y, pos_z)
            link_size = '{} {} {}'.format(size_x, size_y, size_z)
            part = 'left_wall'
        elif link == 1: # right wall
            link_xyz = '{} {} {}'.format(pos_x, -pos_y, pos_z)
            link_size = '{} {} {}'.format(size_x, size_y, size_z)
            part = 'right_wall'
        elif link == 2: # left connector to next chunk
            if i+1 < chunks:
                link_xyz = '{} {} {}'.format(sum(length[0:i+1]), (width[i]+width[i+1])/4, pos_z)
                link_size = '{} {} {}'.format(wall_thickness, abs(width[i+1]-width[i])/2 + wall_thickness, size_z)
                part = 'left_conn'
            else:
                break
        elif link == 3: # right connector to next chunk
            if i+1 < chunks:
                link_xyz = '{} {} {}'.format(sum(length[0:i+1]), -(width[i]+width[i+1])/4, pos_z)
                link_size = '{} {} {}'.format(wall_thickness, abs(width[i+1]-width[i])/2 + wall_thickness, size_z)
                part = 'right_conn'
            else:
                break

        chunk_list.append(i+1)
        xyz_list.append(link_xyz)
        size_list.append(link_size)
        part_list.append(part)

# define base specs
base_xyz = '{} {} {}'.format(sum(length)/2, 0, -wall_thickness/2.0)
base_size = '{} {} {}'.format(sum(length), max(width)+2.0*wall_thickness, wall_thickness)


# WRITE XML FILE
# -----------------

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
mass.set("value","50")
origin = etree.SubElement(inertial,"origin")
origin.set("xyz","0 0 -1")
inertia = etree.SubElement(inertial,"inertia")
inertia.set("ixx","50.0")
inertia.set("ixy","0.0")
inertia.set("ixz","0.0")
inertia.set("iyy","50.0")
inertia.set("iyz","0.0")
inertia.set("izz","50.0")


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


etree.ElementTree(root).write("random_hall.urdf", pretty_print=True)
