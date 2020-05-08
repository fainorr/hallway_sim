#!/usr/bin/env python

from lxml import etree
from random import *
from numpy import *
from math import *
import sys

# to install lxml library, run in terminal:
# pip install lxml

# ------------------------
# RANDOMLY GENERATE PEOPLE
# ------------------------

# --- define person specs ---

height_avg = 1.72
height_std = 0.15


# store person index from launch file

try:
    index = sys.argv[1]
except:
    index = 1

# specify person dimensions based on height average and standard deviation

height = round(random.normal(height_avg, height_std),3)
body_height = round(6*height/7,3)
head_height = round(height/7,3)
body_radius = round(height/7,3)
head_radius = round(height/14,3)
body_mass = round(34.2*body_height,3)
head_mass = round(34.2*head_height,3)


# set vectors of link origins, sizes, and offsets [body, head]

names = ['body', 'head']
origins = ['0 0 0', '0 0 0']
radii = ['{}'.format(body_radius), '{}'.format(head_radius)]
lengths = ['{}'.format(body_height), '{}'.format(head_height)]
offsets = ['0 0 {}'.format(body_height/2), '0 0 {}'.format(body_height + head_height/2)]
masses = ['{}'.format(body_mass), '{}'.format(head_mass)]
inertias_xx = ['{}'.format(round((1/12.0)*body_mass*body_height**2,3)), '{}'.format(round((1/12)*head_mass*head_height**2,3))]
inertias_yy = ['{}'.format(round((1/12.0)*body_mass*body_height**2,3)), '{}'.format(round((1/12)*head_mass*head_height**2,3))]
inertias_zz = ['{}'.format(round((1/2.0)*body_mass*body_radius**2,3)), '{}'.format(round((1/2)*head_mass*head_radius**2,3))]


# ---------------
# WRITE URDF FILE
# ---------------

root = etree.Element('robot')
root.set("name","person")

# define materials
material = etree.SubElement(root,"material")
material.set("name",'red')
color = etree.SubElement(material,"color")
color.set("rgba",'0 0 0.8 1')

# create base_footprint link
link = etree.SubElement(root,"link")
link.set("name",'base_footprint')
visual = etree.SubElement(link,"visual")
origin = etree.SubElement(visual,"origin")
origin.set("xyz",'0 0 0')
origin.set("rpy",'0 0 0')
geometry = etree.SubElement(visual,"geometry")
box = etree.SubElement(geometry,"box")
box.set("size",'0.001 0.001 0.001')


# create links
for n in range(0,2):

    # define link
    link = etree.SubElement(root,"link")
    link.set("name", names[n])
    visual = etree.SubElement(link,"visual")

    origin = etree.SubElement(visual,"origin")
    origin.set("xyz",origins[n])
    origin.set("rpy",'0 0 0')

    geometry = etree.SubElement(visual,"geometry")
    cylinder = etree.SubElement(geometry,"cylinder")
    cylinder.set("radius",radii[n])
    cylinder.set("length",lengths[n])

    material = etree.SubElement(visual,"material")
    material.set("name", 'red')

    collision = etree.SubElement(link,"collision")
    origin = etree.SubElement(collision,"origin")
    origin.set("xyz",origins[n])
    origin.set("rpy",'0 0 0')

    geometry = etree.SubElement(collision,"geometry")
    cylinder = etree.SubElement(geometry,"cylinder")
    cylinder.set("radius",radii[n])
    cylinder.set("length",lengths[n])

    inertial = etree.SubElement(link,"inertial")
    mass = etree.SubElement(inertial,"mass")
    mass.set("value",masses[n])
    origin = etree.SubElement(inertial,"origin")
    origin.set("xyz",origins[n])
    inertia = etree.SubElement(inertial,"inertia")
    inertia.set("ixx",inertias_xx[n])
    inertia.set("ixy",'0.0')
    inertia.set("ixz",'0.0')
    inertia.set("iyy",inertias_yy[n])
    inertia.set("iyz",'0.0')
    inertia.set("izz",inertias_zz[n])

    # define joint
    joint = etree.SubElement(root,"joint")
    joint.set("name",'joint_'+names[n])
    joint.set("type",'fixed')

    parent = etree.SubElement(joint,"parent")
    parent.set("link",'base_footprint')

    child = etree.SubElement(joint,"child")
    child.set("link",names[n])

    origin = etree.SubElement(joint,"origin")
    origin.set("xyz",offsets[n])
    origin.set("rpy",'0 0 0')


# define the velocity controller plugin
gazebo = etree.SubElement(root,"gazebo")
plugin = etree.SubElement(gazebo,"plugin")
plugin.set("name",'object_controller')
plugin.set("filename",'libgazebo_ros_planar_move.so')

commandTopic = etree.SubElement(plugin,"commandTopic")
commandTopic.text = "vel_person"+str(index)

odometryTopic = etree.SubElement(plugin,"odometryTopic")
odometryTopic.text = "odom"

odometryFrame = etree.SubElement(plugin,"odometryFrame")
odometryFrame.text = "odom"

odometryRate = etree.SubElement(plugin,"odometryRate")
odometryRate.text = "20.0"

robotBaseFrame = etree.SubElement(plugin,"robotBaseFrame")
robotBaseFrame.text = 'base_footprint'

etree.ElementTree(root).write("person.urdf", pretty_print=True)
