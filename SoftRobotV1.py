# -*- coding: utf-8 -*-
from mecode import G
from math import sqrt

import numpy as np
import matplotlib.pyplot as plt

g = G(
    print_lines=False,
    outfile=r"H:\User Files\Fitzgerald\SoftRobots\Parameterized\gcode\parameterizedValves.pgm",
    aerotech_include=False,
)


# DEFAULT PRINTING PARAMETERS
default_line_pressure = 85
default_com_port = 9
default_start_stop_dwell_time = 0.2
default_travel_speed = 20
default_print_height_abs = 0
default_travel_height_abs = 2
default_print_speed = 1.5
default_z_axis = "z"

# Pressure control Macros
pressure_on = False
def turn_pressure_off(com_port = default_com_port, start_stop_dwell_time = default_start_stop_dwell_time):
    global pressure_on
    if (pressure_on):
        g.toggle_pressure(com_port)
        g.dwell(start_stop_dwell_time)
        pressure_on = False

def turn_pressure_on(com_port = default_com_port, start_stop_dwell_time = default_start_stop_dwell_time):
    global pressure_on
    if (not pressure_on):
        g.dwell(start_stop_dwell_time)
        g.toggle_pressure(com_port)
        pressure_on = True

# Printing Mode Macros
def move_z_abs(height, z_axis = default_z_axis):
    global g
    g.abs_move(**{z_axis:height})
    
def travel_mode(travel_speed = default_travel_speed, travel_height_abs = default_travel_height_abs):
    """"Stop Extrusion, move to travel height"""
    global g
    turn_pressure_off()
    g.feed(travel_speed)
    move_z_abs(travel_height_abs)

def travel_mode(travel_speed = default_travel_speed, travel_height_abs = default_travel_height_abs, whipe_distance=0, whipe_angle=0):
    """"Stop Extrusion, whipe, move to travel height, unwhipe"""
    global g
    turn_pressure_off()
    g.feed(travel_speed)
    move_x(whipe_distance,whipe_angle)
    move_z_abs(travel_height_abs)  
    move_x(whipe_distance,whipe_angle+np.pi)  
    
def print_mode(travel_speed = default_travel_speed, print_height_abs = default_print_height_abs, print_speed = default_print_speed):
    """Move to print height, start Extrusion"""
    g.feed(travel_speed)
    move_z_abs(print_height_abs)
    turn_pressure_on()
    g.feed(print_speed)
  
def move_x(distance, theta=0):
    global g
    if (distance!=0):
        g.move(x=np.cos(theta)*distance, y=np.sin(theta)*distance)        
     
def move_y(distance, theta=0):
    global g
    if (distance!=0):
        g.move(x=np.sin(theta)*distance, y=np.cos(theta)*distance)                     
 
def move_xy(x_distance, y_distance, theta=0):
    C=np.cos(theta)
    S=np.sin(theta)
    if (x_distance!=0 and y_distance!=0):
        g.move(x=x_distance*C-y_distance*S, y=x_distance*S+y_distance*C)
        
def print_actuator(pressure=85, com_port=9, theta=0, travel_speed = default_travel_speed, print_height_abs=default_print_height_abs):
    """Prints a soft actuator with the stem starting in the current position and rotated by theta. Assume nozzle is already at the correct height"""
    global g           
                    
    # stems
    stem_print_speed = 1.5
    stem_length =   6
    pad_separation_stem_length = 4
    
    #pad
    pad_length = 2.6 # length in Y
    pad_width = 2.6 # width in x
    n_meanders = 8
    pad_print_speed = 3.375
    meander_separation_dist = pad_length/n_meanders
    
    def print_actuator_pad():
        g.feed(pad_print_speed)
        move_x(-pad_width/2) #move to the lower left corner of the pad
        x_sign = 1
        for meander in range(n_meanders-1):
            move_y(meander_separation_dist)  # vertical down one meander width     
            move_x(x_sign *pad_width)        # horizontal across the whole pad
            x_sign = - x_sign
        move_y(meander_separation_dist)      # vertical down one meander width     
        move_x(x_sign*pad_width/2)           # move to the middle of the top of the pad
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    print_mode(print_height_abs = print_height_abs,print_speed = stem_print_speed)
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    #print stem to actuator pad 1
    move_y(stem_length, theta)
    
    #print actuator pad 1
    print_actuator_pad()
    
    #print connection stme to actuator pad 2
    g.feed(stem_print_speed)
    move_y(pad_separation_stem_length)
    
    #print actuator pad 2
    print_actuator_pad()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    travel_mode(whipe_angle = np.pi/2)

def print_robot():
    
    # mold parameters
    mold_z_zero_abs = 0     # absolute zero of the top of the mold
    mold_center_x = 53.5    # x coordinate of the center of the robot, relative to mold top left corner
    mold_front_actuator_y = - 13  # y cooredinate of the center of the front/foreward actuators, relative to mold top left corner
    mold_actuator_z_bottom_abs = mold_z_zero_abs - 1.5 # relative to top of mold
    mold_actuator_z_top = mold_z_zero_abs - 1 # relative top top of mold (expected)
    mold_depth = 7.62
    mold_body_width = 25.4
    
    # travel
    travel_height_abs = mold_z_zero_abs + 5
    
    # needle inlet connections in the abdomen
    inlet_length = 2
    inlet_print_speed = 0.5
    def print_inlet(height):
        print_mode(print_height_abs=height, print_speed = inlet_print_speed)
        g.move(y=inlet_length)
    
    #actuators 
    actuator_separation_y = 7 #distance between the "legs"
    n_actuator_rows = 4
    
    #make a arrays of Z interconnect points (locations of actuators)
    actuator_z_connect_inset = 5
    left_actuators_interconnects_x = mold_center_x - mold_body_width/2 + actuator_z_connect_inset
    right_actuators_interconnects_x = mold_center_x + mold_body_width/2 - actuator_z_connect_inset
    actuator_A_connection_points = []
    actuator_B_connection_points = []
    row_y = 0
    for actuator in range(n_actuator_rows/2):
        row_y = actuator*actuator_separation_y
        actuator_A_connection_points.append((left_actuators_interconnects_x, mold_front_actuator_y - row_y))
        actuator_A_connection_points.append((right_actuators_interconnects_x, mold_front_actuator_y -row_y - actuator_separation_y))
        actuator_B_connection_points.append((right_actuators_interconnects_x, mold_front_actuator_y - row_y - actuator_separation_y))
        actuator_B_connection_points.append((left_actuators_interconnects_x, mold_front_actuator_y - row_y))
 
    #find the centers of the actuator arrays
    channel_A_interconnect_center_y = sum([coordinate[1] for coordinate in actuator_A_connection_points])/n_actuator_rows
    channel_B_interconnect_center_y = sum([coordinate[1] for coordinate in actuator_B_connection_points])/n_actuator_rows
    
    # control lines
    control_line_A_height = mold_z_zero_abs-(1.0/3.0)*mold_depth
    control_line_B_height = mold_z_zero_abs-(2.0/3.0)*mold_depth
    controle_distribution_node_dwell_time = 2
    
    # pressure chambers
    pressure_chamber_length = 3
    pressure_chamber_print_speed = 0.25
    pressure_chamber_stem_length = 2
    pressure_chamber_print_height = mold_z_zero_abs - mold_depth/2.0
    pressure_chamber_y = mold_front_actuator_y - 4*actuator_separation_y - 4
    pressure_chamber_x_dist_from_center_line = (1.0/6.0)*mold_body_width
    pressure_chamber_A_x = mold_center_x - pressure_chamber_x_dist_from_center_line
    pressure_chamber_B_x = mold_center_x + pressure_chamber_x_dist_from_center_line
 
    ################ START PRINTING ################
    
    # set the current X and Y as the origin of the current work coordinates
    g.write("\nG92 X0 Y0 "+default_z_axis+"0 ; set the current position as the absolute work coordinate zero orgin\n")
    g.absolute()
    
    # print pressure chamber A (left)
    travel_mode(whipe_distance=0)
    g.abs_move(x=pressure_chamber_A_x, y = pressure_chamber_y-pressure_chamber_length-pressure_chamber_stem_length-inlet_length)
    print_inlet(pressure_chamber_print_height)
    
    # go to the Z interconnect hub for channel A
    travel_mode(whipe_angle = np.pi/2)
    g.abs_move(x=mold_center_x, y = channel_A_interconnect_center_y)
    print_mode(pressure_chamber_print_height)
    move_z_abs(control_line_A_height)
    g.dwell(controle_distribution_node_dwell_time)

#main program
print_robot()     
                   
g.view()
g.teardown()