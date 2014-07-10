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
default_travel_speed = 10
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
def abs_move_z(height, z_axis = default_z_axis):
    global g
    g.abs_move(**{z_axis:height})
    
def travel_mode(travel_speed = default_travel_speed, travel_height_abs = default_travel_height_abs):
    """"Stop Extrusion, move to travel height"""
    global g
    turn_pressure_off()
    g.feed(travel_speed)
    abs_move_z(travel_height_abs)

def travel_mode(travel_speed = default_travel_speed, travel_height_abs = default_travel_height_abs, whipe_angle=0):
    """"Stop Extrusion, whipe, move to travel height, unwhipe"""
    global g
    whipe_distance = 0.5
    turn_pressure_off()
    g.feed(travel_speed)
    move_x(whipe_distance,whipe_angle)
    abs_move_z(travel_height_abs)  
    move_x(whipe_distance,whipe_angle+np.pi)  
    
def print_mode(travel_speed = default_travel_speed, print_height_abs = default_print_height_abs, print_speed = default_print_speed):
    """Move to print height, start Extrusion"""
    g.feed(travel_speed)
    abs_move_z(print_height_abs)
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
        
def print_actuator(pressure=85, com_port=9, theta=0, print_height_abs=default_print_height_abs):
    """Prints a soft actuator with the stem starting in the current position and rotated by theta. Assume nozzle is already at the correct height"""
    global g           
                    
  # stem
  stem_print_speed = 1.5
                                                                                                                        
                                                                                                                                                                                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                      
def printValve(pressure=85,com_port=9, theta=0, stem_print_speed = default_print_speed, print_height_abs=default_print_height_abs):
    """Prints a valve with the stem starting in the current position and rotated by theta. Assume nozzle is already at the correct height"""
    global g
    
    # stems
    stem_print_speed = 1.5
    
    #general
    pad_z_separation = 0.2+0.19
    junction_dist = 4
    
    #control pads
    control_pad_stem_length = 8
    control_pad_width = 2
    control_pad_meander_separation = 0.35
    control_pad_length = 5*control_pad_meander_separation
    control_pad_print_speed = 1
    
    # flow pad
    flow_pad_length = 1.5
    flow_pad_width = 1.5
    flow_stem_length = 2
    flow_pad_print_speed = 2
    flow_pad_meander_separation = 0.175
    
    #feeds
    stem_print_speed = 1.5
    matrix_travel_speed = 2
    pad_print_speed_1 = 1
    pad_print_speed_2 = 4
     
    ##### START WRITING THE SCRIPT ####
    g.write("\n\n; PRINT A VALVE rotation = " + str(theta) + ".")      
    
    #assume we start above the center of the pad
    g.relative()
    
    #print the bottom pad and stem, starting at the edge of the pad
    print_mode(print_speed = control_pad_print_speed)
    y_sign = 1
    n_meanders = int(control_pad_length/control_pad_meander_separation)
    for meander in range(n_meanders):
        move_y(y_sign*control_pad_width, theta)
        move_x(control_pad_meander_separation, theta)
        y_sign = -1*y_sign
    move_y(y_sign*control_pad_width/2, theta)
    
    #print the stem from the bottom pad
    g.feed(stem_print_speed)
    move_x(control_pad_stem_length)
    
    #travel to bottom interconect of flow lines
    travel_mode(whipe_angle=theta+np.pi/2)
    move_xy(x_distance=-control_pad_stem_length-0.5*control_pad_length, y_distance=-flow_stem_length-0.5*control_pad_width)
    print_mode(print_height_abs=print_height_abs+pad_z_separation)
        
    #bottom flow line
    g.feed(stem_print_speed)
    move_y(flow_stem_length+0.5*(control_pad_width-flow_pad_length))
    #flow pad
    n_meanders = int(flow_pad_length/flow_pad_meander_separation)
    x_sign = -1
    g.feed(flow_pad_print_speed)
    move_x(distance=(0.5*flow_pad_length))
    for meander in range(n_meanders-1):
        move_y(flow_pad_meander_separation,theta)
        move_x(x_sign * flow_pad_width,theta)
        x_sign = -1*x_sign
    move_y(flow_pad_meander_separation,theta)    
    move_x(x_sign*flow_pad_width/2, theta)
    #top flow line
    g.feed(stem_print_speed)
    move_y(flow_stem_length+0.5*(control_pad_width-flow_pad_length))
    
    #travel over control pad junction
    travel_mode(whipe_angle=theta+np.pi)
    move_xy(x_distance=junction_dist+0.5*control_pad_length,y_distance=-0.5*flow_pad_length)
    
#main program
#printValve();        
print_actuator()                      

g.view()
g.teardown()