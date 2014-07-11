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
        move_x(-pad_width/2, theta) #move to the lower left corner of the pad
        x_sign = 1
        for meander in range(n_meanders-1):
            move_y(meander_separation_dist, theta)  # vertical down one meander width     
            move_x(x_sign *pad_width, theta)        # horizontal across the whole pad
            x_sign = - x_sign
        move_y(meander_separation_dist, theta)      # vertical down one meander width     
        move_x(x_sign*pad_width/2, theta)           # move to the middle of the top of the pad
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
    print_mode(print_height_abs = print_height_abs,print_speed = stem_print_speed)
    
    g.relative()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    #print stem to actuator pad 1
    move_y(stem_length, theta)
    
    #print actuator pad 1
    print_actuator_pad()
    
    #print connection stem to actuator pad 2
    g.feed(stem_print_speed)
    move_y(pad_separation_stem_length, theta)
    
    #print actuator pad 2
    print_actuator_pad()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
    travel_mode(whipe_angle = theta+np.pi/2)
    g.absolute()

def print_robot():
    
    # mold parameters
    mold_z_zero_abs = 0     # absolute zero of the top of the mold
    mold_center_x = 53.5    # x coordinate of the center of the robot, relative to mold top left corner
    mold_front_actuator_y = - 13  # y cooredinate of the center of the front/foreward actuators, relative to mold top left corner
    mold_actuator_z_bottom_abs = mold_z_zero_abs - 1.5 # relative to top of mold
    mold_actuator_z_top = mold_z_zero_abs - 1 # relative top top of mold (expected)
    mold_depth = 7.62
    mold_body_width = 25.4
    mold_body_length = 65.2
    mold_head_y = -4.9
    
    # travel
    travel_height_abs = mold_z_zero_abs + 5
    
    # needle inlet connections in the abdomen
    inlet_length = 2
    inlet_print_speed = 0.5
    inlet_distance_from_edge = 4 
    
    #actuators 
    actuator_print_height_offset = 0.1 # nozzle height above ecoflex
    actuator_print_height = mold_actuator_z_top +  actuator_print_height_offset
    actuator_separation_y = 7 #distance between the "legs"    actuator_z_connect_inset = 5
    actuator_z_connect_inset = 5
    left_actuators_interconnects_x = mold_center_x - mold_body_width/2 + actuator_z_connect_inset
    right_actuators_interconnects_x = mold_center_x + mold_body_width/2 - actuator_z_connect_inset
    
    def print_right_actuator():
        print_actuator(theta = 0.5*np.pi)
    
    def print_left_actuator():
        print_actuator(theta = 1.5*np.pi)
    
    #make a arrays of Z interconnect points (locations of actuators)
    n_actuator_rows = 4
    #actuator_A_connection_points = []
    #actuator_B_connection_points = []
    #row_y = 0
    #for actuator in range(n_actuator_rows/2):
    #    row_y = actuator*actuator_separation_y
    #    actuator_A_connection_points.append((left_actuators_interconnects_x, mold_front_actuator_y - row_y))
    #    actuator_A_connection_points.append((right_actuators_interconnects_x, mold_front_actuator_y -row_y - actuator_separation_y))
    #    actuator_B_connection_points.append((right_actuators_interconnects_x, mold_front_actuator_y - row_y - actuator_separation_y))
    #    actuator_B_connection_points.append((left_actuators_interconnects_x, mold_front_actuator_y - row_y))
         
    ## control lines
    control_line_height_abs = mold_z_zero_abs - mold_depth/2.0
    control_line_x_dist_from_center_line = (1.0/6.0)*mold_body_width
    control_line_A_x = mold_center_x - control_line_x_dist_from_center_line
    control_line_B_x = mold_center_x + control_line_x_dist_from_center_line
    #control_line_A_height = mold_z_zero_abs-(1.0/3.0)*mold_depth
    #control_line_B_height = mold_z_zero_abs-(2.0/3.0)*mold_depth
    #control_node_A = (mold_center_x, channel_A_interconnect_center_y, control_line_A_height)
    #control_node_B = (mold_center_x, channel_B_interconnect_center_y, control_line_B_height)
    #controle_distribution_node_dwell_time = 2
    
    ## pressure chambers
    #pressure_chamber_length = 3
    #pressure_chamber_print_speed = 0.25
    #pressure_chamber_stem_length = 2
    #pressure_chamber_height_z_abs = mold_z_zero_abs - mold_depth/2.0
    #pressure_chamber_y = mold_front_actuator_y - 4*actuator_separation_y - 4


 
    ################ START PRINTING ################
    
    # set the current X and Y as the origin of the current work coordinates
    g.write("\nG92 X0 Y0 "+default_z_axis+"0 ; set the current position as the absolute work coordinate zero origin\n")
    g.absolute()
    
    ################ Valves ################
    
    
    ################ Control Lines and Actuators ################
    
    #print control line A
    travel_mode(whipe_distance=0)
    g.abs_move(x=control_line_A_x, y = mold_front_actuator_y - n_actuator_rows*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    g.abs_move(y = mold_front_actuator_y)
            
    #print the top left actuator (A1) directly from the end of  control line A
    g.abs_move(x=left_actuators_interconnects_x)
    move_z_abs(actuator_print_height)
    print_left_actuator()
    
    #print control line B
    g.abs_move(x=control_line_B_x, y = mold_front_actuator_y - n_actuator_rows*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    g.abs_move(y = mold_front_actuator_y)
    
    #print actuator B1 (top right) directly from end of control line B
    g.abs_move(x=right_actuators_interconnects_x)
    move_z_abs(actuator_print_height)
    print_right_actuator()
    
    #print actuator A3 (second from top, right)
    g.abs_move(x=control_line_A_x, y=mold_front_actuator_y - 1*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    move_z_abs(actuator_print_height)
    g.abs_move(right_actuators_interconnects_x)
    print_right_actuator()
    
    #print actuator B3 (second from top, left)
    g.abs_move(x=control_line_B_x, y=mold_front_actuator_y - 1.5*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    move_z_abs(actuator_print_height)
    g.abs_move(x=left_actuators_interconnects_x)
    g.abs_move(y=mold_front_actuator_y - 1.0*actuator_separation_y)
    print_left_actuator()
    
    ##print actuator A2 (second from bottom, left)
    g.abs_move(x=control_line_A_x, y = mold_front_actuator_y - 2*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    g.abs_move(x=left_actuators_interconnects_x)
    print_left_actuator()
    
    #print actuator B2 (second from bottom, right)
    g.abs_move(x=control_line_B_x, y = mold_front_actuator_y - 2*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    g.abs_move(x=right_actuators_interconnects_x)
    print_right_actuator()
    
    #print actuator A4 (bottom right)
    g.abs_move(x=control_line_B_x, y = mold_front_actuator_y - 3*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    move_z_abs(actuator_print_height)
    g.abs_move(x=right_actuators_interconnects_x)
    print_right_actuator()
    
    #print actuator B4 (bottom, left)
    g.abs_move(x=control_line_B_x, y = mold_front_actuator_y - 2.5*actuator_separation_y)
    print_mode(print_height_abs = control_line_height_abs)
    move_z_abs(actuator_print_height)
    g.abs_move(x=left_actuators_interconnects_x)
    g.abs_move(y=mold_front_actuator_y - 3*actuator_separation_y)
    print_left_actuator()
        
    

#main program
print_robot()     
                   
g.view()
g.teardown()