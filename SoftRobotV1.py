# -*- coding: utf-8 -*-
from mecode import G
from math import sqrt

import numpy as np
import matplotlib.pyplot as plt

start_script_string = """
; Soft Robot V    1 G Code
; Daniel Fitzgerald
; 06-1-2014
; Soft robot with 4 control valves and inlets

DVAR $hFile        
DVAR $cCheck
DVAR $press
DVAR $length
DVAR $lame  

DVAR $COM
DVAR $pressure
DVAR $moveSpeed
DVAR $printSpeedXY
DVAR $printSpeedZ
DVAR $printHeight
DVAR $edgeLength
Primary
FILECLOSE

$COM=9
$pressure=85

; Below, we call several required GCode commands; these commands begin with a G:
G71            ; Standard GCode command for metric units
G76            ; Standard GCode command for time base seconds
G68            
G65 F3000        ; Sets an acceleration speed in mm/s^2
G66 F3000        ; Sets a deceleration speed in mm/s^2

; The “ENABLE” command allows us to enable axes on the printer; this is required for printing
ENABLE X Y A B C D U     ; enables all the axis 


Incremental                ; “Incremental” tells the printer to referring to relative coordinates

"""

end_script_string = """
VELOCITY OFF

M2

;----------------------------------------------------------
; END OF CODE - Function definitions below
;----------------------------------------------------------
DFS setPress        
         
	$strtask1 = DBLTOSTR( $P, 0 )            
	$strtask1 = "COM" + $strtask1
	$hFile = FILEOPEN $strtask1, 2
	COMMINIT $hFile, "baud=115200 parity=N data=8 stop=1"
	COMMSETTIMEOUT $hFile, -1, -1, 1000
                             
	$press = $Q * 10.0                             
	$strtask2 = DBLTOSTR( $press , 0 )  
      
      
	$length = STRLEN( $strtask2 )      
	WHILE $length < 4.0
		$strtask2 = "0" + $strtask2    
		$length = STRLEN( $strtask2 ) 
	ENDWHILE

	$strtask2 = "08PS  " + $strtask2
                                    
	$cCheck = 0.00     
	$lame = STRTOASCII ($strtask2, 0)
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 1) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 2) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 3) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 4)
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 5) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 6) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 7) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 8) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 9)  
	$cCheck = $cCheck - $lame
                        
	WHILE( $cCheck) < 0
		$cCheck = $cCheck + 256
	ENDWHILE                        

	$strtask3 = makestring "{#H}" $cCheck   
	$strtask3 = STRUPR( $strtask3 )
	$strtask2 = "\x02" + $strtask2 + $strtask3 + "\x03"
            
	FILEWRITE $hFile "\x05"
	FILEWRITE $hFile $strtask2
	FILEWRITE $hFile "\x04"

	FILECLOSE $hFile

ENDDFS

DFS togglePress        
         
	$strtask1 = DBLTOSTR( $P, 0 )            
	$strtask1 = "COM" + $strtask1
	$hFile = FILEOPEN $strtask1, 2
	COMMINIT $hFile, "baud=115200 parity=N data=8 stop=1"
	COMMSETTIMEOUT $hFile, -1, -1, 1000

	$strtask2 = "04DI  "
                                    
	$cCheck = 0.00     
	$lame = STRTOASCII ($strtask2, 0)
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 1) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 2) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 3) 
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 4)
	$cCheck = $cCheck - $lame
	$lame = STRTOASCII( $strtask2, 5) 
	$cCheck = $cCheck - $lame
                        
	WHILE( $cCheck) < 0
		$cCheck = $cCheck + 256
	ENDWHILE                        

	$strtask3 = makestring "{#H}" $cCheck   
	$strtask3 = STRUPR( $strtask3 )
	$strtask2 = "\x02" + $strtask2 + $strtask3 + "\x03"
                  
	FILEWRITE $hFile "\x05"
	FILEWRITE $hFile $strtask2
	FILEWRITE $hFile "\x04"

	FILECLOSE $hFile

ENDDFS  
;----------------------------------------------------------
;----------------------------------------------------------
;----------------------------------------------------------
"""



g = G(
    print_lines=False,
    outfile=r"H:\User Files\Fitzgerald\SoftRobots\SoftRobotV1\gcode\SoftRobotFourActuators.pgm",
    aerotech_include=False,
)


g.write(start_script_string)


# DEFAULT PRINTING PARAMETERS
default_line_pressure = 85
default_com_port = 9
default_start_stop_dwell_time = 0.2
default_travel_speed = 20
default_matrix_travel_speed = 1 # speed to travel in matrix (used for traveling vertically)
default_mold_top_abs = 0
default_travel_height_abs = default_mold_top_abs+2
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
def move_z_abs(height, z_axis = default_z_axis, vertical_travel_speed = default_matrix_travel_speed):
    global g
    cur_pos = g.get_current_position()
    prev_speed=default_travel_speed
    if (cur_pos):
        prev_speed = cur_pos[3]
        g.feed(vertical_travel_speed)
    if ((not cur_pos) or (cur_pos[2] != height)):
        g.abs_move(**{z_axis:height})
    g.feed(prev_speed)
    
#def travel_mode(travel_speed = default_travel_speed, travel_height_abs = default_travel_height_abs):
#    """"Stop Extrusion, move to travel height"""
#    global g
#    turn_pressure_off()
#    g.feed(travel_speed)
#    move_z_abs(travel_height_abs)

def travel_mode(travel_speed = default_travel_speed, travel_height_abs = default_travel_height_abs, whipe_distance=0, whipe_angle=0):
    """"Stop Extrusion, whipe, move to travel height, unwhipe"""
    global g
    turn_pressure_off()
    g.dwell(default_start_stop_dwell_time)
#    move_x(whipe_distance,whipe_angle)
    move_z_abs(travel_height_abs)  
    g.feed(travel_speed)
#    move_x(whipe_distance,whipe_angle+np.pi)  

    
def print_mode(print_height_abs, travel_speed = default_travel_speed, print_speed = default_print_speed, whipe_distance=0, whipe_angle=0):
    """Move to print height, start Extrusion"""
    g.feed(travel_speed)
#    move_x(whipe_distance,whipe_angle)
    
    #go down to to mold zero if neccisary
    if (g.get_current_position()[2]>default_mold_top_abs):
        move_z_abs(default_mold_top_abs, vertical_travel_speed=travel_speed)
        
    #go the rest of the way in the default_matrix_travel_speed
    move_z_abs(print_height_abs)
    
    #start extrusion
    turn_pressure_on()
    g.dwell(default_start_stop_dwell_time)
    
#    move_x(-whipe_distance,whipe_angle)

    #return with the print_speed being used
    g.feed(print_speed)
  
def move_x(distance, theta=0):
    global g
    if (distance!=0):
        g.move(x=np.cos(theta)*distance, y=np.sin(theta)*distance)        
     
def move_y(distance, theta=0):
    global g
    if (distance!=0):
        g.move(x=-np.sin(theta)*distance, y=np.cos(theta)*distance)                     
 
def move_xy(x_distance, y_distance, theta=0):
    C=np.cos(theta)
    S=np.sin(theta)
    if (x_distance!=0 and y_distance!=0):
        g.move(x=x_distance*C-y_distance*S, y=x_distance*S+y_distance*C)
        
def print_valve(print_height_abs, pressure=85,com_port=9, theta=0, stem_print_speed = default_print_speed, flow_connection_x = 3, control_connection_y = 5):
    """Prints a valve with the stem starting in the current position and rotated by theta. Assume nozzle is already at the correct height"""
    """Overall connection geometry is as follows: 
        back flow connector = (0,-3)
        front flow connector = (0,3)
        control line connector = (5,0)"""
    
    global g
    
    # stems
    stem_print_speed = 1.5
    inlet_print_speed = 0.5
    inlet_stem_length = 2
    inlet_needle_length = 3
    inlet_total_length = inlet_stem_length+inlet_needle_length
    def print_inlet():
        g.feed(stem_print_speed)
        move_y(inlet_stem_length, theta)
        g.feed(inlet_print_speed)
        move_y(inlet_needle_length, theta)
    
    #general
    pad_z_separation = 0.2+0.19
    junction_dist = 2
    
    #control pads
    control_pad_width = 2 # width in Y
    control_pad_meander_separation = 0.35
    control_pad_n_meanders = 5
    control_pad_length = control_pad_n_meanders*control_pad_meander_separation # length in X
    control_pad_print_speed = 1
    control_pad_stem_length = control_connection_y - 0.5*control_pad_length
        
    # flow pad
    flow_pad_n_meanders = 8
    flow_pad_meander_separation = 0.175
    flow_pad_length = flow_pad_meander_separation*flow_pad_n_meanders # length in Y
    flow_pad_width = 1.5 #width in X
    flow_stem_length = flow_connection_x-0.5*control_pad_width
    flow_pad_print_speed = 2

    #feeds
    stem_print_speed = 1.5
    #matrix_travel_speed = 2
    #pad_print_speed_1 = 1
    #pad_print_speed_2 = 4
     
    ##### START WRITING THE SCRIPT ####
    g.write("\n\n; PRINT A VALVE rotation = " + str(theta) + ".")      
    
    #assume we start above the center of the pad
    g.relative()
    
    #move from above the pad center to its lower corner
    travel_mode()
    move_xy(x_distance= -control_pad_length/2.0 , y_distance=-control_pad_width/2.0,theta=theta)
    
    #print the bottom control pad, starting at the edge of the pad
    print_mode(print_height_abs=print_height_abs, print_speed = control_pad_print_speed)
    y_sign = 1
    n_meanders = int(control_pad_length/control_pad_meander_separation)
    for meander in range(n_meanders):
        move_y(y_sign*control_pad_width, theta)
        move_x(control_pad_meander_separation, theta)
        y_sign = -1*y_sign
    move_y(y_sign*control_pad_width/2, theta)
    
    #print the stem from the bottom control pad
    g.feed(stem_print_speed)
    move_x(control_pad_stem_length, theta)
    
    #print the control stem inlet
    print_inlet()
    
    #travel to the back (-y) interconect point of flow lines
    travel_mode(whipe_angle=theta+np.pi/2)
    move_xy(x_distance = -control_pad_stem_length-control_pad_length/2.0, y_distance = -inlet_total_length - flow_stem_length - 0.5*control_pad_width, theta=theta)
    print_mode(print_height_abs=print_height_abs+pad_z_separation)
        
    #bottom flow line
    g.feed(stem_print_speed)
    move_y(flow_stem_length+0.5*(control_pad_width-flow_pad_length), theta)
    
    #flow pad
    n_meanders = int(flow_pad_length/flow_pad_meander_separation)
    x_sign = -1
    g.feed(flow_pad_print_speed)
    move_x(distance=(flow_pad_width/2.0), theta=theta)
    for meander in range(n_meanders-1):
        move_y(flow_pad_meander_separation,theta)
        move_x(x_sign * flow_pad_width,theta)
        x_sign = -1*x_sign
    move_y(flow_pad_meander_separation,theta)    
    move_x(x_sign*(flow_pad_width/2.0), theta)
    
    #top flow line
    g.feed(stem_print_speed)
    move_y(flow_stem_length+0.5*(control_pad_width-flow_pad_length), theta)
    
    #flow inlet
    print_inlet()
    
    #travel over control pad junction
    travel_mode(whipe_angle=theta+np.pi)
    move_xy(x_distance=junction_dist+0.5*control_pad_length, y_distance = -inlet_total_length -flow_stem_length-(control_pad_width-flow_pad_length)/2.0-flow_pad_length/2.0, theta=theta)
    
    #print stem up to control top pad
    print_mode(print_height_abs=print_height_abs)
    g.move(**{default_z_axis:2*pad_z_separation})
    move_x(-junction_dist, theta)
    # BUG TODO FIX ME: can;t move relative x y and z for connection from bottom stem to top control pad
    #move_xy(x_distance=-junction_dist, y_distance=0, z_distance=2*pad_z_separation, theta=theta)
#    move_x(-0.5*(control_pad_width-flow_pad_length), theta=theta)
    
    #print the top flow pad
    g.feed(control_pad_print_speed)
    y_sign = 1
    n_meanders = int(control_pad_length/control_pad_meander_separation)
    move_y(-control_pad_width/2.0, theta)
    move_x(-control_pad_meander_separation, theta)
    for meander in range(n_meanders-1):
        move_y(y_sign*control_pad_width, theta)
        move_x(-control_pad_meander_separation, theta)
        y_sign = -1*y_sign
    move_y(y_sign*control_pad_width, theta)
    travel_mode(whipe_angle=theta+1.0*np.pi)
            
def print_actuator(print_height_abs, pressure=85, com_port=9, theta=0, travel_speed = default_travel_speed):
    """Prints a soft actuator with the stem starting in the current position and rotated by theta. Assume nozzle is already at the correct height"""
    global g           
                    
    # stems
    stem_print_speed = 1.5
    stem_length =   10+4
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
    mold_front_leg_row_y = - 13  # y cooredinate of the center of the front/foreward actuators, relative to mold top left corner
    mold_back_leg_row_y = -34
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
    n_actuator_rows=4
    actuator_print_height_offset = 0.1 # nozzle height above ecoflex
    actuator_print_height = mold_actuator_z_top +  actuator_print_height_offset
    actuator_separation_y = 7 #distance between the "legs"    actuator_z_connect_inset = 5
    actuator_z_connect_inset = 5
    left_actuators_interconnects_x = mold_center_x - mold_body_width/2 + actuator_z_connect_inset
    right_actuators_interconnects_x = mold_center_x + mold_body_width/2 - actuator_z_connect_inset
    
    def print_right_actuator():
        print_actuator(theta = 1.5*np.pi, print_height_abs = actuator_print_height)
    
    def print_left_actuator():
        print_actuator(theta = 0.5*np.pi, print_height_abs = actuator_print_height)
 
    ## control lines
    control_line_height_abs = mold_z_zero_abs - mold_depth/2.0
    control_line_bridge_height_abs = control_line_height_abs + 2
    control_line_x_dist_from_center_line = (1.0/6.0)*mold_body_width
    control_line_A_x = mold_center_x - control_line_x_dist_from_center_line
    control_line_B_x = mold_center_x + control_line_x_dist_from_center_line
 
    ################ START PRINTING ################
    
    # set the current X and Y as the origin of the current work coordinates
    g.write("\nG92 X0 Y0 "+default_z_axis+"0 ; set the current position as the absolute work coordinate zero origin\n")
    travel_mode()
    g.abs_move(x=0,y=0,**{default_z_axis:0})
    g.absolute()
    
    ################ Valves ################
    abdomen_length = 41.5
    n_valves = 4
    abdomen_clearence = 4
    valve_separation_y = (abdomen_length-2*abdomen_clearence)/(n_valves+1.0)
    valve_flow_connection = 3
    valve_control_connection = 3
    valve_print_height = control_line_height_abs -0.39 #this is the pad_z_separation from the print_valve function
    valve_flow_height = valve_print_height + 0.39 
    valve_y_positions = [mold_head_y - mold_body_length + abdomen_clearence+(n_valves-n)*valve_separation_y for n in range(n_valves)]
    print "Valve Y Positions: " + str(valve_y_positions)
    valve_x = mold_center_x
    valve_angle = np.pi*0.5
    travel_mode(whipe_distance=0)
    for valve_y in valve_y_positions:
        g.abs_move(x=valve_x, y = valve_y)
        print_valve(flow_connection_x = valve_flow_connection, control_connection_y = valve_control_connection, print_height_abs=valve_print_height, theta=valve_angle)
    
    #connect the flow lines of the front two valves together, then to control line A
    g.abs_move(x=valve_x+valve_flow_connection, y=valve_y_positions[1])
    print_mode(print_height_abs = valve_flow_height)
    g.abs_move(y=valve_y_positions[0])
    g.dwell(default_start_stop_dwell_time)
    g.abs_move(y=valve_y_positions[0]+valve_control_connection+2)
    
    ################ Control Lines and Actuators ################
    
    #print control line A
#    travel_mode(whipe_distance=0)
    g.abs_move(x=control_line_A_x, y = mold_back_leg_row_y)
#    print_mode(print_height_abs = control_line_height_abs)
    g.abs_move(y = mold_front_leg_row_y)
            
    #print the top left actuator (A1) directly from the end of  control line A
    g.abs_move(x=left_actuators_interconnects_x)
    move_z_abs(actuator_print_height)
    print_left_actuator()
    
    # Connect the flow lines of the back two valves together, then to control line B
    g.abs_move(x=valve_x+valve_flow_connection, y=valve_y_positions[3])
    print_mode(print_height_abs = valve_flow_height)
    g.abs_move(y=valve_y_positions[2])
    g.dwell(default_start_stop_dwell_time)
    g.abs_move(x = valve_x+valve_flow_connection + 2, y = valve_y_positions[2]+2)
    g.abs_move(y=valve_y_positions[0]+valve_control_connection+2)
    
    #print control line B
    g.abs_move(x=control_line_B_x, y = mold_back_leg_row_y)
#    print_mode(print_height_abs = control_line_height_abs)
    g.abs_move(y = mold_front_leg_row_y)
    
    #print actuator B1 (top right) directly from end of control line B
    g.abs_move(x=right_actuators_interconnects_x)
    move_z_abs(actuator_print_height)
    print_right_actuator()
    
    def print_mode_control_line_B():
        print_mode(print_height_abs = control_line_height_abs, whipe_distance = 1, whipe_angle = np.pi)
    
    def print_mode_control_line_A():
        print_mode(print_height_abs = control_line_height_abs, whipe_distance = 1, whipe_angle = 0)
        
    #print actuator A3 (second from top, right) bridging over control line B
    g.abs_move(x=control_line_A_x, y=mold_front_leg_row_y - 1*actuator_separation_y)
    print_mode_control_line_A()
    move_z_abs(control_line_bridge_height_abs)
    g.abs_move(right_actuators_interconnects_x)
    print_right_actuator()
    
    #print actuator B3 (second from top, left) going around the control line of A3
    g.abs_move(x=control_line_B_x, y=mold_front_leg_row_y - 1.5*actuator_separation_y)
    print_mode_control_line_B()
    move_z_abs(control_line_bridge_height_abs)
    g.abs_move(x=left_actuators_interconnects_x)
    g.abs_move(y=mold_front_leg_row_y - 1.0*actuator_separation_y)
    print_left_actuator()
    
    ##print actuator A2 (second from bottom, left)
    g.abs_move(x=control_line_A_x, y = mold_front_leg_row_y - 2*actuator_separation_y)
    print_mode_control_line_A()
    g.abs_move(x=left_actuators_interconnects_x)
    print_left_actuator()
    
    #print actuator B2 (second from bottom, right)
    g.abs_move(x=control_line_B_x, y = mold_front_leg_row_y - 2*actuator_separation_y)
    print_mode_control_line_B()
    g.abs_move(x=right_actuators_interconnects_x)
    print_right_actuator()
    
    #print actuator B4 (bottom, left) bridging over control line A
    g.abs_move(x=control_line_B_x, y = mold_front_leg_row_y - 3*actuator_separation_y)
    print_mode_control_line_B()
    move_z_abs(control_line_bridge_height_abs)
    g.abs_move(x=left_actuators_interconnects_x)
    print_left_actuator()
        
    #print actuator A4 (bottom right) going around the control line of B4
    g.abs_move(x=control_line_A_x, y = mold_front_leg_row_y - 2.5*actuator_separation_y)
    print_mode_control_line_A()
    move_z_abs(control_line_bridge_height_abs)
    g.abs_move(x=right_actuators_interconnects_x)
    g.abs_move(y=mold_front_leg_row_y - 3*actuator_separation_y)
    print_right_actuator()

    #go back to home
    g.abs_move(x=0,y=0,**{default_z_axis:0})

#main program
print_robot()     

print (g.get_bounding_box())

g.write(end_script_string)                            
g.view()
g.teardown()