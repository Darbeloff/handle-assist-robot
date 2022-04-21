#!/usr/bin/python
"""
Python script to control the ODrives and UR10 on the Handle Assist Robot
Roberto Bolli Jr.

Based on work from Daniel J. Gonzalez, Rachel Hoffman-Bice and Jerry Ng
for 2.12 Intro to Robotics
"""

import odrive
from odrive.enums import *
import time
import math
import fibre
import serial
import struct
import signal
import sys
import pdb
import pygame
import logging
import argparse
from interpreter.interpreter import InterpreterHelper
import socket
import random

#Useful constants
pi = 3.1415927
in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000
Nm2A = 0.00000604

class Odrive:
    def __init__(self, usb_serial, axes, kp, kd, full_init=False):
        self.usb_serials=usb_serial
        self.axes = axes
        self.CPR2RAD = (2*math.pi/400000)
        self.connect_odrive_serial()
        if full_init == True:
            self.full_init()
        self.set_gains(kp, kd)
        self.is_control_mode_initialized = False

    def connect_odrive_serial(self):
        #Connects to odrives of specified serial ids
        print("Finding odrive: ", self.usb_serials, "...")
        odrv = odrive.find_any(serial_number = self.usb_serials)
        print("Found odrive!")
        self.odrv=odrv
        self.odrv.clear_errors()

    #Error checking print functions
    def print_controllers(self):
        print(' axis 0 controller: ', self.odrv.axis0.controller)
        print(' axis 1 controller: ', self.odrv.axis1.controller)

    def print_encoders(self):
        print(' axis 0 controller: ', self.odrv.axis0.encoder)
        print(' axis 1 controller: ', self.odrv.axis1.encoder)

    def printErrorStates(self):
        print(' axis 0 error:',hex(self.odrv.axis0.error))
        print(' motor 0 error:',hex(self.odrv.axis0.motor.error))
        print(' encoder 0 error:',hex(self.odrv.axis0.encoder.error))
        print(' axis 1 error:',hex(self.odrv.axis1.error))
        print(' motor 1 error:',hex(self.odrv.axis1.motor.error))
        print(' encoder 1 error:',hex(self.odrv.axis1.encoder.error))

    def printPos(self):
        print(' pos_estimate: ', self.odrv.axis1.encoder.pos_estimate)
        print(' count_in_cpr: ',self.odrv.axis1.encoder.count_in_cpr)
        print(' shadow_count: ', self.odrv.axis1.encoder.shadow_count)

    def print_all(self):
        self.printErrorStates()
        self.print_encoders()
        self.print_controllers()

    def reboot(self):
        #Reboot and reconnect function
        self.odrv.reboot()
        time.sleep(5)
        connect_odrive_serial()
        print('Rebooted ')
        
    def pos_control(self):
        if self.is_control_mode_initialized == False:
            print('Changing mode to position control.')
            if self.axes[0]:
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
                self.odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            if self.axes[1]:
                self.odrv.axis1.requested_state = AXIS_STATE_IDLE
                self.odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1)
            
            self.is_control_mode_initialized = True


    def pos_move(self, desired_angle): #Desired position in turns/gear ratio (in our case, turns/100)
        self.pos_control()
        if self.axes[0]:
            try:
                self.odrv.axis0.controller.input_pos = desired_angle[0]
            except AttributeError:
                print('Failed to move axis 0')

        if self.axes[1]:
            try:
                self.odrv.axis1.controller.input_pos = desired_angle[1]
            except AttributeError:
                print('Failed to move axis 1')

    def vel_control(self):
        if self.is_control_mode_initialized == False:
            print('Changing mode to velocity control.')
            if self.axes[0]:
                self.odrv.axis0.requested_state = AXIS_STATE_IDLE
                self.odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                self.odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            if self.axes[1]:
                self.odrv.axis1.requested_state = AXIS_STATE_IDLE
                self.odrv.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                self.odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1)
        
            self.is_control_mode_initialized = True

    def vel_move(self, desired_vel): #Desired velocity in turns/gear ratio
        self.vel_control()
        if self.axes[0]:
            try:
                self.odrv.axis0.controller.input_vel = desired_vel[0]
            except AttributeError:
                print('Failed to move axis 0')

        if self.axes[1]:
            try:
                self.odrv.axis1.controller.input_vel = desired_vel[1]
            except AttributeError:
                print('Failed to move axis 1')

    def set_idle(self, axes):
        if self.axes[0]:
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE
        if self.axes[1]:
            self.odrv.axis1.requested_state = AXIS_STATE_IDLE


    def erase_and_reboot(self):
        #Erase the configuration of the system and reboots
        print('erasing config')
        self.odrv.erase_configuration()
        print('reboot')
        self.odrv.reboot()


    def startup_init(self):
        print('Initializing encoder calibration sequence')

        if self.axes[0]:
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            self.odrv.axis0requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            self.odrv.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(10)
            self.odrv.axis0.requested_state = AXIS_STATE_IDLE
            time.sleep(1)

        if self.axes[1]:
            self.odrv.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(1)
            self.odrv.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            time.sleep(10)
            self.odrv.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            time.sleep(10)
            self.odrv.axis1.requested_state = AXIS_STATE_IDLE
            time.sleep(1)

    def full_init(self):
        self.printErrorStates()
        if self.axes[0]:
            self.odrv.axis0.motor.config.pre_calibrated = False
            #pole pairs
            self.odrv.axis0.motor.config.pole_pairs = 4
            self.odrv.axis0.controller.config.vel_limit = 50 
            self.odrv.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            self.odrv.axis0.encoder.config.cpr = 4000
            self.odrv.axis0.encoder.config.use_index = True
            self.odrv.axis0.encoder.config.pre_calibrated = False
            #motor calibration current
            self.odrv.axis0.motor.config.calibration_current = 4

            time.sleep(1)
            self.printErrorStates()
            self.odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print('Axis 0 calibration sequence in progress')
            time.sleep(20)
            self.printErrorStates()
            self.odrv.axis0.motor.config.pre_calibrated=True
            self.odrv.axis0.config.startup_encoder_index_search = True
            self.odrv.axis0.config.startup_encoder_offset_calibration = True
            self.odrv.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        if self.axes[1]:
            self.odrv.axis1.motor.config.pre_calibrated = False
            #pole pairs
            self.odrv.axis1.motor.config.pole_pairs = 4
            self.odrv.axis1.controller.config.vel_limit = 50
            self.odrv.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            self.odrv.axis1.encoder.config.cpr = 4000
            self.odrv.axis1.encoder.config.use_index = True
            self.odrv.axis1.encoder.config.pre_calibrated = False
            #motor calibration current
            self.odrv.axis1.motor.config.calibration_current = 4

            time.sleep(1)
            self.printErrorStates()
            self.odrv.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            print('Axis 1 calibration sequence in progress')
            time.sleep(20)
            self.printErrorStates()
            self.odrv.axis1.motor.config.pre_calibrated=True
            self.odrv.axis1.config.startup_encoder_index_search = True
            self.odrv.axis1.config.startup_encoder_offset_calibration = True
            self.odrv.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        #self.odrv.save_configuration()
        time.sleep(2)
        print('Finished full calibration')
        self.printErrorStates()


    def set_gains(self, kp, kd, ki = [0, 0], make_permanent = 0):
        if self.axes[0]:
            self.odrv.axis0.requested_state=AXIS_STATE_IDLE
            self.odrv.axis0.controller.config.pos_gain = kp[0]
            self.odrv.axis0.controller.config.vel_gain = kd[0]
            self.odrv.axis0.controller.config.vel_integrator_gain = ki[0]

        if self.axes[1]:
            self.odrv.axis1.requested_state=AXIS_STATE_IDLE
            self.odrv.axis1.controller.config.pos_gain = kp[1]
            self.odrv.axis1.controller.config.vel_gain = kd[1]
            self.odrv.axis1.controller.config.vel_integrator_gain = ki[1]

        time.sleep(1)
        if make_permanent==1:
            self.odrv.save_configuration()

def parseArgs():
    parser = argparse.ArgumentParser(description = 'Send Interpreter commands from file')
    parser.add_argument('-ip', '--ip', help='Specify the IP of the robot (required)')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')
    parser.add_argument('-d', '--debug', help='print debug level messages', action='store_true')
    args = parser.parse_args()

    if args.ip is None:
        args.ip = "192.168.1.102"
        print('Using default robot IP address', args.ip)

    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    elif args.verbose:
        logging.basicConfig(level=logging.INFO)

    return args

def get_reply(socket):
        """
        read one line from the socket
        :return: text until new line
        """
        collected = b''
        while True:
            part = socket.recv(1)
            if part != b"\n":
                collected += part
            elif part == b"\n":
                break
        return collected.decode("utf-8")
    
            
if __name__ == '__main__':

    odrives = True
    UR10e = False
    
    # Connect to the joysticks
    pygame.init()
    pygame.joystick.init()
    drive_joystick = pygame.joystick.Joystick(0)
    arm_joystick = pygame.joystick.Joystick(1)
    drive_joystick.init()
    arm_joystick.init()

    if odrives:
        board_0_driver = Odrive(usb_serial = '2061377C3548', axes = [True, True],
                                kp = [2, 2], kd = [0.02, 0.02], full_init = False)
        board_1_driver = Odrive(usb_serial = '204A3880304E', axes = [True, True],
                                kp = [2, 2], kd = [0.02, 0.02], full_init = False)
        print('The motor driver has completed connection and calibration. It is ready to run.')
        print('Board 0 error states')
        board_0_driver.printErrorStates()
        print('Board 1 error states')
        board_1_driver.printErrorStates()

    if UR10e:
        UR10e_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        num_msgs = 3
        default_joint_pose = [-107/180*pi, -68/180*pi, -111/180*pi, -180/180*pi, -60/180*pi, 89.5/180*pi]
        new_joint_pose = default_joint_pose.copy()
        a1 = 24*in2m
        a2 = 26*in2m
        x_def = -a1*math.cos(new_joint_pose[1]) - a2*math.cos(new_joint_pose[1] + new_joint_pose[2])
        y_def = -a1*math.sin(new_joint_pose[1]) - a2*math.sin(new_joint_pose[1] + new_joint_pose[2])
        x_cur = x_def #In and out motion from base
        y_cur = y_def #Height off of the ground
        x_prev = x_cur
        
        args = parseArgs()
        interpreter = InterpreterHelper(args.ip)
        interpreter.connect()
        interpreter.execute_command("clear_interpreter()")

        # Set up a port for the UR10 to send back messages
        port = random.randint(4000,6000)
        conn = []
        interpreter.execute_command("socket_open(\"192.168.1.101\"," + str(port) + ", \"socket_to_PC\")")
        try:
            UR10e_socket.bind(('192.168.1.101', port))
            UR10e_socket.listen(9)
            conn, address = UR10e_socket.accept()
            print('Connected to UR10e:', address[0], address[1])
        except socket.error:
            print('Could not read data from UR10e')

        #Do not touch the handlebar when zeroing the force/torque sensor
        interpreter.execute_command("zero_ftsensor()")
        time.sleep(0.1)
        interpreter.execute_command("movej(" + str(default_joint_pose) + ", a=0.5, v=1, t=0, r=0)")
        pygame.event.get()
        print('Move the right joystick to continue')
        while (arm_joystick.get_axis(1) == 0 and arm_joystick.get_axis(1) == 0):
            pygame.event.get()
            time.sleep(0.05)
        print('Joystick controlled mode')
        
    prev_timestamp = time.time()

    # Main command loop for joystick control
    while True:
        
        delta_t = time.time() - prev_timestamp
        prev_timestamp = time.time()
        if (delta_t == 0): delta_t = 0.00001
        time.sleep(0.017) #Yields a refresh rate of approximately 50 Hz
        #print("Loop hz:", 1/delta_t)
        
        pygame.event.get()
        
        if odrives:
            axis0 = drive_joystick.get_axis(0)
            axis1 = drive_joystick.get_axis(1)
            axis2 = drive_joystick.get_axis(2)
            if (-0.15 < axis2 < 0.15): axis2 = 0
            #print('Drive axes:', axis0, axis1, axis2)
            
            board_0_velcmd = [-axis2*30 - axis1*100 - axis0*100, -axis2*30 - axis1*100 + axis0*100]
            board_1_velcmd = [-axis2*30 + axis1*100 + axis0*100, -axis2*30 + axis1*100 - axis0*100]
            board_0_driver.vel_move(board_0_velcmd)
            board_1_driver.vel_move(board_1_velcmd)
            #print(board_1_velcmd)

        if UR10e:
            arm_x = -arm_joystick.get_axis(1)
            if (-0.25 < arm_x < 0.25): arm_x = 0
            arm_y = -arm_joystick.get_axis(2)
            arm_rot = arm_joystick.get_axis(0)
            if (-0.25 < arm_rot < 0.25): arm_rot = 0
            #print('Arm axes:', arm_x, '\t', arm_y, '\t', arm_rot)
            
            handle_rot = 0
            if (arm_joystick.get_button(8) == 1):
                handle_rot = 0.04
            elif (arm_joystick.get_button(7) == 1):
                handle_rot = -0.04

            if (arm_joystick.get_button(2) == 1): # Button 3 pressed (read force values mode)
                arm_x = 0
                arm_rot = 0
            
            x_cur += arm_x/90
            y_cur = y_def + arm_y/3
            if (x_cur > x_def + 0.4):
                x_cur = x_def + 0.4
            elif (x_cur < x_def - 0.3):
                x_cur = x_def - 0.3
            j2 = -math.acos((x_cur**2 + y_cur**2 - a1**2  - a2**2)/(2*a1*a2))
            j1 = -pi + math.atan2(y_cur,x_cur) - math.atan2(a2*math.sin(j2), (a1 + a2*math.cos(j2)))
            new_joint_pose[0] += ( -(-1.82*(x_cur-x_def) + 1.45)*(x_cur - x_prev) - arm_rot/40) #base
            new_joint_pose[1] = j1 #1st joint
            new_joint_pose[2] = j2 #2nd joint
            new_joint_pose[3] = default_joint_pose[3] - (j2 + 1.94) - (j1 + 1.18)
            new_joint_pose[4] += (-(-1.82*(x_cur-x_def) + 1.45)*(x_cur - x_prev) + handle_rot)
            x_prev = x_cur

            # Constrain the base rotation
            if (new_joint_pose[0] < default_joint_pose[0] - 100/180*pi):
                new_joint_pose[0] = default_joint_pose[0] - 100/180*pi
            elif (new_joint_pose[0] > default_joint_pose[0] + 100/180*pi):
                new_joint_pose[0] = default_joint_pose[0] + 100/180*pi
            # Constrain the handle rotation
            if (new_joint_pose[4] < -165/180*pi):
                new_joint_pose[4] = -165/180*pi
            elif (new_joint_pose[4] > 45/180*pi):
                new_joint_pose[4] = 45/180*pi

            #UR10e proportional control
            move_cmd = "servoj(" + str(new_joint_pose) + ", 0, 0, 0.1, 0.2, 100)"
            #move_cmd = "movej(" + str(new_joint_pose) + ", a=2, v=4, t=0, r=0)"
            
            if num_msgs > 490:
                interpreter.execute_command("skipbuffer")
                interpreter.execute_command("clear_interpreter()")
                num_msgs = 0
                #print('Clearing interpreter')

            if (arm_joystick.get_button(2) == 1):  # Button 3
                interpreter.execute_command("socket_send_line(get_tcp_force(),\"socket_to_PC\")")
                num_msgs += 1
                wrench_str = get_reply(conn)
                wrench_str = wrench_str[2:len(wrench_str)-1]
                wrench_list = wrench_str.split(',')
                for i in range(len(wrench_list)):
                    wrench_list[i] = float(wrench_list[i])
                net_handle_force = round(math.sqrt(wrench_list[0]**2 + wrench_list[1]**2 + wrench_list[2]**2)*100)/100.0
                print('Net force (N):', net_handle_force)
            else:
                if (num_msgs != 0):
                    interpreter.execute_command("skipbuffer")
                    num_msgs += 1
                interpreter.execute_command(move_cmd)
                num_msgs += 1

        if (arm_joystick.get_button(0) == 1):
            if UR10e == True:
                print("Switching to freedrive mode")
                interpreter.execute_command("freedrive_mode (freeAxes=[1, 1, 1, 1, 1, 1], feature=p[0, 0, 0, 0, 0, 0])")
                UR10e = False
            else:
                print('Switching back to joystick-controlled mode')
                interpreter.execute_command("end_freedrive_mode()")
                interpreter.execute_command("movej(" + str(new_joint_pose) + ", a=0.5, v=1, t=0, r=0)")                
                UR10e = True
                
            while (arm_joystick.get_button(0) == 1):
                pygame.event.get()
                time.sleep(0.1)
        
        if (drive_joystick.get_button(0) == 1):  # Trigger button
            if odrives == True:
                board_0_driver.set_idle([True, True])
                board_1_driver.set_idle([True, True])
                board_0_driver.is_control_mode_initialized = False
                board_1_driver.is_control_mode_initialized = False
                odrives = False
            else:
                odrives = True
                
            while (drive_joystick.get_button(0) == 1):
                pygame.event.get()
                time.sleep(0.1)
    # End of main loop

    if odrives:
        board_0_driver.set_idle([True, True])
        board_1_driver.set_idle([True, True])
##    interpreter.end_interpreter()
