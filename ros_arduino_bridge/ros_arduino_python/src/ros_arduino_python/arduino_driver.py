#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""
# Se adapta el programa para establecer la conexion a traves de una red Wifi
# Modificaciones realizadas por Alberto Diaz Rdoriguez
# Universidad de La Laguna

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial
import socket

SERVO_MAX = 180
SERVO_MIN = 0

class Arduino:
    ''' Configuration Parameters
    '''
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def __init__(self, IP="192.168.43.168", socket=4445, timeout=0.5, motors_reversed=False):

        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.socket = socket
	    self.IP = IP
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.
        self.motors_reversed = motors_reversed
        # Keep things thread safe
        self.mutex = thread.allocate_lock()

        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    def connect(self):
        try:
            print "Connecting to Arduino on port", self.IP, " ", self.socket, "..."
	    self.port = socket.socket(socket.AF_INET,  socket.SOCK_DGRAM)

	    self.port.settimeout(2)
            #self.port = Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.writeTimeout)
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(0.1)
            #test = self.get_baud()
            #if test != self.baudrate:
                #time.sleep(1)
                #test = self.get_baud()
                #if test != self.baudrate:
                    #raise SerialException
            print "Connected at", self.socket
            print "Arduino is ready."

        except SerialException:
            print "Serial Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self):
        ''' Open the serial port.
        '''
        self.port.open()

    def close(self):
        ''' Close the serial port.
        '''
        self.port.close()

    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.sendto(cmd + '\r',(self.IP,self.socket))

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        while c != '\r':
            c = self.port.recv(10)
            value += c
            attempts += 1
            if attempts * self.interCharTimeout > timeout:
                return None

        value = value.strip('\r')

        return value

    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return map(int, values)
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning a single integer value.
        '''
        self.mutex.acquire()
        
        

        ntries = 1
        attempts = 0


        #try:
            ##print "Enviando" +  self.IP + self.socket
        self.port.sendto(cmd + '\r',(self.IP,self.socket))
            ##value = self.port.recv(15)


        self.mutex.release()
        #return int(value)
        return 1
    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        try:
            self.port.sendto(cmd + '\r',(self.IP,self.socket))
            values = self.recv_array()
            while attempts < ntries and (values == '' or values == 'Invalid Command' or values == [] or values == None):
                try:
                    self.port.flushInput()
                    self.port.sendto(cmd + '\r',(self.IP,self.socket))
                    values = self.recv_array()
                except:
                    print("Exception executing command: " + cmd)
                attempts += 1
        except:
            self.mutex.release()
            print "Exception executing command: " + cmd
            raise SerialException
            return []

        try:
            values = map(int, values)
        except:
            values = []

        self.mutex.release()
        return values

    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()

        try:
            self.port.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        #try:
        self.port.sendto(cmd + '\r',(self.IP,self.socket))

        self.mutex.release()
        return 1

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print "Updating PID parameters"
        cmd = 'u ' + str(Kp) + ':' + str(Kd) + ':' + str(Ki) + ':' + str(Ko)
        self.execute_ack(cmd)

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        try:
            return int(self.execute('b'));
        except:
            return None

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2:
            print "Encoder count was not 2"
            raise SerialException
            return None
        else:
            if self.motors_reversed:
                values[0], values[1] = -values[0], values[1]
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')

    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        if self.motors_reversed:
            left, right = -left, -right
        return self.execute_ack('m %d %d' %(right, left))

    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )

    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    def analog_read(self, pin):
        return self.execute('a %d' %pin)

    #def motor_1_speed(self, pin):
        #return self.execute('x %d' %pin)

    def motors_speed(self, vel1, vel2, vel3):
        return self.execute_ack('x %d %d %d' %(vel1, vel2, vel3))

    def analog_write(self, pin, value):
        return self.execute_ack('v %d %d' %(pin, value))

    def digital_read(self, pin):
        return self.execute('d %d' %pin)

    def digital_write(self, pin, value):
        return self.execute_ack('w %d %d' %(pin, value))

    def pin_mode(self, pin, mode):
        return self.execute_ack('c %d %d' %(pin, mode))

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''
        return self.execute_ack('s %d %d' %(id, min(SERVO_MAX, max(SERVO_MIN, degrees(pos)))))

    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''
        return radians(self.execute('t %d' %id))

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        return self.execute('p %d' %pin);




""" Basic test for connectivity """
if __name__ == "__main__":
    if os.name == "posix":
        IP_name = "192.168.43.168"


    myArduino = Arduino(IP=IP_name, socket=4445, timeout=0.5)
    myArduino.connect()

    print "Sleeping for 0.1 second..."
    time.sleep(0.1)

    print "Estableciendo velocidad de los motores", myArduino.motors_speed(150,150,150)

    print "Connection test successful.",

    #myArduino.stop()
    myArduino.close()

    print "Shutting down Arduino."
