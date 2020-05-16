# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = 'Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns'
__credits__ = 'Cody Jorgensen, Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


from threading import Thread, Lock

from dynamixel_driver.dynamixel_serial_proxy import SerialProxy

import rospy

class DummyDynamixelIO(object):
    """ Provides low level IO with the Dynamixel servos through pyserial. Has the
    ability to write instruction packets, request and read register value
    packets, send and receive a response to a ping packet, and send a SYNC WRITE
    multi-servo instruction packet.
    """

    def __init__(self, port, baudrate, readback_echo=False):
        """ Constructor takes serial port and baudrate as arguments. """
        self.serial_mutex = Lock()
        self.port_name = port
        self.readback_echo = readback_echo

        self.goal = {}
        self.position = {}
        self.speed = {}

    def ping(self, servo_id):
        self.goal[servo_id] = 2048
        self.position[servo_id] = 2048
        self.speed[servo_id] = 0
        return True;

    def get_model_number(self, servo_id):
        """ Reads the servo's model number (e.g. 12 for AX-12+). """
        return 12

    def get_angle_limits(self, servo_id):
        cwLimit = 0
        ccwLimit = 4096
        return {'min':cwLimit, 'max':ccwLimit}

    def get_voltage(self, servo_id):
        return 12

    def get_voltage_limits(self, servo_id):
        # return the data in a dictionary
        min_voltage = 8
        max_voltage = 12
        return {'min':min_voltage, 'max':max_voltage}

    def get_position(self, servo_id):
        return self.position[servo_id]

    def get_speed(self, servo_id):
        return self.speed[servo_id]

    def get_current(self, servo_id):
        return 0

    def get_firmware_version(self, servo_id):
        return None

    def get_return_delay_time(self, servo_id):
        """ Reads the servo's return delay time. """
        return 5

    def get_feedback(self, servo_id):
        """
        Returns the id, goal, position, error, speed, load, voltage, temperature
        and moving values from the specified servo.
        """

        error = 0
        if self.speed[servo_id] != 0 :
            speed = self.speed[servo_id]
            self.goal[servo_id] += speed
        else:
            speed = (self.goal[servo_id] -  self.position[servo_id])*0.1
        self.position[servo_id] += speed
        return { 'timestamp': rospy.Time.now().to_sec(),
                 'id': servo_id,
                 'goal': self.goal[servo_id],
                 'position': self.position[servo_id],
                 'error': error,
                 'speed': speed}

    def set_multi_speed(self, valueTuples):
        for servo_id,val in valueTuples:
            self.speed[servo_id] = val
        return True

    def set_multi_position(self, valueTuples):
        for servo_id,val in valueTuples:
            self.goal[servo_id] = val
            self.speed[servo_id] = 0
        return True


class DummyProxy(SerialProxy):
    def connect(self):
        self.dxl_io = DummyDynamixelIO(self.port_name, self.baud_rate, self.readback_echo)
        self._SerialProxy__find_motors()
        self.running = True
        if self.update_rate > 0: Thread(target=self._SerialProxy__update_motor_states).start()
        if self.diagnostics_rate > 0: Thread(target=self._SerialProxy__publish_diagnostic_information).start()
