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


import math
import sys
import errno
from threading import Thread

import roslib
roslib.load_manifest('dynamixel_driver')

import rospy
import dynamixel_io
from dynamixel_driver.dynamixel_const import *

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

class SerialProxy():
    def __init__(self, port_name='/dev/ttyUSB0', baud_rate='1000000', min_motor_id=1, max_motor_id=25, update_rate=5):
        self.port_name = port_name
        self.port_namespace = port_name[port_name.rfind('/') + 1:]
        self.baud_rate = baud_rate
        self.min_motor_id = min_motor_id
        self.max_motor_id = max_motor_id
        self.update_rate = update_rate
        
        self.motor_states_pub = rospy.Publisher('motor_states/%s' % self.port_namespace, MotorStateList)

    def connect(self):
        try:
            self.dxl_io = dynamixel_io.DynamixelIO(self.port_name, self.baud_rate)
            self.__find_motors()
        except dynamixel_io.SerialOpenError, e:
            rospy.logfatal(e.message)
            sys.exit(1)
            
        self.running = True
        if self.update_rate > 0: Thread(target=self.__update_motor_states).start()

    def disconnect(self):
        self.running = False

    def __fill_motor_parameters(self, motor_id, model_number):
        """
        Stores some extra information about each motor on the parameter server.
        Some of these paramters are used in joint controller implementation.
        """
        angles = self.dxl_io.get_angle_limits(motor_id)
        voltage = self.dxl_io.get_voltage(motor_id)
        
        rospy.set_param('dynamixel/%s/%d/model_number' %(self.port_namespace, motor_id), model_number)
        rospy.set_param('dynamixel/%s/%d/model_name' %(self.port_namespace, motor_id), DXL_MODEL_TO_PARAMS[model_number]['name'])
        rospy.set_param('dynamixel/%s/%d/min_angle' %(self.port_namespace, motor_id), angles['min'])
        rospy.set_param('dynamixel/%s/%d/max_angle' %(self.port_namespace, motor_id), angles['max'])
        
        torque_per_volt = DXL_MODEL_TO_PARAMS[model_number]['torque_per_volt']
        rospy.set_param('dynamixel/%s/%d/torque_per_volt' %(self.port_namespace, motor_id), torque_per_volt)
        rospy.set_param('dynamixel/%s/%d/max_torque' %(self.port_namespace, motor_id), torque_per_volt * voltage)
        
        velocity_per_volt = DXL_MODEL_TO_PARAMS[model_number]['velocity_per_volt']
        rospy.set_param('dynamixel/%s/%d/velocity_per_volt' %(self.port_namespace, motor_id), velocity_per_volt)
        rospy.set_param('dynamixel/%s/%d/max_velocity' %(self.port_namespace, motor_id), velocity_per_volt * voltage)
        rospy.set_param('dynamixel/%s/%d/radians_second_per_encoder_tick' %(self.port_namespace, motor_id), velocity_per_volt * voltage / DXL_MAX_SPEED_TICK)
        
        encoder_resolution = DXL_MODEL_TO_PARAMS[model_number]['encoder_resolution']
        range_degrees = DXL_MODEL_TO_PARAMS[model_number]['range_degrees']
        range_radians = math.radians(range_degrees)
        rospy.set_param('dynamixel/%s/%d/encoder_resolution' %(self.port_namespace, motor_id), encoder_resolution)
        rospy.set_param('dynamixel/%s/%d/range_degrees' %(self.port_namespace, motor_id), range_degrees)
        rospy.set_param('dynamixel/%s/%d/range_radians' %(self.port_namespace, motor_id), range_radians)
        rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_degree' %(self.port_namespace, motor_id), encoder_resolution / range_degrees)
        rospy.set_param('dynamixel/%s/%d/encoder_ticks_per_radian' %(self.port_namespace, motor_id), encoder_resolution / range_radians)
        rospy.set_param('dynamixel/%s/%d/degrees_per_encoder_tick' %(self.port_namespace, motor_id), range_degrees / encoder_resolution)
        rospy.set_param('dynamixel/%s/%d/radians_per_encoder_tick' %(self.port_namespace, motor_id), range_radians / encoder_resolution)

    def __find_motors(self):
        rospy.loginfo('Pinging motor IDs %d through %d...' % (self.min_motor_id, self.max_motor_id))
        self.motors = []
        
        try:
            for motor_id in range(self.min_motor_id, self.max_motor_id + 1):
                result = self.dxl_io.ping(motor_id)
                if result: self.motors.append(motor_id)
                
            if self.motors:
                rospy.loginfo('Found motors with IDs: %s.' % str(self.motors))
            else:
                rospy.logfatal('No motors found, aborting.')
                sys.exit(1)
                
            rospy.set_param('dynamixel/%s/connected_ids' % self.port_namespace, self.motors)
            
            counts = {10: 0, 12: 0, 18: 0, 24: 0, 28: 0, 29: 0, 64: 0, 107: 0, 113: 0, 116: 0, 117: 0}
            
            for motor_id in self.motors:
                model_number = self.dxl_io.get_model_number(motor_id)
                counts[model_number] += 1
                self.__fill_motor_parameters(motor_id, model_number)
                
            status_str = 'There are '
            for model_number,count in counts.items():
                if count: status_str += '%d %s, ' % (count, DXL_MODEL_TO_PARAMS[model_number]['name'])
                
            rospy.loginfo('%s servos connected' % status_str[:-2])
            rospy.loginfo('Dynamixel Manager on port %s initialized' % self.port_name)
        except dynamixel_io.FatalErrorCodeError, fece:
            rospy.logfatal(fece)
            signal_shutdown(fece)
        except dynamixel_io.NonfatalErrorCodeError, nfece:
            rospy.logwarn(nfece)
        except dynamixel_io.ChecksumError, cse:
            rospy.logwarn(cse)
        except dynamixel_io.DroppedPacketError, dpe:
            rospy.loginfo(dpe.message)

    def __update_motor_states(self):
        rate = rospy.Rate(self.update_rate)
        while self.running:
            # get current state of all motors and publish to motor_states topic
            motor_states = []
            for motor_id in self.motors:
                try:
                    state = self.dxl_io.get_feedback(motor_id)
                    if state:
                        motor_states.append(MotorState(**state))
                        if dynamixel_io.exception: raise dynamixel_io.exception
                except dynamixel_io.FatalErrorCodeError, fece:
                    rospy.logfatal(fece)
                    rospy.signal_shutdown(fece)
                except dynamixel_io.NonfatalErrorCodeError, nfece:
                    rospy.logdebug(nfece)
                except dynamixel_io.ChecksumError, cse:
                    rospy.logdebug(cse)
                except dynamixel_io.DroppedPacketError, dpe:
                    rospy.logdebug(dpe.message)
                except OSError, ose:
                    if ose.errno != errno.EAGAIN:
                        rospy.logfatal(errno.errorcode[ose.errno])
                        rospy.signal_shutdown(errno.errorcode[ose.errno])
                        
            if motor_states:
                msl = MotorStateList()
                msl.motor_states = motor_states
                self.motor_states_pub.publish(msl)
                
            rate.sleep()

if __name__ == '__main__':
    try:
        serial_proxy = SerialProxy()
        serial_proxy.connect()
        rospy.spin()
        serial_proxy.disconnect()
    except rospy.ROSInterruptException: pass

