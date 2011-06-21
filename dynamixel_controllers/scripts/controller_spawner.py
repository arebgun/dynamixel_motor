#!/usr/bin/env python
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

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import sys
import os
from optparse import OptionParser

import roslib
roslib.load_manifest('dynamixel_controllers')

from roslib.packages import InvalidROSPkgException

import rospy
from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import RestartController

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-c', '--command', metavar='COMMAND', default='start',
                      help='command to perform on specified controllers: start, stop or restart [default: %default]')
                      
    (options, args) = parser.parse_args(rospy.myargv()[1:])
    
    if len(args) < 1:
        print 'Specify at least one controller name to manage'
        
    port = options.port
    command = options.command
    joint_controllers = args
    
    device_namespace = port[port.rfind('/') + 1:]
    start_service_name = 'start_controller/%s' % device_namespace
    stop_service_name = 'stop_controller/%s' % device_namespace
    restart_service_name = 'restart_controller/%s' % device_namespace
    
    parent_namespace = 'global' if rospy.get_namespace() == '/' else rospy.get_namespace()
    rospy.loginfo('%s controller_spawner: waiting for controller_manager to startup in %s namespace...' % (device_namespace, parent_namespace))
    
    rospy.wait_for_service(start_service_name)
    rospy.wait_for_service(stop_service_name)
    rospy.wait_for_service(restart_service_name)
    
    start_controller = rospy.ServiceProxy(start_service_name, StartController)
    stop_controller = rospy.ServiceProxy(stop_service_name, StopController)
    restart_controller = rospy.ServiceProxy(restart_service_name, RestartController)
    
    rospy.loginfo('%s controller_spawner: All services are up, spawning controllers...' % device_namespace)
    
    for controller_name in joint_controllers:
        try:
            controller = rospy.get_param(controller_name + '/controller')
            package_path = roslib.packages.get_pkg_dir(controller['package'])
            package_path = os.path.join(package_path, 'src', controller['package'])
            module_name = controller['module']
            class_name = controller['type']
        except KeyError as ke:
            rospy.logerr('[%s] configuration error: could not find controller parameters on parameter server' % controller_name)
            sys.exit(1)
        except InvalidROSPkgException as pe:
            rospy.logerr('[%s] configuration error: %s' % (controller_name, pe))
            sys.exit(1)
        except Exception as e:
            rospy.logerr('[%s]: %s' % (controller_name, e))
            sys.exit(1)
            
        if command.lower() == 'start':
            try:
                response = start_controller(port, package_path, module_name, class_name, controller_name)
                if response.success: rospy.loginfo(response.reason)
                else: rospy.logerr(response.reason)
            except rospy.ServiceException, e:
                rospy.logerr('Service call failed: %s' % e)
        elif command.lower() == 'stop':
            try:
                response = stop_controller(controller_name)
                if response.success: rospy.loginfo(response.reason)
                else: rospy.logerr(response.reason)
            except rospy.ServiceException, e:
                rospy.logerr('Service call failed: %s' % e)
        elif command.lower() == 'restart':
            try:
                response = restart_controller(port, package_path, module_name, class_name, controller_name)
                if response.success: rospy.loginfo(response.reason)
                else: rospy.logerr(response.reason)
            except rospy.ServiceException, e:
                rospy.logerr('Service call failed: %s' % e)
        else:
            rospy.logerr('Invalid command.')
            parser.print_help()

