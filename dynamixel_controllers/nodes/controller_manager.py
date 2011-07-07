#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Antons Rebguns, Cody Jorgensen, Cara Slutter.
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


__author__ = 'Antons Rebguns, Cody Jorgensen, Cara Slutter'
__copyright__ = 'Copyright (c) 2010-2011 Antons Rebguns, Cody Jorgensen, Cara Slutter'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import sys

import roslib
roslib.load_manifest('dynamixel_controllers')

import rospy

from dynamixel_driver.dynamixel_serial_proxy import SerialProxy

from dynamixel_controllers.srv import StartController
from dynamixel_controllers.srv import StartControllerResponse
from dynamixel_controllers.srv import StopController
from dynamixel_controllers.srv import StopControllerResponse
from dynamixel_controllers.srv import RestartController
from dynamixel_controllers.srv import RestartControllerResponse

class ControllerManager:
    def __init__(self):
        rospy.init_node('dynamixel_controller_manager', anonymous=False)
        
        port_name = rospy.get_param('~port_name', '/dev/ttyUSB0')
        baud_rate = rospy.get_param('~baud_rate', 1000000)
        min_motor_id = rospy.get_param('~min_motor_id', 1)
        max_motor_id = rospy.get_param('~max_motor_id', 25)
        update_rate = rospy.get_param("~update_rate", 5)
        namespace = port_name[port_name.rfind('/') + 1:]
        
        self.serial_proxy = SerialProxy(port_name, baud_rate, min_motor_id, max_motor_id, update_rate)
        self.serial_proxy.connect()
        
        rospy.on_shutdown(self.serial_proxy.disconnect)
        
        self.controllers = {}
        
        rospy.Service('start_controller/%s' % namespace, StartController, self.start_controller)
        rospy.Service('stop_controller/%s' % namespace, StopController, self.stop_controller)
        rospy.Service('restart_controller/%s' % namespace, RestartController, self.restart_controller)

    def start_controller(self, req):
        port_name = req.port_name
        package_path = req.package_path
        module_name = req.module_name
        class_name = req.class_name
        controller_name = req.controller_name
        
        if controller_name in self.controllers:
            return StartControllerResponse(False, 'Controller [%s] already started. If you want to restart it, call restart.' % controller_name)
            
        # make sure the package_path is in PYTHONPATH
        if not package_path in sys.path:
            sys.path.append(package_path)
            
        try:
            if module_name not in sys.modules:
                # import if module not previously imported
                controller_module = __import__(module_name)
            else:
                # reload module if previously imported
                controller_module = reload(sys.modules[module_name])
        except ImportError, ie:
            return StartControllerResponse(False, 'Cannot find controller module. Unable to start controller %s\n%s' % (module_name, str(ie)))
        except SyntaxError, se:
            return StartControllerResponse(False, 'Syntax error in controller module. Unable to start controller %s\n%s' % (module_name, str(se)))
        except Exception, e:
            return StartControllerResponse(False, 'Unknown error has occured. Unable to start controller %s\n%s' % (module_name, str(e)))
        
        kls = getattr(controller_module, class_name)
        controller = kls(self.serial_proxy.dxl_io, controller_name, port_name)
        
        if controller.initialize():
            controller.start()
            self.controllers[controller_name] = controller
            return StartControllerResponse(True, 'Controller %s successfully started.' % controller_name)
        else:
            return StartControllerResponse(False, 'Initialization failed. Unable to start controller %s' % controller_name)

    def stop_controller(self, req):
        controller_name = req.controller_name
        
        if controller_name in self.controllers:
            self.controllers[controller_name].stop()
            del self.controllers[controller_name]
            return StopControllerResponse(True, 'controller %s successfully stopped.' % controller_name)
        else:
            return StopControllerResponse(False, 'controller %s was not running.' % controller_name)

    def restart_controller(self, req):
        response1 = self.stop_controller(req)
        response2 = self.start_controller(req)
        return RestartControllerResponse(response1.success and response2.success, '%s\n%s' % (response1.reason, response2.reason))

if __name__ == '__main__':
    try:
        manager = ControllerManager()
        rospy.spin()
    except rospy.ROSInterruptException: pass

