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


from __future__ import division

import roslib
roslib.load_manifest('dynamixel_controllers')

import rospy
import actionlib

from dynamixel_driver.dynamixel_const import DXL_MIN_SPEED_RAD

from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from pr2_controllers_msgs.msg import JointTrajectoryAction
from dynamixel_msgs.msg import JointState

from pr2_controllers_msgs.srv import QueryTrajectoryState
from dynamixel_controllers.srv import SetSpeed

class Segment():
    def __init__(self, num_joints):
        self.start_time = 0.0                   # trajectory segment start time
        self.duration = 0.0                     # trajectory segment duration
        self.positions = [0.0] * num_joints
        self.velocities = [0.0] * num_joints

class JointTrajectoryActionController():
    def __init__(self):
        self.update_rate = 1000
        self.trajectory = []
        
        self.controller_namespace = rospy.get_param('~controller_namespace', 'l_arm_controller')
        self.joint_controllers = rospy.get_param(self.controller_namespace + '/joint_controllers', [])
        self.joint_names = []
        
        for controller in self.joint_controllers:
            self.joint_names.append(rospy.get_param(controller + '/joint_name'))
            
        self.joint_states = dict(zip(self.joint_names, [JointState(name=jn) for jn in self.joint_names]))
        self.num_joints = len(self.joint_names)
        self.joint_to_idx = dict(zip(self.joint_names, range(self.num_joints)))
        
        ns = self.controller_namespace + '/joint_trajectory_action_node/constraints'
        self.goal_time_constraint = rospy.get_param(ns + '/goal_time', 0.0)
        self.stopped_velocity_tolerance = rospy.get_param(ns + '/stopped_velocity_tolerance', 0.01)
        self.goal_constraints = []
        self.trajectory_constraints = []
        self.min_velocity = rospy.get_param(self.controller_namespace + '/joint_trajectory_action_node/min_velocity', DXL_MIN_SPEED_RAD)
        
        for joint in self.joint_names:
            self.goal_constraints.append(rospy.get_param(ns + '/' + joint + '/goal', -1.0))
            self.trajectory_constraints.append(rospy.get_param(ns + '/' + joint + '/trajectory', -1.0))
            
        # Message containing current state for all controlled joints
        self.msg = JointTrajectoryControllerState()
        self.msg.joint_names = self.joint_names
        self.msg.desired.positions = [0.0] * self.num_joints
        self.msg.desired.velocities = [0.0] * self.num_joints
        self.msg.desired.accelerations = [0.0] * self.num_joints
        self.msg.actual.positions = [0.0] * self.num_joints
        self.msg.actual.velocities = [0.0] * self.num_joints
        self.msg.error.positions = [0.0] * self.num_joints
        self.msg.error.velocities = [0.0] * self.num_joints
        
        # Keep track of last position and velocity sent to each joint
        self.last_commanded = {}
        for joint in self.joint_names:
            self.last_commanded[joint] = { 'position': None, 'velocity': None }
            
        # Publishers
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state', JointTrajectoryControllerState)
        self.joint_position_pubs = [rospy.Publisher(controller + '/command', Float64) for controller in self.joint_controllers]
        
        # Subscribers
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/command', JointTrajectory, self.process_command)
        self.joint_state_subs = [rospy.Subscriber(controller + '/state', JointState, self.process_joint_states) for controller in self.joint_controllers]
        
        # Services
        self.joint_velocity_srvs = [rospy.ServiceProxy(controller + '/set_speed', SetSpeed, persistent=True) for controller in self.joint_controllers]
        self.query_state_service = rospy.Service(self.controller_namespace + '/query_state', QueryTrajectoryState, self.process_query_state)
        self.action_server = actionlib.SimpleActionServer(self.controller_namespace + '/joint_trajectory_action',
                                                          JointTrajectoryAction,
                                                          execute_cb=self.process_trajectory_action)

    def process_command(self, msg):
        if self.action_server.is_active(): self.action_server.set_preempted()
        
        while self.action_server.is_active():
            sleep(0.01)
            
        process_trajectory_action(msg)

    def process_query_state(self, req):
        if not self.trajectory: return False
        
        # Determines which segment of the trajectory to use
        seg = -1
        while seg + 1 < len(self.trajectory) and self.trajectory[seg+1].start_time < req.time.to_sec():
            seg += 1
            
        if seg == -1: return False
        
        resp = QueryTrajectoryState()
        resp.name = self.joint_names
        resp.position = [0.0] * self.num_joints
        resp.velocity = [0.0] * self.num_joints
        resp.acceleration = [0.0] * self.num_joints
        
        seg_end_time = self.trajectory.start_time + self.trajectory.duration
        t = self.trajectory.duration - (seg_end_time - req.time)
        
        for j, joint in enumerate(self.joint_names):
            resp.position[j] = self.joint_states[joint].current_pos
            resp.velocity[j] = self.joint_states[joint].velocity
            
        return resp

    def process_trajectory_action(self, goal):
        traj = goal.trajectory
        
        # ensure that the joints in the goal match the joints of the controller
        if set(self.joint_names) != set(traj.joint_names):
            msg = "Incoming trajectory joints don't match our joints"
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            return
            
        # make sure trajectory is not empty
        if not traj.points:
            msg = "Incoming trajectory is empty"
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            return
            
        # correlate the joints we're commanding to the joints in the message
        # map from an index of joint in the controller to an index in the trajectory
        try:
            lookup = [traj.joint_names.index(joint) for joint in self.joint_names]
        except ValueError as val:
            msg = 'Joint in the controller and those specified in the trajectory do not match.'
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            return
            
        durations  = [0.0] * len(traj.points)
        
        # find out the duration of each segment in the trajectory
        if len(traj.points) > 0:
            durations[0] = traj.points[0].time_from_start.to_sec()
            
        for i in range(1, len(traj.points)):
            durations[i] = (traj.points[i].time_from_start - traj.points[i-1].time_from_start).to_sec()
            
        if not traj.points[0].positions:
            msg = 'First point of trajectory has no positions'
            rospy.logerr(msg)
            self.action_server.set_aborted(text=msg)
            return
            
        trajectory = []
        time = rospy.Time.now() + rospy.Duration(0.01)
        
        for i in range(len(traj.points)):
            seg = Segment(self.num_joints)
            
            if traj.header.stamp == rospy.Time(0.0):
                seg.start_time = (time + traj.points[i].time_from_start).to_sec() - durations[i]
            else:
                seg.start_time = (traj.header.stamp + traj.points[i].time_from_start).to_sec() - durations[i]
                
            seg.duration = durations[i]
            
            # Checks that the incoming segment has the right number of elements.
            if traj.points[i].velocities and len(traj.points[i].velocities) != self.num_joints:
                msg = 'Command point %d has %d elements for the velocities' % (i, len(traj.points[i].velocities))
                rospy.logerr(msg)
                self.action_server.set_aborted(text=msg)
                return
                
            if len(traj.points[i].positions) != self.num_joints:
                msg = 'Command point %d has %d elements for the positions' % (i, len(traj.points[i].positions))
                rospy.logerr(msg)
                self.action_server.set_aborted(text=msg)
                return
                
            for j in range(self.num_joints):
                if traj.points[i].velocities:
                    seg.velocities[j] = traj.points[i].velocities[lookup[j]]
                if traj.points[i].positions:
                    seg.positions[j] = traj.points[i].positions[lookup[j]]
                    
            trajectory.append(seg)
            
        rospy.loginfo('Trajectory start requested at %.3lf, waiting...', traj.header.stamp.to_sec())
        rate = rospy.Rate(self.update_rate)
        
        while traj.header.stamp > time:
            time = rospy.Time.now()
            rate.sleep()
            
        end_time = traj.header.stamp + rospy.Duration(sum(durations))
        seg_end_times = [rospy.Time.from_sec(trajectory[seg].start_time + durations[seg]) for seg in range(len(trajectory))]
        
        rospy.loginfo('Trajectory start time is %.3lf, end time is %.3lf, total duration is %.3lf', time.to_sec(), end_time.to_sec(), sum(durations))
        
        q  = [0.0] * self.num_joints  # desired positions
        qd = [0.0] * self.num_joints  # desired velocities
        
        self.trajectory = trajectory
        traj_start_time = rospy.Time.now()
        
        for seg in range(len(trajectory)):
            rospy.logdebug('current segment is %d time left %f cur time %f' % (seg, durations[seg] - (time.to_sec() - trajectory[seg].start_time), time.to_sec()))
            rospy.logdebug('goal positions are: %s' % str(trajectory[seg].positions))
            
            # first point in trajectories calculated by OMPL is current position with duration of 0 seconds, skip it
            if durations[seg] == 0:
                rospy.logdebug('skipping segment %d with duration of 0 seconds' % seg)
                continue
                
            # loop in reverse to send commands to less important joints first,
            # e.g. start from wrist and work up to shoulder joint
            for joint in reversed(self.joint_names):
                j = self.joint_names.index(joint)
                
                start_position = self.joint_states[joint].current_pos
                if seg != 0: start_position = trajectory[seg-1].positions[j]
                    
                q[j] = trajectory[seg].positions[j]
                qd[j] = max(self.min_velocity, abs(q[j] - start_position) / durations[seg])
                
                self.msg.desired.positions[j] = q[j]
                self.msg.desired.velocities[j] = qd[j]
                
                self.set_joint_velocity(joint, qd[j])
                self.set_joint_angle(joint, q[j])
                
            while time < seg_end_times[seg]:
                # check if new trajectory was received, if so abort current trajectory execution
                if self.action_server.is_preempt_requested():
                    msg = 'New trajectory received. Aborting old trajectory.'
                    
                    for j in range(self.num_joints):
                        cur_pos = self.joint_states[self.joint_names[j]].current_pos
                        self.set_joint_angle(self.joint_names[j], cur_pos)
                        
                    self.action_server.set_preempted(text=msg)
                    rospy.logwarn(msg)
                    return
                    
                rate.sleep()
                time = rospy.Time.now()
                
            # Verifies trajectory constraints
            for j, joint in enumerate(self.joint_names):
                if self.trajectory_constraints[j] > 0 and self.msg.error.positions[j] > self.trajectory_constraints[j]:
                    msg = 'Unsatisfied position constraint for %s, trajectory point %d, %f is larger than %f' % \
                           (joint, seg, self.msg.error.positions[j], self.trajectory_constraints[j])
                    rospy.logwarn(msg)
                    self.action_server.set_aborted(text=msg)
                    return
                    
        traj_actual_time = (rospy.Time.now() - traj_start_time).to_sec()
        traj_time_err = traj_actual_time - sum(durations)
        rospy.loginfo('Trajectory execution took %f seconds, error is %f' % (traj_actual_time, traj_time_err))
        
        # let motors roll for specified amount of time
        rospy.sleep(self.goal_time_constraint)
        
        for i, j in enumerate(self.joint_names):
            rospy.logdebug('desired pos was %f, actual pos is %f, error is %f' % (trajectory[-1].positions[i], self.joint_states[j].current_pos, self.joint_states[j].current_pos - trajectory[-1].positions[i]))
            
        # Checks that we have ended inside the goal constraints
        for (joint, pos_error, pos_constraint) in zip(self.joint_names, self.msg.error.positions, self.goal_constraints):
            if pos_constraint > 0 and abs(pos_error) > pos_constraint:
                msg = 'Aborting because %s joint wound up outside the goal constraints, %f is larger than %f' % \
                      (joint, pos_error, pos_constraint)
                rospy.logwarn(msg)
                self.action_server.set_aborted(text=msg)
                break
        else:
            rospy.loginfo('Trajectory execution successfully completed')
            self.action_server.set_succeeded()



    ################################################################################
    #----------------------- Low-level servo control functions --------------------#
    ################################################################################

    def process_joint_states(self, msg):
        self.msg.header.stamp = rospy.Time.now()
        
        self.joint_states[msg.name] = msg
        
        # Publish current joint state
        for i, joint in enumerate(self.joint_names):
            state = self.joint_states[joint]
            self.msg.actual.positions[i] = state.current_pos
            self.msg.actual.velocities[i] = abs(state.velocity)
            self.msg.error.positions[i] = self.msg.actual.positions[i] - self.msg.desired.positions[i]
            self.msg.error.velocities[i] = self.msg.actual.velocities[i] - self.msg.desired.velocities[i]
            
        self.state_pub.publish(self.msg)

    def command_will_update(self, command, joint, value):
        current_state = None
        command_resolution = None
        
        if command == 'position':
            current_state = self.joint_states[joint].current_pos
            command_resolution = 0.006
        elif command == 'velocity':
            current_state = self.joint_states[joint].velocity
            command_resolution = DXL_MIN_SPEED_RAD
        else:
            rospy.logerr('Unrecognized motor command %s while setting %s to %f', command, joint, value)
            return False
            
        last_commanded = self.last_commanded[joint][command]
        
        # no sense in sending command, change is too small for the motors to move
        if last_commanded is not None and (abs(value - last_commanded) < command_resolution): return False
        if current_state is not None and (abs(value - current_state) < command_resolution): return False
        
        self.last_commanded[joint][command] = value
        
        return True

    def set_joint_velocity(self, joint, velocity):
        if self.command_will_update('velocity', joint, velocity):
            self.joint_velocity_srvs[self.joint_to_idx[joint]](velocity)

    def set_joint_angle(self, joint, angle):
        if self.command_will_update('position', joint, angle):
            self.joint_position_pubs[self.joint_to_idx[joint]].publish(angle)


if __name__ == '__main__':
    try:
        rospy.init_node('joint_trajectory_action_controller', anonymous=True)
        traj_controller = JointTrajectoryActionController()
        rospy.spin()
    except rospy.ROSInterruptException: pass

