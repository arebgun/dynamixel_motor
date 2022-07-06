import rospy
import actionlib
from dynamixel_controllers.joint_position_controller import (
    JointPositionController,
)
from dynamixel_msgs.msg import MotorStateList
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
from dynamixel_controllers.msg import (
    CalibJointAction,
    CalibJointResult,
)


class CalibRequiredJointController(JointPositionController):
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        JointPositionController.__init__(self, dxl_io, controller_namespace,
                                         port_namespace)

        self.calib_speed = rospy.get_param(
                self.controller_namespace + '/calib_speed',
                0.1)
        self.calib_torque_limit = rospy.get_param(
                self.controller_namespace + '/calib_torque_limit',
                0.3)
        self.detect_limit_load = rospy.get_param(
                self.controller_namespace + '/detect_limit_load',
                0.15)
        self.is_multiturn = rospy.get_param(
                self.controller_namespace + '/is_multiturn',
                False)

        self.calib_server = actionlib.SimpleActionServer(
                self.controller_namespace + '/calib',
                CalibJointAction,
                execute_cb=self.on_calib_action,
                auto_start=False)

    def initialize(self):
        if not JointPositionController.initialize(self):
            return False

        self.__calib()
        self.calib_server.start()

        return (not rospy.is_shutdown())

    def on_calib_action(self, goal):
        self.__calib()
        self.calib_server.set_succeeded(CalibJointResult())

    def __calib(self):
        # Initialize joint position

        self.motor_states_for_init = {}
        motor_states_sub_for_init = rospy.Subscriber('motor_states/%s' % self.port_namespace, MotorStateList, self.get_motor_states_for_init)
        self.set_speed(0.0)
        # Backup current angle limits
        prev_limits = self.__get_angle_limits()
        # Change to wheel mode
        self.__set_angle_limits(0, 0)
        self.set_torque_limit(self.calib_torque_limit)
        self.__set_speed_wheel(0.0)
        # release torque by disabling it
        self.set_torque_enable(False)
        rospy.sleep(0.2)
        self.set_torque_enable(True)  # re-enable it
        rospy.sleep(0.2)
        if self.flipped:
            self.__set_speed_wheel(self.calib_speed)
        else:
            self.__set_speed_wheel(-self.calib_speed)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                init_pos = self.motor_states_for_init['position']
                if abs(self.motor_states_for_init['load']) > self.detect_limit_load:
                    break
            except KeyError:
                pass
            rate.sleep()
        self.__set_speed_wheel(0.0)
        if self.is_multiturn:
            # Change to multiturn mode
            self.__set_angle_limits(4095, 4095)
        else:
            # Change to previous mode
            self.__set_angle_limits(prev_limits['min'], prev_limits['max'])
        self.set_torque_enable(False)
        self.set_speed(self.joint_speed)
        rospy.sleep(0.5)
        if self.torque_limit is not None:
            self.set_torque_limit(self.torque_limit)
        motor_states_sub_for_init.unregister()

        # Remember initial joint position
        diff = init_pos - self.initial_position_raw
        self.initial_position_raw += diff
        rospy.set_param(self.controller_namespace + '/motor/init',
                        self.initial_position_raw)
        self.min_angle_raw += diff
        rospy.set_param(self.controller_namespace + '/motor/min',
                        self.min_angle_raw)
        self.max_angle_raw += diff
        rospy.set_param(self.controller_namespace + '/motor/max',
                        self.max_angle_raw)
        if self.flipped:
            self.min_angle = ((self.initial_position_raw -
                               self.min_angle_raw) *
                              self.RADIANS_PER_ENCODER_TICK)
            self.max_angle = ((self.initial_position_raw -
                               self.max_angle_raw) *
                              self.RADIANS_PER_ENCODER_TICK)
        else:
            self.min_angle = ((self.min_angle_raw -
                               self.initial_position_raw) *
                              self.RADIANS_PER_ENCODER_TICK)
            self.max_angle = ((self.max_angle_raw -
                               self.initial_position_raw) *
                              self.RADIANS_PER_ENCODER_TICK)

    def __set_angle_limits(self, min_angle, max_angle):
        self.dxl_io.set_angle_limits(self.motor_id, min_angle, max_angle)

    def __get_angle_limits(self):
        return self.dxl_io.get_angle_limits(self.motor_id)

    def get_motor_states_for_init(self, state_list):
        state = filter(lambda state: state.id == self.motor_id, state_list.motor_states)
        if state:
            state = state[0]
            self.motor_states_for_init['position'] = state.position
            self.motor_states_for_init['load'] = state.load

    def __spd_rad_to_raw_wheel(self, spd_rad):
        if spd_rad < -self.joint_max_speed:
            spd_rad = -self.joint_max_speed
        elif spd_rad > self.joint_max_speed:
            spd_rad = self.joint_max_speed
        return int(round(spd_rad / self.VELOCITY_PER_TICK))

    def __set_speed_wheel(self, speed):
        mcv = (self.motor_id, self.__spd_rad_to_raw_wheel(speed))
        self.dxl_io.set_multi_speed([mcv])
