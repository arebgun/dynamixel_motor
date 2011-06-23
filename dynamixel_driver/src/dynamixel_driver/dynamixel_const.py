# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Cody Jorgensen, Antons Rebguns.
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


__author__ = 'Cody Jorgensen, Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Cody Jorgensen, Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


"""
Dynamixel Constants
"""
# Control Table Constants
DXL_MODEL_NUMBER_L = 0
DXL_MODEL_NUMBER_H = 1
DXL_VERSION = 2
DXL_ID = 3
DXL_BAUD_RATE = 4
DXL_RETURN_DELAY_TIME = 5
DXL_CW_ANGLE_LIMIT_L = 6
DXL_CW_ANGLE_LIMIT_H = 7
DXL_CCW_ANGLE_LIMIT_L = 8
DXL_CCW_ANGLE_LIMIT_H = 9
DXL_DRIVE_MODE = 10
DXL_LIMIT_TEMPERATURE = 11
DXL_DOWN_LIMIT_VOLTAGE = 12
DXL_UP_LIMIT_VOLTAGE = 13
DXL_MAX_TORQUE_L = 14
DXL_MAX_TORQUE_H = 15
DXL_RETURN_LEVEL = 16
DXL_ALARM_LED = 17
DXL_ALARM_SHUTDOWN = 18
DXL_OPERATING_MODE = 19
DXL_DOWN_CALIBRATION_L = 20
DXL_DOWN_CALIBRATION_H = 21
DXL_UP_CALIBRATION_L = 22
DXL_UP_CALIBRATION_H = 23
DXL_TORQUE_ENABLE = 24
DXL_LED = 25
DXL_CW_COMPLIANCE_MARGIN = 26
DXL_CCW_COMPLIANCE_MARGIN = 27
DXL_CW_COMPLIANCE_SLOPE = 28
DXL_CCW_COMPLIANCE_SLOPE = 29
DXL_GOAL_POSITION_L = 30
DXL_GOAL_POSITION_H = 31
DXL_GOAL_SPEED_L = 32
DXL_GOAL_SPEED_H = 33
DXL_TORQUE_LIMIT_L = 34
DXL_TORQUE_LIMIT_H = 35
DXL_PRESENT_POSITION_L = 36
DXL_PRESENT_POSITION_H = 37
DXL_PRESENT_SPEED_L = 38
DXL_PRESENT_SPEED_H = 39
DXL_PRESENT_LOAD_L = 40
DXL_PRESENT_LOAD_H = 41
DXL_PRESENT_VOLTAGE = 42
DXL_PRESENT_TEMPERATURE = 43
DXL_REGISTERED_INSTRUCTION = 44
DXL_PAUSE_TIME = 45
DXL_MOVING = 46
DXL_LOCK = 47
DXL_PUNCH_L = 48
DXL_PUNCH_H = 49
DXL_SENSED_CURRENT_L = 56
DXL_SENSED_CURRENT_H = 57

# Status Return Levels
DXL_RETURN_NONE = 0
DXL_RETURN_READ = 1
DXL_RETURN_ALL = 2

# Instruction Set
DXL_PING = 1
DXL_READ_DATA = 2
DXL_WRITE_DATA = 3
DXL_REG_WRITE = 4
DXL_ACTION = 5
DXL_RESET = 6
DXL_SYNC_WRITE = 131

# Broadcast Constant
DXL_BROADCAST = 254

# Error Codes
DXL_INSTRUCTION_ERROR = 64
DXL_OVERLOAD_ERROR = 32
DXL_CHECKSUM_ERROR = 16
DXL_RANGE_ERROR = 8
DXL_OVERHEATING_ERROR = 4
DXL_ANGLE_LIMIT_ERROR = 2
DXL_INPUT_VOLTAGE_ERROR = 1
DXL_NO_ERROR = 0

# Static parameters
DXL_MIN_COMPLIANCE_MARGIN = 0
DXL_MAX_COMPLIANCE_MARGIN = 255

DXL_MIN_COMPLIANCE_SLOPE = 0
DXL_MAX_COMPLIANCE_SLOPE = 254

# These are guesses as Dynamixel documentation doesn't have any info about it
DXL_MIN_PUNCH = 0
DXL_MAX_PUNCH = 255

DXL_MODEL_TO_NAME = \
{
    113: 'DX-113',
    116: 'DX-116',
    117: 'DX-117',
     12: 'AX-12+',
     18: 'AX-18F',
     10: 'RX-10',
     24: 'RX-24F',
     28: 'RX-28',
     64: 'RX-64',
    107: 'EX-106+',
     29: 'MX-28',
}

KGCM_TO_NM = 0.0980665                      # 1 kg-cm is that many N-m
DXL_MAX_TORQUE_TICK = 1023                  # maximum torque in encoder units
DXL_MODEL_TO_TORQUE = \
{
    113: ( 10 * KGCM_TO_NM) / 12.0,         #  10 kg-cm @ 12V
    116: ( 21 * KGCM_TO_NM) / 12.0,         #  21 kg-cm @ 12V
    117: ( 37 * KGCM_TO_NM) / 18.5,         #  37 kg-cm @ 18.5V
     12: ( 15 * KGCM_TO_NM) / 12.0,         #  15 kg-cm @ 12V
     18: ( 18 * KGCM_TO_NM) / 12.0,         #  18 kg-cm @ 12V
     10: ( 13 * KGCM_TO_NM) / 12.0,         #  13 kg-cm @ 12V
     24: ( 26 * KGCM_TO_NM) / 12.0,         #  26 kg-cm @ 12V
     28: ( 37 * KGCM_TO_NM) / 18.5,         #  37 kg-cm @ 18.5V
     64: ( 52 * KGCM_TO_NM) / 18.5,         #  52 kg-cm @ 18.5V
    107: (107 * KGCM_TO_NM) / 18.5,         # 107 kg-cm @ 18.5V
     29: ( 24 * KGCM_TO_NM) / 12.0,         #  24 kg-cm @ 12V
}                                           # holding torque in N-m per volt

RPM_TO_RADSEC = 0.104719755                 # 1 RPM is that many rad/sec
DXL_MAX_SPEED_TICK = 1023                   # maximum speed in encoder units
DXL_MODEL_TO_MAX_VELOCITY = \
{
    113: ( 54 * RPM_TO_RADSEC) / 12.0,      #  54 RPM @ 12V
    116: ( 78 * RPM_TO_RADSEC) / 12.0,      #  78 RPM @ 12V
    117: ( 85 * RPM_TO_RADSEC) / 18.5,      #  85 RPM @ 18.5V
     12: ( 59 * RPM_TO_RADSEC) / 12.0,      #  59 RPM @ 12V
     18: ( 97 * RPM_TO_RADSEC) / 12.0,      #  97 RPM @ 12V
     10: ( 54 * RPM_TO_RADSEC) / 12.0,      #  54 RPM @ 12V
     24: (126 * RPM_TO_RADSEC) / 12.0,      # 126 RPM @ 12V
     28: ( 85 * RPM_TO_RADSEC) / 18.5,      #  85 RPM @ 18.5V
     64: ( 64 * RPM_TO_RADSEC) / 18.5,      #  64 RPM @ 18.5V
    107: ( 91 * RPM_TO_RADSEC) / 18.5,      #  91 RPM @ 18.5V
     29: ( 54 * RPM_TO_RADSEC) / 12.0,      #  85 RPM @ 18.5V
}                                           # maximum velocity rad/sec per volt

