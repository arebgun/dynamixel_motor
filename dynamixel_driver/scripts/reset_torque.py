#!/usr/bin/python

import sys
from optparse import OptionParser

import roslib
roslib.load_manifest('dynamixel_driver')

from dynamixel_driver import dynamixel_io

if __name__ == '__main__':
    usage_msg = 'Usage: %prog [options] ID [On|Off]'
    desc_msg = 'Turns the torque of specified Dynamixel servo motor on or off.'
    epi_msg = 'Example: %s --port=/dev/ttyUSB1 --baud=57600 1 Off' % sys.argv[0]
    
    parser = OptionParser(usage=usage_msg, description=desc_msg, epilog=epi_msg)
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 3:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    motor_id = int(args[1])
    torque_on = args[2]

    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        print 'Turning torque %s for motor %d' % (torque_on, motor_id)
        ret = dxl_io.ping(motor_id)
        if ret:
            from dynamixel_driver import dynamixel_const
            if torque_on.lower() == 'off':
                torque_on = False
            elif torque_on.lower() == 'on':
                if not ret[4] & dynamixel_const.DXL_OVERHEATING_ERROR == 0:
                    dxl_io.set_torque_limit(motor_id, 1023)
                    print "OVERHEATING -> Reset Torque"
                if not ret[4] & dynamixel_const.DXL_OVERLOAD_ERROR == 0:
                    dxl_io.set_torque_limit(motor_id, 1023)
                    print "OVERLOAD -> Reset Torque"
                torque_on = True
            else:
                parser.print_help()
                exit(1)
            ret = dxl_io.set_torque_enabled(motor_id, torque_on)
            if not ret[4] & dynamixel_const.DXL_OVERHEATING_ERROR == 0:
                print "OVERHEATING"
                exit (1)
            if not ret[4] & dynamixel_const.DXL_OVERLOAD_ERROR == 0:
                print "OVERLOAD"
                exit (1)
            print "done."
        else:
            print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id
            exit (1)

