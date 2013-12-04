#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
from optparse import OptionParser

import roslib
roslib.load_manifest('dynamixel_driver')

from dynamixel_driver import dynamixel_io

if __name__ == '__main__':
    parser = OptionParser(usage='Usage: %prog [options]', description='Changes the unique ID of a Dynamixel servo motor.')
    parser.add_option('-p', '--port', metavar='PORT', default='/dev/ttyUSB0',
                      help='motors of specified controllers are connected to PORT [default: %default]')
    parser.add_option('-b', '--baud', metavar='BAUD', type="int", default=1000000,
                      help='connection to serial port will be established at BAUD bps [default: %default]')
    parser.add_option('-f', '--from-id', metavar='FROM_ID', type="int", default=1,
                      help='from id [default: %default]')
    parser.add_option('-t', '--to-id', metavar='TO_ID', type="int", default=7,
                      help='to id [default: %default]')
                      
    (options, args) = parser.parse_args(sys.argv)
    
    if len(args) < 1:
        parser.print_help()
        exit(1)
        
    port = options.port
    baudrate = options.baud
    
    try:
        dxl_io = dynamixel_io.DynamixelIO(port, baudrate)
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
        for idx in map(lambda x : x + options.from_id , range(options.to_id - options.from_id + 1)):
            print 'Scanning %d...' %(idx),
            if dxl_io.ping(idx):
                print 'The motor %d respond to a ping' %(idx)
            else:
                print 'ERROR: The specified motor did not respond to id %d.' % idx
