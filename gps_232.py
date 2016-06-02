#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import serial
import sys
import string
import operator



from sensor_msgs.msg import NavSatFix
#print("import Nav")
from work1.msg import comb
#print("import comb")

def checksum(nmea_str0):
    nmea_str = nmea_str0[0:-5]
    return reduce(operator.xor, map(ord, nmea_str), 0)


port = '/dev/ttyUSB1'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)


def talker():

    ser.flush()

    pub = rospy.Publisher('gps', NavSatFix, queue_size=100)
    pub_n = rospy.Publisher('comb', comb, queue_size=100)
    rospy.init_node('gps_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    GPSmsg = NavSatFix()
    combMsg = comb()

    seq = 0

    while not rospy.is_shutdown():

        if (ser.read() != '$'):
            continue
        line0 = ser.readline()
        cs = line0[-4:-2]
        cs1 = int(cs, 16)
        cs2 = checksum(line0)
        print("cs1,cs2", cs1, cs2)
        if (cs1 != cs2):
            continue
        line = line0.replace("GPFPD,", "")
        line = line.replace("\r\n", "")
        if ("\x00" in line):
            continue
        if (not string.find('*', line)):
            continue
        line = line.replace("*", ",")

        words = string.split(line, ",")

        GPSWeek = string.atoi(words[0])
        GPSTime = string.atof(words[1])
        Heading = string.atof(words[2])
        Pitch = string.atof(words[3])
        Roll = string.atof(words[4])
        Lattitude = string.atof(words[5])
        Longitude = string.atof(words[6])
        Altitude = string.atof(words[7])
        Ve = string.atof(words[8])
        Vn = string.atof(words[9])
        Vu = string.atof(words[10])
        Baseline = string.atof(words[11])
        NSV1 = string.atoi(words[12])
        NSV2 = string.atoi(words[13])
        Status = words[14]

        combMsg.GPSWeek = GPSWeek
        combMsg.GPSTime = GPSTime
        combMsg.Heading = Heading
        combMsg.Pitch = Pitch
        combMsg.Roll = Roll
        combMsg.Lattitude = Lattitude
        combMsg.Longitude = Longitude
        combMsg.Altitude = Altitude
        combMsg.Ve = Ve
        combMsg.Vn = Vn
        combMsg.Vu = Vu
        combMsg.Baseline = Baseline
        combMsg.NSV1 = NSV1
        combMsg.NSV2 = NSV2
        combMsg.Status = Status

        GPSmsg.latitude = Lattitude
        GPSmsg.longitude = Longitude
        GPSmsg.altitude = Altitude
        #GPSmsg.status.status = Status
        GPSmsg.header.stamp = rospy.Time.now()
        GPSmsg.header.frame_id = 'gps_node'
        GPSmsg.header.seq = seq

        #rospy.loginfo(hello_str)
        pub.publish(GPSmsg)
        print("pub GPS")
        pub_n.publish(combMsg)
        print("pub comb")
        seq = seq + 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
