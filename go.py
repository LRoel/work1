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
import math
import serial
import numpy as np

from struct import *
from work1.msg import comb



flag_arrived = 0


tolerance_arrived = 1
r_map = 4
c_map = 5
x_min_map = 0-tolerance_arrived
x_max_map = 80+tolerance_arrived
y_min_map = 0-tolerance_arrived
y_max_map = 60+tolerance_arrived


def MapMade():
    map = np.zeros((4, 5, 2))

    map[0, 0, 0] = 0
    map[0, 0, 1] = 0
    map[0, 1, 0] = 20
    map[0, 1, 1] = 0
    map[0, 2, 0] = 40
    map[0, 2, 1] = 0
    map[0, 3, 0] = 60
    map[0, 3, 1] = 0
    map[0, 4, 0] = 80
    map[0, 4, 1] = 0

    map[1, 0, 0] = 0
    map[1, 0, 1] = 20
    map[1, 1, 0] = 20
    map[1, 1, 1] = 20
    map[1, 2, 0] = 40
    map[1, 2, 1] = 20
    map[1, 3, 0] = 60
    map[1, 3, 1] = 20
    map[1, 4, 0] = 80
    map[1, 4, 1] = 20

    map[2, 0, 0] = 0
    map[2, 0, 1] = 40
    map[2, 1, 0] = 20
    map[2, 1, 1] = 40
    map[2, 2, 0] = 40
    map[2, 2, 1] = 40
    map[2, 3, 0] = 60
    map[2, 3, 1] = 40
    map[2, 4, 0] = 80
    map[2, 4, 1] = 40

    map[3, 0, 0] = 0
    map[3, 0, 1] = 60
    map[3, 1, 0] = 20
    map[3, 1, 1] = 60
    map[3, 2, 0] = 40
    map[3, 2, 1] = 60
    map[3, 3, 0] = 60
    map[3, 3, 1] = 60
    map[3, 4, 0] = 80
    map[3, 4, 1] = 60

    return map

def GetVecAngle(x, y):
    r_xy = math.sqrt(x * x + y * y)
    if (y >= 0):
        VecAngle = math.acos(x / r_xy)
    else:
        VecAngle = 2 * math.pi - math.acos(x / r_xy)
    return VecAngle


def CalDriveAngle(x_c, y_c, x_s, y_s, x_e, y_e, r):

    if (not ((x_e - x_s) == 0)):
        k = (y_e - y_s) / (x_e - x_s)
        temp = y_s - k * x_s - y_c


        bb_4ac = (2 * temp * k - 2 * x_c) ** 2 - 4 * (1 + k * k) * (temp ** 2 + x_c ** 2 - r ** 2)
        if (bb_4ac < 0):
            DriveAngle = 10
            return DriveAngle
        else:

            x1 = (-(2 * temp * k - 2 * x_c) + math.sqrt(bb_4ac)) / 2 / (1 + k * k)
            y1 = k * (x1 - x_s) + y_s
            x2 = (-(2 * temp * k - 2 * x_c) - math.sqrt(bb_4ac)) / 2 / (1 + k * k)
            y2 = k * (x2 - x_s) + y_s
    else:
        con = x_e
        if (con < x_c - r or con > x_c + r):

            DriveAngle = 10
            return DriveAngle
        else:

            x1 = x_e
            y1 = y_c + math.sqrt(r ** 2 - (x1 - x_c) ** 2)
            x2 = x_e
            y2 = y_c - math.sqrt(r ** 2 - (x1 - x_c) ** 2)

    d_1e = (x1 - x_e) ** 2 + (y1 - y_e) ** 2
    d_2e = (x2 - x_e) ** 2 + (y2 - y_e) ** 2
    if (d_1e > d_2e):
        x_next = x2
        y_next = y2
    else:
        x_next = x1
        y_next = y1

    x_cn = x_next - x_c
    y_cn = y_next - y_c
    angle_cn = GetVecAngle(x_cn, y_cn)

    DriveAngle = angle_cn

    return DriveAngle



def Get_StartEnd(row1, colum1, row2, colum2, flag_pos, row_target, colum_target):

    if (flag_pos == 1):
        d_mat1 = abs(row1 - row_target) + abs(colum1 - colum_target)
        d_mat2 = abs(row2 - row_target) + abs(colum2 - colum_target)
        if (d_mat1 > d_mat2):
            r_start = row1
            c_start = colum1
            r_end = row2
            c_end = colum2
        else:
            r_start = row2
            c_start = colum2
            r_end = row1
            c_end = colum1
    elif (flag_pos == 0):
        if (row_target != row1):
            if (row_target > row1):
                r_start = row1
                c_start = colum1
                r_end = row1 + 1
                c_end = colum1
            else:
                r_start = row1
                c_start = colum1
                r_end = row1 - 1
                c_end = colum1
        else:
            if (colum_target > colum1):
                r_start = row1
                c_start = colum1
                r_end = row1
                c_end = colum1 + 1
            else:
                r_start = row1
                c_start = colum1
                r_end = row1
                c_end = colum1 - 1
    # elif( flag_pos ==2 ):
    return (r_start, c_start, r_end, c_end)



def GetMatPos(x_robot, y_robot, mat_map):

    global tolerance_arrived
    global r_map
    global c_map
    global x_min_map
    global x_max_map
    global y_min_map
    global y_max_map

    if ((x_robot <= x_min_map) or (x_robot >= x_max_map) or (y_robot <= y_min_map) or (y_robot >= y_max_map)):
        row1 = 1
        colum1 = 1
        row2 = 2
        colum2 = 1
        flag_pos = 4
        return (row1, colum1, row2, colum2, flag_pos)

    d_r_closest = abs(mat_map[0, 0, 1] - y_robot)
    r_closest = 1
    r_closer = 2
    n = 2
    while (n <= r_map):
        if (abs(mat_map[n - 1, 0, 1] - y_robot) < d_r_closest):
            r_closer = r_closest
            r_closest = n
            d_r_closest = abs(mat_map[n - 1, 0, 1] - y_robot)
        elif (abs(mat_map[n - 1, 0, 1] - y_robot) < abs(mat_map[r_closer - 1, 0, 1] - y_robot)):
            r_closer = n
        n = n + 1


    d_c_closest = abs(mat_map[0, 0, 0] - x_robot)
    c_closest = 1
    c_closer = 2
    n = 2
    while (n <= r_map):
        if (abs(mat_map[0, n - 1, 0] - x_robot) < d_c_closest):
            c_closer = c_closest
            c_closest = n
            d_c_closest = abs(mat_map[0, n - 1, 0] - x_robot)
        elif (abs(mat_map[0, n - 1, 0] - x_robot) < abs(mat_map[1, c_closer - 1, 0] - x_robot)):
            c_closer = n
        n = n + 1

    if ((d_r_closest < tolerance_arrived) and (
        d_c_closest < tolerance_arrived)):
        row1 = r_closest
        colum1 = c_closest
        row2 = r_closest
        colum2 = c_closest
        flag_pos = 0
    elif (d_r_closest < tolerance_arrived):
        row1 = r_closest
        colum1 = c_closer
        row2 = r_closest
        colum2 = c_closest
        flag_pos = 1
    elif (d_c_closest < tolerance_arrived):
        row1 = r_closer
        colum1 = c_closest
        row2 = r_closest
        colum2 = c_closest
        flag_pos = 1
    else:
        if (d_r_closest < d_c_closest):
            row1 = r_closer
            colum1 = c_closest
            row2 = r_closest
            colum2 = c_closest
        else:
            row1 = r_closest
            colum1 = c_closer
            row2 = r_closest
            colum2 = c_closest
        flag_pos = 2

    return (row1, colum1, row2, colum2, flag_pos)


def LL2xy(longitude_0,latitude_0,longitude_1,latitude_1):


    e=1.0/298.3
    R_e=6378254
    Rm=R_e*(1-e*e)/( 1-e*e*math.sin(latitude_0)*math.sin(latitude_0) )**(3.0/2)
    Rn=R_e/( 1-e*e*math.sin(latitude_0)*math.sin(latitude_0) )**(1.0/2)

    lon_1_0 = longitude_1 - longitude_0
    lat_1_0 = latitude_1 - latitude_0

    x_0_1 = Rn*math.cos(latitude_0)*lon_1_0
    y_0_1 = Rm*lat_1_0

    return (x_0_1,y_0_1)


def go(x_robot,y_robot,row_target,colum_target,map):


    global flag_arrived
    r = 2

    rr_robot_target = (map[row_target-1, colum_target-1,0] - x_robot) ** 2 + (map[row_target, colum_target,1] - y_robot) ** 2
    if (rr_robot_target < tolerance_arrived ** 2):
        rospy.loginfo("arrived!")
        flag_arrived = 1

    (row1, colum1, row2, colum2, flag_pos) = GetMatPos(x_robot, y_robot, map)

    if (flag_pos < 2):

        (r_start, c_start, r_end, c_end) = Get_StartEnd(row1, colum1, row2, colum2, flag_pos, row_target, colum_target)

        while (1):
            DriveAngle = CalDriveAngle(x_robot, y_robot, map[r_start - 1, c_start - 1, 0],
                                       map[r_start - 1, c_start - 1, 1],
                                       map[r_end - 1, c_end - 1, 0], map[r_end - 1, c_end - 1, 1], r)
            if (DriveAngle > 9):
                r = r + 1
            else:
                r = 2
                break

    else:

        while (1):
            DriveAngle = CalDriveAngle(x_robot, y_robot, map[row1 - 1, colum1 - 1, 0], map[row1 - 1, colum1 - 1, 1],
                                       map[row2 - 1, colum2 - 1, 0], map[row2 - 1, colum2 - 1, 1], r)
            if (DriveAngle > 9):
                r = r + 1
            else:
                r = 1
                break

    return DriveAngle


def serial32(port,linear,angular,am_angular):

    linear_h = linear >> 8
    linear_l = linear & 0xff
    angular_h = angular >> 8
    angular_l = angular & 0xff
    am_angular_h = am_angular >> 8
    am_angular_l = am_angular & 0xff

    cs = linear_h + linear_l + angular_h + angular_l + am_angular_h + am_angular_l
    cs = cs & 0xff

    # buffer = pack('9B',85,linear_h,linear_l,angular_h,angular_l,am_angular_h,am_angular_l,0,cs)
    # test = pack('>bhhhbb',85,linear,angular,am_anguler,0,cs)
    buffer = pack('>bh2H2B', 85, linear, angular, am_angular, 0, cs)

    ser = serial.Serial(port,115200)
    ser.write(buffer)

def comb_callback(combMsg):

    global flag_arrived
    port = '/dev/ttyUSB0'

    if flag_arrived == 0:
        map = MapMade()



        row_target = 1
        colum_target = 2
        Lattitude = combMsg.Lattitude
        Longitude = combMsg.Longitude
        Heading = combMsg.Heading


        (x_robot,y_robot) = LL2xy(map[0,0,0],map[0,0,1],Lattitude,Longitude)
        # x_robot = combMsg.Pitch
        # y_robot = combMsg.Roll
        DriveAngle = go(x_robot, y_robot, row_target, colum_target, map)

        linear = 50
        angular = int(Heading*10)
        am_angular = int(DriveAngle*180*10/math.pi)

        #serial32(port, linear, angular, am_angular)
	serial32(port, linear, 1000, 100)

        rospy.loginfo(am_angular)
        rospy.loginfo(angular)
    else:
        rospy.loginfo("!!!!")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('go_node', anonymous=True)

    rospy.Subscriber('comb', comb, comb_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
