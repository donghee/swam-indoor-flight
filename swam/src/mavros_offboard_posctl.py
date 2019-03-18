#!/usr/bin/env python2
#***************************************************************************
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#***************************************************************************/
# Donghee Park <dongheepark@gmail.com>
# base code on Andreas Antener <andreas@uaventure.com>
#
# The shebang of this file is currently Python2 because some
# dependencies such as pymavlink don't play well with Python3 yet.
from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_common import MavrosCommon
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler

# heartbeat and statustext
from mavros_msgs.msg import Mavlink, StatusText
from mavros import mavlink

class MavrosOffboardPosctl(MavrosCommon):
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.
    For the test to be successful it needs to reach all setpoints in a certain time.
    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """

    def __init__(self):
        MavrosCommon.__init__(self)

        self.pos = PoseStamped()
        #self.radius = 0.05
        self.radius = 0.1

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=3)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

        # send hearbeat
        self.ids = None
        self.mavlink_sub = rospy.Subscriber('mavlink/from', Mavlink, self.mavlink_from_callback)
        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
        self.heartbeat_thread = Thread(target=self.send_heartbeat, args=())
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

        # get statustext
        # self.statustext_sub = rospy.Subscriber('mavros/statustext/recv', StatusText, self.statustext_callback)

    def mavlink_from_callback(self, data):
        self.ids = (data.sysid, data.compid)
        self.mavlink_sub.unregister()

    def statustext_callback(self, data):
        rospy.loginfo('STATUSTEXT: {0} {1}'.format(data.severity, data.text))

    def send_heartbeat(self):
        rate = rospy.Rate(2)  # Hz
        while not rospy.is_shutdown():
            try:
                if self.ids is not None:
                    self.heartbeat_msg = mavutil.mavlink.MAVLink_heartbeat_message(
                        mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
                    # rospy.loginfo('ID: {0}, {1}'.format(self.ids[0], self.ids[1]))
                    # self.heartbeat_msg.pack(mavutil.mavlink.MAVLink('', self.ids[0], self.ids[1])) # fcu
                    self.heartbeat_msg.pack(mavutil.mavlink.MAVLink('', 1, 1)) # simulator
                    self.heartbeat_ros = mavlink.convert_to_rosmsg(self.heartbeat_msg)
                    self.mavlink_pub.publish(self.heartbeat_ros)
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
                                                                                

    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 90  # North for ENU
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        if reached == False:
            rospy.loginfo(
                "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
                format(self.local_position.pose.position.x,
                       self.local_position.pose.position.y,
                       self.local_position.pose.position.z, timeout))

    def posctl(self, positions):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.set_mode("AUTO.LAND", 5)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 10)
        self.set_arm(True, 5)

        rospy.loginfo("Run mission")

        for i in xrange(len(positions)):
            #self.reach_position(positions[i][0], positions[i][1], positions[i][2], 30)
            self.reach_position(x, y, positions[i][2], 30)

        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)

    def goal(self, goal_pose):
        # TODO limit
        # rospy.logwarn("Cannot set <= Z setpoint!")
        rospy.loginfo("goal position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z ))
        self.pos.pose.position.x = goal_pose.pose.position.x
        self.pos.pose.position.y = goal_pose.pose.position.y
        self.pos.pose.position.z = goal_pose.pose.position.z
        
        yaw_degrees = 90  # North for ENU
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

    def ready_to_offboard(self):
        # on gazebo
        # self.log_topic_vars()
        # self.set_mode("OFFBOARD", 10)
        # self.set_arm(True, 5)

        # on pixracer
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.set_mode("AUTO.LAND", 5)
        self.set_arm(True, 5)
        self.set_mode("AUTO.TAKEOFF", 5)
        rospy.sleep(1) # wait for AUTO.LOITOR
        
        self.log_topic_vars()
        self.set_mode("OFFBOARD", 10)

        rospy.loginfo("Ready to OFFBOARD")
        

    def play(self):
        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.set_mode("AUTO.LAND", 5)
        self.set_arm(True, 5)
        self.set_mode("AUTO.TAKEOFF", 5)
        rospy.sleep(1) # wait for AUTO.LOITOR
        
        self.log_topic_vars()
        self.set_mode("OFFBOARD", 10)

        rospy.loginfo("Start Play Goal")

    def stop(self):
        rospy.loginfo("Stop Play")
        self.set_mode("AUTO.LAND", 5)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   45, 0)
        self.set_arm(False, 5)

from std_srvs.srv import Trigger, TriggerResponse

def swam_ready_response(request):
    mav.ready_to_offboard()
    return TriggerResponse(success=True, message='Good')

def swam_start_response(request):
    mav.play()
    return TriggerResponse(success=True, message='Good')

def swam_stop_response(request):
    mav.stop()
    return TriggerResponse(success=True, message='Good')

if __name__ == '__main__':
    rospy.init_node('offboard_posctl', anonymous=True)
    mav = MavrosOffboardPosctl()

    swam_ready_service = rospy.Service('swam/ready', Trigger, swam_ready_response) # r
    swam_play_service = rospy.Service('swam/play', Trigger, swam_start_response) # Here
    swam_stop_service = rospy.Service('swam/stop', Trigger, swam_stop_response) # land
    swam_goal_sub = rospy.Subscriber('swam/goal', PoseStamped, mav.goal) # Set

    rospy.spin()

    mav = MavrosOffboardPosctl()
    #positions = ((1, 0, 1), (0, 1, 1), (1, 0, 1), (0, 1, 1), (1, 0, 1), (0, 1, 1))
    #mav.posctl(positions)
