#!/usr/bin/env python2

from __future__ import division

PKG = 'swam'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_test_common import MavrosCommon
from pymavlink import mavutil
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MavrosSendPos(MavrosCommon):
    def __init__(self):
        MavrosCommon.__init__(self)

        self.radius = 0.1
        self.pos = PoseStamped()
        self.pos_vision_pos_pub = rospy.Publisher(
            'mavros/vision_pose/pose', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_vision_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def send_vision_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_vision_pos_pub.publish(self.pos)
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

    def run(self, x, y, z, yaw, timeout):
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        current_yaw = euler_from_quaternion([self.local_position.pose.orientation.x,
                                            self.local_position.pose.orientation.y,
                                            self.local_position.pose.orientation.z,
                                            self.local_position.pose.orientation.w])[2]
        rospy.loginfo(
            "vision pose | x: {0}, y: {1}, z: {2} yaw: {3} | current pose x: {4:.2f}, y: {5:.2f}, z: {6:.2f}, yaw:{7:.2f}".
            format(x, y, z, yaw, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z, current_yaw))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        rate.sleep()

        # reached = False
        # # for i in xrange(timeout * loop_freq):
        #     try:
        #         rate.sleep()
        #     except rospy.ROSException as e:
        #         self.fail(e)

# import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
import json

class PozyxMqttClient(mqtt.Client):
    def __init__(self):
        mqtt.Client.__init__(self)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def start(self):
        self.connect("localhost", 1883, 60)
        self.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code {0:s}".format(str(rc)))
        self.subscribe("tags")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            print("Unexpected disconnection.")

    def on_message(self, client, userdata, message):
        # global x, y, z
        payload = json.loads(message.payload)[0]
        # print(payload['tagId'])
        if payload['tagId'] == '26394':
        # if payload['tagId'] == '26478':
            self.x = payload['data']['coordinates']['x'] / 1000.0
            self.y = payload['data']['coordinates']['y'] / 1000.0
            self.z = payload['data']['coordinates']['z'] / 1000.0
            self.roll = payload['data']['orientation']['roll']
            self.pitch = payload['data']['orientation']['pitch']
            self.yaw = payload['data']['orientation']['yaw']

    def get_position(self):
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)

if __name__ == '__main__':
    rospy.init_node('offboard_sendpos', anonymous=True)

    mav = MavrosSendPos()
    pozyx = PozyxMqttClient()
    pozyx.start()

    try:
        while not rospy.is_shutdown():
            (x, y, z, roll, pitch, yaw) = pozyx.get_position()
            mav.run(x, y, z, yaw, 1)
            #mav.run(0, 0, i/100.0, 1)
    except rospy.ROSInterruptException:
        pass
