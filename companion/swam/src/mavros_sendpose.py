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
            'mavros/vision_pose/pose', PoseStamped, queue_size=10)        

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_vision_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def send_vision_pos(self):
        rate = rospy.Rate(50)  # Hz
        self.pos.header = Header()
        # self.pos.header.frame_id = "base_footprint"
        # self.pos.header.frame_id = "fcu"
        self.pos.header.frame_id = "local_origin"                

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

    def run(self, x, y, z, roll, pitch, yaw, timeout):
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        (current_roll, current_pitch, current_yaw) = euler_from_quaternion([self.local_position.pose.orientation.x,
                                                                          self.local_position.pose.orientation.y,
                                                                          self.local_position.pose.orientation.z,
                                                                          self.local_position.pose.orientation.w])
        # rospy.loginfo(
        #     "vision pose ENU x: {0}, y: {1}, z: {2} roll: {3}, pitch: {4:.2f}, yaw: {5:.2f} | current pose ENU x: {6:.2f}, y: {7:.2f}, z: {8:.2f}, roll: {9:.2f}, pitch: {10:.2f}, yaw:{11:.2f}".
        #     format(x, y, z,
        #            roll,
        #            pitch,
        #            yaw,
        #            self.local_position.pose.position.x,
        #            self.local_position.pose.position.y,
        #            self.local_position.pose.position.z,
        #            current_roll,
        #            current_pitch,
        #            current_yaw))

        quaternion = quaternion_from_euler(roll, pitch, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # reached = False
        # # for i in xrange(timeout * loop_freq):
        #     try:
        #         rate.sleep()
        #     except rospy.ROSException as e:
        #         self.fail(e)


import pypozyx

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, POZYX_FLASH_REGS, DeviceCoordinates, SingleRegister, NetworkID, DeviceList)

from pypozyx.definitions.registers import *

class PozyxSerialClient():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.remote_id = None
        
        self.tag_ids = [None, 0xA001, 0xA002, 0xA003]
        # self.tag_ids = [None, 0x671a, 0x676e]

	# BARIBARILAB
        #self.anchors = [DeviceCoordinates(0x6714, 1, Coordinates(0, 0, 1500)),
        #                DeviceCoordinates(0x6711, 1, Coordinates(785, 2900, 1200)),
        #                DeviceCoordinates(0x6717, 1, Coordinates(5416, 3670, 2000)),
        #                DeviceCoordinates(0x675b, 1, Coordinates(5000, 0, 1800)),
        #                DeviceCoordinates(0x6743, 1, Coordinates(3140, 4252, 900)),
        #                DeviceCoordinates(0x671b, 1, Coordinates(6050, 1400, 1000))]

	# ETRI EMBEDDED LAB
        self.anchors = [DeviceCoordinates(0x6714, 1, Coordinates(0, 0, 2500)),
                        DeviceCoordinates(0x6711, 1, Coordinates(1400, 4650, 1500)),
                        DeviceCoordinates(0x6717, 1, Coordinates(5600, 4600, 1950)),
                        DeviceCoordinates(0x675b, 1, Coordinates(6000, 0, 2000)),
                        DeviceCoordinates(0x6743, 1, Coordinates(4000, 6050, 2650)),
                        DeviceCoordinates(0x671b, 1, Coordinates(6950, 1500, 900))]

    def setAnchorsManual(self, save_to_flash=False):
        for tag_id in self.tag_ids:
            status = self.pozyx.clearDevices(tag_id)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag_id)
            
            if len(self.anchors) > 4:
                status &= self.pozyx.setSelectionOfAnchors(pypozyx.PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors), remote_id=tag_id)
                
            if save_to_flash:
                self.pozyx.saveAnchorIds(tag_id)
                self.pozyx.saveRegisters([pypozyx.PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], tag_id)

            self.printPublishConfigurationResult(status, tag_id)
    
    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % ("0x%0.4x"%tag_id))
        else:
            self.printPublishErrorCode("configuration", tag_id)

    def printPublishErrorCode(self, operation, network_id):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if not(status == POZYX_SUCCESS):
            print("Error %s on ID %s, %s" % (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            
    def setNewId(self, new_id, remote_id=None):
        try:
            rospy.loginfo("Setting the Pozyx ID to 0x%0.4x" % new_id)
            self.pozyx.setNetworkId(new_id, remote_id)
            if self.pozyx.saveConfiguration(POZYX_FLASH_REGS, [POZYX_NETWORK_ID], remote_id) == POZYX_SUCCESS:
                rospy.loginfo("Saving new ID successful! Resetting system...")
                if self.pozyx.resetSystem(remote_id) == POZYX_SUCCESS:
                    rospy.loginfo("Done")
        except rospy.ROSException as e:            
            rospy.loginfo(e)

    def printPublishConfigurationResultMore(self, remote_id):
        if not(remote_id is None):
            print("----")
            print("Pozyx ID: 0x%0.4x" % remote_id)
        list_size = SingleRegister()
        self.pozyx.getDeviceListSize(list_size, remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)
            
        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))

    def start(self):
        try:
            print(pypozyx.get_serial_ports()[1].device)
            self.pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[1].device)
            whoami = pypozyx.SingleRegister()
            self.pozyx.getWhoAmI(whoami)
            rospy.loginfo("Pozyx WhoAmI {0}".format(whoami)) # will return 0x43

            # self.setNewId(0xA003, None)
            # rospy.sleep(1)
        
            networkid = pypozyx.NetworkID()
            self.pozyx.getNetworkId(networkid)
            rospy.loginfo("Network ID {0}".format(networkid))
            self.pozyx.setPositionFilter(pypozyx.PozyxConstants.FILTER_TYPE_NONE, 0)
            # self.pozyx.setPositionFilter(pypozyx.PozyxConstants.FILTER_TYPE_MOVING_MEDIAN, 10)
        
            self.setAnchorsManual(save_to_flash=False)
            #self.setAnchorsManual(save_to_flash=True)

            self.pozyx.printDeviceInfo()
            
        except rospy.ROSException as e:
            rospy.loginfo("Pozyx not connected: {0}", e)
            return        

        self.read_thread = Thread(target=self.read, args=())
        self.read_thread.daemon = True
        self.read_thread.start()
        
        rospy.sleep(3) # wait to get stable pozyx position        

    def read(self):
        rate = rospy.Rate(60)  # Hz

        while not rospy.is_shutdown():
            coords = pypozyx.Coordinates()
            euler_angles = pypozyx.EulerAngles()        
            status = self.pozyx.doPositioning(coords, remote_id=self.remote_id)
            if status == pypozyx.POZYX_SUCCESS:
                self.x = coords.x / 1000.0
                self.y = coords.y / 1000.0
                self.z = coords.z / 1000.0
            
            status = self.pozyx.getEulerAngles_deg(euler_angles, remote_id=self.remote_id)
            if status == pypozyx.POZYX_SUCCESS:
                self.roll = euler_angles.roll * 3.14159 / 180.0
                self.pitch = euler_angles.pitch * 3.14159 / 180.0
                self.yaw = (90-euler_angles.heading) * 3.14159 / 180.0
                if self.yaw < -3.14159:
                    self.yaw = self.yaw + (2 * 3.14159)
                
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
        
    def get_position(self):
        rospy.loginfo(
            "pozyx | x: {0}, y: {1}, z: {2}, roll {3}, pitch: {4}, yaw: {5} ".
            format(self.x, self.y, self.z, self.roll, self.pitch, self.yaw))
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)

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
        #self.connect("dronemap-NUC8i3BEK.local", 1883, 60)
        self.connect("192.168.88.239", 1883, 60)        
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
        # if payload['tagId'] == '26394': # 0x671a
        if payload['tagId'] == '26478': # 0x676e
            self.x = payload['data']['coordinates']['x'] / 1000.0
            self.y = payload['data']['coordinates']['y'] / 1000.0
            self.z = payload['data']['coordinates']['z'] / 1000.0
            self.roll = payload['data']['orientation']['roll'] * 3.14159 / 180.0
            self.pitch = payload['data']['orientation']['pitch'] * 3.14159 / 180.0
            self.yaw_degree = payload['data']['orientation']['yaw'] 
            self.yaw = (90-self.yaw_degree) * 3.14159 / 180.0
            if self.yaw < -3.14159:
                self.yaw = self.yaw + (2 * 3.14159)
            

    def get_position(self):
        return (self.x, self.y, self.z, self.roll, self.pitch, self.yaw)

if __name__ == '__main__':
    rospy.init_node('send_vision_pose', anonymous=True)

    # pozyx = PozyxMqttClient()
    pozyx = PozyxSerialClient()    
    pozyx.start()
    
    mav = MavrosSendPos()

    from mavros_set_home import set_home
    # Global position of the origin
    lat = 37.4933566 * 1e7
    lon = 126.8339491 * 1e7
    alt = 200 * 1e3
    # set_home(lat, lon, alt, 0, 0, 0)
        
    loop_freq = 50  # Hz
    rate = rospy.Rate(loop_freq)
    
    try:
        while not rospy.is_shutdown():
            (x, y, z, roll, pitch, yaw) = pozyx.get_position()
            mav.run(x, y, z, -roll, pitch, yaw, 1) # pozyx ENU
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
