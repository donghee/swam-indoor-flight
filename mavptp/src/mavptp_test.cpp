#include "ros/ros.h"
#include <mavros_msgs/mavlink_convert.h>
#include <mavconn/interface.h>
#include <iostream>
#include <mavlink/v2.0/mavlink_helpers.h>

void packMavlinkMessage(const mavlink::Message& mavMsg, mavros_msgs::Mavlink &rosMsg)
{
	mavlink::mavlink_message_t msg;
	mavlink::MsgMap map(msg);
	mavMsg.serialize(map);
	auto mi = mavMsg.get_message_info();

	mavlink::mavlink_status_t *status = mavlink::mavlink_get_channel_status(mavlink::MAVLINK_COMM_0);
	status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
	mavlink::mavlink_finalize_message_buffer(&msg, 1, 1, status, mi.min_length, mi.length, mi.crc_extra);

	mavros_msgs::mavlink::convert(msg, rosMsg);
}

void mavlinkCallback(const mavros_msgs::Mavlink& msg)
{
	mavlink::mavlink_message_t mavMsg;
	mavros_msgs::mavlink::convert(msg, mavMsg);
        if(mavMsg.msgid == mavlink::common::msg::HEARTBEAT::MSG_ID)
	{
          ROS_INFO("%s", "Heartbeat");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mavptp-node");
	ros::NodeHandle n;
	ros::Publisher mavlink_pub_ = n.advertise<mavros_msgs::Mavlink>("/uav1/mavlink/to", 1000);
	ros::Subscriber mavlink_sub_ = n.subscribe("/uav1/mavlink/from", 1000, mavlinkCallback);
	ros::Rate loop_rate(10);

	ROS_INFO("%s", "Talker started");
	while (ros::ok())
	{
		//pub.publish(mavrosMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}


        return 0;
}
