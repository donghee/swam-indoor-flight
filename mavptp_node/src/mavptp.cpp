#include <iostream>
#include <fstream>
#include <math.h> 
#include <string>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "mavros_msgs/State.h"

#include <mavros_msgs/mavlink_convert.h>
#include <mavconn/interface.h>
#include <iostream>
#include <mavlink/v2.0/mavlink_helpers.h>

using mavlink::mavlink_message_t;

#include <mavlink/v2.0/common/mavlink.h>

#define PTP_SYNC 0x00
#define PTP_FOLLOW_UP 0x08
#define PTP_DELAY_REQUEST 0x01
#define PTP_DELAY_RESPONSE 0x09

#define NSEC_PER_SEC          1000000000L /* Seconds */
#define NSEC_PER_MSEC         1000000L /* Seconds */

class MavPtpNode
{
public:
  MavPtpNode();
  ~MavPtpNode();

  void run();
private:

  ros::NodeHandle n;

  int sysid  = 0;
  int compid  = 0;

  // Subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber mavlink_sub_;
  ros::Publisher mavlink_pub_;

  //ros::Rate loop_rate(10);

  // Private variables
  bool currently_armed_ = false;

  void mavlinkMessageCallback(const mavros_msgs::Mavlink& msg);
  void positionCallback(const geometry_msgs::PoseStamped msg);                  // position and orientation from PX4

  void printPose(geometry_msgs::PoseStamped pos);
  void send_message_ptp_timesync(int sysid, int compid, uint8_t seq,
			       uint8_t msg_type, uint64_t tv_sec, uint64_t tv_nsec);
  void handle_message_ptp_timesync(mavlink_message_t *msg);
};


void MavPtpNode::send_message_ptp_timesync(int sysid, int compid, uint8_t seq,
			       uint8_t msg_type, uint64_t tv_sec, uint64_t tv_nsec)
{
	mavlink_ptp_timesync_t tsync;
	mavlink_message_t msg = {};

	tsync.seq = seq;
	tsync.msg_type = msg_type;
	tsync.tv_sec = tv_sec;
	tsync.tv_nsec = tv_nsec;
	mavlink_msg_ptp_timesync_encode(sysid, compid, &msg, &tsync);

        mavros_msgs::Mavlink mavrosMsg;
	mavros_msgs::mavlink::convert(msg, mavrosMsg);
	mavlink_pub_.publish(mavrosMsg);

	//serial1->write_message(msg);
	printf("[PTP TIMESYNC] S seq: %d, msg_type: %d, tv_sec: %llu, tv_nsec: %llu \n", tsync.seq, tsync.msg_type, tsync.tv_sec, tsync.tv_nsec);
}


// buf needs to store 30 characters
int timespec2str(char *buf, uint len, struct timespec *ts)
{
	int ret;
	struct tm t;

	tzset();

	if (gmtime_r(&(ts->tv_sec), &t) == NULL) {
		// if (localtime_r(&(ts->tv_sec), &t) == NULL) {
		return 1;
	}

	ret = strftime(buf, len, "%F %T", &t);

	if (ret == 0) {
		return 2;
	}

	len -= ret - 1;

	ret = snprintf(&buf[strlen(buf)], len, ".%09ld", ts->tv_nsec);

	if (ret >= len) {
		return 3;
	}

	return 0;
}

void MavPtpNode::handle_message_ptp_timesync(mavlink_message_t *msg)
{
	mavlink_ptp_timesync_t tsync = {};
	mavlink_msg_ptp_timesync_decode(msg, &tsync);
	printf("[PTP TIMESYNC] R seq: %d, msg_type: %d, tv_sec: %llu, tv_nsec: %llu \n", tsync.seq, tsync.msg_type,
	        tsync.tv_sec, tsync.tv_nsec);

	// DELAY_REQUEST
	if (tsync.msg_type == PTP_DELAY_REQUEST) {
		struct timespec t4 = {};
		clock_gettime(CLOCK_REALTIME, &t4);

		printf("[PTP TIMESYNC] DELAY_REQUEST\n");

		// DELAY_RESPONSE
		send_message_ptp_timesync(sysid,
					  compid, 4, PTP_DELAY_RESPONSE,
					  t4.tv_sec, t4.tv_nsec);

		// struct timespec current_time = {};
		// clock_gettime(CLOCK_REALTIME, &current_time);
		printf("[PTP TIMESYNC] current_time: %lu.%04lu sec\n", (unsigned long)t4.tv_sec,
		       (unsigned long) t4.tv_nsec / 100000);

		const uint TIME_FMT = strlen("2012-12-31 12:59:59.123456789") + 1;
		char timestr[TIME_FMT];
		timespec2str(timestr, sizeof(timestr), &t4);
		printf("TIME: %s\n", timestr);
	}
}



void MavPtpNode::mavlinkMessageCallback(const mavros_msgs::Mavlink& msg)
{
	mavlink_message_t mavMsg;
	mavros_msgs::mavlink::convert(msg, mavMsg);


	mavlink_heartbeat_t hb;

	if(mavMsg.msgid == mavlink::common::msg::HEARTBEAT::MSG_ID)
	{
		sysid = mavMsg.sysid;
		compid = mavMsg.compid;

		ROS_INFO("sysid: %d, compid: %d", sysid, compid);
		ROS_INFO("%s", "Heartbeat");
/*
		mavlink_msg_heartbeat_decode(&mavMsg, &hb);
		ROS_INFO("%d", hb.custom_mode);
		ROS_INFO("%d", hb.type);
		ROS_INFO("%d", hb.autopilot);
		ROS_INFO("%d", hb.base_mode);
		ROS_INFO("%d", hb.system_status);
		ROS_INFO("%d", hb.mavlink_version);
*/
	}
	if(mavMsg.msgid == mavlink::common::msg::PTP_TIMESYNC::MSG_ID)
	{
		handle_message_ptp_timesync(&mavMsg);
	}
}

MavPtpNode::MavPtpNode()
{
    n = ros::NodeHandle("~");

    std::string ns = ros::this_node::getNamespace();

    // Initialize all topics
    mavlink_pub_ = n.advertise<mavros_msgs::Mavlink>(ns + "/mavlink/to", 1000);
    mavlink_sub_ = n.subscribe(ns + "/mavlink/from", 1000, &MavPtpNode::mavlinkMessageCallback, this);
}


MavPtpNode::~MavPtpNode() {}

void MavPtpNode::printPose(geometry_msgs::PoseStamped pos)
{
    printf("Pose\n");
    printf("Position: x=%.3f y=%.3f z=%.3f\n", pos.pose.position.x, pos.pose.position.y, pos.pose.position.z);
    printf("Orientation: x=%.3f y=%.3f z=%.3f w=%.3f\n", pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z, pos.pose.orientation.w);
    printf("-------------------------------------\n");
}

void MavPtpNode::positionCallback(const geometry_msgs::PoseStamped msg) {

   printPose(msg);
}


void MavPtpNode::run() {
	int rate = 1;
	ros::Rate r(rate);

	while(n.ok())
	{
		if (sysid && compid) {
			// SYNC
			struct timespec t1 = {};
			clock_gettime(CLOCK_REALTIME, &t1);

			printf("[PTP TIMESYNC] SYNC\n");
			send_message_ptp_timesync(sysid,
					compid, 0, PTP_SYNC,
					0, 0);
			usleep(1000);  // 1ms

			// FOLLOW_UP 
			printf("[PTP TIMESYNC] FOLLOW UP\n");
			send_message_ptp_timesync(sysid,
					compid, 1, PTP_FOLLOW_UP,
					t1.tv_sec, t1.tv_nsec);
		}

		ros::spinOnce();
		r.sleep();
	}
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "mavptp_node");

  MavPtpNode mavptp_node;
  mavptp_node.run();

  return 0;
}
