#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <autoware_msgs/TrafficLight.h>
#include <tf/transform_broadcaster.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/CANInfo.h>
#include <limits.h>

class StopperDistance
{
private:
	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_waypoint_, sub_light_color_, sub_camera_light_color_, sub_temporari_flag_;
	ros::Subscriber sub_obstacle_waypoint_, sub_mobileye_velocity_, sub_temporary_fixed_velocity_;
	ros::Subscriber sub_can502_, sub_can_info_;

	ros::Publisher pub_stopline_distance_, pub_obstacle_offset_;

	double const front_bumper_to_baselink = 6.25 - 1.7;

	double temporary_fixed_velocity_;
	void callbackTemporaryFixedVelocity(const std_msgs::Float64& msg)
	{
		temporary_fixed_velocity_ = msg.data;
	}

	double obstacle_velocity_;
	void callbackMobileyeVelocity(const std_msgs::Float64& msg)
	{
		obstacle_velocity_ = msg.data;
	}

	int light_color_;
	void callbackCameraLightColor(const autoware_msgs::TrafficLight& msg)
	{
		light_color_ = msg.traffic_light;
		std::cout << light_color_ << std::endl;
	}
	void callbackLightColor(const autoware_msgs::TrafficLight& msg)
	{
		light_color_ = msg.traffic_light;
		std::cout << light_color_ << std::endl;
	}

	int temporary_flag_;
	void callbackTemporaryFlag(const std_msgs::Int32& msg)
	{
		temporary_flag_ = msg.data;
	}

	int obstacle_waypoint_;
	//const int obstacle_offset_ = 2;
	void callbackObstacleWaypoint(const std_msgs::Int32& msg)
	{
		obstacle_waypoint_ = msg.data;
	}

	void waypointCallback(const autoware_msgs::Lane& msg)
	{
		autoware_msgs::StopperDistance pubmsg;
		pubmsg.distance = -1;
		pubmsg.send_process = autoware_msgs::StopperDistance::UNKNOWN;
		pubmsg.fixed_velocity = 0;

		double dis = 0;
		for(int i=1; i<msg.waypoints.size(); i++)
		{
			tf::Vector3 v1(msg.waypoints[i-1].pose.pose.position.x,
						   msg.waypoints[i-1].pose.pose.position.y, 0);

			tf::Vector3 v2(msg.waypoints[i].pose.pose.position.x,
						   msg.waypoints[i].pose.pose.position.y, 0);

			dis += tf::tfDistance(v1, v2);

			autoware_msgs::WaypointParam waypoint_param = msg.waypoints[i].waypoint_param;
			if(waypoint_param.signal_stop_line > 0 && light_color_ == 0)
			{
				pubmsg.distance = dis - front_bumper_to_baselink;
				pubmsg.send_process = autoware_msgs::StopperDistance::SIGNAL;
				pubmsg.fixed_velocity = 0;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				break;
			}
			else if(waypoint_param.temporary_stop_line > 0 && temporary_flag_ == 1)
			{
				pubmsg.distance = dis - front_bumper_to_baselink;
				pubmsg.send_process = autoware_msgs::StopperDistance::TEMPORARY_STOPPER;
				pubmsg.fixed_velocity = temporary_fixed_velocity_;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				break;
			}
			else if(obstacle_waypoint_ > 0 && obstacle_waypoint_ == i)
			{
				//pubmsg.distance = dis - (front_bumper_to_baselink + obstacle_offset_);
				double km_h;
				switch(can_read_flag)
				{
				case StopperDistance::CANREAD_LIESSE:
					km_h = can502_.velocity_mps/3.6; break;
				case StopperDistance::CANREAD_PRIUS:
					km_h = can_info_.speed;
				}
				double a = (km_h/2.0 > 3.0) ? km_h/2.0 : 3.0;
				//double obstacle_offset = (a > 4) ? 4 : a;
				double obstacle_offset = 4;
				pubmsg.distance = dis - (front_bumper_to_baselink + obstacle_offset);
				pubmsg.send_process = autoware_msgs::StopperDistance::OBSTACLE;
				pubmsg.fixed_velocity = obstacle_velocity_;
				if(pubmsg.distance < 0) pubmsg.distance = 0;
				break;
			}
		}
		pub_stopline_distance_.publish(pubmsg);
	}

	static const int CANREAD_NOT =0;
	static const int CANREAD_LIESSE =1;
	static const int CANREAD_PRIUS =2;
	autoware_can_msgs::MicroBusCan502 can502_;
	int can_read_flag;
	void callbackCan502(const autoware_can_msgs::MicroBusCan502 &msg)
	{
		can502_ = msg;
		can_read_flag = StopperDistance::CANREAD_LIESSE;
	}

	autoware_can_msgs::CANInfo can_info_;
	void callbackCanInfo(const autoware_can_msgs::CANInfo &msg)
	{
		can_info_ = msg;
		can_read_flag = StopperDistance::CANREAD_PRIUS;
	} 
public:
	StopperDistance(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: obstacle_velocity_(0)
		, temporary_fixed_velocity_(0)
		, light_color_(1)
		, temporary_flag_(0)
		, obstacle_waypoint_(-1)
		, can_read_flag(CANREAD_NOT)
	{
		nh_ = nh;  private_nh_ = p_nh;

		pub_stopline_distance_ = nh_.advertise<autoware_msgs::StopperDistance>("/stopper_distance", 1);
		pub_obstacle_offset_ = nh_.advertise<std_msgs::Float64>("/obstacle_offset", 1);

		sub_waypoint_ = nh_.subscribe("/final_waypoints", 1, &StopperDistance::waypointCallback, this);
		sub_light_color_ = nh_.subscribe("/light_color", 1, &StopperDistance::callbackLightColor, this);
		sub_camera_light_color_ = nh_.subscribe("/camera_light_color", 1, &StopperDistance::callbackCameraLightColor, this);
		sub_temporari_flag_ = nh_.subscribe("/temporary_flag", 1, &StopperDistance::callbackTemporaryFlag, this);
		sub_obstacle_waypoint_ = nh_.subscribe("/obstacle_waypoint", 1, &StopperDistance::callbackObstacleWaypoint, this);
		sub_mobileye_velocity_ = nh_.subscribe("/obstacle_velocity", 1, &StopperDistance::callbackMobileyeVelocity, this);
		sub_temporary_fixed_velocity_ = nh_.subscribe("/temporary_fixed_velocity", 1, &StopperDistance::callbackTemporaryFixedVelocity, this);
		sub_can502_ = nh_.subscribe("/microbus/can_receive502", 1, &StopperDistance::callbackCan502, this);
		sub_can_info_ = nh_.subscribe("/can_info", 1, &StopperDistance::callbackCanInfo, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stopper_distance");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	StopperDistance sd(nh, private_nh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
