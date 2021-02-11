#include <ros/ros.h>
#include <autoware_msgs/Lane.h>

class LocalWaypointAdjustment
{
private:
	ros::NodeHandle nh_, p_nh_;

	ros::Subscriber sub_local_waypoints_;
	ros::Publisher pub_position_adjustment_waypoints_;

	void callbackLocalWaypoints(const autoware_msgs::Lane &msg)
	{
		autoware_msgs::Lane ret = msg;

		for(int ind=2; ind<msg.waypoints.size(); ind++)
		{
			autoware_msgs::Waypoint way1 = msg.waypoints[ind-1];
			autoware_msgs::Waypoint way2 = msg.waypoints[ind];
			double magn = way2.waypoint_param.position_adjustment_magn;

			double x1 = way1.pose.pose.position.x;
			double y1 = way1.pose.pose.position.y;
			double z1 = way1.pose.pose.position.z;
			double x2 = way2.pose.pose.position.x;
			double y2 = way2.pose.pose.position.y;
			double z2 = way2.pose.pose.position.z;
			double denominator = sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
			double newX = x1 + magn * (y2 - y1) / denominator;
			double newY = y1 + magn * (x1 - x2) / denominator;
//std::cout << newX << "," << newY << std::endl;
			ret.waypoints[ind].pose.pose.position.x = newX;
			ret.waypoints[ind].pose.pose.position.y = newY;
		}

		pub_position_adjustment_waypoints_.publish(ret);
	}
public:
	LocalWaypointAdjustment(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
	{
		pub_position_adjustment_waypoints_ = nh.advertise<autoware_msgs::Lane>("/position_adjustment_waypoints", 10);
		sub_local_waypoints_ = nh_.subscribe("/safety_waypoints", 10, &LocalWaypointAdjustment::callbackLocalWaypoints, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "local_waypoint_adjustment");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	LocalWaypointAdjustment lwa(nh, private_nh);
	ros::spin();
	return 0;
}
