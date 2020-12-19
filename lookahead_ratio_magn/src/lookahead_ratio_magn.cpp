#include <ros/ros.h>
#include <autoware_config_msgs/ConfigLookAheadRatioMagn.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <autoware_msgs/Lane.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <autoware_msgs/DifferenceToWaypointDistance.h>
#include <std_msgs/Float64.h>

static const int SYNC_FRAMES = 50;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::PoseStamped>
    TwistPoseSync;

class LookaheadRatioMagn
{
private:
	const double timeout = 0.5;

	ros::NodeHandle nh_, private_nh_;

	ros::Subscriber sub_config_, sub_waypoints_;
	ros::Publisher pub_difference_to_waypoint_distance_;

	tf::TransformListener *listener_;

	message_filters::Subscriber<geometry_msgs::TwistStamped> *sub_current_velocity_;
	message_filters::Subscriber<geometry_msgs::PoseStamped> *sub_current_pose_;
	message_filters::Synchronizer<TwistPoseSync> *sync_twist_pose_;

	autoware_config_msgs::ConfigLookAheadRatioMagn config_;

	//geometry_msgs::Pose waypose1_, waypose2_;
	std::vector<geometry_msgs::Pose> waypose_;
	geometry_msgs::TwistStamped current_twist_;
	geometry_msgs::PoseStamped current_pose_;

	void callbackConfig(const autoware_config_msgs::ConfigLookAheadRatioMagn &msg)
	{
		config_ = msg;
	}

	void callbackWaypoints(const autoware_msgs::Lane &msg)
	{
		waypose_.clear();
		if(msg.waypoints.size() < 3) return;

		for(int i=0; i<msg.waypoints.size(); i++)
			waypose_.push_back(msg.waypoints[i].pose.pose);
		//waypose1_ = msg.waypoints[1].pose.pose;
		//waypose2_ = msg.waypoints[2].pose.pose;
	}

	void TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
	                       const geometry_msgs::PoseStampedConstPtr &pose_msg)
	{
		current_twist_ = *twist_msg;
		current_pose_ = *pose_msg;
		std::cout << "cur : " << current_pose_.pose.position.x << "," << current_pose_.pose.position.y << "," << current_pose_.pose.position.z << std::endl;
	}
public:
	LookaheadRatioMagn(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, private_nh_(p_nh)
	{
		listener_ = new tf::TransformListener();

		pub_difference_to_waypoint_distance_ = nh_.advertise<autoware_msgs::DifferenceToWaypointDistance>("/difference_to_waypoint_distance", 1);
		sub_config_= nh_.subscribe("/config/lookahead_ratio_magn", 1, &LookaheadRatioMagn::callbackConfig, this);
		sub_waypoints_= nh_.subscribe("/final_waypoints", 1, &LookaheadRatioMagn::callbackWaypoints, this);
		sub_current_pose_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/current_pose", 10);
		sub_current_velocity_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "/current_velocity", 10);
		sync_twist_pose_ = new message_filters::Synchronizer<TwistPoseSync>(TwistPoseSync(SYNC_FRAMES), *sub_current_velocity_, *sub_current_pose_);
		sync_twist_pose_->registerCallback(boost::bind(&LookaheadRatioMagn::TwistPoseCallback, this, _1, _2));
	}
	
	~LookaheadRatioMagn()
	{
		delete listener_;
	}

	double math_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, geometry_msgs::Pose cur)
	{
		double x1 = pose1.position.x, x2 = pose2.position.x;
		double y1 = pose1.position.y, y2 = pose2.position.y;
		double a = y2 - y1;
		double b = x1 - x2;
		double c = - x1 * y2 + y1 * x2;

		//double x0 = current_pose_.pose.position.x, y0 = current_pose_.pose.position.y;
		double x0 = cur.position.x, y0 = cur.position.y;
		double db = sqrt(a * a + b * b);
		if(db == 0)
		{
			return 0;
		}
		return (a * x0 + b * y0 + c) / db;
	}

	double math_distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2, tf::Vector3 cur)
	{
		double x1 = pose1.position.x, x2 = pose2.position.x;
		double y1 = pose1.position.y, y2 = pose2.position.y;
		double a = y2 - y1;
		double b = x1 - x2;
		double c = - x1 * y2 + y1 * x2;

		//double x0 = current_pose_.pose.position.x, y0 = current_pose_.pose.position.y;
		double x0 = cur.getX(), y0 = cur.getY();
		double db = sqrt(a * a + b * b);
		if(db == 0)
		{
			return 0;
		}
		return (a * x0 + b * y0 + c) / db;
	}

	double euclid(double x1, double y1, double x2, double y2)
	{
		double x = x1 - x2;
		double y = y1 - y2;
		return sqrt(x*x + y*y);
	}

	tf::Matrix3x3 math_angular(geometry_msgs::Pose pose, geometry_msgs::Pose cur_pose)
	{
		tf::Quaternion current_q(cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z,
			cur_pose.orientation.w);

		tf::Quaternion way_q(pose.orientation.x, pose.orientation.y, pose.orientation.z,
			pose.orientation.w);
		tf::Matrix3x3 way_m(way_q);
		double way_yaw, way_roll, way_pitch;
		way_m.getRPY(way_roll, way_pitch, way_yaw);

		tf::Quaternion sa_q = way_q * current_q.inverse();
		tf::Matrix3x3 sa_m(sa_q);
		return sa_m;
	}

	void run()
	{
		tf::Vector3 front_baselink_point;
		tf::Quaternion front_baselink_quat;
		if(sub_current_pose_->getTopic() == "/current_pose")
		{
			tf::StampedTransform frontbaselink_to_map;
			try{
				listener_->lookupTransform("/map", "/front_base_link", 
				ros::Time(0), frontbaselink_to_map);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				return;
			}
		
			front_baselink_point = frontbaselink_to_map.getOrigin();
			front_baselink_quat = frontbaselink_to_map.getRotation();
		}

		if(waypose_.size() < 2)
		{
			autoware_msgs::DifferenceToWaypointDistance dist;
			dist.header.stamp = ros::Time::now();
			dist.baselink_distance = 0;
			dist.baselink_angular = 0;
			dist.front_baselink_distance = 0;
			dist.front_baselink_angular = 0;
			pub_difference_to_waypoint_distance_.publish(dist);
			return;
		}

		double d = math_distance(waypose_[1],  waypose_[2], current_pose_.pose);

		geometry_msgs::Pose front_pose;
		front_pose.position.x = front_baselink_point.getX();
		front_pose.position.y = front_baselink_point.getY();
		front_pose.position.z = front_baselink_point.getZ();
		front_pose.orientation.x = front_baselink_quat.getX();
		front_pose.orientation.y = front_baselink_quat.getY();
		front_pose.orientation.z = front_baselink_quat.getZ();
		front_pose.orientation.w = front_baselink_quat.getW();
		double fd = math_distance(waypose_[1],  waypose_[2], front_pose);

		autoware_msgs::DifferenceToWaypointDistance dist;
		dist.header.stamp = ros::Time::now();
		tf::Matrix3x3 sa_m = math_angular(waypose_[1], current_pose_.pose);
		double sa_yaw, sa_roll, sa_pitch;
		sa_m.getRPY(sa_roll, sa_pitch, sa_yaw);
		dist.baselink_distance = d;
		dist.baselink_angular = sa_yaw;
		dist.front_baselink_distance = fd;
		tf::Matrix3x3 fsa_m = math_angular(waypose_[1], front_pose);
		double fsa_yaw, fsa_roll, fsa_pitch;
		fsa_m.getRPY(fsa_roll, fsa_pitch, fsa_yaw);
		dist.front_baselink_angular = fsa_yaw;
		pub_difference_to_waypoint_distance_.publish(dist);
		return;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "lookahead_ratio_magn");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	LookaheadRatioMagn lhm(nh, private_nh);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		lhm.run();
		loop_rate.sleep();
	}
	return 0;
}
