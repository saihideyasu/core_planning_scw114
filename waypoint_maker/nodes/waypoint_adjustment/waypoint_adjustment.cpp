#include <ros/ros.h>
#include <autoware_config_msgs/ConfigWaypointAdjustment.h>
#include <autoware_msgs/LaneArray.h>
#
class WaypointAdjustment
{
private:
	ros::NodeHandle nh_, p_nh_;

	ros::Subscriber sub_config_, sub_global_waypoints_;

	autoware_config_msgs::ConfigWaypointAdjustment config_;
	autoware_msgs::Lane global_waypoints_;

	bool read_config_ = false;
	void callbackConfig(const autoware_config_msgs::ConfigWaypointAdjustment &msg)
	{
		config_ = msg;
		read_config_ = true;
	}

	bool read_waypoints_ = false;
	void callbackGlobalWaypoints(const autoware_msgs::LaneArray &msg)
	{
		//std::cout << "a" << std::endl;
		global_waypoints_ = msg.lanes[0];
		read_waypoints_ = true;
	}

	std::vector<double> vec_magn;
	void adjustment(int ind, double magn)
	{
		autoware_msgs::Waypoint way1 = global_waypoints_.waypoints[ind-1];
		autoware_msgs::Waypoint way2 = global_waypoints_.waypoints[ind];

		double x1 = way1.pose.pose.position.x;
		double y1 = way1.pose.pose.position.y;
		double z1 = way1.pose.pose.position.z;
		double x2 = way2.pose.pose.position.x;
		double y2 = way2.pose.pose.position.y;
		double z2 = way2.pose.pose.position.z;
		double denominator = sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
		double newX = x1 + magn * (y2 - y1) / denominator;
		double newY = y1 + magn * (x1 - x2) / denominator;

		std::cout << way2.waypoint_param.id << ",";
		std::cout << std::fixed;
		std::cout << std::setprecision(4) << newX << "," << newY << std::endl;

		vec_magn.push_back(magn);
		/*std::cout <<  way2.waypoint_param.id << std::endl;
		std::cout << std::fixed;
		std::cout << std::setprecision(8) << "," << magn << std::endl;*/
	}
public:
	WaypointAdjustment(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
	{
		sub_config_ = nh_.subscribe("/config/waypoint_adjustment", 1, &WaypointAdjustment::callbackConfig, this);
		sub_global_waypoints_ = nh_.subscribe("/lane_waypoints_array", 1, &WaypointAdjustment::callbackGlobalWaypoints, this);
	}

	bool run()
	{
		if(read_config_ == false || read_waypoints_ == false) return false;

		double magn_step1;
		int magn_count1 = config_.start_id1 - config_.start_id2 + 1;
		if(config_.start_id1 == config_.start_id2) magn_step1 = 0;
		else magn_step1 = (config_.start_magn1 - config_.start_magn2) / (magn_count1-1);

		double magn_step2;
		int magn_count2 = config_.start_id2 - config_.end_id1 + 1;
		if(config_.start_id2 == config_.end_id1) magn_step2 = 0;
		else magn_step2 = (config_.start_magn2 - config_.end_magn1) / (magn_count2-1);

		double magn_step3;
		int magn_count3 = config_.end_id1 - config_.end_id2 + 1;
		if(config_.end_id1 == config_.end_id2) magn_step3 = 0;
		else magn_step3 = (config_.end_magn1 - config_.end_magn2) / (magn_count3-1);

		for(int id=config_.start_id1; id< config_.start_id2; id++)
		{
			//std::cout << id << std::endl;
			for(int i=0; i<global_waypoints_.waypoints.size(); i++)
			{
				if(global_waypoints_.waypoints[i].waypoint_param.id == id)
				{
					int count = id - config_.start_id1;
					double magn = config_.start_magn1 + magn_step1 * count; //std::cout << "magn:" << magn_step1 << std::endl;
					adjustment(i, magn);
					break;
				}
			}
		}

		for(int id=config_.start_id2; id< config_.end_id1; id++)
		{
			for(int i=0; i<global_waypoints_.waypoints.size(); i++)
			{
				if(global_waypoints_.waypoints[i].waypoint_param.id == id)
				{
					int count = id - config_.start_id2;
					double magn = config_.start_magn2 + magn_step2 * count;
					adjustment(i, magn);
					break;
				}
			}
		}

		for(int id=config_.end_id1; id<= config_.end_id2; id++)
		{
			for(int i=0; i<global_waypoints_.waypoints.size(); i++)
			{
				if(global_waypoints_.waypoints[i].waypoint_param.id == id)
				{
					int count = id - config_.end_id1;
					double magn = config_.end_magn1 + magn_step3 * count; //std::cout << "magn:" << magn_step3 << std::endl;
					adjustment(i, magn);
					break;
				}
			}
		}

		for(int i=0; i<vec_magn.size(); i++)
		{
			std::cout << std::fixed;
			std::cout << std::setprecision(8) << vec_magn[i] << std::endl;	
		}
		return true;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_adjustment");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	WaypointAdjustment wa(nh, private_nh);
	ros::Rate rate(10);
	bool run_flag = false;
	while(ros::ok())
	{
		ros::spinOnce();
		if(run_flag == false)
		{
			run_flag = wa.run();
			if(run_flag == true) std::cout << "finish!" << std::endl;
		}
	}
	return 0;
}