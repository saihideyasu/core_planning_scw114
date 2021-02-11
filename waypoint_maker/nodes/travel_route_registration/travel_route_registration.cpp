#include <ros/ros.h>
#include <autoware_config_msgs/ConfigTravelRouteRegistration.h>
#include <fstream>
#include <unordered_map>
#include <autoware_msgs/Lane.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

struct TravelInfo
{
	int id;
};

void parseColumns(const std::string& line, std::vector<std::string>* columns)
{
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ','))
  {
    while (1)
    {
      auto res = std::find(column.begin(), column.end(), ' ');
      if (res == column.end())
      {
        break;
      }
      column.erase(res);
    }
    if (!column.empty())
    {
      columns->emplace_back(column);
    }
  }
}

class TravelRouteRegistration
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Subscriber sub_config_;

	std::string out_file_name_;//出力ファイル名
	std::vector<autoware_msgs::Waypoint> global_waypoints_;//global経路
	std::vector<geometry_msgs::Pose> log_data_;//logファイルデータ

	//waypointの各フィールド項目値を取得
	void parseGlobalWaypoint(const std::string& line, const std::vector<std::string>& contents,
												autoware_msgs::Waypoint* wp)
	{
		std::vector<std::string> columns;
		parseColumns(line, &columns);
		std::unordered_map<std::string, std::string> map;
		for (size_t i = 0; i < contents.size(); i++)
		{
			map[contents.at(i)] = columns.at(i);
		}

		wp->pose.pose.position.x = std::stod(map["x"]); //std::cout << std::setprecision(6) << wp->position.x << std::endl;
		wp->pose.pose.position.y = std::stod(map["y"]);
		wp->pose.pose.position.z = std::stod(map["z"]);
		wp->pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(map["yaw"]));
		wp->waypoint_param.id = std::stod(map["id"]);
		wp->twist.twist.linear.x = std::stod(map["velocity"]);
	}

	//logの各フィールド項目値を取得
	void parseLogWaypoint(const std::string& line, const std::vector<std::string>& contents,
												geometry_msgs::Pose *pose)
	{
		std::vector<std::string> columns;
		parseColumns(line, &columns);
		std::unordered_map<std::string, std::string> map;
		for (size_t i = 0; i < contents.size(); i++)
		{
			map[contents.at(i)] = columns.at(i);
		}
	}

	bool read_global_csv()
	{
		std::string file_name;
		pnh_.param<std::string>("global_file_name", file_name, "");
		pnh_.param<std::string>("output_file_name", out_file_name_, "/tmp/move.csv");

		std::ifstream ifs(file_name);
		if(!ifs.is_open()) return false;

		//フィールド名の取得
		std::string line;
		std::getline(ifs, line);
		std::vector<std::string> contents;
		parseColumns(line, &contents);

		//各列のフィールド項目値を取得
		while (std::getline(ifs, line))
		{
			autoware_msgs::Waypoint wp;
			parseGlobalWaypoint(line, contents, &wp);
			global_waypoints_.emplace_back(wp);
		}

		return true;
	}

	bool read_log_csv()
	{
		std::string file_name;
		pnh_.param<std::string>("log_file_name", file_name, "");

		std::ifstream ifs(file_name);
		if(!ifs.is_open()) return false;

		//フィールド名の取得
		std::string line;
		std::getline(ifs, line);
		std::vector<std::string> contents;
		parseColumns(line, &contents);

		//各列のフィールド項目値を取得
		while (std::getline(ifs, line))
		{
			geometry_msgs::Pose pose;
			parseLogWaypoint(line, contents, &pose);
			//global_waypoints_.emplace_back(wp);
		}

		return true;
	}
public:
	TravelRouteRegistration(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		if(!read_global_csv())
		{
			std::cout << "error : read file" << std::endl;
			return;
		}

		std::vector<TravelInfo> travel_list;
		
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "travel_route_registration");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	TravelRouteRegistration trg(nh, private_nh);
	ros::spin();
	return 0;
}