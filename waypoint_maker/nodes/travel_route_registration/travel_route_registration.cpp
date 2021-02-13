#include <ros/ros.h>
#include <autoware_config_msgs/ConfigTravelRouteRegistration.h>
#include <fstream>
#include <unordered_map>
#include <autoware_msgs/Lane.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

struct LogInfo
{
	int id_;
	geometry_msgs::Pose pose_;
};

//geometry_msgs/Quaternionからroll,pitch,yawを取得
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

double pointDt(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
	double x = p1.x - p2.x;
	double y = p1.y - p2.y;
	double z = p1.z - p2.z;
	return sqrt(x*x + y*y + z*z);
}

void parseColumns(const std::string& line, std::vector<std::string>* columns)
{
  std::istringstream ss(line);
  std::string column;
  int count = 0;
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
    }//std::cout << count << std::endl;
	count++;
  }
  //std::cout << "field count : " << count << std::endl;
}

class TravelRouteRegistration
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Subscriber sub_config_;

	std::vector<autoware_msgs::Waypoint> global_waypoints_;//global経路
	std::vector<LogInfo> log_data_;//logファイルデータ

	//waypointの各フィールド項目値を取得
	void parseGlobalWaypoint(const std::string& line, const std::vector<std::string>& contents,
												autoware_msgs::Waypoint* wp)
	{
		std::vector<std::string> columns;
		parseColumns(line, &columns);
		std::unordered_map<std::string, std::string> map;
		for (size_t i = 0; i < columns.size(); i++)
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
												LogInfo *pose)
	{static int count = 0;
		std::vector<std::string> columns;
		parseColumns(line, &columns);
		std::unordered_map<std::string, std::string> map;
		for (size_t i = 0; i < columns.size(); i++)
		{
			map[contents.at(i)] = columns.at(i); if(i == 23) std::cout << i << "," << contents.at(i) << "," << columns.at(i)<< std::endl;
		}

		pose->id_ = std::stoi(map["waypoint_id"]);
		pose->pose_.position.x = std::stod(map["gnssx"]);
		pose->pose_.position.y = std::stod(map["gnssy"]);
		pose->pose_.position.z = std::stod(map["gnssz"]);
		pose->pose_.orientation = tf::createQuaternionMsgFromYaw(std::stod(map["gnss_yaw"])); count++;
	}

	bool read_waypoint_csv()
	{
		std::string file_name;
		pnh_.param<std::string>("waypoint_file_name", file_name, "");

		std::ifstream ifs(file_name);
		if(!ifs.is_open())
		{
			std::cout << "open error : waypoint_file" << std::endl;
			return false;
		}

		//フィールド名の取得
		std::string line;
		std::getline(ifs, line);
		std::vector<std::string> contents;
		parseColumns(line, &contents);
		std::cout << "waypoints field read" << std::endl;

		//各列のフィールド項目値を取得
		while (std::getline(ifs, line))
		{
			autoware_msgs::Waypoint wp;
			parseGlobalWaypoint(line, contents, &wp);
			global_waypoints_.emplace_back(wp);
		}

		ifs.close();
		return true;
	}

	bool read_log_csv()
	{
		std::string file_name;
		pnh_.param<std::string>("log_file_name", file_name, "");

		std::ifstream ifs(file_name);
		if(!ifs.is_open())
		{
			std::cout << "open error : log_file" << std::endl;
			return false;
		}

		//フィールド名の取得
		std::string line;
		std::getline(ifs, line);
		std::vector<std::string> contents;
		parseColumns(line, &contents); for(int i=0;i<contents.size();i++) std::cout << i << "," << contents[i] << std::endl;
		std::cout << "log field read" << std::endl;

		//各列のフィールド項目値を取得
		while (std::getline(ifs, line))
		{//std::cout << line << std::endl;
			LogInfo pose;
			parseLogWaypoint(line, contents, &pose);
			log_data_.emplace_back(pose);
		}

		ifs.close();
		return true;
	}

	void createOutputData()
	{
		for(int way_ind=0; way_ind<global_waypoints_.size(); way_ind++)
		{
			const geometry_msgs::Point &way_point = global_waypoints_[way_ind].pose.pose.position;

			double min_dt = DBL_MAX;//探索したもので、一番近傍なdistance
			geometry_msgs::Pose min_pose;//探索したもので、一番近傍な位置
			for(int log_ind=0; log_ind<log_data_.size(); log_ind++)
			{
				const geometry_msgs::Point &log_point = log_data_[log_ind].pose_.position;
				double dt = pointDt(way_point, log_point);
				if(dt < min_dt)
				{
					min_dt = dt;
					min_pose = log_data_[log_ind].pose_;
				}
			}

			global_waypoints_[way_ind].waypoint_param.global_pose = min_pose;
		}
	}

	void writeData()
	{
		std::string file_name;//出力ファイル名
		pnh_.param<std::string>("output_file_name", file_name, "/tmp/move.csv");

		std::ofstream ofs(file_name);
		if(ofs.is_open() == false)
		{
			std::cout << "open error : output_file" << std::endl;
			return;
		}

		ofs << "history_id,history_x,history_y,history_z,history_yaw\n";
		for(int i=0; i<global_waypoints_.size(); i++)
		{
			int id = global_waypoints_[i].waypoint_param.id;
			double x = global_waypoints_[i].waypoint_param.global_pose.position.x;
			double y = global_waypoints_[i].waypoint_param.global_pose.position.y;
			double z = global_waypoints_[i].waypoint_param.global_pose.position.z;
			double roll, pitch, yaw;
			geometry_quat_to_rpy(roll, pitch, yaw, global_waypoints_[i].waypoint_param.global_pose.orientation);

			ofs << std::fixed;
			ofs << std::setprecision(6) << id << "," << x << "," << y << "," << z << "," << yaw << "\n";
		}

		ofs.close();
	}
public:
	TravelRouteRegistration(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		//経路ファイル読み込み
		std::cout <<"waypoint read start" << std::endl;
		if(!read_waypoint_csv())
		{
			std::cout << "error : read waypoint file" << std::endl;
			return;
		}

		//logファイル読み込み
		std::cout <<"log read start" << std::endl;
		if(!read_log_csv())
		{
			std::cout << "error : read log file" << std::endl;
			return;
		}

		std::cout << "data create" << std::endl;
		createOutputData();
		std::cout << "data write" << std::endl;
		writeData();
		std::cout << "end" << std::endl;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "travel_route_registration");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	TravelRouteRegistration trg(nh, private_nh);
	//ros::spin();
	return 0;
}