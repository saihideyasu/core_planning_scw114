#include <ros/ros.h>
#include <fstream>
#include <unordered_map>
#include <std_msgs/Empty.h>
#include <autoware_config_msgs/ConfigStraightLineFix.h>
#include <autoware_msgs/Lane.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

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

class StraightLineFix
{
private:
	ros::NodeHandle nh_, pnh_;
	ros::Subscriber sub_config_, sub_file_write_;

	autoware_config_msgs::ConfigStraightLineFix config_;
	std::string out_file_name_;
	std::vector<autoware_msgs::Waypoint> waypoints_;

	void callbackConfig(const autoware_config_msgs::ConfigStraightLineFix &msg)
	{
		config_ = msg;
	}

	//waypointの各フィールド項目値を取得
	void parseWaypoint(const std::string& line, const std::vector<std::string>& contents,
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

	bool read_csv()
	{
		std::string file_name;
		pnh_.param<std::string>("input_file_name", file_name, "/home/autoware/load_data/okabe/tyokusen_kensyo/go.csv");
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
			parseWaypoint(line, contents, &wp);
			waypoints_.emplace_back(wp);
		}

		return true;
	}

	double math_line_yaw()
	{
		geometry_msgs::Point start_po, end_po;
		for(const autoware_msgs::Waypoint &way : waypoints_)
		{
			if(way.waypoint_param.id == config_.start_id)
				start_po = way.pose.pose.position;
			if(way.waypoint_param.id == config_.end_id)
				end_po = way.pose.pose.position;
		}

		double x = end_po.x - start_po.x;
		double y = end_po.y - start_po.y;
		return atan2(y, x);
	}

	int searchStartPointIndex()
	{
		for(int i=0; i<waypoints_.size(); i++)
		{
			if(config_.start_id == waypoints_[i].waypoint_param.id) return i;
		}
		return 0;
	}

	int searchEndPointIndex()
	{
		for(int i=0; i<waypoints_.size(); i++)
		{
			if(config_.end_id == waypoints_[i].waypoint_param.id) return i;
		}
		return 0;
	}

	void callbackWrite(const std_msgs::Empty &msg)
	{
		double line_yaw = math_line_yaw();
		int start_ind = searchStartPointIndex();
		int end_ind = searchEndPointIndex();

		autoware_msgs::Waypoint &start_way = waypoints_[start_ind];
		autoware_msgs::Waypoint &end_way = waypoints_[end_ind];
		double base_x = start_way.pose.pose.position.x;
		double base_y = start_way.pose.pose.position.y;
		double x_length = end_way.pose.pose.position.x - start_way.pose.pose.position.x;
		double y_length = end_way.pose.pose.position.y - start_way.pose.pose.position.y;
		int fix_point_count = end_ind - start_ind;
		std::vector<autoware_msgs::Waypoint> copy_waypoints_ = waypoints_;

		for(int ind=0; ind<=fix_point_count; ind++)
		{
			double weight = (double)ind / (double)fix_point_count;
			copy_waypoints_[start_ind + ind].pose.pose.position.x = base_x + x_length * weight;
			copy_waypoints_[start_ind + ind].pose.pose.position.y = base_y + y_length * weight;
		}

		std::ofstream ofs(out_file_name_);
		if(!ofs.is_open())
		{
			std::cout << "error : out file not open" << std::endl;
			return;
		}

		ofs << "id,x,y,z,yaw,velocity,change_flag\n";//必要最低限のフィールド(idを除く)
		for(int ind=0; ind<copy_waypoints_.size(); ind++)
		{
			ofs << std::fixed;
			ofs << copy_waypoints_[ind].waypoint_param.id << ",";
			ofs << std::setprecision(8) << copy_waypoints_[ind].pose.pose.position.x << ",";
			ofs << std::setprecision(8) << copy_waypoints_[ind].pose.pose.position.y << ",";
			ofs << std::setprecision(8) << copy_waypoints_[ind].pose.pose.position.z << ",";
			double roll, pitch, yaw;
			geometry_quat_to_rpy(roll, pitch, yaw, copy_waypoints_[ind].pose.pose.orientation);
			ofs << std::setprecision(8) << yaw << ",";
			ofs << std::setprecision(8) << copy_waypoints_[ind].twist.twist.linear.x << ",";
			ofs << "0\n";
		}
	}
public:
	StraightLineFix(ros::NodeHandle nh, ros::NodeHandle pnh)
		: nh_(nh)
		, pnh_(pnh)
	{
		if(!read_csv())
		{
			std::cout << "error : read file" << std::endl;
			return;
		}

		sub_config_ = nh_.subscribe("/config/straight_line_fix", 1, &StraightLineFix::callbackConfig, this);
		sub_file_write_ = nh.subscribe("/straight_line_fix/write", 10 , &StraightLineFix::callbackWrite, this);
	}
};

int main(int argc, char**argv)
{
	ros::init(argc, argv, "straight_line_fix");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	StraightLineFix slf(nh, private_nh);
	ros::spin();
	return 0;
}