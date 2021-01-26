#include <ros/ros.h>
#include <fstream>
#include <unordered_map>
#include <std_msgs/Empty.h>
#include <autoware_config_msgs/ConfigStraightLineFix.h>
#include <autoware_msgs/Lane.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//geometry_msgs/Quaternionからroll,pitch,yawを取得
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

//直線(変数1,2,3,4)と点(変数5,6)の垂線の交点をrexX,retYに代入
void math_way_current_point(double lineX1, double lineY1, double lineX2, double lineY2,
						double pointX, double pointY, double *retX, double *retY)
{
	double x1 = lineX1, x2 = lineX2;
	double y1 = lineY1, y2 = lineY2;
	double a = y2 - y1;
	double b = x1 - x2;
	double c = - x1 * y2 + y1 * x2;
	double x0 = pointX, y0 = pointY;
	double x_numerator = b * (b * x0 - a * y0) - a * c;
	double x_denominator = a * a + b * b;
	*retX = x_numerator / x_denominator;
	double y_numerator = a * (-b * x0 + a * y0) - b * c;
	double y_denominator = a * a + b * b;
	*retY = y_numerator / y_denominator;
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

	//ファイルの読み込み
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

	//直線の始点と終点から経路全体のyawを計算
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

	//直線の始点のwaypoint indexを取得
	int searchStartPointIndex()
	{
		for(int i=0; i<waypoints_.size(); i++)
			if(config_.start_id == waypoints_[i].waypoint_param.id) return i;
		return 0;
	}

	//直線の終点のwaypoint indexを取得
	int searchEndPointIndex()
	{
		for(int i=0; i<waypoints_.size(); i++)
			if(config_.end_id == waypoints_[i].waypoint_param.id) return i;
		return 0;
	}

	//ファイル出力callback
	void callbackWrite(const std_msgs::Empty &msg)
	{
		double line_yaw = math_line_yaw();//経路全体のyaw
		int start_ind = searchStartPointIndex();//直線始点
		int end_ind = searchEndPointIndex();//直線終点

		geometry_msgs::Point &start_po  = waypoints_[start_ind].pose.pose.position;//直線始点の座標
		geometry_msgs::Point &end_po  = waypoints_[end_ind].pose.pose.position;//直線終点の座標
		int fix_point_count = end_ind - start_ind;//補正するwaypoint総数
		std::vector<autoware_msgs::Waypoint> copy_waypoints_ = waypoints_;

		for(int ind=0; ind<=fix_point_count; ind++)
		{
			geometry_msgs::Point &cur_po = waypoints_[start_ind + ind].pose.pose.position;//補正するwaypoint座標
			double new_x, new_y;
			math_way_current_point(start_po.x, start_po.y, end_po.x, end_po.y, cur_po.x, cur_po.y, &new_x, &new_y);//補正関数
			copy_waypoints_[start_ind + ind].pose.pose.position.x = new_x;
			copy_waypoints_[start_ind + ind].pose.pose.position.y = new_y;
			copy_waypoints_[start_ind + ind].pose.pose.orientation = tf::createQuaternionMsgFromYaw(line_yaw);
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
		sub_file_write_ = nh.subscribe("/straight_line_fix/write", 10 , &StraightLineFix::callbackWrite, this);//ファイル書き込み指令
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