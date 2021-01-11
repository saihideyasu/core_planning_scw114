#include <fstream>
#include <unordered_map>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <autoware_config_msgs/ConfigWaypointFileMove.h>
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

class WaypointFileMove
{
private:
	ros::NodeHandle nh_, p_nh_;
	ros::Subscriber sub_config_, sub_pose_, sub_file_write_;

	autoware_config_msgs::ConfigWaypointFileMove config_;
	std::vector<autoware_msgs::Waypoint> waypoints_, move_waypoints_;
	std::string out_file_name_;

	void callbackConfig(const autoware_config_msgs::ConfigWaypointFileMove &msg)
	{
		config_ = msg;
	}

	void callbackPose(const geometry_msgs::PoseStamped &msg)
	{
		move_waypoints_.clear();

		Eigen::Vector3d pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
		double roll, pitch, yaw;
		geometry_quat_to_rpy(roll, pitch, yaw, msg.pose.orientation);
		Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

		//Eigen::Vector3d base_id_pose;
		geometry_msgs::Pose base_id_pose;
		Eigen::Quaterniond base_id_qua;
		for(int ind=0; ind<waypoints_.size(); ind++)
		{
			if(waypoints_[ind].waypoint_param.id == config_.base_id)
			{
				base_id_pose = waypoints_[ind].pose.pose;
				//geometry_msgs::Pose &po = waypoints_[ind].pose.pose;
				//base_id_pose = Eigen::Vector3d(po.position.x, po.position.y, po.position.z);
				double base_id_roll, base_id_pitch, base_id_yaw;
				geometry_quat_to_rpy(base_id_roll, base_id_pitch, base_id_yaw, base_id_pose.orientation);
				base_id_qua = Eigen::Quaterniond(Eigen::AngleAxisd(base_id_yaw, Eigen::Vector3d::UnitZ()));
				break;
			}
		}

		move_waypoints_ = waypoints_;
		for(int ind=0; ind<waypoints_.size(); ind++)
		{
			geometry_msgs::Pose &po = waypoints_[ind].pose.pose;
			Eigen::Vector3d ei_po = Eigen::Vector3d(po.position.x - base_id_pose.position.x + msg.pose.position.x - pose.x(),
			                               po.position.y - base_id_pose.position.y + msg.pose.position.y - pose.y(),
										   po.position.z - base_id_pose.position.z + msg.pose.position.z - pose.z());
			Eigen::Quaterniond diff_qua = qua * base_id_qua.inverse();
			Eigen::Vector3d po_rot = diff_qua * ei_po;
			move_waypoints_[ind].pose.pose.position.x = po_rot.x() + pose.x();
			move_waypoints_[ind].pose.pose.position.y = po_rot.y() + pose.y();
			move_waypoints_[ind].pose.pose.position.z = po_rot.z() + pose.z();

			double cur_roll, cur_pitch, cur_yaw;
			geometry_quat_to_rpy(cur_roll, cur_pitch, cur_yaw, po.orientation);
			Eigen::Quaterniond ei_qua = Eigen::Quaterniond(Eigen::AngleAxisd(cur_yaw, Eigen::Vector3d::UnitZ()));
			Eigen::Quaterniond change_qua = ei_qua * diff_qua;
			move_waypoints_[ind].pose.pose.orientation.x = change_qua.x();
			move_waypoints_[ind].pose.pose.orientation.y = change_qua.y();
			move_waypoints_[ind].pose.pose.orientation.z = change_qua.z();
			move_waypoints_[ind].pose.pose.orientation.w = change_qua.w();
		}
	}

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
	}

	bool read_csv()
	{
		std::string file_name;
		p_nh_.param<std::string>("input_file_name", file_name, "/home/autoware/load_data/okabe/tyokusen_kensyo/go.csv");
		p_nh_.param<std::string>("output_file_name", out_file_name_, "/tmp/move.csv");

		std::ifstream ifs(file_name);
		if(!ifs.is_open()) return false;

		std::string line;
		std::getline(ifs, line);  // get first line
		std::vector<std::string> contents;
		parseColumns(line, &contents);

		// std::getline(ifs, line);  // remove second line
		while (std::getline(ifs, line))
		{
			autoware_msgs::Waypoint wp;
			parseWaypoint(line, contents, &wp);
			waypoints_.emplace_back(wp);
		}

		return true;
	}

	void callbackWrite(const std_msgs::Empty &msg)
	{
		std::ofstream ofs(out_file_name_);
		if(!ofs.is_open())
		{
			std::cout << "error : out file not open" << std::endl;
			return;
		}

		ofs << "id,x,y,z,yaw\n";
		for(int ind=0; ind<move_waypoints_.size(); ind++)
		{
			ofs << move_waypoints_[ind].waypoint_param.id << ",";
			ofs << move_waypoints_[ind].pose.pose.position.x << ",";
			ofs << move_waypoints_[ind].pose.pose.position.y << ",";
			ofs << move_waypoints_[ind].pose.pose.position.z << ",";
			double roll, pitch, yaw;
			geometry_quat_to_rpy(roll, pitch, yaw, move_waypoints_[ind].pose.pose.orientation);
			ofs << yaw << "\n";
		}
	}
public:
	WaypointFileMove(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
	{
		if(!read_csv())
		{
			std::cout << "error : read file" << std::endl;
			return;
		}

		sub_config_ = nh.subscribe("/config/waypoint_file_move", 10 , &WaypointFileMove::callbackConfig, this);
		sub_pose_ = nh.subscribe("/current_pose", 10 , &WaypointFileMove::callbackPose, this);
		sub_file_write_ = nh.subscribe("/waypoint_file_move/write", 10 , &WaypointFileMove::callbackWrite, this);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_file_move");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	WaypointFileMove wfm(nh, private_nh);
	ros::spin();
	return 0;
}