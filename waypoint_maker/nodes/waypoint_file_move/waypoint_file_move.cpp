/*
------経路csvファイルの移動ノード-------
---経路csvファイルに必要なフィールドは以下の通り
  id       : 経路id  1から連番であること
  x        : waypointのx座標値
  y        : waypointのy座標値
  z        : waypointのz座標値
  yaw      : waypointのyaw角
  velocity : waypointの設定速度
---appの説明
  input_file_name  : 読み込みたい経路csvファイルのpath
  output_file_name : 移動させた経路データを書き込むファイルのpath
  base_id          : 読み込んだ経路の移動中心とする経路id番号
  first_id         : 移動させたい最初のwaypointのid  このid未満のwaypointは出力されない
  end_id           : 移動させたい最後のwaypointのid  このidより上のwaypointは出力されない
---トピックの説明
  /config/waypoint_file_move  : このノードの設定  appのbase_id,first_id,end_idはこのトピックで設定される
  /current_pose               : 移動基準とする座標(車のbase_link)
  /waypoint_file_move/write   : 移動経路のファイル出力を行う
---実行方法
  appの項目を適切に設定し、ノードを実行、/current_poseトピックをpublishすると移動経路が生成される。
  その後/waypoint_file_move/writeで出力できる
*/

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

		//current_poseのpositionとquaternion
		Eigen::Vector3d pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
		double roll, pitch, yaw;
		geometry_quat_to_rpy(roll, pitch, yaw, msg.pose.orientation);
		Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

		//base_idのpositionとquaternion
		geometry_msgs::Pose base_id_pose;
		Eigen::Quaterniond base_id_qua;
		for(int ind=0; ind<waypoints_.size(); ind++)
		{
			if(waypoints_[ind].waypoint_param.id == config_.base_id)
			{
				base_id_pose = waypoints_[ind].pose.pose;
				double base_id_roll, base_id_pitch, base_id_yaw;
				geometry_quat_to_rpy(base_id_roll, base_id_pitch, base_id_yaw, base_id_pose.orientation);
				base_id_qua = Eigen::Quaterniond(Eigen::AngleAxisd(base_id_yaw, Eigen::Vector3d::UnitZ()));
				break;
			}
		}

		//current_poseとbase_idの情報から移動経路を作成
		move_waypoints_ = waypoints_;
		for(int ind=0; ind<waypoints_.size(); ind++)
		{
			//waypont座標を移動
			geometry_msgs::Pose &po = waypoints_[ind].pose.pose;
			Eigen::Vector3d ei_po = Eigen::Vector3d(po.position.x - base_id_pose.position.x + msg.pose.position.x - pose.x(),
			                               po.position.y - base_id_pose.position.y + msg.pose.position.y - pose.y(),
										   po.position.z - base_id_pose.position.z + msg.pose.position.z - pose.z());
			Eigen::Quaterniond diff_qua = qua * base_id_qua.inverse();
			Eigen::Vector3d po_rot = diff_qua * ei_po;
			move_waypoints_[ind].pose.pose.position.x = po_rot.x() + pose.x();
			move_waypoints_[ind].pose.pose.position.y = po_rot.y() + pose.y();
			move_waypoints_[ind].pose.pose.position.z = po_rot.z() + pose.z();

			//waypontのベクトルを変換
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
		p_nh_.param<std::string>("input_file_name", file_name, "/home/autoware/load_data/okabe/tyokusen_kensyo/go.csv");
		p_nh_.param<std::string>("output_file_name", out_file_name_, "/tmp/move.csv");

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

	void callbackWrite(const std_msgs::Empty &msg)
	{
		std::ofstream ofs(out_file_name_);
		if(!ofs.is_open())
		{
			std::cout << "error : out file not open" << std::endl;
			return;
		}

		ofs << "id,x,y,z,yaw,velocity,change_flag\n";//必要最低限のフィールド(idを除く)
		for(int ind=0; ind<move_waypoints_.size(); ind++)
		{
			int id = ind + 1;
			if(id >= config_.first_id && id <= config_.end_id)
			{
				ofs << std::fixed;
				ofs << move_waypoints_[ind].waypoint_param.id << ",";
				ofs << std::setprecision(8) << move_waypoints_[ind].pose.pose.position.x << ",";
				ofs << std::setprecision(8) << move_waypoints_[ind].pose.pose.position.y << ",";
				ofs << std::setprecision(8) << move_waypoints_[ind].pose.pose.position.z << ",";
				double roll, pitch, yaw;
				geometry_quat_to_rpy(roll, pitch, yaw, move_waypoints_[ind].pose.pose.orientation);
				ofs << std::setprecision(8) << yaw << ",";
				ofs << std::setprecision(8) << move_waypoints_[ind].twist.twist.linear.x << ",";
				ofs << "0\n";
			}
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