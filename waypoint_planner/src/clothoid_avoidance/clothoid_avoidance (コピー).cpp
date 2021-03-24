//参考サイト https://qiita.com/Ushio/items/707b0bf7c758508f2b1a
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <autoware_config_msgs/ConfigClothoidAvoidance.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/AvoidanceLaneInfoList.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

double euclidDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	double x = p1.x - p2.x;
	double y = p1.y - p2.y;
	double z = p1.z - p2.z;
	return sqrt(x*x + y*y + z*z);
}

//void tf_quat_to_rpy(double& roll, double& pitch, double& yaw, tf::Quaternion quat){
//	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
//}
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, const geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

//クロソイド曲線式のΦの計算関数
//phi0 : 初期角度
//phiV : もし曲率が変化しない場合にh分進んだ場合の角度増分(例えばφuが0なら、この曲線は円を描きます)
//phiU : クロソイドによる角度の増分
//S    : 0~1に正規化した曲線長上の位置
float phi(float phi0, float phiV, float phiU, float S) {
	return phi0 + phiV * S + phiU * S * S;
}

//クロソイド曲線の積分内部の計算
std::complex<float> slope_f(float phi0, float phiV, float phiU, float S) {
	std::complex<float> j(0.0f, 1.0f);
	return std::exp(j * phi(phi0, phiV, phiU, S));
}

//シンプソン則での数値積分
template <class T, class Real, class R>
void simpson_integral(T f, Real a, Real b, R *r) {
	Real mul = (b - a) * static_cast<Real>(1.0 / 6.0);
	*r = mul * (f(a) + static_cast<Real>(4.0) * f((a + b) * static_cast<Real>(0.5)) + f(b));
}

//積分内部用オブジェクト
struct Slope {
	float phi0;
	float phiV;
	float phiU;

	std::complex<float> operator()(float S) {
		return slope_f(phi0, phiV, phiU, S);
	}
};

//clothoidに変換するレーン情報
struct ConversionXY
{
	int waypoint_index_;//変換するwayointのインデックス
	//int waypoint_id_;//waypoint_paramのid
	autoware_msgs::AvoidanceLaneInfo xy_;//waypointのxy座標
};

//clothoid長と経路長の対応表用構造体
struct TableClothoidAndLane
{
	double clothoid_length_;
	double lane_length_;
};

//clothoid曲線を用いたS字回避経路の作成
//config : 各種clothoid param
//clothoid_length : clothoid曲線長
//rotation_center : ここを原点として作成したclothoidは回転する
//aviodance_flag : 回避経路作成だとtrue  帰り経路作成だとfalse
std::vector<autoware_msgs::AvoidanceLaneInfo> clothoid(const autoware_config_msgs::ConfigClothoidAvoidance config, const double clothoid_length, const bool avoidance_flag)
{
	int sgn = (avoidance_flag == true) ? -1 : 1;

	Slope slope;
	slope.phi0 = 0 * M_PI / 180.0;
	slope.phiU = sgn * config.phiU * M_PI / 180.0;
	slope.phiV = sgn * config.phiV * M_PI / 180.0;
	//std::cout << slope.phiU << "," << slope.phiV << std::endl;

	float stepS = 1.0 / config.step_count;
	std::complex<float> P_Vector;
	std::vector<autoware_msgs::AvoidanceLaneInfo> xy_list;
	autoware_msgs::AvoidanceLaneInfo center_xy;

	int counter = 0;
	for(int i = 0 ; i < config.step_count ; i++)
	{
		float S = stepS * i;

		std::complex<float> r;
		simpson_integral(slope, S, S + stepS, &r);
		P_Vector += r;

		float x = P_Vector.real();
		float y = P_Vector.imag();
		autoware_msgs::AvoidanceLaneInfo xy;// = {clothoid_length * x, clothoid_length * y, counter+1}; 
		xy.x = clothoid_length * x;  xy.y = clothoid_length * y;  xy.ratio = counter + 1;
		counter++;
		//std::cout << "" << xy.x << "," << xy.y << std::endl;
		if((sgn == 1 && xy.y > config.move_lane_length / 2.0) || (sgn == (-1) &&  xy.y < -config.move_lane_length / 2.0))
		{
			center_xy = xy;
			break;
		}
		xy_list.push_back(xy);
	}

	std::size_t list_size = xy_list.size();
	for(int i = list_size-1; i >= 0; i--)
	{
		autoware_msgs::AvoidanceLaneInfo xy = xy_list[i];
		double dtx = center_xy.x - xy.x;
		double dty = center_xy.y - xy.y;
		autoware_msgs::AvoidanceLaneInfo xyplus;// = {center_xy.x + dtx, center_xy.y + dty, counter+1};
		xyplus.x = center_xy.x + dtx;  xyplus.y = center_xy.y + dty;  xyplus.ratio = counter + 1;
		counter++;

		xy_list.push_back(xyplus);
		//std::cout << "" << xyplus.x << "," << xyplus.y << std::endl;
	}

	std::vector<autoware_msgs::AvoidanceLaneInfo> ret;
	for(autoware_msgs::AvoidanceLaneInfo xy : xy_list)
	{
		xy.ratio /= counter;
		ret.push_back(xy);
	}
	return ret;
}

//waypointのy方向にwidthを追加
autoware_msgs::AvoidanceLaneInfo way_pose_convert(const geometry_msgs::Pose way_pose, double width)
{
	Eigen::Vector3d po(way_pose.position.x, way_pose.position.y, way_pose.position.z);
	double roll, pitch, yaw;
	geometry_quat_to_rpy(roll, pitch, yaw, way_pose.orientation);
	Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_rot = qua * po;
	po_rot += Eigen::Vector3d(0, width, 0);
	Eigen::Quaterniond qua_rev = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	Eigen::Vector3d po_move = qua_rev * po_rot;

	autoware_msgs::AvoidanceLaneInfo ret;
	ret.x = po_move.x();  ret.y = po_move.y();
	return ret;
}

class ObstacleAvoidance
{
public:
	//run関数の強制実行関連
	static const int FORCED_NO = 0;//avoidance_switchを使用
	static const int FORCED_AVOIDANCE = 1;//強制的に回避経路を作成
	static const int FORCED_RETURN = 2;//強制的に戻り経路を作成
private:
	static const double path_length = 50.0;//clothoidを適用する経路の長さ　現在は暫定的に固定値

	//回避作成スイッチ関連
	static const int AVOIDANCE_SWITCH_OFF = 0;//障害物回避なし
	static const int AVOIDANCE_SWITCH_GO_CREATING = 1;//障害物回避作成待ち
	static const int AVOIDANCE_SWITCH_GO_ON = 2;//障害物回避実行中
	static const int AVOIDANCE_SWITCH_RETURN_CREATE = 3;//戻り経路作成待ち
	static const int AVOIDANCE_SWITCH_RETURN_ON = 4;//戻り経路実行中

	ros::NodeHandle nh_, p_nh_;

	ros::Publisher pub_avoidance_lane_info_list_;
	ros::Subscriber sub_config_, sub_local_waypoints_, sub_no_curve_waypoints_, sub_avoidance_switch_;

	autoware_config_msgs::ConfigClothoidAvoidance config_;
	int avoidance_switch_;//障害物回避フラグ 詳細は上記のAVOIDANCE_SWITCH定数を参照
	std::vector<TableClothoidAndLane> table_clothoid_and_lane_;//clothoid長とレーン長の対応表
	autoware_msgs::Lane local_waypoints_, no_curve_waypoints_;
	autoware_msgs::AvoidanceLaneInfoList avoidance_list_;

	//回避切り替え関数
	void avoidSwitching(bool flag)
	{
		if(flag == true)//回避経路作成
		{
			if(avoidance_switch_ == AVOIDANCE_SWITCH_OFF || avoidance_switch_ == AVOIDANCE_SWITCH_RETURN_ON)
				avoidance_switch_ = AVOIDANCE_SWITCH_GO_CREATING;
		}
		else//戻り経路作成
		{
			if(avoidance_switch_ == AVOIDANCE_SWITCH_GO_ON)
			{
				avoidance_switch_ = AVOIDANCE_SWITCH_RETURN_CREATE;
			}
		}
	}

	int config_read_count_;
	void callbackConfig(const autoware_config_msgs::ConfigClothoidAvoidance &msg)
	{
		autoware_config_msgs::ConfigClothoidAvoidance prev_config = config_;
		config_ = msg;
		createCorrespondenceTable();

		config_read_count_++;
		if(config_read_count_ > 1)//autowareからノードを起動した場合、configが自動的に読み込まれるので、初回のsubscribeでは経路変更フラグを変更しない
		{
			if(config_.move_lane_length != prev_config.move_lane_length ||
				config_.phiU != prev_config.phiU ||
				config_.phiV != prev_config.phiV ||
				config_.step_count != prev_config.step_count)
			{
				if(avoidance_switch_ == AVOIDANCE_SWITCH_GO_ON) run(false, FORCED_AVOIDANCE);
				else if(avoidance_switch_ == AVOIDANCE_SWITCH_RETURN_ON) run(false, FORCED_RETURN);
			}
			else if(msg.avoid_flag == autoware_config_msgs::ConfigClothoidAvoidance::AVOID_GO)
				avoidSwitching(true);
			else avoidSwitching(false);
		}
		else config_.avoid_flag = autoware_config_msgs::ConfigClothoidAvoidance::AVOID_RETURN;
	}

	void callbackLocalWaypoints(const autoware_msgs::Lane &msg)
	{
		local_waypoints_ = msg;
	}

	void callbackNoCurveWaypoints(const autoware_msgs::Lane &msg)
	{
		no_curve_waypoints_ = msg;
	}

	//回避行動を行うかのフラグのcallback
	void callbackAvoidanceSwitch(const std_msgs::Bool &msg)
	{
		avoidSwitching(msg.data);
	}

	//clothoid曲線長と経路長との対応表作成
	//const double table_clothoid_length = 120.0;//clothoid曲線長の最大値
	//対応表はable_clothoid_and_lane_メンバ変数に入る
	const double table_step_ = 0.5;//曲線長ステップ値
	const double table_lane_length_ = 300.0;//レーン長の最大値
	bool table_create_flag;
	void createCorrespondenceTable()
	{
		if(table_create_flag == false)
		{
			table_clothoid_and_lane_.clear();
			//for(double h=table_step; h<=table_clothoid_length; h+=table_step)
			for(double h=table_step_; true; h+=table_step_)
			{
				//double clothoid_length = h * h / 8.0;
				std::vector<autoware_msgs::AvoidanceLaneInfo> xy_list = clothoid(config_, h, true);//clothoid_length);
				TableClothoidAndLane t;
				t.clothoid_length_ = h;
				t.lane_length_ = xy_list[xy_list.size()-1].x;//xy_list.end()->x;;
				table_clothoid_and_lane_.push_back(t);
				//std::fixed;
				//std::cout << std::setprecision(10) << t.clothoid_length_ << "," << t.lane_length_ << std::endl;
				if(t.lane_length_ > table_lane_length_) break;
			}
		}
		table_create_flag = true;
	}

	void avoidanceListClear()
	{
		avoidance_list_.move_width = 0;
		avoidance_list_.list.clear();
	}

	//waypointをclothoidS字にそって曲げた座標を作成
	//clothoid_length : 使用するclothoid曲線長
	//avoidance_flag : trueなら回避経路作成  falseなら戻り経路作成
	//start_index : local_waypointの経路作成位置
	//use_curve_waypoints : local_waypoints_を使用するならtrue  no_curve_waypoints_を使用するならfalse
	void waypointPlusClothoidWidth(const double clothoid_length, const bool avoidance_flag, const int start_index, const bool use_curve_waypoints)
	{
		autoware_msgs::Lane &lane = (use_curve_waypoints == true) ? local_waypoints_ : no_curve_waypoints_;
		avoidanceListClear();
		if(start_index+1 >= lane.waypoints.size())
		{
			emptyWaypointsInfoPublish(); return;
		}

		int sgn = (avoidance_flag == true) ? -1 : 1;

		Slope slope;
		slope.phi0 = 0 * M_PI / 180.0;
		slope.phiU = sgn * config_.phiU * M_PI / 180.0;
		slope.phiV = sgn * config_.phiV * M_PI / 180.0;

		float stepS = 1.0 / config_.step_count;
		std::complex<float> P_Vector;
		std::vector<autoware_msgs::AvoidanceLaneInfo> xy_list;
		autoware_msgs::AvoidanceLaneInfo center_xy;

		int counter = 0;
		int lind = start_index+1;
		double sum_waypoint_length = euclidDistance(lane.waypoints[lind-1].pose.pose.position, lane.waypoints[lind].pose.pose.position);
		std::vector<autoware_msgs::AvoidanceLaneInfo> avoid_info_tmp;
		for(int i = 0 ; i < config_.step_count ; i++)//S字前半
		{
			float S = stepS * i;

			std::complex<float> r;
			simpson_integral(slope, S, S + stepS, &r);
			P_Vector += r;

			float x = P_Vector.real();
			float y = P_Vector.imag();
			autoware_msgs::AvoidanceLaneInfo xy;// = {clothoid_length * x, clothoid_length * y, counter+1}; 
			counter++;
			xy.x = clothoid_length * x;  xy.y = clothoid_length * y;  //xy.ratio = counter;
			xy.waypoint_id = lane.waypoints[lind].waypoint_param.id;

			if(xy.x >= sum_waypoint_length)//clothoid座標がwaypoint長の総和を超えたら、座標を登録
			{
				autoware_msgs::Waypoint &way = lane.waypoints[lind];
				autoware_msgs::AvoidanceLaneInfo info = way_pose_convert(way.pose.pose, xy.y);
				info.waypoint_id = way.waypoint_param.id;
				info.ratio = counter;

				lind++;
				if(lind == lane.waypoints.size())
				{
					emptyWaypointsInfoPublish(); return;
				}
				sum_waypoint_length += euclidDistance(lane.waypoints[lind-1].pose.pose.position, lane.waypoints[lind].pose.pose.position);
				avoid_info_tmp.push_back(info);
				xy_list.push_back(xy);
			}

			if((sgn == 1 && xy.y > config_.move_lane_length / 2.0) || (sgn == (-1) &&  xy.y < -config_.move_lane_length / 2.0))
			{
				center_xy = xy;
				break;
			}
			xy_list.push_back(xy);
		}

		bool leak_flag = false;
		for(int i = xy_list.size() -1 ; i >= 0; i--)//S字後半
		{
			autoware_msgs::AvoidanceLaneInfo list = xy_list[i];
			double dtx = center_xy.x - list.x;
			double dty = center_xy.y - list.y;
			autoware_msgs::AvoidanceLaneInfo xyplus;// = {center_xy.x + dtx, center_xy.y + dty, counter+1};
			counter++;
			xyplus.x = center_xy.x + dtx;  xyplus.y = center_xy.y + dty;  xyplus.ratio = counter + 1;
			xyplus.waypoint_id = lane.waypoints[lind].waypoint_param.id;

			if(xyplus.x >= sum_waypoint_length)//clothoid座標がwaypoint長の総和を超えたら、座標を登録
			{
				autoware_msgs::Waypoint &way = lane.waypoints[lind];
				autoware_msgs::AvoidanceLaneInfo info = way_pose_convert(way.pose.pose, xyplus.y);
				info.waypoint_id = way.waypoint_param.id;
				info.ratio = counter;

				//中間点のwaypointが飛ばされることがあるので、その場合は前後で線形補間
				autoware_msgs::AvoidanceLaneInfo prev_info = avoid_info_tmp[avoid_info_tmp.size() - 1];
				int id_diff = info.waypoint_id - prev_info.waypoint_id;
				if(id_diff > 1)
				{
					leak_flag = true;
					double x_range = prev_info.x - info.x;
					double y_range = prev_info.y - info.y;
					std::cout << x_range << "," << y_range << std::endl;
					for(int j=1; j<id_diff; j++)
					{
						std::cout << prev_info.waypoint_id + 1 << std::endl;
						autoware_msgs::AvoidanceLaneInfo leak_info;
						leak_info.x = info.x + x_range * j / id_diff;
						leak_info.y = info.y + y_range * j / id_diff;
						leak_info.waypoint_id = prev_info.waypoint_id + 1;
						leak_info.ratio = prev_info.ratio;
						avoid_info_tmp.push_back(leak_info);
					}
				}

				lind++;
				if(lind == lane.waypoints.size())
				{
					emptyWaypointsInfoPublish(); return;
				}
				sum_waypoint_length += euclidDistance(lane.waypoints[lind-1].pose.pose.position, lane.waypoints[lind].pose.pose.position);
				avoid_info_tmp.push_back(info);

				/*int id1 = avoid_info_tmp[avoid_info_tmp.size() - 1].waypoint_id;
				int id2 = avoid_info_tmp[avoid_info_tmp.size() - 2].waypoint_id;
				int id_diff = id1 - id2;
				if(id_diff > 1) for(int j=id2+1; j<id1; j++) leak_id_list.push_back(j);*/
			}
		}

		if(avoidance_flag == true)
		{
			avoidance_list_.move_width = -config_.move_lane_length;
			avoidance_list_.status = autoware_msgs::AvoidanceLaneInfoList::STATUS_AVOIDANCE_GO;
		}
		else
		{
			avoidance_list_.move_width = 0;
			avoidance_list_.status = autoware_msgs::AvoidanceLaneInfoList::STATUS_AVOIDANCE_RETURN;
		}

		autoware_msgs::AvoidanceLaneInfo start_info;
		start_info.ratio = 0;
		start_info.waypoint_id = lane.waypoints[start_index].waypoint_param.id;
		start_info.x = lane.waypoints[start_index].pose.pose.position.x;
		start_info.y = lane.waypoints[start_index].pose.pose.position.y;
		avoidance_list_.list.push_back(start_info);
		for(int i=0; i<avoid_info_tmp.size(); i++)
		{
			autoware_msgs::AvoidanceLaneInfo info = avoid_info_tmp[i];
			info.ratio /= counter;
			avoidance_list_.list.push_back(info);
		}

		/*if(leak_flag == true)
		{
			for(int i=0;i<avoidance_list_.list.size();i++)
			{
				std::fixed;
				std::cout << std::setprecision(8) << avoidance_list_.list[i].waypoint_id << "," << avoidance_list_.list[i].x << "," << avoidance_list_.list[i].y << "," << avoidance_list_.list[i].ratio << std::endl;
			}
		}*/
	}

	//空の回避経路情報をpublish（回避しない）
	//stamp : タイムスタンプ
	//frame_id : 所属tfフレームの名前
	void emptyWaypointsInfoPublish()
	{
		autoware_msgs::AvoidanceLaneInfoList list;
		list.header.stamp = ros::Time(0);
		list.header.frame_id = "";
		list.status = autoware_msgs::AvoidanceLaneInfoList::STATUS_NORMAL;
		list.move_width = 0;
		pub_avoidance_lane_info_list_.publish(list);
	}

	//作成した回避経路情報をpublish
	void avoidWaypointsInfoPublish(ros::Time stamp)
	{
		ros::Time time = avoidance_list_.header.stamp;
		avoidance_list_.header.stamp = stamp;
		pub_avoidance_lane_info_list_.publish(avoidance_list_);
		avoidance_list_.header.stamp = time;
	}

	void avoidancePublisher(int start_index, bool use_curve_waypoints, ros::Time nowtime)
	{
		//対応表から使用するclothoid長を検索
		double clothoid_length = table_clothoid_and_lane_[table_clothoid_and_lane_.size()-1].clothoid_length_;
		for(int i=0;i<table_clothoid_and_lane_.size() ;i++)
		{
			if(table_clothoid_and_lane_[i].lane_length_ > ObstacleAvoidance::path_length)
			{
				clothoid_length = table_clothoid_and_lane_[i].clothoid_length_;
				break;
			}
		}

		waypointPlusClothoidWidth(clothoid_length, true, start_index, use_curve_waypoints);
		avoidWaypointsInfoPublish(nowtime);

		avoidance_switch_ = ObstacleAvoidance::AVOIDANCE_SWITCH_GO_ON;
	}

	void returnPublisher(int start_index, bool use_curve_waypoints, ros::Time nowtime)
	{
		double clothoid_length = table_clothoid_and_lane_[table_clothoid_and_lane_.size()-1].clothoid_length_;
		for(int i=0;i<table_clothoid_and_lane_.size() ;i++)
		{
			if(table_clothoid_and_lane_[i].lane_length_ > path_length)
			{
				clothoid_length = table_clothoid_and_lane_[i].clothoid_length_;
				break;
			}
		}

		waypointPlusClothoidWidth(clothoid_length, false, start_index, use_curve_waypoints);
		avoidWaypointsInfoPublish(nowtime);

		avoidance_switch_ = ObstacleAvoidance::AVOIDANCE_SWITCH_RETURN_ON;
	}
public:
	ObstacleAvoidance(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
		, avoidance_switch_(false)
		, table_create_flag(false)
		, config_read_count_(0)
	{
		pub_avoidance_lane_info_list_ = nh_.advertise<autoware_msgs::AvoidanceLaneInfoList>("/avoidance_lane_info_list", 1);
		sub_config_ = nh_.subscribe("/config/clothoid_avoidance", 1, &ObstacleAvoidance::callbackConfig, this);
		sub_local_waypoints_ = nh_.subscribe("/safety_waypoints", 1, &ObstacleAvoidance::callbackLocalWaypoints, this);
		sub_no_curve_waypoints_ = nh_.subscribe("/no_curve_waypoints", 1, &ObstacleAvoidance::callbackNoCurveWaypoints, this);
		sub_avoidance_switch_ = nh_.subscribe("/avoidance_switch", 1, &ObstacleAvoidance::callbackAvoidanceSwitch, this);

		//clothoid 初期パラメータ代入
		config_.move_lane_length = 1.5;////y終端位置（経路のy方向の終わり）
		config_.phiV = 30;//もし曲率が変化しない場合にh分進んだ場合のdeg角度増分(例えばφuが0なら、この曲線は円を描きます)
		config_.phiU = 500;//クロソイドによるdeg角度の増分
		config_.step_count = 5000;//計算ステップ数（あまり小さいと分解能が下がる）

		createCorrespondenceTable();
	}

	//use_curve_waypoints : local_waypoints_を使用するならtrue  no_curve_waypoints_を使用するならfalse
	//forced : 強制実行フラグ FORCED定数を参照
	void run(const bool use_curve_waypoints, int forced)
	{
		autoware_msgs::Lane &lane = (use_curve_waypoints == true) ? local_waypoints_ : no_curve_waypoints_;

		//クロソイド曲線で移動させる経路の長さpath_length文のクロソイドを計算
		const int start_index = 0;//回避経路を作成するwaypointのインデックス(1ならbase_linkの次)　現在は暫定的にbase_linkの次のwaypoint

		ros::Time nowtime = ros::Time::now();

		if(lane.waypoints.size() < 2)
		{
			emptyWaypointsInfoPublish();
		}

		//forcedに強制実行フラグがある場合の処理
		switch(forced)
		{
			case FORCED_AVOIDANCE:
				avoidancePublisher(start_index, use_curve_waypoints, nowtime);
				return;
			case FORCED_RETURN:
				returnPublisher(start_index, use_curve_waypoints, nowtime);
				return;
		}

		switch(avoidance_switch_)
		{
			case ObstacleAvoidance::AVOIDANCE_SWITCH_OFF:
				emptyWaypointsInfoPublish();
				break;
			case ObstacleAvoidance::AVOIDANCE_SWITCH_GO_CREATING:
				avoidancePublisher(start_index, use_curve_waypoints, nowtime);
				break;
			case ObstacleAvoidance::AVOIDANCE_SWITCH_GO_ON:
				avoidWaypointsInfoPublish(nowtime);
				break;
			case ObstacleAvoidance::AVOIDANCE_SWITCH_RETURN_CREATE:
				returnPublisher(start_index, use_curve_waypoints, nowtime);
				break;
			case ObstacleAvoidance::AVOIDANCE_SWITCH_RETURN_ON:
				avoidWaypointsInfoPublish(nowtime);
				break;
			default:
				emptyWaypointsInfoPublish();
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_avoidance");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	ObstacleAvoidance oa(nh, private_nh);
	ros::Rate rate(30);
	while(ros::ok())
	{
		ros::spinOnce();
		oa.run(true, ObstacleAvoidance::FORCED_NO);
		rate.sleep();
	}
	return 0;
}