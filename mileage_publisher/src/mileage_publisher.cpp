#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/GnssSurfaceSpeed.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_msgs/AutoMileage.h>
#include <autoware_config_msgs/ConfigMileagePublisher.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <limits.h>

const double simpsons_rule(double a, double fa, double b, double fb, double fm)
{
	double front = (b-a) / 6.0;
	double back = fa + 4.0*fm + fb;
	return front * back;
}

std::vector<std::string> split(const std::string &string, const char sep)
{
	std::vector<std::string> str_vec_ptr;
	std::string token;
	std::stringstream ss(string);

	while (getline(ss, token, sep))
		str_vec_ptr.push_back(token);

	return str_vec_ptr;
}

double euclidDistance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
	double x = p1.x - p2.x;
	double y = p1.y - p2.y;
	double z = p1.z - p2.z;
	return sqrt(x*x + y*y + z*z);
}

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

double math_way_current_distance(double lineX1, double lineY1, double lineX2, double lineY2,
						double pointX, double pointY)
{
	double x1 = lineX1, x2 = lineX2;
	double y1 = lineY1, y2 = lineY2;
	double a = y2 - y1;
	double b = x1 - x2;
	double c = - x1 * y2 + y1 * x2;

	//double x0 = current_pose_.pose.position.x, y0 = current_pose_.pose.position.y;
	double x0 = pointX, y0 = pointY;
	double db = sqrt(a * a + b * b);
	if(db == 0)
	{
		return 0;
	}
	return (a * x0 + b * y0 + c) / db;
}

struct WayCurrentDistance
{
	int way_id_;
	double distance_;
};

class MileagePublisher
{
private:
	const double CUR_BASE_DISTANCE_TH = 3.0;//current_poseとbase_waypointsの先頭がこの距離(m)離れていた場合、処理をしない

	ros::NodeHandle nh_, p_nh_;

	ros::Publisher pub_distance_, pub_way_increase_distance_, pub_way_current_distance_all_;
	ros::Subscriber sub_config_, sub_microbus_can502_, sub_microbus_can503_, sub_odom_,  sub_base_waypoints_, sub_current_pose_;
	ros::Subscriber sub_gnss_speed_;

	autoware_config_msgs::ConfigMileagePublisher config_;
	autoware_can_msgs::MicroBusCan502 microbus_can502_;//microbusのcan情報（ステアと速度）
	autoware_can_msgs::MicroBusCan503 microbus_can503_;//microbusのcan情報（ペダル）
	bool recording_flag_;//記録開始フラグ
	double running_distance_;//現在の自動走行時の走行距離
	double running_distance_all_;//全部の自動走行時の走行距離
	nav_msgs::Odometry odometry_;//オドメトリー情報
	autoware_msgs::GnssSurfaceSpeed gnss_speed_;//gnssの速度

	double automode_mileage_; //自動走行時の経路走行距離
	autoware_msgs::Lane base_waypoints_;//単一レーンのglobal waypoints(lane_selectノードの/base_waypointsトピックを想定)
	geometry_msgs::Point prev_way_current_pose_;//前回のcurrent_poseと経路の交差点
	double automode_mileage_distance_;//自動経路走行距離

	void callbackConfig(const autoware_config_msgs::ConfigMileagePublisher &msg)
	{
		config_ = msg;
	}

	void callbackMicrobusCan502(const autoware_can_msgs::MicroBusCan502 &msg)
	{
		if(msg.clutch == true && microbus_can502_.clutch == false)
		{
			recording_flag_ = true;
		}
		if(msg.clutch == false && microbus_can502_.clutch == true)
		{
			if(microbus_can503_.clutch == false)
			{
				ros::Time nowtime = ros::Time::now();
				recording_flag_ = false;
				runningDistanceAllWrite(nowtime);
				automodeMileageWrite(nowtime);
				running_distance_ = 0;
				automode_mileage_distance_ = 0;

				std_msgs::Float64 ret;
				ret.data = 0;
				pub_way_current_distance_all_.publish(ret);
			}
		}
		microbus_can502_ = msg;
	}

	void callbackMicrobusCan503(const autoware_can_msgs::MicroBusCan503 &msg)
	{
		if(msg.clutch == true && microbus_can503_.clutch == false)
		{
			recording_flag_ = true;
		}
		if(msg.clutch == false && microbus_can503_.clutch == true)
		{
			if(microbus_can502_.clutch == false)
			{
				ros::Time nowtime = ros::Time::now();
				recording_flag_ = false;
				runningDistanceAllWrite(nowtime);
				automodeMileageWrite(nowtime);
				running_distance_ = 0;
				automode_mileage_distance_ = 0;

				std_msgs::Float64 ret;
				ret.data = 0;
				pub_way_current_distance_all_.publish(ret);
			}
		}
		microbus_can503_ = msg;
	}

	void running_distance_add(ros::Time current_time, ros::Time prev_time, double current_speed, double prev_speed)
	{
		ros::Duration ros_time_diff = current_time - prev_time;
		double time_diff = ros_time_diff.sec + ros_time_diff.nsec * 1E-9;

		double va = 0,  vb = time_diff;
		double vfa = prev_speed,  vfb = current_speed;
		double vpm = (vfa + vfb) / 2.0;
		double distance_v = simpsons_rule(va, vfa, vb, vfb, vpm);
		running_distance_ += distance_v;
		running_distance_all_ += distance_v;
	}

	void callbackOdom(const nav_msgs::Odometry &msg)
	{
		if(recording_flag_ == true && config_.use_velocity == autoware_config_msgs::ConfigMileagePublisher::USE_VELOCITY_ODOMETRY)
			running_distance_add(msg.header.stamp, odometry_.header.stamp, msg.twist.twist.linear.x, odometry_.twist.twist.linear.x);
		odometry_ = msg;
	}

	void callbackGnssSpeed(const autoware_msgs::GnssSurfaceSpeed &msg)
	{
		if(recording_flag_ == true && config_.use_velocity == autoware_config_msgs::ConfigMileagePublisher::USE_VELOCITY_GNSS_SPEED)
			running_distance_add(msg.header.stamp, gnss_speed_.header.stamp, msg.surface_speed, gnss_speed_.surface_speed);
		gnss_speed_ = msg;
	}

	void runningDistanceAllRead()
	{
		running_distance_all_ = 0;
		std::string all_distance_path = ros::package::getPath("runtime_manager") + "/all_autorunning_distance.csv";
		std::ifstream file_dis(all_distance_path, std::ios_base::in);
		if(file_dis)
		{
			while(!file_dis.eof())
			{
				std::string line;
				std::getline(file_dis, line);
				std::istringstream iss(line);
				std::string str;
				iss >> str;
				std::vector<std::string> fields = split(str, ',');//std::cout << fields.size() << std::endl;
				if(fields.size() == 2)
				{
					//std::cout << fields[1] << std::endl;
					double val = atof(fields[1].c_str());
					running_distance_all_ += val;
				}
			}
		}
	}

	void runningDistanceAllWrite(ros::Time nowtime)
	{
		char time_str[30];
		time_t t = nowtime.sec;
		strftime(time_str, sizeof(time_str), "%Y/%m/%d_%a_%H:%M:%S", localtime(&t));
		std::string path = ros::package::getPath("runtime_manager") + "/all_autorunning_distance.csv";
		std::ofstream file_dis(path, std::ios_base::app);
		file_dis.write(time_str, strlen(time_str));
		std::stringstream ss;
		ss << "," << running_distance_ << "\n";
		file_dis.write(ss.str().c_str(), strlen(ss.str().c_str()));
		file_dis.close();
	}

	void automodeMileageWrite(ros::Time nowtime)
	{
		char time_str[30];
		time_t t = nowtime.sec;
		strftime(time_str, sizeof(time_str), "%Y/%m/%d_%a_%H:%M:%S", localtime(&t));
		std::string path = ros::package::getPath("runtime_manager") + "/automode_mileage.csv";
		std::ofstream ofs(path, std::ios_base::app);
		ofs.write(time_str, strlen(time_str));
		std::stringstream ss;
		ss << "," << automode_mileage_distance_ << "\n";
		ofs.write(ss.str().c_str(), strlen(ss.str().c_str()));
		ofs.close();
	}

	void callbackBaseWaypoints(const autoware_msgs::Lane &msg)
	{
		base_waypoints_ = msg;
	}

	void callbackCurrentPose(const geometry_msgs::PoseStamped &msg)
	{
		//global_waypointsのサイズが2以下、もしくはodometryの速度が0の場合は処理しない
		if(base_waypoints_.waypoints.size() < 2 || odometry_.twist.twist.linear.x == 0)
			return;
		//current_poseとbase_waypointsの先頭がこの距離(m)離れていた場合、処理をしない
		if(euclidDistance(msg.pose.position, base_waypoints_.waypoints[0].pose.pose.position) > CUR_BASE_DISTANCE_TH)
			return;

		//current_poseに一番近いwaypointの探索
		double min_distance = DBL_MAX;
		int min_ind = -1;
		for(int ind=0; ind<base_waypoints_.waypoints.size(); ind++)
		{
			geometry_msgs::Point way_pose = base_waypoints_.waypoints[ind].pose.pose.position;
			double dis = euclidDistance(msg.pose.position, way_pose);
			if(min_distance > dis)
			{
				min_distance = dis;
				min_ind = ind;
			}
		}

		//curent_poseに一番近い経路上の交差点を探索
		geometry_msgs::Point base_point = base_waypoints_.waypoints[min_ind].pose.pose.position;
		geometry_msgs::Point second_point;
		geometry_msgs::Point way_current_pose;//交差点
		if(min_ind == 0)
		{
			second_point = base_waypoints_.waypoints[min_ind+1].pose.pose.position;
		}
		else if(min_ind == base_waypoints_.waypoints.size()-1)
		{
			second_point = base_waypoints_.waypoints[min_ind-1].pose.pose.position;
		}
		else
		{
			geometry_msgs::Point front_point = base_waypoints_.waypoints[min_ind+1].pose.pose.position;
			geometry_msgs::Point back_point = base_waypoints_.waypoints[min_ind-1].pose.pose.position;
			double front_distance = euclidDistance(front_point, base_point);
			double back_distance = euclidDistance(back_point, base_point);
			second_point = (front_distance < back_distance) ? front_point : back_point;
		}
		math_way_current_point(base_point.x, base_point.y, second_point.x, second_point.y,
						 msg.pose.position.x, msg.pose.position.y, &way_current_pose.x, &way_current_pose.y);
		way_current_pose.z = base_point.z;

		//進行した距離を出力
		if(recording_flag_ == true)
		{
			if(prev_way_current_pose_.x != 0 || prev_way_current_pose_.y != 0 || prev_way_current_pose_.z !=0)
			{
				double dt = euclidDistance(way_current_pose, prev_way_current_pose_);
				automode_mileage_distance_ += dt;

				std_msgs::Float64 way_dt;
				way_dt.data = dt;
				pub_way_increase_distance_.publish(way_dt);

				std_msgs::Float64 way_distance;
				way_distance.data = automode_mileage_distance_;
				pub_way_current_distance_all_.publish(way_distance);
			}
		}

		prev_way_current_pose_ = way_current_pose;
	}
public:
	MileagePublisher(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
		, recording_flag_(false)
		, running_distance_(0)
		, automode_mileage_(0)
		, automode_mileage_distance_(0)
	{
		pub_distance_ = nh_.advertise<autoware_msgs::AutoMileage>("/auto_running_distance", 1);
		pub_way_increase_distance_ = nh_.advertise<std_msgs::Float64>("/way_increase_distance", 1);
		pub_way_current_distance_all_ = nh_.advertise<std_msgs::Float64>("/way_current_distance_all", 1);

		sub_config_ = nh_.subscribe("/config/mileage_publisher", 10, &MileagePublisher::callbackConfig, this);
		sub_microbus_can502_ = nh_.subscribe("/microbus/can_receive502", 10, &MileagePublisher::callbackMicrobusCan502, this);
		sub_microbus_can503_ = nh_.subscribe("/microbus/can_receive503", 10, &MileagePublisher::callbackMicrobusCan503, this);
		sub_odom_ = nh_.subscribe("/vehicle/odom", 10, &MileagePublisher::callbackOdom, this);
		sub_gnss_speed_ = nh_.subscribe("/nmea2tfpose_RTK2/gnss_surface_speed", 10, &MileagePublisher::callbackGnssSpeed, this);
		sub_base_waypoints_ = nh_.subscribe("/base_waypoints", 10, &MileagePublisher::callbackBaseWaypoints, this);
		sub_current_pose_ = nh_.subscribe("/current_pose", 10, &MileagePublisher::callbackCurrentPose, this);

		runningDistanceAllRead();
		prev_way_current_pose_.x = prev_way_current_pose_.y = prev_way_current_pose_.z = 0;
	}

	void publishDistance()
	{
		ros::Time nowtime = ros::Time::now();
		autoware_msgs::AutoMileage ret;
		ret.header.stamp = nowtime;
		ret.auto_mileage = running_distance_;
		ret.auto_mileage_all = running_distance_all_;
		pub_distance_.publish(ret);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mileage_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	MileagePublisher am(nh, private_nh);

	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		am.publishDistance();
		rate.sleep();
	}
	return 0;
}