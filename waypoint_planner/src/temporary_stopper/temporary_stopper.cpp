#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_config_msgs/ConfigTemporaryStopper.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <visualization_msgs/MarkerArray.h>

class TemporaryStopper
{
private:
	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_waypoint_, sub_waypoint_param_, sub_can503_, sub_current_velocity_, sub_distance_, sub_config_;
	ros::Publisher pub_waypoint_, pub_temporary_line_, pub_temporary_distance_, pub_temporari_flag_, pub_temporary_fixed_velocity_;
	ros::Publisher pub_stop_waypoint_id_;

	autoware_config_msgs::ConfigTemporaryStopper config_;
	autoware_can_msgs::MicroBusCan503 can503_;
	bool can503_read_;
	geometry_msgs::TwistStamped current_velocity_;

	double front_bumper_to_baselink_;
	uint32_t stop_waypoint_id_;
	ros::Time timer_;
	double stop_time_;
	autoware_msgs::StopperDistance distance_;
	double fixed_velocity_;

	autoware_msgs::Lane apply_acceleration(const autoware_msgs::Lane& lane, double acceleration, int start_index,
										   size_t fixed_cnt, double fixed_vel, bool reverse_flag, double velocity_limit)
	{
	  autoware_msgs::Lane l = lane;

	  if (fixed_cnt == 0)
		return l;

	  double square_vel = fixed_vel * fixed_vel;
	  double distance = 0;
	  for (int i = start_index; i < l.waypoints.size(); ++i)
	  {
		if (i - start_index < fixed_cnt)
		{
			l.waypoints[i].twist.twist.linear.x = fixed_vel;
			continue;
		}

		geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
		geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
		distance += hypot(b.x - a.x, b.y - a.y);

		double v = sqrt(square_vel + 2 * acceleration * distance);
		if (v < l.waypoints[i].twist.twist.linear.x)
		{
			//if(fixed_vel < v) l.waypoints[i].twist.twist.linear.x = v;
			//else l.waypoints[i].twist.twist.linear.x = fixed_vel;
			if(velocity_limit < 0) l.waypoints[i].twist.twist.linear.x = v;//速度の最大値がない場合(velocity_limitがマイナス)
			else if(v > velocity_limit) l.waypoints[i].twist.twist.linear.x = velocity_limit;//vがvelocity_limitを超えている場合は、速度をvelocity_limitにする
			else l.waypoints[i].twist.twist.linear.x = v;
	  	}
		else if(reverse_flag == true)//停止線より下側は中断処理をしない
			break;
	  }

	  return l;
	}

	autoware_msgs::Lane apply_decceleration(const autoware_msgs::Lane& lane, double decceleration, int start_index,
										   size_t fixed_cnt, double fixed_vel)
	{
	  autoware_msgs::Lane l = lane;
	  if(decceleration > 0) 
		return l;
	  if (fixed_cnt == 0)
		return l;

	  double square_vel = fixed_vel * fixed_vel;
	  double distance = 0;
	  for (int i = start_index; i > 0; --i)
	  {
		if (i > start_index -fixed_cnt)
		{
			l.waypoints[i].twist.twist.linear.x = fixed_vel;
			continue;
		}

		geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
		geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
		distance += hypot(b.x - a.x, b.y - a.y);
	
		double v = sqrt(square_vel - 2 * decceleration * distance);
		std::cout << v << std::endl;
		if (v < l.waypoints[i].twist.twist.linear.x)
			l.waypoints[i].twist.twist.linear.x = v;
		else
			break;
	  }

	  return l;
	}

	int stop_line_search(const autoware_msgs::Lane& way)
	{
		std_msgs::Int32 dis;
		dis.data = -1;

		if(way.waypoints.size() == 0 || way.waypoints[0].waypoint_param.id < 5)
		{
			pub_temporary_distance_.publish(dis);
			return -1;
		}

		for(int i=0; i<way.waypoints.size() || i<config_.search_distance; i++)
		{
			//std::cout << "i," << (int)way.waypoints[i].waypoint_param.temporary_stop_line << std::endl;
			if(way.waypoints[i].waypoint_param.temporary_stop_line > 0)
			{
				stop_time_ = way.waypoints[i].waypoint_param.temporary_stop_line;
				fixed_velocity_ = way.waypoints[i].waypoint_param.temporary_fixed_velocity / 3.6;

				visualization_msgs::Marker marker;
				marker.header.frame_id = "/map";
				marker.ns = "Temporary Line";
				marker.id = 0;
				marker.type = 1;
				marker.action = 0;
				marker.pose = way.waypoints[i].pose.pose;
				marker.pose.position.z += 1;
				marker.scale.x = 0.1;
				marker.scale.y = 15;
				marker.scale.z = 2;
				marker.color.r = 0;
				marker.color.g = 0;
				marker.color.b = 1;
				marker.color.a = 0.3;
				marker.frame_locked = true;
				marker.mesh_use_embedded_materials = false;
				marker.lifetime = ros::Duration(1.0);
				visualization_msgs::MarkerArray array;
				array.markers.push_back(marker);
				pub_temporary_line_.publish(array);

				if(fixed_velocity_ <= 0) dis.data = i;
				else dis.data = -1;
				//dis.data = i;
				pub_temporary_distance_.publish(dis);
				pub_temporary_fixed_velocity_.publish(fixed_velocity_);

				return i;
			}
		}

		pub_temporary_distance_.publish(dis);
		return -1;
	}

	autoware_msgs::Lane apply_stopline_acceleration(const autoware_msgs::Lane& lane, double acceleration,
													int ahead_cnt, int behind_cnt)
	{
		std_msgs::Int32 flag;

		int stop_index = stop_line_search(lane); std::cout << stop_index << std::endl;
		if(stop_index < 0)
		{
			stop_waypoint_id_ = 0;
			flag.data = 0;//そのまま
			pub_temporari_flag_.publish(flag);
			timer_ = ros::Time::now();
			std_msgs::Int32 stop_id_ret;
			stop_id_ret.data = -1;
			pub_stop_waypoint_id_.publish(stop_id_ret);
			return lane;
		}

		std_msgs::Int32 stop_id_ret;
		stop_id_ret.data = lane.waypoints[stop_index].waypoint_param.id;
		pub_stop_waypoint_id_.publish(stop_id_ret);
		
		if(can503_.clutch == false) timer_ = ros::Time(0);//手動運転の場合はTimer処理はしない

		ros::Time now_time = ros::Time::now();
		if(timer_ < now_time)
		{
//			std::cout << "distance_1 : " << distance_ << std::endl;
			if(stop_waypoint_id_ == lane.waypoints[stop_index].waypoint_param.id)
			{
				//std_msgs::Int32 tmp;
				//tmp.data = stop_waypoint_id_;
				//pub_stop_waypoint_id_.publish(tmp);

				flag.data = 2;
				pub_temporari_flag_.publish(flag);
				return lane;//停止判定終了
			}
			else
			{
				stop_waypoint_id_ = 0;
				//if(can_.velocity_actual <= config_.stop_speed_threshold)   //幅を持たせなくてよいか？
				if(current_velocity_.twist.linear.x <= config_.stop_speed_threshold
						&& distance_.distance <= 0.5 && distance_.distance >=0
						&& distance_.send_process == autoware_msgs::StopperDistance::TEMPORARY_STOPPER)
				{
					stop_waypoint_id_ = lane.waypoints[stop_index].waypoint_param.id;
					//timer_ = ros::Time(now_time.sec + (int)stop_time_, now_time.nsec);
					ros::Duration ros_stop_time(stop_time_);
					if(can503_.clutch == true) timer_ = now_time + ros_stop_time;
					else timer_ = ros::Time(0);//手動運転の場合はTimer処理はしない
					//std::cout << "distance_0 : " << distance_ << std::endl;
					std::cout << "time : " << timer_.sec << "," << now_time.sec << std::endl;
				}
				flag.data = 1;//停止判定あり
				pub_temporari_flag_.publish(flag);
			}
		}
		//else if(can503_read_ == true && can503_.clutch == false) timer_ = ros::Time(0);//停止状態でクラッチが切れている（手動）場合、停止処理を強制終了

		// front バンパーが一時停止線にあるときに、ベースリンクに一番近い2個のwaypointのうち前のもの。
		// ≒ stop_index - 5
		double stop_line_to_baselink = 0;
		int stop_line_to_baselink_index;
		for(int cou=stop_index; cou>=1; cou--)
		{
			double x1 = lane.waypoints[cou].pose.pose.position.x;
			double y1 = lane.waypoints[cou].pose.pose.position.y;
			double x2 = lane.waypoints[cou-1].pose.pose.position.x;
			double y2 = lane.waypoints[cou-1].pose.pose.position.y;
			double sx = x1-x2,  sy = y1-y2;
			stop_line_to_baselink += sqrt(sx*sx + sy*sy);
			if(stop_line_to_baselink >= front_bumper_to_baselink_)
			{
				stop_line_to_baselink_index = cou;//+1;
				break;
			}
		}
		if(stop_line_to_baselink < front_bumper_to_baselink_ && fixed_velocity_ <= 0)
		{
			autoware_msgs::Lane l = lane;
			l.waypoints[0].twist.twist.linear.x = 0;
			return l;
		}

		autoware_msgs::Lane l;
		l = apply_acceleration(lane, acceleration, stop_line_to_baselink_index /*stop_index*/, behind_cnt + 1, fixed_velocity_, false, -1);
		std::reverse(l.waypoints.begin(), l.waypoints.end());
		int reverse_stop_index = l.waypoints.size() - stop_line_to_baselink_index - 1;
		l = apply_acceleration(l, acceleration, reverse_stop_index, ahead_cnt + 1, fixed_velocity_, true, config_.velocity_limit / 3.6);
		std::reverse(l.waypoints.begin(), l.waypoints.end());

		return l;
	}

	void callbackCan503(const autoware_can_msgs::MicroBusCan503& msg)
	{
		can503_ = msg;
		can503_read_ = true;
	}

	void callbackDistance(const autoware_msgs::StopperDistance& msg)
	{
		distance_ = msg;
	}

	void callbackCurrentVelocity(const geometry_msgs::TwistStamped& msg)
	{
		current_velocity_ = msg;
	}

	void callbackConfig(const autoware_config_msgs::ConfigTemporaryStopper& msg)
	{
		config_ = msg;
		fixed_velocity_ = config_.fixed_velocity / 3.6;
		//std::cout << config_.fixed_velocity << std::endl;

		std_msgs::Float64 fixed_vel;
		fixed_vel.data = fixed_velocity_;
		pub_temporary_fixed_velocity_.publish(fixed_velocity_);
	}

	void callbackWaypoint(const autoware_msgs::Lane& msg)
	{
		autoware_msgs::Lane lane = apply_stopline_acceleration(msg, config_.deceleration,
									config_.number_of_zeros_ahead, config_.number_of_zeros_behind);
		pub_waypoint_.publish(lane);
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam& msg)
	{
		if(msg.temporary_deceleration >= 0.0)
			config_.deceleration = msg.temporary_deceleration;
		//std::cout << "aaa : " << msg.temporary_fixed_velocity << std::endl;
		/*if(msg.temporary_fixed_velocity >= 0.0)
		{
			fixed_velocity_ = msg.temporary_fixed_velocity;
			fixed_velocity_ /= 3.6;
			std_msgs::Float64 fixed_vel;
			fixed_vel.data = fixed_velocity_;
			pub_temporary_fixed_velocity_.publish(fixed_velocity_);
		}*/
	}
public:
	TemporaryStopper(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: stop_waypoint_id_(100)//0
		, stop_time_(5.0)
		, fixed_velocity_(0)
		, can503_read_(false)
	{
		nh_ = nh;  private_nh_ = p_nh;

		/*config_.search_distance = 7;
		config_.deceleration = 1;
		config_.number_of_zeros_ahead = 0;
		config_.number_of_zeros_behind = 5;
		config_.stop_speed_threshold = 0.018;*/

		timer_ = ros::Time::now();

		nh_.param<double>("/vehicle_info/front_bumper_to_baselink", front_bumper_to_baselink_, 4.55);
		pub_waypoint_ = nh_.advertise<autoware_msgs::Lane>("/temporary_stop_waypoints", 1);
		//pub_waypoint_ = nh_.advertise<autoware_msgs::Lane>("/final_waypoints", 1);
		pub_temporary_line_ = nh_.advertise<visualization_msgs::MarkerArray>("/temporary_line", 1);
		pub_temporary_distance_ = nh_.advertise<std_msgs::Int32>("/temporary_distance", 1);
		pub_temporari_flag_ = nh_.advertise<std_msgs::Int32>("/temporary_flag", 1);
		pub_temporary_fixed_velocity_ = nh_.advertise<std_msgs::Float64>("/temporary_fixed_velocity", 1, true);
		pub_stop_waypoint_id_ = nh_.advertise<std_msgs::Int32>("/stop_waypoint_id", 1);

		//sub_waypoint_ = nh_.subscribe("/curve_appropriate_waypoints", 1, &TemporaryStopper::callbackWaypoint, this);
		sub_waypoint_ = nh_.subscribe("/safety_waypoints", 1, &TemporaryStopper::callbackWaypoint, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 1, &TemporaryStopper::callbackWaypointParam, this);
		sub_can503_ = nh_.subscribe("/microbus/can_receive503", 1, &TemporaryStopper::callbackCan503, this);
		sub_current_velocity_ = nh_.subscribe("/current_velocity", 1, &TemporaryStopper::callbackCurrentVelocity, this);
		sub_distance_ = nh_.subscribe("/stopper_distance", 1, &TemporaryStopper::callbackDistance, this);
		sub_config_ = nh_.subscribe("/config/temporary_stopper", 1, &TemporaryStopper::callbackConfig, this);
		
		distance_.distance = 1000;

		std_msgs::Float64 fixed_vel;
		fixed_vel.data = fixed_velocity_;
		pub_temporary_fixed_velocity_.publish(fixed_velocity_);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "temporary_stopper");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	TemporaryStopper ts(nh, private_nh);
	ros::Rate rate(100);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}