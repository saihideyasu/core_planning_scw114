#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_config_msgs/ConfigCmdSelector.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/WaypointParam.h>

class CmdPublisher
{
public:
	static int publish_select_;
private:
	ros::NodeHandle nh_, p_nh_;
	ros::Subscriber sub_twist_, sub_ctrl_;
	ros::Publisher pub_tiwst_, pub_ctrl_;

	int instance_num_;

	void callbackTwist(const geometry_msgs::TwistStamped &msg)
	{
		if(publish_select_ == instance_num_) pub_tiwst_.publish(msg);
	}

	void callbackCtrl(const autoware_msgs::ControlCommandStamped &msg)
	{
		if(publish_select_ == instance_num_) pub_ctrl_.publish(msg);
	}
public:
	CmdPublisher(int instance_num, ros::NodeHandle nh, ros::NodeHandle p_nh, std::string twist_cmd, std::string ctrl_cmd)
		: instance_num_(instance_num)
		, nh_(nh)
		, p_nh_(p_nh)
	{
		pub_tiwst_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_raw", 1);
		pub_ctrl_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_raw", 1);
		sub_twist_ = nh_.subscribe(twist_cmd, 10 , &CmdPublisher::callbackTwist, this);
		sub_ctrl_ = nh_.subscribe(ctrl_cmd, 10 , &CmdPublisher::callbackCtrl, this);
		std::cout << publish_select_ << "," << instance_num_ << std::endl;
	}

	CmdPublisher(const CmdPublisher &obj)
		: pub_tiwst_(obj.pub_tiwst_)
		, pub_ctrl_(obj.pub_ctrl_)
		, sub_twist_(obj.sub_twist_)
		, sub_ctrl_(obj.sub_ctrl_)
		, instance_num_(obj.instance_num_)
	{
	}

	CmdPublisher operator =(const CmdPublisher &obj)
	{
		pub_tiwst_ = obj.pub_tiwst_;
		pub_ctrl_ = obj.pub_ctrl_;
		sub_twist_ = obj.sub_twist_;
		sub_ctrl_ = obj.sub_ctrl_;
		instance_num_ = obj.instance_num_;
	}
};

int CmdPublisher::publish_select_ = 0;

class CmdSelector
{
private:
	static const int MAX_SELECT_NUM_ = 2;

	ros::NodeHandle nh_, p_nh_;
	ros::Publisher pub_select_;
	ros::Subscriber sub_config_, sub_waypoint_param_;

	CmdPublisher *cmd_publisher_[MAX_SELECT_NUM_];

	void publishSelect()
	{
		std_msgs::Int32 val;
		val.data = CmdPublisher::publish_select_;
		pub_select_.publish(val);
	}

	void callbackConfig(const autoware_config_msgs::ConfigCmdSelector &msg)
	{
		CmdPublisher::publish_select_ = atoi(msg.cmd_select.c_str());
		publishSelect();
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam &msg)
	{
		if(msg.cmd_select > 0 && msg.cmd_select <= MAX_SELECT_NUM_)
		{
			CmdPublisher::publish_select_ = msg.cmd_select;
			publishSelect();
		}
	}
public:
	CmdSelector(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
	{
		CmdPublisher::publish_select_ = 1;

		for(int i=0; i<MAX_SELECT_NUM_; i++)
		{
			std::string twist, ctrl;
			std::stringstream ss_twist;
			ss_twist << "twist_raw" << i+1;
			p_nh_.param<std::string>(ss_twist.str(), twist, "");
			std::stringstream ss_ctrl;
			ss_ctrl << "ctrl_raw" << i+1;
			p_nh_.param<std::string>(ss_ctrl.str(), ctrl, "");

			cmd_publisher_[i] = new CmdPublisher(i+1, nh_, p_nh_, twist, ctrl);
		}

		pub_select_ = nh_.advertise<std_msgs::Int32>("/cmd_selector/select", 1, true);
		sub_config_ = nh.subscribe("/config/cmd_selector", 10 , &CmdSelector::callbackConfig, this);
		sub_waypoint_param_ = nh.subscribe("/waypoint_param", 10 , &CmdSelector::callbackWaypointParam, this);
		publishSelect();
	}

	~CmdSelector()
	{
		for(int i=0; i<MAX_SELECT_NUM_; i++) delete cmd_publisher_[i];
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cmd_selector");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	CmdSelector selector(nh, private_nh);
	ros::spin();
	return 0;
}