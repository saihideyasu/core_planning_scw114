#ifndef WAYPOINT_PARAM_INIT
#define WAYPOINT_PARAM_INIT

#include <ros/ros.h>
#include <vector>
#include <autoware_msgs/Waypoint.h>
#include <unordered_map>

namespace waypoint_maker
{
	void waypoint_param_init(autoware_msgs::Waypoint *wp, std::unordered_map<std::string, std::string> map, const unsigned int id_counter)
	{
		wp->pose.pose.position.x = std::stod(map["x"]);
		wp->pose.pose.position.y = std::stod(map["y"]);
		wp->pose.pose.position.z = std::stod(map["z"]);
		wp->pose.pose.orientation = (map.find("yaw") != map.end()) ? tf::createQuaternionMsgFromYaw(std::stod(map["yaw"])) : tf::createQuaternionMsgFromYaw(0);
		wp->twist.twist.linear.x = kmph2mps(std::stod(map["velocity"]));
		wp->change_flag = std::stoi(map["change_flag"]);
		wp->wpstate.steering_state = (map.find("steering_flag") != map.end()) ? std::stoi(map["steering_flag"]) : 0;
		wp->wpstate.accel_state = (map.find("accel_flag") != map.end()) ? std::stoi(map["accel_flag"]) : 0;
		wp->wpstate.stop_state = (map.find("stop_flag") != map.end()) ? std::stoi(map["stop_flag"]) : 0;
		wp->wpstate.event_state = (map.find("event_flag") != map.end()) ? std::stoi(map["event_flag"]) : 0;

		wp->waypoint_param.id = id_counter;  //id_counter_++;
		wp->waypoint_param.weight = (map.find("weight") != map.end()) ? std::stof(map["weight"]) : 0;
		wp->waypoint_param.feat_proj_x = (map.find("feat_proj_x") != map.end()) ? std::stof(map["feat_proj_x"]) : -10000;
		wp->waypoint_param.feat_proj_y = (map.find("feat_proj_y") != map.end()) ? std::stof(map["feat_proj_y"]) : -10000;
		wp->waypoint_param.blinker = (map.find("blinker") != map.end()) ? std::stof(map["blinker"]) : -10000;
		wp->waypoint_param.velocity_KPPlus = (map.find("velocity_KPPlus") != map.end()) ? std::stof(map["velocity_KPPlus"]) : -1;
		wp->waypoint_param.velocity_KPMinus = (map.find("velocity_KPMinus") != map.end()) ? std::stof(map["velocity_KPMinus"]) : -1;
		wp->waypoint_param.velocity_punchPlus = (map.find("velocity_punchPlus") != map.end()) ? std::stof(map["velocity_punchPlus"]) : -1;
		wp->waypoint_param.velocity_punchMinus = (map.find("velocity_punchMinus") != map.end()) ? std::stof(map["velocity_punchMinus"]) : -1;
		wp->waypoint_param.velocity_windowPlus = (map.find("velocity_windowPlus") != map.end()) ? std::stof(map["velocity_windowPlus"]) : -1;
		wp->waypoint_param.velocity_windowMinus = (map.find("velocity_windowMinus") != map.end()) ? std::stof(map["velocity_windowMinus"]) : -1;
		wp->waypoint_param.drive_stroke = (map.find("drive_stroke") != map.end()) ? std::stof(map["drive_stroke"]) : -1;
		wp->waypoint_param.brake_stroke = (map.find("brake_stroke") != map.end()) ? std::stof(map["brake_stroke"]) : -1;
		wp->waypoint_param.brake_stroke = (map.find("mb_pedal") != map.end()) ? std::stoi(map["mb_pedal"]) : 0;
		wp->waypoint_param.pause = (map.find("pause") != map.end()) ? std::stof(map["pause"]) : 0;
		wp->waypoint_param.vgf_leafsize = (map.find("vgf_leafsize") != map.end()) ? std::stof(map["vgf_leafsize"]) : -1;
		wp->waypoint_param.vgf_measurement_range = (map.find("vgf_measurement_range") != map.end()) ? std::stof(map["vgf_measurement_range"]) : -1;
		wp->waypoint_param.automatic_door = (char)((map.find("automatic_door") != map.end()) ? std::stoi(map["automatic_door"]) : 0);
		wp->waypoint_param.signal_stop_line = (char)((map.find("signal_stop_line") != map.end()) ? std::stoi(map["signal_stop_line"]) : 0);
		wp->waypoint_param.temporary_stop_line = ((map.find("temporary_stop_line") != map.end()) ? std::stod(map["temporary_stop_line"]) : 0);
		wp->waypoint_param.temporary_fixed_velocity = ((map.find("temporary_fixed_velocity") != map.end()) ? std::stod(map["temporary_fixed_velocity"]) : 0);
		wp->waypoint_param.period_signal_time_first = ((map.find("period_signal_time_first") != map.end()) ? map["period_signal_time_first"] : "");
		wp->waypoint_param.period_signal_time_step_green = ((map.find("period_signal_time_step_green") != map.end()) ? std::stod(map["period_signal_time_step_green"]) : 0);
		wp->waypoint_param.period_signal_time_step_yellow = ((map.find("period_signal_time_step_yellow") != map.end()) ? std::stod(map["period_signal_time_step_yellow"]) : 0);
		wp->waypoint_param.period_signal_time_step_red = ((map.find("period_signal_time_step_red") != map.end()) ? std::stod(map["period_signal_time_step_red"]) : 0);
		wp->waypoint_param.fusion_select = (char)((map.find("fusion_select") != map.end()) ? std::stoi(map["fusion_select"]) : -1);
		wp->waypoint_param.liesse.shift = (char)((map.find("liesse_shift") != map.end()) ? std::stoi(map["liesse_shift"]) : -1);
		wp->waypoint_param.steer_correction = ((map.find("steer_correction") != map.end()) ? std::stod(map["steer_correction"]) : 1.0);
		wp->waypoint_param.lookahead_ratio = ((map.find("lookahead_ratio") != map.end()) ? std::stod(map["lookahead_ratio"]) : 0.0);
		wp->waypoint_param.minimum_lookahead_distance = ((map.find("minimum_lookahead_distance") != map.end()) ? std::stod(map["minimum_lookahead_distance"]) : 0.0);
		wp->waypoint_param.lookahead_ratio_magn = ((map.find("lookahead_ratio_magn") != map.end()) ? std::stod(map["lookahead_ratio_magn"]) : -1.0);
		wp->waypoint_param.steer_pid_on = ((map.find("steer_pid_on") != map.end()) ? std::stoi(map["steer_pid_on"]) : 1);
		wp->waypoint_param.ndt_yaw_correction = ((map.find("ndt_yaw_correction") != map.end()) ? std::stod(map["ndt_yaw_correction"]) : -100);
		wp->waypoint_param.gnss_yaw_correction = ((map.find("gnss_yaw_correction") != map.end()) ? std::stod(map["gnss_yaw_correction"]) : -100);
		wp->waypoint_param.localizer_check = ((map.find("localizer_check") != map.end()) ? std::stoi(map["localizer_check"]) : -1);
		wp->waypoint_param.accel_stroke_offset = ((map.find("accel_stroke_offset") != map.end()) ? std::stoi(map["accel_stroke_offset"]) : -1);
		wp->waypoint_param.temporary_deceleration = ((map.find("temporary_deceleration") != map.end()) ? std::stod(map["temporary_deceleration"]) : -1);
		wp->waypoint_param.accel_avoidance_distance_min = ((map.find("accel_avoidance_distance_min") != map.end()) ? std::stod(map["accel_avoidance_distance_min"]) : -1);
		wp->waypoint_param.stop_stroke_max = ((map.find("stop_stroke_max") != map.end()) ? std::stod(map["stop_stroke_max"]) : -1);
		wp->waypoint_param.accel_stroke_max = ((map.find("accel_stroke_max") != map.end()) ? std::stod(map["accel_stroke_max"]) : -1);
		wp->waypoint_param.k_accel_p_velocity = ((map.find("k_accel_p_velocity") != map.end()) ? std::stod(map["k_accel_p_velocity"]) : -1);
		wp->waypoint_param.k_accel_i_velocity = ((map.find("k_accel_i_velocity") != map.end()) ? std::stod(map["k_accel_i_velocity"]) : -1);
		wp->waypoint_param.k_accel_d_velocity = ((map.find("k_accel_d_velocity") != map.end()) ? std::stod(map["k_accel_d_velocity"]) : -1);
		wp->waypoint_param.k_brake_p_velocity = ((map.find("k_brake_p_velocity") != map.end()) ? std::stod(map["k_brake_p_velocity"]) : -1);
		wp->waypoint_param.k_brake_i_velocity = ((map.find("k_brake_i_velocity") != map.end()) ? std::stod(map["k_brake_i_velocity"]) : -1);
		wp->waypoint_param.k_brake_d_velocity = ((map.find("k_brake_d_velocity") != map.end()) ? std::stod(map["k_brake_d_velocity"]) : -1);
		wp->waypoint_param.in_accel_mode = ((map.find("in_accel_mode") != map.end()) ? std::stoi(map["in_accel_mode"]) : 1);
		wp->waypoint_param.in_brake_mode = ((map.find("in_brake_mode") != map.end()) ? std::stoi(map["in_brake_mode"]) : 1);
		wp->waypoint_param.use_stopper_distance = ((map.find("use_stopper_distance") != map.end()) ? std::stoi(map["use_stopper_distance"]) : 1);
		wp->waypoint_param.stopper_distance1 = ((map.find("stopper_distance1") != map.end()) ? std::stod(map["stopper_distance1"]) : -1);
		wp->waypoint_param.stopper_distance2 = ((map.find("stopper_distance2") != map.end()) ? std::stod(map["stopper_distance2"]) : -1);
		wp->waypoint_param.stopper_distance3 = ((map.find("stopper_distance3") != map.end()) ? std::stod(map["stopper_distance3"]) : -1);
		wp->waypoint_param.detection_use_point_cloud = ((map.find("use_point_cloud") != map.end()) ? std::stoi(map["use_point_cloud"]) : -1);
		wp->waypoint_param.detection_use_point_pillar = ((map.find("use_point_pillar") != map.end()) ? std::stoi(map["use_point_pillar"]) : -1);
		wp->waypoint_param.detection_use_mobileye = ((map.find("use_mobileye") != map.end()) ? std::stoi(map["use_mobileye"]) : -1);
		wp->waypoint_param.steer_deg = ((map.find("steer_deg") != map.end()) ? std::stod(map["steer_deg"]) : 0);
		wp->waypoint_param.steer_deg_acc = ((map.find("steer_deg_acc") != map.end()) ? std::stod(map["steer_deg_acc"]) : 0);
		wp->waypoint_param.obstacle_deceleration = ((map.find("obstacle_deceleration") != map.end()) ? std::stod(map["obstacle_deceleration"]) : -1);
		wp->waypoint_param.lane_rule_deceleration = ((map.find("lane_rule_deceleration") != map.end()) ? std::stod(map["lane_rule_deceleration"]) : -1);
		wp->waypoint_param.position_adjustment_magn = ((map.find("position_adjustment") != map.end()) ? std::stod(map["position_adjustment"]) : 0);
		wp->waypoint_param.intersection_id = ((map.find("intersection_id") != map.end()) ? std::stoi(map["intersection_id"]) : -1);
		wp->waypoint_param.routes_id = ((map.find("routes_id") != map.end()) ? std::stoi(map["routes_id"]) : -1);
		wp->waypoint_param.blue_arrow_flag = ((map.find("blue_arrow_flag") != map.end()) ? std::stoi(map["blue_arrow_flag"]) : 0);
		wp->waypoint_param.accel_stroke_step_max = ((map.find("accel_stroke_step_max") != map.end()) ? std::stod(map["accel_stroke_step_max"]) : -1);
		wp->waypoint_param.accel_stroke_step_min = ((map.find("accel_stroke_step_min") != map.end()) ? std::stod(map["accel_stroke_step_min"]) : -1);
		wp->waypoint_param.brake_stroke_step_max = ((map.find("brake_stroke_step_max") != map.end()) ? std::stod(map["brake_stroke_step_max"]) : -1);
		wp->waypoint_param.brake_stroke_step_min = ((map.find("brake_stroke_step_min") != map.end()) ? std::stod(map["brake_stroke_step_min"]) : -1);
		wp->waypoint_param.mpc_prediction_horizon = ((map.find("mpc_prediction_horizon") != map.end()) ? std::stoi(map["mpc_prediction_horizon"]) : -1);
		wp->waypoint_param.mpc_moving_average_number_of_times = ((map.find("mpc_moving_average_number_of_times") != map.end()) ? std::stoi(map["mpc_moving_average_number_of_times"]) : -1);
		wp->waypoint_param.mpc_curvature_smoothing_number = ((map.find("mpc_curvature_smoothing_number") != map.end()) ? std::stoi(map["mpc_curvature_smoothing_number"]) : -1);
		wp->waypoint_param.mpc_ctrl_period = ((map.find("mpc_ctrl_period") != map.end()) ? std::stod(map["mpc_ctrl_period"]) : -1);
		wp->waypoint_param.mpc_wheelbase = ((map.find("mpc_wheelbase") != map.end()) ? std::stod(map["mpc_wheelbase"]) : -1);
		wp->waypoint_param.steer_overwrite = ((map.find("steer_overwrite") != map.end()) ? std::stod(map["steer_overwrite"]) : -100000);
	}

	void waypoint_param_init(autoware_msgs::Waypoint *wp, const unsigned int id_counter)
	{
		std::unordered_map<std::string, std::string> map;
		waypoint_param_init(wp, map, id_counter);
	}
}

#endif