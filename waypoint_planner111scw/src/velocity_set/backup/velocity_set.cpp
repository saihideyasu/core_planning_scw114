/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>

#include "../../src/velocity_set/libvelocity_set.h"
#include "../../src/velocity_set/velocity_set_info.h"
#include "../../src/velocity_set/velocity_set_path.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace
{

tf::TransformListener *listener;
tf::TransformBroadcaster *br;

constexpr int LOOP_RATE = 10;
constexpr double DECELERATION_SEARCH_DISTANCE = 30;
constexpr double STOP_SEARCH_DISTANCE = 60;

autoware_msgs::DetectedObjectArray object_tracker;

void objectTrackerCallback(const autoware_msgs::DetectedObjectArrayConstPtr& msg)
{
  object_tracker = *msg;
  //std::cout << "get daze" << std::endl;
}

void obstacleColorByKind(const EControl kind, std_msgs::ColorRGBA &color, const double alpha=0.5)
{
  if (kind == EControl::STOP)
  {
    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = alpha;  // red
  }
  else if (kind == EControl::STOPLINE)
  {
    color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = alpha;  // blue
  }
  else if (kind == EControl::DECELERATE)
  {
    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = alpha;  // yellow
  }
  else
  {
    color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = alpha;  // white
  }
}

// Display a detected obstacle
void displayObstacle(const EControl& kind, const ObstaclePoints& obstacle_points, const ros::Publisher& obstacle_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  static geometry_msgs::Point prev_obstacle_point;
  if (kind == EControl::STOP || kind == EControl::STOPLINE || kind == EControl::DECELERATE)
  {
    marker.pose.position = obstacle_points.getObstaclePoint(kind);
    prev_obstacle_point = marker.pose.position;
  }
  else  // kind == OTHERS
  {
    marker.pose.position = prev_obstacle_point;
  }
  geometry_msgs::Quaternion quat;
  marker.pose.orientation = quat;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;
  obstacleColorByKind(kind, marker.color, 0.7);

  obstacle_pub.publish(marker);
}

void displayDetectionRange(const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, const int closest_waypoint,
                           const EControl& kind, const int obstacle_waypoint, const double stop_range,
                           const double deceleration_range, const ros::Publisher& detection_range_pub)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker crosswalk_marker;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  crosswalk_marker.header.frame_id = "/map";
  crosswalk_marker.header.stamp = ros::Time();
  crosswalk_marker.id = 0;
  crosswalk_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  crosswalk_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker_stop = crosswalk_marker;
  waypoint_marker_decelerate = crosswalk_marker;
  stop_line = crosswalk_marker;
  stop_line.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  crosswalk_marker.ns = "Crosswalk Detection";
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";

  // set scale and color
  double scale = 2 * stop_range;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (stop_range + deceleration_range);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  if (obstacle_waypoint > -1)
  {
    stop_line.pose.position = lane.waypoints[obstacle_waypoint].pose.pose.position;
    stop_line.pose.orientation = lane.waypoints[obstacle_waypoint].pose.pose.orientation;
  }
  stop_line.pose.position.z += 1.0;
  stop_line.scale.x = 0.1;
  stop_line.scale.y = 15.0;
  stop_line.scale.z = 2.0;
  stop_line.lifetime = ros::Duration(0.1);
  stop_line.frame_locked = true;
  obstacleColorByKind(kind, stop_line.color, 0.3);

  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  if (crosswalk_id > 0)
    scale = crosswalk.getDetectionPoints(crosswalk_id).width;
  crosswalk_marker.scale.x = scale;
  crosswalk_marker.scale.y = scale;
  crosswalk_marker.scale.z = scale;
  crosswalk_marker.color.a = 0.5;
  crosswalk_marker.color.r = 0.0;
  crosswalk_marker.color.g = 1.0;
  crosswalk_marker.color.b = 0.0;
  crosswalk_marker.frame_locked = true;

  // set marker points coordinate
  for (int i = 0; i < STOP_SEARCH_DISTANCE; i++)
  {
    if (closest_waypoint < 0 || i + closest_waypoint > static_cast<int>(lane.waypoints.size()) - 1)
      break;

    geometry_msgs::Point point;
    point = lane.waypoints[closest_waypoint + i].pose.pose.position;

    waypoint_marker_stop.points.push_back(point);

    if (i > DECELERATION_SEARCH_DISTANCE)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  if (crosswalk_id > 0)
  {
    if (!crosswalk.isMultipleDetection())
    {
      for (const auto& p : crosswalk.getDetectionPoints(crosswalk_id).points)
        crosswalk_marker.points.push_back(p);
    }
    else
    {
      for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
      {
        for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
        {
          scale = crosswalk.getDetectionPoints(c_id).width;
          crosswalk_marker.points.push_back(p);
        }
      }
    }
  }
  // publish marker
  marker_array.markers.push_back(crosswalk_marker);
  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  if (kind != EControl::KEEP)
    marker_array.markers.push_back(stop_line);
  detection_range_pub.publish(marker_array);
  marker_array.markers.clear();
}

// obstacle detection for crosswalk
EControl crossWalkDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const CrossWalk& crosswalk,
                            const geometry_msgs::PoseStamped& localizer_pose, const int points_threshold,
                            ObstaclePoints* obstacle_points)
{
  int crosswalk_id = crosswalk.getDetectionCrossWalkID();
  double search_radius = crosswalk.getDetectionPoints(crosswalk_id).width / 2;
  // std::vector<int> crosswalk_ids crosswalk.getDetectionCrossWalkIDs();

  // Search each calculated points in the crosswalk
  for (const auto& c_id : crosswalk.getDetectionCrossWalkIDs())
  {
    for (const auto& p : crosswalk.getDetectionPoints(c_id).points)
    {
      geometry_msgs::Point detection_point = calcRelativeCoordinate(p, localizer_pose.pose);
      tf::Vector3 detection_vector = point2vector(detection_point);
      detection_vector.setZ(0.0);

      int stop_count = 0;  // the number of points in the detection area
      for (const auto& p : points)
      {
        tf::Vector3 point_vector(p.x, p.y, 0.0);
        double distance = tf::tfDistance(point_vector, detection_vector);
        if (distance < search_radius)
        {
          stop_count++;
          geometry_msgs::Point point_temp;
          point_temp.x = p.x;
          point_temp.y = p.y;
          point_temp.z = p.z;
          obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
        }
        if (stop_count > points_threshold)
          return EControl::STOP;
      }
    }

    obstacle_points->clearStopPoints();
    if (!crosswalk.isMultipleDetection())
      break;
  }
  return EControl::KEEP;  // find no obstacles
}

int detectStopObstacle(const pcl::PointCloud<pcl::PointXYZ>& points,
                       const std::vector<mobileye_560_660_msgs::ObstacleData>& mobileye_obstacle, const int closest_waypoint,
                       const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, double stop_range,
                       double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                       ObstaclePoints* obstacle_points, int* obstacle_type,
                       const int wpidx_detection_result_by_other_nodes, bool enable_mobileye,
                       double *pillar_velocity, const ros::Publisher detection_moblieye_pub, double *mobileye_velocity,
                       const bool use_point_cloud, const bool use_point_pillar, const bool use_mobileye)
{
  int stop_obstacle_waypoint = -1;
  int ob_type = (int)EObstacleType::NONE;
  int end_waypoint = closest_waypoint + STOP_SEARCH_DISTANCE;
  bool check_point = false, check_pillar = false, check_mobileye = false;

  tf::StampedTransform mobileye_transform;
  listener->lookupTransform("map", "me_viz", ros::Time(0), mobileye_transform);
  ros::Time nowtime = ros::Time::now();
  //std::cout << "mobileye xyz," << mobileye_transform.getOrigin().getX() << "," << mobileye_transform.getOrigin().getY() << "," << mobileye_transform.getOrigin().getZ() << std::endl;
  
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + STOP_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    if(!check_point && !check_pillar && !check_mobileye)
    {
      // detection another nodes
      if (wpidx_detection_result_by_other_nodes >= 0 &&
          lane.waypoints.at(i).gid == wpidx_detection_result_by_other_nodes)
      {
        stop_obstacle_waypoint = i;
        ob_type = (int)EObstacleType::STOPLINE;
        obstacle_points->setStopPoint(lane.waypoints.at(i).pose.pose.position); // for vizuialization
        break;
      }

      // Detection for cross walk
      if (i == crosswalk.getDetectionWaypoint())
      {
        // found an obstacle in the cross walk
        if (crossWalkDetection(points, crosswalk, localizer_pose, points_threshold, obstacle_points) == EControl::STOP)
        {
        stop_obstacle_waypoint = i;
        ob_type = (int)EObstacleType::ON_CROSSWALK;
        break;
        }
      }
    }

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    if(!check_point && use_point_cloud)
    {
      int stop_point_count = 0;
      for (const auto& p : points)
      {
        tf::Vector3 point_vector(p.x, p.y, 0);

        // 2D distance between waypoint and points (obstacle)
        double dt = tf::tfDistance(point_vector, tf_waypoint);
        if (dt < stop_range)
        {
        stop_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
        }
      }

      // there is an obstacle if the number of points exceeded the threshold
      if (stop_point_count > points_threshold)
      {
        if(!check_pillar) stop_obstacle_waypoint = i;
        ob_type |= (int)EObstacleType::ON_WAYPOINTS;
        //break;
        check_point = true;
        end_waypoint = i + 1;
        if(end_waypoint > closest_waypoint + STOP_SEARCH_DISTANCE)
          end_waypoint = closest_waypoint + STOP_SEARCH_DISTANCE;
      }
    }

  	obstacle_points->clearStopPoints();

  	double min_dt = 100000;
	  if(!check_pillar && !check_mobileye && use_point_pillar)
    {
      //point pillar
      for(int obj_i=0; obj_i<object_tracker.objects.size(); obj_i++)
      {
        double height = object_tracker.objects[obj_i].dimensions.z;
        double width = object_tracker.objects[obj_i].dimensions.y;
        double length = object_tracker.objects[obj_i].dimensions.x;
        geometry_msgs::Pose pose = object_tracker.objects[obj_i].pose;

        const int mesh = 20;
        for(int cou1=0; cou1<mesh; cou1++)
        {
          for(int cou2=0; cou2<mesh; cou2++)
          {
            for(int cou3=0; cou3<mesh; cou3++)
            {
              double x = pose.position.x + (double)cou3 * length / (double)(mesh-1) - length/2.0;
              double y = pose.position.y + (double)cou2 * width / (double)(mesh-1) - width/2.0;
              double z = pose.position.z + (double)cou1 * height / (double)(mesh-1) - height/2.0;
              tf::Vector3 point_pillar_vector(x,y,0);
              double dt = tf::tfDistance(point_pillar_vector, tf_waypoint);
              if(dt < min_dt) {min_dt = dt;}
              if (dt < stop_range)
              {
                //std::cout << "pp : x," << point_pillar_vector_map.getX() << " y," << point_pillar_vector_map.getY() << " z," << point_pillar_vector_map.getZ() << std::endl;
                ob_type |= (int)EObstacleType::ON_POINT_PILLAR;
                *pillar_velocity = object_tracker.objects[obj_i].velocity.linear.x;
                goto JMP_PILLAR;
              }
            }
          }
        }
      }
      JMP_PILLAR:;

      std::cout << "min_dt : " << min_dt << "," << ob_type << std::endl;
      if(ob_type & (int)EObstacleType::ON_POINT_PILLAR)
      {
        //if(!check_point) stop_obstacle_waypoint = i;
        if(stop_obstacle_waypoint < 0) stop_obstacle_waypoint = i;
        //break;
        check_pillar = true;
        end_waypoint = i + 1;
        if(end_waypoint > closest_waypoint + STOP_SEARCH_DISTANCE)
          end_waypoint = closest_waypoint + STOP_SEARCH_DISTANCE;
      }
    }

    if(!check_mobileye && use_mobileye)
    {
      /*double x = lane.waypoints[i].pose.pose.position.x;// + mobileye_transform.getOrigin().getX();
      double y = lane.waypoints[i].pose.pose.position.y;// + mobileye_transform.getOrigin().getY();
      double z = lane.waypoints[i].pose.pose.position.z;// + mobileye_transform.getOrigin().getZ();
      tf::Vector3 tf_waypoint_mobileye(x,y,0);//z);*/

      /*for(int obj_i=0; obj_i<mobileye_obstacle.size(); obj_i++)
      {
        mobileye_560_660_msgs::ObstacleData mobileye_obj = mobileye_obstacle[obj_i];

        for(double wid=-mobileye_obj.obstacle_width/2; wid<mobileye_obj.obstacle_width/2; wid+=0.1)
        {
          tf::Vector3 mobileye_vector(mobileye_obj.obstacle_pos_x+wid , mobileye_obj.obstacle_pos_y, 0);

          double dt = tf::tfDistance(mobileye_vector, tf_waypoint);
          if (dt < stop_range)
          {std::cout << "dt," << dt << "," << stop_range << std::endl;
            detection_moblieye_pub.publish(mobileye_obj);
            ob_type |= (int)EObstacleType::ON_MOBILEYE;
            *mobileye_velocity = mobileye_obj.obstacle_rel_vel_x;
            break;
          }
        }*/
      
      for(int obj_i=0; obj_i<mobileye_obstacle.size(); obj_i++)
      {
        std::cout << "obj_i," << obj_i << std::endl;
        mobileye_560_660_msgs::ObstacleData mobileye_obj = mobileye_obstacle[obj_i];
        tf::Transform tf_detction;
        tf::Vector3 xyz;
        xyz.setX(mobileye_obj.obstacle_pos_x); xyz.setY(mobileye_obj.obstacle_pos_y);
        tf_detction.setOrigin(tf::Vector3(mobileye_obj.obstacle_pos_x, mobileye_obj.obstacle_pos_y, 0));
        tf_detction.setRotation(tf::Quaternion::getIdentity());

        std::stringstream str;
        str << "m_detect" << obj_i;
        br->sendTransform(tf::StampedTransform(tf_detction, nowtime, "me_viz", str.str().c_str()));

        tf::StampedTransform tf_detect;
        ros::Time t = ros::Time::now();
        listener->waitForTransform("map", str.str().c_str(), nowtime, ros::Duration(3.0));
        listener->lookupTransform("map", str.str().c_str(), nowtime, tf_detect);
      
        /*if(ob_type & (int)EObstacleType::ON_MOBILEYE)
        {
          if(!check_point) stop_obstacle_waypoint = i;
          check_mobileye = true;
          end_waypoint = i + 1;
          if(end_waypoint > closest_waypoint + STOP_SEARCH_DISTANCE)
            end_waypoint = closest_waypoint + STOP_SEARCH_DISTANCE;
        }*/
      }
    }

    /*static int count = 0;
    for(const auto& m : mobileye_obstacle)
    {
      for(double wid=-m.obstacle_width/2; wid<m.obstacle_width/2; wid+=0.1)
      {
        tf::Vector3 mobileye_vector(m.obstacle_pos_x+wid , m.obstacle_pos_y, 0);

        double dt = tf::tfDistance(mobileye_vector, tf_waypoint);
        if (dt < stop_range)
        {
          stop_obstacle_waypoint = i;
          ob_type = EObstacleType::ON_WAYPOINTS;
          if(count == 0 && stop_obstacle_waypoint - closest_waypoint < 23)
          {
            //system("/home/autoware/lane_change2.sh");
            count++;
          }
          break;
        }
      }
      if(stop_obstacle_waypoint >= 0) break;

      // 2D distance between waypoint and points (obstacle)
      tf::Vector3 mobileye_vector(m.obstacle_pos_x , m.obstacle_pos_y, 0);

      double dt = tf::tfDistance(mobileye_vector, tf_waypoint);
      if (dt < stop_range)
      {
        stop_obstacle_waypoint = i;
        ob_type = EObstacleType::ON_WAYPOINTS;
      }
    }*/

      // check next waypoint...
    }

    *obstacle_type = ob_type;
    return stop_obstacle_waypoint;
  }

int detectDecelerateObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                             const autoware_msgs::Lane& lane, const double stop_range, const double deceleration_range,
                             const double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                             ObstaclePoints* obstacle_points)
{
  int decelerate_obstacle_waypoint = -1;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + DECELERATION_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

	// point cloud
    int decelerate_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt > stop_range && dt < stop_range + deceleration_range)
      {
        decelerate_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setDeceleratePoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (decelerate_point_count > points_threshold)
    {
      decelerate_obstacle_waypoint = i;
      break;
    }

    obstacle_points->clearDeceleratePoints();

    // check next waypoint...
  }

  return decelerate_obstacle_waypoint;
}

// Detect an obstacle by using pointcloud
EControl pointsDetection(const pcl::PointCloud<pcl::PointXYZ>& points,
                         const std::vector<mobileye_560_660_msgs::ObstacleData>& mobileye_obstacle, const int closest_waypoint,
                         const autoware_msgs::Lane& lane, const CrossWalk& crosswalk, const VelocitySetInfo& vs_info,
                         int* obstacle_waypoint, ObstaclePoints* obstacle_points, int *obstacle_type, bool enableMobileye,
                         const VelocitySetPath vs_path, double *pillar_velocity, double *mobileye_velocity ,const ros::Publisher& pillar_velocity_pub,
                         const ros::Publisher mobileye_velocity_pub, const ros::Publisher detection_mobileye_pub)
{
  // no input for detection || no closest waypoint
  //std::cout << vs_info.getDetectionResultByOtherNodes() << "," << vs_info.getMobileyeObstacle().size() << "," << closest_waypoint << std::endl;
  if ((points.empty() == true && vs_info.getDetectionResultByOtherNodes() == -1
        && (object_tracker.objects.size() == 0 || vs_info.getUsePointPillar() == false)
        && (vs_info.getMobileyeObstacle().size() == 0 || vs_info.getUseMobileye() == false))
        || closest_waypoint < 0)
	  return EControl::KEEP;
  int ob_type = (int)EObstacleType::NONE;
  int stop_obstacle_waypoint =
      detectStopObstacle(points, mobileye_obstacle, closest_waypoint, lane, crosswalk, vs_info.getStopRange(),
                         vs_info.getPointsThreshold(), vs_info.getLocalizerPose(),
                         obstacle_points, &ob_type, vs_info.getDetectionResultByOtherNodes(), enableMobileye,
                         pillar_velocity, detection_mobileye_pub, mobileye_velocity,
                         vs_info.getUsePointCloud(), vs_info.getUsePointPillar(), vs_info.getUseMobileye());
  *obstacle_type = ob_type;

  // skip searching deceleration range
  if (vs_info.getDecelerationRange() < 0.01)
  {
	  *obstacle_waypoint = stop_obstacle_waypoint;
	  if (stop_obstacle_waypoint < 0)
		return EControl::KEEP;
	  //else if (ob_type == (int)EObstacleType::ON_WAYPOINTS || ob_type == (int)EObstacleType::ON_CROSSWALK || ob_type == (int)EObstacleType::ON_POINT_PILLAR)
	  if(ob_type == (int)EObstacleType::ON_CROSSWALK)
		return EControl::STOP;
	  else if (ob_type == (int)EObstacleType::STOPLINE)
		return EControl::STOPLINE;
    else if(ob_type & (int)EObstacleType::ON_MOBILEYE)
    {
        std_msgs::Float64 vel;
			  vel.data = *pillar_velocity;
			  mobileye_velocity_pub.publish(vel);
        if(0 >= *mobileye_velocity)
				  return EControl::KEEP;
			  else return EControl::STOP;
    }
	  else if(ob_type & (int)EObstacleType::ON_POINT_PILLAR)
    {
		  	std_msgs::Float64 vel;
			  vel.data = *pillar_velocity;
			  pillar_velocity_pub.publish(vel);
			  //if(vs_path.getCurrentVelocity() < *pillar_velocity)
			  if(0 >= *pillar_velocity)
				  return EControl::KEEP;
			  else return EControl::STOP;
    }
	  else if(ob_type & (int)EObstacleType::ON_WAYPOINTS)
	  {
		  /*if(ob_type & (int)EObstacleType::ON_POINT_PILLAR)
		  {
			  std_msgs::Float64 vel;
			  vel.data = *pillar_velocity;
			  pillar_velocity_pub.publish(vel);
			  //if(vs_path.getCurrentVelocity() < *pillar_velocity)
			  if(0 >= *pillar_velocity)
				  return EControl::KEEP;
			  else return EControl::STOP;
		  }
      else if(ob_type & (int)EObstacleType::ON_MOBILEYE)
      {
        std_msgs::Float64 vel;
			  vel.data = *pillar_velocity;
			  mobileye_velocity_pub.publish(vel);
        if(0 >= *mobileye_velocity)
				  return EControl::KEEP;
			  else return EControl::STOP;
      }
		  else*/ return EControl::STOP;
	  }
	  else
		return EControl::OTHERS;
  }

  int decelerate_obstacle_waypoint =
      detectDecelerateObstacle(points, closest_waypoint, lane, vs_info.getStopRange(), vs_info.getDecelerationRange(),
                               vs_info.getPointsThreshold(), vs_info.getLocalizerPose(), obstacle_points);

  // stop obstacle was not found
  if (stop_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return decelerate_obstacle_waypoint < 0 ? EControl::KEEP : EControl::DECELERATE;
  }

  // stop obstacle was found but decelerate obstacle was not found
  if (decelerate_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }

  // about 5.0 meter
  double waypoint_interval =
      getPlaneDistance(lane.waypoints[0].pose.pose.position, lane.waypoints[1].pose.pose.position);
  int stop_decelerate_threshold = 5 / waypoint_interval;

  // both were found
  if (stop_obstacle_waypoint - decelerate_obstacle_waypoint > stop_decelerate_threshold)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return EControl::DECELERATE;
  }
  else
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }
}

void obstacleTypeView(const int obstacle_type, const ros::Publisher obstacle_type_pub,
                      const double pillar_velocity, const double mobileye_velocity)
{
	jsk_rviz_plugins::OverlayText obstacle_type_text;
	obstacle_type_text.action = 0;
	obstacle_type_text.width = 800;
	obstacle_type_text.height = 100;
	obstacle_type_text.left = 10;
	obstacle_type_text.top = 10;
	obstacle_type_text.bg_color.r = 0.0;
	obstacle_type_text.bg_color.g = 0.0;
	obstacle_type_text.bg_color.b = 0.0;
	obstacle_type_text.bg_color.a = 0.20000000298;
	obstacle_type_text.line_width = 2;
	obstacle_type_text.text_size = 16;
	obstacle_type_text.font = "";
	obstacle_type_text.fg_color.r = 0.0;
	obstacle_type_text.fg_color.g = 1.0;
	obstacle_type_text.fg_color.b = 0.0;
	obstacle_type_text.fg_color.a = 1.0;

	switch(obstacle_type)
	{
	    case (int)EObstacleType::ON_CROSSWALK:
		    obstacle_type_text.text = "CROSSWALK";
		    break;
	    case (int)EObstacleType::STOPLINE:
		    obstacle_type_text.text = "STOPLINE";
		    break;
	    case (int)EObstacleType::ON_WAYPOINTS:
		    obstacle_type_text.text = "WAYPOINTS";
		    break;
	    case (int)EObstacleType::ON_POINT_PILLAR:
	    {
		    std::stringstream str;
			str << "POINT PILLAR : pillar velocity " << pillar_velocity * 3.6 << std::endl;
			obstacle_type_text.text = str.str();
			break;
	    }
	    case (int)EObstacleType::ON_WAYPOINTS + (int)EObstacleType::ON_POINT_PILLAR:
	    {
		    std::stringstream str;
			  str << "WAYPOINT & POINT PILLAR : pillar velocity " << pillar_velocity * 3.6 << std::endl;
			  obstacle_type_text.text = str.str();
			  break;
	    }
      case (int)EObstacleType::ON_MOBILEYE:
	    {
		    std::stringstream str;
			  str << "MOBILEYE : mobileye velocity " << mobileye_velocity * 3.6 << std::endl;
			  obstacle_type_text.text = str.str();
			  break;
	    }
	    case (int)EObstacleType::ON_WAYPOINTS + (int)EObstacleType::ON_MOBILEYE:
	    {
		    std::stringstream str;
			  str << "WAYPOINT & MOBILEYE : mobileye velocity " << mobileye_velocity * 3.6 << std::endl;
			  obstacle_type_text.text = str.str();
			  break;
	    }
	}
	obstacle_type_pub.publish(obstacle_type_text);
}

EControl obstacleDetection(int closest_waypoint, const autoware_msgs::Lane& lane, const CrossWalk& crosswalk,
                           const VelocitySetPath vs_path, const VelocitySetInfo vs_info,
                           const ros::Publisher& detection_range_pub, const ros::Publisher& obstacle_pub,
                           const ros::Publisher detection_mobileye_pub,
                           const ros::Publisher& pillar_velocity_pub, const ros::Publisher mobileye_velocity_pub,
                           int* obstacle_waypoint, bool enableMobileye, const ros::Publisher obstacle_type_pub)
{
  ObstaclePoints obstacle_points;
  int obstacle_type;
  double pillar_velocity, mobileye_velocity;
  EControl detection_result = pointsDetection(vs_info.getPoints(), vs_info.getMobileyeObstacle(),
                                              closest_waypoint, lane, crosswalk, vs_info,
                                              obstacle_waypoint, &obstacle_points, &obstacle_type, enableMobileye,
                                              vs_path, &pillar_velocity, &mobileye_velocity, pillar_velocity_pub, mobileye_velocity_pub, detection_mobileye_pub);
  displayDetectionRange(lane, crosswalk, closest_waypoint, detection_result, *obstacle_waypoint, vs_info.getStopRange(),
                        vs_info.getDecelerationRange(), detection_range_pub);
  obstacleTypeView(obstacle_type, obstacle_type_pub, pillar_velocity, mobileye_velocity);

  static int false_count = 0;
  static EControl prev_detection = EControl::KEEP;
  static int prev_obstacle_waypoint = -1;

  // stop or decelerate because we found obstacles
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE || detection_result == EControl::DECELERATE)
  {
    displayObstacle(detection_result, obstacle_points, obstacle_pub);
    prev_detection = detection_result;
    false_count = 0;
	prev_obstacle_waypoint = *obstacle_waypoint;
    return detection_result;
  }

  // there are no obstacles, but wait a little for safety
  if (prev_detection == EControl::STOP || prev_detection == EControl::STOPLINE || prev_detection == EControl::DECELERATE)
  {
    false_count++;

    if (false_count < LOOP_RATE / 2)
    {
      *obstacle_waypoint = prev_obstacle_waypoint;
		displayObstacle(EControl::OTHERS, obstacle_points, obstacle_pub);
      return prev_detection;
    }
  }

  // there are no obstacles, so we move forward
  *obstacle_waypoint = -1;
  false_count = 0;
  prev_detection = EControl::KEEP;
  return detection_result;
}

void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, const ros::Publisher& final_waypoints_pub, VelocitySetPath* vs_path)
{
  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  {  // STOP for obstacle/stopline
    // stop_waypoint is about stop_distance meter away from obstacles/stoplines
    int stop_distance = (detection_result == EControl::STOP)
      ? vs_info.getStopDistanceObstacle() : vs_info.getStopDistanceStopline();
    double deceleration = (detection_result == EControl::STOP)
      ? vs_info.getDecelerationObstacle() : vs_info.getDecelerationStopline();
    int stop_waypoint =
        calcWaypointIndexReverse(vs_path->getPrevWaypoints(), obstacle_waypoint, stop_distance);
    // change waypoints to stop by the stop_waypoint
    vs_path->changeWaypointsForStopping(stop_waypoint, obstacle_waypoint, closest_waypoint, deceleration);
    vs_path->avoidSuddenAcceleration(deceleration, closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), deceleration, closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
	final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else if (detection_result == EControl::DECELERATE)
  {  // DECELERATE for obstacles
    vs_path->initializeNewWaypoints();
    vs_path->changeWaypointsForDeceleration(vs_info.getDecelerationObstacle(), closest_waypoint, obstacle_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->avoidSuddenAcceleration(vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
	final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else
  {  // ACCELERATE or KEEP
    vs_path->initializeNewWaypoints();
	//vs_path->avoidSuddenAcceleration(vs_info.getDecelerationObstacle(), closest_waypoint);
	vs_path->avoidSuddenAcceleration(2.0, closest_waypoint);
	//vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
	vs_path->avoidSuddenDeceleration(1.0, 5.0, closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
	final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
}

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  listener = new tf::TransformListener();
  br = new tf::TransformBroadcaster();

  bool use_crosswalk_detection;
  bool enable_multiple_crosswalk_detection;
  bool enablePlannerDynamicSwitch;
  bool enableMobileye;

  std::string points_topic;
  private_nh.param<bool>("use_crosswalk_detection", use_crosswalk_detection, true);
  private_nh.param<bool>("enable_multiple_crosswalk_detection", enable_multiple_crosswalk_detection, true);
  private_nh.param<bool>("enablePlannerDynamicSwitch", enablePlannerDynamicSwitch, false);
  private_nh.param<bool>("enableMobileye", enableMobileye, false);

  private_nh.param<std::string>("points_topic", points_topic, "points_lanes");

  // class
  CrossWalk crosswalk;
  VelocitySetPath vs_path;
  VelocitySetInfo vs_info;

  // velocity set subs criber
  ros::Subscriber waypoints_sub = nh.subscribe("safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
  ros::Subscriber current_vel_sub =
      nh.subscribe("current_velocity", 1, &VelocitySetPath::currentVelocityCallback, &vs_path);
  ros::Subscriber sub_object_tracker = nh.subscribe("/detection/object_tracker/objects", 1, objectTrackerCallback);

  // velocity set info subscriber
  ros::Subscriber config_sub = nh.subscribe("config/velocity_set", 1, &VelocitySetInfo::configCallback, &vs_info);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 1, &VelocitySetInfo::pointsCallback, &vs_info);
  ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, &VelocitySetInfo::localizerPoseCallback, &vs_info);
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber detectionresult_sub = nh.subscribe("/state/stopline_wpidx", 1, &VelocitySetInfo::detectionCallback, &vs_info);
  ros::Subscriber mobileye_obstacle_sub = nh.subscribe("/parsed_tx/obstacle_data", 1, &VelocitySetInfo::mobileyeObstacleCallback, &vs_info);
  ros::Subscriber sub_waypoint_param = nh.subscribe("/waypoint_param", 1, &VelocitySetInfo::waypointParamCallback, &vs_info);

  // vector map subscriber
  ros::Subscriber sub_dtlane = nh.subscribe("vector_map_info/cross_walk", 1, &CrossWalk::crossWalkCallback, &crosswalk);
  ros::Subscriber sub_area = nh.subscribe("vector_map_info/area", 1, &CrossWalk::areaCallback, &crosswalk);
  ros::Subscriber sub_line = nh.subscribe("vector_map_info/line", 1, &CrossWalk::lineCallback, &crosswalk);
  ros::Subscriber sub_point = nh.subscribe("vector_map_info/point", 1, &CrossWalk::pointCallback, &crosswalk);

  // publisher
  ros::Publisher detection_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 1);
  ros::Publisher obstacle_waypoint_pub = nh.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);
  ros::Publisher econtrol_pub = nh.advertise<std_msgs::Int8>("econtrol", 1, false);
  ros::Publisher obstacle_type_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("obstacle_type", 1);
  ros::Publisher pillar_velocity_pub = nh.advertise<std_msgs::Float64>("pillar_velocity", 1);
  ros::Publisher mobileye_velocity_pub = nh.advertise<std_msgs::Float64>("mobileye_velocity", 1);
  ros::Publisher detection_mobileye_pub = nh.advertise<mobileye_560_660_msgs::ObstacleData>("detection_mobileye", 1);

  ros::Publisher final_waypoints_pub;
  if(enablePlannerDynamicSwitch){
	  final_waypoints_pub = nh.advertise<autoware_msgs::Lane>("astar/final_waypoints", 1, true);
  }else{
	  final_waypoints_pub = nh.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);
  }

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();

    int closest_waypoint = 0;

    if (crosswalk.loaded_all && !crosswalk.set_points)
      crosswalk.setCrossWalkPoints();

    if (!vs_info.getSetPose() || !vs_path.getSetPath())
    {
      loop_rate.sleep();
      continue;
    }

    crosswalk.setMultipleDetectionFlag(enable_multiple_crosswalk_detection);

    if (use_crosswalk_detection)
      crosswalk.setDetectionWaypoint(
          crosswalk.findClosestCrosswalk(closest_waypoint, vs_path.getPrevWaypoints(), STOP_SEARCH_DISTANCE));

    int obstacle_waypoint = -1;
	  EControl detection_result = obstacleDetection(closest_waypoint, vs_path.getPrevWaypoints(), crosswalk, vs_path, vs_info,
	                                              detection_range_pub, obstacle_pub, detection_mobileye_pub,
                                                pillar_velocity_pub, mobileye_velocity_pub ,&obstacle_waypoint, enableMobileye, obstacle_type_pub);

    changeWaypoints(vs_info, detection_result, closest_waypoint,
	                obstacle_waypoint, final_waypoints_pub, &vs_path);

	std_msgs::Int8 econtrol_msg;
	econtrol_msg.data = (char)detection_result;
	econtrol_pub.publish(econtrol_msg);

	vs_info.clearPoints();
	vs_info.clearMobileyeObstacle();

    // publish obstacle waypoint index
    std_msgs::Int32 obstacle_waypoint_index;
    obstacle_waypoint_index.data = obstacle_waypoint;
    obstacle_waypoint_pub.publish(obstacle_waypoint_index);

    vs_path.resetFlag();

    loop_rate.sleep();
  }

  delete listener;
  delete br;
  return 0;
}
