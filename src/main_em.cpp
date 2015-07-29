/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>

#include "ratslam/experience_map.h"
#include <ratslam_ros/TopologicalAction.h>
#include <nav_msgs/Odometry.h>
#include "graphics/experience_map_scene.h"
#include <ratslam_ros/TopologicalMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

// Distance server by Mr-Yellow 2015-04-25
#include <ratslam_ros/GetDistance.h>


ros::Publisher pub_em;
ros::Publisher pub_pose;
ros::Publisher pub_em_markers;
ros::Publisher pub_goal_path;
geometry_msgs::PoseStamped pose_output;
ratslam_ros::TopologicalMap em_map;
visualization_msgs::Marker em_marker;

#ifdef HAVE_IRRLICHT
#include "graphics/experience_map_scene.h"
ratslam::ExperienceMapScene *ems;
bool use_graphics;
#endif

using namespace ratslam;

void odo_callback(nav_msgs::OdometryConstPtr odo, ratslam::ExperienceMap *em)
{
  ROS_DEBUG_STREAM("EM:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z);
//ros::Time::now()为取到当前ROS时间

  static ros::Time prev_time(0);  //创建一个时间,从0秒开始

  if (prev_time.toSec() > 0)
  {
    double time_diff = (odo->header.stamp - prev_time).toSec();
    //ROS_DEBUG_STREAM("EM:odo_callback{" << ros::Time::now() << "} seq=" << odo->header.seq << " v=" << odo->twist.twist.linear.x << " r=" << odo->twist.twist.angular.z << " t=" << time_diff << " goal=" << em->get_current_goal_id());
    em->on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff);  //一点一点累加出坐标x,y
  }

  static ros::Time prev_goal_update(0);  //用于存放视觉里程计的时间戳

  if (em->get_current_goal_id() >= 0)  //没有目标发布者时为-1,有目标发布者为(int)goal_list.front()容器头成员的地址
  {
    // (prev_goal_update.toSec() == 0 || (odo->header.stamp - prev_goal_update).toSec() > 5)
    //em->calculate_path_to_goal(odo->header.stamp.toSec());

    prev_goal_update = odo->header.stamp;  //存放视觉里程计的时间戳

    em->calculate_path_to_goal(odo->header.stamp.toSec());  //计算目标路径,利用迪杰斯特拉算法

//Path消息包括:
//             Header header
//             geometry_msgs/PoseStamped[] poses
//                                              Header header
//                                              Pose pose
//                                                       Point position
//                                                                     float64 x
//                                                                     float64 y
//                                                                     float64 z
//                                                       Quaternion orientation
//                                                                             float64 x
//                                                                             float64 y
//                                                                             float64 z
//                                                                             float64 w
    static nav_msgs::Path path;
    if (em->get_current_goal_id() >= 0)  //再次确认,有目标发布者为(int)goal_list.front()容器头成员的地址
    {
      em->get_goal_waypoint();  //排斥掉临近点后为waypoint_exp_id赋值目标点id,若目标点就在临近点,则为waypoint_exp_id赋值当前点id

      static geometry_msgs::PoseStamped pose;
      path.header.stamp = ros::Time::now();
      path.header.frame_id = "1";

      pose.header.seq = 0;
      pose.header.frame_id = "1";
      path.poses.clear();
      unsigned int trace_exp_id = em->get_goals()[0];  //返回指向第一个指向目标点的指针,trace_exp_id为第一个目标点id
      while (trace_exp_id != em->get_goal_path_final_exp())  //若当前id不等于第一个目标点id
      {
        pose.pose.position.x = em->get_experience(trace_exp_id)->x_m;  //get_experience返回指向当前经验地图的指针
        pose.pose.position.y = em->get_experience(trace_exp_id)->y_m;
        path.poses.push_back(pose);  //将路径话题中的poses数组
        pose.header.seq++;

        trace_exp_id = em->get_experience(trace_exp_id)->goal_to_current;  //赋予 动态 起始点id
      }

      pub_goal_path.publish(path);  //发布这条路经

      path.header.seq++;

    }
    else
    {
      path.header.stamp = ros::Time::now();
      path.header.frame_id = "1";
      path.poses.clear();
      pub_goal_path.publish(path);

      path.header.seq++;
    }
  }

  prev_time = odo->header.stamp;  //维护一个时间为里程计的上一个时间戳,用来计算△t再计算x,y
}

//TopologicalAction消息中包括:
//                            uint32 CREATE_NODE=1
//                            uint32 CREATE_EDGE=2
//                            uint32 SET_NODE=3
//                            Header header
//                            uint32 action
//                            uint32 src_id
//                            uint32 dest_id
//                            float64 relative_rad

void action_callback(ratslam_ros::TopologicalActionConstPtr action, ratslam::ExperienceMap *em)
{
  ROS_DEBUG_STREAM("EM:action_callback{" << ros::Time::now() << "} action=" << action->action << " src=" << action->src_id << " dst=" << action->dest_id);

  switch (action->action)  //ratslam_ros::TopologicalAction action->action有四个值NO_ACTION的值为0,CREATE_NODE为1,CREATE_EDGE为2,SET_NODE为3.没有行动,创建节点,创建边缘,重置节点
  {
  //第一次和走未探索区域时进来,新建一个经验地图和链接
    case ratslam_ros::TopologicalAction::CREATE_NODE:
      em->on_create_experience(action->dest_id);  //返回值为经验地图的长度-1,为每张经验地图给定id、坐标值、角度参数,并当经验地图长度不为1时创建一个经验链接，链接里存放整体位移,两种角度差,累加时间等一些中间参数
      em->on_set_experience(action->dest_id, 0);  //返回值为bool型,当dest_id小于经验地图id时,就将累加出来的坐标值清空但保留角度值(猜测目的应该让pc赶上来)
      break;
  //应该调用在发生了交会的那个点上,只是建立一个链接而不建立经验地图
    case ratslam_ros::TopologicalAction::CREATE_EDGE:
      em->on_create_link(action->src_id, action->dest_id, action->relative_rad);  //创建一个经验链接,内存放整体位移,两种角度差,累加时间等一些中间参数
      em->on_set_experience(action->dest_id, action->relative_rad);
      break;
  //发生完了交会,走已经走过的路,不需要再建立经验地图和链接
    case ratslam_ros::TopologicalAction::SET_NODE:
      em->on_set_experience(action->dest_id, action->relative_rad);
      break;

  }

  em->iterate();  //迭代,以将经过多次的同一条路重合起来

//此处发布了一个机器当前所在坐标的消息
  pose_output.header.stamp = ros::Time::now();  //为ROS的当前时间
  pose_output.header.seq++;
  pose_output.header.frame_id = "1";
  pose_output.pose.position.x = em->get_experience(em->get_current_id())->x_m;
  pose_output.pose.position.y = em->get_experience(em->get_current_id())->y_m;
  pose_output.pose.position.z = 0;
  pose_output.pose.orientation.x = 0;
  pose_output.pose.orientation.y = 0;
  pose_output.pose.orientation.z = sin(em->get_experience(em->get_current_id())->th_rad / 2.0);
  pose_output.pose.orientation.w = cos(em->get_experience(em->get_current_id())->th_rad / 2.0);
  pub_pose.publish(pose_output);

  static ros::Time prev_pub_time(0);

//TopologicalMap消息中包含:
//                         Header header
//                         uint32 node_count
//                         uint32 edge_count
//                         TopologicalNode[] node
//                                               geometry_msgs/Pose pose
//                                                                      Point position
//                                                                                    float64 x
//                                                                                    float64 y
//                                                                                    float64 z
//                                                                      Quaternion orientation
//                                                                                            float64 x
//                                                                                            float64 y
//                                                                                            float64 z
//                                                                                            float64 w
//                         TopologicalEdge[] edge
//                                               uint32 id
//                                               uint32 source_id
//                                               uint32 destination_id
//                                               duration duration
//                                                                未找到
//                                               geometry_msgs/Transform transform
//                                                                                Vector3 translation
//                                                                                                   float64 x
//                                                                                                   float64 y
//                                                                                                   float64 z
//                                                                                Quaternion rotation
//                                                                                                   float64 x
//                                                                                                   float64 y
//                                                                                                   float64 z
//                                                                                                   float64 w

  if (action->header.stamp - prev_pub_time > ros::Duration(30.0))  //当pc消息时间和action_callback时差大于30秒会进来.发布一个以TopologicalMap为消息的话题:topic_root+"/ExperienceMap/Map",无人订阅
  {
    prev_pub_time = action->header.stamp;

    em_map.header.stamp = ros::Time::now();  //TopologicalMap
    em_map.header.seq++;
    em_map.node_count = em->get_num_experiences();  //返回经验地图的成员数量
    em_map.node.resize(em->get_num_experiences());  //让消息数量也为成员数量
    for (int i = 0; i < em->get_num_experiences(); i++)
    {
      em_map.node[i].id = em->get_experience(i)->id;
      em_map.node[i].pose.position.x = em->get_experience(i)->x_m;
      em_map.node[i].pose.position.y = em->get_experience(i)->y_m;
      em_map.node[i].pose.orientation.x = 0;
      em_map.node[i].pose.orientation.y = 0;
      em_map.node[i].pose.orientation.z = sin(em->get_experience(i)->th_rad / 2.0);
      em_map.node[i].pose.orientation.w = cos(em->get_experience(i)->th_rad / 2.0);
    }

    em_map.edge_count = em->get_num_links();
    em_map.edge.resize(em->get_num_links());
    for (int i = 0; i < em->get_num_links(); i++)
    {
      em_map.edge[i].source_id = em->get_link(i)->exp_from_id;
      em_map.edge[i].destination_id = em->get_link(i)->exp_to_id;
      em_map.edge[i].duration = ros::Duration(em->get_link(i)->delta_time_s);
      em_map.edge[i].transform.translation.x = em->get_link(i)->d * cos(em->get_link(i)->heading_rad);
      em_map.edge[i].transform.translation.y = em->get_link(i)->d * sin(em->get_link(i)->heading_rad);
      em_map.edge[i].transform.rotation.x = 0;
      em_map.edge[i].transform.rotation.y = 0;
      em_map.edge[i].transform.rotation.z = sin(em->get_link(i)->facing_rad / 2.0);
      em_map.edge[i].transform.rotation.w = cos(em->get_link(i)->facing_rad / 2.0);
    }
    pub_em.publish(em_map);
  }


  em_marker.header.stamp = ros::Time::now();
  em_marker.header.seq++;
  em_marker.header.frame_id = "1";  //rviz中用"1"?
  em_marker.type = visualization_msgs::Marker::LINE_LIST;  //发送指定标记的形状为一个链
  em_marker.points.resize(em->get_num_links() * 2);  //标记当前坐标值的消息长度为links.size的2倍
  em_marker.action = visualization_msgs::Marker::ADD;  //ADD为创建或修改,visualization_msgs::Marker::DELETE为删除,DELETEALL删除所有标记的特定Rviz显示无论ID或名称空间
  em_marker.scale.x = 0.01;  //指定标记的规模
  //em_marker.scale.y = 1;
  //em_marker.scale.z = 1;
  em_marker.color.a = 1;  //标志的颜色,由（float）r、g、b、a四个成员决定，a=1表示完全不透明
  em_marker.ns = "em";  //创建名称空间为em
  em_marker.id = 0;  //名称空间(ns)和id用于创建一个唯一的名称标记。如果接收到消息标志与ns和id相同,新标志将取代旧的
  em_marker.pose.orientation.x = 0;  //设置标记的姿势,用四元数设置了方向,位置为原点,朝向0度
  em_marker.pose.orientation.y = 0;
  em_marker.pose.orientation.z = 0;
  em_marker.pose.orientation.w = 1;
  for (int i = 0; i < em->get_num_links(); i++)  //做links.size次，设置标记当前的位置，不断刷新已跑过的位置和当前位置，*2是将路径显示缩放2倍

  {
    em_marker.points[i * 2].x = em->get_experience(em->get_link(i)->exp_from_id)->x_m;
    em_marker.points[i * 2].y = em->get_experience(em->get_link(i)->exp_from_id)->y_m;
    em_marker.points[i * 2].z = 0;
    em_marker.points[i * 2 + 1].x = em->get_experience(em->get_link(i)->exp_to_id)->x_m;
    em_marker.points[i * 2 + 1].y = em->get_experience(em->get_link(i)->exp_to_id)->y_m;
    em_marker.points[i * 2 + 1].z = 0;
  }

  pub_em_markers.publish(em_marker);  //打印

#ifdef HAVE_IRRLICHT
  if (use_graphics)
  {
    ems->update_scene();
    ems->draw_all();
  }
#endif
}

//PoseStamped消息为：
//                  Header header
//                  Pose pose
//                           Point position
//                                         float64 x
//                                         float64 y
//                                         float64 z
//                           Quaternion orientation
//                                                 float64 x
//                                                 float64 y
//                                                 float64 z
//                                                 float64 w

void set_goal_pose_callback(geometry_msgs::PoseStampedConstPtr pose, ratslam::ExperienceMap * em)  //订阅了一个目标点消息,和机器人位置消息用的同一消息格式
{
  ROS_DEBUG_STREAM("EM:set_goal_pose_callback x=" << pose->pose.position.x << " y=" << pose->pose.position.y);
  em->add_goal(pose->pose.position.x, pose->pose.position.y);  //记录下目标点和经验地图的某个点距离小于0.1米的目标点
}

/**
 * Service to retrieve distance between two points.
 * @author Mr-Yellow<mr-yellow@mr-yellow.com>
 * @date 2015-04-25
 * @method get_distance_callback
 * @param {ratslam_ros::GetDistance::Request} req
 * @param {ratslam_ros::GetDistance::Response} res
 * @param {ratslam::ExperienceMap} em
 * @return {bool}
*/
bool get_distance_callback(ratslam_ros::GetDistance::Request  &req, ratslam_ros::GetDistance::Response &res, ratslam::ExperienceMap * em)
{
  res.distance = em->dijkstra_distance_between_experiences(req.id1, req.id2);  //迪杰斯特拉算法计算并返回id1到id2间最短路经所用总时间
  ROS_INFO("request: x=%d, y=%d", (int)req.id1, (int)req.id2);
  ROS_INFO("sending back response: [%ld]", (long int)res.distance);
  return true;
}

int main(int argc, char * argv[])
{
  ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");
  ROS_INFO_STREAM("Modified by Mr-Yellow.");

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  std::string topic_root = "";
  boost::property_tree::ptree settings, general_settings, ratslam_settings;
  read_ini(argv[1], settings);

  get_setting_child(ratslam_settings, settings, "ratslam", true);
  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMExperienceMap");
  }
  ros::NodeHandle node;

  ratslam::ExperienceMap * em = new ratslam::ExperienceMap(ratslam_settings);

//------------------------------全为发布-----------------------------------
  pub_em = node.advertise<ratslam_ros::TopologicalMap>(topic_root + "/ExperienceMap/Map", 1);  //无人订阅
  pub_em_markers = node.advertise<visualization_msgs::Marker>(topic_root + "/ExperienceMap/MapMarker", 1);  //无人订阅,用于构建rviz图形
  pub_pose = node.advertise<geometry_msgs::PoseStamped>(topic_root + "/ExperienceMap/RobotPose", 1);  //无人订阅，发布了一个机器当前所在坐标的消息
  pub_goal_path = node.advertise<nav_msgs::Path>(topic_root + "/ExperienceMap/PathToGoal", 1);  //无人订阅,发布了一个迪杰斯特拉算法计算出的实时路径
//------------------------------全为订阅-----------------------------------
  ros::Subscriber sub_odometry = node.subscribe<nav_msgs::Odometry>(topic_root + "/odom", 0, boost::bind(odo_callback, _1, em), ros::VoidConstPtr(),
                                                                    ros::TransportHints().tcpNoDelay());  //来自于vo
  ros::Subscriber sub_action = node.subscribe<ratslam_ros::TopologicalAction>(topic_root + "/PoseCell/TopologicalAction", 0, boost::bind(action_callback, _1, em),
                                                                    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());  //来自于pc
  ros::Subscriber sub_goal = node.subscribe<geometry_msgs::PoseStamped>(topic_root + "/ExperienceMap/SetGoalPose", 0, boost::bind(set_goal_pose_callback, _1, em),
                                                                    ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());  //找不到消息发布者,订阅了一个目标点消息,和机器人位置消息用的同一消息格式
//-------------------------------------------------------------------------
  // Distance server by Mr-Yellow 2015-04-25
  ros::ServiceServer service = node.advertiseService<ratslam_ros::GetDistance::Request, ratslam_ros::GetDistance::Response>(topic_root + "/ExperienceMap/GetDistance", boost::bind(get_distance_callback, _1, _2, em));  //无人订阅
  

#ifdef HAVE_IRRLICHT
  boost::property_tree::ptree draw_settings;
  get_setting_child(draw_settings, settings, "draw", true);
  get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
  if (use_graphics)
  {
    ems = new ratslam::ExperienceMapScene(draw_settings, em);
  }
#endif

  ros::spin();

  return 0;
}

