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

#include <iostream>
using namespace std;

#include "utils/utils.h"
#include "ratslam/visual_odometry.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>


ratslam::VisualOdometry *vo = NULL;
ros::Publisher pub_vo;

using namespace ratslam;

void image_callback(sensor_msgs::ImageConstPtr image)  //ConstPtr前没加::可以?
{
  ROS_DEBUG_STREAM("VO:image_callback{" << ros::Time::now() << "} seq=" << image->header.seq);  //调试消息中若有错误消息,就将这个消息的序号打印出来

  static nav_msgs::Odometry odom_output;

  // FIXME: intelligent handling of image encoding
  vo->on_image(&image->data[0], (image->encoding == "rgb8" ? false : true), image->width, image->height, &odom_output.twist.twist.linear.x, &odom_output.twist.twist.angular.z);

  odom_output.header.stamp = image->header.stamp;  //image的header.stamp是用来记录图像的采集时间,此时将它同步到标准里程计的时间戳上
  odom_output.header.seq++;  //维护消息的序列号,其实和image->header.seq相等

  pub_vo.publish(odom_output);  //发布视觉里程计
}


int main(int argc, char * argv[])
{
  ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }

  std::string topic_root = "";

  boost::property_tree::ptree settings, general_settings, vo_settings;
  read_ini(argv[1], settings);  //读取config文件夹中的ini文件参数,全部放入settings
  ratslam::get_setting_child(vo_settings, settings, "visual_odometry", true);  //给child(第一个形参)引用的boost::property_tree::ptree对象赋ini文件[visual_odometry]参数值,最后又get_setting_from_ptree()给各个常量或变量
  ratslam::get_setting_child(general_settings, settings, "general", true);  //同上
  ratslam::get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string) "");

  vo = new ratslam::VisualOdometry(vo_settings);  //构造过程为给var(第一个形参)引用的常量或变量赋ini文件各个参数具体值.

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMVisualOdometry");
  }
  ros::NodeHandle node;

  pub_vo = node.advertise<nav_msgs::Odometry>(topic_root + "/odom", 0);

  image_transport::ImageTransport it(node);  //NodeHandle被image_transport后用来订阅图片
  image_transport::Subscriber sub = it.subscribe(topic_root, 1, image_callback);

  ros::spin();

  return 0;
}
