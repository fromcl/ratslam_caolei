/*
 * openRatSLAM
 *
 * visual_odometry - simple visual odometry module based on image differencing
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

#include "visual_odometry.h"
#include "../utils/utils.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ratslam
{

VisualOdometry::VisualOdometry(ptree settings)
{
  get_setting_from_ptree(VTRANS_IMAGE_X_MIN, settings, "vtrans_image_x_min", 0);
  get_setting_from_ptree(VTRANS_IMAGE_X_MAX, settings, "vtrans_image_x_max", IMAGE_WIDTH);
  get_setting_from_ptree(VTRANS_IMAGE_Y_MIN, settings, "vtrans_image_y_min", 0);
  get_setting_from_ptree(VTRANS_IMAGE_Y_MAX, settings, "vtrans_image_y_max", IMAGE_HEIGHT);

  get_setting_from_ptree(VROT_IMAGE_X_MIN, settings, "vrot_image_x_min", 0);
  get_setting_from_ptree(VROT_IMAGE_X_MAX, settings, "vrot_image_x_max", IMAGE_WIDTH);
  get_setting_from_ptree(VROT_IMAGE_Y_MIN, settings, "vrot_image_y_min", 0);
  get_setting_from_ptree(VROT_IMAGE_Y_MAX, settings, "vrot_image_y_max", IMAGE_HEIGHT);

  get_setting_from_ptree(CAMERA_FOV_DEG, settings, "camera_fov_deg", 50.0);
  get_setting_from_ptree(CAMERA_HZ, settings, "camera_hz", 10.0);

  get_setting_from_ptree(VTRANS_SCALING, settings, "vtrans_scaling", 100.0);
  get_setting_from_ptree(VTRANS_MAX, settings, "vtrans_max", 20.0);

  vtrans_profile.resize(VTRANS_IMAGE_X_MAX - VTRANS_IMAGE_X_MIN);  //确定图片横向像素点个数来确定容器大小400,用于模板不确定
  vtrans_prev_profile.resize(VTRANS_IMAGE_X_MAX - VTRANS_IMAGE_X_MIN);  //用于视觉里程计或上一个模板不确定
  vrot_profile.resize(VROT_IMAGE_X_MAX - VROT_IMAGE_X_MIN);  //400未知
  vrot_prev_profile.resize(VROT_IMAGE_X_MAX - VROT_IMAGE_X_MIN);  //未知

  first = true;
}

//vo->on_image(&image->data[0], (image->encoding == "rgb8" ? false : true), image->width, image->height, &odom_output.twist.twist.linear.x, &odom_output.twist.twist.angular.z);\
linear.x为x方向角速度,angular.z为绕z轴旋转角速度
void VisualOdometry::on_image(const unsigned char * data, bool greyscale, unsigned int image_width, unsigned int image_height, double *vtrans_ms, double *vrot_rads)
{
  double dummy;

  IMAGE_WIDTH = image_width;  //400
  IMAGE_HEIGHT = image_height;  //300

  if (first)  //判断构造函数结束
  {
    for (unsigned int i = 0; i < vtrans_profile.size(); i++)  //可能是把某一张图作为模版存起来
    {
      vtrans_prev_profile[i] = vtrans_profile[i];
    }
    for (unsigned int i = 0; i < vrot_profile.size(); i++)
    {
      vrot_prev_profile[i] = vrot_profile[i];
    }
    first = false;
  }

  convert_view_to_view_template(&vtrans_profile[0], data, greyscale, VTRANS_IMAGE_X_MIN, VTRANS_IMAGE_X_MAX, VTRANS_IMAGE_Y_MIN, VTRANS_IMAGE_Y_MAX);
  visual_odo(&vtrans_profile[0], vtrans_profile.size(), &vtrans_prev_profile[0], vtrans_ms, &dummy);

  convert_view_to_view_template(&vrot_profile[0], data, greyscale, VROT_IMAGE_X_MIN, VROT_IMAGE_X_MAX, VROT_IMAGE_Y_MIN, VROT_IMAGE_Y_MAX);
  visual_odo(&vrot_profile[0], vrot_profile.size(), &vrot_prev_profile[0], &dummy, vrot_rads);
}

void VisualOdometry::visual_odo(double *data, unsigned short width, double *olddata, double *vtrans_ms, double *vrot_rads)
{
  double mindiff = 1e6;
  double minoffset = 0;
  double cdiff;
  int offset;

  int cwl = width;
  int slen = 40;

  int k;
  //  data, olddata are 1D arrays of the intensity profiles  (current and previous);
  // slen is the range of offsets in pixels to consider i.e. slen = 0 considers only the no offset case
  // cwl is the length of the intensity profile to actually compare, and must be < than image width – 1 * slen

  for (offset = 0; offset < slen; offset++)
  {
    cdiff = 0;

    for (k = 0; k < cwl - offset; k++)
    {
      cdiff += fabs(data[k] - olddata[k + offset]);
    }

    cdiff /= (1.0 * (cwl - offset));

    if (cdiff < mindiff)
    {
      mindiff = cdiff;
      minoffset = -offset;
    }
  }

  for (offset = 0; offset < slen; offset++)
  {
    cdiff = 0;

    for (k = 0; k < cwl - offset; k++)
    {
      cdiff += fabs(data[k + offset] - olddata[k]);
    }

    cdiff /= (1.0 * (cwl - offset));

    if (cdiff < mindiff)
    {
      mindiff = cdiff;
      minoffset = offset;
    }
  }

  for (unsigned int i = 0; i < width; i++)
  {
    olddata[i] = data[i];
  }
  *vrot_rads = minoffset * CAMERA_FOV_DEG / IMAGE_WIDTH * CAMERA_HZ * M_PI / 180.0;
  *vtrans_ms = mindiff * VTRANS_SCALING;
  if (*vtrans_ms > VTRANS_MAX)
    *vtrans_ms = VTRANS_MAX;

}

//convert_view_to_view_template                   (&vtrans_profile[0], data, greyscale, VTRANS_IMAGE_X_MIN, VTRANS_IMAGE_X_MAX, VTRANS_IMAGE_Y_MIN, VTRANS_IMAGE_Y_MAX);
void VisualOdometry::convert_view_to_view_template(double *current_view, const unsigned char *view_rgb, bool grayscale, int X_RANGE_MIN, int X_RANGE_MAX, int Y_RANGE_MIN,
                                                   int Y_RANGE_MAX)
{
  unsigned int TEMPLATE_Y_SIZE = 1;
  unsigned int TEMPLATE_X_SIZE = X_RANGE_MAX - X_RANGE_MIN;  //400

  int data_next = 0;
  for (int i = 0; i < TEMPLATE_X_SIZE; i++)  //将存放横向像素点的容器全初始化为0,等价于vtrans_profile
  {
    current_view[i] = 0;
  }

  int sub_range_x = X_RANGE_MAX - X_RANGE_MIN;  //400
  int sub_range_y = Y_RANGE_MAX - Y_RANGE_MIN;  //300
  int x_block_size = sub_range_x / TEMPLATE_X_SIZE;  //结果为1
  int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;  //结果为300
  int pos;

  if (grayscale)
  {//                  0                                               1                           300
    for (int y_block = Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)  //做1次,y_block一口气加到300,但没有用
    {//                  0                                               400                         1
      for (int x_block = X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)  //做400次,x_block逐一加到399
      {//                                    1
        for (int x = x_block; x < (x_block + x_block_size); x++)  //做1次,x每次不一样[0,1,2...399]
        {//                                    300
          for (int y = y_block; y < (y_block + y_block_size); y++)  //300次,y逐一加到299
          {//
            pos = (x + y * IMAGE_WIDTH);  //IMAGE_WIDTH为400,实质上是参数vrot_image_x_max
            current_view[data_next] += (double)(view_rgb[pos]);  //为vtrans_profile移位自加image->data的第(x+400y)位
          }
        }
        current_view[data_next] /= (255.0);
        current_view[data_next] /= (x_block_size * y_block_size);  //自除以300
        data_next++;
      }
    }
  }
  else
  {
    for (int y_block = Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
    {
      for (int x_block = X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * IMAGE_WIDTH) * 3;
            current_view[data_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1]) + (double)(view_rgb[pos + 2]));
          }
        }
        current_view[data_next] /= (255.0 * 3.0);
        current_view[data_next] /= (x_block_size * y_block_size);
        data_next++;
      }
    }
  }

}

}
;
// namespace ratslam
