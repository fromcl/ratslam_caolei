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

#include "local_view_match.h"
#include "../utils/utils.h"

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iostream>
#include <iomanip>
using namespace std;
#include <boost/foreach.hpp>
#include <algorithm>

#include <stdio.h>

namespace ratslam
{



LocalViewMatch::LocalViewMatch(ptree settings)
{
  get_setting_from_ptree(VT_MIN_PATCH_NORMALISATION_STD, settings, "vt_min_patch_normalisation_std", (double)0);
  get_setting_from_ptree(VT_PATCH_NORMALISATION, settings, "vt_patch_normalise", 0);
  get_setting_from_ptree(VT_NORMALISATION, settings, "vt_normalisation", (double) 0);
  get_setting_from_ptree(VT_SHIFT_MATCH, settings, "vt_shift_match", 25);
  get_setting_from_ptree(VT_STEP_MATCH, settings, "vt_step_match", 5);
  get_setting_from_ptree(VT_PANORAMIC, settings, "vt_panoramic", 0);
 
  get_setting_from_ptree(VT_MATCH_THRESHOLD, settings, "vt_match_threshold", 0.03);
  get_setting_from_ptree(TEMPLATE_X_SIZE, settings, "template_x_size", 1);
  get_setting_from_ptree(TEMPLATE_Y_SIZE, settings, "template_y_size", 1);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MIN, settings, "image_crop_x_min", 0);
  get_setting_from_ptree(IMAGE_VT_X_RANGE_MAX, settings, "image_crop_x_max", -1);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MIN, settings, "image_crop_y_min", 0);
  get_setting_from_ptree(IMAGE_VT_Y_RANGE_MAX, settings, "image_crop_y_max", -1);

  TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;  //3000=60*50

  templates.reserve(10000);  //用于存放一个一个视图模板

  current_view.resize(TEMPLATE_SIZE);  //3000，经过标准化处理的视觉模板数据部分（即template->data）由这里产生

  current_vt = 0;
  prev_vt = 0;
}


LocalViewMatch::~LocalViewMatch()
{

}

//lv->on_image               (&image->data[0], (image->encoding == "rgb8" ? false : true), image->width, image->height);
void LocalViewMatch::on_image(const unsigned char *view_rgb, bool greyscale, unsigned int image_width, unsigned int image_height)
{
  if (view_rgb == NULL)
    return;

  IMAGE_WIDTH = image_width;  //假定400
  IMAGE_HEIGHT = image_height;  //假定300

  if (IMAGE_VT_X_RANGE_MAX == -1)  //config内定义图像裁剪的大小,x为320
    IMAGE_VT_X_RANGE_MAX = IMAGE_WIDTH;
  if (IMAGE_VT_Y_RANGE_MAX == -1)  //y为20-260,若都未设置,则不裁剪
    IMAGE_VT_Y_RANGE_MAX = IMAGE_HEIGHT;

  this->view_rgb = view_rgb;  //如此做法不改变原值
  this->greyscale = greyscale;

  convert_view_to_view_template(greyscale);  //从当前视图到视图模板，将current_view[i]经过一种标准化处理
  prev_vt = get_current_vt();  //返回值为current_vt，初值为0
  unsigned int vt_match_id;  //区别最小的模板的ID号
  compare(vt_error, vt_match_id);  //一开始并未给vt_error，vt_match_id赋值，第一次没模板时直接返回vt_error双精度浮点数最大值,之后返回最小区别模板的区别值,和该模板的id号
  if (vt_error <= VT_MATCH_THRESHOLD)  //返回的比较值和匹配阀值相比较
  {
    set_current_vt((int)vt_match_id);  //返回模板像素平均值相似附近模板匹配最小差距模板的vt.id
    cout << "VTM[" << setw(4) << get_current_vt() << "] " << endl;
    cout.flush();
  }
  else
  {
    vt_relative_rad = 0;
    set_current_vt(create_template());  //create_template()返回当前模板的id号，set_current_vt将当前模板号赋值给current_vt（初值为0）
    cout << "VTN[" << setw(4) << get_current_vt() << "] " << endl;  //用get_current_vt()把current_vt值返回出来，再将该模板id号打印出来,setw(4)输出宽度为4个字符
    cout.flush();  //用于刷新缓冲,无条件地将缓冲区中的输出信息送显示器
  }

}


void LocalViewMatch::clip_view_x_y(int &x, int &y)  //约束x,y的上下限
{
  if (x < 0)
    x = 0;
  else if (x > TEMPLATE_X_SIZE - 1)
    x = TEMPLATE_X_SIZE - 1;  //x=60-1

  if (y < 0)
    y = 0;
  else if (y > TEMPLATE_Y_SIZE - 1)
    y = TEMPLATE_Y_SIZE - 1;  //y=50-1

}

void LocalViewMatch::convert_view_to_view_template(bool grayscale)  //从当前视图到视图模板，将current_view[i]经过一种标准化处理，再将视觉模板一个一个装入template容器中
{
  int data_next = 0;
  int sub_range_x = IMAGE_VT_X_RANGE_MAX - IMAGE_VT_X_RANGE_MIN;  //320=320-0
  int sub_range_y = IMAGE_VT_Y_RANGE_MAX - IMAGE_VT_Y_RANGE_MIN;  //240=260-20
  int x_block_size = sub_range_x / TEMPLATE_X_SIZE;  //(int)5.3333=320/60
  int y_block_size = sub_range_y / TEMPLATE_Y_SIZE;  //(int)4.8=240/50  以上两步应该是为了计算模板块的大小
  int pos;

  for (unsigned int i; i < current_view.size(); i++)  //size为3000
    current_view[i] = 0;

  if (grayscale)  //利用一种滤波得到视图模板,和odo中的滤波不同,这看起来是把图片压扁在再滤波得到一个更长的一个数组3000位
  {//                  20                                                       50                     20+=4
    for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)  //做50次
    {//                  0                                                        60                      0+=5
      for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)  //做60次
      {//                                    5
        for (int x = x_block; x < (x_block + x_block_size); x++)  //做5次,x=0,5,10,15...
        {//                                    4
          for (int y = y_block; y < (y_block + y_block_size); y++)  //做4次,y=0,4,8,12...
          {
            pos = (x + y * IMAGE_WIDTH);  //pos=x+400y
            current_view[data_next] += (double)(view_rgb[pos]);  //给current_view移位自加image->data的第(x+400y)位,每自加20次移位1次,共移位60次
          }
        }
        current_view[data_next] /= (255.0);
        current_view[data_next] /= (x_block_size * y_block_size);  //这两步给current_view[data_next]除5100
        data_next++;
      }
    }
  }
  else
  {
    for (int y_block = IMAGE_VT_Y_RANGE_MIN, y_block_count = 0; y_block_count < TEMPLATE_Y_SIZE; y_block += y_block_size, y_block_count++)
    {
      for (int x_block = IMAGE_VT_X_RANGE_MIN, x_block_count = 0; x_block_count < TEMPLATE_X_SIZE; x_block += x_block_size, x_block_count++)
      {
        for (int x = x_block; x < (x_block + x_block_size); x++)
        {
          for (int y = y_block; y < (y_block + y_block_size); y++)
          {
            pos = (x + y * IMAGE_WIDTH) * 3;
            current_view[data_next] += ((double)(view_rgb[pos]) + (double)(view_rgb[pos + 1])
                + (double)(view_rgb[pos + 2]));
          }
        }
        current_view[data_next] /= (255.0 * 3.0);
        current_view[data_next] /= (x_block_size * y_block_size);

        data_next++;
      }
    }
  }

  if (VT_NORMALISATION > 0)  //为0.5,默认值为0
  {
    double avg_value = 0;

    for (unsigned int i = 0; i < current_view.size(); i++)  //做3000次
    {
      avg_value += current_view[i];
    }

    avg_value /= current_view.size();  //÷3000

    for (unsigned int i = 0; i < current_view.size(); i++)
    {
      current_view[i] = std::max(0.0, std::min(current_view[i] * VT_NORMALISATION / avg_value, 1.0));  //又一次滤波,成员除以平均值不知它的数学意义是什么
    }
  }

  // now do patch normalisation
  // +- patch size on the pixel, ie 4 will give a 9x9
  if (VT_PATCH_NORMALISATION > 0)  //未设置,默认值为0，以下假定为1来盲算
  {
    int patch_size = VT_PATCH_NORMALISATION;  //VT标准化
    int patch_total = (patch_size * 2 + 1) * (patch_size * 2 + 1);  //9=(1 * 2 + 1) * (1 * 2 + 1)
    double patch_sum;
    double patch_mean;
    double patch_std;
    int patch_x_clip;
    int patch_y_clip;

    // first make a copy of the view
    std::vector<double> current_view_copy;
    current_view_copy.resize(current_view.size());
    for (unsigned int i = 0; i < current_view.size(); i++)
      current_view_copy[i] = current_view[i];

    // this code could be significantly optimimised ....这个代码可以显著优化
    for (int x = 0; x < TEMPLATE_X_SIZE; x++)  //做60次
    {
      for (int y = 0; y < TEMPLATE_Y_SIZE; y++)  //做50次
      {
        patch_sum = 0;//       1                         1
        for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)  //做3次
        {//                      1                         1
          for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)  //做3次
          {
            patch_x_clip = patch_x;  //第一次进来x,y都为-1,之后9次（x，y）为（-1，-1）（-1,0）(-1，1）（0,-1）（0，0）（0,1）（1，-1）（1,0）（1,1）
            patch_y_clip = patch_y;
            clip_view_x_y(patch_x_clip, patch_y_clip);  //约束x,y的上下限在0-59、49间                                                      //       □ □ □ □
                                                                                                                                          //       □ □ □ □
            patch_sum += current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE];  //此计算公式抽象为黑点之和，边界外黑点向内重合：// 先↑   ■ ■ ■ □ □
          }                                                                                                                               //     ■ ■ ■ □ □
        }                                                                                                                                 //     ■ ■ ■后→
        patch_mean = patch_sum / patch_total;  //÷9求像素平均

        patch_sum = 0;//       1                         1
        for (int patch_x = x - patch_size; patch_x < x + patch_size + 1; patch_x++)  //做3次
        {//                      1                         1
          for (int patch_y = y - patch_size; patch_y < y + patch_size + 1; patch_y++)  //做3次
          {
            patch_x_clip = patch_x;
            patch_y_clip = patch_y;
            clip_view_x_y(patch_x_clip, patch_y_clip);  //约束x,y的上下限

            patch_sum += ((current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean) * (current_view_copy[patch_x_clip + patch_y_clip * TEMPLATE_X_SIZE] - patch_mean));
            //patch_sum为方差*9
          }
        }

        patch_std = sqrt(patch_sum / patch_total);  //patch_std为这9个点像素值与平均值的 方差

        if (patch_std < VT_MIN_PATCH_NORMALISATION_STD)  //vt_min_patch_normalisation_std未定义，默认值为0
          current_view[x + y * TEMPLATE_X_SIZE] = 0.5;
        else {
          current_view[x + y * TEMPLATE_X_SIZE] = max((double) 0, min(1.0, (((current_view_copy[x + y * TEMPLATE_X_SIZE] - patch_mean) / patch_std) + 3.0)/6.0 ));
          //将最小块标准化的一个算法，未知算法
        }
      }
    }
  }

  double sum = 0;

  // find the mean of the data计算数据的平均值
  for (int i = 0; i < current_view.size(); i++)
    sum += current_view[i];

  current_mean = sum/current_view.size();

}

// create and add a visual template to the collection创建视觉模板将current_view添加到模板结构体中
int LocalViewMatch::create_template()
{
  templates.resize(templates.size() + 1);  //长度变为1，之后开始加1
  VisualTemplate * new_template = &(*(templates.end() - 1));  //VisualTemplate为一个结构体类型，是一个视图模板，new_template指针为templates容器的最后一个成员的地址

  new_template->id = templates.size() - 1;  //id是从0开始计数
  double * data_ptr = &current_view[0];  //data_ptr指向被经过一种标准化处理的current_view[i]数组
  new_template->data.reserve(TEMPLATE_SIZE);  //告知长度为3000
  for (int i = 0; i < TEMPLATE_SIZE; i++)  //将这个视图模板放入该结构体中
    new_template->data.push_back(*(data_ptr++));

  new_template->mean = current_mean;  //整个模板像素平均值

  return templates.size() - 1;  //每个模板存在templates容器中，且返回当前模板ID
}

// compare a visual template to all the stored templates, allowing for 
// slen pixel shifts in each direction
// returns the matching template and the MSE  比较视觉模板所有存储的模板,允许slen像素变化每个方向返回匹配的模板和均方误差
void LocalViewMatch::compare(double &vt_err, unsigned int &vt_match_id)
{
  if (templates.size() == 0)  //第一次比较时并没有模板，直接返回
  {
    vt_err = DBL_MAX;
    vt_error = vt_err;
    return;
  }

  double *data = &current_view[0];  //经过标准化处理的视觉模板数据部分
  double mindiff, cdiff;
  mindiff = DBL_MAX;

  vt_err = DBL_MAX;
  int min_template = 0;

  double *template_ptr;
  double *column_ptr;
  double *template_row_ptr;
  double *column_row_ptr;
  double *template_start_ptr;
  double *column_start_ptr;
  int row_size;
  int sub_row_size;
  double *column_end_ptr;
  VisualTemplate vt;
  int min_offset;

  int offset;
  double epsilon = 0.005;

  if (VT_PANORAMIC)  //未设置，默认值为0,若为1则精算,若为0则粗算
  {

	  BOOST_FOREACH(vt, templates)  //遍历templates容器，依次将值赋给vt拿来用
    {
//                                                       0.005
	    if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)  //vt_match_threshold设置值为0.02，默认值为0.03,每次遍历用当前的模板像素平均值,去对比所有模板的像素平均值,差距太大就跳出BOOST本次循环
	      continue;

	// for each vt try matching the view at different offsets
	// try to fast break based on error already great than previous errors
	// handles 2d images shifting only in the x direction
	// note I haven't tested on a 1d yet.
  	  for (offset = 0; offset < TEMPLATE_X_SIZE; offset += VT_STEP_MATCH)  //做60次，vt_step_match为1，默认值为5
      {
        cdiff = 0;
        template_start_ptr = &vt.data[0] + offset;  //当前模板像素平均值临近的若干张模板
        column_start_ptr = &data[0];  //当前拍摄的模板
        row_size = TEMPLATE_X_SIZE;  //60
        column_end_ptr = &data[0] + TEMPLATE_SIZE - offset;  //第一次column_end_ptr指向的数据第3000位,也就是角码[2999]处,之后逐次减1靠前指
        sub_row_size = TEMPLATE_X_SIZE - offset;  //column_start_ptr到column_end_ptr间距离60,59,58...

	  // do from offset to end
//                        &data[0]                             &vt.data[0] + offset                 &data[0]+TEMPLATE_SIZE-offset   60                          60
	      for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)  //做50次,60个为1组
	      {//                                                                             等效于&data[0]+sub_row_size
	  	    for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)  //第一次做60次,第二次做59次...
	  	    {
    	      cdiff += abs(*column_ptr - *template_ptr);  //当前模板 右移 与每个临近的template每60位进行移位对比累加,就像百叶窗打开
	  	    }

		// fast breaks
		      if (cdiff > mindiff)
		        break;
	      }

	  // do from start to offset
	      template_start_ptr = &vt.data[0];
	      column_start_ptr = &data[0] + TEMPLATE_X_SIZE - offset;  //当前拍摄的模板从60为开始
	      row_size = TEMPLATE_X_SIZE;
	      column_end_ptr = &data[0] + TEMPLATE_SIZE;  //指向模板的结束
	      sub_row_size = offset;
	      for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
	      {
		      for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
		      {
		        cdiff += abs(*column_ptr - *template_ptr);  //当前模板 左移 与每个临近的template每60位进行移位对比之差累加,就像百叶窗打开
		      }

		// fast breaks
		    if (cdiff > mindiff)  //两次累加和不能无穷大
		      break;
	      }


	      if (cdiff < mindiff)  //找到那个区别最小的模板(应该是当前模板),存储它的ID号、累加次数
	      {
		     mindiff = cdiff;
		     min_template = vt.id;
		     min_offset = offset;
	      }
      } 
    }

	  vt_relative_rad = (double) min_offset/TEMPLATE_X_SIZE * 2.0 * M_PI;
	  if (vt_relative_rad > M_PI)
	    vt_relative_rad = vt_relative_rad - 2.0 * M_PI;
    vt_err = mindiff / (double) TEMPLATE_SIZE;
    vt_match_id = min_template;
    vt_error = vt_err;

  } else {

	  BOOST_FOREACH(vt, templates)
	  {

	    if (abs(current_mean - vt.mean) > VT_MATCH_THRESHOLD + epsilon)
	      continue;

	// for each vt try matching the view at different offsets
	// try to fast break based on error already great than previous errors
	// handles 2d images shifting only in the x direction
	// note I haven't tested on a 1d yet.
	    for (offset = 0; offset < VT_SHIFT_MATCH*2+1; offset += VT_STEP_MATCH)  //做9次,vt_shift_match设置值为4,默认值为25
	    {
	      cdiff = 0;
	      template_start_ptr = &vt.data[0] + offset;
	      column_start_ptr = &data[0] + VT_SHIFT_MATCH;
	      row_size = TEMPLATE_X_SIZE;
	      column_end_ptr = &data[0] + TEMPLATE_SIZE - VT_SHIFT_MATCH;
	      sub_row_size = TEMPLATE_X_SIZE - 2*VT_SHIFT_MATCH;

	      for (column_row_ptr = column_start_ptr, template_row_ptr = template_start_ptr; column_row_ptr < column_end_ptr; column_row_ptr+=row_size, template_row_ptr+=row_size)
	      {
		      for (column_ptr = column_row_ptr, template_ptr = template_row_ptr; column_ptr < column_row_ptr + sub_row_size; column_ptr++, template_ptr++)
		      {
		        cdiff += abs(*column_ptr - *template_ptr);
		      }

		// fast breaks
		    if (cdiff > mindiff)
		      break;
	      }

	      if (cdiff < mindiff)
	      {
		      mindiff = cdiff;
		      min_template = vt.id;
		      min_offset = 0;
	      }
	    }
	  }

    vt_relative_rad = 0;
    vt_err = mindiff / (double)(TEMPLATE_SIZE - 2 * VT_SHIFT_MATCH * TEMPLATE_Y_SIZE);
    vt_match_id = min_template;
    vt_error = vt_err;
  }
}

}
