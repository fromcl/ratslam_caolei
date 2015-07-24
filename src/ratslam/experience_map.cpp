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
#include "experience_map.h"
#include "../utils/utils.h"

#include <queue>
#include <float.h>
#include <iostream>

using namespace std;

namespace ratslam
{

ExperienceMap::ExperienceMap(ptree settings)
{
  get_setting_from_ptree(EXP_CORRECTION, settings, "exp_correction", 0.5);
  get_setting_from_ptree(EXP_LOOPS, settings, "exp_loops", 10);
  get_setting_from_ptree(EXP_INITIAL_EM_DEG, settings, "exp_initial_em_deg", 90.0);

  MAX_GOALS = 10;

  experiences.reserve(10000);
  links.reserve(10000);  //有6个常成员的结构体列表

  current_exp_id = 0;
  prev_exp_id = 0;
  waypoint_exp_id = -1;
  goal_timeout_s = 0;
  goal_success = false;

  accum_delta_facing = EXP_INITIAL_EM_DEG * M_PI/180;  //90×π÷180,为3.14/2
  accum_delta_x = 0;
  accum_delta_y = 0;
  accum_delta_time_s = 0;

  relative_rad = 0;

}

ExperienceMap::~ExperienceMap()
{
  links.clear();
  experiences.clear();
}

// create a new experience for a given position
//                                                   exp_id为action->dest_id为int32型的
int ExperienceMap::on_create_experience(unsigned int exp_id)  //为每张经验地图给定坐标值角度参数
{

  experiences.resize(experiences.size() + 1);  //experiences为vector类型，长度变为了1,之前的experiences.reserve(10000);并没有给它分配10000的空间
  Experience * new_exp = &(*(experiences.end() - 1));  //Experience为一个结构体类型，是经验地图的一个节点，new_exp指针为experiences容器的最后一个成员的地址

  if (experiences.size() == 0)                                                                 //struct Experience
  {                                                                                            //{
    new_exp->x_m = 0;                                                                          //  int id; //它自己的标识
    new_exp->y_m = 0;                                                                          //  double x_m, y_m, th_rad;
    new_exp->th_rad = 0;                                                                       //  int vt_id;
  }                                                                                            //
  else                                                                                         //  std::vector<unsigned int> links_from; //从这个经验地图链接
  {                                                                                            //  std::vector<unsigned int> links_to; //链接到本次经验地图
    new_exp->x_m = experiences[current_exp_id].x_m + accum_delta_x;  //experiences[id].x_m+x   //
    new_exp->y_m = experiences[current_exp_id].y_m + accum_delta_y;                            //
    new_exp->th_rad = clip_rad_180(accum_delta_facing);  //第一次为π/2+里程计角度               //  //目标导航
  }//                                       之后和里程计角度、相对弧度累加共同构成经验地图th_rad  //  double time_from_current_s;  //当前时间秒
  new_exp->id = experiences.size() - 1;  //id是从0开始计数                                      //  unsigned int goal_to_current, current_to_goal;  //目标到当前,当前到目标
                                                                                               //  template<typename Archive>
  new_exp->goal_to_current = -1;  //目标到当前为-1                                              //    void serialize(Archive& ar, const unsigned int version)
  new_exp->current_to_goal = -1;  //当前到目标为-1                                              //    {
                                                                                               //      ar & id;
  // Link the current experience to the last.                                                  //      ar & x_m & y_m & th_rad;
  // FIXME: jumps back to last set pose with wheel odom?                                       //      ar & vt_id;
  if (experiences.size() != 1)  //每个经验地图对应一个链接,当action为CREATE_NODE,CREATE_EDGE进入 //      ar & links_from & links_to;
    on_create_link(get_current_id(), experiences.size() - 1, 0);                               //      ar & time_from_current_s;
                                                                                               //      ar & goal_to_current & current_to_goal;
  return experiences.size() - 1;  //可以认为返回值是当前经验地图的id号                           //    }
}                                                                                              //};

// update the current position of the experience map
// since the last experience
//      on_odo(odo->twist.twist.linear.x, odo->twist.twist.angular.z, time_diff)
void ExperienceMap::on_odo(double vtrans, double vrot, double time_diff_s)
{
  vtrans = vtrans * time_diff_s;  //速度乘以时间
  vrot = vrot * time_diff_s;  //角速度乘以时间
  accum_delta_facing = clip_rad_180(accum_delta_facing + vrot);  //accum_delta_facing为π/2+里程计角度
  accum_delta_x = accum_delta_x + vtrans * cos(accum_delta_facing);  //坐标x,y,会在pc的dest_id小于当前经验地图id时在on_set_experience里清零,on_set_experience每次进入action_callback都会被执行
  accum_delta_y = accum_delta_y + vtrans * sin(accum_delta_facing);
  accum_delta_time_s += time_diff_s;  //累加时间,从进入vo开始算,不会被清零
}

// iterate the experience map. Perform a graph relaxing algorithm to allow
// the map to partially converge.
bool ExperienceMap::iterate()  //迭代,以将经过多次的同一条路重合起来
{
  int i;
  unsigned int link_id;
  unsigned int exp_id;
  Experience * link_from, *link_to;
  Link * link;
  double lx, ly, df;

  for (i = 0; i < EXP_LOOPS; i++)  //做20次
  {
    for (exp_id = 0; exp_id < experiences.size(); exp_id++)  //每张经验地图都会被做一次
    {
      link_from = &experiences[exp_id];  //link_from指向experiences

      for (link_id = 0; link_id < link_from->links_from.size(); link_id++)  //links_from.size()为每张经验地图所对应的links id,长度应该为1
      {
        //%             //% experience 0 has a link to experience 1 由经验0链接到经验1
        link = &links[link_from->links_from[link_id]];  //link指向links
        link_to = &experiences[link->exp_to_id];  //link_to指向experiences

        //%             //% work out where e0 thinks e1 (x,y) should be based on the stored
        //%             //% link information
        lx = link_from->x_m + link->d * cos(link_from->th_rad + link->heading_rad);  //累加出当前x,y的坐标值,应为link->heading_rad为0
        ly = link_from->y_m + link->d * sin(link_from->th_rad + link->heading_rad);

        //%             //% correct e0 and e1 (x,y) by equal but opposite amounts
        //%             //% a 0.5 correction parameter means that e0 and e1 will be fully
        //%             //% corrected based on e0's link information
        link_from->x_m += (link_to->x_m - lx) * EXP_CORRECTION;  //exp_correction为0.5
        link_from->y_m += (link_to->y_m - ly) * EXP_CORRECTION;
        link_to->x_m -= (link_to->x_m - lx) * EXP_CORRECTION;
        link_to->y_m -= (link_to->y_m - ly) * EXP_CORRECTION;

        //%             //% determine the angle between where e0 thinks e1's facing
        //%             //% should be based on the link information
        df = get_signed_delta_rad(link_from->th_rad + link->facing_rad, link_to->th_rad);  //的到两角间最小差

        //%             //% correct e0 and e1 facing by equal but opposite amounts
        //%             //% a 0.5 correction parameter means that e0 and e1 will be fully
        //%             //% corrected based on e0's link information
        link_from->th_rad = clip_rad_180(link_from->th_rad + df * EXP_CORRECTION);
        link_to->th_rad = clip_rad_180(link_to->th_rad - df * EXP_CORRECTION);
      }
    }
  }

  return true;
}

// create a link between two experiences创建一个经验链接
//由on_create_experience传参进来为on_create_link(get_current_id(), experiences.size() - 1, 0);   get_current_id()直接返回current_exp_id第一次进来为0
//              em->on_create_link(action->src_id, action->dest_id, action->relative_rad);
bool ExperienceMap::on_create_link(int exp_id_from, int exp_id_to, double rel_rad)
{
  Experience * current_exp = &experiences[exp_id_from];

  // check if the link already exists检查连接是否已存在
  for (unsigned int i = 0; i < experiences[exp_id_from].links_from.size(); i++)
  {
    if (links[experiences[current_exp_id].links_from[i]].exp_to_id == exp_id_to)
      return false;
  }

  for (unsigned int i = 0; i < experiences[exp_id_to].links_from.size(); i++)
  {
    if (links[experiences[exp_id_to].links_from[i]].exp_to_id == exp_id_from)
      return false;
  }

  links.resize(links.size() + 1);
  Link * new_link = &(*(links.end() - 1));

  new_link->exp_to_id = exp_id_to;  //每个经验地图对应一个相同id的links
  new_link->exp_from_id = exp_id_from;  //等同经验地图的id
  new_link->d = sqrt(accum_delta_x * accum_delta_x + accum_delta_y * accum_delta_y);  //这里存了一个整体位移
  new_link->heading_rad = get_signed_delta_rad(current_exp->th_rad, atan2(accum_delta_y, accum_delta_x));  //得到经验地图△theta和(x,y)点与x轴夹角之差
  new_link->facing_rad = get_signed_delta_rad(current_exp->th_rad, clip_rad_180(accum_delta_facing + rel_rad));  //得到经验地图△theta和经验地图累加角度之差
  new_link->delta_time_s = accum_delta_time_s;  //赋予累加时间,从进入vo开始算

  // add this link to the 'to exp' so we can go backwards through the em
  experiences[exp_id_from].links_from.push_back(links.size() - 1);
  experiences[exp_id_to].links_to.push_back(links.size() - 1);  //次两句告诉经验地图是和哪个links对应

  return true;
}

// change the current experience改变目前的经验
//             em->on_set_experience(action->dest_id, 0);
int ExperienceMap::on_set_experience(int new_exp_id, double rel_rad)  //当dest_id小于经验地图id时,就将累加出来的坐标值清空但保留角度值(猜测目的应该让pc赶上来)
{
  if (new_exp_id > experiences.size() - 1)
    return 0;

  if (new_exp_id == current_exp_id)
  {
    return 1;
  }

//猜测:此处可能是pc发生交汇,节点id直接跳回在之前的某一个点上,让current_exp_id等于dest_id使经验地图回到那个点上,但朝向的角另作维护

  prev_exp_id = current_exp_id;  //交给上一个id
  current_exp_id = new_exp_id;  //覆盖当前经验地图的id号
  accum_delta_x = 0;  //将累加出来的x,y清0
  accum_delta_y = 0;
  accum_delta_facing = clip_rad_180(experiences[current_exp_id].th_rad + rel_rad);  //给当前经验地图角度加0,也可能action->relative_rad,只有当dest_id小于经验id时相对弧度才有作用

  relative_rad = rel_rad;  //relative_rad相对弧度,该参数并未被使用

  return 1;
}

struct compare
{
  bool operator()(const Experience *exp1, const Experience *exp2)  //被用做谓词,time_from_current_s小的优先级高
  {
    return exp1->time_from_current_s > exp2->time_from_current_s;
  }
};

double exp_euclidean_m(Experience *exp1, Experience *exp2)  //返回相距位移  sqrt(△x²+△y²)
{
  return sqrt((double)((exp1->x_m - exp2->x_m) * (exp1->x_m - exp2->x_m) + (exp1->y_m - exp2->y_m) * (exp1->y_m - exp2->y_m)));
}

double ExperienceMap::dijkstra_distance_between_experiences(int id1, int id2)  //迪杰斯特拉算法计算两点间距
{
  double link_time_s;
  unsigned int id;

  std::priority_queue<Experience*, std::vector<Experience*>, compare> exp_heap;  //用vector创建一个Experience类型的优先级队列,谓词compare表示time_from_current_s小的优先级高,排在队首被先取出

  for (id = 0; id < experiences.size(); id++)  //先初始化队列,所有成员优先级一致,顺序未变
  {
    experiences[id].time_from_current_s = DBL_MAX;
    exp_heap.push(&experiences[id]);
  }

  experiences[id1].time_from_current_s = 0;  //将第一个id的地图成员放在队列末尾
  goal_path_final_exp_id = current_exp_id;  //取出当前位置所在经验地图的id

  while (!exp_heap.empty())  //检查是否为空队列,有成员则返回0
  {
    std::make_heap(const_cast<Experience**>(&exp_heap.top()), const_cast<Experience**>(&exp_heap.top()) + exp_heap.size(), compare());

    Experience* exp = exp_heap.top();
    if (exp->time_from_current_s == DBL_MAX)
    {
      return DBL_MAX;
    }
    exp_heap.pop();

    for (id = 0; id < exp->links_to.size(); id++)
    {
      Link *link = &links[exp->links_to[id]];
      link_time_s = exp->time_from_current_s + link->delta_time_s;
      if (link_time_s < experiences[link->exp_from_id].time_from_current_s)
      {
        experiences[link->exp_from_id].time_from_current_s = link_time_s;
        experiences[link->exp_from_id].goal_to_current = exp->id;
      }
    }

    for (id = 0; id < exp->links_from.size(); id++)
    {
      Link *link = &links[exp->links_from[id]];
      link_time_s = exp->time_from_current_s + link->delta_time_s;
      if (link_time_s < experiences[link->exp_to_id].time_from_current_s)
      {
        experiences[link->exp_to_id].time_from_current_s = link_time_s;
        experiences[link->exp_to_id].goal_to_current = exp->id;
      }
    }

    if (exp->id == id2)
    {
      return exp->time_from_current_s;
    }
  }

  // DB added to stop warning
  return DBL_MAX;
}

// return true if path to goal found如果路径发现目标，返回true
bool ExperienceMap::calculate_path_to_goal(double time_s)  //计算目标路径,形参time_s为里程计时间戳
{

  unsigned int id;
  waypoint_exp_id = -1;  //路径点id

  if (goal_list.size() == 0)  //goal_list为一个整型的双端容器(deque)
    return false;

  // check if we are within thres of the goal or timeout
  if (exp_euclidean_m(&experiences[current_exp_id], &experiences[goal_list[0]]) < 0.1 || ((goal_timeout_s != 0) && time_s > goal_timeout_s))
  {
    if (goal_timeout_s != 0 && time_s > goal_timeout_s)
    {
      //cout << "Timed out reaching goal ... sigh" << endl;
      goal_success = false;
    }
    if (exp_euclidean_m(&experiences[current_exp_id], &experiences[goal_list[0]]) < 0.1)
    {
      goal_success = true;
      //cout << "Goal reached ... yay!" << endl;
    }
    goal_list.pop_front();  //将deque容器头成员删除
    goal_timeout_s = 0;

    for (id = 0; id < experiences.size(); id++)
    {
      experiences[id].time_from_current_s = DBL_MAX;
    }
  }

  if (goal_list.size() == 0)
    return false;

  if (goal_timeout_s == 0)
  {
    double link_time_s;

    std::priority_queue<Experience*, std::vector<Experience*>, compare> exp_heap;

    for (id = 0; id < experiences.size(); id++)
    {
      experiences[id].time_from_current_s = DBL_MAX;
      exp_heap.push(&experiences[id]);
    }

    experiences[current_exp_id].time_from_current_s = 0;
    goal_path_final_exp_id = current_exp_id;

    std::make_heap(const_cast<Experience**>(&exp_heap.top()),
                   const_cast<Experience**>(&exp_heap.top()) + exp_heap.size(), compare());

    while (!exp_heap.empty())
    {
      Experience* exp = exp_heap.top();
      if (exp->time_from_current_s == DBL_MAX)
      {
        cout << "Unable to find path to goal" << endl;
        goal_list.pop_front();
        return false;
      }
      exp_heap.pop();

      for (id = 0; id < exp->links_to.size(); id++)
      {
        Link *link = &links[exp->links_to[id]];
        link_time_s = exp->time_from_current_s + link->delta_time_s;
        if (link_time_s < experiences[link->exp_from_id].time_from_current_s)
        {
          experiences[link->exp_from_id].time_from_current_s = link_time_s;
          experiences[link->exp_from_id].goal_to_current = exp->id;
        }
      }

      for (id = 0; id < exp->links_from.size(); id++)
      {
        Link *link = &links[exp->links_from[id]];
        link_time_s = exp->time_from_current_s + link->delta_time_s;
        if (link_time_s < experiences[link->exp_to_id].time_from_current_s)
        {
          experiences[link->exp_to_id].time_from_current_s = link_time_s;
          experiences[link->exp_to_id].goal_to_current = exp->id;
        }
      }

      if (!exp_heap.empty())
        std::make_heap(const_cast<Experience**>(&exp_heap.top()),
                       const_cast<Experience**>(&exp_heap.top()) + exp_heap.size(), compare());

    }

    // now do the current to goal links
    unsigned int trace_exp_id = goal_list[0];
    while (trace_exp_id != current_exp_id)
    {
      experiences[experiences[trace_exp_id].goal_to_current].current_to_goal = trace_exp_id;
      trace_exp_id = experiences[trace_exp_id].goal_to_current;
    }

    // means we need a new time out
    if (goal_timeout_s == 0)
    {
      goal_timeout_s = time_s + experiences[goal_list[0]].time_from_current_s;
      cout << "Goal timeout in " << goal_timeout_s - time_s << "s" << endl;
    }
  }

  return true;
}

bool ExperienceMap::get_goal_waypoint()
{
  if (goal_list.size() == 0)
    return false;

  waypoint_exp_id = -1;

  double dist;
  unsigned int trace_exp_id = goal_list[0];
  Experience *robot_exp = &experiences[current_exp_id];

  while (trace_exp_id != goal_path_final_exp_id)
  {
    dist = exp_euclidean_m(&experiences[trace_exp_id], robot_exp);
    waypoint_exp_id = experiences[trace_exp_id].id;
    if (dist < 0.2)
    {
      break;
    }
    trace_exp_id = experiences[trace_exp_id].goal_to_current;
  }

  if (waypoint_exp_id == -1)
    waypoint_exp_id = current_exp_id;

  return true;
}

//                  add_goal(pose->pose.position.x, pose->pose.position.y);
void ExperienceMap::add_goal(double x_m, double y_m)  //未发布目标点就不会进入,记录下目标点和经验地图的某个点距离小于0.1米的目标点
{
  int min_id = -1;
  double min_dist = DBL_MAX;
  double dist;
  //goal_list为一个整型的双端容器(deque),未指定过长度
  if (MAX_GOALS != 0 && goal_list.size() >= MAX_GOALS)  //MAX_GOALS等于10,goal_list.size()大于10就会return
    return;

  for (unsigned int i = 0; i < experiences.size(); i++)  //找到经验地图中距离目标点最小的位移距离的点，记录该距离和该点id号
  {
    dist = sqrt((experiences[i].x_m - x_m) * (experiences[i].x_m - x_m) + (experiences[i].y_m - y_m) * (experiences[i].y_m - y_m));
    if (dist < min_dist)
    {
      min_id = i;
      min_dist = dist;
    }
  }

  if (min_dist < 0.1)
    add_goal(min_id);  //另一个重载函数，等效于goal_list.push_back(id);若目标点和经验地图的某个点小于0.1米，就记录下这个地图id,最多记录10个

}

double ExperienceMap::get_subgoal_m() const
{
  return (waypoint_exp_id == -1 ? 0 : sqrt((double)pow((experiences[waypoint_exp_id].x_m - experiences[current_exp_id].x_m), 2) + (double)pow((experiences[waypoint_exp_id].y_m - experiences[current_exp_id].y_m), 2)));
}

double ExperienceMap::get_subgoal_rad() const
{
  if (waypoint_exp_id == -1)
    return 0;
  else
  {
    double curr_goal_rad = atan2((double)(experiences[waypoint_exp_id].y_m - experiences[current_exp_id].y_m),
                                 (double)(experiences[waypoint_exp_id].x_m - experiences[current_exp_id].x_m));
    return (get_signed_delta_rad(experiences[current_exp_id].th_rad, curr_goal_rad));
  }
}

} // namespace ratslam

