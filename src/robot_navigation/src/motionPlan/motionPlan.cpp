/*
 * @Author: your name
 * @Date: 2023-04-28 10:46:52
 * @LastEditTime: 2025-04-12 20:56:21
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_navigation/src/motionPlan/motionPlan.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "motionPlan/motionPlan.h"


#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <tf/transform_listener.h>

// 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
motionPlan::motionPlan(/* args */) : private_node("~")
{
  // 初始化
  motionInit();

  ros::Rate LoopRate(control_hz);
  while (ros::ok())
  {

    auto timeStart = steady_clock::now();
    // 回调函数数据刷新
    ros::spinOnce();
    pathPlanning(startPoint,endPoint);
    //--运行速度测量
    auto timeEnd = steady_clock::now();
    auto timeDuration = duration_cast<microseconds>(timeEnd - timeStart);
    // cout << "运行时间 " << timeDuration.count()/1000 << endl;
    LoopRate.sleep();
  }

}

motionPlan::~motionPlan()
{
}

void motionPlan::BuildPathTest_Follow(void)
{
  BuildFixPath(pathNav.worldpath);
  PublishPath(optPathPub,pathNav.worldpath);
}

void motionPlan::motionInit(void)
{
  // 参数初始化
  startPoint = Eigen::Vector2d(0.00f, 0.00f);
  endPoint = Eigen::Vector2d(0.00f, 0.00f);

  // 获取是否仿真
  private_node.param("motion_node/is_sim", is_sim, false); // 权重a值
  // 获取是否巡航标志位
  private_node.param("motion_node/is_cruise", is_cruise, false); // 权重a值
  // 获取控制频率
  private_node.param("motion_plan/control_hz",control_hz,50);
  // 获取使用哪种规划算法
  private_node.param("motion_plan/control_method",control_method,1);
  // 获取停车边缘距离
  private_node.param("motion_plan/stop_margin",stop_margin,0.1);

  private_node.param("cruise_num",cruise_num,5);
  // 加载巡航点
  private_node.param("first_point_x",first_point_x,0.1);
  private_node.param("first_point_y",first_point_y,0.1);
  private_node.param("second_point_x",second_point_x,0.1);
  private_node.param("second_point_y",second_point_y,0.1);
  private_node.param("third_point_x",third_point_x,0.1);
  private_node.param("third_point_y",third_point_y,0.1);
  private_node.param("forth_point_x",forth_point_x,0.1);
  private_node.param("forth_point_y",forth_point_y,0.1);
  private_node.param("fifth_point_x",fifth_point_x,0.1);
  private_node.param("fifth_point_y",fifth_point_y,0.1);

  // 重新加载巡航点数目
  cruise_points.resize(cruise_num);
  // 当进入巡航模式的时候巡航
  if(is_cruise == true)
  {
    cruise_points[0][0] = first_point_x,cruise_points[0][1] = first_point_y;
    cruise_points[1][0] = second_point_x,cruise_points[1][1] = second_point_y;
    cruise_points[2][0] = third_point_x,cruise_points[2][1] = third_point_y;
    cruise_points[3][0] = forth_point_x,cruise_points[3][1] = forth_point_y;
    cruise_points[4][0] = fifth_point_x,cruise_points[4][1] = fifth_point_y;
  }

  // minimum_snap路径优化参数加载
  double max_vel = 1.0f, max_acce = 1.0f;
  int order = 0;
  private_node.param("minimum_snap/max_vel", max_vel, 1.0);
  private_node.param("minimum_snap/max_acce", max_acce, 1.0);
  private_node.param("minimum_snap/order", order, 3);
  MinimumSnapFlow.setParams(order, max_vel, max_acce);

  // bezier_curve路径优化参数加载
  double max_vel_b = 1.0f, time_resolution = 0.01;
  int sub = 3;
  private_node.param("bezier_curve/sub", sub, 3);
  private_node.param("bezier_curve/max_vel", max_vel_b, 1.0);
  private_node.param("bezier_curve/time_resolution", 0.01);
  Bezier_Flow.setParams(max_vel_b, time_resolution, sub);

  // 路径跟随参数加载
  private_node.param("pid_p", pidFollow.pid_params[0], 1.0);
  private_node.param("pid_i", pidFollow.pid_params[1], 0.0);
  private_node.param("pid_d", pidFollow.pid_params[2], 0.0);
  private_node.param("pid_outmax", pidFollow.pid_params[3], 0.0);
  private_node.param("pid_outmin", pidFollow.pid_params[4], 0.0);
  private_node.param("pid_intmax", pidFollow.pid_params[5], 0.0);
  private_node.param("pid_intdt", pidFollow.pid_params[5], 0.0);
  private_node.param("forwardDistance", pidFollow.forwardDistance, 0.0);

  pidFollow.Init();

  //初始化路径算法 20251209
  astar_path_finder.reset(new AstarPathFinder);
  astar_path_finder->InitParams(private_node);

  astar_esdf_path_finder.reset(new AstarPathFinder);
  astar_esdf_path_finder->InitParams(private_node,true);

  jps_path_finder.reset(new JPSPathFinder);
  // 参数初始化
  jps_path_finder->InitParams(private_node);

  hybrid_astar_finder.reset(new Hybrid_astar);
  // 混合astar初始化
  hybrid_astar_finder->InitParams(private_node);

  // 
  rrt_path_finder.reset(new RRTstarPlanner);

  // Fast_Security初始化-初始化安全优化模块
  fast_security_opt.reset(new Fast_Security);
  fast_security_opt->InitParams(private_node);

  // 订阅静态地图
  staticMap = motPlan.subscribe("/global_map_esdf_display", 10, &motionPlan::staticMapCallback, this);
  // staticMap = motPlan.subscribe("/local_map_esdf", 10, &motionPlan::staticMapCallback, this);
  // 订阅鼠标点击目标点
  clickSub = motPlan.subscribe("/move_base_simple/goal", 10, &motionPlan::clickCallback, this);
  // 发布全局终点
  globalEndPub = motPlan.advertise<geometry_msgs::PoseStamped>("/globalEnd",1);
  // 订阅rviz初始位姿消息发布
  initalposeSub = motPlan.subscribe("/initialpose", 10, &motionPlan::initalPoseCallback,this);

  // 仿真和实际的定位消息是不一样的
  if (is_sim == false)
  {
    // 订阅定位消息 carto_odom
    //localizationSub = motPlan.subscribe("/odom_carto", 10, &motionPlan::localizationCallback, this);
    //订阅mid360雷达的位置消息
    localizationSub = motPlan.subscribe("/carto_odom", 10, &motionPlan::localizationCallback_mid360, this);
  
  }
  else
  {
    // 订阅真值里程计进行仿真
    localizationSub = motPlan.subscribe("/truth_pose_odom", 10, &motionPlan::localizationCallback, this);
  }
  // 订阅局部地图
  dynamicMap = motPlan.subscribe("/local_map_esdf", 10, &motionPlan::dynamicMapCallback, this);
  // 订阅局部速度
  localVelocitySub = motPlan.subscribe("/local_velocity", 10, &motionPlan::localVelocityCallback, this);

  // 发布话题
  // 规划路径发布
  oriPathPub = motPlan.advertise<nav_msgs::Path>("/ori_path", 10);
  // 优化路径发布
  optPathPub = motPlan.advertise<nav_msgs::Path>("/opt_path", 10);
  // 2次优化路径发布
  optPath2Pub = motPlan.advertise<nav_msgs::Path>("/opt2_path", 10);

  esdfPathPub = motPlan.advertise<nav_msgs::Path>("/esdf_path", 10);

  // 查看补充扩展节点
  optpathNodePub = motPlan.advertise<visualization_msgs::Marker>("/Opt_path_Nodes",10);
  // 发布终点可视化
  goalPointPub = motPlan.advertise<visualization_msgs::Marker>("/goal_point",1);
  // 发布起点可视化
  startPointPub = motPlan.advertise<visualization_msgs::Marker>("/start_point",1);
  
  // 发布终点可视化
  firstPointPub = motPlan.advertise<visualization_msgs::Marker>("/first_point",1);
  // 发布起点可视化
  secondPointPub = motPlan.advertise<visualization_msgs::Marker>("/second_point",1);

  // 获取frame_id名称
  if (!private_node.getParam("motion_plan/frame_id", frame_id_name))
  {
    frame_id_name = "odom";
  }
}

void motionPlan::BuildFixPath(vector<Eigen::Vector2d> &fixpath)
{
  static int start = 0;
#if 0

  start++;
  if(start >= 1900)
  {
    start = 1900;
  }

  fixpath.resize(2000-start+1);

  fixpath[0](0) = startPoint(0);
  fixpath[0](1) = startPoint(1);

  for (int i = start; i<2000; i++)
  {
    double x = i/100.0;
    double sinx = 2*sin(x);
    // double cosx = 2*cos(x);
    double y = x + sinx;

    fixpath[i-start+1](0) = x;
    fixpath[i-start+1](1) = y;
  }
#endif

#if 0

  start++;
  if(start >= 1890)
  {
    start = 1860;
  }

  fixpath.resize(1890-start+1);

  // 第一个点为起点
  fixpath[0](0) = startPoint(0);
  fixpath[0](1) = startPoint(1);
  
  // 画个圆
  for(int i = start; i<1890; i++)
  {
    double t = (i-942)/300.0;
    double x = 4+3*cos(t);
    double y = 4+3*sin(t);

    fixpath[i-start+1](0) = x;
    fixpath[i-start+1](1) = y;    
  }

#endif

#if 1
  start++;
  if(start >= 2500)
  {
    start = 2400;
  }

  fixpath.resize(2500-start+1);

  fixpath[0](0) = startPoint(0);
  fixpath[0](1) = startPoint(1);

  for (int i = start; i<2500; i++)
  {
    double x,y;
    if(i<=240)
    {
      x = 0;
      y = i/40.0;
    }
    else if(i>240 && i <= 400)
    {
      x = (i-240)/40.0;
      y = 6;
    }
    else if(i>400 && i <= 1040)
    {
      x = 4;
      y = 6+(i-400)/40.0;
    }
    else if(i>1040 && i<=1440)
    {
      x = 4+(i-1040)/40.0;
      y = 22;
    }
    else if(i>1440 && i <= 1680)
    {
      x = 14;
      y = 22 - (i-1440)/40.0;
    }
    else if(i>1680 && i <= 1840)
    {
      x = 14 + (i-1680)/40.0;
      y = 16;
    }
    else if(i>1840 && i <= 2160)
    {
      x = 18;
      y = 16 - (i-1840)/40.0;
    }
    else if(i>2160 && i <= 2400)
    {
      x = 18 - (i-2160)/40.0;
      y = 8;
    }

    fixpath[i-start+1](0) = x;
    fixpath[i-start+1](1) = y;
  }
#endif

#if 0
  start++;
  if(start >= 1900)
  {
    start = 1900;
  }

  fixpath.resize(2000-start+1);

  fixpath[0](0) = startPoint(0);
  fixpath[0](1) = startPoint(1);

  for (int i = start; i<2000; i++)
  {
    double x,y;
    if(i<=120)
    {
      x = i/20.0;
      y = 0;
    }
    else if(i>120 && i <= 240)
    {
      x = 6;
      y = (i-120)/20.0;
    }
    else if(i>240 && i <= 360)
    {
      x = 6-(i-240)/20.0;
      y = 6;
    }
    else if(i>360 && i <= 480)
    {
      x = 0;
      y = 6 - (i-360)/20.0;
    }

    fixpath[i-start+1](0) = x;
    fixpath[i-start+1](1) = y;
  }  
#endif
}

/**
 * @brief:接收鼠标点击 回调函数
 * @note:
 */

// double END_X[49] = {22.286304, 13.190655, 23.871437, 15.892069, 14.681456, -0.026707, 4.414207, 25.583168, 3.525618, 24.400595, 15.935325, 13.48995, 2.573387, 2.086534, 26.071432, 3.105234, 13.314043, 8.637252, 23.917389, 12.289492, 24.355686, 2.995398, 6.018532, 4.476551, 24.401062, 10.258701, 19.899309, 24.646425, 3.937813, -0.363672, 24.778547, 9.682827, 17.741503, 26.226082, -0.766001, 3.4923, 12.698448, 19.655499, 15.543932, 22.556267, 24.246189, 23.928759, -0.277508, 24.966366, 18.952032, 0.85046, 25.995693, -0.586432, 22.88752};
// double END_Y[49] = {24.702721, 0.075047, 18.234934, 26.149124, 2.74314, 0.044941, 25.123859, 18.674471, 25.792923, 9.87179, 26.765629, -0.501511, 24.994278, -0.427064, 16.863529, 5.948899, 22.496895, 2.645923, 13.771075, 26.759895, -0.720959, 20.456001, -0.677169, 26.385349, 9.684115, -1.13895, 25.669258, 2.212066, 25.179253, 11.057452, 20.284357, -1.104068, 26.100925, 1.2653, 2.199572, 25.763985, -0.348499, 25.586298, 4.353338, 25.208347, -0.688014, 25.704678, 26.538513, -0.800352, 24.889019, 2.031443, 15.947336, 14.373705, 1.894693};
void motionPlan::clickCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  global_goal = *msg;

  double roll, pitch, yaw;
  // 四元数向欧拉角转换，当前车的角度
  tf2::Quaternion quat(
      global_goal.pose.orientation.x,
      global_goal.pose.orientation.y,
      global_goal.pose.orientation.z,
      global_goal.pose.orientation.w);
  // 四元数向欧拉角进行转换 对local的角度
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  // 如果不巡航的话直接按照现给的点作为终点
  if(is_cruise == false)
  {
    // 获取规划点
    endPoint[0] = msg->pose.position.x;
    endPoint[1] = msg->pose.position.y;
    getEndFlag = true;
    visualPoints(goalPointPub,endPoint,1,1,0,0,5);
  }
  else
  {
    // 巡航模式按照固定的点去给
    // 获取规划点
    endPoint = cruise_points[0];
    getEndFlag = true;
    visualPoints(goalPointPub,endPoint,1,1,0,0,5);    
  }

  // static int map_point = 1;
  // if(map_point == 100)
  // {
  //   map_point = 1;
  // }

  // cout<<"当前起点终点位置:"<<map_point<<endl;
  // startPoint[0] = START_X[map_point-1];
  // startPoint[1] = START_Y[map_point-1];

  // endPoint[0] = END_X[map_point-1];
  // endPoint[1] = END_Y[map_point-1];
  // map_point ++;

  // static int map_point = 3;
  // if(map_point == 4)
  // {
  //   map_point = 3;
  // }

  // cout<<"当前起点终点位置:"<<second_test[map_point]<<endl;
  // startPoint[0] = START_X[second_test[map_point]];
  // startPoint[1] = START_Y[second_test[map_point]];

  // endPoint[0] = END_X[second_test[map_point]];
  // endPoint[1] = END_Y[second_test[map_point]];
  // map_point ++;

  // visualPoints(goalPointPub,endPoint,1,1,0,0,5);
  // getEndFlag = true;
  // getStartFlag = true; // 获取到起点标志位

  // static int map_point = 9;
  // if(map_point == 50)
  // {
  //   map_point = 9;
  // }
  // cout<<"设定起点终点位置:"<<map_point<<endl;
  // Vector2d firstPoint,secondPoint;
  // firstPoint[0] = START_X[map_point-1];
  // firstPoint[1] = START_Y[map_point-1];

  // secondPoint[0] = END_X[map_point-1];
  // secondPoint[1] = END_Y[map_point-1];

  // // endPoint[0] = 22.780041;
  // // endPoint[1] = -17.675241;

  // endPoint = secondPoint;

  // map_point ++;

  // global_goal.pose.position.x = secondPoint[0];
  // global_goal.pose.position.y = secondPoint[1];
  // globalEndPub.publish(global_goal);

  // visualPoints(firstPointPub,firstPoint,1,0,0,1,3);
//   visualPoints(secondPointPub,secondPoint,1,1,0,1,3);
//   getEndFlag = true;
//   visualPoints(goalPointPub,endPoint,1,1,0,0,5);
}

// 发布可视化的点，起点终点这类的点
void motionPlan::visualPoints(ros::Publisher pointPublish,Vector2d visitnodes,float a_set,float r_set,float g_set,float b_set,float length)
{
  visualization_msgs::Marker node_vis2;
  node_vis2.header.frame_id = "map";
  node_vis2.header.stamp = ros::Time::now();
  node_vis2.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis2.action = visualization_msgs::Marker::ADD;
  node_vis2.id = 0;

  node_vis2.pose.orientation.x = 0.0;
  node_vis2.pose.orientation.y = 0.0;
  node_vis2.pose.orientation.z = 0.0;
  node_vis2.pose.orientation.w = 1.0;

  node_vis2.color.a = a_set;
  node_vis2.color.r = r_set;
  node_vis2.color.g = g_set;
  node_vis2.color.b = b_set;
  node_vis2.scale.x = pathNav.resolution*length;
  node_vis2.scale.y = pathNav.resolution*length;
  node_vis2.scale.z = pathNav.resolution*length;

  geometry_msgs::Point pt2;
  pt2.x = visitnodes[0];
  pt2.y = visitnodes[1];
  pt2.z = 0.0;
  node_vis2.points.push_back(pt2);
  pointPublish.publish(node_vis2);
}

// 初始化位姿回调函数
void motionPlan::initalPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  // 初始位置回调函数
  startPoint[0] = msg->pose.pose.position.x;
  startPoint[1] = msg->pose.pose.position.y;

  if(fabs((startPoint-endPoint).norm()) <= stop_margin)
  {
    has_arrived_end = true;
  }
  else{
    has_arrived_end = false;
  }

  getStartFlag = true; // 获取到起点标志位
  visualPoints(startPointPub,startPoint,1,0,1,0,5);
}

void motionPlan::localizationCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg)
{
  // 获取起点等规划消息
  localData.xPosition = msg->xPosition;
  localData.yPosition = msg->yPosition;
  localData.xSpeed = msg->xSpeed;
  localData.ySpeed = msg->ySpeed;
  localData.xAccel = msg->xAccel;
  localData.yAccel = msg->yAccel;
  localData.chassisGyro = msg->chassisGyro;
  localData.chassisAngle = msg->chassisAngle;

  startPoint[0] = msg->xPosition;
  startPoint[1] = msg->yPosition;

  static int i = 0;
  if(fabs((startPoint-endPoint).norm()) <= stop_margin)
  {
    has_arrived_end = true;
    // 巡航的时候
    if(is_cruise)
    {
      endPoint = cruise_points[i];
      getEndFlag = true;
      visualPoints(goalPointPub,endPoint,1,1,0,0,5); 
      i ++;
    }
  }
  else
  {
    has_arrived_end = false;
    // 如果巡航
    if(is_cruise)
    {
      // 巡航模式按照固定的点去给
      // 获取规划点
      if(i == 5)
      {
        i = 0;
      }
    }
  }
  getStartFlag = true; // 获取到起点标志位

  // 2026-1-7修改 全局规划一直运行
  //getEndFlag = true;
}
// 改变消息类型，
void motionPlan::localizationCallback_mid360(const nav_msgs::OdometryConstPtr& msg)
{
    // 1) 从 Odometry 提取位姿
  // ---------------------------
  const double x = msg->pose.pose.position.x;
  const double y = msg->pose.pose.position.y;
  // 获取起点等规划消息
  localData.xPosition = msg->pose.pose.position.x;
  localData.yPosition = msg->pose.pose.position.y;

  // yaw：四元数 -> 欧拉角
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  {
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  // 2) 从 Odometry 提取速度
  // ---------------------------
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  const double wz = msg->twist.twist.angular.z;   // 作为 gyro / 角速度

  // ---------------------------
  // 3) 填充你的 localData（保持字段意义一致）
  // ---------------------------
  localData.xPosition = x;
  localData.yPosition = y;

  localData.xSpeed = vx;
  localData.ySpeed = vy;

  // nav_msgs/Odometry 没有加速度：先置0（最安全）
  localData.xAccel = 0.0;
  localData.yAccel = 0.0;

  // gyro 用角速度 z（如果你原来 chassisGyro 表示 yaw rate）
  localData.chassisGyro  = wz;

  // chassisAngle 用 yaw（如果你原来是底盘航向角）
  localData.chassisAngle = yaw;

  // 4) 起点赋值
  // ---------------------------
  startPoint[0] = x;
  startPoint[1] = y;

  static int i = 0;
  if(fabs((startPoint-endPoint).norm()) <= stop_margin)
  {
    has_arrived_end = true;
    // 巡航的时候
    if(is_cruise)
    {
      endPoint = cruise_points[i];
      getEndFlag = true;
      visualPoints(goalPointPub,endPoint,1,1,0,0,5); 
      i ++;
    }
  }
  else
  {
    has_arrived_end = false;
    // 如果巡航
    if(is_cruise)
    {
      // 巡航模式按照固定的点去给
      // 获取规划点
      if(i == 5)
      {
        i = 0;
      }
    }
  }
  getStartFlag = true; // 获取到起点标志位

  // 2026-1-7修改 全局规划一直运行
  //getEndFlag = true;
}

// 静态地图订阅回调函数
void motionPlan::staticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  pathNav.origin_x = msg->info.origin.position.x; // 获得栅格地图的原点x值(相对世界坐标系),单位为m
  pathNav.origin_y = msg->info.origin.position.y; // 获得栅格地图的原点y值(相对世界坐标系),单位为m
  pathNav.resolution = msg->info.resolution;      // 获得栅格地图的分辨率
  pathNav.width = msg->info.width;                // 获得栅格地图的宽
  pathNav.height = msg->info.height;              // 获得栅格地图的高

  if(stop_margin < pathNav.resolution)
  {
    stop_margin = pathNav.resolution;
  }

  ROS_INFO("\033[1;32m ***********map message**********\033[0m");
  ROS_INFO("\033[1;32m origin_x: %f  \033[0m",
           pathNav.origin_x);
  ROS_INFO("\033[1;32m origin_y: %f  \033[0m",
           pathNav.origin_y);
  ROS_INFO("\033[1;32m resolution: %f  \033[0m",
           pathNav.resolution);
  ROS_INFO("\033[1;32m width: %d  \033[0m",
           pathNav.width);
  ROS_INFO("\033[1;32m height: %d  \033[0m",
           pathNav.height);
  ROS_INFO("\033[1;32m *********************************\033[0m");

  pathNav.mapData.resize(pathNav.width * pathNav.height);
  for (int i = 0; i < pathNav.width; i++)
  {
    for (int j = 0; j < pathNav.height; j++)
    { /* 这行代码非常非常非常重要，将ros解析的地图转变为正常的先行后列的顺序 */
      pathNav.mapData[i * pathNav.height + j] = int(msg->data[j * pathNav.width + i]);
    }
  }
  
  if(control_method == 1)
  {
    astar_path_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                              pathNav.width, pathNav.height, pathNav.mapData);

    // 对照实验esdf a*地图初始化
    astar_esdf_path_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                              pathNav.width, pathNav.height, pathNav.mapData); 
    // 设定地图参数
    fast_security_opt->SetMapParams(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                              pathNav.width, pathNav.height,msg->data);
  }
  if(control_method == 2)
  {
    hybrid_astar_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                              pathNav.width, pathNav.height,pathNav.mapData);
  }
  if(control_method == 3)
  {
    jps_path_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                              pathNav.width, pathNav.height, pathNav.mapData);
  }
  if(control_method == 4)
  {
    rrt_path_finder->initMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                              pathNav.width, pathNav.height,pathNav.mapData);
  }

  mapInitFlag = true;
}

// 动态地图订阅回调函数
// void motionPlan::dynamicMapCallback(const robot_communication::ESDFmap::ConstPtr &map)
void motionPlan::dynamicMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  vector<int> tempmap;
  tempmap.resize(map->info.width * map->info.height);
  for (int i = 0; i < map->info.width; i++)
  {
    for (int j = 0; j < map->info.height; j++)
    { /* 这行代码非常非常非常重要，将ros解析的地图转变为正常的先行后列的顺序 */
      tempmap[i * map->info.height + j] = int(map->data[j * map->info.width + i]);
    }
  }

  // hybrid_astar_finder->InitMap(map->info.resolution, 
  //                             map->info.origin.position.x, 
  //                             map->info.origin.position.y,
  //                             map->info.width, 
  //                             map->info.height,tempmap);

  // cout<<"map size:"<<map->info.width<<"  "<<map->info.height<<endl;
  localMapFlag = true;
}

// 规划出来的局部速度的回调函数
void motionPlan::localVelocityCallback(const robot_communication::chassisControlConstPtr &velocity)
{
  motionLocal = *velocity;
}

// 
void motionPlan::visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.color.a = a_set;
  node_vis.color.r = r_set;
  node_vis.color.g = g_set;
  node_vis.color.b = b_set;
  node_vis.ns = "fast_security_visited";

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.scale.x = 0.05*length;
  node_vis.scale.y = 0.05*length;
  node_vis.scale.z = 0.05*length;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(visitnodes.size()); i++)
  {
    pt.x = visitnodes[i][0];
    pt.y = visitnodes[i][1];
    node_vis.points.push_back(pt);
  }

  pathPublish.publish(node_vis);  
}

void motionPlan::PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path)
{
  nav_msgs::Path pathTopic; // Astar路径的话题名
  pathTopic.header.frame_id = frame_id_name;
  for (unsigned int i = 0; i < path.size(); i++)
  {
    geometry_msgs::PoseStamped pathPose;
    pathPose.pose.position.x = path[i][0];
    pathPose.pose.position.y = path[i][1];
    pathPose.pose.position.z = 0;

    pathTopic.header.stamp = ros::Time::now();
    pathTopic.poses.push_back(pathPose);
  }
  pathPublish.publish(pathTopic);
}

// 路径规划
void motionPlan::pathPlanning(Eigen::Vector2d startMapPoint, Eigen::Vector2d goalMapPoint)
{
  // 输出终点坐标
  // cout<<"Goal is:"<<goalMapPoint[0]<<" "<<goalMapPoint[1]<<endl;
  // if(!localMapFlag){
  //   cout<<"Not have local map!"<<endl;
  //   return;
  // }



  if(has_arrived_end){
    cout<<"Arrived the end!"<<endl;
    return;
  }

  // 使用普通astar进行规划
  if(control_method == 1)
  {
    // 开始进行规划
    if (mapInitFlag == true && getStartFlag == true )
    {
      if(getEndFlag == true)  //需要重新规划的时候
      {
        static tf::TransformListener listener;
        tf::StampedTransform transform;
        try {
            // 注意：这里 frame_id_name 必须是 "map"！
            if (listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0))) {
                listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
                startPoint[0] = transform.getOrigin().x();
                startPoint[1] = transform.getOrigin().y();
                startMapPoint = startPoint; // 更新传入参数
                ROS_INFO("【重新规划】当前起点: (%.2f, %.2f)", startPoint[0], startPoint[1]);
            }
        } catch (tf::TransformException &ex) {
            ROS_ERROR("获取起点失败: %s", ex.what());
            return; // 没获取到位置就别规划了y
        }


        cout<<"Start is: "<<startMapPoint[0]<<" "<<startMapPoint[1]<<endl;
        cout<<"Goal is: "<<goalMapPoint[0]<<" "<<goalMapPoint[1]<<endl;

        // // 使用过的地图复位
        // astar_path_finder->resetUsedGrids();
        // // 记录路径搜索需要的时间
        // ros::Time time_1 = ros::Time::now();
        // // 开始路径搜索
        // astar_path_finder->AstarWorldSearch(startMapPoint,goalMapPoint);
        // ros::Time time_2 = ros::Time::now();
        // // 获取astar搜索到的路径
        // pathNav.worldpath = astar_path_finder->getWorldPath();
        // ROS_WARN("astar search time is:%f ms",(time_2-time_1).toSec() * 1000.0);
        // int astar_sum_esdf;
        // double astar_ave_esdf;
        // astar_path_finder->GetPath_ESDFvalue(astar_sum_esdf,astar_ave_esdf);
        // cout<<"astar_sum_esdf: "<<astar_sum_esdf<<"  "<<"astar_ave_esdf: "<<astar_ave_esdf
        // <<"  "<<"astar_num: "<<pathNav.worldpath.size()<<endl;
        // PublishPath(oriPathPub,pathNav.worldpath); // 发布路径

        // // 优化路径
        // ros::Time time_3 = ros::Time::now();
        // pathNav.optpath = fast_security_opt->Fast_Security_Search(pathNav.worldpath);
        // ros::Time time_4 = ros::Time::now();
        // cout<<"fast security 扩展总的栅格数目: "<<fast_security_opt->getCorridorVisitedNum()<<endl;
        // cout<<"fast security 优化的路径长度: "<<fast_security_opt->getOptedPathLength()<<" m"<<endl;
        // ROS_WARN("path opt time is: %f ms",(time_4-time_3).toSec() * 1000.0);
        // int fast_sum_esdf,fast_num;
        // double fast_ave_esdf;
        // fast_security_opt->GetPathESDFvalue(fast_sum_esdf,fast_ave_esdf,fast_num);
        // cout<<"astar_sum_esdf: "<<fast_sum_esdf<<"  "<<"astar_ave_esdf: "<<fast_ave_esdf<<
        // "  "<<"fast_num: "<<fast_num<<endl;
        // fast_security_opt->visualNodes();
        // PublishPath(optPathPub,pathNav.optpath);
        // visual_VisitedNode(optpathNodePub,pathNav.optpath,1,1,0.5,0.1,2);

        // 使用过的地图复位
        astar_esdf_path_finder->resetUsedGrids();
        // 记录路径搜索需要的时间
        ros::Time time_5 = ros::Time::now();
        // 开始路径搜索
        astar_esdf_path_finder->AstarWorldSearch(startMapPoint,goalMapPoint);
        // 获取astar_esdf搜索到的路径
        vector<Vector2d> World_Path = astar_esdf_path_finder->getWorldPath();
        pathNav.worldpath = astar_esdf_path_finder->getWorldPath();
        World_Path = astar_esdf_path_finder->getSamples(0.5);

        pathNav.optpath = MinimumSnapFlow.Minimum_Snap(World_Path);
        ros::Time time_6 = ros::Time::now();
        cout<<"astar_esdf 扩展总的栅格数目: "<<astar_esdf_path_finder->getVisitedNodesNum()<<endl;
        cout<<"astar_esdf 优化的路径长度: "<<astar_esdf_path_finder->getWorldPathLength()<<" m"<<endl;
        ROS_WARN("astar esdf search time is: %f ms",(time_6-time_5).toSec() * 1000.0);

        PublishPath(oriPathPub,pathNav.worldpath); // 发布路径
        PublishPath(optPathPub,pathNav.optpath);

        if(pathNav.optpath.size() > 0)
        {
          getEndFlag = false;
        }
      }
      else
      {
        PublishPath(oriPathPub,pathNav.worldpath); // 发布路径
        PublishPath(optPathPub,pathNav.optpath);
      }
    }
  }
  
  // 使用Hybrid astar进行规划
  else if(control_method == 2)
  {
    // 开始进行规划
    if (mapInitFlag == true && getStartFlag == true )
    {
      if(getEndFlag == true)  //需要重新规划的时候
      {
        // 记录路径搜索需要的时间
        ros::Time time_1 = ros::Time::now();

        // 再次复位
        hybrid_astar_finder->reset();
        // 重新搜索
        int status = hybrid_astar_finder->search(startMapPoint,
                                      Eigen::Vector2d(0, 0),
                                      Eigen::Vector2d(0, 0),
                                      goalMapPoint, 
                                      Eigen::Vector2d(0, 0),true,false,-1.0);
        if (status == Hybrid_astar::NO_PATH)
        {
          cout << "[planner]: init search fail!" << endl;
          hybrid_astar_finder->reset();
          status = hybrid_astar_finder->search(startMapPoint, 
                                                Eigen::Vector2d(0, 0), 
                                                Eigen::Vector2d(0, 0), 
                                                goalMapPoint, 
                                                Eigen::Vector2d(0, 0), false, false,-1.0);
          if (status == Hybrid_astar::NO_PATH)
          {
            cout << "[planner]: Can't find path." << endl;
          }
          else
          {
            cout << "[planner]: retry search success." << endl;
          }
        }
        ros::Time time_2 = ros::Time::now(); 

        pathNav.worldpath = hybrid_astar_finder->getKinoTraj(0.02);
        cout<<"原始路径点数目:"<<tempath.size()<<endl;
        ROS_WARN("hybrid a star search time is:%f",(time_2-time_1).toSec() * 1000.0);

        PublishPath(optPathPub, pathNav.worldpath); // 发布路径
      }
      else
      {
        PublishPath(optPathPub, pathNav.worldpath); // 发布路径
      }      
    }
  }
  // 使用JPS进行规划
  else if(control_method == 3)
  {
    // 开始进行规划
    if (mapInitFlag == true && getStartFlag == true )
    {
      if(getEndFlag == true)  //需要重新规划的时候
      {
        getEndFlag = false;
        // 记录路径搜索需要的时间
        ros::Time time_1 = ros::Time::now();

        // 使用过的地图复位
        jps_path_finder->resetUsedGrids();
        // 开始路径搜索
        int status = jps_path_finder->JPSWorldSearch(startMapPoint,goalMapPoint);
        ros::Time time_2 = ros::Time::now();

        // 获取astar搜索到的路径
        pathNav.worldpath = jps_path_finder->getWorldPath();
        cout<<"原始路径点数目:"<<pathNav.worldpath.size()<<endl;
        ROS_WARN("jps search time is:%f",(time_2-time_1).toSec() * 1000.0);
        PublishPath(oriPathPub,pathNav.worldpath); // 发布路径
      }
      else
      {
        PublishPath(oriPathPub,pathNav.worldpath); // 发布路径
      }
    }
  }
  // 使用rrt进行规划
  else if(control_method == 4)
  {
    // 开始进行规划
    if (mapInitFlag == true && getStartFlag == true )
    {
      if(getEndFlag == true)  //需要重新规划的时候
      {
        getEndFlag = false;
        // 记录路径搜索需要的时间
        ros::Time time_1 = ros::Time::now();
        rrt_path_finder->FindPath(startMapPoint, goalMapPoint);
        ros::Time time_2 = ros::Time::now();

        // 获取astar搜索到的路径
        pathNav.worldpath = rrt_path_finder->path;
        cout<<"原始路径点数目:"<<pathNav.worldpath.size()<<endl;
        ROS_WARN("rrt star search time is:%f",(time_2-time_1).toSec() * 1000.0);
        PublishPath(oriPathPub,pathNav.worldpath); // 发布路径
      }
      else
      {
        PublishPath(oriPathPub,pathNav.worldpath); // 发布路径
      }
    }
  }  
}

