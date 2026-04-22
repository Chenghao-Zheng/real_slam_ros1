#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h> // 必须包含 TF 监听器头文件

#include <robot_communication/localizationInfoBroadcast.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

//把角度差归一化到 [-pi, pi]，避免“359° - 1° = 358°”这种跳变
static inline double wrap_to_pi(double a)
{
  while (a > M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

//把值夹在范围内，速度限幅用
static inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

struct PID
{
  double kp{0}, ki{0}, kd{0};
  double i{0}, prev_e{0};  // 上一次误差
  double i_limit{1.0};     // 积分限幅

  //给误差 e、时间间隔 dt，输出控制量u: kp*e + ki*i + kd*de
  double step(double e, double dt)
  {
    if (dt <= 1e-6) return kp*e;

    // I
    i += e*dt;
    i = clamp(i, -i_limit, i_limit);

    // D
    const double de = (e - prev_e)/dt;
    prev_e = e;

    return kp*e + ki*i + kd*de;
  }

  // 把积分和 prev_e 清零
  void reset()
  {
    i = 0;
    prev_e = 0;
  }
};

class OmnidirectionalPIDLocalPlanner
{
public:
  OmnidirectionalPIDLocalPlanner()
  : nh_(), pnh_("~")
  {
    nh_.param("IS_SIM", is_sim_, false);

    nh_.param("LOOKAHEAD_DIST", lookahead_dist_, 0.5);  //前瞻距离
    nh_.param("GOAL_TOL", goal_tol_, 0.25);

    nh_.param("MAX_VX", max_vx_, 0.4);
    nh_.param("MAX_VY", max_vy_, 0.4);
    nh_.param("MAX_WZ", max_wz_, 0.8);

    nh_.param("KP_X", pid_x_.kp, 0.9);
    nh_.param("KI_X", pid_x_.ki, 0.0);
    nh_.param("KD_X", pid_x_.kd, 0.0);

    nh_.param("KP_Y", pid_y_.kp, 0.9);
    nh_.param("KI_Y", pid_y_.ki, 0.0);
    nh_.param("KD_Y", pid_y_.kd, 0.0);

    nh_.param("KP_YAW", pid_yaw_.kp, 1.2);
    nh_.param("KI_YAW", pid_yaw_.ki, 0.0);
    nh_.param("KD_YAW", pid_yaw_.kd, 0.0);

    nh_.param("I_LIMIT_X", pid_x_.i_limit, 0.6);
    nh_.param("I_LIMIT_Y", pid_y_.i_limit, 0.6);
    nh_.param("I_LIMIT_YAW", pid_yaw_.i_limit, 1.0);

    nh_.param("CMD_TIMEOUT", cmd_timeout_, 0.5);
    nh_.param("PUB_HZ", pub_hz_, 30.0);

    nh_.param("IDLE_LATCH_ON_REACHED", idle_latch_on_reached_, true);
    nh_.param("PUBLISH_ZERO_WHEN_IDLE", publish_zero_when_idle_, true);
    nh_.param("STOP_V_TOL", stop_v_tol_, 0.03);
    nh_.param("STOP_W_TOL", stop_w_tol_, 0.05);
    nh_.param("USE_SPEED_CHECK", use_speed_check_, true);
    nh_.param("NEW_PATH_EPS", new_path_eps_, 0.02);

    path_sub_ = nh_.subscribe("/opt_path", 1, &OmnidirectionalPIDLocalPlanner::pathCb, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OmnidirectionalPIDLocalPlanner::goalCb, this);

    if (is_sim_)
    {
      odom_sim_sub_ = nh_.subscribe("/truth_pose_odom", 1, &OmnidirectionalPIDLocalPlanner::simOdomCb, this);
    }
    else
    {
      odom_carto_sub_ = nh_.subscribe("/carto_odom", 1, &OmnidirectionalPIDLocalPlanner::odomCb, this);
    }

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_auto", 1);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_goal", 1, true);

    last_cmd_time_ = ros::Time(0);

    timer_ = nh_.createTimer(ros::Duration(1.0/std::max(1.0, pub_hz_)),
                             &OmnidirectionalPIDLocalPlanner::onTimer, this);

    ROS_INFO("[robot_pid_local_planner] started. IS_SIM=%s, lookahead=%.2f, pub_hz=%.1f",
             is_sim_ ? "true":"false", lookahead_dist_, pub_hz_);
  }

private:
  void pathCb(const nav_msgs::PathConstPtr& msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();

    bool new_path = false;
    const size_t sz = path_.poses.size();

    if (sz != last_path_size_) {
      if (std::abs((int)sz - (int)last_path_size_) >= 2) new_path = true;
    }

    if (sz > 0) {
      // 1. 检查末端点变化
      const auto& last = path_.poses.back().pose.position;
      const double dlast = std::hypot(last.x - last_path_last_x_, last.y - last_path_last_y_);
      if (dlast > new_path_eps_) new_path = true;

      // 2. 检查起始点变化（重定位后新路径起点必变）
      const auto& first = path_.poses.front().pose.position;
      const double dfirst = std::hypot(first.x - last_path_first_x_, first.y - last_path_first_y_);
      if (dfirst > new_path_eps_) new_path = true;

      // 更新记录
      last_path_last_x_ = last.x;
      last_path_last_y_ = last.y;
      last_path_first_x_ = first.x;
      last_path_first_y_ = first.y;
    }
    last_path_size_ = sz;

    if (has_path_ && new_path) {
      reached_latched_ = false;
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      ROS_INFO("[robot_pid_local_planner] new path detected -> exit IDLE");
    }
  }

  void goalCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    goal_ = *msg;
    has_goal_ = true;
    reached_latched_ = false;
    pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
  }

  void simOdomCb(const robot_communication::localizationInfoBroadcastConstPtr& msg)
  {
    // 仿真模式下，直接获取位置（不能删）
    x_ = msg->xPosition;
    y_ = msg->yPosition;
    yaw_ = msg->chassisAngle;

    vx_fb_ = msg->xSpeed;
    vy_fb_ = msg->ySpeed;
    wz_fb_ = msg->chassisGyro;

    has_odom_ = true;
  }

  void odomCb(const nav_msgs::OdometryConstPtr& msg)
  {
    // 实车模式下，只从 odom 获取速度
    vx_fb_ = msg->twist.twist.linear.x;
    vy_fb_ = msg->twist.twist.linear.y;
    wz_fb_ = msg->twist.twist.angular.z;

    has_odom_ = true;
  }

  bool computeLookaheadPoint(double& gx, double& gy, double& g_yaw_target)
  {
    if (!has_path_ || path_.poses.empty()) return false;

    int nearest = 0;
    double best_d2 = 1e100;
    for (int i = 0; i < (int)path_.poses.size(); ++i)
    {
      const double px = path_.poses[i].pose.position.x;
      const double py = path_.poses[i].pose.position.y;
      const double dx = px - x_;
      const double dy = py - y_;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; nearest = i; }
    }

    double acc = 0.0;
    int idx = nearest;
    for (int i = nearest; i+1 < (int)path_.poses.size(); ++i)
    {
      const double x0 = path_.poses[i].pose.position.x;
      const double y0 = path_.poses[i].pose.position.y;
      const double x1 = path_.poses[i+1].pose.position.x;
      const double y1 = path_.poses[i+1].pose.position.y;
      const double seg = std::hypot(x1-x0, y1-y0);
      acc += seg;
      if (acc >= lookahead_dist_) { idx = i+1; break; }
      idx = i+1;
    }

    gx = path_.poses[idx].pose.position.x;
    gy = path_.poses[idx].pose.position.y;

    g_yaw_target = std::atan2(gy - y_, gx - x_);

    nav_msgs::Path local;
    local.header = path_.header;
    for (int i = nearest; i <= idx; ++i) local.poses.push_back(path_.poses[i]);
    local_path_pub_.publish(local);

    visualization_msgs::Marker mk;
    mk.header = path_.header;
    mk.ns = "pid_local_goal";
    mk.id = 1;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = gx;
    mk.pose.position.y = gy;
    mk.pose.position.z = 0.1;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.18;
    mk.scale.y = 0.18;
    mk.scale.z = 0.18;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.8;
    mk.color.b = 0.0;
    marker_pub_.publish(mk);

    return true;
  }

  bool isReachedGoal() const
  {
    if (!has_path_ || path_.poses.empty()) return false;
    const auto& last = path_.poses.back().pose.position;
    const double dist_goal = std::hypot(last.x - x_, last.y - y_);
    if (dist_goal > goal_tol_) return false;

    if (use_speed_check_)
    {
      const double v = std::hypot(vx_fb_, vy_fb_);
      if (v > stop_v_tol_) return false;
      if (std::fabs(wz_fb_) > stop_w_tol_) return false;
    }

    return true;
  }

  void publishZero()
  {
    geometry_msgs::Twist z;
    cmd_pub_.publish(z);
  }

  void onTimer(const ros::TimerEvent& ev)
  {
    if (!has_odom_)
    {
      publishZero();
      return;
    }

    // ========= 【核心修复】通过 TF 获取小车在路径坐标系下的真实位姿 =========
    if (!is_sim_ && has_path_) 
    {
      tf::StampedTransform transform;
      try {
        // 查询 base_link 在路径所在坐标系（通常是 map）下的坐标
        tf_listener_.lookupTransform(path_.header.frame_id, "base_link", ros::Time(0), transform);
        x_ = transform.getOrigin().x();
        y_ = transform.getOrigin().y();
        yaw_ = tf::getYaw(transform.getRotation());
      } catch (tf::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "[robot_pid_local_planner] TF 获取失败等待中: %s", ex.what());
        publishZero();
        return;
      }
    }

    if (idle_latch_on_reached_ && reached_latched_)
    {
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      if (publish_zero_when_idle_) publishZero();
      return;
    }

    if (idle_latch_on_reached_ && isReachedGoal())
    {
      reached_latched_ = true;
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      publishZero();
      ROS_INFO_THROTTLE(1.0, "[robot_pid_local_planner] reached goal -> IDLE (latched). manual can take over.");
      return;
    }

    double gx, gy, yaw_target;
    if (!computeLookaheadPoint(gx, gy, yaw_target))
    {
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      publishZero();
      return;
    }

    const double dt = (last_cmd_time_.isZero()) ? (1.0/std::max(1.0, pub_hz_))
                                                : (ros::Time::now() - last_cmd_time_).toSec();
    last_cmd_time_ = ros::Time::now();

    const double dx = gx - x_;
    const double dy = gy - y_;

    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);
    const double ex =  cy*dx + sy*dy;
    const double ey = -sy*dx + cy*dy;

    const double e_yaw = wrap_to_pi(yaw_target - yaw_);

    double vx = pid_x_.step(ex, dt);
    double vy = pid_y_.step(ey, dt);
    double wz = pid_yaw_.step(e_yaw, dt);

    vx = clamp(vx, -max_vx_, max_vx_);
    vy = clamp(vy, -max_vy_, max_vy_);
    wz = clamp(wz, -max_wz_, max_wz_);

    geometry_msgs::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;

    cmd_pub_.publish(cmd);

    if (idle_latch_on_reached_ && isReachedGoal())
    {
      reached_latched_ = true;
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      publishZero();

      const auto& last = path_.poses.back().pose.position;
      const double dist_goal = std::hypot(last.x - x_, last.y - y_);
      ROS_WARN_THROTTLE(1.0, "[robot_pid_local_planner] REACHED & LATCHED IDLE. dist=%.3f", dist_goal);
      return;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sim_sub_;
  ros::Subscriber odom_carto_sub_;

  ros::Publisher cmd_pub_;
  ros::Publisher local_path_pub_;
  ros::Publisher marker_pub_;

  ros::Timer timer_;
  ros::Time last_cmd_time_;

  nav_msgs::Path path_;
  geometry_msgs::PoseStamped goal_;

  // ==========================
  // 【修复点】：放在类的 private 成员里
  // ==========================
  tf::TransformListener tf_listener_;
  double last_path_first_x_{0.0};
  double last_path_first_y_{0.0};
  // ==========================

  bool is_sim_{true};
  bool has_path_{false};
  bool has_goal_{false};
  bool has_odom_{false};

  bool idle_latch_on_reached_{true};
  bool publish_zero_when_idle_{true};
  bool reached_latched_{false};
  bool use_speed_check_{true};
  double stop_v_tol_{0.03};
  double stop_w_tol_{0.05};
  double new_path_eps_{0.02};
  ros::Time last_path_stamp_{0};
  size_t last_path_size_{0};
  double last_path_last_x_{0.0}, last_path_last_y_{0.0};

  double lookahead_dist_{0.6};
  double goal_tol_{0.25};
  double max_vx_{0.4}, max_vy_{0.4}, max_wz_{0.8};
  double cmd_timeout_{0.5};
  double pub_hz_{30.0};

  double x_{0}, y_{0}, yaw_{0};
  double vx_fb_{0}, vy_fb_{0}, wz_fb_{0};

  PID pid_x_, pid_y_, pid_yaw_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omnidirectional_pid_local_planner");
  OmnidirectionalPIDLocalPlanner node;
  ros::spin();
  return 0;
}