### 仿真环境下的导航重定位
#!/usr/bin/env bash
source ~/.bashrc
set -e
# 1) 启动 roscore
gnome-terminal --title="roscore" -- bash -c "roscore"
# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done
# 2) 启动 Gazebo + robot
gnome-terminal --title="gazebo_start" -- bash -c "source ~/RealSlamRos1/devel/setup.bash; roslaunch tri_steer_gazebo sim_gazebo_rviz.launch"
# 3) 雷达点云反转成水平车体下的point-cloud2 暂时无用
gnome-terminal -t "laser to cloud" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch read_laser_data laser_to_cloud.launch; exec bash"
# 等待 Gazebo 节点起来（/gazebo/model_states 出现再继续）
until rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; do
  sleep 0.5
done
# 4）算法重定位
gnome-terminal -t "carto_relocalize" -- bash -lc "cd /home/ubuntu/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash; roslaunch cartographer_ros my_robot_2d_localization.launch; exec bash"
# 等待 Cartographer 发布 /map（或你也可以等 TF map->odom）
until rostopic list 2>/dev/null | grep -q "^/map$"; do
  sleep 0.5
done
# 5）启动地图节点
gnome-terminal -t "map start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_costmap map_deal.launch;exec bash"
# 6) 运动规划节点
gnome-terminal -t "motion Plan start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_navigation motionPlan_sim.launch"
# 7）局部规划pid算法
gnome-terminal -t "PID_control" -- bash -c "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_pid_local_planner Omnidirectional_PID.launch; exec bash"
# 解析carto算法发布的tf变换到机器人
gnome-terminal -t "carto_odom_start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch carto_tf_to_odom carto_tf_to_odom_sim.launch; exec bash"
# 9) 启动并发布机器人真值里程计消息
gnome-terminal -t "truth Odom start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch robot_locatization truth_odometry.launch;exec bash"
# 10）订阅控制消息
gnome-terminal -t "control start" -- bash -lc "source ~/RealSlamRos1/devel/setup.bash; roslaunch tri_steer_drive bringup.launch use_teleop:=true;exec bash"