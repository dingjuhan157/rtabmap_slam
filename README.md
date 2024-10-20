手动建图
roslaunch robot_ros map.launch

roslaunch robot_ros stm32_control.launch

导航
roslaunch robot_ros nav.launch

stm32 control
roslaunch robot_ros stm32_control.launch

roslaunch realsense2_camera rs_camera.launch

realsense-viewer

rtabmap-databaseViewer ~/.ros/rtabmap.db

查看节点与话题关系 rqt_graph

生成TF树pdf rosrun tf view_frames 查看TF树 rosrun rqt_tf_tree rqt_tf_tree

rostopic echo

rosbag record -o 3.bag /camera/rgb/image_rect_color /camera/depth_registered/image_raw /camera/rgb/camera_info

roslaunch realsense2_camera rs_camera.launch align_depth:=true depth_width:=640 depth_height:=480 depth_fps:=30 color_width:=640 color_height:=480 color_fps:=30

roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/camera/aligned_depth_to_color/image_raw rgb_topic:=/camera/color/image_raw camera_info_topic:=/camera/color/camera_info approx_sync:=true

rosrun rviz rviz -d $(rospack find rtabmap_ros)/launch/config/rgbd.rviz
