#ifndef MOTION_DETECTING_H
#define MOTION_DETECTING_H

#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <time.h>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"
#include <visualization_msgs/Marker.h>

#include "lidar_tracking/com_def.h"
#include "lidar_tracking/motion_tracking.h"



double DisTwo(double a,double b,double c,double d);


/*激光运动检测，找到违反静态的点
 * */
class motion_detecting
{
  
public:
  motion_detecting(ros::NodeHandle n_);
  ~motion_detecting();
  void laserReceived(const sensor_msgs::LaserScanConstPtr& laserScan);
  bool initialized(const sensor_msgs::LaserScanConstPtr& laserScan);//initialize the parameters of LRF
  
  bool closedFrame(d_frame & open_frame, d_frame & closed_frame,ros::Time laser_stamp);//close set operator
  bool getPose(tf::Stamped<tf::Pose>& pose,double& x ,double& y, double& yaw,const ros::Time& t,const std::string& f);//get the transform between odom and f(parameter)
  void transformOdomClosedPoints();//after detecting, transform points into closed form
  void transformCurrentPoints();//before detecting, transform points into odom frame
  
  //check points whether it is avoiding the static assmpution or not;the result is flag[]
  bool* checkPoints(double (*ranges)[2]);
  bool* checkAllPoints(double (* ranges)[2]);
  //check the point of laser scan , in this polygon
  bool checkOnePointInPolygon(double x,double y);
  bool* checkOneToAll(double (* ranges)[2]);
  int sumFlags();
  
  //for clustering points into obstacles and judge this obstacle is dynamic?
  std::vector<TwoINT> clusteringObstacles(double (* ranges)[2]);//for get obstacles_flag_;
  
  //link the clusters result and flag, filter out flase
  std::vector<TwoINT> linkClusterAndFlag(std::vector<TwoINT> & obstacles_f, bool * flag_f);
  
  //clear the temp data when claculating
  void clearTemp();
  
  void publishMarkers();
  
private:
  
  ros::NodeHandle private_nh_;
  bool visualize_;//how to visualize the result of detecing and or not
  bool initialized_;
  bool publish_closed_;//publish the close set of ladir data, corresponding to closed_last_frame_
  bool publish_markers_;
  ros::Publisher markers_pub_;
  //ros::Stamped
  tf::Stamped<TwoINT> two_int_;
  //last LaserScan
  float * last_scan_;
  
  int points_num_;//num of laser points 1080
  
  ros::Publisher closed_pub_;
  ros::Subscriber laserScan_sub_;
  //ros::
  message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
  int queue_num_;//需求的激光点队列数
  float min_angle_;
  float max_angle_;
  float angle_increment_;
  float time_increment_;
  float scan_time_;
  float max_dis_;
  std::string laser_frame_id_;
  std::string base_frame_id_;
  tf::Stamped<tf::Pose> laser_pose_;//the transform between base_link to laser
  tf::Stamped<tf::Pose> latest_odom_pose_;//saving last time the odom pose in odom frame
  tf::Stamped<tf::Pose> latest_laser_pose_;//saving last time the laser pose in odom frame
  ros::Time laser_stamp_;
  int count_;
  //last frame with scan and odom ,&& after close set the frame is closed_last_frame_;
  int window_size_;
  d_frame current_frame_;
  d_frame closed_last_frame_;
  d_frame closed_current_frame_;
  double (*current_ranges_)[2];//for saving the cuurent scan in odom frame
  double (*last_ranges_)[2];
  double (*pLast_ranges_)[2];
  bool usePlast_;
  
  //clustering obstacles
  std::vector<TwoINT> obstacles_flag_;
  std::vector<TwoINT> obstacles_last_flag_;
  std::vector<TwoINT> dynamic_flag_;
  std::vector<TwoINT> dynamic_last_flag_;
  std::vector<TwoINT>::iterator obstacle_iter_;
  TwoINT one_obstacle_;
  
  bool *flag;
  bool *last_flag;
  
  FILE * f_save_;
  std::string odom_frame_id_;
  tf::TransformListener* tf_;
  
  std::vector<float> features_;
  motion_tracking *m_t_;
};
#endif
