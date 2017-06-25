#include "lidar_tracking/motion_detecting.h"
#include <stdio.h>

double DisTwo(double a,double b,double c,double d)
{
  return sqrt(pow((a-c),2)+pow((b-d),2));
}

motion_detecting::motion_detecting(ros::NodeHandle n_)
{
   private_nh_=n_;
   initialized_=false;
   publish_closed_=true;
   publish_markers_=true;
   usePlast_=false;
   //save the flag of dynamic and analyze
   f_save_=fopen("/home/jc/ladir_ws/dynamic.txt","w+");
   count_=-1;
   window_size_=9;//4
   max_dis_=10;
   odom_frame_id_="odom";
   base_frame_id_="base_link";
   tf_ = new tf::TransformListener();
   SDEBUG("initialized");
   
   
   
   laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(private_nh_, "/scan", 100);
   laser_scan_filter_ = 
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_, 
                                                        *tf_, 
                                                        odom_frame_id_, 
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&motion_detecting::laserReceived,
                                                   this, _1));
  //only use the subscribe ,it will lead to a problam that cannot find the odom->base_link transform at laserScan->header.stamp
  //laserScan_sub_=private_nh_.subscribe("/scan",1,&motion_detecting::laserReceived,this);
  if(publish_closed_)
    closed_pub_=private_nh_.advertise<sensor_msgs::LaserScan>("closed_scan",100);
  
  if(publish_markers_)
    markers_pub_=private_nh_.advertise<visualization_msgs::Marker>("visualization_marker",20);
  
  
  m_t_ = new motion_tracking();
}

void motion_detecting::laserReceived(const sensor_msgs::LaserScanConstPtr& laserScan)
{
  clock_t startTime,endTime;
  startTime=clock();
  
  if(!initialized_)
  {
    //execuate only one time   ,
    //when we cannot get the transform between base_link and laser,it's false
    initialized_=initialized(laserScan);
  }
  
  //it can be used to filter out some scans with threshold
  if(++count_>100)
    count_=1;
  //SDEBUG("points_num_ %d count_ %d\n",points_num_,count_++);
  if(count_%2!=0)
    return;
  
  
  //change 0 to max for closed set, and get the current_frame_;
  for (int i=0;i<points_num_;i++)
  {
    current_frame_.scan[i]=laserScan->ranges[i];
    if(laserScan->ranges[i]<=0.001)
      current_frame_.scan[i]=max_dis_;
    //hear the odom->laser from tf tree
    
  }
  //get the time stamp of scan and save
  current_frame_.rTime=laserScan->header.stamp;
  //count_++;
  //SDEBUG("points_num_ %d count_ %d\n",points_num_,count_);
  //latest_odom_pose_ is odom -> base_frame_id_(base_link)
  if(!getPose(latest_odom_pose_, current_frame_.odom.v[0], current_frame_.odom.v[1], current_frame_.odom.v[2],
                  laserScan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }
  //PDEBUG("base_linke pose in odom frame %f   %f   %f\n",current_frame_.odom.v[0], current_frame_.odom.v[1], current_frame_.odom.v[2]);
  
  if(!getPose(latest_laser_pose_, current_frame_.odom.v[0], current_frame_.odom.v[1], current_frame_.odom.v[2],
                  laserScan->header.stamp, laserScan->header.frame_id))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }
  //PDEBUG("laser pose in odom frame %f   %f   %f\n",current_frame_.odom.v[0], current_frame_.odom.v[1], current_frame_.odom.v[2]);
  
  //initialize the program only this time count_ is 0;
  if(count_==0)
  {
    laser_stamp_=laserScan->header.stamp;// get the stamp
    closedFrame(current_frame_,closed_last_frame_,laser_stamp_);
    transformOdomClosedPoints();//after getting the points of closed form, 
    transformCurrentPoints();
    //give the ranges in odom frame save as last_ranges_;
    last_ranges_=current_ranges_;
    //last_ranges_->size;
  }
  else{
    /*
     * judge current from last  
     * */
    transformCurrentPoints();//get the new current_ranges_ in odom
    //using the current_ranges_ and closed_last_frame_.ranges detecting moving objects
    //check current scan data which of them are in the free area
    //checkOneToAll();
    //checkPoints();
    //flag=checkPoints(current_ranges_);
    flag=checkAllPoints(current_ranges_);
    //flag=checkOneToAll(current_ranges_);
    //PDEBUG("sumFlags  %d\n",sumFlags());
    obstacles_flag_=clusteringObstacles(current_ranges_);//clustering current scan into obstacles
    //PDEBUG("sumFlags  %d; obstacles %d\n",sumFlags(),int(obstacles_flag_.size()));
    // link sumFlags() and obstacles_flag_
    dynamic_flag_=linkClusterAndFlag(obstacles_flag_,flag);    
    //filtering out false positive ,and also need to grow the dynamic points    
    
    //PDEBUG("sumFlags  %d; obstacles %d  dynamic %d\n",sumFlags(),int(obstacles_flag_.size()),int(dynamic_flag_.size()));   
    /*
     * judge last from current at last we know "last_ranges_=current_ranges_;"
     * the goal is dynamic_last_flag_
     * */    
    laser_stamp_=laserScan->header.stamp;// get the stamp
    closedFrame(current_frame_,closed_last_frame_,laser_stamp_);
    //current frame after closing get closed_last_frame_,means updating the closed_last_frame_
    //now, we need judge last_ranges_ in closed_last_frame_; both in odom frame
    //in t-2 -> t, make closed(t) bigger;
    
    obstacles_last_flag_=clusteringObstacles(last_ranges_);
    for(size_t p=0;p<closed_last_frame_.points_num;p++)
      closed_last_frame_.scan[p]+=0.02;
    transformOdomClosedPoints();//after getting the points of closed form,   
    //obstacles_flag_ erase
    last_flag=checkOneToAll(last_ranges_);
    //last_flag=checkOneToAll(last_ranges_);
    bool * pLast_falg;
    if(usePlast_)
    {
       //obstacles_last_flag_=clusteringObstacles(last_ranges_);
       pLast_falg=new bool[points_num_];
       pLast_falg=checkOneToAll(pLast_ranges_);
       //last_flag=checkOneToAll(last_ranges_);
       //dynamic_last_flag_=linkClusterAndFlag(obstacles_last_flag_,last_flag);
       for(size_t kk=0;kk<points_num_;kk++)
       {
	 if(pLast_falg[kk]==true)
	   last_flag[kk]=true;
       }
    }
    
    dynamic_last_flag_=linkClusterAndFlag(obstacles_last_flag_,last_flag);
    PDEBUG("sumFlags  %d; clusters %d;  dynamic %d;  last %d\n",sumFlags(),int(obstacles_flag_.size()),int(dynamic_flag_.size()),int(dynamic_last_flag_.size()));
    //!!!modify the dynamic_flag_ with dynamic_last_flag_;
    
    publishMarkers(); 
    
    pLast_ranges_=last_ranges_;
    usePlast_=true;
    for(size_t p=0;p<closed_last_frame_.points_num;p++)
      closed_last_frame_.scan[p]-=0.025;
    transformOdomClosedPoints();
    last_ranges_=current_ranges_;
    obstacles_last_flag_.clear();
    obstacles_flag_.clear();
    dynamic_flag_.clear();
    dynamic_last_flag_.clear();
  }
  
  //clear the temp data in one iteration execuate
  clearTemp();
  
  endTime=clock();
  SDEBUG("total Time  :  %f \n", (double)(endTime-startTime)/CLOCKS_PER_SEC);
}

//corresponding to the current scans
void motion_detecting::publishMarkers()
{
  TwoINT temp;int i=1;
  double d_dis;
  if (publish_markers_)
  {
    for(obstacle_iter_=dynamic_flag_.begin();obstacle_iter_!=dynamic_flag_.end();obstacle_iter_++)
    {
      //sum=0;
      temp=*obstacle_iter_;
      features_=m_t_->calculateFeatures(temp,current_ranges_);
      
      d_dis=DisTwo(current_ranges_[temp.start][0],current_ranges_[temp.start][1],current_ranges_[temp.end][0],current_ranges_[temp.end][1]);
      //i=temp.start;
      //if(d_dis>0.15 && d_dis<0.3 || d_dis>0.25 && d_dis<0.65)
      if((d_dis<0.45) && m_t_->judgeFeatures(features_))
      {
        for(size_t k=temp.start;k<temp.end;k++,i++)
	{
	  visualization_msgs::Marker m;
	  m.header.stamp = current_frame_.rTime;
	  m.header.frame_id = odom_frame_id_;//odom
	  m.ns = "dynamic";
	  m.id = i;
	  m.type = m.SPHERE;
	  
	  m.pose.position.x = current_ranges_[k][0];// (*sf_iter)->position_[0];
	  m.pose.position.y = current_ranges_[k][1];//(*sf_iter)->position_[1];
	  m.pose.position.z = 0;//(*sf_iter)->position_[2];
	  m.scale.x = .15;
	  m.scale.y = .15;
	  m.scale.z = .15;
	  m.color.a = 1;
	  m.lifetime = ros::Duration(0.05);//0.05s
	  d_dis=(temp.end-temp.start+1)/(temp.end-temp.start+6);
	  m.color.r = 1;
	  m.color.b = 0;
	  markers_pub_.publish(m);
	}
      }
    }
    
    for(obstacle_iter_=dynamic_last_flag_.begin();obstacle_iter_!=dynamic_last_flag_.end();obstacle_iter_++,i++)
    {
      //sum=0;
      temp=*obstacle_iter_;
      features_=m_t_->calculateFeatures(temp,current_ranges_);
      //i=temp.start;
      d_dis=DisTwo(last_ranges_[temp.start][0],last_ranges_[temp.start][1],last_ranges_[temp.end][0],last_ranges_[temp.end][1]);
      //i=temp.start;
      //if(d_dis>0.15 && d_dis<0.3 || d_dis>0.25 && d_dis<0.45)
      if((d_dis< 0.45) && m_t_->judgeFeatures(features_))
      {
	for(size_t k=temp.start;k<temp.end;k++,i++)
	{
	  visualization_msgs::Marker m;
	  m.header.stamp = current_frame_.rTime;
	  m.header.frame_id = odom_frame_id_;//odom
	  m.ns = "dynamic";
	  m.id = i;
	  m.type = m.SPHERE;
	  m.pose.position.x = last_ranges_[k][0];// (*sf_iter)->position_[0];
	  m.pose.position.y = last_ranges_[k][1];//(*sf_iter)->position_[1];
	  m.pose.position.z = 0;//(*sf_iter)->position_[2];
	  m.scale.x = .15;
	  m.scale.y = .15;
	  m.scale.z = .15;
	  m.color.a = 1;
	  m.lifetime = ros::Duration(0.05);//0.05s
	  d_dis=(temp.end-temp.start+1)/(temp.end-temp.start+6);
	  m.color.r = 0;
	  m.color.b = 1;//d_dis;
	  markers_pub_.publish(m);
	}
      }
    } 
  }
}

void motion_detecting::clearTemp()
{
  for(int i=0;i<points_num_;i++)
  {
    flag[i]=false;
  }
}

//
std::vector<TwoINT> motion_detecting::linkClusterAndFlag(std::vector<TwoINT> & obstacles_f, bool * flag_f)
{
  //int i;
  //for every obstacle 
  std::vector<TwoINT> temp_v;
  TwoINT temp;//temp save the current obstacle
  int i,sum=0;
  for(obstacle_iter_=obstacles_f.begin();obstacle_iter_!=obstacles_f.end();obstacle_iter_++)
  {
    sum=0;
    temp=*obstacle_iter_;
    i=temp.start;
    while(i<=temp.end)
    {
      if(flag_f[i++]==true)
	sum++;
    }
    
    if(sum>=1)//0.4*(temp.end-temp.start+1))
    {
      temp_v.push_back(temp);   
      fprintf(f_save_,"%d  %d  %d \n", count_,temp.start,temp.end);
    }
  }
  return temp_v;
}

std::vector<TwoINT> motion_detecting::clusteringObstacles(double (* ranges)[2])
{
  double p1x,p1y,p2x,p2y;
  double distance,dis;
  std::vector<TwoINT> temp;
  one_obstacle_.start=0;
  int k=1;
  for(int i=1;i<points_num_;i++)
  {
    p1x=ranges[i-1][0];
    p1y=ranges[i-1][1];
    p2x=ranges[i][0];
    p2y=ranges[i][1];
    distance=5*sqrt(p2x*p2x+p2y*p2y)*(angle_increment_);
    dis=sqrt((p2x-p1x)*(p2x-p1x)+(p2y-p1y)*(p2y-p1y));
    if(dis>distance)
    {
      //mark this obstacle
      
      one_obstacle_.end=i-1;
      if(k>8)
      {
	temp.push_back(one_obstacle_);
      }      
      // next obstacle
      one_obstacle_.start=i;
      k=1;
    }
    
    //come to the end,judge >4 then it is a new obstacle
    if(i==points_num_-1 && dis<=distance)//logical
    {
      one_obstacle_.end=i-1;
      if(k>5)
      {
	temp.push_back(one_obstacle_);
      }      
    }
    k++;
  }
  return temp;
}

bool* motion_detecting::checkOneToAll(double (*ranges)[2])
{
  bool* f_flag;
  //must initialize the f_flag
  f_flag=new bool[points_num_];
  double px,py;
  for (int i=0;i<points_num_;i++)
  {
    //int ncross=0;
    px=ranges[i][0];
    py=ranges[i][1];
    f_flag[i]=checkOnePointInPolygon(px,py);
  }
  //sample for filter out  '010'
  for(int i=1;i<points_num_-1;i++)
  {
    if(f_flag[i]==true)
    {
      f_flag[i]= f_flag[i-1] || f_flag[i+1];
      
    }
  }
  return f_flag;
}

//judge (x,y) in polygon(closed_last_frame_)
bool motion_detecting::checkOnePointInPolygon(double x,double y)
{
  bool f=false;
  int j=points_num_;
  for(int i=0;i<=closed_last_frame_.points_num;i++)
  {
    if((closed_last_frame_.ranges[i][1]<y && closed_last_frame_.ranges[j][1]>=y
         ||   closed_last_frame_.ranges[j][1]<y && closed_last_frame_.ranges[i][1]>=y)
      && (closed_last_frame_.ranges[i][0]<=x || closed_last_frame_.ranges[j][0]<=x))
    {
      f^=(closed_last_frame_.ranges[i][0]+(y-closed_last_frame_.ranges[i][1])/(closed_last_frame_.ranges[j][1]-closed_last_frame_.ranges[i][1])*(closed_last_frame_.ranges[j][1]-closed_last_frame_.ranges[i][1])<x);
      
    }
    j=i;
  }
  return f;
}

//sum all flags which is true 
int motion_detecting::sumFlags()
{
  int sum=0;
  for(int i=points_num_/4;i<3*points_num_/4;i++)
    if(flag[i]==true)
      sum++;
  return sum;
}

bool* motion_detecting::checkAllPoints(double (* ranges)[2])
{
  //for all points in ranges,check whether it is in the static area or not
  int k=0;
  bool * f_flag;
  f_flag = new bool[points_num_];
  //learning from motionDetecting.cpp thanks a lot
  double px,py,p1x,p1y,p2x,p2y;
  for (int i=0;i<points_num_;i++)
  {
    int ncross=0;
    px=ranges[i][0];
    py=ranges[i][1];
    
    for(int j=0;j<=points_num_;j++)
    {
      //the first point should be the laser pose in odom and ,the final also
      p1x=closed_last_frame_.ranges[j][0];
      p1y=closed_last_frame_.ranges[j][1];
      p2x=closed_last_frame_.ranges[(j+1)%(points_num_+1)][0];
      p2y=closed_last_frame_.ranges[(j+1)%(points_num_+1)][1];
      if((p1x<px)&&(p2x<px))
      {
	continue;
      }
	double min,max;
	if(p1y<p2y)
	{
		min=p1y;
		max=p2y;
	}
	else
	{
		min=p2y;
		max=p1y;
	}
	if(p1y==p2y)
		continue;
        if(py<min||py>max)
		continue;
	if(p2y==py&&px<p2x)
	{
		ncross++;
		continue;
	}
			
	double x1;
	if(p1x==p2x)
	{
	  if(px<p1x)//
	  {
		  ncross++;
		  continue;
	  }
	}
	else
	{
		x1=(py-p1y)*(p2x-p1x)/(p2y-p1y)+p1x;
		if(x1>px+0.01)
		{
			ncross++;
			//file<<"x1="<<x1<<"/px"<<px<<endl;
			continue;
		}
	} 
	}
		//if(ncross%2==1)file<<i<<"/"<<ncross<<endl;
	if(ncross%2==1)
		f_flag[k++]=1;
	else
	  f_flag[k++]=0;
    
  }
  //sample filter out liers
  /*
  for(int i=1;i<points_num_-1;i++)
  {
    if(f_flag[i]==true)
    {
      f_flag[i]= f_flag[i-1] || f_flag[i+1];
      
    }
  }
  */
  return f_flag;
}

//check points whether it is avoiding the static assmpution or not;
//the result is flag[], it is the initialize result of detecting
//after this ,we need gather all flag[] and tracking objects
bool * motion_detecting::checkPoints(double (*ranges)[2])
{
  //for all points in ranges,check whether it is in the static area or not
  int k=0;
  bool * tempflag;
  tempflag= new bool[points_num_];
  //learning from motionDetecting.cpp thanks a lot
  double px,py,p1x,p1y,p2x,p2y;
  for (int i=0;i<points_num_;i++)
  {
    int ncross=0;
    px=ranges[i][0];
    py=ranges[i][1];
    
    for(int j=0;j<=points_num_;j++)
    {
      //the first point should be the laser pose in odom and ,the final also
      p1x=closed_last_frame_.ranges[j][0];
      p1y=closed_last_frame_.ranges[j][1];
      p2x=closed_last_frame_.ranges[(j+1)%(points_num_+1)][0];
      p2y=closed_last_frame_.ranges[(j+1)%(points_num_+1)][1];
      if((p1x<px)&&(p2x<px))
      {
	continue;
      }
	double min,max;
	if(p1y<p2y)
	{
		min=p1y;
		max=p2y;
	}
	else
	{
		min=p2y;
		max=p1y;
	}
	if(p1y==p2y)
		continue;
        if(py<min||py>max)
		continue;
	if(p2y==py&&px<p2x)
	{
		ncross++;
		continue;
	}
			
	double x1;
	if(p1x==p2x)
	{
	  if(px<p1x)//
	  {
		  ncross++;
		  continue;
	  }
	}
	else
	{
		x1=(py-p1y)*(p2x-p1x)/(p2y-p1y)+p1x;
		if(x1>px+0.01)
		{
			ncross++;
			//file<<"x1="<<x1<<"/px"<<px<<endl;
			continue;
		}
	} 
	}
		//if(ncross%2==1)file<<i<<"/"<<ncross<<endl;
	if(ncross%2==1)
		tempflag[k++]=1;
	else
	  tempflag[k++]=0;
    
  }
  return tempflag;
  //sample filter out liers
  /*
  for(int i=1;i<points_num_-1;i++)
  {
    if(flag[i]==true)
    {
      flag[i]= flag[i-1] || flag[i+1];
      
    }
  }
  */
}

//transform cuurent laserScan into current_ranges_[][] in odom frame
void motion_detecting::transformCurrentPoints()
{
  for (int i=0;i<points_num_;i++)
  {
    current_ranges_[i][0]=current_frame_.scan[i]*cos(min_angle_+i*angle_increment_+current_frame_.odom.v[2])+current_frame_.odom.v[0];
    
    //before 01/10,there is a bug  current_frame_.odom.v[0] ===>>>current_frame_.odom.v[1]
    current_ranges_[i][1]=current_frame_.scan[i]*sin(min_angle_+i*angle_increment_+current_frame_.odom.v[2])+current_frame_.odom.v[1];
  }
}


//transform the closed points into cartessian of odom
void motion_detecting:: transformOdomClosedPoints()
{
  for(int i=0;i<closed_last_frame_.points_num;i++)
  {
    closed_last_frame_.ranges[i][0]=closed_last_frame_.scan[i]*cos(min_angle_+i*angle_increment_+closed_last_frame_.odom.v[2])+closed_last_frame_.odom.v[0];//x
    closed_last_frame_.ranges[i][1]=closed_last_frame_.scan[i]*sin(min_angle_+i*angle_increment_+closed_last_frame_.odom.v[2])+closed_last_frame_.odom.v[1];//y
  }
  closed_last_frame_.ranges[closed_last_frame_.points_num][0]=closed_last_frame_.odom.v[0];
  closed_last_frame_.ranges[closed_last_frame_.points_num][1]=closed_last_frame_.odom.v[1];
}

//set the LRF parameters at the start time
bool motion_detecting::initialized(const sensor_msgs::LaserScanConstPtr& laserScan)
{
  points_num_=laserScan->ranges.size();
  angle_increment_=laserScan->angle_increment;
  min_angle_=laserScan->angle_min;
  max_angle_=laserScan->angle_max;
  time_increment_=laserScan->time_increment;
  scan_time_=laserScan->scan_time;
  laser_frame_id_=laserScan->header.frame_id;
  //laserScan->header.stamp;
  SDEBUG("parameters : time_increment  %f, points_num  %d\n",time_increment_,points_num_);
  
  current_ranges_=new double[points_num_][2];
  last_ranges_=new double[points_num_][2];
  pLast_ranges_=new double[points_num_][2];
  
  flag=new bool[points_num_];
  last_flag=new bool[points_num_];
  
  current_frame_.points_num=points_num_;
  current_frame_.angle_increment=angle_increment_;
  current_frame_.scan=new double[current_frame_.points_num];
  current_frame_.ranges=new double[current_frame_.points_num][2];
  
  closed_last_frame_.points_num=points_num_;//the one is for the laser pose in odom
  closed_last_frame_.angle_increment=angle_increment_;
  closed_last_frame_.scan=new double[closed_last_frame_.points_num];//the scan is the same as laserScan
  closed_last_frame_.ranges=new double[closed_last_frame_.points_num+1][2];
  
  closed_current_frame_.points_num=points_num_;//the one is for the laser pose in odom
  closed_current_frame_.angle_increment=angle_increment_;
  closed_current_frame_.scan=new double[closed_current_frame_.points_num];//the scan is the same as laserScan
  closed_current_frame_.ranges=new double[closed_current_frame_.points_num+1][2];
  
  //get the base_link->laser transform matrix
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), laserScan->header.frame_id);
   
   try
   {
     this->tf_->transformPose(base_frame_id_, ident, laser_pose_);
     PDEBUG("laser_pose%f  %f\n",laser_pose_.getOrigin().x(),laser_pose_.getOrigin().y());
   }
   
   catch(tf::TransformException& e)
   {
     ROS_ERROR("Couldn't transform from %s to %s, "
               "even though the message notifier is in use",
               laserScan->header.frame_id.c_str(),
               base_frame_id_.c_str());
     return false;
   }
  return true;
}

//solve the scan by close set operator and then judge whether publish it or not 
bool motion_detecting::closedFrame(d_frame & open_frame, d_frame & closed_frame,ros::Time laser_stamp)
{
  double min=0;
  int step=(window_size_-1)/2; // when window_size_ is 11 ,step is 5;
  closed_frame.rTime=open_frame.rTime;
  //1->1077
  for(int i=step;i<open_frame.points_num-step;i++)
  {
    min=open_frame.scan[i];
    for(int j=-1*step;j<=step;j++)
    {
      if(min>open_frame.scan[i+j])
	 min=open_frame.scan[i+j];
    }
    closed_frame.scan[i]=min;
  }
  //0-> step-1;
  int k=0;
  while(k<step)
  {
    closed_frame.scan[k]=closed_frame.scan[step];
    k++;
  }
  //open_frame.points_num-step  ->  open_frame.points_num-1
  k=open_frame.points_num-step;
  while(k<open_frame.points_num)
  {
    closed_frame.scan[k]=closed_frame.scan[open_frame.points_num-step-1];
    k++;
  }
  
  closed_frame.odom=open_frame.odom;// give the odom -> laser pose to the closed form
  /*min=open_frame.scan[0];
  if(min>open_frame.scan[1])
    min=open_frame.scan[1];
  if(min>open_frame.scan[2])
    min=open_frame.scan[2];
  closed_frame.scan[0]=min;*/
  
  //1078&1079 the same
  /*min=open_frame.scan[open_frame.points_num-window_size_+1];
  if(min>open_frame.scan[open_frame.points_num-window_size_+2])
    min=open_frame.scan[open_frame.points_num-window_size_+1];
  if(min>open_frame.scan[open_frame.points_num-window_size_+3])
    min=open_frame.scan[open_frame.points_num-window_size_+3];
  closed_frame.scan[open_frame.points_num-window_size_+2]=min;
  closed_frame.scan[open_frame.points_num-window_size_+3]=min;*/
  for(size_t p=0;p<closed_frame.points_num;p++)
    closed_frame.scan[p]-=0.01;
  
  //publish the closed_last_frame_ using LaserScan data type
  if(publish_closed_)
  {
    int ranges=closed_frame.points_num;
    sensor_msgs::LaserScan closed_scan;
    //closed_scan.header.stamp= laser_stamp;
    closed_scan.header.stamp= closed_frame.rTime;//ros::Time::now();
    closed_scan.header.frame_id=laser_frame_id_;
    closed_scan.angle_increment=closed_frame.angle_increment;
    closed_scan.angle_max=max_angle_;
    closed_scan.angle_min=min_angle_;
    closed_scan.scan_time=scan_time_;
    closed_scan.time_increment=time_increment_;
    closed_scan.ranges.resize(ranges);
    closed_scan.range_max=30.0;
    closed_scan.range_min=0.02;
    for(int ii=0;ii<ranges;ii++)
    {
      closed_scan.ranges[ii]=closed_frame.scan[ii];
    }
    closed_pub_.publish(closed_scan);
  }
  return true;
}

//get the transform between odom and f(parameter)
bool motion_detecting::getPose(tf::Stamped<tf::Pose>& pose,double& x ,double& y, double& yaw,const ros::Time& t,const std::string& f)
{
  //tf::Stamped<tf::Pose> odom_pose;
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_->transformPose(odom_frame_id_, ident, pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = pose.getOrigin().x();
  y = pose.getOrigin().y();
  double pitch,roll;
  pose.getBasis().getEulerYPR(yaw, pitch, roll);
  return true;
}


motion_detecting::~motion_detecting()
{
  delete[] flag;
  delete[] current_ranges_; 
  
  /*
   * if(temp_obs){
	for(int k=0; k < max_samples; k++){
	  delete [] temp_obs[k];
	}
	delete []temp_obs; 
     }
   * */
}
