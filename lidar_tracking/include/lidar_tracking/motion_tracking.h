#ifndef MOTION_TRACKING_H
#define MOTION_TRACKING_H

//#include "ladir_tracking/motion_detecting.h"
#include "lidar_tracking/com_def.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"

/// using for tacking dynamic obstacles 
/// when detect dynamic at time continues time, link them
/// when miss detecting, find the nearest and the most likely one
class motion_tracking
{
public:
  motion_tracking();
  ~motion_tracking();
  
  /// caculate some features about given obstacle
  std::vector<float> calculateFeatures(TwoINT& obstacle, double (* ranges)[2]);
  
  bool judgeFeatures(std::vector<float> & features);
};

#endif
