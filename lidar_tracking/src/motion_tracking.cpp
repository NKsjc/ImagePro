#include "lidar_tracking/motion_tracking.h"

motion_tracking::motion_tracking()
{
}

motion_tracking::~motion_tracking()
{
}

std::vector<float> motion_tracking::calculateFeatures(TwoINT & obstacle, double (*ranges)[2])
{
  std::vector<float> features;
  
  /// compute linearity
  int num_points=obstacle.end-obstacle.start+1;
  float x_mean=0.0,y_mean=0.0;
  
  for(size_t ii=obstacle.start;ii<=obstacle.end;ii++)
  {
    x_mean += ranges[ii][0]/num_points;
    y_mean += ranges[ii][1]/num_points;
  }
  CvMat* points =cvCreateMat(num_points,2,CV_64FC1);
  {
    int j=0;
    for(size_t i=obstacle.start;i<=obstacle.end;i++)
    {
      cvmSet(points,j,0,ranges[i][0]-x_mean);
      cvmSet(points,j,1,ranges[i][1]-y_mean);
      j++;
    }
  }
  CvMat* W=cvCreateMat(2,2,CV_64FC1);
  CvMat* U = cvCreateMat(num_points, 2, CV_64FC1);
  CvMat* V = cvCreateMat(2, 2, CV_64FC1);
  /// points = U W V'
  cvSVD(points, W, U, V);

  CvMat* rot_points = cvCreateMat(num_points, 2, CV_64FC1);
  
  /// rot_points = U * W
  cvMatMul(U, W, rot_points);
  float linearity = 0.0;
  for (int i = 0; i < num_points; i++)
  {
    linearity += pow(cvmGet(rot_points, i, 1), 2);
  }

  cvReleaseMat(&points);
  points = 0;
  cvReleaseMat(&W);
  W = 0;
  cvReleaseMat(&U);
  U = 0;
  cvReleaseMat(&V);
  V = 0;
  cvReleaseMat(&rot_points);
  rot_points = 0;

  features.push_back(linearity);
  
  /// compute circularity
  CvMat* A=cvCreateMat(num_points,3,CV_64FC1);
  
  CvMat* B = cvCreateMat(num_points, 1, CV_64FC1);
  {
    int j = 0;
    for(size_t pp=obstacle.start;pp<=obstacle.end;pp++)
    {
      float x = ranges[pp][0];
      float y = ranges[pp][1];

      cvmSet(A, j, 0, -2.0 * x);
      cvmSet(A, j, 1, -2.0 * y);
      cvmSet(A, j, 2, 1);

      cvmSet(B, j, 0, -pow(x, 2) - pow(y, 2));
      j++;
    }
  }
  CvMat* sol = cvCreateMat(3, 1, CV_64FC1);

  cvSolve(A, B, sol, CV_SVD);

  float xc = cvmGet(sol, 0, 0);
  float yc = cvmGet(sol, 1, 0);
  float rc = sqrt(pow(xc, 2) + pow(yc, 2) - cvmGet(sol, 2, 0));

  cvReleaseMat(&A);
  A = 0;
  cvReleaseMat(&B);
  B = 0;
  cvReleaseMat(&sol);
  sol = 0;

  float circularity = 0.0;
  for(size_t pp=obstacle.start;pp<=obstacle.end;pp++)
  {
    circularity += pow(rc - sqrt(pow(xc - ranges[pp][0], 2) + pow(yc - ranges[pp][0], 2)), 2);
  }

  features.push_back(circularity);

  // Radius
  float radius = rc;

  features.push_back(radius);
  return features;
}

bool motion_tracking::judgeFeatures(std::vector<float> & features)
{
  if(features.at(2) < 0.4)
    return true;
  return false;
}
