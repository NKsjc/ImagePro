#include<iostream>  
//#include"highgui.h"  
//#include"cv.h"  
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 
  
using namespace std;  
using namespace cv;  
  
int main()  
{  
    Mat image(cvSize(1487,2105),16);  
    cout << "image channels"<<image.channels()<<endl;
    for(size_t y=0;y<image.cols;y++)
    {
      unsigned char * row_ptr=image.ptr<unsigned char>(y);
      for(size_t x=0;x<image.rows;x++)
      {
	
	unsigned char * data_ptr =& row_ptr[x*image.channels()];
	for (int c=0;c!=image.channels();c++)
	{
	  image.at<Vec3b>(y,x)[c]=0;//data_ptr[c];
	}
      }
    }
    imshow("test1",image);  

    int nc = image.channels();  
      
    int nWidthOfROI = 160;  
  
    for (int j=0;j<image.rows;j++)  
    {  
        uchar* data= image.ptr<uchar>(j);  
        for(int i=0;i<image.cols*nc;i+=nc)  
        {             
            if( (i/nc/nWidthOfROI + j/nWidthOfROI) % 2)  
            {  
                // bgr  
                data[i/nc*nc + 0] = 255 ;  
                data[i/nc*nc + 1] = 255 ;  
                data[i/nc*nc + 2] = 255 ;                 
            }  
        }  
    }  
  
    imshow("test",image);  
    waitKey(0);  
    
    cv::destroyAllWindows();
  
}  