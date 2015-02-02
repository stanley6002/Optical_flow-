//
//  main.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 7/1/14.
//  Copyright 2014 __MyCompanyName__. All rights reserved.
//

#include <iostream>

#include "opencv/cv.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

CvCapture *camCapture;
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
IplImage*  frame ;
int Img_width = 640;
int Img_height= 480;
IplImage* skipNFrames(CvCapture* capture, int n);


using namespace cv;
using namespace std;
int main (int argc, const char * argv[])
{
    
    
//    if (!(camCapture = cvCaptureFromCAM(CV_CAP_ANY))) 
//    {
//        cout << "Failed to capture from camera" << endl;
////        // goto exitCameraOpenFailed;
//    }
    
    
    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_WIDTH,  640); 
    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_HEIGHT, 480); 
    
    cout << "Camera opened successfully" << endl;
    
    IplImage *cameraFrame;    
    bool _1stframe=true;
    cameraFrame = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U,1);
    
    IplImage * imgA=0;        
    IplImage * imgB=0;     
    IplImage * imgC=0;  
    
    IplImage *imgGrayA=0;
    IplImage *imgGrayB=0;
    
    Img_width=  640;
    Img_height= 480;
    
    int win_size=21;   
    
    imgA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgC = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    
    imgGrayA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);
    imgGrayB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1); 
    
    
    imgA = cvLoadImage("/P1_1.jpg", 3);
    imgB = cvLoadImage("/P1_2.jpg", 3);
    
    int grabFrameRet;
 //   do 
 //       if ((cameraFrame = cvQueryFrame(camCapture))) 
 //       {
 //           frame = skipNFrames(camCapture,10);
 //           frame = cvQueryFrame(camCapture);
            
            //Img_width= frame->width;
            //Img_height= frame->height;
            Img_width=  640;
            Img_height= 480;
//            if (_1stframe== true)
//            {
//                imgB=cvCloneImage(frame);           
//            }
//            if(_1stframe== false)
//           {
//                imgA= frame;
                
//                imgC= cvCloneImage(imgB);
//                imgB= cvCloneImage(frame);
                
                cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);  
                cvCvtColor(imgB,imgGrayB, CV_BGR2GRAY);  
                
                
                cv::Mat flow_mat;
                vector<Point2f> corners,nextPts; vector<uchar> status; vector<float> err;
                goodFeaturesToTrack(imgGrayA, corners, 500, 0.001,10);
                cornerSubPix(imgGrayA, corners, Size(17,17), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                calcOpticalFlowPyrLK(imgGrayA, imgGrayB, corners, nextPts, status, err, Size(45,45));
           
                 for ( int y = 0; y < (int) corners.size(); y++ ) 
                   {
                       int location_x= corners[y].x;
                       int location_y= corners[y].y;
                       float location_xB= nextPts[y].x;
                       float location_yB= nextPts[y].y;
                       
                       CvScalar s= cvGet2D(imgA, location_y, location_x);
                       //cvCircle(imgA, cvPoint(location_x, location_y), 1, CV_RGB(0, 255, 0), -1);
                       cvCircle(imgB, cvPoint(location_xB, location_yB), 1, CV_RGB(0, 255, 0), -1);
                       //cout<<location_x<<" "<<location_y<<" "<<location_xB<<" "<<location_yB<<" "<<s.val[0]<<" "<<s.val[1]<<" "<<s.val[2]<<endl;
                       cout<<location_x<<" "<<location_y<<" "<<location_xB<<" "<<location_yB<<" "<<0<<" "<<0<<" "<<0<<endl;
                   }
//                 calcOpticalFlowFarneback( imgGrayA, imgGrayB, flow_mat, 0.1, 3, 15, 3, 5, 1.5,  cv::OPTFLOW_FARNEBACK_GAUSSIAN);                        
               
                //vector<CvPoint2D32f> left_points, right_points;
//                
//                for ( int y = 0; y < imgGrayA->height; y+=14 ) 
//                {
//                    for ( int x = 0; x < imgGrayA->width; x+=14 ) 
//                    {
//                        /* Flow is basically the delta between left and right points */
//                         const cv::Point2f& fxy = flow_mat.at<cv::Point2f>(y, x);
//                         /*  There's no need to calculate for every single point,
//                          if there's not much change, just ignore it
//                        */
//                        if ((x+fxy.x)>0 && (y+fxy.y)>0)
//                        {
//                          if ((sqrt((fxy.x*fxy.x)+(fxy.y*fxy.y))>20) && (sqrt((fxy.x*fxy.x)+(fxy.y*fxy.y)))<160)
//                         //if( fabs(fxy.x) > 4 || fabs(fxy.y) > 4 )
//                          {
//                         //continue;
//                         cvCircle(imgB, cvPoint(x+fxy.x, y+fxy.y), 1, CV_RGB(0, 255, 0), -1);
//                         //cvCircle(imgA, cvPoin t(x, y), 1, CV_RGB(0, 255, 0), -1);
//                         //CvScalar s= cvGet2D(imgA, y, x);
//                         //cout<<x<<" "<<y<<" "<< x+fxy.x<<" "<<y +fxy.y<<" "<<" "<<s.val[2]<<" "<<s.val[1]<<" "<<s.val[0]<<endl;
//                          }
//                        }
//                      
//                        //left_points.push_back(  cvPoint2D32f ( x, y ) );
//                        //right_points.push_back( cvPoint2D32f( x + fxy.x, y + fxy.y ) );
//                    }
//                }
//                
//                
//                flow_mat.release();
//                
//                // printf("[%d] - Sheta:%lf, Length:%lf\n",i , fVecSetha, fVecLength);     
//                
                cvShowImage("frame1",imgB);
//                //cout<<"test 1"<<endl;
//                    cvShowImage("frame2",imgC);
//            }        
//            _1stframe=false;   
            
            
//        }
    
    //while (true) ;
    cvReleaseImage(&frame);
    cvReleaseImage(&cameraFrame);
    cvReleaseImage(&imgA);
    cvReleaseImage(&imgB);
    cvReleaseImage(&imgC);
    cvReleaseImage(&imgGrayA);
    cvReleaseImage(&imgGrayB);
    
}

IplImage* skipNFrames(CvCapture* capture, int n)
{
    for(int i = 0; i < n; ++i)
    {
        if(cvQueryFrame(capture) == NULL)
        {
            return NULL;
        }
    }
    
    return cvQueryFrame(capture);
}