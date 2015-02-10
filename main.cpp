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
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include <cv.hpp>
#include <cxmisc.h>
#include <cxcore.h>
#include <cvaux.h>

CvCapture *camCapture;
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
IplImage*  frame ;
int Img_width = 640;
int Img_height= 480;
IplImage* skipNFrames(CvCapture* capture, int n);




using namespace cv;
using namespace std;

void RefindMatchdPts(vector<Point2f>& corners , vector<Point2f>& NestPts, int ImgWidth ,int ImgHeight)
{
    int Size_= (int) corners.size();
    
    cout<<"before "<<Size_<<endl;
    
    vector<Point2f> Tempcorner;
    vector<Point2f> TempNestPts;    
    
    for (int i =0; i<Size_;i++)
    {
        if (corners[i].x >0 && NestPts[i].x > 0)
        {
            if (corners[i].x < ImgWidth && NestPts[i].x< ImgWidth)
            {
                if (corners[i].y> 0 && NestPts[i].y >0)
                {
                    if (corners[i].y< ImgHeight && NestPts[i].y< ImgHeight)
                    {
                        
                        Tempcorner.push_back(corners[i]);
                        TempNestPts.push_back(NestPts[i]);
                    }
                }
            }
            
        }
    }
    
    corners.clear();
    NestPts.clear();
     
    corners = Tempcorner;
    NestPts = TempNestPts;
    
    cout<<"after "<<(int)corners.size()<<endl;

}
void ConnectedVideoSequence(vector<Point2f> tempCorners,vector<Point2f> NestPts, vector<Point>& tempPts)
{
    
    int ReferencePtsize =   (int) tempCorners.size();
    int MatchedPtsize   =   (int) NestPts.size();
       
    vector<Point2f> referencePts;
    vector<Point2f> matchedPts;
    //vector<Point>   tempPts;
    
    referencePts = tempCorners;
    matchedPts   = NestPts;
    
    
    for (int i=0; i< ReferencePtsize; i++)
    {
        //if (referencePts[i].x >0 && referencePts[i].y > 0)
            
        //{
            if (referencePts[i].x !=-99999 && referencePts[i].y !=-99999  )
            {
                int x =  referencePts[i].x;
                int y =  referencePts[i].y;
                
                for (int j=0; j<MatchedPtsize; j++)
                {
                    if  (matchedPts[j],x > 0 && matchedPts[j].y > 0)
                    {
                        //if ( matchedPts[j].x !=-99999 && matchedPts[j].y !=-99999 )
                        //{
                            int x_m =  matchedPts[j].x;
                            int y_m =  matchedPts[j].y;
                            
                            if (sqrt(((x-x_m)*(x-x_m))+((y-y_m)*(y-y_m)))<=1.414)
                            {
                                Point pt;
                                
                                pt.x = i;
                                pt.y = j;
                                
                                tempPts.push_back(pt);
                                
                                matchedPts[j].x=-99999;
                                matchedPts[j].y=-99999;
                                
                                referencePts[i].x = -99999;
                                referencePts[j].y = -99999;
                                
                            }
                            
                        } 
                    }
                }
            }
     //   }
    //}
}


int main (int argc, const char * argv[])
{
    
    
    if (!(camCapture = cvCaptureFromCAM(CV_CAP_ANY))) 
    {
        cout << "Failed to capture from camera" << endl;
        // goto exitCameraOpenFailed;
    }
    
    
    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_WIDTH,  320); 
    cvSetCaptureProperty(camCapture, CV_CAP_PROP_FRAME_HEIGHT, 240); 
    
    cout << "Camera opened successfully" << endl;
    
    IplImage *cameraFrame;    
    bool _1stframe=true;
    bool _1sttrack=true;
    cameraFrame = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U,1);
    
    IplImage * imgA=0;        
    IplImage * imgB=0;     
    IplImage * imgC=0;  
    
    IplImage *imgGrayA=0;
    IplImage *imgGrayB=0;
    
    Img_width=  320;
    Img_height= 240;
    
    imgA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgC = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    
    imgGrayA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);
    imgGrayB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1); 
    
    
       
    vector<Point2f> tempCorners;
    do 
        if ((cameraFrame = cvQueryFrame(camCapture))) 
        {
            frame = skipNFrames(camCapture,5);
            frame = cvQueryFrame(camCapture);
            
            Img_width= frame->width;
            Img_height= frame->height;
            
            if (_1stframe== true)
            {
                imgB=cvCloneImage(frame);           
            }
            
            if(_1stframe== false)
            {
                imgA= frame;               // new frame //
                imgC= cvCloneImage(imgB);  // previous frame //
                                
                cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);  
                cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);  
                   
                //cv::Mat flow_mat;
                vector<Point2f> corners, nextPts, trackedPts;
                vector<uchar> status;
                vector<float> err;
                vector<Point2f> conersNew;
               
                //std::vector<unsigned char> FAST_match_mask;
        
               if(_1sttrack==true)
                {
                   goodFeaturesToTrack(imgGrayB, corners, 300, 0.01 ,11);
                   cornerSubPix(imgGrayB, corners, Size(15,15), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                   calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(45,45));
                   _1sttrack=false;
                   tempCorners=nextPts;
                }
                
                if(_1sttrack==false)
                {
                    vector<Point>  tempPts;     //add temp points i->previous frame  j->current frame                                
                    
                    //cornerSubPix(imgGrayB, tempCorners, Size(13,13), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                    //calcOpticalFlowPyrLK(imgGrayB, imgGrayA, tempCorners, trackedPts, status, err, Size(45,45));
                    
                    goodFeaturesToTrack(imgGrayB, corners, 300, 0.01,11);
                    cornerSubPix(imgGrayB, corners, Size(15,15), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                    calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(45,45));
                    
                    RefindMatchdPts(corners , nextPts, Img_width, Img_height);
                    
                    cout<< " skip this line"<<endl;
                    cout<<endl;
                    
                    ConnectedVideoSequence(tempCorners, corners, tempPts);
                    
                    bool *tempPlotPts  = new bool [(int) nextPts.size()];
                    int  NumOverlapPts = (int)    tempPts.size();    
                    
                    cout<< "connect_pts "<< tempPts.size()<<endl;
                    
                    for (int i=0;i<NumOverlapPts;i++)
                    {
                        int idx = tempPts[i].y;  
                        tempPlotPts[idx]=1;
                    }
                    
                    
                    for (int y=0;y<corners.size();y++)
                    {
                        
                        //float location_x  = (int)  corners[y].x;
                        //float location_y  = (int)  corners[y].y;		
                        
                        float location1_x = (int)  nextPts[y].x;
                        float location1_y = (int)  nextPts[y].y;
                        
                        if  (tempPlotPts[y]==0)
                            cvCircle(imgA, cvPoint(location1_y, location1_x), 1, CV_RGB(0, 255, 0), -1); 
                        if  (tempPlotPts[y]==1)
                            cvCircle(imgA, cvPoint(location1_y, location1_x), 1, CV_RGB(255,0, 0), -1); 
                        
                    }
                    // add update corners //
                    tempCorners.clear();
                    tempCorners=nextPts;
                    tempPts.clear();
                    
                    delete [] tempPlotPts;
                    //cout<<(int) corners.size()<<endl;
                }
               
                 cout<< " second  "<<endl;
                 cout<<endl;
        
                              
               imgB= cvCloneImage(frame);
               /*
                 for ( int y = 0; y < (int) corners.size(); y++ ) 
                   {
                       int location_x= corners[y].x;
                       int location_y= corners[y].y;
                       float location_xB= nextPts[y].x;
                       float location_yB= nextPts[y].y;
                       CvScalar s;
                       //cout<<location_x<<" "<<location_y<<" "<<location_xB<<" "<<location_yB<<" "<<endl;
                       
                        if(location_x <320-1 && location_y < 240-1)
                         {
                         
                                                    
                          if(location_x>0 && location_y >0)
                           {
                              //s.val[0]=0;
                              //s.val[1]=0;
                              //s.val[2]=0;                           
                         
                           //else
                           //{
                           //s = cvGet2D(imgA, location_x, location_y);
                             s = cvGet2D(imgA,location_y, location_x);
                             cvCircle(imgA, cvPoint(location_y, location_x), 1, CV_RGB(0, 255, 0), -1);
                             //cout<<location_x<<" "<<location_y<<" "<<location_xB<<" "<<location_yB<<" "<<s.val[0]<<" "<<s.val[1]<<" "<<s.val[2]<<" "<<endl;
                              cout<<location_x<<" "<<location_y<<" "<<endl;
                           }
                       }
                       
                       //cvCircle(imgA, cvPoint(location_y, location_x), 1, CV_RGB(0, 255, 0), -1);
                       //cvCircle(imgB, cvPoint(location_xB, location_yB), 1, CV_RGB(0, 255, 0), -1);
               
                       // cout<<location_x<<" "<<location_y<<" "<<location_xB<<" "<<location_yB<<" "<<s.val[0]<<" "<<s.val[1]<<" "<<s.val[2]<<" "<<endl;
                   }
                 */ 
                  // _1stframe = false ;
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
//               cout<<"second image"<<endl; 
//               flow_mat.release();
//                
//                // printf("[%d] - Sheta:%lf, Length:%lf\n",i , fVecSetha, fVecLength);     
//                
                cvShowImage("frame1",imgA);
//                //cout<<"test 1"<<endl;
//                    cvShowImage("frame2",imgC);
            }        
            _1stframe=false;    
      }
    
    while (true) ;
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