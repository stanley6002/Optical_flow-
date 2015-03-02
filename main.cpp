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
#include "F_matrix.h"

CvCapture *camCapture;
#define IMAGE_WIDTH  320
#define IMAGE_HEIGHT 240
IplImage*  frame ;
int Img_width = 320;
int Img_height= 240;
IplImage* skipNFrames(CvCapture* capture, int n);

using namespace cv;
using namespace std;

# define PlotFeatureTracking


void RefindMatchedPts(vector<Point2f>& corners , vector<Point2f>& NestPts, int ImgWidth ,int ImgHeight)
{
    int Size_= (int) corners.size();
    
    //cout<<"before "<<Size_<<endl;
    
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
    
    //cout<<"after "<<(int)corners.size()<<endl;

}
void ConnectedVideoSequence(vector<Point2f> tempCorners,vector<Point2f> NestPts, vector<Point>& tempPts)
{
    
    int ReferencePtsize =   (int) tempCorners.size();
    int MatchedPtsize   =   (int) NestPts.size();
       
    vector<Point2f> referencePts;
    vector<Point2f> matchedPts;
    
    referencePts = tempCorners;
    matchedPts   = NestPts;
    
    
    for (int i=0; i< ReferencePtsize; i++)
    {
        if (referencePts[i].x !=-99999 && referencePts[i].y !=-99999  )
        {
            int x =  referencePts[i].x;
            int y =  referencePts[i].y;
            
            for (int j=0; j<MatchedPtsize; j++)
            {
                
                if (matchedPts[j].x !=-99999 && matchedPts[j].y !=-99999  )
                {
                    if  (matchedPts[j].x > 0 && matchedPts[j].y > 0)
                    {
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
                            referencePts[i].y = -99999;
                            
                        }
                        
                    } 
                }    
            }
        }
     }
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
    
    cameraFrame = cvCreateImage(cvSize (IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U,1);
    
    IplImage * imgA=0;        
    IplImage * imgB=0;     
    IplImage * imgC=0;  
    
    IplImage * imgGrayA=0;
    IplImage * imgGrayB=0;
    
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
            frame = skipNFrames(camCapture,8);
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
                   goodFeaturesToTrack(imgGrayB, corners, 350, 0.001, 10);
                   cornerSubPix(imgGrayB, corners, Size(9,9), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                   calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(45,45));
                   _1sttrack=false;
                   tempCorners=nextPts;
                 }
                
                if(_1sttrack==false)
                 {
                    vector<Point>  tempPts;     // add temp points i->previous frame  j->current frame                                
                    goodFeaturesToTrack(imgGrayB, corners, 350, 0.001, 10);
                    cornerSubPix(imgGrayB, corners, Size(9,9), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                                        calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(45,45));
                    RefindMatchedPts(corners , nextPts, Img_width, Img_height);
                    
                    //cout<< " skip this line"<<en-dl;
                    
                   
                    ConnectedVideoSequence(tempCorners, corners, tempPts);
                   
                    int NumbeofPts = (int) corners.size();
                    
                    double R_out[9];
                    double t_out[3];
                    double camera2t[3];
                     
                    //double K1[9],K2[9];
                    int Ransac_rounds = 50; 
                    double Ransac_threshold= 2 ;
                    
                    v2_t *cornersv2   = new v2_t [NumbeofPts];
                    v2_t *nextPtsv2   = new v2_t [NumbeofPts];
                    
                    double K1[9]= {
                         320.0, 0.0,  0.0, 
                         0.000000e+00, 320.0, 0.0, 
                         0.000000e+00,  0.000000e+00,  1.000000e+00} ;
                   
                    double K2[9]= {
                         320.0, 0.0,  0.0, 
                         0.000000e+00, 320.0, 0.0, 
                         0.000000e+00,  0.000000e+00,  1.000000e+00} ; 
                     
                     for (int i=0;i<NumbeofPts;i++)
                     {
                         v2_t p;
                         v2_t q;
                         p.p[0] = corners[i].x-160;
                         p.p[1] = corners[i].y-120;
                         q.p[0] = nextPts[i].x-160;
                         q.p[1] = nextPts[i].y-120;
                         
                         cornersv2[i].p[0] =p.p[0];
                         cornersv2[i].p[1] =p.p[1];
                         nextPtsv2[i].p[0] =q.p[0];
                         nextPtsv2[i].p[1] =q.p[1];
                         //cout<< lrefined_pt[i].p[0]<<" "<< lrefined_pt[i].p[1]<<" "<< rrefined_pt[i].p[0]<<" "<<rrefined_pt[i].p[1]<<endl;
                     }
                     
                     compute_pose_ransac(NumbeofPts, cornersv2, nextPtsv2, K1, K2, Ransac_threshold, Ransac_rounds , R_out ,t_out);
                     float Angle =  Apical_Angle(cornersv2, nextPtsv2, R_out , t_out , K1, NumbeofPts);
                     
                     matrix_transpose_product(3, 3, 3, 1, R_out, t_out , camera2t);
                     matrix_scale(3, 1, camera2t, -1.0, t_out);
                     //matrix_print(3,1,t_out);
                     
                     cout<<Angle<<" ";
                     delete [] nextPtsv2;
                     delete [] cornersv2;
#ifdef PlotFeatureTracking                  
                     // separate the points  for plat result//
                     //  tempoarary array to save the location of overlapped points
                     bool *tempPlotPts  = new bool [(int) nextPts.size()];
                     int  NumOverlapPts = (int)    tempPts.size();                        
                     cout<< "connect_pts "<< tempPts.size()<<endl;
                    
                    for (int i=0;i<NumOverlapPts;i++)
                    {
                        int idx = tempPts[i].y;  
                        // mark overlapped points to one 
                        tempPlotPts[idx]=1;
                    }
                    
                    for (int y=0;y<corners.size();y++)
                    {
                        
                        float location1_x = (int)  nextPts[y].x;
                        float location1_y = (int)  nextPts[y].y;
                        
                        if  (tempPlotPts[y]==0)     // Plot new Points
                            cvCircle(imgA, cvPoint(location1_x, location1_y),3, CV_RGB(0, 255, 0), -1); 
                        if  (tempPlotPts[y]==1)     // overlapped Points 
                            cvCircle(imgA, cvPoint(location1_x, location1_y),3, CV_RGB(255,0, 0), -1); 
                    }
#endif                    
                    // add update corners //
                    tempCorners.clear();
                    tempCorners=nextPts;
                    tempPts.clear();
                    
#ifdef PlotFeatureTracking
                    delete [] tempPlotPts;
#endif                    
                }
                
                
                
                
//                cout<< " second  "<<endl;
//                cout<<endl;
                imgB= cvCloneImage(frame);
                cvShowImage("frame1",imgA);
                
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