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
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
IplImage*  frame ;
int Img_width = 640;
int Img_height= 480;
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
    cameraFrame = cvCreateImage(cvSize(IMAGE_WIDTH,IMAGE_HEIGHT), IPL_DEPTH_8U,1);
    
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
                   goodFeaturesToTrack(imgGrayB, corners, 300, 0.01,9);
                   cornerSubPix(imgGrayB, corners, Size(7,7), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                   calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(45,45));
                   _1sttrack=false;
                   tempCorners=nextPts;
                 }
                
                if(_1sttrack==false)
                 {
                    vector<Point>  tempPts;     // add temp points i->previous frame  j->current frame                                
                    goodFeaturesToTrack(imgGrayB, corners, 300, 0.01,9);
                    cornerSubPix(imgGrayB, corners, Size(7,7), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                    calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(45,45));
                    RefindMatchedPts(corners , nextPts, Img_width, Img_height);
                    
                    //cout<< " skip this line"<<endl;
                    cout<<endl;
                    ConnectedVideoSequence(tempCorners, corners, tempPts);
                   
                     
                    //compute_pose_ransac(<#int n#>, <#v2_t *r_pts#>, <#v2_t *l_pts#>, <#double *K1#>, <#double *K2#>, <#double ransac_threshold#>, <#int ransac_rounds#>, <#double *R_out#>, <#double *t_out#>);
                    
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
                        
                        //cvLine(imgA, corners[y], nextPts[y], CV_RGB(0, 255, 0) ,1, 8, 0 );
                        
                    }
#endif                    
                    // add update corners //
                    tempCorners.clear();
                    tempCorners=nextPts;
                    tempPts.clear();
                    
#ifdef PlotFeatureTracking
                    delete [] tempPlotPts;
#endif                    
                    //cout<<(int) corners.size()<<endl;
                }
                
                cout<< " second  "<<endl;
                cout<<endl;
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