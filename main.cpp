//
//  main.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 7/1/14.
//  Copyright 2014 __MyCompanyName__. All rights reserved.
//

#include <iostream>
#include "F_matrix.h"
#include "videoprocessing.h"
#include "matchingandtracking.h"
#include "FFME.h"
#include "miscul.h"
#include "OpenglPlot.h"

CvCapture *camCapture;

#define CenterX(x) ((x)-IMAGE_WIDTH/2)
#define CenterY(x) ((x)-IMAGE_HEIGHT/2)

int Img_width = 320;
int Img_height= 240;

IplImage* skipNFrames(CvCapture* capture, int n);

IplImage* plot_two_imagesf(IplImage *IGray, IplImage *IGray1, CvPoint *corners1, CvPoint *corners3, int vector_index);

using namespace cv;
using namespace std;


# define PlotFeatureTracking
void _3DPtsRefinement (const v2_t* nextPtsv2, const v2_t*cornersv2, v3_t*_3DPts, const int NumofPts,  vector< v2_t > & nextPtsvRefined , vector< v2_t > &cornersvRefined, vector <v3_t> &_3DPtsRefined )
{
    int tempvector[NumofPts];
    memset(tempvector,0, sizeof(tempvector) );
    int tempNumPts= NumofPts;
    
    for(int i=0;i<NumofPts;i++) 
    {
         if( _3DPts[i].p[2] >=0)
         {
               tempvector[i]=1;
               tempNumPts-=1;
                
         }  
     }
    
    for (int i=0;i<NumofPts;i++) 
    {
       if ( tempvector[i]==0 )
       {
           nextPtsvRefined.push_back(nextPtsv2[i]);
           cornersvRefined.push_back(cornersv2[i]);
           _3DPtsRefined.push_back(_3DPts[i]);
       }
    }
  
    cout<< "numberofPts"<< NumofPts<<endl;
    cout<<"size "<< nextPtsvRefined.size()<<" "<<cornersvRefined.size()<<" "<<_3DPtsRefined.size()<<endl;


}

void RefindMatchedPts(vector<Point2f>& corners , vector<Point2f>& NestPts, int ImgWidth ,int ImgHeight)
{
    int Size_= (int) corners.size();
    
    vector<Point2f> Tempcorner;
    vector<Point2f> TempNestPts;    
    
    for (int i =0; i<Size_;i++)
    {
        if (corners[i].x >5 && NestPts[i].x > 5)
        {
            if (corners[i].x < ImgWidth-5 && NestPts[i].x< ImgWidth-5 )
             {
                if (corners[i].y> 5 && NestPts[i].y >5)
                {
                    if (corners[i].y< ImgHeight-5 && NestPts[i].y< ImgHeight-5)
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
    //bool _1sttrack=true;
    
    cameraFrame = cvCreateImage(cvSize (Img_width,Img_height), IPL_DEPTH_8U,3);
    
    IplImage * imgA=0;        
    IplImage * imgB=0;     
    IplImage * imgC=0;  
    
    IplImage * imgGrayA=0;
    IplImage * imgGrayB=0;
    
    Img_width=  IMAGE_WIDTH ;
    Img_height= IMAGE_HEIGHT ;
    
    imgA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgC = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    
    imgGrayA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);
    imgGrayB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1); 
    
    //IplImage* Two_image = 0;
    
    vector<Point2f> tempCorners;
    
    IplImage*  frame ;
    
    VideoProcessing VideoProcessing (Img_width, Img_height);
    
    OpenGLPlot OpenGLPlot (Img_width*2, Img_height*2);
    
    /// flow control parameters
    
    int firstcapture = 1;
    bool SkipthisFrame=0;
    bool ThirdFrame=0;
    
    //  flow control 
    do 
        if ((cameraFrame = cvQueryFrame(camCapture))) 
         {
                    
            if ( _1stframe)
            {
                frame = cvQueryFrame(camCapture);
                frame = skipNFrames(camCapture, 10);
                imgB  = cvCloneImage(frame);           
            }
            
            if( ! _1stframe)
            {
                bool CaptureFrames=0; 
                if (! SkipthisFrame) 
                {
                    frame = skipNFrames(camCapture, 1);  
                    frame = VideoProcessing.CaptureInitialFrame(camCapture);
                  
                    if (! CaptureFrames)
                    {                                         
                        if (VideoProcessing.captureNextFrame)
                        {
                            CaptureFrames =1;
                        }
                     }
                }
                else 
                {
                    cout<<"Frame skipped "<<endl;
                    frame = VideoProcessing.CaptureInitialFrame(camCapture);
                    
                    if (! CaptureFrames)
                    {                                         
                        if (VideoProcessing.captureNextFrame)
                        {
                            CaptureFrames =1;
                        }
                    }

                }
                if(CaptureFrames==1)
                {              
                    imgA= frame;  // new frame //
                    imgC= cvCloneImage(imgB);  // previous frame //
                    
                    cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);  
                    cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);
                  
                    LKFeatures LKFeatures (imgGrayA,imgGrayB, LKFeatures.Optical_flow);                   
                    std::vector<CvPoint2D32f> match_query; 
                    std::vector<CvPoint2D32f> match_train;
                    LKFeatures.FeaturesMatched (match_query, match_train);
                    
                    // for plot 
                    int size_match= (int) match_query.size();
                    
                    int numTrialFmatrix = 50;
                    int numTrialRelativePose  =50;
                    int Focuslength= 280;
                    int Ransac_threshold= 2.0;
                    
                    EpipolarGeometry EpipolarGeometry(match_query, match_train, size_match, numTrialFmatrix, numTrialRelativePose, Focuslength, Ransac_threshold);  
                    EpipolarGeometry.FindFundamentalMatrix(); 
                    EpipolarGeometry.FindRelativePose();
                    
                    float MaxAngle =0.065;
                    
                    EpipolarGeometry.FindApicalAngle(MaxAngle);
                    
                    IplImage* Two_image=EpipolarGeometry.plot_two_imagesf(imgC, imgA);
                    cvShowImage("test", Two_image);
                    
                    cout<<EpipolarGeometry.ApicalAngle<<endl;
                    cout<<endl;
                   
                    if  (EpipolarGeometry.SkipFrame())
                    {   
                        SkipthisFrame =1;
                    }
                    else
                    {
                      if(! ThirdFrame)
                      {
                        vector<v2_t> left_pts;
                        vector<v2_t> right_pts;
                        vector<v3_t> V3Dpts;    
                        
                        SkipthisFrame =0; 
                        EpipolarGeometry.InitializeFirstPmatrix();
                        EpipolarGeometry.TwoviewTriangulation(left_pts,right_pts,V3Dpts);
                        
                        OpenGLPlot. Setview();
                        OpenGLPlot. PlotCamera(EpipolarGeometry.t_relative);
                        OpenGLPlot. PlotVertex(EpipolarGeometry.NumofPts(), V3Dpts);                           
                          
                      }             
                        imgB= cvCloneImage(frame);
                      }   
                  }
                 firstcapture=0;
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