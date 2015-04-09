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
//#include "FeaturePoint.h"
#include "Relative_Pose.h"


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
    OpenGLPlot. Setview();
    
    CameraPose CameraPose;
    
    FeaturePts FeaturePts;
    
    /// flow control parameters
    
    int firstcapture = 1;
    bool SkipthisFrame=0;
    bool ThirdFrame=0;
    int loop =0;
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
                    frame = skipNFrames(camCapture, 0);  
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
                    
                    int numTrialFmatrix = 30;
                    int numTrialRelativePose = 30;
                    int Focuslength= 220;
                    int Ransac_threshold= 2.0;
                    float MaxAngle =0.08;
                   
                    EpipolarGeometry EpipolarGeometry(match_query, match_train, size_match, numTrialFmatrix, numTrialRelativePose, Focuslength, Ransac_threshold);  
                    
                    EpipolarGeometry.FindFundamentalMatrix(); 
                    EpipolarGeometry.FindRelativePose(EpipolarGeometry. FivePoints );    
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
                        vector<v3_t> V3Dpts1;
                        if(! ThirdFrame)
                        {
                            vector<v2_t> left_pts;
                            vector<v2_t> right_pts;
                            vector<v3_t> V3Dpts;    
                            

                            SkipthisFrame =0; 
                            EpipolarGeometry.InitializeFirstPmatrix();
                            EpipolarGeometry.TwoviewTriangulation(left_pts,right_pts,V3Dpts);
                           
                            CameraPose.InitializeFirstTwoKMatrix(EpipolarGeometry.K1matrix, EpipolarGeometry.K2matrix);
                            
                            CameraPose.First2viewInitialization(EpipolarGeometry.R1matrix, EpipolarGeometry.R_relative, EpipolarGeometry.t1matrix, EpipolarGeometry.t_relative);
                          
                            //OpenGLPlot. PlotCamera(EpipolarGeometry.t_relative);
                            //OpenGLPlot. PlotVertex(EpipolarGeometry.NumofPts(), V3Dpts);   
                                                        
                            //for (int i=0;i<(int) V3Dpts.size() ;i++)
                            // cout<<V3Dpts[i].p[0]<<" "<<V3Dpts[i].p[1]<<" "<<V3Dpts[i].p[2]<<endl;
                            
                            FeaturePts.Loadv2Pts( left_pts, right_pts);  
                            
                            // frame: Right-> left -> Right 
                           
                            FeaturePts.Loadv3Pts(V3Dpts); 
                            int FrameNum= 2 ;
                            FeaturePts.LoadFeatureList(FrameNum);
                        }
                        else
                        {
                       
                            CameraPose. InitializeKMatrix(Focuslength);
                            
                            int FrameNum =2;
                            
                            // Connect feature point and create feature tracks 
                            
                            FeaturePts.ConnectedVideoSequence( FeaturePts.m_rightPts, EpipolarGeometry.lrefined_pt /*connected pts*/ , EpipolarGeometry.rrefined_pt  /* current pts*/ , EpipolarGeometry.num_ofrefined_pts,FrameNum);
                             
                            CameraPose.Egomotion(EpipolarGeometry.R_relative, EpipolarGeometry.t_relative, FeaturePts);
                            
                            
                            //double T[3];
                            //memcpy(T, CameraPose.mTcMatrix[FrameNum].n,3*sizeof(double));
                            //matrix_print(3,1,T);
                            
                            cout<< FeaturePts.mv2_frame.size()<<endl;

                            CameraPose.TriangulationN_Frames( FeaturePts);
                            
                            cout<< FeaturePts.mv2_frame.size()<<endl;
                                                        
                            int idx= CameraPose.mtriTcmatrix.size();
                            
                            CameraPose.RemoveTri();
                            
                            vector<v2_t> left_pts;
                            vector<v2_t> right_pts;
                            vector<v3_t> V3Dpts;   
                            
                            loop++;
                            if (loop>1)
                            {
                              DumpPointsToPly("/Users/chih-hsiangchang/Desktop/Archive/result.ply", FeaturePts. _3DLocation
                                            , FeaturePts. _3DLocation.size());
                                break;
                             }
                            
                            
                            FeaturePts. UpdatedFeatureTrack(left_pts,right_pts,V3Dpts, FrameNum);
                            
                            FeaturePts. CleanFeatureTrack();
                            
                            FeaturePts.Loadv2Pts( left_pts, right_pts);  
                            
                            FeaturePts.Loadv3Pts(V3Dpts); 
                            
                            FeaturePts.LoadFeatureList(FrameNum);
                            
                            
                            //OpenGLPlot. Setview();
                            //OpenGLPlot. PlotCamera(T);
                            
                            //OpenGLPlot. PlotVertex(EpipolarGeometry.NumofPts(), V3Dpts); 
                            
                            //cout<< CameraPose.SizeofPose()<<endl;
                            //cout<< FeaturePts.NumReproject<<endl;

                                       
                        }
                        
                        imgB= cvCloneImage(frame);
                        ThirdFrame= true;
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