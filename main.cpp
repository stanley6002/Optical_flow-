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

CvCapture *camCapture;
#define IMAGE_WIDTH  320
#define IMAGE_HEIGHT 240

int Img_width = 320;
int Img_height= 240;

IplImage* skipNFrames(CvCapture* capture, int n);

IplImage* plot_two_imagesf(IplImage *IGray, IplImage *IGray1, CvPoint *corners1, CvPoint *corners3, int vector_index);

using namespace cv;
using namespace std;


# define PlotFeatureTracking
void _3DPtsRefinement (const v2_t* nextPtsv2, const v2_t*cornersv2, v3_t*_3DPts, const int NumofPts, 
                                vector< v2_t > & nextPtsvRefined , vector< v2_t > &cornersvRefined, vector <v3_t> &_3DPtsRefined )
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
    
    //cout<<"before "<<Size_<<endl;
    
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
    
    Img_width=  320;
    Img_height= 240;
    
    imgA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgC = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    
    imgGrayA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);
    imgGrayB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1); 
    
    //IplImage* Two_image = 0;
    
    vector<Point2f> tempCorners;
    //int SkipThisFrame=0;
    //int firstcapture =1;
    
    IplImage*  frame ;
    VideoProcessing VideoProcessing (Img_width, Img_height);
    int firstcapture =1;
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
               
                //if(firstcapture==1)
                //{
                  
                //    frame = skipNFrames(camCapture, 2);
                  
                //}
                
                //else
                //{
                frame = skipNFrames(camCapture, 2);  
                //}
                frame = VideoProcessing.CaptureInitialFrame(camCapture);
              
                if (! CaptureFrames)
                {                                          
                   
                    if (VideoProcessing.captureNextFrame)
                    {
                        CaptureFrames =1;
                    }
                }

                if(CaptureFrames==1)
                {              
                    
                    imgA= frame;  // new frame //
                    imgC= cvCloneImage(imgB);  // previous frame //
                    
                    cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);  
                    cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);
                  
                      //OrbFeatureDetector
                    
                    
//                    int noMaxPoints=1000;
//                    CvPoint2D32f* singPoints1;
//                    CvPoint2D32f* singPoints2;
//                    
//                    int noSingPoints1;
//                    int noSingPoints2;
//                    int noCorr;
//                    singPoints1 = (CvPoint2D32f*)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f));
//                    singPoints2 = (CvPoint2D32f*)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f));
//                
//                    float** descriptors1;
//                    float** descriptors2;
//                    CvPoint2D32f** correspondences;
//                    
//                    FFME ffme;
//                    ffme.iniFFME(Img_width, Img_height, imgA->origin, 1000);
//                    
//                    descriptors1 = (float**)cvAlloc(noMaxPoints * sizeof(float*));
//                    descriptors2 = (float**)cvAlloc(noMaxPoints * sizeof(float*));
//                    
//                    int lengthDesc = ffme.m_widthArrayHist*ffme.m_widthArrayHist*ffme.m_noBinsOriHist;
//                    
//                    for(int i=0;i<noMaxPoints;i++)
//                    {
//                        descriptors1[i] = (float*)cvAlloc(lengthDesc * sizeof(float));
//                    }
//                    for(int i=0;i<noMaxPoints;i++)
//                    {
//                        descriptors2[i] = (float*)cvAlloc(lengthDesc * sizeof(float));
//                    }
//                     correspondences = (CvPoint2D32f**)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f*));
//                    for(int i=0;i<noMaxPoints;i++)
//                    {
//                        correspondences[i] = (CvPoint2D32f*)cvAlloc(2 * sizeof(CvPoint2D32f));
//                    }
//
//                    
//                    ffme.singPtoDetLut(imgGrayA, singPoints1, &noSingPoints1);
//                    ffme.singPtoDetLut(imgGrayB, singPoints2, &noSingPoints2);
//                    ffme.m_radMaxSearch =32;
//                    ffme.matchSingPtos(singPoints2, noSingPoints2, descriptors2, singPoints1, noSingPoints1, descriptors1, correspondences, &noCorr);
//                    
//                    cout<<noCorr<<endl;
//                    CvPoint *pt_new_query_t = new CvPoint [noCorr];
//                    CvPoint *pt_new_train_t= new CvPoint  [noCorr];
//                    
//                    
//                    for (int i=0; i< noCorr;i++)
//                    {
//                        pt_new_query_t[i].x = correspondences[i][0].x;
//                        pt_new_query_t[i].y = correspondences[i][0].y ;
//                       
//                        pt_new_train_t[i].x = correspondences[i][1].x;
//                        pt_new_train_t[i].y = correspondences[i][1].y ;
//
//                    
//                    }
//                    
//                    IplImage* Two_image_1= plot_two_imagesf(imgC,imgA, pt_new_query_t, pt_new_train_t,noCorr);
//                    cvShowImage("test", Two_image_1);
//            
//                    int size_match = noCorr;
//                    v3_t * r_pt = new v3_t [size_match];
//                    v3_t * l_pt = new v3_t [size_match];
//                      for(int i=0 ; i< size_match;i++) 
//                              {
//                                            //pt_new_query[i].x = match_query[i].x;
//                                            //pt_new_query[i].y = match_query[i].y;
//                                            
//                                            //pt_new_train[i].x = match_train[i].x;
//                                            //pt_new_train[i].y = match_train[i].y;
//                                            
//                                            r_pt[i].p[0]=pt_new_query_t[i].x;
//                                            r_pt[i].p[1]=pt_new_query_t[i].y;
//                                            r_pt[i].p[2]=1.0;
//                                            
//                                            l_pt[i].p[0]=pt_new_train_t[i].x;
//                                            l_pt[i].p[1]=pt_new_train_t[i].y;
//                                            l_pt[i].p[2]=1.0;
//                    
//                                            
//                                        }

                    
                    
//                    LKFeatures LKFeatures (imgGrayB,imgGrayA);
                    
//                    std::vector<CvPoint2D32f> match_query; 
//                    std::vector<CvPoint2D32f> match_train;
                    
//                    LKFeatures.FeaturesMatched (match_query, match_train);
                  
             

                     FAST_ FAST_ (30, imgGrayA, imgGrayB , FAST_.SURF_descriptor);
                     std::vector<CvPoint2D32f> match_query;
                     std::vector<CvPoint2D32f> match_train;
                     FAST_.FAST_tracking(match_query, match_train);
                    
                    int size_match= (int) match_query.size();
                    CvPoint *pt_new_query = new CvPoint [size_match];
                    CvPoint *pt_new_train= new CvPoint [size_match];
                    
                    //for(int i=0 ; i< size_match;i++) 
                    //{
                    //    pt_new_query[i].x = match_query[i].x;
                    //    pt_new_query[i].y = match_query[i].y;
                        
                    //    pt_new_train[i].x = match_train[i].x;
                    //    pt_new_train[i].y = match_train[i].y;
                        
                    //}

                                   
                    for(int i=0 ; i<size_match;i++)
                    {
                        pt_new_query[i].x = match_query[i].x;
                        pt_new_query[i].y = match_query[i].y;
                        
                        pt_new_train[i].x = match_train[i].x;
                        pt_new_train[i].y = match_train[i].y;
                        
                        cout<<pt_new_query[i].x<<" "<<pt_new_query[i].y<<" "<<pt_new_train[i].x<<" "<<pt_new_train[i].y<<endl;
                    }

                    
                    v3_t * r_pt = new v3_t [size_match];
                    v3_t * l_pt = new v3_t [size_match];
                    
                    for(int i=0 ; i< size_match;i++) 
                    {
                        pt_new_query[i].x = match_query[i].x;
                        pt_new_query[i].y = match_query[i].y;
                        
                        pt_new_train[i].x = match_train[i].x;
                        pt_new_train[i].y = match_train[i].y;
                        
                        r_pt[i].p[0]=match_query[i].x;
                        r_pt[i].p[1]=match_query[i].y;
                        r_pt[i].p[2]=1.0;
                        
                        l_pt[i].p[0]=match_train[i].x;
                        l_pt[i].p[1]=match_train[i].y;
                        l_pt[i].p[2]=1.0;

                        
                    }
                    //size_match=100;
                    IplImage* Two_image= plot_two_imagesf(imgA,imgC, pt_new_query, pt_new_train,size_match );
                    cvShowImage("test", Two_image);

                    double F[9];
                    int essential=0;
                    int num_trial=200;
                    matched * refined_pts= new matched[1];     
                    
                    F_matrix_process (size_match,  r_pt, l_pt, F, num_trial, 10 ,  essential, refined_pts, 0);
                    
                    int num_ofrefined_pts= (int)refined_pts[0].R_pts.size();
                    
                    //cout ith element's data length   
                    //F_key_matrix *F_matrix=new F_key_matrix[2];                                               /// number of F_matrix
                    //pushback_Fmatrix(F,F_matrix,0);                                                          // 0 is ith pairs
                    /// edit to stadard form
                    
                
                    v2_t *lrefined_pt= new v2_t[num_ofrefined_pts];
                    v2_t *rrefined_pt= new v2_t[num_ofrefined_pts];

                    pop_backpts_WI(lrefined_pt,rrefined_pt,refined_pts,0);
                    
                    for (int i=0;i<num_ofrefined_pts;i++)
                    {
                        v2_t p;
                        v2_t q;
                        
                        p.p[0] = lrefined_pt[i].p[0]-160;
                        p.p[1] = lrefined_pt[i].p[1]-120;
                        q.p[0] = rrefined_pt[i].p[0]-160;
                        q.p[1] = rrefined_pt[i].p[1]-120;
                        
                        lrefined_pt[i].p[0]=p.p[0];
                        lrefined_pt[i].p[1]=p.p[1];
                        rrefined_pt[i].p[0]=q.p[0];
                        rrefined_pt[i].p[1]=q.p[1];
                        //cout<< lrefined_pt[i].p[0]<<" "<< lrefined_pt[i].p[1]<<" "<< rrefined_pt[i].p[0]<<" "<<rrefined_pt[i].p[1]<<endl;
                    }

                    
                    int NumberofPts = num_ofrefined_pts; 
                    double R_out[9];
                    double t_out[3];
                    double camera2t[3];
                    
                    //double K1[9],K2[9];
                    
                    int Ransac_rounds = 300; 
                    double Ransac_threshold= 2 ;
                    
                    v2_t *cornersv2   = new v2_t [NumberofPts];
                    v2_t *nextPtsv2   = new v2_t [NumberofPts];
                    
                    double K1[9]= {
                        320.0, 0.0,  0.0, 
                        0.000000e+00, 320.0, 0.0, 
                        0.000000e+00,  0.000000e+00,  1.000000e+00} ;
                    
                    double K2[9]= {
                        320.0, 0.0,  0.0, 
                        0.000000e+00, 320.0, 0.0, 
                        0.000000e+00,  0.000000e+00,  1.000000e+00} ; 
                    
//                    for (int i=0;i<NumberofPts;i++)
//                    {
//                        v2_t p;
//                        v2_t q;
//                        p.p[0] = match_query[i].x-160;
//                        p.p[1] = match_query[i].y-120;
//                        q.p[0] = match_train[i].x-160;
//                        q.p[1] = match_train[i].y-120;
//                        
//                        cornersv2[i].p[0] =p.p[0];
//                        cornersv2[i].p[1] =p.p[1];
//                        nextPtsv2[i].p[0] =q.p[0];
//                        nextPtsv2[i].p[1] =q.p[1];
//                        //cout<< lrefined_pt[i].p[0]<<" "<< lrefined_pt[i].p[1]<<" "<< rrefined_pt[i].p[0]<<" "<<rrefined_pt[i].p[1]<<endl;
//                    }

                    compute_pose_ransac(NumberofPts, lrefined_pt, rrefined_pt, K1, K2, Ransac_threshold, Ransac_rounds , R_out ,t_out);
                  
                    matrix_transpose_product(3, 3, 3, 1, R_out, t_out , camera2t);
                    matrix_scale(3, 1, camera2t, -1.0, t_out);
                    matrix_print(3,1,t_out);
                    cout<<endl;
                     float Angle =  Apical_Angle(cornersv2, nextPtsv2, R_out , t_out , K1, NumberofPts); 
                    cout<<Angle<<endl;
                    double camera_1R[9];
                    double camera_1t[3];
                                 
                    double camera_2R[9];
                    double camera_2t[3];
                                         
                    memcpy(camera_2R, R_out, sizeof(double)*9);
                    memcpy(camera_2t, t_out, sizeof(double)*3);        
                    
                    camera_1R[0] = 1.0;  camera_1R[1] = 0.0;  camera_1R[2] = 0.0;
                    camera_1R[3] = 0.0;  camera_1R[4] = 1.0;  camera_1R[5] = 0.0;
                    camera_1R[6] = 0.0;  camera_1R[7] = 0.0;  camera_1R[8] = 1.0;        
                    camera_1t[0] = 0.0;  camera_1t[1] = 0.0;  camera_1t[2] = 0.0; 
                    
                     double error_tr=0;
                         
                     for (int i=0; i<NumberofPts ;i++)
                     {
                         bool in_front = true;
                         double angle = 0.0;
                         v3_t temp; 
                         v2_t p;
                         v2_t q;
                         p.p[0] = lrefined_pt[i].p[0];
                         p.p[1] = lrefined_pt[i].p[1];
                         q.p[0] = rrefined_pt[i].p[0];
                         q.p[1] = rrefined_pt[i].p[1]; 
                         temp = Triangulate(p, q, camera_1R, camera_1t,camera_2R,camera_2t, error_tr, in_front, angle,true,K1,K2);  
                         printf("%0.4f %0.4f %0.4f\n", temp.p[0], temp.p[1],temp.p[2]);
                         //                         printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", temp.p[0], temp.p[1],temp.p[2] ,p.p[0]+160, p.p[1]+120,q.p[0]+160,q.p[1]+120);
                                               }

                    break;
                   //if(SkipThisFrame ==0) 
                   //imgC= cvCloneImage(imgB);  // previous frame //
                                
                   //cvCvtColor(imgA,imgGrayA, CV_BGR2GRAY);  
                   //cvCvtColor(imgC,imgGrayB, CV_BGR2GRAY);  
                   
                   //cv::Mat flow_mat;
                   //vector<Point2f> corners, nextPts, trackedPts;
                   //vector<uchar> status;
                   //vector<float> err;
                   //vector<Point2f> conersNew;
               
                 //std::vector<unsigned char> FAST_match_mask;
         
                 //if(_1sttrack==true)
                 // {
                 //  goodFeaturesToTrack(imgGrayB, corners, 400, 0.001, 8);
                 //  cornerSubPix(imgGrayB, corners, Size(7,7), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 20, 0.03 ));
                 //  calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(50,50));
                 //  _1sttrack=false;
                 //  tempCorners=nextPts;
                 // }
                
                 //if(_1sttrack==false)
                 //{
                 //   vector<Point>  tempPts;     // add temp points i->previous frame  j->current frame                                
                    
                 //    std::vector<uchar> features_found; 
                 //    std::vector<float> feature_errors;
                     
                 //  goodFeaturesToTrack(imgGrayB, corners, 400, 0.001, 8);
                 //  cornerSubPix(imgGrayB, corners, Size(7,7), Size(-1,-1), TermCriteria(  CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 20, 0.03 ));
                 //   calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(50,50));
                  
                 //   RefindMatchedPts(corners , nextPts, Img_width, Img_height);
                     
                 //  cout<< " skip this line"<<endl;
                    
                 //  ConnectedVideoSequence(tempCorners, corners, tempPts);
                   
                 //  int NumbeofPts = (int) corners.size();
                 /* 
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
                     matrix_print(3,1,t_out);
                     
                     cout<<Angle<<" "<<endl;
                     //SkipThisFrame=0;
                     
                     if (Angle < 0.01)
                         SkipThisFrame=1;   
                     else 
                         SkipThisFrame=0; 
                         
                         
                     delete [] nextPtsv2;
                     delete [] cornersv2;
                     
                     v3_t *_3DPts = new v3_t [NumbeofPts];
                     */
                    
//                     if (Angle>0.06)
//                     {
//                     // initialized first one camera relative-pose //
//                     double camera_1R[9];
//                     double camera_1t[3];
//                     
//                     double camera_2R[9];
//                     double camera_2t[3];
//                     
//                     memcpy(camera_2R, R_out, sizeof(double)*9);
//                     memcpy(camera_2t, t_out, sizeof(double)*3);        
//
//                     camera_1R[0] = 1.0;  camera_1R[1] = 0.0;  camera_1R[2] = 0.0;
//                     camera_1R[3] = 0.0;  camera_1R[4] = 1.0;  camera_1R[5] = 0.0;
//                     camera_1R[6] = 0.0;  camera_1R[7] = 0.0;  camera_1R[8] = 1.0;        
//                     camera_1t[0] = 0.0;  camera_1t[1] = 0.0;  camera_1t[2] = 0.0; 
//
//                     double error_tr=0;
//                         
//                     for (int i=0; i<NumbeofPts ;i++)
//                     {
//                         bool in_front = true;
//                         double angle = 0.0;
//                         v3_t temp; 
//                         v2_t p;
//                         v2_t q;
//                         p.p[0] = cornersv2[i].p[0];
//                         p.p[1] = cornersv2[i].p[1];
//                         q.p[0] = nextPtsv2[i].p[0];
//                         q.p[1] = nextPtsv2[i].p[1];
//                         
//                         temp = Triangulate(p, q, camera_1R, camera_1t,camera_2R,camera_2t, error_tr, in_front, angle,true,K1,K2);     
//                         _3DPts[i]= temp;
//                         //printf("%0.4f %0.4f %0.4f\n", temp.p[0], temp.p[1],temp.p[2]);
//                         printf("%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\n", temp.p[0], temp.p[1],temp.p[2] ,p.p[0]+160, p.p[1]+120,q.p[0]+160,q.p[1]+120);
//                      }
//                       vector<v2_t> nextPtsvRefined;
//                       vector<v2_t> cornersvRefined;
//                       vector<v3_t> _3DPtsRefined;
//                         
//                         _3DPtsRefinement ( nextPtsv2, cornersv2,  _3DPts,NumbeofPts,  nextPtsvRefined ,  cornersvRefined,  _3DPtsRefined );
//                         
//                         int ptsize = (int) _3DPtsRefined.size();
//                         
//                         for (int i=0; i<ptsize ;i++)
//                         {
//                             bool in_front = true;
//                             double angle = 0.0;
//                             v3_t temp; 
//                             v2_t p;
//                             v2_t q;
//                             p.p[0] = cornersvRefined[i].p[0];
//                             p.p[1] = cornersvRefined[i].p[1];
//                             q.p[0] = nextPtsvRefined[i].p[0];
//                             q.p[1] = nextPtsvRefined[i].p[1];
//                             
//                             temp = Triangulate(p, q, camera_1R, camera_1t,camera_2R,camera_2t, error_tr, in_front, angle,true,K1,K2);     
//                             //_3DPts[i]= temp;
//                              
//                              printf(" %0.4f %0.4f %0.4f\n", temp.p[0], temp.p[1],temp.p[2]);
//                              //printf(" %0.6f %0.6f %0.6f\n", _3DPtsRefined[i].p[0], _3DPtsRefined[i].p[1],_3DPtsRefined[i].p[2]);
//                         }
//                    
//                        
//                         
//                         break;
//                     }
                    
                     
#ifdef PlotFeatureTracking                  
                     // separate the points  for plat result//
                     //  tempoarary array to save the location of overlapped points
//                   bool *tempPlotPts  = new bool [(int) nextPts.size()];
//                   int  NumOverlapPts = (int)    tempPts.size();                        
                   
                    
//                    for (int i=0;i<NumOverlapPts;i++)
//                    {
//                        int idx = tempPts[i].y;  
                        // mark overlapped points to one 
//                        tempPlotPts[idx]=1;
//                    }
                    
//                   CvPoint *pt_new_query = new CvPoint [NumbeofPts];
//                   CvPoint *pt_new_train= new CvPoint  [NumbeofPts];
                     
                     
                     //NumbeofPts=NumbeofPts-100;
                     //int size_=80;
//                     for(int i=0 ; i<NumbeofPts;i++)
//                         //  for(; i< 1;i++)
//                     {
                         
                         //CvPoint pt;
//                         pt_new_query[i].x = cornersv2[i].p[0]+160;
//                         pt_new_query[i].y = cornersv2[i].p[1]+120;
                         
//                         pt_new_train[i].x = nextPtsv2[i].p[0]+160;
//                         pt_new_train[i].y = nextPtsv2[i].p[1]+120;
                         
                         //pt.x= pt_new_train[i].x;
                         //pt.y= pt_new_train[i].y;
                         //cvCircle(img1, pt, 5, CV_RGB(0,255,0));
                         
//                     }

                     //int tempsize =0;
                     //Two_image= plot_two_imagesf(imgC,imgA, pt_new_query , pt_new_train , NumbeofPts);
//                     Two_image= plot_two_imagesf(imgC,imgA, pt_new_query , pt_new_train , NumbeofPts);
                   
                     /*
                    for (int y=0;y<corners.size();y++)
                    {
                        
                        float location1_x = (int)  nextPts[y].x;
                        float location1_y = (int)  nextPts[y].y;
                        
                        if  (tempPlotPts[y]==0)     // Plot new Points
                            cvCircle(imgA, cvPoint(location1_x, location1_y),3, CV_RGB(0, 255, 0), -1,8,0); 
                        if  (tempPlotPts[y]==1)     // overlapped Points 
                            cvCircle(imgA, cvPoint(location1_x, location1_y),3, CV_RGB(255,0, 0),  -1,8,0); 
                    }*/
#endif                    
                    // add update corners //
//                    delete [] pt_new_query;
//                    delete [] pt_new_train;
                     
//                      tempCorners.clear();
//                    tempCorners=nextPts;
//                    tempPts.clear();
                    
#ifdef PlotFeatureTracking
//                    delete [] tempPlotPts;
#endif              
                    imgB= cvCloneImage(frame);

                }
                firstcapture=0;
                
                //if (SkipThisFrame !=1)
                //   {
                                      //      cout<<"save this freame "<<endl;
                 // }
                 //else
                 // {
                 //   cout<<"skip this frame "<<endl;
                //}
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
IplImage* plot_two_imagesf(IplImage *IGray, IplImage *IGray1, CvPoint *corners1, CvPoint *corners3, int vector_index)
{
    CvPoint newmatched;       
    int  ttw  =  IGray->width+ IGray->width;
    int  ttl  =  IGray->height;
    IplImage* Imagedisplay  = cvCreateImage(cvSize(2*IGray->width,IGray->height), IGray->depth, IGray->nChannels );
    for (int i=0; i<ttl;i++)
    {
        for (int j=0; j<ttw; j++)
        {
            if (j< IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray,i,j));
            }
            if (j>IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray1,i,j-(IGray1->width)));
            }
        }
    }
    for (int i=0;i<vector_index; i++)
    {           
        newmatched.x= (int)(corners3[i].x+(IGray->width));
        newmatched.y= (int)(corners3[i].y);
        cvLine(Imagedisplay, 
               cvPoint( corners1[i].x-1, corners1[i].y ), 
               cvPoint( newmatched.x, newmatched.y ), 
               CV_RGB(256,256,256)
               );
        cvCircle(Imagedisplay, newmatched, 3, CV_RGB(0,255,0));
        cvCircle(Imagedisplay, corners1[i], 3, CV_RGB(0,255,0));
        
    }   
    //double rect2_width, rect2_height;
    //double rect1_width, rect1_height;
    
    //rect2_width = rect2[3].x - rect2[0].x;
    //rect2_height= rect2[3].y - rect2[0].y;
    
    //rect1_width = rect1[3].x - rect1[0].x;
    //rect1_height= rect1[3].y - rect1[0].y;
    
    
    //cvRectangle(Imagedisplay, cvPoint(rect2[0].x, rect2[0].y), cvPoint(rect2[0].x+rect2_width,rect2[0].y+rect2_height),
    //            cvScalar(0xff,0xff,0xff) );
    
    //for (int i=1;i<3;i++)
    //{
    
    //    cvLine(Imagedisplay, 
    //           cvPoint(rect1[0].x+ (IGray->width), rect1[0].y ), 
    //           cvPoint(rect1[1].x+ (IGray->width), rect1[1].y ), 
    //           CV_RGB(0,0,255)
    //           );
    //    cvLine(Imagedisplay, 
    //           cvPoint(rect1[1].x+ (IGray->width), rect1[1].y ), 
    //           cvPoint(rect1[3].x+ (IGray->width), rect1[3].y ), 
    //           CV_RGB(0,0,255)
    //           );
    //   cvLine(Imagedisplay, 
    //           cvPoint(rect1[2].x+ (IGray->width), rect1[2].y ), 
    //           cvPoint(rect1[3].x+ (IGray->width), rect1[3].y ), 
    //           CV_RGB(0,0,255)
    //           );
    //   cvLine(Imagedisplay, 
    //           cvPoint(rect1[0].x+ (IGray->width), rect1[0].y ), 
    //           cvPoint(rect1[2].x+ (IGray->width), rect1[2].y ), 
    //           CV_RGB(0,0,255)
    //           );
    
    //}
    
    
    return (Imagedisplay);
}
