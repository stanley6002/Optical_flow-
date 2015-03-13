//
//  matchingandtracking.cpp
//  Homography_Calibration
//
//  Created by chih-hsiang chang on 3/9/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "matchingandtracking.h"

 FAST_ :: FAST_(int level, IplImage* imgGrayA, IplImage* imgGrayB, Function Descriptor)
{
    
    FAST_::Level= level;    

     ImageGray1 = cvCloneImage(imgGrayA);
    
     ImageGray2 = cvCloneImage(imgGrayB);
    
    if (Descriptor == SURF_descriptor)
    { 
        Surf_activate=1;
    }
    else 
        Surf_activate=0;

}

void FAST_ :: FAST_tracking(std::vector<CvPoint2D32f>& match_query, std::vector<CvPoint2D32f>& match_train)
 {

     if(! Surf_activate)
     {
         cv::FAST(ImageGray1,  FAST_query_kpts,  Level);
         cv::FAST(ImageGray2,  FAST_train_kpts,  Level);

         FAST_descriptor = new cv::BriefDescriptorExtractor(64);
         
         FAST_descriptor->compute(ImageGray1,  FAST_query_kpts, FAST_query_desc);
         FAST_descriptor->compute(ImageGray2,  FAST_train_kpts,FAST_train_desc);
       
         FAST_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
              std::vector<cv::KeyPoint> test_kpts;
         FAST_H_prev = cv::Mat::eye(3,3,CV_32FC1);
         
         warpKeypoints(FAST_H_prev.inv(), FAST_query_kpts, test_kpts);
         cv::Mat FAST_mask = windowedMatchingMask(test_kpts, FAST_train_kpts, 30, 30);
         FAST_matcher->match(FAST_query_desc, FAST_train_desc, FAST_matche, FAST_mask);
            int i=0;
         for (; i< FAST_matche.size(); i++)
         //for (; i< FAST_matcher.size(); i++)
         {
             int queryIdx = FAST_matche[i].queryIdx;
             int trainIdx = FAST_matche[i].trainIdx;
             
             //int queryIdx = FAST_matcher[i].queryIdx;
             //int trainIdx = FAST_matcher[i].trainIdx;
             
             match_query.push_back(FAST_query_kpts[queryIdx].pt);
             match_train.push_back(FAST_train_kpts[trainIdx].pt);
             
         }
     }
     
     if(Surf_activate)
     {
         cv::FAST(ImageGray1,  FAST_query_kpts,  Level);
         cv::FAST(ImageGray2,  FAST_train_kpts,  Level);
         
         FAST_descriptor= new cv::SurfDescriptorExtractor(4,2);
         
         FAST_descriptor->compute(ImageGray1, FAST_query_kpts, FAST_query_desc);
         FAST_descriptor->compute(ImageGray2, FAST_train_kpts, FAST_train_desc);
         cv::BruteForceMatcher<cv::L2<float> > matcher;
         std::vector<cv::DMatch> FAST_matcher;
         matcher.match(FAST_query_desc, FAST_train_desc, FAST_matcher);
         
         //std::vector<std::vector<cv::DMatch> > matches;
             int i=0;
         for (; i< FAST_matcher.size(); i++)
          {

             int queryIdx = FAST_matcher[i].queryIdx;
             int trainIdx = FAST_matcher[i].trainIdx;
             
             match_query.push_back(FAST_query_kpts[queryIdx].pt);
             match_train.push_back(FAST_train_kpts[trainIdx].pt);
         
           }
     }
} 

 FAST_::~ FAST_()
{
    cvReleaseImage(&ImageGray1);
    cvReleaseImage(&ImageGray2);
}

void FAST_ :: warpKeypoints(const cv::Mat& H, const vector<cv::KeyPoint>& in, vector<cv::KeyPoint>& out)
{
    vector<cv::Point2f> pts;
    keypoints2points(in, pts);
    vector<cv::Point2f> pts_w(pts.size());
    cv::Mat m_pts_w(pts_w);
    
    
    perspectiveTransform(cv::Mat(pts), m_pts_w, H);
    
    
    points2keypoints(pts_w, out);
}

void FAST_ :: keypoints2points(const vector<cv::KeyPoint>& in, vector<cv::Point2f>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(in[i].pt);
    }
}

void FAST_ :: points2keypoints(const vector<cv::Point2f>& in, vector<cv::KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        
        out.push_back(cv::KeyPoint(in[i], 1));
    }
}

LKFeatures :: LKFeatures(IplImage* imgGrayA, IplImage* imgGrayB)
{

  int Numberofcorner=400;
  int threshold1=0.01;
  int Windowsize=7;
  int Wsize =16;
  ParametersInitialized(Numberofcorner, threshold1, Wsize);
  ImageGray1 = cvCloneImage(imgGrayA);    
  ImageGray2 = cvCloneImage(imgGrayB);
  LKFeaturesTracking();    

}
void LKFeatures:: ParametersInitialized (int Numberofcorner, int threshold1, int Wsize )
{
      NumberCorn = Numberofcorner;
      Threshold  = threshold1;
      W_size = Wsize;
      Windowsize =7;
}

void LKFeatures::  LKFeaturesTracking ()
{
 
 goodFeaturesToTrack(ImageGray1, corners, 400, 0.001, 10);
 cornerSubPix(ImageGray2, corners, cv::Size(7,7), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 20, 0.001 ));
 calcOpticalFlowPyrLK(ImageGray2, ImageGray1, corners, nextPts, status, err, cv::Size(30,30));

}
void LKFeatures ::  FeaturesMatched  (std::vector<CvPoint2D32f> &match_query, std::vector<CvPoint2D32f> &match_train)
{
    int size = (int) corners.size();
    for (int i=0;i< size; i++)
    {
        CvPoint2D32f pt_q;
        CvPoint2D32f pt_t;
       pt_q.x = corners[i].x;
       pt_q.y = corners[i].y; 
       pt_t.x = nextPts[i].x;
       pt_t.y = nextPts[i].y;
        
        match_query.push_back(pt_q);
        match_train.push_back(pt_t);
    }  
}
LKFeatures:: ~LKFeatures()
{  
 cvReleaseImage(&ImageGray1);
 cvReleaseImage(&ImageGray2);
}




