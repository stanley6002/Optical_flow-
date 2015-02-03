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

void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(in[i].pt);
    }
}

//Takes an xy point and appends that to a keypoint structure
void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out)
{
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i)
    {
        out.push_back(KeyPoint(in[i], 1));
    }
}


void warpKeypoints(const Mat& H, const vector<KeyPoint>& in, vector<KeyPoint>& out)
{
    vector<Point2f> pts;
    keypoints2points(in, pts);
    vector<Point2f> pts_w(pts.size());
    Mat m_pts_w(pts_w);
    perspectiveTransform(Mat(pts), m_pts_w, H);
    points2keypoints(pts_w, out);
}
void matches2points(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
                    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
                    std::vector<Point2f>& pts_query)
{
    
    pts_train.clear();
    pts_query.clear();
    pts_train.reserve(matches.size());
    pts_query.reserve(matches.size());
    
    size_t i = 0;
    
    for (; i < matches.size(); i++)
    {
        
        const DMatch & dmatch = matches[i];
        
        pts_query.push_back(query[dmatch.queryIdx].pt);
        pts_train.push_back(train[dmatch.trainIdx].pt);
        
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
    
    IplImage *imgGrayA=0;
    IplImage *imgGrayB=0;
    
    Img_width=  320;
    Img_height= 240;
    
    imgA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    imgC = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 3);
    
    imgGrayA = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1);
    imgGrayB = cvCreateImage(cvSize(Img_width, Img_height), IPL_DEPTH_8U, 1); 
    
    
    //imgA = cvLoadImage("/P1_1.jpg", 3);
    //imgB = cvLoadImage("/P1_2.jpg", 3);
    
    //int grabFrameRet;
    vector<Point2f> tempCorners;
    do 
        if ((cameraFrame = cvQueryFrame(camCapture))) 
        {
            frame = skipNFrames(camCapture,10);
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
                   
                cv::Mat flow_mat;
                vector<Point2f> corners, nextPts;
                vector<uchar> status;
                vector<float> err;
                vector<Point2f> conersNew;
               
               cv::Mat FAST_outputImg;
               cv::Mat FAST_H_prev;       
               
               Ptr<FeatureDetector> FAST_detector;
               vector<KeyPoint> FAST_train_kpts, FAST_query_kpts;
               vector<Point2f> FAST_train_pts, FAST_query_pts;
               Ptr<DescriptorExtractor> FAST_descriptor;
               Mat FAST_train_desc, FAST_query_desc;
               
               cv::Ptr<cv::DescriptorMatcher> FAST_matcher;
               std::vector<cv::DMatch> FAST_matches;
               std::vector<unsigned char> FAST_match_mask;
               
               cv::FAST(imgGrayA, FAST_query_kpts, 70);
               cv::FAST(imgGrayB, FAST_train_kpts, 70);
               
               cout<<FAST_query_kpts.size()<<endl;
               FAST_detector = new cv::GridAdaptedFeatureDetector(new FastFeatureDetector(40, true),500, 2, 2);
               FAST_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
               
               FAST_descriptor = new cv::BriefDescriptorExtractor(16);
              
               FAST_detector->detect(imgGrayA, FAST_query_kpts);
               FAST_descriptor->compute(imgGrayA, FAST_query_kpts, FAST_query_desc);
              
               FAST_detector->detect(imgGrayB, FAST_train_kpts);
               FAST_descriptor->compute(imgGrayB, FAST_train_kpts, FAST_train_desc);
            
               std::vector<cv::KeyPoint> test_kpts;
               FAST_H_prev = cv::Mat::eye(3,3,CV_32FC1);
               
               warpKeypoints(FAST_H_prev.inv(), FAST_query_kpts, test_kpts);

               cv::Mat FAST_mask = windowedMatchingMask(test_kpts, FAST_train_kpts, 30, 30);
               FAST_matcher->match(FAST_query_desc, FAST_train_desc, FAST_matches);
               
               int i=0;
               for (; i< FAST_matches.size(); i++)
               {
                   int queryIdx = FAST_matches[i].queryIdx;
                   int trainIdx = FAST_matches[i].trainIdx;
                   
                   CvPoint pt;
                   CvPoint pt2;
                   pt.x = FAST_query_kpts[queryIdx].pt.x;
                   pt.y = FAST_query_kpts[queryIdx].pt.y;
                   
                   pt2.x = FAST_train_kpts[trainIdx].pt.x;
                   pt2.y = FAST_train_kpts[trainIdx].pt.y;
                   
                   //cvCircle(imgA, cvPoint(pt.x, pt.y), 1, CV_RGB(0, 255, 0), -1);
                   CvScalar s;
                   //s = cvGet2D(imgA, location_x, location_y);
                   //s = cvGet2D(imgA,pt.y, pt.x);
                   cvCircle(imgA, cvPoint(pt.x, pt.y), 1, CV_RGB(0, 255, 0), -1);
                   cout<<pt.x<<" "<<pt.y<<" "<<pt2.x<<" "<<pt2.y<<" "<<s.val[0]<<" "<<s.val[1]<<" "<<s.val[2]<<" "<<endl;
                   
                   
                   
                   //pts_query.push_back(query[dmatch.queryIdx].pt);
                   //pts_train.push_back(train[dmatch.trainIdx].pt);
                   
               }
               //std::vector<unsigned char> FAST_match_mask;
        
               //if(_1sttrack==true)
               //{
               //    goodFeaturesToTrack(imgGrayB, corners, 500, 0.001,10);
               //    cornerSubPix(imgGrayB, corners, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
               //    calcOpticalFlowPyrLK(imgGrayB, imgGrayA, corners, nextPts, status, err, Size(45,45));
                   _1sttrack=false;
                   tempCorners=nextPts;
               //}
               
               /*             
               if(_1sttrack==false)
               {
                  cout<<(int) tempCorners.size()<<endl;
                  goodFeaturesToTrack(imgGrayA, conersNew, 500, 0.001,10);
                  cornerSubPix(imgGrayA, conersNew, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 ));
                  
                  nextPts = tempCorners;
                  calcOpticalFlowPyrLK(imgGrayB, imgGrayA, tempCorners, corners, status, err, Size(45,45));
                  tempCorners.clear();
                  tempCorners=corners;
                   
                  cout<<(int) corners.size()<<endl;
               }*/
               
               //for ( int y = 0; y < (int) conersNew.size(); y++ ) 
               //{
               //    int location_x= conersNew[y].x;
               //    int location_y= conersNew[y].y;
               //    cout<<location_x<<" "<<location_y<<" "<<endl;
               
               //}
               
               cout<<"corners"<<endl;
                
               imgB= cvCloneImage(frame);
               
                 /*for ( int y = 0; y < (int) corners.size(); y++ ) 
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
               cout<<"second image"<<endl; 
               flow_mat.release();
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