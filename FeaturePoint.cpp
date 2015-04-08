//
//  FeaturePoint.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/27/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "FeaturePoint.h"
#include <math.h>

#define NUM -99999
using namespace std;   


FeaturePts::FeaturePts ()
{


    
}
FeaturePts::~FeaturePts ()
{
    delete[] StackIndex;
}

// create feature track with new upcoming frame //
void FeaturePts::CreateFeatureTrack(int* tempCurrent, int ConnectedPtsize, v2_t* Connected_pts, v2_t* Current_pts, int FrameNumber)
{
    for (int i =0;i<ConnectedPtsize ;i++)
    {
        if(tempCurrent[i] != NUM)
        {
            mv2_frame.push_back(vector<int>());
            int size_frame =(int)mv2_frame.size();
            
            //cout<<"size of frame list "<<size_frame<<endl;
            mv2_frame[size_frame-1].push_back(FrameNumber-1);
            mv2_frame[size_frame-1].push_back(FrameNumber);
            
            //initialized 
            mv2_location.push_back(vector<v2_t>());
            int size_2Dlocation =(int)mv2_location.size();
            
            mv2_location[size_2Dlocation-1].push_back(Connected_pts[i]);
            mv2_location[size_2Dlocation-1].push_back(Current_pts[i]);
        }
    }
}

// add connected feature points to existing tracks
// collect 
void FeaturePts::CollectFeatureTrackProjectPts(int Previous_ptsize , v2_t* Current_pts, int FrameNum)
{
   
    for (int i=0;i<Previous_ptsize;i++)
    {
        if (StackIndex[i] != 0 )
        {
            int index  = StackIndex[i];
            mv2_location[i].push_back(Current_pts[index]);    // add connected feature point to existing list and collect 3D->2d POINTS 
            mv2_frame[i].push_back(FrameNum);
            mv2ReprojectPts.push_back(Current_pts[index]);    // collect 2D reprojection pts 
            mv3ProjectionPts.push_back(m_3Dpts[i]);           // collect  
        }
    }
    
    // Update the number of reprojection points
    
    this->NumReproject = (int) mv3ProjectionPts.size();
    // cout<< " Projection size "<<  mv3ProjectionPts.size()<<endl;
    // cout<< " ReProjection size "<< mv2ReprojectPts.size()<<endl;
}
 
void FeaturePts::ConnectedVideoSequence(vector<v2_t> Previous_pts   /*previous frame*/, v2_t* Connected_pts /*current new frame*/,v2_t* Current_pts, int Numpts , int FrameNumber)
{
    int Previous_ptsize   =   (int)Previous_pts.size();
    int ConnectedPtsize   =   Numpts;
    
    // add frame number in here
    
    //int FrameNumber       = 2;    // second and third overlapped
    
    this-> StackIndex=  new int [Previous_ptsize](); 
    
    int* tempPrevious = new int [Previous_ptsize];
    int* tempCurrent  = new int [ConnectedPtsize];
    
    memset(tempPrevious, 0, Previous_ptsize*sizeof(int));
    memset(tempCurrent,  0, ConnectedPtsize*sizeof(int));
    
    
    //for (int i=0;i< Previous_ptsize;i++)
    //     StackIndex[i]=NUM;
    
    for (int i=0; i< Previous_ptsize; i++)
    {
        if (tempPrevious[i] != NUM && tempPrevious[i] != NUM  )
        {
             int x =  Previous_pts[i].p[0];
             int y =  Previous_pts[i].p[1];
            
            for (int j=0; j< ConnectedPtsize; j++)
            {              
                if (tempCurrent[j] != NUM && tempCurrent[j] != NUM  )
                    {
                        int x_m =  Connected_pts[j].p[0];
                        int y_m =  Connected_pts[j].p[1];
                        
                        if (sqrt(((x-x_m)*(x-x_m))+((y-y_m)*(y-y_m)))<=1.414)
                        {
                            StackIndex[i]=j;
                            
                            tempCurrent[j] =NUM;
                            tempPrevious[i]=NUM;
                            
                            break;
                      }
                }    
            }
        }
    }

    CreateFeatureTrack(tempCurrent, ConnectedPtsize, Connected_pts, Current_pts , FrameNumber);
    CollectFeatureTrackProjectPts(Previous_ptsize,Current_pts, FrameNumber);
   
       
//    for (int i=0;i<300;i++)
//    {
//        for (int j=0;j<mv2_frame[i].size();j++)
//        {
//            cout<< mv2_frame[i][j]<<" ";
//        }       
//        
//        cout<<endl;
//    }
//    
//    cout<<" "<<endl;
//    
//    for (int i=0;i<300;i++)
//    {
//        for (int j=0;j<mv2_location[i].size();j++)
//        {
//            cout<<mv2_location[i][j].p[0]<<" "<<mv2_location[i][j].p[1]<<" ";
//        }       
//        
//        cout<<endl;
//    }
    
    delete [] tempCurrent;
    delete [] tempPrevious;
}
void FeaturePts:: UpdatedFeatureTrack(vector<v2_t> left_pts, vector<v2_t> right_pts, vector<v3_t>V3Dpts, int FrameNum)
{

    int size_= (int) mv2_frame.size();
       for(int i=0;i<size_ ; i++)
         {
        int index = (int) mv2_frame[i].size()-1;
         if (mv2_frame[i][index]== FrameNum)
          {
            // pick up 2D and 3D points
          v2_t leftpt  =mv2_location[i][index-1];
          v2_t rightpt =mv2_location[i][index];
          v3_t V3pt    =m_3Dpts[i];
          left_pts.push_back(leftpt);
          right_pts.push_back(rightpt);    
          V3Dpts.push_back(V3pt);
          }
          
    }
    //cout<<left_pts.size()<<" "<<right_pts.size()<<" "<<V3Dpts.size()<<endl;
}
void FeaturePts::CleanFeatureTrack()
{
    vector<v2_t> leftPtsEmpty;
    vector<v2_t> rightPtsEmpty;
    vector<v3_t> _3DptsEmpty;
    
    m_leftPts.swap (leftPtsEmpty);
    m_rightPts.swap(rightPtsEmpty);
    m_3Dpts.swap(_3DptsEmpty);
}