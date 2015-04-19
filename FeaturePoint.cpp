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
    //delete[] StackIndex;
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
void FeaturePts::CollectFeatureTrackProjectPts(int Previous_ptsize , v2_t* Current_pts, int FrameNum, int* StackIdx)
{
   
    for (int i=0;i<Previous_ptsize;i++)
    {
        if (StackIdx[i] != 0 )
        {
            //int index  = StackIndex[i];
            int index  = StackIdx[i];
            mv2_location[i].push_back(Current_pts[index]);    // add connected feature point to existing list and collect 3D->2d POINTS 
            mv2_frame[i].push_back(FrameNum);
            mv2ReprojectPts.push_back(Current_pts[index]);    // collect 2D reprojection pts 
            mv3ProjectionPts.push_back(m_3Dpts[i]);           // collect  
            
        }
    }
    
    // Update the number of reprojection points
    
    this->NumReproject = (int) mv3ProjectionPts.size();
     cout<< " Projection size "<<  mv3ProjectionPts.size()<<endl;
     cout<< " ReProjection size "<< mv2ReprojectPts.size()<<endl;
}
 
void FeaturePts::ConnectedVideoSequence(vector<v2_t> Previous_pts   /*previous frame*/, v2_t* Connected_pts /*current new frame*/,v2_t* Current_pts, int Numpts , int FrameNumber)
{
    int Previous_ptsize   =   (int)Previous_pts.size();
    int ConnectedPtsize   =   Numpts;
    
    // add frame number in here
    
    //int FrameNumber       = 2;    // second and third overlapped
    
    //this-> StackIndex=  new int [Previous_ptsize](); 
    int* StackIdx = new int [Previous_ptsize];
    int* tempPrevious = new int [Previous_ptsize];
    int* tempCurrent  = new int [ConnectedPtsize];
    
    memset(tempPrevious, 0, Previous_ptsize*sizeof(int));
    memset(tempCurrent,  0, ConnectedPtsize*sizeof(int));
    memset(StackIdx, 0, Previous_ptsize*sizeof(int));
    
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
                        int x_m =  (int) Connected_pts[j].p[0];
                        int y_m =  (int) Connected_pts[j].p[1];
                        
                        if (sqrt(((x-x_m)*(x-x_m))+((y-y_m)*(y-y_m)))<=1.414)
                        {
                            //StackIndex[i]=j;
                            StackIdx[i]=j;
                            tempCurrent[j] =NUM;
                            tempPrevious[i]=NUM;
                            break;
                      }
                }    
            }
        }
    }

    CreateFeatureTrack(tempCurrent, ConnectedPtsize, Connected_pts, Current_pts , FrameNumber);
    CollectFeatureTrackProjectPts(Previous_ptsize,Current_pts, FrameNumber, StackIdx);
   
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
    delete [] StackIdx;
}
void FeaturePts:: UpdatedFeatureTrack(vector<v2_t>& left_pts, vector<v2_t>& right_pts, vector<v3_t>& V3Dpts, int FrameNum)
{

    int size_= (int) mv2_frame.size();
    cout<< "after "<<size_<<endl;
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
    //cout<< left_pts.size()<<" "<<right_pts.size()<<endl;
}
void FeaturePts::CleanFeatureTrack()
{
    vector<v2_t> leftPtsEmpty;
    vector<v2_t> rightPtsEmpty;
    vector<v3_t> _3DptsEmpty;
    vector<vector<int> > mv2_frameEmpty;
    vector<vector<v2_t> > mv2_locationEmpty;
    
    vector<v2_t> mv2ReprojectPtsEmpty;
    vector<v3_t> mv3ProjectionPtsEmpty;
    
    m_leftPts.swap (leftPtsEmpty);
    m_rightPts.swap(rightPtsEmpty);
    m_3Dpts.swap(_3DptsEmpty);
    
    mv2_location.swap(mv2_locationEmpty);   // show 2D point locations
    mv2_frame.swap(mv2_frameEmpty);        // show frame list 

    mv2ReprojectPts.swap(mv2ReprojectPtsEmpty); 
    mv3ProjectionPts.swap(mv3ProjectionPtsEmpty);
    
    //this-> StackIndex= NULL;
}

void FeaturePts::PointRefinement(vector<v3_t> &  Tempv3Dpts, vector <bool> tempvector)
{
     int NumPts = (int) Tempv3Dpts.size();
   
   
     int shift_index=0;
        for(int i=0;i< NumPts; i++)
        {
            if(tempvector[i]== true)
            {
                int removal_index =i;
                removal_index -=  shift_index;
                Tempv3Dpts.erase(Tempv3Dpts.begin()+removal_index);
                mv2_frame.erase(mv2_frame.begin()+removal_index);
                mv2_location.erase(mv2_location.begin()+removal_index);
 
                shift_index++;
            }   
        }
       
       cout<<" after" <<mv2_frame.size()<<" "<<mv2_location.size() <<endl;
     
        _3DLocation.insert(_3DLocation.end(),Tempv3Dpts.begin(),Tempv3Dpts.end());
        m_3Dpts.swap(Tempv3Dpts);
}