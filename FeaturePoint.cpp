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

CameraPose::CameraPose()
{
}
CameraPose::~CameraPose()
{
}

void CameraPose::PrintRotmatrix(int i)
{
    double tempR[9];
    PopRotmatrix(i, tempR);
    matrix_print(3,3,tempR);
}
void CameraPose::PrintTmatrix(int i)
{
    double tempT[3];
    PopTmatrix(i,tempT);
    matrix_print(3,1,tempT);
}

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
            mv2_frame[size_frame-1].push_back(FrameNumber);
            mv2_frame[size_frame-1].push_back(FrameNumber+1);
            
            //initialized 
            mv2_location.push_back(vector<v2_t>());
            int size_2Dlocation =(int)mv2_location.size();
            //cout<<"size of feature  list "<<size_2Dlocation<<endl;
            //cout<<size_2Dlocation<<endl;
            mv2_location[size_2Dlocation-1].push_back(Connected_pts[i]);
            mv2_location[size_2Dlocation-1].push_back(Current_pts[i]);
        }
    }
}

// add connected feature points to existing tracks
void FeaturePts::CollectProjectPts(int Previous_ptsize , v2_t* Current_pts)
{
    // add connected feature point to existing list and collect 3D->2d POINTS 
    for (int i=0;i<Previous_ptsize;i++)
    {
        if (StackIndex[i] != NUM)
        {
            int index  = StackIndex[i];
            mv2_location[i].push_back(Current_pts[index]);
            mv2ReprojectPts.push_back(Current_pts[index]);   // collect 2D reprojection pts 
            mv3ProjectionPts.push_back(m_3Dpts[i]);          
        }
    }
    cout<< " Projection size "<< mv3ProjectionPts.size()<<endl;
    cout<< " ReProjection size "<< mv2ReprojectPts.size()<<endl;
}
 
void FeaturePts::ConnectedVideoSequence(vector<v2_t> Previous_pts   /*previous frame*/, v2_t* Connected_pts /*current new frame*/,v2_t* Current_pts, int Numpts)
{
    
  
    int Previous_ptsize   =   (int)Previous_pts.size();
    int ConnectedPtsize   =   Numpts;
    
    // add frame number in here
    
    int FrameNumber       = 3; 
    
    this-> StackIndex= new int [Previous_ptsize]; 
    
    int* tempPrevious = new int [Previous_ptsize];
    int* tempCurrent  = new int [ConnectedPtsize];
    
    memset(tempPrevious, 0, Previous_ptsize*sizeof(int));
    memset(tempCurrent,  0, ConnectedPtsize*sizeof(int));
    
    
    for (int i=0;i< Previous_ptsize;i++)
         StackIndex[i]=NUM;
    
    
    for (int i=0; i< Previous_ptsize; i++)
    {
        if (tempPrevious[i] !=NUM && tempPrevious[i] !=NUM  )
        {
            int x =  Previous_pts[i].p[0];
            int y =  Previous_pts[i].p[1];
            
            for (int j=0; j< ConnectedPtsize; j++)
            {              
                if (tempCurrent[j] !=NUM && tempCurrent[j] !=NUM  )
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
    CollectProjectPts(Previous_ptsize,Current_pts);
   
 
    for (int i=0;i<200;i++)
    {
        int m=(int) mv2_location[i].size();
        for(int j=0;j<m;j++)
        {
            cout<< mv2_location[i][j].p[0]<<" "<< mv2_location[i][j].p[1];
            cout<<" ";
        }
        cout<<endl;
    }
    
    delete [] tempCurrent;
    delete [] tempPrevious;
}






//void feature_track_main(v2_t* lrefined_pt, v2_t* rrefined_pt, int N, int sizeof_pt)
//void feature_track_main(int N, matched *pts, vector<v2_t>& _2DSolvedforparameters,vector<v3_t>&_3DSolvedforparameters,vector<int>&_2D_refinement_index,vector<int>&_2Daligment)
//{
//    int i;
//    int track_index;
//    bool frame_checked;
//    bool _2D_locationchecked;
//    int row = (int)v2_frame.size();
//    v3_t _P3DSolvedforparameters;
//    vector<int> index_stack;
//    //vector<v2_t>  Vector_2DSolvedforparameters;
//    //vector<v3_t>  Vector_3DSolvedforparameters;
//    
//    
//    for (i=0; i<row; i++)
//    {
//        int col =(int)v2_frame[i].size();
//        int Num_frame=N;
//        frame_checked=check_frame_num(Num_frame,i,col) ;   
//        if (frame_checked==true)
//        {
//            index_stack.push_back(i);
//            // _2DSolvedforparameters.push_back(v2_t());
//        }
//    }   
//    
//    bool *inx_vector= new bool[(int)index_stack.size()];
//    // new add this 
//    for ( i=0; i< index_stack.size();i++)
//    {
//        
//        inx_vector[i]=1;
//    }
//    //
//    for (int j=0;j<pts[0].L_pts.size();j++)
//    {
//        v2_t location /* frame right */, location_updated /* frame left */;
//        
//        location.p[0]= pts[0].L_pts[j].p[0];
//        location.p[1]= pts[0].L_pts[j].p[1];
//        location_updated.p[0]= pts[0].R_pts[j].p[0];
//        location_updated.p[1]= pts[0].R_pts[j].p[1];
//        
//        int check_stack_size =  index_stack.size();  
//        
//        for ( i=0; i< check_stack_size;i++)
//        {
//            int cheked_index2D   = index_stack[i];
//            //int cheked_index3D   = index_stack[i];
//            if(inx_vector[i]==1)
//            {
//                
//                _2D_locationchecked = check_2Dlocation(cheked_index2D, location);
//                
//            }
//            if  (_2D_locationchecked)
//            {
//                track_index=cheked_index2D;
//                inx_vector[i]=0;   // new add this 
//                break;
//            }
//        }
//        if  (_2D_locationchecked)
//        {
//            // int _indexChecklocation=int(v2_location[i].size())-1;
//            _2DSolvedforparameters.push_back(location_updated);
//            _P3DSolvedforparameters=_3Dlocation[track_index];
//            _3DSolvedforparameters.push_back(_P3DSolvedforparameters);
//            _2Daligment.push_back(track_index);
//            add_track(N,track_index,location_updated);
//            check_stack_size-=1;   // new add this 
//            // cout<<" v2_location.size() "<<(int)v2_location[track_index].size()<<endl;
//            // cout<< location.p[0]<<" "<<location.p[1]<<" "<<location_updated.p[0]<<" "<<location_updated.p[1]<<endl;
//        }
//        else
//        {
//            create_track(N,location,location_updated,_2D_refinement_index);
//        }
//        _2D_locationchecked=false;
//    }
//    delete []  inx_vector;
//    
//    index_stack.clear();
//    
//} 

//static void add_track(int N, int &track_index, v2_t & location_updated)
//{
//    int row_size;
//    row_size= (int) v2_location[track_index].size();
//    v2_location[track_index].push_back(location_updated);
//    v2_frame[track_index].push_back(N+1);
//    
//    
//}
//
//static void create_track(int N,v2_t &location, v2_t &location_updated, vector<int>&_2D_refinement_index) 
//{
//    //cout<<(int)v2_frame.size()<<endl;
//    //v2_frame.push_back(vector<int>());
//    v2_frame.push_back(vector<int>());
//    int size_frame =(int)v2_frame.size();
//    //cout<<size_frame<<endl;
//    v2_frame[size_frame-1].push_back(N);
//    v2_frame[size_frame-1].push_back(N+1);
//    // initialized 
//    v2_location.push_back(vector<v2_t>());
//    int size_2Dlocation =(int)v2_location.size();
//    //cout<<size_2Dlocation<<endl;
//    v2_location[size_2Dlocation-1].push_back(location);
//    v2_location[size_2Dlocation-1].push_back(location_updated);
//    _2D_refinement_index.push_back(size_2Dlocation-1);
//    _3Dlocation.push_back(v3_t());
//    
//    
//}
//
//bool check_frame_num(int &N /*frame number */,int &row ,int &col)
//{
//    int index= N;              /// given fram number increased by 1;
//    int col_index= col-1;      // given col size , but actual location decrease by 1 
//    bool result;
//    if (v2_frame[row][col_index]==index)
//        result=true;
//    else
//        result =false;
//    
//    return(result);
//}
//
//bool check_2Dlocation(int &i, v2_t location)
//{
//    bool result;
//    int indexChecklocation;
//    indexChecklocation=int(v2_location[i].size())-1;
//    
//    if (v2_location[i][indexChecklocation].p[0]==location.p[0] && v2_location[i][indexChecklocation].p[1]==location.p[1])
//    {
//        result=true;
//    }
//    else 
//        result=false;
//    
//    return(result);
//}
//
//void initializelist(int frame_num, matched * pts)
//{
//    int num_row=(int)pts[0].L_pts.size();     /// num_row for list
//    initialized_framelist(num_row);
//    initialized_2Dlocationlist(num_row, pts);
//    
//    //  print_framelist(v2_frame);
//    //  print_locationlist(v2_location);
//    //  cout<<" ";
//}
//
//static void initialized_2Dlocationlist(int N, matched *pts)
//{
//    int i;
//    v2_t L,R;
//    for (i=0;i<N;i++)
//    {
//        v2_location.push_back(vector<v2_t>());
//        _3Dlocation.push_back(v3_t());
//    }
//    int max_= int (pts[0].L_pts.size());
//    for (int i=0;i<max_;i++)
//    {
//        L.p[0]= pts[0].L_pts[i].p[0];
//        L.p[1]= pts[0].L_pts[i].p[1];
//        R.p[0]= pts[0].R_pts[i].p[0];
//        R.p[1]= pts[0].R_pts[i].p[1];
//        v2_location[i].push_back(L);
//        v2_location[i].push_back(R);   
//    }
//}
//
//static void print_framelist(vector<vector<int> >v2d)
//{
//    int i;
//    int row =(int)v2d.size();
//    for (i=0;i<row;i++)
//    {
//        int col =(int)v2d[i].size();
//        int j;
//        for (j=0;j<col;j++)
//        {
//            cout<<v2d[i][j]<<" ";
//            //<<" "<<v2d[i][j].p[1]<<" ";
//        }
//        cout<<endl;
//    }
//}
//
//static void initialized_framelist(int N)
//{
//    int i;
//    for (i=0;i<N;i++)
//    {
//        v2_frame.push_back(vector<int>());
//    }
//    for (i=0;i<N;i++)
//    {
//        v2_frame[i].push_back(0);
//        v2_frame[i].push_back(1);
//    }
//    //  print_list(v2_frame);
//}
//
//static void print_locationlist(vector<vector<v2_t> >v2d)
//{
//    
//    int i;
//    int j;
//    int row =(int)v2d.size();
//    for (i=0;i<row;i++)
//    {
//        int col =(int) v2d[i].size();
//        j= col;
//        for (j=0;j<col;j++)
//        {
//            cout<<v2d[i][j].p[0]<<" "<<v2d[i][j].p[1]<<" ";
//        }
//        cout<<endl;
//    }
//    
//}
//void Save_3DpointfromInitial(v3_t&V3D_pts, int i)
//{
//    
//    _3Dlocation[i].p[0]= V3D_pts.p[0];
//    _3Dlocation[i].p[1]= V3D_pts.p[1];
//    _3Dlocation[i].p[2]= V3D_pts.p[2];
//}
//void Print_3Dlocation()
//{
//    cout<<_3Dlocation[0].p[0]<<" "<<_3Dlocation[0].p[1]<<" "<<_3Dlocation[0].p[2]<<endl;
//    
//}
