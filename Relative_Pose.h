//
//  Relative_Pose.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 4/1/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//


# include <vector>
# include <iostream>
# include "matrix.h"
# include "vector.h"
# include "matrix.h"
# include "F_matrix.h"
# include "FeaturePoint.h"

//#include "FeaturePoint.h"

using namespace std;

typedef struct 
{
    double n[9];
    
}  RotMat;

typedef  struct 
{
    
    double n[3];
    
} TMat;

class CameraPose
{
   public:
    
    vector <RotMat> mRcMatrix;
    vector <TMat>   mTcMatrix;
    
    inline void LoadRotcMatrix(double* R )
    {
        RotMat tempR;
        memcpy( tempR.n , R, 9* sizeof(double));
        mRcMatrix.push_back(tempR);
    }
    
    inline void LoadTcMatrix(double* T)
    {
        TMat tempT;
        memcpy( tempT.n , T, 3* sizeof(double));
        mTcMatrix.push_back(tempT);
    }
    
    inline void First2viewInitialization(double* R1, double* R_relative, double* T1, double* T_relative)
    {
        LoadRotcMatrix(R1);
        LoadRotcMatrix(R_relative);
        
        LoadTcMatrix(T1);
        LoadTcMatrix(T_relative);
    }
    
    inline void PopRotcMatrix(int i, double* R)
    {
        memcpy(R,mRcMatrix[i].n, 9*sizeof(double));
    }
    inline void PopTcMatrix(int i, double* T)
    {
        memcpy(T,mTcMatrix[i].n,3*sizeof(double));
    }
    
    // inline void LoadRelativePose(double* R_relative, double* T_relative)
    //  {
    //    LoadTcMatrix(T_relative);
    //    LoadRotcMatrix(R_relative);
    // }
   
    inline int SizeofPose()
    {   
        if (mRcMatrix.size() != mTcMatrix.size())
            
        {  
             cout<<"size inconsistent "<<endl;
             }
        
        return((int) mRcMatrix.size());
    }
   
    void Egomotion(EpipolarGeometry EG, FeaturePts FeaturePts);
    void PrintRotmatrix(int i);
    void PrintTmatrix(int i);
    void TwoDalighment(int NumofProject, double*Rot, double*trans, vector<v3_t> P__3DSolvedforparameters, 
                       vector<v2_t> P__2DSolvedforparameters, EpipolarGeometry EG);
    //inline void PopRotmatirx(double* R);
    //inline void PopTmatrix(double* T);
    CameraPose ();
    ~CameraPose ();
    
};
