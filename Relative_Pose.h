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
} Kmat;
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
    vector <Kmat>   KMatrix;
    
    inline void InitializeFirstTwoKMatrix(double* K1Matrix, double*K2Matrix)
    {
        Kmat temp1 ;
        Kmat temp2;
        
        memcpy( temp1.n , K1Matrix, 9* sizeof(double));
        KMatrix.push_back(temp1);

        memcpy( temp2.n , K2Matrix, 9* sizeof(double));
        KMatrix.push_back(temp2);
    }
    
    inline void InitializeKMatrix (int Foucslength)
    {
        Kmat temp ;
        
        temp.n[0]= Foucslength,          temp.n[1]=0.0                , temp.n[2]= 0.0;
        temp.n[3]= 0.0                ,  temp.n[4]= Foucslength ,       temp.n[5]= 0.0;
        temp.n[6]= 0.0 ,                 temp.n[7]= 0.0               , temp.n[8]= 1.0;
    
        KMatrix.push_back(temp);
    }
    
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
    
    inline void PopKMattix(int i, double*K)
    {
         memcpy(K,KMatrix[i].n, 9*sizeof(double));
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
    
    void TwoDalighment( int NumofProject, double*Rot, double*trans, vector<v3_t> P__3DSolvedforparameters, 
                       vector<v2_t> P__2DSolvedforparameters, EpipolarGeometry EG, double *Tcmatrix);
  
    //inline void PopRotmatirx(double* R);
    //inline void PopTmatrix(double* T);
    
    CameraPose ();
    ~CameraPose ();
    
    private:
    
      void   deltavector(v2_t* _2Dpt,  v3_t* _3Dpt, double* Parametrvec_);
      double Euclidence_3D (v2_t _2Dpt, v3_t _3Dpt, double* Parameter_vec);
      void   DeltaVector_Ransac(v2_t* _2Dpt, v3_t* _3Dpt, int size_ , double *       Parameter_vec, int Ransac_runs, double error);

    
};
