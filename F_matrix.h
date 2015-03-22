#ifndef __included_F_matrix_h
#define __included_F_matrix_h

#include "vector.h"
//#include "SURF_feature_matching.h"

#include <vector>
#include "epipolar.h"
#include "typedefine.h"
#include "Opencvheaders.h"

#define CenterX(x) ((x)-IMAGE_WIDTH/2)
#define CenterY(x) ((x)-IMAGE_HEIGHT/2)

class EpipolarGeometry 
{
    
public:
    
    double R_relative[9];
    double t_relative[3];
    double K1matrix[9];
    double K2matrix[9];
    double FocusLength;
    double R1matrix[9];
    double t1matrix[3];

    int num_ofrefined_pts;
    int Ransac_threshold;
    float ApicalAngle;
    
    
    v3_t* l_pt;
    v3_t* r_pt;
    
    v2_t *lrefined_pt;
    v2_t *rrefined_pt;
    
    v3_t* _3Dpts;
  
    EpipolarGeometry(const std::vector<CvPoint2D32f> match_query , const std::vector<CvPoint2D32f> match_train,
           int num_pts,int num_trial_Fmatrix, int num_trial_relativepose , int FocusLength, int Ransac_threshold);
    
    void FindFundamentalMatrix();   // Find the fundamental matrix and push the refinded points
    void MainProcess();

    void FindRelativePose();
    void CenterizedFeaturePoint    (v2_t* lrefined_pt, v2_t* rrefined_pt, int num_ofrefined_pts);
    void InitializeIntrinsicMatrix (double* Kmatrix);
    
    void TwoviewTriangulation ();
    void FindApicalAngle (float MaxAngle);
    void InitializeFirstPmatrix(); 
    static float  Variance (v3_t* _3Dpts, const  float depth , const int size_);
    
    void PointRefinement();
    
    inline bool CheckCheirality(v3_t pt)
    {
        bool Cheirality=false;
        if(pt.p[2]>0)
            Cheirality=true;
            return(Cheirality);
    }
    
    IplImage*plot_two_imagesf(IplImage *IGray, IplImage *IGray1);
    
    inline bool SkipFrame()
    { 
        return (skipFrame);
    }
    
    ~ EpipolarGeometry ();
    
    
private:
    bool skipFrame;
    int num_trial_Fmatrix;
    int num_trial_relativepose;
   
    double Numofpts;
    int essential; // control f _matrix
    double Fmatrix[9];
    static void _3DdepthRefine (v3_t* _3Dpts, bool* tempvector, int num_ofrefined_pts);
    //IplImage* ImageGray1;
    //IplImage* ImageGray2;
    //bool Surf_activate ;
    //bool countFrameRest;
};


enum Motion {MotionHomography,MotionRigid};

void F_matrix_process (int num_pts, v3_t* r_pt, v3_t* l_pt,double *F, int num_trial, int F_threshold, int essential,matched *refined_pts,int ith_pairs);
void pop_backpts_WI(v2_t*lpts, v2_t*rpts, matched *pts, int ith_pair);
void pop_backpts(v3_t*lpts, v3_t*rpts, matched *pts);
void pushback_Fmatrix(double* F, F_key_matrix *F_matrix,int ith_pair);
void push_backpts(v3_t *lpts, v3_t*rpts, matched *pts, int inlier_size, int ith_pairs);
void f_matching_analysis(int num_pts, v3_t* r_pt, v3_t* l_pt);
void EstimateTransform(v2_t*lpts, v2_t*rpts, Motion MotionSelect ,int num_size,  int _round /*128 m_homography_rounds*/, 
                        int _homography_threshold/*6.0m_homography_threshold*/,double *H, double *K);

double align_horn(int n, v3_t *right_pts, v3_t *left_pts, 
                  double *R, double *T, 
                  double *Tout, double *scale, double *weight);
static int CountInliers(const v2_t* lpts, 
                        const v2_t* rpts, 
                        double *M, double thresh, std::vector<int> &inliers, int _size);
#endif;