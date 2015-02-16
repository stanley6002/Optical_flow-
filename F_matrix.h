#ifndef __included_F_matrix_h
#define __included_F_matrix_h

#include "vector.h"
//#include "SURF_feature_matching.h"
#include <vector>
#include "epipolar.h"
#include "typedefine.h"

enum Motion {MotionHomography,MotionRigid};

void F_matrix_process (int num_pts, v3_t* r_pt, v3_t* l_pt,double *F, int num_trial, int F_threshold, int essential,matched *refined_pts,int ith_pairs);
void pop_backpts_WI(v2_t*lpts, v2_t*rpts, matched *pts, int ith_pair);
void pop_backpts(v3_t*lpts, v3_t*rpts, matched *pts);
void pushback_Fmatrix(double* F, F_key_matrix *F_matrix,int ith_pair);
void push_backpts(v3_t *lpts, v3_t*rpts, matched *pts, int inlier_size, int ith_pairs);
void f_matching_analysis(int num_pts, v3_t* r_pt, v3_t* l_pt);
void  EstimateTransform(v2_t*lpts, v2_t*rpts, Motion MotionSelect ,int num_size,  int _round /*128 m_homography_rounds*/, 
                        int _homography_threshold/*6.0m_homography_threshold*/,double *H, double *K);
double align_horn(int n, v3_t *right_pts, v3_t *left_pts, 
                  double *R, double *T, 
                  double *Tout, double *scale, double *weight);
static int CountInliers(const v2_t* lpts, 
                        const v2_t* rpts, 
                        double *M, double thresh, std::vector<int> &inliers, int _size);
#endif;