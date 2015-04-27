#ifndef __RunSFM_Main_h__
#define __RunSFM_Main_h__

//#ifdef __cplusplus
//extern "C" {
//#endif

#include "vector.h"
#include "sfm.h"
#include <vector>
# include "Relative_Pose.h"
using namespace std;

#define NUM_CAMERA_PARAMS 9
#define POLY_INVERSE_DEGREE 6
double ReprojectError( double *R, double* Tc, v3_t Pts, v2_t Projpts, double * Kmatrix);
//typedef struct {
//    double R[9];     /* Rotation */
//    double t[3];     /* Translation */
//    double f;        /* Focal length */
//    double k[2];     /* Undistortion parameters */
//    double k_inv[POLY_INVERSE_DEGREE]; /* Inverse undistortion parameters */
//    char constrained[NUM_CAMERA_PARAMS];
//    double constraints[NUM_CAMERA_PARAMS];  /* Constraints (if used) */
//    double weights[NUM_CAMERA_PARAMS];      /* Weights on the constraints */
//    double K_known[9];  /* Intrinsics (if known) */
//    double k_known[5];  /* Distortion params (if known) */
//    char fisheye;            /* Is this a fisheye image? */
//    char known_intrinsics;   /* Are the intrinsics known? */
//    double f_cx, f_cy;       /* Fisheye center */
//    double f_rad, f_angle;   /* Other fisheye parameters */
//    double f_focal;          /* Fisheye focal length */
//    double f_scale, k_scale; /* Scale on focal length, distortion params */
//} camera_params_t;


void InitializeCameraParams(camera_params_t &camera);


//void InitializeCameraParams(camera_params_t &camera ,int i);
//double RunSFM(int num_pts, int num_cameras, int start_camera,
//              bool fix_points, camera_params_t *init_camera_params,
//              v3_t *init_pts,
//              /*std::vector<ImageKeyVector> &pt_views*/ v2_t* l_refined, v2_t* r_refined , double eps2, 
//              double *S, double *U, double *V, double *W,
//             bool remove_outliers);

//double RunSFM_2(int num_pts, int num_cameras, int start_camera,
//                bool fix_points, camera_params_t *init_camera_params,
//                v3_t *init_pts,
//                std::vector<ImageKeyVector> &pt_viewsv2_t* _1strefined ,v2_t* _2ndrefined,double eps2, 
//                double *S, double *U, double *V, double *W,
//                bool remove_outliers,v3_t *_3D);

double RunSFM_Nviews_Main(int num_pts /*number of 3D pts */, 
                          int num_cameras, 
                          int start_camera,                   
                          vector<RotMat>  mtriRotmatrix,     /*camera rotation matrix*/
                          vector<TMat>    mtriTcmatrix,      /*camera translation matrix*/
                          vector<Kmat>    mtriKmatrix,       /*camera instrinstic matrix*/ 
                          vector<vector<v2_t> > mv2_location /*2D points location*/ , 
                          vector<vector<int> >  mv2_frame    /*frame number*/, 
                          vector<v3_t>& v3Pts                /*triangulation output*/);

double RunSFM_Nviews(int num_pts, int num_cameras, int start_camera, int Numofframe, 
                     camera_params_t *init_camera_params, v3_t* sfm3Dpts ,
                     char* vmask, double* projections,  
                     bool EstimateFocal, bool UseFocalconstraints,  bool fix_focal, 
                     bool Explictcenter, bool fix_points, bool UsePointConstraint, 
                     bool EstimateDistortion,
                     int nz_count ,double eps2, 
                     double *S, double *U, double *V, double *W,
                     bool remove_outliers);

double RunSFM_Nviews(int num_pts, int num_cameras, int start_camera,
                     bool fix_points, camera_params_t *init_camera_params, double eps2, 
                     double *S, double *U, double *V, double *W,
                     bool remove_outliers);


//double RunSFM_Nviews_Main(int num_pts,int num_cameras);

//void   InvertDistortion_Bundle(int n_in, int n_out, double r0, double r1, 
//                      double *k_in, double *k_out);
//#ifdef __cplusplus
//}
//#endif


void SetCameraConstraints(camera_params_t params, bool estimate_distoration); 
void SetFocalConstraint( camera_params_t params);

 class CameraInfo {
 public:
 bool m_constrained[7];
 double m_constraints[7];
 double m_constraint_weights[7];
     
 CameraInfo() 
     {
         m_constrained[0] = false;
         m_constrained[1] = false;
         m_constrained[2] = false; 
         m_constrained[3] = false;
         m_constrained[4] = false;
         m_constrained[5] = false; 
         m_constrained[6] = false;

         m_constraints[0] = 0.0;
         m_constraints[1] = 0.0;
         m_constraints[2] = 0.0;
         m_constraints[3] = 0.0;
         m_constraints[4] = 0.0;
         m_constraints[5] = 0.0;
         m_constraints[6] = 0.0;

         m_constraint_weights[0] = 0.0;
         m_constraint_weights[1] = 0.0;
         m_constraint_weights[2] = 0.0;
         m_constraint_weights[3] = 0.0;
         m_constraint_weights[4] = 0.0;
         m_constraint_weights[5] = 0.0;
         m_constraint_weights[6] = 0.0;      
     }
};


void InitializedCameraParameters ( 
                             int i,         
                             vector<RotMat>  mtriRotmatrix,     /*camera rotation matrix*/
                             vector<TMat>    mtriTcmatrix,      /*camera translation matrix*/
                             vector<Kmat>    mtriKmatrix,
                             camera_params_t* CameraPara
                                  );

#endif 