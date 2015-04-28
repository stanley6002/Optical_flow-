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


void InitializeCameraParams(camera_params_t &camera);


double RunSFM_Nviews_Main(int num_pts /*number of 3D pts */, 
                          int num_cameras, 
                          int start_camera,                   
                          vector<RotMat>&  mtriRotmatrix,     /*camera rotation matrix*/
                          vector<TMat>&    mtriTcmatrix,      /*camera translation matrix*/
                          vector<Kmat>&    mtriKmatrix,       /*camera instrinstic matrix*/ 
                          vector<vector<v2_t> > mv2_location /*2D points location*/ , 
                          vector<vector<int> >  mv2_frame    /*frame number*/, 
                          vector<v3_t>& v3Pts                /*triangulation output*/);

double RunSFM_Nviews(int num_pts, int num_cameras, int start_camera, int Numofframe, 
                     camera_params_t *init_camera_params, v3_t* sfm3Dpts ,
                     char* vmask, double* projections,  
                     bool EstimateFocal, bool UseFocalconstraints, 
                     bool fix_points, bool UsePointConstraint,double eps2, 
                     double *S, double *U, double *V, double *W,
                     int NumIteration,bool remove_outliers);


void SetCameraConstraints(camera_params_t params, bool estimate_distoration); 
void SetFocalConstraint( camera_params_t params);
void InitializedCameraParameters ( 
                             int i,         
                             vector<RotMat>  mtriRotmatrix,     /*camera rotation matrix*/
                             vector<TMat>    mtriTcmatrix,      /*camera translation matrix*/
                             vector<Kmat>    mtriKmatrix,
                             camera_params_t* CameraPara
                                  );

#endif 