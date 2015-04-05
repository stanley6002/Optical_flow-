//
//  CameraPoseRefinement.c
//  Optical_flow 
//
//  Created by chih-hsiang chang on 4/3/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "CameraPoseRefinement.h"




#ifdef WIN32
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif /* WIN32 */



static int GLOBAL_num_pts;
static  v2_t *GLOBAL_proj;
static  v3_t *GLOBAL_points;
static double GLOBAL_error ;
static double GLOBAL_error1 = 9999;
static double *GLOBAL_Rot;
static double *GLOBAL_Tc;
static double *GLOBAL_Kmatrix;
static int sw =0;
static int revert;


void CameraRotRefine(int num_points, v3_t *points, v2_t *projs , double *R, double *T, double*Kmatrix, double* UpdateR)
{

    GLOBAL_num_pts =  num_points;
    GLOBAL_proj    =  malloc(sizeof(v2_t)*num_points);
    GLOBAL_points  =  malloc(sizeof(v3_t)*num_points);
    GLOBAL_Rot     =  malloc(sizeof(double)*9);
    GLOBAL_Tc      =  malloc(sizeof(double)*3);
    GLOBAL_Kmatrix =  malloc(sizeof(double)*9);
  
    
    
    memcpy(GLOBAL_proj, projs, sizeof(v2_t)*num_points );
    memcpy(GLOBAL_points, points, sizeof(v3_t)*num_points );
    memcpy(GLOBAL_Rot ,R, sizeof(double)*9 );
    memcpy(GLOBAL_Tc ,T, sizeof(double)*3 );
    memcpy(GLOBAL_Kmatrix ,Kmatrix, sizeof(double)*9 );
    
    //double UpdatedR[9];
    Sing_camera_refine( num_points, points, projs , UpdateR);  //add new update rotmatrix//
    
    //matrix_print(3,3,UpdatedR);
    
    free (GLOBAL_proj); free (GLOBAL_points);
    free (GLOBAL_Rot);  free (GLOBAL_Tc);
    free (GLOBAL_Kmatrix);
    
}

static void Sing_camera_refine(int num_points, v3_t *points, v2_t *projs, double*R )
{
    
    //double x[6] = {0.0, 0.0, 0.0, GLOBAL_Tc[0],GLOBAL_Tc[1],GLOBAL_Tc[2]};  
    double x[3] = {0.0, 0.0, 0.0};  
    lmdif_driver2( CameraReprojectionError, 2 * num_points, 3 , x, 0.0000001);     
    
    double Rnew[9];
    double Tnew[3];
    //double k[1];
    double Rvec[3];  
    
    memcpy( Rvec, x ,  3 * sizeof(double));  
    Sing_rot_update(GLOBAL_Rot, Rvec , Rnew);
    
    memcpy(R,Rnew, 9*sizeof(double));
    //matrix_print(3,1,GLOBAL_Tc);
    //matrix_print(3,3,Rnew);

    
}
static void CameraReprojectionError(const int *m, const int *n, 
                                        double *x, double *fvec, int *iflag) 
{
    int i;
    double error3 = 0.0, error2 = 0.0;
    
    //double error_temp;
    for (i = 0; i < GLOBAL_num_pts; i++)
    {
        double pt[3] = {
            Vx(GLOBAL_points[i]), 
            Vy(GLOBAL_points[i]),
            Vz(GLOBAL_points[i]) 
                       };
        
        double proj[2], dx, dy;
        
        CameraProjectPoint (x /*updated rotation*/ , pt /*3D*/, proj);
        
        dx = Vx(GLOBAL_proj[i]) - proj[0];
        dy = Vy(GLOBAL_proj[i]) - proj[1];
        
        //printf(" %f %f %f %f \n",Vx(GLOBAL_proj[i]), Vy(GLOBAL_proj[i]), proj[0], proj[1]);
        
        error3 = sqrt(dx * dx + dy * dy);
       
        fvec[2 * i + 0] = dx;
        fvec[2 * i + 1] = dy;

            
    }
    
   
 
    printf(" %f \n",error3 );
    
//    if (*iflag == 0)
//    {
//        printf("  Round[%d]: RMS error = %0.8f [%0.8f]\n", 
//               GLOBAL_runs, sqrt(error / GLOBAL_num_pts),
//               error2 / GLOBAL_num_pts);        
//        GLOBAL_runs++;
//    }
}
static void CameraProjectPoint(double *aj, double *bi, 
                                   double *xij)
{

    double *w;
    //double *t;

    /* Compute translation, rotation update */
    w = aj ;  // pick up w //
    //t = aj+3;
   
    
    double R[9]={GLOBAL_Rot[0],GLOBAL_Rot[1],GLOBAL_Rot[2]
                ,GLOBAL_Rot[3],GLOBAL_Rot[4],GLOBAL_Rot[5]
                ,GLOBAL_Rot[6],GLOBAL_Rot[7],GLOBAL_Rot[8] };
    
    double Rnew[9];
    double tnew[3];
    
    double b_cam[3], b_proj[3];
    
    Sing_rot_update(R, w, Rnew);  // recover w to Rotation matrix //
    
    tnew[0] =GLOBAL_Tc[0]; 
    tnew[1] =GLOBAL_Tc[1]; 
    tnew[2] =GLOBAL_Tc[2]; 
    
    //tnew[0] =t[0]; 
    //tnew[1] =t[1]; 
    //tnew[2] =t[2]; 
    
    
    double b2[3];  
    //double K_up[9]={k[0],0,0,0,k[0],0,0,0,1};
    
    //K*[R|-Rtc]X//
    //K*R*[X-tc] //
    
    b2[0] = bi[0] - tnew[0];
    b2[1] = bi[1] - tnew[1];
    b2[2] = bi[2] - tnew[2];
    
    matrix_product331(Rnew, b2, b_cam);  
    
    matrix_product331(GLOBAL_Kmatrix, b_cam, b_proj);
    //matrix_product331(K_up, b_cam, b_proj);

    xij[0] = -b_proj[0] / b_proj[2];
    xij[1] = -b_proj[1] / b_proj[2];
    
}
void Sing_rot_update(double *R, double *w, double *Rnew) 
{
    double theta, sinth, costh, n[3];
    double nx[9], nxsq[9];
    double term2[9], term3[9];
    double tmp[9], dR[9];
    
    double ident[9] = 
	{ 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0 };
    
    theta = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    
    if (theta == 0.0) 
    {
        memcpy(Rnew, R, sizeof(double) * 9);
        return;
    }
    
    n[0] = w[0] / theta;
    n[1] = w[1] / theta;
    n[2] = w[2] / theta;
    
    nx[0] = 0.0;   nx[1] = -n[2];  nx[2] = n[1];
    nx[3] = n[2];  nx[4] = 0.0;    nx[5] = -n[0];
    nx[6] = -n[1]; nx[7] = n[0];   nx[8] = 0.0;
    
    matrix_product33(nx, nx, nxsq);
    
    sinth = sin(theta);
    costh = cos(theta);
    
    matrix_scale(3, 3, nx, sinth, term2);
    matrix_scale(3, 3, nxsq, 1.0 - costh, term3);
    
    matrix_sum(3, 3, 3, 3, ident, term2, tmp);
    matrix_sum(3, 3, 3, 3, tmp, term3, dR);
    
    matrix_product33(dR, R, Rnew);
}