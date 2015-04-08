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
//static double GLOBAL_error ;
//static double GLOBAL_error1 = 9999;
static double *GLOBAL_Rot;
static double *GLOBAL_Tc;
static double *GLOBAL_Kmatrix;
//static int sw =0;
//static int revert;


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
    Sing_camera_refine( num_points, points, projs ,UpdateR);  //add new update rotmatrix//
    
    free (GLOBAL_proj); free (GLOBAL_points);
    free (GLOBAL_Rot);  free (GLOBAL_Tc);
    free (GLOBAL_Kmatrix);
    
}

static void Sing_camera_refine(int num_points, v3_t *points, v2_t *projs, double*R )
{
    
    //double x[7] = {0.0, 0.0, 0.0, GLOBAL_Tc[0], GLOBAL_Tc[1], GLOBAL_Tc[2], GLOBAL_Kmatrix[0]};  
    double x[3] = {0.0, 0.0, 0.0};
    lmdif_driver2( CameraReprojectionError, 2 * num_points, 3 , x, 1.0e-12);     
    
    double Rnew[9];
    //double Tnew[3];
    double Rvec[3];  
    //double K_result[1];
    
    
    memcpy( Rvec, x ,  3 * sizeof(double));  
    //memcpy(Tnew,x+3,3*sizeof(double));
    //memcpy(K_result,x+6,1*sizeof(double));
    
    //double Kmatrix[9]= {K_result[0], 0, 0, 0, K_result[0],0,0,0,1};
    
    Sing_rot_update(GLOBAL_Rot, Rvec , Rnew);
    
    memcpy(R,Rnew, 9*sizeof(double));
    //memcpy(T,Tnew,3*sizeof(double));
    //memcpy(K,Kmatrix,9*sizeof(double));
    
}
static void CameraReprojectionError(const int *m, const int *n, 
                                        double *x, double *fvec, int *iflag) 
{
    int i;
    double error3 = 0.0;
    
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
//    printf(" %f \n",error3 );
    
}
static void CameraProjectPoint(double *aj, double *bi, 
                                   double *xij)
{

    double *w;
    //double *t;
    //double *k;
    /* Compute translation, rotation update */
    w = aj ;  // pick up w //
    //t = aj+3;
    //k = aj+6;
    
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


int find_projection_3x4_ransac(int num_pts, v3_t *points, v2_t *projs, 
                               double *P, 
                               int ransac_rounds, double ransac_threshold) 
{
    if (num_pts < 6) {
        printf("[find_projection_3x4_ransac] Error: need at least 6 points!\n");
        return -1;
    } else {
#define MIN_PTS 6
        // const int min_pts = 6;
        int *inliers = (int *) malloc(sizeof(int) * num_pts);
        int indices[MIN_PTS];
        int round, i, j;
        int max_inliers = 0;
        double max_error = 0.0;
        double Pbest[12];
        int num_inliers = 0, num_inliers_new = 0;
        v3_t *pts_final = NULL;
        v2_t *projs_final = NULL;
        double Plinear[12];
        
        double Rinit[9];
        double triangular[9], orthogonal[9];
        int neg, sign;
        
        double thresh_sq = ransac_threshold * ransac_threshold;
        double error = 0.0;
        
        int num_inliers_polished = 0;
        
        for (round = 0; round < ransac_rounds; round++) {
            v3_t pts_inner[MIN_PTS];
            v2_t projs_inner[MIN_PTS];
            double Ptmp[12];
            
            num_inliers = 0;
            for (i = 0; i < MIN_PTS; i++) {
                int redo = 0;
                int idx;
                int redo_count = 0;
                
                do {
                    if (redo_count > 50000) {
                        free(inliers);
                        return -1;
                    }
                    
                    idx = rand() % num_pts;
                    
                    redo = 0;
                    for (j = 0; j < i; j++) {
                        if (idx == indices[j]) {
                            redo = 1;
                            break;
                        } else if (Vx(projs[idx]) == Vx(projs[indices[j]]) && 
                                   Vy(projs[idx]) == Vy(projs[indices[j]])) {
                            redo = 1;
                        }
                    }
                    
                    redo_count++;
                } while(redo);
                
                indices[i] = idx;
                pts_inner[i] = points[idx];
                projs_inner[i] = projs[idx];
            }
            
            /* Solve for the parameters */
            find_projection_3x4(MIN_PTS, pts_inner, projs_inner, Ptmp);
            
#if 1
            /* Fix the sign on the P matrix */
            memcpy(Rinit + 0, Ptmp + 0, 3 * sizeof(double));
            memcpy(Rinit + 3, Ptmp + 4, 3 * sizeof(double));
            memcpy(Rinit + 6, Ptmp + 8, 3 * sizeof(double));
            
            dgerqf_driver(3, 3, Rinit, triangular, orthogonal);	    
            
            /* Check the parity along the diagonal */
            neg = 
            (triangular[0] < 0.0) + 
            (triangular[4] < 0.0) + 
            (triangular[8] < 0.0);
            
            if ((neg % 2) == 1) {
                sign = -1;
            } else {
                sign = 1;
            }
#endif
            
            /* Count the number of inliers */
            
            error = 0.0;
            for (i = 0; i < num_pts; i++) {
                double pt[4] = { Vx(points[i]), 
                    Vy(points[i]), 
                    Vz(points[i]), 1.0 };
                double pr[3];
                double dx, dy, dist;
                
                matrix_product341(Ptmp, pt, pr);
                
                /* Check cheirality */
                // EDIT!!!
                if (sign * pr[2] > 0.0) 
                    continue;
                
                // EDIT!!!
                pr[0] /= -pr[2];
                pr[1] /= -pr[2];
                
                dx = pr[0] - Vx(projs[i]);
                dy = pr[1] - Vy(projs[i]);
                
                dist = dx * dx + dy * dy;
                
                if (dist < thresh_sq) {
                    inliers[num_inliers] = i;
                    num_inliers++;
                    error += dist;
                }
                
            }
            
            
            if (num_inliers > max_inliers) {
                memcpy(Pbest, Ptmp, sizeof(double) * 12);
                max_error = error;
                max_inliers = num_inliers;
            }
        }
        
        memcpy(P, Pbest, sizeof(double) * 12);
        
        printf("[find_projection_3x4_ransac] num_inliers = %d (out of %d)\n",
               max_inliers, num_pts);	
        printf("[find_projection_3x4_ransac] Avg error = %0.3f\n",
               (max_error/num_pts));
        printf("[find_projection_3x4_ransac] error = %0.3f\n", 
               sqrt(max_error / max_inliers));
        
        if (max_inliers < 6) {
            printf("[find_projection_3x4_ransac] "
                   "Too few inliers to continue.\n");
            
            free(inliers);
            
            return -1;
        }
        
        /* Do the final least squares minimization */
        
#if 1
        /* Fix the sign on the P matrix */
        memcpy(Rinit + 0, Pbest + 0, 3 * sizeof(double));
        memcpy(Rinit + 3, Pbest + 4, 3 * sizeof(double));
        memcpy(Rinit + 6, Pbest + 8, 3 * sizeof(double));
        
        dgerqf_driver(3, 3, Rinit, triangular, orthogonal);	    
        
        /* Check the parity along the diagonal */
        neg = 
        (triangular[0] < 0.0) + 
        (triangular[4] < 0.0) + 
        (triangular[8] < 0.0);
        
        if ((neg % 2) == 1) {
            sign = -1;
        } else {
            sign = 1;
        }
#endif
        
        num_inliers = 0;
        pts_final = (v3_t *) malloc(sizeof(v3_t) * max_inliers);
        projs_final = (v2_t *) malloc(sizeof(v2_t) * max_inliers);
        
        for (i = 0; i < num_pts; i++) {
            double pt[4] = { Vx(points[i]), 
                Vy(points[i]), 
                Vz(points[i]), 1.0 };
            
            double pr[3];
            double dx, dy, dist;
            
            matrix_product341(Pbest, pt, pr);
            
            /* Check cheirality */
            // EDIT!!!
            if (sign * pr[2] > 0.0) 
                continue;
            
            // EDIT!!!
            pr[0] /= -pr[2];
            pr[1] /= -pr[2];
            
            dx = pr[0] - Vx(projs[i]);
            dy = pr[1] - Vy(projs[i]);
            dist = dx * dx + dy * dy;
            if (dist < thresh_sq) 
            {
                pts_final[num_inliers] = points[i];
                projs_final[num_inliers] = projs[i];
                num_inliers++;
            }
        }
        
        if (num_inliers != max_inliers) {
            printf("[find_projection_3x4_ransac] Error! There was a miscount "
                   "somewhere: (%d != %d)\n", num_inliers, max_inliers);
        }
        
        find_projection_3x4(max_inliers, pts_final, projs_final, Plinear);
        
#if 1
        /* Fix the sign on the P matrix */
        memcpy(Rinit + 0, Plinear + 0, 3 * sizeof(double));
        memcpy(Rinit + 3, Plinear + 4, 3 * sizeof(double));
        memcpy(Rinit + 6, Plinear + 8, 3 * sizeof(double));
        
        dgerqf_driver(3, 3, Rinit, triangular, orthogonal);	    
        
        /* Check the parity along the diagonal */
        neg = 
        (triangular[0] < 0.0) + 
        (triangular[4] < 0.0) + 
        (triangular[8] < 0.0);
        
        if ((neg % 2) == 1) {
            sign = -1;
        } else {
            sign = 1;
        }
#endif
        
        for (i = 0; i < num_pts; i++) {
            double pt[4] = 
            { Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0 };
            double pr[3];
            double dx, dy, dist;
            
            matrix_product341(Plinear, pt, pr);
            
            // EDIT!!!
            if (sign * pr[2] > 0.0)
                continue;
            
            // EDIT!!!
            pr[0] /= -pr[2];
            pr[1] /= -pr[2];
            
            dx = pr[0] - Vx(projs[i]);
            dy = pr[1] - Vy(projs[i]);
            
            dist = dx * dx + dy * dy;
            
            if (dist < thresh_sq) {
                num_inliers_new++;
            }
        }
        
        if (num_inliers_new < max_inliers) 
        {
            printf("[find_projection_3x4_ransac] Reverting to old solution\n");
            memcpy(Plinear, Pbest, 12 * sizeof(double));
    	}
        printf("Best matrix (pre-opt):\n");
        matrix_print(3, 4, Plinear);
        error = 0.0;
        for (i = 0; i < max_inliers; i++) {
            double pt[4] = 
            { Vx(pts_final[i]), Vy(pts_final[i]), Vz(pts_final[i]), 1.0 };
            double pr[3];
            double dx, dy, dist;
            
            matrix_product341(Plinear, pt, pr);
            pr[0] /= pr[2];
            pr[1] /= pr[2];
            
            dx = pr[0] - Vx(projs_final[i]);
            dy = pr[1] - Vy(projs_final[i]);
            
            dist = dx * dx + dy * dy;
            
            error += dist;
        }
        
        printf("Old error: %0.3e\n", sqrt(error / max_inliers));
        
        /* Polish the result */
        if (max_inliers >= 6) {
            int num_inliers_polished = 0;
            find_projection_3x4_nonlinear(max_inliers, pts_final, projs_final,
                                          Plinear, P);
            
#if 1
            /* Fix the sign on the P matrix */
            memcpy(Rinit + 0, P + 0, 3 * sizeof(double));
            memcpy(Rinit + 3, P + 4, 3 * sizeof(double));
            memcpy(Rinit + 6, P + 8, 3 * sizeof(double));
            
            dgerqf_driver(3, 3, Rinit, triangular, orthogonal);	    
            
            /* Check the parity along the diagonal */
            neg = 
            (triangular[0] < 0.0) + 
            (triangular[4] < 0.0) + 
            (triangular[8] < 0.0);
            
            if ((neg % 2) == 1) {
                sign = -1;
            } else {
                sign = 1;
            }
#endif
            
            /* Check that the number of inliers hasn't gone down */
            num_inliers_polished = 0;
            for (i = 0; i < num_pts; i++) {
                double pt[4] = 
                { Vx(points[i]), Vy(points[i]), Vz(points[i]), 1.0 };
                double pr[3];
                double dx, dy, dist;
                
                matrix_product341(P, pt, pr);
                
                // EDIT!!!
                if (sign * pr[2] > 0.0)
                    continue;
                
                // EDIT!!!
                pr[0] /= -pr[2];
                pr[1] /= -pr[2];
                
                dx = pr[0] - Vx(projs[i]);
                dy = pr[1] - Vy(projs[i]);
                
                dist = dx * dx + dy * dy;
                
                if (dist < thresh_sq) {
                    num_inliers_polished++;
                }
            }
            
            if (num_inliers_polished < max_inliers) {
                printf("Decreased number of inliers (%d < %d), reverting\n",
                       num_inliers_polished, max_inliers);
                
                memcpy(P, Plinear, sizeof(double) * 12);		
            }
        } 
        else 
        {
            memcpy(P, Plinear, sizeof(double) * 12);
        }
        
        printf("Best matrix (post-opt):\n");
        matrix_print(3, 4, P);
        
        error = 0.0;
        for (i = 0; i < max_inliers; i++) {
            double pt[4] = 
            { Vx(pts_final[i]), Vy(pts_final[i]), Vz(pts_final[i]), 1.0 };
            double pr[3];
            double dx, dy, dist;
            
            matrix_product341(P, pt, pr);
            
            // EDIT!!!
            pr[0] /= -pr[2];
            pr[1] /= -pr[2];
            
            dx = pr[0] - Vx(projs_final[i]);
            dy = pr[1] - Vy(projs_final[i]);
            
            dist = dx * dx + dy * dy;
            
            error += dist;
        }
        
        printf("New error: %0.3e\n", sqrt(error / max_inliers));
        
        free(inliers);
        free(pts_final);
        free(projs_final);
        
        return max_inliers;
    }
#undef MIN_PTS
} 
static int global_num_pts;
static v3_t *global_points;
static v2_t *global_projs;

int find_projection_3x4_nonlinear(int num_pts, v3_t *points, v2_t *projs, 
                                  double *Pin, double *Pout) 
{
    if (num_pts < 6) {
        printf("[find_projection_3x4_nonlinear] Need at least 6 points!\n");
        return -1;
    } else {
        int num_eqns = 2 * num_pts;
        int num_vars = 11;
        double x[11];
        
        global_num_pts = num_pts;
        global_points = points;
        global_projs = projs;
        // Pin[11]=1.0;
        memcpy(x, Pin, sizeof(double) * 11);
        
        //lmdif_driver(projection_residual, num_eqns, num_vars, x, 1.0e-6);
        lmdif_driver(projection_residual, num_eqns, num_vars, x, 1.0e-12);
        memcpy(Pout, x, sizeof(double) * 11);
        Pout[11] = 1.0;
        
        return 0;
    }
}
static void projection_residual(const int *m, const int *n, double *x, 
                                double *fvec, double *iflag) 
{
    int i;
    
    double P[12];
    memcpy(P, x, sizeof(double) * 11);
    P[11] = 1.0;
    
    for (i = 0; i < global_num_pts; i++) {
        double pt[4] = { Vx(global_points[i]), 
            Vy(global_points[i]), 
            Vz(global_points[i]), 1.0 };
        
        double pr[3];
        double dx, dy;
        
        matrix_product341(P, pt, pr);
        // EDIT!!
        pr[0] /= -pr[2];
        pr[1] /= -pr[2];
	    
        dx = pr[0] - Vx(global_projs[i]);
        dy = pr[1] - Vy(global_projs[i]);
        
        fvec[2 * i + 0] = dx;
        fvec[2 * i + 1] = dy;
    }
}

void CameraParameter_Process(double*P, double *Rinit, double *tinit,double *K/* frame number*/ )
{
    double KRinit[9], Kinit[9];
    //, Rinit[9], tinit[3];
    memcpy(KRinit + 0, P + 0, 3 * sizeof(double));
    memcpy(KRinit + 3, P + 4, 3 * sizeof(double));
    memcpy(KRinit + 6, P + 8, 3 * sizeof(double));
    dgerqf_driver(3, 3, KRinit, Kinit, Rinit);	    
    /* We want our intrinsics to have a certain form */
    FixIntrinsics(P, Kinit, Rinit, tinit);
    matrix_scale(3, 3, Kinit, 1.0 / Kinit[8], Kinit);
    //cout<<"[FindAndVerifyCamera] Estimated intrinsics:\n"<<endl;
    //cout<<"K_matrix"<<endl;
    printf("kmatrix /n");
    matrix_print(3, 3, Kinit);

    // double Rigid[12] = 
    // { 
    // Rinit[0], Rinit[1], Rinit[2], tinit[0],
    // Rinit[3], Rinit[4], Rinit[5], tinit[1],
    // Rinit[6], Rinit[7], Rinit[8], tinit[2] 
    //                };
    //                int num_behind = 0;
    //                for (int j = 0; j < num_pts_index; j++)
    //                {
    //                    double p[4] = 
    //                    { 
    //                        Vx(point_solve[j]), 
    //                        Vy(point_solve[j]),
    //                        Vz(point_solve[j]), 1.0 
    //                    };
    //                    double q[3], q2[3];
    //                    matrix_product(3, 4, 4, 1, Rigid, p, q);
    //                    matrix_product331(Kinit, q, q2);
    //                    
    //                    double pimg[2] = { -q2[0] / q2[2], -q2[1] / q2[2] };
    //                    double diff = 
    //                    (pimg[0] - Vx(projs_solve[j])) * 
    //                    (pimg[0] - Vx(projs_solve[j])) + 
    //                    (pimg[1] - Vy(projs_solve[j])) * 
    //                    (pimg[1] - Vy(projs_solve[j]));
    //                    diff = sqrt(diff);
    //                    //    cout<<diff<<endl;
    //                    //   cout << pimg[0]<<" "<<pimg[1]<<" "<<projs_solve[j].p[0]<<" "<<projs_solve[j].p[1]<<endl;
    //                }
    /*double camera_new_f = 0.5 * (Kinit[0] + Kinit[4]);*/
    // double Knew[9] = 
    // { 
    //     1200.0,0,0,
    //     0,1200.0,0,
    //     0,0,1
    // };
    
    K[0] =  0.5*(Kinit[0]+Kinit[4]);  K[1] =  0;  K[2] =  0;
    K[3]=0;                           K[4] = 0.5*(Kinit[0]+Kinit[4]); K[5]=0;
    K[6]=0;                           K[7]=0;                         K[8]=1;
    //    {
    //               0.5*(Kinit[0]+Kinit[4]),0,0,
    //               0,0.5*(Kinit[0]+Kinit[4]),0,
    //               0,0,1
    //               };
    
    double camera_newt[3];
    double camera_newt_1[3];
    camera_newt_1[0]=tinit[0];
    camera_newt_1[1]=tinit[1];
    camera_newt_1[2]=tinit[2];
    
    matrix_transpose_product(3, 3, 3, 1, Rinit, tinit, camera_newt);
    matrix_scale(3, 1, camera_newt, -1.0, tinit);
    matrix_print(3, 3, Rinit);
    matrix_print(3, 1, tinit);
    
    
}
void FixIntrinsics(double *P, double *K, double *R, double *t) 
{
    /* Check the parity along the diagonal */
    int neg = (K[0] < 0.0) + (K[4] < 0.0) + (K[8] < 0.0);
    
    /* If odd parity, negate the instrinsic matrix */
    if ((neg % 2) == 1)
    {
        matrix_scale(3, 3, K, -1.0, K);
        matrix_scale(3, 4, P, -1.0, P);
    }
    
    /* Now deal with case of even parity */
    double fix[9];
    matrix_ident(3, fix);
    double tmp[9], tmp2[12];
    
    if (K[0] < 0.0 && K[4] < 0.0) 
    {
        fix[0] = -1.0;
        fix[4] = -1.0;
    } 
    else if (K[0] < 0.0) 
    {
        fix[0] = -1.0;
        fix[8] = -1.0;
    } 
    else if (K[4] < 0.0)
    {
        fix[4] = -1.0;
        fix[8] = -1.0;
    } 
    else 
    {
        /* No change needed */
    }
    
    matrix_product(3, 3, 3, 3, K, fix, tmp);
    memcpy(K, tmp, sizeof(double) * 3 * 3);
    
    double Kinv[9];
    matrix_invert(3, K, Kinv);
    
    matrix_product(3, 3, 3, 4, Kinv, P, tmp2);
    
    memcpy(R + 0, tmp2 + 0, sizeof(double) * 3);
    memcpy(R + 3, tmp2 + 4, sizeof(double) * 3);
    memcpy(R + 6, tmp2 + 8, sizeof(double) * 3);
    
    t[0] = tmp2[3];
    t[1] = tmp2[7];
    t[2] = tmp2[11];
}
