/* 
 *  Copyright (c) 2008  Noah Snavely (snavely (at) cs.washington.edu)
 *    and the University of Washington
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

/* sfm-driver.h */
/* Driver for sfm routines */

#ifndef __sfm_h__
#define __sfm_h__

#ifdef __cplusplus
extern "C" {
#endif

#include "vector.h"

//#define NUM_CAMERA_PARAMS 9
//#define POLY_INVERSE_DEGREE 6

typedef struct {
    double R[9];     /* Rotation */
    double t[3];     /* Translation */
    double f;        /* Focal length */
    double K_known[9];  /* Intrinsics (if known) */
    double f_scale; /* Scale on focal length, distortion params */
} camera_params_t;

/* Compute an updated rotation matrix given the initial rotation (R)
 * and the correction (w) */
void rot_update(double *R, double *w, double *Rnew);
    
void sfm_project(camera_params_t *init, double *K, 
		 double *w, double *dt, double *b, double *p);

void sfm_project_rd(camera_params_t *init, double *K, double *k,
                    double *R, double *dt, double *b, double *p);
    
//v2_t sfm_project_final(camera_params_t *params, v3_t pt,
//		       int explicit_camera_centers, int undistort);

void run_sfm(int num_pts, int num_cameras, int ncons,
             char *vmask,
             double *projections,
             int est_focal_length,
             camera_params_t *init_camera_params,
             v3_t *init_pts, 
             int use_constraints, 
             int use_point_constraints,
             int fix_points,
             int NumIteration,
             double eps2,
             double *Vout,
             double *Sout,
             double *Uout, double *Wout);

/* Refine the position of a single camera */
//void camera_refine(int num_points, v3_t *points, v2_t *projs, 
//		   camera_params_t *params, int adjust_focal,
//                   int estimate_distortion);
//
/* Refine the position of a single camera, keeping the first fixed */
//void camera_refine_fix_free(int num_points, v3_t *points, 
//                            v2_t *projs0, v2_t *projs1,
//                            camera_params_t *params0, 
//                            camera_params_t *params1, double *H);
    
#ifdef __cplusplus
}
#endif

#endif /* __sfm_h__ */