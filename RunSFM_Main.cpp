

# include "RunSFM_Main.hpp"

double m_distortion_weight=0;

double ReprojectError( double *R, double* Tc, v3_t Pts, v2_t Projpts, double * Kmatrix)
{
    double error =0;
    double b2[3];  
    double b_cam[3];
    double b_proj[3];
    double xij[2];   
    //K*[R|-Rtc]X//
    //K*R*[X-tc] //
        
    b2[0] = Pts.p[0] - Tc[0];
    b2[1] = Pts.p[1] - Tc[1];
    b2[2] = Pts.p[2] - Tc[2];
        
    matrix_product331(R, b2, b_cam);  
        
    matrix_product331(Kmatrix, b_cam, b_proj);
        
    xij[0] = -b_proj[0] / b_proj[2];
    xij[1] = -b_proj[1] / b_proj[2];
        
    double dx = Projpts.p[0] - xij[0];
    double dy = Projpts.p[1] - xij[1];
    
    //cout<< Projpts.p[0]<<" "<<Projpts.p[1]<<" "<< xij[0]<<" "<<xij[1]<<endl;   
    error += sqrt(dx * dx + dy * dy);
    //cout<< Vx(Projpts[i])<<" "<<Vy(Projpts[i])<<" "<< xij[0]<<" "<<xij[1]<<" "<< sqrt(dx * dx + dy * dy) <<endl;
    //}
    return(error);
}

double RunSFM_Nviews_Main(int num_pts /*number of 3D pts */, 
                          int num_cameras, 
                          int start_camera,                   
                          vector<RotMat>  mtriRotmatrix,     /*camera rotation matrix*/
                          vector<TMat>    mtriTcmatrix,      /*camera translation matrix*/
                          vector<Kmat>    mtriKmatrix,       /*camera instrinstic matrix*/ 
                          vector<vector<v2_t> > mv2_location /*2D points location*/ , 
                          vector<vector<int> >  mv2_frame    /*frame number*/, 
                          vector<v3_t>& v3Pts                /*triangulation output*/)
{

    double *S = new double[num_cameras*num_cameras*7*7];
    double esp2;

    camera_params_t* CameraPara= new camera_params_t[num_cameras];
    
    for(int i=0;i< num_cameras;i++)
    {
        InitializedCameraParameters ( i,         
                                      mtriRotmatrix,     /*camera rotation matrix*/
                                      mtriTcmatrix,      /*camera translation matrix*/
                                      mtriKmatrix,
                                      CameraPara);
    }
    
    int Numofframe=0;
    for (int i=0;i<num_pts;i++)
    {
        //cout<< (int) mv2_frame[i].size()<<endl;
        Numofframe += (int) mv2_frame[i].size();
    }
    //cout<<"# of frame"<<Numofframe<<endl;
    char* vmask = new char[num_pts * num_cameras];
    double* projections = new double[2 * Numofframe];
    v3_t * sfm3Dpts  = new v3_t[num_pts];
    
    for (int i = 0; i < num_pts * num_cameras; i++)
    {
        vmask[i] = 0;
    }
   
    int arr_idx  = 0;
    int nz_count = 0;
    
    for (int i = 0; i < num_pts; i++) 
    {
       for (int j = 0; j < mv2_frame[i].size(); j++) 
        {
            int c = mv2_frame[i][j];
          
            vmask[nz_count * num_cameras + c] = 1;
            projections[2 * arr_idx + 0] = mv2_location[i][j].p[0];
            projections[2 * arr_idx + 1] = mv2_location[i][j].p[1];
            arr_idx++;
        }
        sfm3Dpts [nz_count] =  v3Pts[i];   /*3D points*/ 
        nz_count++;
    }
    
    double rep_error =0;
    bool EstimateFocal=1;
    bool UseFocalconstraints=0;  
    bool fix_focal =0; 
    bool EstimateDistortion=0;
    bool Explictcenter=1; 
    bool fix_points= 1; 
    bool UsePointConstraint= 0;
    bool remove_outliers=0;
    double error=0;
    
    //    for(int i=0;i< (int) v3Pts.size();i++)
    //    {
    //        double temperr=0;
    //        for(int j=0;j <  mv2_frame[i].size();j++)
    //        {
    //            int c = mv2_frame[i][j];
    //            v3_t _3dpts = v3Pts[i]; 
    //            v2_t reprojection;
    //            reprojection.p[0] = mv2_location[i][j].p[0];
    //            reprojection.p[1] = mv2_location[i][j].p[1];
    //            double R[9]; double T[3];  double K[9];
    //            memcpy(R,mtriRotmatrix[c].n,sizeof(double)*9);
    //            memcpy(T,mtriTcmatrix[c].n,sizeof(double)*3);
    //            memcpy(K,mtriKmatrix[c].n,sizeof(double)*9);
    //            //cout<<"frame number"<< c<<" ";
    //            double err = ReprojectError(R,T, _3dpts,reprojection,K);
    //            temperr += err;
    //                    
    //        }
    //        cout<<temperr/ (int) mv2_frame[i].size()  <<" "<<endl;
    //
    //    }
    //     cout<<error/ (int) v3Pts.size()<<endl;
    
     error=  RunSFM_Nviews( num_pts, num_cameras, start_camera, Numofframe, 
                                 CameraPara, sfm3Dpts ,
                                 vmask, projections,  
                                 EstimateFocal, UseFocalconstraints, fix_focal, 
                                 Explictcenter, fix_points, UsePointConstraint, 
                                 EstimateDistortion,
                                 nz_count , esp2, 
                                 S,  NULL,  NULL,  NULL,
                                 remove_outliers);
    delete [] vmask;
    delete [] projections;
    delete [] sfm3Dpts;
    delete [] CameraPara; 
    delete [] S;
    return(error);

}


double RunSFM_Nviews(int num_pts, int num_cameras, int start_camera, int Numofframe, 
                     camera_params_t *init_camera_params, v3_t* sfm3Dpts ,
                     char* vmask, double* projections,  
                     bool EstimateFocal, bool UseFocalconstraints,  bool fix_focal, 
                     bool Explictcenter, bool fix_points, bool UsePointConstraint, 
                     bool EstimateDistortion,
                     int nz_count ,double eps2, 
                     double *S, double *U, double *V, double *W,
                     bool remove_outliers)
{  
    int num_projections= num_projections = num_pts*num_cameras;
    v3_t *m_point_constraints_1= new v3_t[nz_count];
    double m_point_constraint_weight=0;
    
    //for(int i=0;i<num_cameras;i++)
    //{
    //    SetCameraConstraints(init_camera_params[i],0);
    //    SetFocalConstraint(init_camera_params[i]);
    //}
    
    //bool fixed_focal;
    //double m_use_point_constraints;   
    
    run_sfm(nz_count, num_cameras, start_camera , vmask , projections, 
            /*focal length estimatation ? 0 : 1*/  EstimateFocal   ,
            /* fix focal constant*/  fix_focal ,
            /* m_estimate_distortion ? 1 : 0 */  EstimateDistortion, 
            /* explict center*/  Explictcenter   ,
            /* initial camera parameters*/ init_camera_params    , 
            /*initial 3D points */ sfm3Dpts, 
            /*(m_use_constraints || m_constrain_focal) ? 1 : 0*/ UseFocalconstraints,
            /*(m_use_point_constraints) ?*/ UsePointConstraint ,
            /*m_point_constraints_1*/  m_point_constraints_1, 
            /*m_point_constraint_weight*/ m_point_constraint_weight,
            /*fix_points ? 1 : 0*/ fix_points, 
            /*m_optimize_for_fisheye*/0 , eps2, V, S, U, W);

             delete[] vmask;
             delete[] projections;

    //       for (int i = 0; i < num_pts; i++)
    //         {
    //          _3Dlocation[i].p[0]= nz_pts[i].p[0];
    //          _3Dlocation[i].p[1]= nz_pts[i].p[1];
    //          _3Dlocation[i].p[2]= nz_pts[i].p[2]; 
    //         }
   
    //        }
    //        
    //        if (!remove_outliers) break;
    //        
    //    } while (num_outliers > 0);
    
    
          delete [] m_point_constraints_1;
    //    delete [] remap;
    //      delete [] nz_pts;
    //    return dist_total / num_dists;
}
void SetCameraConstraints(camera_params_t params, bool _estimate_distortion)
{   
       CameraInfo cam;
   
        params.constrained[0] = cam.m_constrained[0];
        params.constrained[1] = cam.m_constrained[1];
        params.constrained[2] = cam.m_constrained[2];
        params.constrained[3] = cam.m_constrained[3];
        params.constrained[4] = cam.m_constrained[4];
        params.constrained[5] = cam.m_constrained[5];
        params.constrained[6] = cam.m_constrained[6];
        
        if (_estimate_distortion) {
            params.constrained[7] = true;
            params.constrained[8] = true;
        } else {
            params.constrained[7] = false;
            params.constrained[8] = false;
        }
        
        params.constraints[0] = cam.m_constraints[0];
        params.constraints[1] = cam.m_constraints[1];
        params.constraints[2] = cam.m_constraints[2];
        params.constraints[3] = cam.m_constraints[3];
        params.constraints[4] = cam.m_constraints[4];
        params.constraints[5] = cam.m_constraints[5];
        params.constraints[6] = cam.m_constraints[6];
        params.constraints[7] = 0.0;
        params.constraints[8] = 0.0;
        
        params.weights[0] = cam.m_constraint_weights[0];
        params.weights[1] = cam.m_constraint_weights[1];
        params.weights[2] = cam.m_constraint_weights[2];
        params.weights[3] = cam.m_constraint_weights[3];
        params.weights[4] = cam.m_constraint_weights[4];
        params.weights[5] = cam.m_constraint_weights[5];
        params.weights[6] = cam.m_constraint_weights[6];
        
        if (_estimate_distortion) 
        {
            params.weights[7] = m_distortion_weight;
            params.weights[8] = m_distortion_weight;
        } 
        else 
      {
            params.weights[7] = 200.0;
            params.weights[8] = 200.0;
      }
}

void SetFocalConstraint( camera_params_t params)
{
        params.constrained[6] = true;
        params.constraints[6] = params.f;
        params.weights[6] = 5;
}
void InitializedCameraParameters ( 
                                 int i,         
                                 vector<RotMat>  mtriRotmatrix,     /*camera rotation matrix*/
                                 vector<TMat>    mtriTcmatrix,      /*camera translation matrix*/
                                 vector<Kmat>    mtriKmatrix,
                                 camera_params_t* CameraPara
                                 )

{
         memcpy(CameraPara[i].R, mtriRotmatrix[i].n, 9*sizeof(double));
         memcpy(CameraPara[i].t, mtriTcmatrix[i].n, 3*sizeof(double));
         memcpy(CameraPara[i].K_known , mtriKmatrix[i].n, 9*sizeof(double));
}
