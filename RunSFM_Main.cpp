

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
                          vector<RotMat>&  mtriRotmatrix,     /*camera rotation matrix*/
                          vector<TMat>&    mtriTcmatrix,      /*camera translation matrix*/
                          vector<Kmat>&    mtriKmatrix,       /*camera instrinstic matrix*/ 
                          vector<vector<v2_t> > mv2_location /*2D points location*/ , 
                          vector<vector<int> >  mv2_frame    /*frame number*/, 
                          vector<v3_t>& v3Pts                /*triangulation output*/)
{
    double error ;
    
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
    //        error += temperr;
    //    }
    //    cout<<"error before sfm "<<error/ (int) v3Pts.size()<<endl;
    
    for (int i = 0; i < num_cameras; i++) 
    {
        
        double t[3] =  {mtriTcmatrix[i].n[0],mtriTcmatrix[i].n[1],mtriTcmatrix[i].n[2]};
        cout<< "camera_center"<<endl; 
        matrix_print(3,1, t); 
        
    } 

    
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

    bool EstimateFocal=1;
    bool UseFocalconstraints=0;  
    bool fix_points= 0 ; 
    bool UsePointConstraint= 0;
    bool remove_outliers=1;
    int  NumIteration=7;
    
  

    
    run_sfm(num_pts, num_cameras, start_camera , vmask , projections, 
            /*focal length estimatation ? 0 : 1*/  EstimateFocal   ,
            /* initial camera parameters*/ CameraPara, 
            /*initial 3D points */ sfm3Dpts, 
            /*(m_use_constraints || m_constrain_focal) ? 1 : 0*/ UseFocalconstraints,
            /*(m_use_point_constraints) ?*/ UsePointConstraint ,
            /*fix_points ? 1 : 0*/ fix_points, NumIteration, esp2, NULL, S, NULL, NULL);
    
     
    int i=0;
    for (i = 0; i < num_cameras; i++) 
    {

        double K[9] =  { CameraPara[i].f, 0.0, 0.0, 
    			        0.0, CameraPara[i].f, 0.0,
                        0.0, 0.0, 1.0 };
        
        memcpy(mtriKmatrix[i].n,K, 9*sizeof(double));
        
        memcpy(mtriRotmatrix[i].n,CameraPara[i].R, 9*sizeof(double)); 
        memcpy(mtriTcmatrix[i].n,CameraPara[i].t, 3*sizeof(double) ); 
    
    } 
    if (! fix_points)
    {
        for(i=0;i< num_pts;i++)
            v3Pts[i]= sfm3Dpts[i];    
    }
    if(remove_outliers)
    {
        for (int i = 0; i < num_cameras; i++) 
        {
            
            double t[3] =  {mtriTcmatrix[i].n[0],mtriTcmatrix[i].n[1],mtriTcmatrix[i].n[2]};
            cout<< "camera_center"<<endl; 
            matrix_print(3,1, t); 
            
        } 
        //            error=0;
        //            for(int i=0;i< (int) v3Pts.size();i++)
        //            {
        //                double temperr=0;
        //                for(int j=0;j <  mv2_frame[i].size();j++)
        //                {
        //                    int c = mv2_frame[i][j];
        //                    v3_t _3dpts = v3Pts[i]; 
        //                    v2_t reprojection;
        //                    reprojection.p[0] = mv2_location[i][j].p[0];
        //                    reprojection.p[1] = mv2_location[i][j].p[1];
        //                    double R[9]; double T[3];  double K[9];
        //                    memcpy(R,mtriRotmatrix[c].n,sizeof(double)*9);
        //                    memcpy(T,mtriTcmatrix[c].n,sizeof(double)*3);
        //                    memcpy(K,mtriKmatrix[c].n,sizeof(double)*9);
        //                    //cout<<"frame number"<< c<<" ";
        //                    double err = ReprojectError(R,T, _3dpts,reprojection,K);
        //                    temperr += err;
        //                            
        //                }
        //                cout<<temperr/ (int) mv2_frame[i].size()  <<" "<<endl;
        //                error += temperr;
        //            }
        //             cout<<"error after sfm "<< error/ (int) v3Pts.size()<<endl;
    }
    

    
    
    delete [] vmask;
    delete [] projections;
    delete [] sfm3Dpts;
    delete [] CameraPara; 
    delete [] S;
    return(error);

}

void InitializedCameraParameters ( 
                                 int i,         
                                 vector<RotMat>  mtriRotmatrix,     /*camera rotation matrix*/
                                 vector<TMat>    mtriTcmatrix,      /*camera translation matrix*/
                                 vector<Kmat>    mtriKmatrix,
                                 camera_params_t* CameraPara
                                 )

{
         memcpy(CameraPara[i].R, mtriRotmatrix[i].n, 9*sizeof(double));   /*Initialized rotation matrix*/
         memcpy(CameraPara[i].t, mtriTcmatrix[i].n, 3*sizeof(double));    /*Initialized translation matrix*/
         memcpy(CameraPara[i].K_known , mtriKmatrix[i].n, 9*sizeof(double)); /*Initialized focal length and Instrinstic matrix*/
         CameraPara[i].f=mtriKmatrix[i].n[0];
}
