//
//  Relative_Pose.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 4/1/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "Relative_Pose.h"
#include "CameraPoseRefinement.h"

double CameraPose:: CameraReprojectError(int NumPts, double *R, double* Tc, vector<v3_t> Pts,vector<v2_t> Projpts, double * Kmatrix)
{
    
   
    double error =0;
    for(int i  =0;i< NumPts;i++)
    {
     double b2[3];  
     double b_cam[3];
     double b_proj[3];
     double xij[2];   
    //K*[R|-Rtc]X//
    //K*R*[X-tc] //
    
    b2[0] = Pts[i].p[0] - Tc[0];
    b2[1] = Pts[i].p[1] - Tc[1];
    b2[2] = Pts[i].p[2] - Tc[2];
    
    matrix_product331(R, b2, b_cam);  
    
    matrix_product331(Kmatrix, b_cam, b_proj);
    
    xij[0] = -b_proj[0] / b_proj[2];
    xij[1] = -b_proj[1] / b_proj[2];
    
    double dx = Vx(Projpts[i]) - xij[0];
    double dy = Vy(Projpts[i]) - xij[1];
    
    //cout<< Vx(Projpts[i])<<" "<<Vy(Projpts[i])<<" "<< xij[0]<<" "<<xij[1]<<endl;   
     error += sqrt(dx * dx + dy * dy);
    }
    return(error);
}



CameraPose::CameraPose()
{
}
CameraPose::~CameraPose()
{
}

void CameraPose::PrintRotmatrix(int i)
{
    double tempR[9];
    PopRotcMatrix(i, tempR);
    matrix_print(3,3,tempR);
}

void CameraPose::PrintTmatrix(int i)
{
    double tempT[3];
    PopTcMatrix(i,tempT);
    matrix_print(3,1,tempT);
}

/* t1                = R12*t2+t12 */
// Previous camera         Current            relative 
//   center          =   camera rotation   +    
//                           matrix           translation

// Update process 
/*   Center of t2   */
//   t2= -(R12)'*t12
//   Frame 2 --> Frame 3 
//    
//   t3=R23'(t2-t23)
//   t3=-R23'*R12'*t12-R23'*t23
//

void CameraPose::Egomotion(EpipolarGeometry EG, FeaturePts FeaturePts)
{
    double R_relative[9];
    double T_relative[3];
    
    memcpy(R_relative,EG.R_relative, 9* sizeof(double));
    memcpy(T_relative,EG.t_relative, 3* sizeof(double));
    
    //int FrameNumber= SizeofPose()-1;  
    
    double *Rpre= new double[9];
    double *Tpre= new double[3];
    
    //double *Rcurrent = new double[9];
    //double *Tcurrent = new double[3];
    
    double fin_t[3];
    
    double updated_rotation[9];
    double updated_t[3];
    
    PopRotcMatrix((int) mRcMatrix.size()-1, Rpre);   // load previous 
    PopTcMatrix( (int) mTcMatrix.size()-1,Tpre);
     
    matrix_product33(R_relative, Rpre, updated_rotation);           // RotCurrent * RotPrevious // 
    matrix_transpose_product(3, 3, 3, 1, R_relative, Tpre , fin_t); // ( update  -Ri'*Center of previsous frame )
    
    updated_t[0]=fin_t[0]+T_relative[0];  // add to -(Rij)'*tij
    updated_t[1]=fin_t[1]+T_relative[1];
    updated_t[2]=fin_t[2]+T_relative[2];
    
    cout<<"egomotion"<<endl;
    cout<<updated_t[0]<<" "<<updated_t[1]<<" "<<updated_t[2]<<endl;
    
    double* Kmatrix = new double [9];
    
    PopKMattix((int) KMatrix.size()-1, Kmatrix) ;
    
    int NumofReproject = FeaturePts.NumReproject;
    double Tc_updated[3];
    
    //double error1 =  CameraReprojectError(NumofReproject, updated_rotation, updated_t , FeaturePts.mv3ProjectionPts ,FeaturePts.mv2ReprojectPts ,  Kmatrix);
    //cout<<"reprojection error no alightment  " <<error1<<endl;    
    // this part alight the 3D point with delat vector//
    
    TwoDalighment(NumofReproject, updated_rotation , updated_t , FeaturePts.mv3ProjectionPts, FeaturePts.mv2ReprojectPts, Tc_updated);
    
    
    double error2 =  CameraReprojectError(NumofReproject, updated_rotation, Tc_updated , FeaturePts.mv3ProjectionPts ,FeaturePts.mv2ReprojectPts ,  Kmatrix);
    cout<<"after alightment"<<endl;
    matrix_print(3,1,Tc_updated);
    
    //cout<<"reprojection error " <<error2<<endl;
    
    v3_t* mv3ProjectPts= new v3_t [NumofReproject];
    v2_t* mv2ReprojectPts= new v2_t [NumofReproject];
    
    for(int i=0;i< NumofReproject;i++)
    {
     
        mv3ProjectPts[i]= FeaturePts.mv3ProjectionPts[i];
        mv2ReprojectPts[i]= FeaturePts.mv2ReprojectPts[i];
    
    }
    
    double UpdateR[9];
    CameraRotRefine( NumofReproject ,mv3ProjectPts, mv2ReprojectPts , updated_rotation , Tc_updated , Kmatrix, UpdateR);
    
    //matrix_print(3,3,UpdateR);
    //matrix_print(3,1,Tc_updated);
    
    double error3 =  CameraReprojectError(NumofReproject, UpdateR, Tc_updated , FeaturePts.mv3ProjectionPts ,FeaturePts.mv2ReprojectPts ,  Kmatrix);
   
    //cout<<"reprojection error " <<error3<<endl;
    
       
    // update new camera pose data
    
    LoadTcMatrix(Tc_updated);
    LoadRotcMatrix(updated_rotation);
    
    delete [] mv3ProjectPts;
    delete [] mv2ReprojectPts;    

}
 void  CameraPose:: TwoDalighment(int NumofReproject , double*Rot, double*trans, vector<v3_t> P__3DSolvedforparameters, 
                                vector<v2_t> P__2DSolvedforparameters, double* Tcmatrix)
{
    
# define _2d_aligh 2
    
    double* Kmatrix = new double [9];
    
    PopKMattix((int) KMatrix.size()-1, Kmatrix) ;  

    v3_t *_3Dpt = new v3_t [NumofReproject];
    v2_t *_2Dpt = new v2_t [NumofReproject];

    double Tt[3]; 
    double Rott[9];
    double R_t[3]; 
    double Rott_transpose[9];
    double R_t_T[3];

    memcpy(Rott, Rot, 9*sizeof(double));
    memcpy(Tt, trans, 3*sizeof(double));
    
    matrix_product(3, 3, 3, 1, Rott, Tt, R_t);
    
    for(int i=0;i<NumofReproject;i++)
    {
      double X3D[3],q[3];
        
      X3D[0]= P__3DSolvedforparameters[i].p[0]; 
      X3D[1]= P__3DSolvedforparameters[i].p[1]; 
      X3D[2]= P__3DSolvedforparameters[i].p[2];
        
      matrix_product(3, 3, 3, 1, Rott, X3D, q);
        
      q[0]-=R_t[0]; q[1]-=R_t[1]; q[2]-=R_t[2];    
      
      _3Dpt[i].p[0]=q[0]; _3Dpt[i].p[1]=q[1]; _3Dpt[i].p[2]=q[2];
        
      _2Dpt[i].p[0]= P__2DSolvedforparameters[i].p[0]/Kmatrix[0]; 
      _2Dpt[i].p[1]= P__2DSolvedforparameters[i].p[1]/Kmatrix[4]; 
        
      //cout<<  _3Dpt[i].p[0]<<" "<< _3Dpt[i].p[1]<<" "<< _3Dpt[i].p[2]<<endl;
      //cout<<  _2Dpt[i].p[0]<<" "<<_2Dpt[i].p[1]<<endl;
    }
    
    double Parameter_vec[3];
    
    DeltaVector_Ransac( _2Dpt, _3Dpt, NumofReproject , Parameter_vec, 30 , 0.1);
    
    matrix_transpose(3, 3, Rott, Rott_transpose);
    matrix_product331(Rott_transpose,Parameter_vec , R_t_T);
        
    trans[0]= Tt[0]+ R_t_T[0];
    trans[1]= Tt[1]+ R_t_T[1];
    trans[2]= Tt[2]+ R_t_T[2];
    
    memcpy(Tcmatrix, trans, 3*sizeof(double));
    //cout<<"alightment result"<<endl;

    delete [] _3Dpt;
    delete [] _2Dpt;
    
}

void  CameraPose:: DeltaVector_Ransac(v2_t* _2Dpt, v3_t* _3Dpt, int size_ , double *Parameter_vec, int Ransac_runs, double error)
{
   
# define  MIN_NUM_PT 3
    
    double Minerror=  999999;
    double Parameter_vec_temp_ransac[3];
    double Parameter_vec_temp[3];
    
    for (int round = 0; round < Ransac_runs; round++) 
    {
        int support[MIN_NUM_PT];
        int i, j;
        
        v2_t _pts_2D[MIN_NUM_PT];
        v3_t _pts_3D[MIN_NUM_PT];
        
        //double Rtmp[9];
        int repeat = 0; 
        for (i = 0; i < MIN_NUM_PT; i++) 
        {
            /* Select an index from 0 to n-1 */
            int idx, reselect;
            do {
                reselect = 0;
                idx = rand() % size_;
                for (j = 0; j < i; j++) 
                {
                    if (support[j] == idx)
                    {
                        reselect = 1;
                        break;
                    }
                }
                 repeat++;
                if(repeat==500)
                {
                    cout<<" not enough points"<<endl;
                    break;
                }
                    
            } 
            while (reselect);
            
            support[i] = idx;
            
            _pts_3D[i] = _3Dpt[idx];
            _pts_2D[i] = _2Dpt[idx];
        }  
        
        /* Find out 3D point delta vector* Lef-> right */
        
        deltavector(_pts_2D, _pts_3D, Parameter_vec_temp);
       // matrix_print(3,1, Parameter_vec_temp);
        
        double Error_2Ddis=0;
        
        for (int i=0;i<size_;i++)
        {
             
            Error_2Ddis +=  Euclidence_3D(_2Dpt[i],_3Dpt[i],Parameter_vec_temp);
        }
        
        //cout<<"ransac_ "<<endl;
        //matrix_print(3,1,Parameter_vec_temp);
        //cout<<"error "<<Error_2Ddis<<endl;
        
        if (Minerror>Error_2Ddis)
        {
            Minerror= Error_2Ddis;
            
            Parameter_vec_temp_ransac[0] = Parameter_vec_temp[0];
            Parameter_vec_temp_ransac[1] = Parameter_vec_temp[1];
            Parameter_vec_temp_ransac[2] = Parameter_vec_temp[2];
            
        }
        
        
    }
    
    //cout<<"parameters_best_ransac :"<<Parameter_vec_temp_ransac[0]<<" "<<Parameter_vec_temp_ransac[1]<<" "<<Parameter_vec_temp_ransac[2]<<" minimum error "<<Minerror<<endl;
    //  edit 
    
    Parameter_vec[0]= Parameter_vec_temp_ransac[0];
    Parameter_vec[1]= Parameter_vec_temp_ransac[1];
    Parameter_vec[2]= Parameter_vec_temp_ransac[2];
    
    //cout<<"minimum error " <<Minerror<<endl;
    
}


double CameraPose :: Euclidence_3D (v2_t _2Dpt, v3_t _3Dpt, double* Parameter_vec)
{
    double *temp= new double [3];  
    double *result= new double[2];
    double error;
    
    temp[0]= _3Dpt.p[0]-Parameter_vec[0];
    temp[1]= _3Dpt.p[1]-Parameter_vec[1];
    temp[2]= _3Dpt.p[2]-Parameter_vec[2];
    
    result[0]=_2Dpt.p[0]-(-temp[0]/temp[2]);
    result[1]=_2Dpt.p[1]-(-temp[1]/temp[2]);
    
    error= sqrt((result[0]*result[0])+(result[1]*result[1]));
    
    delete [] temp; 
    delete [] result;
    
    return(error);
}

void CameraPose:: deltavector(v2_t* _2Dpt,  v3_t* _3Dpt, double* Parametrvec_)
{
    // find solution use least square //
    
    double* A=  new double [18];
    double* B = new double[6];
    double* ATA = new double [9]; 
    double* AT = new double[18];
    double* INVATA = new double [9]; 
    double* INVATAAt= new double[18];
    
    A[0]  = -1.0 ;           A[1] = 0.0;            A[2] =  -(_2Dpt[0].p[0]);    
    A[3]  = -1.0 ;           A[4] = 0.0;            A[5] =  -(_2Dpt[1].p[0]);     
    A[6]  = -1.0 ;           A[7] = 0.0;            A[8] =  -(_2Dpt[2].p[0]);    
    A[9]  = 0.0;             A[10] = -1.0;          A[11] = -(_2Dpt[0].p[1]);    
    A[12] = 0.0;             A[13] = -1.0;          A[14] = -(_2Dpt[1].p[1]);   
    A[15] = 0.0;             A[16] = -1.0;          A[17] = -(_2Dpt[2].p[1]);    
    
    
    B[0]= -((_3Dpt[0].p[2]*_2Dpt[0].p[0])+_3Dpt[0].p[0]);
    B[1]= -((_3Dpt[1].p[2]*_2Dpt[1].p[0])+_3Dpt[1].p[0]);
    B[2]= -((_3Dpt[2].p[2]*_2Dpt[2].p[0])+_3Dpt[2].p[0]);
    B[3]= -((_3Dpt[0].p[2]*_2Dpt[0].p[1])+_3Dpt[0].p[1]);
    B[4]= -((_3Dpt[1].p[2]*_2Dpt[1].p[1])+_3Dpt[1].p[1]);
    B[5]= -((_3Dpt[2].p[2]*_2Dpt[2].p[1])+_3Dpt[2].p[1]);
    
    matrix_transpose(6, 3, A, AT);
    
    matrix_product(3, 6, 6, 3, AT, A, ATA);
    
    matrix_invert(3, ATA, INVATA);
    
    matrix_product(3, 3, 3, 6, INVATA, AT, INVATAAt);
    
    matrix_product(3, 6, 6, 1, INVATAAt, B, Parametrvec_);
    
    delete [] A;
    delete [] B; 
    delete [] ATA; delete [] AT; 
    delete [] INVATA;
    delete [] INVATAAt;
    
}

double CameraPose :: TriangulationN_Frames(FeaturePts Pts)
{
    //int index=0;
    int num_frame= (int)v2_frame[index].size();
    //cout<<num_frame<<endl;
    v2_t *pv = new v2_t[num_frame];
    double *Rs= new double [9*  num_frame];
    double *ts = new double[3 * num_frame];
    
    for (int i=0; i<num_frame;i++)
        // for (int i=0; i<2;i++)
    {
        int N = (int)v2_frame[index][i];
        
        double  Pt3[3]= {v2_location[index][i].p[0]-400, v2_location[index][i].p[1]-300, 1.0};
        double Translation_Scaled [3];
        double Translated [3];
        double K [9];
        double Kinv [9];
        double Rotation[9];
        // double Translation_Scaled [3];
        // double Translated [3];
        // double K [9];
        // double Kinv[9];
        // double *Rotation = new double [9];
        // double Rotation[9];
        
        GetIntrinsics(K,N);  /* Get_focal_length* i is ith camera's parameters*/ 
       
        //matrix_print(3, 3, K);
        matrix_invert(3, K, Kinv);
        double p_n[3];
        
        matrix_product(3, 3, 3, 1, Kinv, Pt3, p_n);
        pv[i]= v2_new(-p_n[0],-p_n[1]);
        //pv[i] = UndistortNormalizedPoint(pv[i], cameras_app[N]);
        
        //cout<<"2D  "<<pv[i].p[0]<<" "<<pv[i].p[1]<<endl;
        Rotation[0]= cameras_app[N].R[0], Rotation[1]= cameras_app[N].R[1], Rotation[2]=cameras_app[N].R[2],
        Rotation[3]= cameras_app[N].R[3], Rotation[4]= cameras_app[N].R[4], Rotation[5]=cameras_app[N].R[5],
        Rotation[6]= cameras_app[N].R[6], Rotation[7]= cameras_app[N].R[7], Rotation[8]=cameras_app[N].R[8];
        memcpy(Rs + 9 * i, Rotation, 9 * sizeof(double));
        double Translation[3]={cameras_app[N].t[0],cameras_app[N].t[1],cameras_app[N].t[2]};
        matrix_product(3,3,3,1,Rotation,Translation,Translated);
        matrix_scale(3,1,Translated,-1.0,Translation_Scaled); 
        memcpy(ts + 3 * i,Translation_Scaled, 3 * sizeof(double));
    }
    double error=0;
    
    v3_t pt = triangulate_n(num_frame, pv, Rs, ts, &error);
    //cout<<pt.p[0]<<" "<<pt.p[1]<<" "<<pt.p[2]<<"  "<<endl;
    _3Dlocation[index].p[0]=pt.p[0];
    _3Dlocation[index].p[1]=pt.p[1];
    _3Dlocation[index].p[2]=pt.p[2];
    double _2D_error=0;
    // cout<<endl;
    for (int i=0; i<num_frame;i++)
    {
        int N_reprojection = (int)v2_frame[index][i];
        //double  Pt3__reprojection[3]= {v2_location[index][i].p[0]-400, v2_location[index][i].p[1]-300, 1.0};
        v2_t pr= reprojection_error(&cameras_app[N_reprojection], pt);
        //cout<<"2D"<<v2_location[index][i].p[0]<<" "<<v2_location[index][i].p[1]<<" ";
        //cout<<"reprojection "<<pr.p[0]+400<<" "<<pr.p[1]+300<<" "<<endl;
        double dx = v2_location[index][i].p[0]-(pr.p[0]+400);
        double dy = v2_location[index][i].p[1]-(pr.p[1]+300);
        double err1 = dx * dx + dy * dy;
        //cout<<"  "<<i<<"  "<< err1<<"  ";
        _2D_error+= dx * dx + dy * dy;
    }
    
    //cout<<" reprojectionerror "<<_2D_error<<" num_frame "<<num_frame<<endl;
    
    free(Rs);
    free(ts);
    delete[]pv;
    return(_2D_error);
}

