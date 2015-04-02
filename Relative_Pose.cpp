//
//  Relative_Pose.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 4/1/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "Relative_Pose.h"

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

void CameraPose::Egomotion(EpipolarGeometry EG, FeaturePts FeaturePts)
{
    double R_relative[9];
    double T_relative[3];
    
    memcpy(R_relative,EG.R_relative, 9* sizeof(double));
    memcpy(T_relative,EG.t_relative, 3* sizeof(double));
    
    int size_= SizeofPose();  
    
    double *Rpre= new double[9];
    double *Tpre= new double[3];
    
    //double *Rcurrent = new double[9];
    //double *Tcurrent = new double[3];
    
    double *fin_t = new double[3];
    
    double updated_rotation[9];
    double updated_t[3];
    
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
    
    PopRotcMatrix(size_-1, Rpre);   // load previous 
    PopTcMatrix(size_-1,Tpre);
     
    matrix_product33(R_relative, Rpre, updated_rotation);                 // RotCurrent * RotPrevious // 
    matrix_transpose_product(3, 3, 3, 1, R_relative, Tpre , fin_t);       // ( update  -Ri'*Center of previsous frame )
    
    updated_t[0]=fin_t[0]+T_relative[0];  // add to -(Rij)'*tij
    updated_t[1]=fin_t[1]+T_relative[1];
    updated_t[2]=fin_t[2]+T_relative[2];
    
    
    int NumofReproject = FeaturePts.NumReproject;
    
    TwoDalighment( NumofReproject, updated_rotation , updated_t , FeaturePts.mv3ProjectionPts, FeaturePts.mv2ReprojectPts ,EG);

    
    PrintTmatrix(1);
    cout<<updated_t[0]<<" "<<updated_t[1]<<" "<<updated_t[2]<<endl;

    

}
 void  CameraPose:: TwoDalighment(int NumofReproject , double*Rot, double*trans, vector<v3_t> P__3DSolvedforparameters, 
                                vector<v2_t> P__2DSolvedforparameters, EpipolarGeometry EG)
{
    
# define _2d_aligh 2
    
    double* Kmatrix = new double [9];
    EG.PopIntrinsicMatrix(Kmatrix);
    
//    v3_t* pt= new v3_t[(int)_2dalighment.size()];
//    v3_t* delta = new v3_t[(int)_2dalighment.size()];
//    
//    v3_t* delta_vector = new v3_t [(int)_2dalighment.size()]; 
//    
//    GetIntrinsics(K1,numimagej-2);  
//    GetIntrinsics(K2,numimagej-1); 
   
   
    
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
        
      q[0] -=R_t[0]; q[1] -=R_t[1]; q[2] -=R_t[2]; 
      
      _3Dpt[i].p[0]=q[0]; _3Dpt[i].p[1]=q[1]; _3Dpt[i].p[2]=q[2];
        
      _2Dpt[i].p[0]= P__2DSolvedforparameters[i].p[0]/Kmatrix[0]; 
      _2Dpt[i].p[1]= P__2DSolvedforparameters[i].p[1]/Kmatrix[4]; 
    }
    
    double Parameter_vec[3];
    
    //DeltaVector_Ransac(_2Dpt, _3Dpt, size_, Parameter_vec,   2000, 0.1);
    
    //DeltaVector_Ransac(_2Dpt, _3Dpt, size_, Parameter_vec, 1000, 0.1);
    
    matrix_transpose(3, 3, Rott, Rott_transpose);
    matrix_product331(Rott_transpose,Parameter_vec , R_t_T);
    
    //cout<<"Update_vector"<<endl;
    //cout<<Tt[0]+ R_t_T[0]<<" "<< Tt[1]+ R_t_T[1]<<" "<<Tt[2]+ R_t_T[2]<<endl;
    //cout<<endl;
    
    trans[0]= Tt[0]+ R_t_T[0];
    trans[1]= Tt[1]+ R_t_T[1];
    trans[2]= Tt[2]+ R_t_T[2];
    
    
    free(_3Dpt); free(_2Dpt);
    
}

//void   DeltaVector_Ransac(v2_t* _2Dpt,v3_t* _3Dpt, int size_ , double *Parameter_vec, int Ransac_runs, double error)
//{
//   
//# define  MIN_NUM_PT 3
//    
//    double Minerror=  999999;
//    double Parameter_vec_temp_ransac[3];
//    double Parameter_vec_temp[3];
//    for (int round = 0; round < Ransac_runs; round++) 
//    {
//        int support[MIN_NUM_PT];
//        int i, j;
//        
//        v2_t _pts_2D[MIN_NUM_PT];
//        v3_t _pts_3D[MIN_NUM_PT];
//        
//        //double Rtmp[9];
//        
//        for (i = 0; i < MIN_NUM_PT; i++) 
//        {
//            /* Select an index from 0 to n-1 */
//            int idx, reselect;
//            do {
//                reselect = 0;
//                idx = rand() % size_;
//                for (j = 0; j < i; j++) 
//                {
//                    if (support[j] == idx)
//                    {
//                        reselect = 1;
//                        break;
//                    }
//                }
//            } 
//            while (reselect);
//            
//            support[i] = idx;
//            
//            _pts_3D[i] = _3Dpt[idx];
//            _pts_2D[i] = _2Dpt[idx];
//            
//            
//        }  
//        
//        /* Find out 3D point delta vector* Lef-> right */
//        
//        deltavector(_pts_2D, _pts_3D, Parameter_vec_temp);
//        
//        double Error_2Ddis=0;
//        
//        for (int i=0;i<size_;i++)
//        {
//            
//            Error_2Ddis+= Euclidence_3D(_2Dpt[i],_3Dpt[i],Parameter_vec_temp);
//        }
//        
//        //cout<<"ransac_ "<<endl;
//        //matrix_print(3,1,Parameter_vec_temp);
//        //cout<<"error "<<Error_2Ddis<<endl;
//        
//        if (Minerror>Error_2Ddis)
//        {
//            Minerror= Error_2Ddis;
//            Parameter_vec_temp_ransac[0] = Parameter_vec_temp[0];
//            Parameter_vec_temp_ransac[1] = Parameter_vec_temp[1];
//            Parameter_vec_temp_ransac[2] = Parameter_vec_temp[2];
//            
//        }
//        
//        
//    }
//    
//    cout<<"parameters_best_ransac :"<<Parameter_vec_temp_ransac[0]<<" "<<Parameter_vec_temp_ransac[1]<<" "<<Parameter_vec_temp_ransac[2]<<" minimum error "<<Minerror<<endl;
//    
//    ///  edit 
//    Parameter_vec[0]= Parameter_vec_temp_ransac[0];
//    Parameter_vec[1]= Parameter_vec_temp_ransac[1];
//    Parameter_vec[2]= Parameter_vec_temp_ransac[2];
//    
//    
//    
//}


//double Euclidence_3D (v2_t _2Dpt, v3_t _3Dpt, double* Parameter_vec)
//{
//    double *temp= new double [3];  
//    double *result= new double[2];
//    double error;
//    
//    temp[0]= _3Dpt.p[0]-Parameter_vec[0];
//    temp[1]= _3Dpt.p[1]-Parameter_vec[1];
//    temp[2]= _3Dpt.p[2]-Parameter_vec[2];
//    
//    result[0]=_2Dpt.p[0]-(-temp[0]/temp[2]);
//    result[1]=_2Dpt.p[1]-(-temp[1]/temp[2]);
//    
//    error= sqrt((result[0]*result[0])+(result[1]*result[1]));
//    
//    free(temp); free(result);
//    
//    return(error);
//}

//void deltavector(v2_t* _2Dpt,  v3_t* _3Dpt, double* Parametrvec_)
//{
//    
//    //double A[18];    double B[6];  
//    //double ATA[9];  double AT[18]; double INVATA[9];
//    //double INVATAAt[18]; 
//    double* A=  new double [18];
//    double* B = new double[6];
//    double* ATA = new double [9]; 
//    double* AT = new double[18];
//    double* INVATA = new double [9]; 
//    double* INVATAAt= new double[18];
//    //double Temp_parameter[4];
//    
//    ///  edit A matrix ;
//    
//    A[0]  = -1.0 ;           A[1] = 0.0;            A[2] =  -(_2Dpt[0].p[0]);    
//    A[3]  = -1.0 ;           A[4] = 0.0;            A[5] =  -(_2Dpt[1].p[0]);     
//    A[6]  = -1.0 ;           A[7] = 0.0;            A[8] =  -(_2Dpt[2].p[0]);    
//    A[9]  = 0.0;             A[10] = -1.0;          A[11] = -(_2Dpt[0].p[1]);    
//    A[12] = 0.0;             A[13] = -1.0;          A[14] = -(_2Dpt[1].p[1]);   
//    A[15] = 0.0;             A[16] = -1.0;          A[17] = -(_2Dpt[2].p[1]);    
//    
//    
//    B[0]= -((_3Dpt[0].p[2]*_2Dpt[0].p[0])+_3Dpt[0].p[0]);
//    B[1]= -((_3Dpt[1].p[2]*_2Dpt[1].p[0])+_3Dpt[1].p[0]);
//    B[2]= -((_3Dpt[2].p[2]*_2Dpt[2].p[0])+_3Dpt[2].p[0]);
//    B[3]= -((_3Dpt[0].p[2]*_2Dpt[0].p[1])+_3Dpt[0].p[1]);
//    B[4]= -((_3Dpt[1].p[2]*_2Dpt[1].p[1])+_3Dpt[1].p[1]);
//    B[5]= -((_3Dpt[2].p[2]*_2Dpt[2].p[1])+_3Dpt[2].p[1]);
//    
//    
//    //matrix_print(6,3,A);
//    
//    //matrix_print(6,1,B);
//    
//    matrix_transpose(6, 3, A, AT);
//    
//    matrix_product(3, 6, 6, 3, AT, A, ATA);
//    
//    matrix_invert(3, ATA, INVATA);
//    
//    matrix_product(3, 3, 3, 6, INVATA, AT, INVATAAt);
//    
//    matrix_product(3, 6, 6, 1, INVATAAt, B, Parametrvec_);
//    
//    free(A); free(B); free(ATA); free(AT);free(INVATA); free(INVATAAt);
//    
//}


