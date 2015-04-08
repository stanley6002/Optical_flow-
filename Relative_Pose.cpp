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

void CameraPose::PrintKmatrix(int i)
{
    double tempK[9];
    PopKMattix(i,tempK);
    matrix_print(3,3,tempK);
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
    
    double error1 =  CameraReprojectError(NumofReproject, updated_rotation, updated_t , FeaturePts.mv3ProjectionPts ,FeaturePts.mv2ReprojectPts ,  Kmatrix);
    cout<<"reprojection error no alightment  " <<error1/NumofReproject<<endl;    
    // this part alight the 3D point with delat vector//
    
    TwoDalighment(NumofReproject, updated_rotation , updated_t , FeaturePts.mv3ProjectionPts, FeaturePts.mv2ReprojectPts, Tc_updated);
    
    
    double error2 =  CameraReprojectError(NumofReproject, updated_rotation, Tc_updated , FeaturePts.mv3ProjectionPts ,FeaturePts.mv2ReprojectPts ,  Kmatrix);
    cout<<"after alightment"<<endl;
    matrix_print(3,1,Tc_updated);
    cout<<"reprojection error " <<error2/NumofReproject<<endl;
    
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
   
    cout<<"NumofReproject" << NumofReproject <<"reprojection error " <<error3/NumofReproject<<endl;
    
       
    // update new camera pose data
   
    LoadKMatrix(Kmatrix,(int) KMatrix.size()-1);
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
    
    DeltaVector_Ransac( _2Dpt, _3Dpt, NumofReproject , Parameter_vec, 50 , 0.1);
    
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
                if(repeat==800)
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

double CameraPose :: TriangulationN_Frames(FeaturePts& Pts)
{    
    int NumPts = (int) Pts.mv2_location.size();
    
    //v3_t* _3Dpts = new v3_t[NumPts];
    vector<v3_t> _3Dpts;
    
    for(int i=0;i< NumPts;i++)
    {
    
       int num_frame= (int)Pts.mv2_frame[i].size();  // read number of frame in the list //
       v2_t *pv = new v2_t[num_frame];
       double *Rs= new double [9*  num_frame];
       double *ts = new double[3 * num_frame];
    
     for (int j=0; j<num_frame ;j++)
        // for (int i=0; i<2;i++)
       {
        int N = (int) Pts.mv2_frame[i][j];
        double Pt3[3]= {Pts.mv2_location[i][j].p[0], Pts.mv2_location[i][j].p[1], 1.0};
        double Translation_Scaled [3];
        double Translated [3];
        double K [9];
        double Kinv [9];
        double Rotation[9];
        double Tc[3];
        
        PopKMattix(N, K);  
        /* Get_focal_length* i is ith camera's parameters*/ 
        //cout<<"T vector"<<endl;       
        
        matrix_invert(3, K, Kinv);
        double p_n[3];
        
        matrix_product(3, 3, 3, 1, Kinv, Pt3, p_n);
        pv[j]= v2_new(-p_n[0],-p_n[1]);
        
        PopRotcMatrix(N, Rotation);

        memcpy(Rs + 9 * j, Rotation, 9 * sizeof(double));
       
        PopTcMatrix(N,Tc);
        matrix_product(3,3,3,1,Rotation,Tc,Translated);
        matrix_scale(3,1,Translated,-1.0,Translation_Scaled); 
        memcpy(ts + 3 * j,Translation_Scaled, 3 * sizeof(double));
        
    }
    
    double error=0;
    
    v3_t pt = triangulate_n(num_frame, pv, Rs, ts, &error);
    _3Dpts.push_back(pt); 
        
      free(Rs);
      free(ts);
      delete[]pv;
    }
     bool* tempvector = new bool [NumPts];
      for (int i=0;i<NumPts;i++)
           tempvector[i]= false;
     
    RefineN_FramePoints( _3Dpts , NumPts, tempvector);
    
    int shift_index=0;
    for(int i=0;i< NumPts; i++)
    {
        if(tempvector[i]== true)
        {
          int removal_index =i;
          removal_index -=  shift_index;
          _3Dpts.erase(_3Dpts.begin()+removal_index);
          Pts.mv2_frame.erase(Pts.mv2_frame.begin()+removal_index);
          Pts.mv2_location.erase(Pts.mv2_location.begin()+removal_index);
          shift_index++;
        }   
     }
    Pts. m_3Dpts.swap(_3Dpts);
    DumpPointsToPly("/Users/chih-hsiangchang/Desktop/Archive/result.ply", Pts. m_3Dpts
                    ,(NumPts-shift_index));
    
    cout<<"test"<<endl;
    //return(_2D_error);
}
void RefineN_FramePoints(vector<v3_t>_3DPts, int NumPts, bool* tempvector)
{
    /// check Cheirality
    for (int i=0;i<NumPts;i++)
     {
        if(CheckCheirality(_3DPts[i]))
        { 
            tempvector[i]= true;
        }
    }
    
    // check depth //
    _3DdepthRefine(_3DPts,tempvector, NumPts);
    
 }
void _3DdepthRefine (vector<v3_t> m_3Dpts, bool* tempvector, int num_ofrefined_pts)
{
    
    int size_= num_ofrefined_pts;
    double max_number = -1.0;    // remove outliers from candidated points 
    double min_number = -50.0;
    double range;
    int i;
    int Nbins = 50;
    
    int Bin[50]={};
    
    int mx_index= 0;
    double  Range_low;
    double  Range_upper;
    
    for (int i=0;i< size_;i++)
    {
        if ( m_3Dpts[i].p[2]< min_number)
            tempvector[i]= true;                 
    }
    
    range = (max_number - min_number) / Nbins;
    for(i=0; i<size_;i++)
    {
        if (tempvector[i] == false)
        {
            for(int index=0; index< Nbins; index++)
            {
                
                Range_low = min_number   +   (index)*range;
                Range_upper = min_number +   (index+1)*range;
                float x = (float) m_3Dpts[i].p[2];
                
                if ( Range_low < x && x <= Range_upper)              
                {
                    Bin[index] += 1; 
                    
                }
            }
        }
    }
    int mx_bin  = 0;
    for (int i=0;i<Nbins;i++)
    {
        if(Bin[i]> mx_bin)
        {
            mx_bin = Bin[i];
            mx_index = i;
            
        }
    }
    
    float depth = (min_number+(mx_index)*range);
    float varince= Variance (m_3Dpts, depth, size_);
    float *densitytemp  = new float [size_]; 
    cout<< depth <<"variance "<<varince <<endl;
    for (int i=0;i< size_;i++)
    {
        float x = (float) m_3Dpts[i].p[2];
        float a=-fabs(x-depth)*(1./(1.06*(sqrt(varince))*2.1));
        //float density = exp(a);
        densitytemp[i]=exp(a);
        if (densitytemp[i]<0.3)
            tempvector[i] = true;
    }
}

float Variance (vector<v3_t> _3Dpts, const float depth , const int size_)
{   
    float* tempz= new float [size_];
    float sum=0;
    int num=0;
    for (int i=0;i<size_;i++)
    { 
        if(_3Dpts[i].p[2]< 0 && _3Dpts[i].p[2]> -30 )
        {
            tempz[i]= (float)(_3Dpts[i].p[2]-depth)*(_3Dpts[i].p[2]-depth);
            sum=sum+tempz[i];
            num++;
        }
    }

    return( sum *= 1. / num );

}

void DumpPointsToPly(char *output_directory, vector<v3_t> points
                     ,int num_points) 
{ 
    static char ply_header[] = 
    {"ply\n"
        "format ascii 1.0\n"
        "element face 0\n"
        "property list uint8 int32 vertex_indices\n"
        "element vertex %d\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "end_header\n"};
    
    char ply_out[256];
    //sprintf(ply_out, "%s/%s", output_directory, filename);
    
    FILE *f = fopen(output_directory, "w");
    
    if (f == NULL) 
    {
        printf("Error opening file %s for writing\n", ply_out);
        return;
    }
    
    /* Print the ply header */
    fprintf(f, ply_header,num_points);
    
    for (int i = 0; i < num_points; i++)
    {
        
        /* Output the vertex */
        fprintf(f, "%0.6f %0.6f %0.6f\n", points[i].p[0],points[i].p[1],points[i].p[2]);
    }
    
    fclose(f);
}
bool CheckCheirality(v3_t pt)
{
    bool Cheirality=false;
    if(pt.p[2]>0)
        Cheirality=true;
        return(Cheirality);
}
