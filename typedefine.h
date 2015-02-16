//
//  typedefine.h
//  test_1
//
//  Created by chih-hsiang chang on 2/10/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//

#ifndef __included_typedefine_h
#define __included_typedefine_h

#include "vector.h"
#include <vector>

using namespace std;


typedef struct 
{
    vector <v3_t>R_pts;
    vector <v3_t>L_pts;

}  F_refined;

typedef struct 
{
    vector<v3_t>R_pts;
    vector<v3_t>L_pts;
    
} matched;

typedef struct 
{
    double p[9];
    
} F_key_matrix;

typedef struct 
{
    vector<v3_t>left_input;
    vector<v3_t>right_input;
} p3_t ;

typedef struct
{
    vector<v3_t> threeDpt;
} threeDpt;  

typedef struct 
{
    double R_matrix[9];
    double T_matrix[3];

} cameramatrix;

typedef struct 
{

vector <v3_t> pts; 

}V3D_position;

typedef struct 
{    

    vector <v3_t> Rs; 

}R_reserved;
 typedef struct 
{
    vector<int> ThreeDindex;
    vector<int> TwoDindex;
    
} reserved_index;
typedef struct
{
double ELEMENT [9];
} Rotation_matrix;
typedef struct
{
double ELEMENT [3];
} Translation_matrix;
typedef struct 
{
 double ELEMENT[9];
} Intrinsic_matrix;

#endif