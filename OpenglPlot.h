//
//  OpenglPlot.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/24/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//
#include "glfw3.h"
#include <vector.h>
#include "vector.h"

class OpenGLPlot 
{
    
public:
    int width;
    int height;
    
    GLFWwindow* window;

    OpenGLPlot (int width , int height);
    ~ OpenGLPlot ();
    
    void error_callback(int error, const char* description);
    //void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
    void Initialized();
    void Setview();
    void PlotVertex(int size_, const vector<v3_t> V3Dpts);
    void PlotCamera(double* t_relative);
    void SetupFrustrum();
private:
    
};

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
