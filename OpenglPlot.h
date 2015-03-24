//
//  OpenglPlot.h
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/24/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//
#include "glfw3.h"

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
    void PlotCamera();
private:
    
};

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
