//
//  OpenglPlot.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/24/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "OpenglPlot.h"
#include <iostream>


void OpenGLPlot::error_callback(int error, const char* description)
{
       fputs(description, stderr);
}
/// a global call back function  
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GL_TRUE);
}
void OpenGLPlot::Initialized ()
{
        if (!glfwInit())
              exit(EXIT_FAILURE);
        window= glfwCreateWindow(this-> width , this ->height, "OpenGL", NULL, NULL);
        
        if (!window)
           {
              glfwTerminate();
              exit(EXIT_FAILURE);
           }

         glfwMakeContextCurrent(window);
         glfwSwapInterval(1);
         glfwSetKeyCallback (window, key_callback);
}
void OpenGLPlot:: Setview()
{
        glLineWidth(4);
        glfwGetFramebufferSize(window, &this->width, &this->height);
        float ratio = this->width / (float) this-> height;
                  
        glViewport(0, 0, this->width, this->height);
        glClear(GL_COLOR_BUFFER_BIT);
                  
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
    
        glOrtho(-5, 5, -10.f, 10.f, 10.f, -10.f);
        glMatrixMode(GL_MODELVIEW);
        SetupFrustrum(); 
}
void OpenGLPlot::SetupFrustrum()
{
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        double Znear = 0.03;
        glFrustum(- Znear, Znear , 0.75*Znear, -0.75*Znear , Znear , 20);
        glScalef(1 , 1, -1);
}
void OpenGLPlot::PlotCamera(int size_, const vector<v3_t> V3Dpts)
{

        glLoadIdentity();
        glBegin(GL_POINTS);
        for (int i=0;i<5;i++)
            {
               
             glVertex3f(2.0f,2.0f,-3.0f);
             glColor3f((1.0f),(1.0f),(1.0f));
        
            }
  
    glEnd();
    
    
        glfwSwapBuffers(window);
        glfwPollEvents();

}
OpenGLPlot:: OpenGLPlot (int width, int height)
 {
         OpenGLPlot:: height = height;
         OpenGLPlot:: width  = width;            
         OpenGLPlot::Initialized();
 }
OpenGLPlot::~OpenGLPlot()
{
}