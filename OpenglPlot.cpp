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
/// a call back function  
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
                  glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
                  glMatrixMode(GL_MODELVIEW);
}

void OpenGLPlot::PlotCamera()
{

    glLoadIdentity();
    glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
    
    glBegin(GL_LINES);
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.3f, 0.0f, 0.0f);
    
    glColor3f(0.f, 0.f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.3f, 0.f);
    
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.3f);
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