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
        //glEnable(GL_DEPTH_TEST);
    
        glViewport(0, 0, this->width, this->height);
        glClear(GL_COLOR_BUFFER_BIT);
                  
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);              
        glMatrixMode(GL_MODELVIEW);
       
        float tm[16];
    
        tm[0] = 1.0f;   tm[1] = 0.0f;   tm[2] = 0.0f;   tm[3] = 0.0f;
        tm[4] = 0.0f;   tm[5] = 1.0f;   tm[6] = 0.0f;   tm[7] = 0.0f;
        tm[8] = 0.0f;   tm[9] = 0.0f;   tm[10]=  1.0f;   tm[11]= 0.0f;
        tm[12]= 0.0f;   tm[13]= 0.0f;   tm[14]= -20.0f;  tm[15]= 1.0f;
    
        glLoadMatrixf(tm);

        //glOrtho(-400.0, 400.0, -320.f, 320.f, 10.f, -10.f);
       
        SetupFrustrum(); 
}
void OpenGLPlot::SetupFrustrum()
{
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glFrustum (-4.0, 4.0, -4.0, 4.0, 5.5 , 55.5);
    //glOrtho(0.0,320,240, 0.0f, 10.0f, 20.0f);
    
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix();
}
void OpenGLPlot::PlotCamera(double *t_relative)
{
    glPushMatrix();
    glTranslated(0.0 ,-0.0  ,0.0); 
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.6f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f,0.0f);
    glVertex3f(0.0f, 1.6f, 0.0f);
    glColor3f(0, 0, 1); 
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.6f);
    glEnd();
    
    glTranslated(t_relative[0] ,t_relative[1]  ,t_relative[2]); 
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.6f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f,0.0f);
    glVertex3f(0.0f, 1.6f, 0.0f);
    glColor3f(0, 0, 1); 
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.6f);
    glEnd();
    glPopMatrix();
    //glPushMatrix();
    //glClear(GL_COLOR_BUFFER_BIT);
   
    
}
 void OpenGLPlot::PlotVertex(int size_, const vector<v3_t> V3Dpts)
{

    glLoadIdentity();
    glPointSize(2.0f);
    glBegin( GL_POINTS);
    for (int i=0;i<size_;i++)
    {
        glColor3f(1, 1, 1); 
        glVertex3f(V3Dpts[i].p[0],V3Dpts[i].p[1] ,V3Dpts[i].p[2]);
        //cout<< V3Dpts[i].p[0]<<" "<<V3Dpts[i].p[1]<<" "<<V3Dpts[i].p[2]<<endl;
    }
    glEnd();
    glPushMatrix();


    //glDisable(GL_DEPTH_TEST);
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