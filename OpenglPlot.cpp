//
//  OpenglPlot.cpp
//  Optical_flow 
//
//  Created by chih-hsiang chang on 3/24/15.
//  Copyright 2015 __MyCompanyName__. All rights reserved.
//

#include "OpenglPlot.h"
#include <iostream>


const float DEG2RAD = 3.141593f / 180;
int GlobalzoomFact= 120;

Matrix4X4 matrixModelView;    // = matrixView * matrixModel
Matrix4X4 matrixProjection;

Matrix4X4 matrixModelView2; 




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
        window= glfwCreateWindow(this-> width , this ->height, "OpenGL1", NULL, NULL);
        //window2= glfwCreateWindow(this-> width , this ->height, "OpenGL2", NULL, NULL);
        if (!window)
           {
              glfwTerminate();
              exit(EXIT_FAILURE);
           }

         //glfwMakeContextCurrent(window);
         //glfwMakeContextCurrent(window2);
         glfwSwapInterval(1);
         //glfwSetKeyCallback (window, key_callback);
}
void OpenGLPlot:: Setview(const vector<v3_t> V3Dpts)
{
    
            ReadObject(V3Dpts);
            glfwMakeContextCurrent(window);
        
            //glLineWidth(4);
            glfwGetFramebufferSize(window, &this->width, &this->height);
            //float ratio = this->width / (float) this-> height;
            //glEnable(GL_DEPTH_TEST);
            //glClear(GL_COLOR_BUFFER_BIT);      
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glViewport(0, 0, width, height);
            //SetupFrustrum();   
             Reshape(this->width, this-> height);
             glMatrixMode(GL_MODELVIEW);
             
             
    
             
             matrixModelView.identity();
             matrixModelView.rotate( 30.0, 0.0f, 1.0f, 0.0f);
             //matrixModelView.rotate( 60.0f , 0.0f, 0.0f, 1.0f);
             matrixModelView.rotate( 50.0,  1.0f, 0.0f, 0.0f);
             matrixModelView.translate(0.0 ,0, -f_Distance-300);
           
             cout<<"center "<<Mouse_y_theta<<" "<<Mouse_r_theta<<" "<<endl;             
             glLoadMatrixf(matrixModelView.getTranspose());
             glPushMatrix();
 
    

             PlotVertex((int) V3Dpts.size() , V3Dpts);
             glfwSwapBuffers(window);
             glfwPollEvents();
             
    
    
    
//    glfwMakeContextCurrent(window2);
//    glfwGetFramebufferSize(window2, &width, &height);
//    float ratio = width / (float) height;
//    
//    glViewport(0, 0, width, height);
//    
//    //glClear(GL_COLOR_BUFFER_BIT);              
//    glMatrixMode(GL_MODELVIEW);
//    matrixModelView2.identity();
//    matrixModelView2.rotate( 0.0 ,0.0f, 1.0f, 0.0f);
//    matrixModelView2.rotate( 0.0, 0.0f, 0.0f, 1.0f);
//    matrixModelView2.rotate( 0.0f, 1.0f, 0.0f, 0.0f);
//    matrixModelView2.translate(center_x, center_y+20 , -50);
//    glLoadMatrixf(matrixModelView2.getTranspose());
//    //set_camera2();
//    //glRotatef((float) glfwGetTime() * 10.f, 0.f, 0.f, 1.f);
//      PlotVertex((int) V3Dpts.size() , V3Dpts);
////    glBegin(GL_TRIANGLES);
////    glColor3f(1.f, 0.f, 0.f);
////    glVertex3f(-1.6f, -0.4f, -12.f);
////    glColor3f(0.f, 1.f, 0.f);
////    glVertex3f(0.6f, -0.4f, -12.f);
////    glColor3f(0.f, 0.f, 0.f);
////    glVertex3f(0.f, 0.6f, -8.f);
////    glEnd();
//    
//        
//    glPushMatrix();                     // save current modelview matrix
//    glLoadIdentity();    
//    glMatrixMode (GL_PROJECTION);
//    // Reshape(this->width, this-> height);
//    //glLoadIdentity ();
//    glFrustum (-4.0, 4.0, -4.0, 4.0, 5, 100.0);
//    glMatrixMode (GL_MODELVIEW);
    
    glfwSwapBuffers(window);
//    glfwSwapBuffers(window2);
    glfwPollEvents();

        

    
//        glfwMakeContextCurrent(window2); 
//        glfwGetFramebufferSize(window2, &this->width, &this->height);
//        glMatrixMode(GL_PROJECTION);
//        glLoadIdentity();
//        glViewport(0, 0, width, height);
//        //SetupFrustrum();   
//       
        //Reshape(this->width, this-> height);
        //glMatrixMode(GL_MODELVIEW);
        //glLoadIdentity();
        //matrixModelView2.identity();
        //matrixModelView2.rotate( 0,0.0f, 1.0f, 0.0f);
        //matrixModelView2.rotate( -0.0, 0.0f, 0.0f, 1.0f);
        //matrixModelView2.rotate( 0.0f, 1.0f, 0.0f, 0.0f);
        //matrixModelView2.translate(0, 0, -10);
       // plot camera position and location
      
//    glfwPollEvents();
//       
     
     
}
void OpenGLPlot:: Setrot()
{
    
    //glfwMakeContextCurrent(window);
    
    //glLineWidth(4);
    //glfwGetFramebufferSize(window, &this->width, &this->height);
    //float ratio = this->width / (float) this-> height;
    //glEnable(GL_DEPTH_TEST);
    
    glMatrixMode(GL_MODELVIEW);
    double xpos;
    double ypos;
    //glClear(GL_COLOR_BUFFER_BIT);  
    glfwGetCursorPos(window, &xpos, &ypos);
    
    Mouse_r_theta = Mouse_r_theta + ((float)xpos - Mouse_startx)*0.5;
    Mouse_y_theta = Mouse_y_theta + ((float)ypos - Mouse_starty)*0.5;
    
    Mouse_startx =xpos;
    Mouse_starty =ypos;
    
    glBegin(GL_TRIANGLES);
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(-1.6f, -0.4f, -8.f);
    glColor3f(0.f, 1.f, 0.f);
    glVertex3f(1.6f, -0.4f, -8.f);
    glColor3f(0.f, 0.f, 0.f);
    glVertex3f(-1.0f, 0.6f, -8.f);
    glEnd();
  
    matrixModelView.identity();
    //matrixModelView.rotate( Mouse_r_theta, 0.0f, 0.0f, 1.0f);
    //matrixModelView.rotate( 00.0f , 0.0f, 0.0f, 1.0f);
    matrixModelView.rotate( Mouse_y_theta,  1.0f, 0.0f, 0.0f);
    matrixModelView.translate(0.0 ,0.0, -10);
    glRotatef(Mouse_r_theta, 0.f, 0.f, 1.f);
    cout<<"center "<<Mouse_y_theta<<" "<<Mouse_r_theta<<" "<<endl;             
    glLoadMatrixf(matrixModelView.getTranspose());
    glPushMatrix();
    glfwSwapBuffers(window);
    glfwPollEvents();
}
void OpenGLPlot::SetupFrustrum()
{
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    glFrustum (-8.0, 8.0, -5.5, 5.5, 3.0 , 45.5);
        
    glMatrixMode (GL_MODELVIEW);
    glPushMatrix();
}

Matrix4X4 setFrustum(float l, float r, float b, float t, float n, float f)
{
    Matrix4X4 mat;
    mat[0]  =  2 * n / (r - l);
    mat[2]  =  (r + l) / (r - l);
    mat[5]  =  2 * n / (t - b);
    mat[6]  =  (t + b) / (t - b);
    mat[10] = -(f + n) / (f - n);
    mat[11] = -(2 * f * n) / (f - n);
    mat[14] = -1;
    mat[15] =  0;
    return mat;
}
Matrix4X4 setFrustum(float fovY, float aspectRatio, float front, float back)
{
    float tangent = tanf(fovY/2 * DEG2RAD);   // tangent of half fovY
    float height = front * tangent;           // half height of near plane
    float width = height * aspectRatio;       // half width of near plane 
    // params: left, right, bottom, top, near, far
    return setFrustum(-width, width, -height, height, front, back);
}
void OpenGLPlot::Reshape(int height, int width )
{    /* save new screen dimensions */
    //width =  () window_w;
    //height = (GLdouble) window_h;
    /* tell OpenGL to use the whole window for drawing */
    
    glViewport(0, 0, height, width);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    matrixProjection = setFrustum(GlobalzoomFact,(float)width/height , Dnear , Dfar);    // given near and far from point set
    glLoadMatrixf(matrixProjection.getTranspose());
    cout<<"load projection matrix"<<endl;
    //gluPerspective(zoomFact, (GLfloat) width/ (GLfloat) height ,near, far);
    //GLfloat light_position[]={0.0,0.0,0.0,1.0};
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    
}


void OpenGLPlot::PlotCamera(double *t_relative)
{
    glPushMatrix();
    glTranslated(0.0 ,-0.0  ,0.0); 
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(4.6f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f,0.0f);
    glVertex3f(0.0f, 4.6f, 0.0f);
    glColor3f(0, 0, 1); 
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 4.6f);
    glEnd();
    
    glTranslated(t_relative[0] ,t_relative[1]  ,t_relative[2]); 
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(4.6f, 0.0f, 0.0f);
    glColor3f(0, 1, 0);
    glVertex3f(0.0f, 0.0f,0.0f);
    glVertex3f(0.0f, 4.6f, 0.0f);
    glColor3f(0, 0, 1); 
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 4.6f);
    glEnd();
    glPushMatrix();
    glfwSwapBuffers(window);
    //glClear(GL_COLOR_BUFFER_BIT);
   
    
}
 void OpenGLPlot::PlotVertex(int size_, const vector<v3_t> V3Dpts)
{

    glLoadIdentity();
    glPointSize(2.0f);
    glTranslated(0, -5 , 0.0);
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
    //glfwSwapBuffers(window);
    //glfwPollEvents();

}
void OpenGLPlot::PlotVertex2(int size_, const vector<v3_t> V3Dpts)
{

    glBegin(GL_TRIANGLE_FAN);
    glColor3f(1, 0, 0);
    glVertex3f(0, 0.5, 0);
    
    glColor3f(1, 1, 0);
    glVertex3f(-0.5, -0.5, 0.5);
    
    glColor3f(1, 1, 1);
    glVertex3f(0.5, -0.5, 0.5);
    
    glColor3f(0, 1, 1);
    glVertex3f(0.5, -0.5, -0.5);
    
    glColor3f(0, 0, 1);
    glVertex3f(-0.5, -0.5, -0.5);
    
    glColor3f(0, 1, 0);
    glVertex3f(-0.5, -0.5, 0.5);
    glEnd;
    
//    glLoadIdentity();
//    glPointSize(2.0f);
//    glBegin( GL_POINTS);
//    for (int i=0;i<size_;i++)
//    {
//        glColor3f(1, 1, 1); 
//        glVertex3f(V3Dpts[i].p[0],V3Dpts[i].p[1] ,V3Dpts[i].p[2]);
//        //cout<< V3Dpts[i].p[0]<<" "<<V3Dpts[i].p[1]<<" "<<V3Dpts[i].p[2]<<endl;
//    }
//    glEnd();
    glPushMatrix();
//    
    
    //glDisable(GL_DEPTH_TEST);
    //glfwSwapBuffers(window2);
    //glfwPollEvents();
    
}
void OpenGLPlot::ReadObject(const vector<v3_t> V3Dpts)
{
    
        float Max_VX = -9999;
        float Min_Vx = 9999;
        float Max_VY = -9999;
        float Min_VY = 9999;
        float Max_VZ = -9999;
        float Min_VZ = 9999;
        int i;

        int NumTris= (int)V3Dpts.size();
      


    for (i=0; i<NumTris; i++) // read triangles
      {
        //    fscanf(fp, "%f %f %f %f %f %f\n", 
        //               &(Tris[i].v0), &(Tris[i].v1), &(Tris[i].v2), 
        //               &(Tris[i].Color[0]),&(Tris[i].Color[1]), &(Tris[i].Color[2]));
       
        if (V3Dpts[i].p[0]>(Max_VX))
                Max_VX= V3Dpts[i].p[0];
        if (V3Dpts[i].p[0]<(Min_Vx))    
                Min_Vx= V3Dpts[i].p[0];
        
        if (V3Dpts[i].p[1]>(Max_VY))
                    Max_VY= V3Dpts[i].p[1];
        if (V3Dpts[i].p[1]<(Min_VY))    
                    Min_VY= V3Dpts[i].p[1];
        if (V3Dpts[i].p[2]>(Max_VZ))
                    Max_VZ= V3Dpts[i].p[2];
        if (V3Dpts[i].p[2]<(Min_VZ))    
                    Min_VZ= V3Dpts[i].p[2];
    
      }
         center_x= (Min_Vx+Max_VX)/2.0f;
         center_y= (Min_VY+Max_VY)/2.0f;
         center_z= (Min_VZ+Max_VZ)/2.0f;

        float   radius= sqrt((Max_VX - center_x)*(Max_VX -center_x) + (Max_VY - center_y)*(Max_VY - center_y) + (Max_VZ- center_z)*(Max_VZ- center_z));
         f_Distance = radius/1.53;
        float _Distance=radius/2.0;
        if (Max_VZ<0)
        {
            Max_VZ=-Max_VZ;
        }

        double  D_Max_VZ= double(Max_VZ);
        //Distance = _Distance-D_Max_VZ;
        //if (Distance<0)
        //{
        //    Distance= -Distance;
        //}
        double  D_Max_VY= double((Max_VY-Min_VY)/2);
        //double theta = 120*(2.0*atan2(D_Max_VY,_Distance))/M_pi;

        Dnear = f_Distance - radius;
        Dfar  = f_Distance + radius;


}
void OpenGLPlot::SetCamera()
{

    glLoadIdentity();
    
       
//    matrixModelView.identity();
//    matrixModelView.rotate(90 , 0.0f, 1.0f, 0.0f);
//    matrixModelView.rotate(Mouse_r_theta, 0.0f, 0.0f, 1.0f);
//    //matrixView.rotate(p_theta, 1.0f, 0.0f, 0.0f);
//    matrixModelView.translate(0, 0, -f_Distance-50);
//    cout<<"model view matrix"<<endl;
//    cout<<matrixModelView[0]<<" "<<matrixModelView[1]<<" "<<matrixModelView[2]<<" "<<matrixModelView[3]<<endl;  
//    cout<<matrixModelView[4]<<" "<<matrixModelView[5]<<" "<<matrixModelView[6]<<" "<<matrixModelView[7]<<endl; 
//    cout<<matrixModelView[8]<<" "<<matrixModelView[9]<<" "<<matrixModelView[10]<<" "<<matrixModelView[11]<<endl; 
//    cout<< "f_distance "<< f_Distance<<endl;
    float tm[16];
    
    tm[0] = 1.0f;   tm[1] = 0.0f;   tm[2] = 0.0f;   tm[3] = 0.0f;
    tm[4] = 0.0f;   tm[5] = 0.5f;   tm[6] = 0.0f;   tm[7] = 0.0f;
    tm[8] = 0.0f;   tm[9] = 0.0f;   tm[10]=  1.0f;   tm[11]= 0.0f;
    tm[12]= -5.0f;   tm[13]= -0.0f; tm[14]= -55.0f;  tm[15]= 1.0f;

   
    //glLoadMatrixf(matrixModelView.getTranspose());           
     glLoadMatrixf(tm);  
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