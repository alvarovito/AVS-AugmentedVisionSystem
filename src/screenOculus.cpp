#include <stdio.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

//#include <opencv2/opencv.hpp>
#include <opencv/cv.hpp>

#include "Plano.h"
#include "ros/ros.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>
#include "augmentedVision/twoImages.h"
#include "augmentedVision/rectanglesVector.h"
#include "augmentedVision/rectangle.h"

using namespace cv;

GLFWwindow* window;
Plano lPlane, rPlane;

Mat leftFrame,rightFrame, totalImage;
bool activo=false;
bool activoRight=false;
bool activoLeft=false;
bool cerrar=false;

vector<Rect> leftFaces, rightFaces;

void createWindow()
{
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    //window = glfwCreateWindow(800, 600, "OpenGL", NULL, NULL); // Windowed
    //window = glfwCreateWindow(1920, 1080, "OpenGL", NULL, NULL); // Fullscreen
    window = glfwCreateWindow(1920, 1080, "OpenGL", glfwGetPrimaryMonitor(), NULL); // Fullscreen
}

void activateOpenGL()
{
    glfwMakeContextCurrent(window);

    glewExperimental = GL_TRUE;
    glewInit();
}

void close()
{
    lPlane.close();
    rPlane.close();

    glfwTerminate();
}

void drawScene()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnableVertexAttribArray(0);

    lPlane.applyTransformations();
    lPlane.drawTexture();
    rPlane.applyTransformations();
    rPlane.drawTexture();
}

void updateImages(const augmentedVision::twoImages::ConstPtr& msg)
{
  try
  {
    leftFrame = cv_bridge::toCvCopy( msg->leftimage, "bgr8")->image;
    //lPlane.updateTexture(leftFrame);
    rightFrame = cv_bridge::toCvCopy( msg->rightimage, "bgr8")->image;
    //rPlane.updateTexture(rightFrame);
    //sensor_msgs::Image bla = msg->leftimage;
    //totalImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    //leftFrame = cv_bridge::toCvShare(&bla, "bgr8").image;
    //rightFrame = cv_bridge::toCvShare(msg->rightimage, "bgr8");

    /*cvtColor(leftFrame, leftFrame, CV_BGR2GRAY);
    Canny(leftFrame, leftFrame, 100, 300, 3);
    cvtColor(leftFrame, leftFrame, CV_GRAY2BGR);*/

    if(!activo)
    {
        glfwInit();

        createWindow();
        activateOpenGL();

        //leftFrame = cv_bridge::toCvCopy( msg->leftimage, "bgr8")->image;
        //rightFrame = cv_bridge::toCvCopy( msg->rightimage, "bgr8")->image;

        lPlane.initialize(1);
        lPlane.createTexture(leftFrame);
        lPlane.createShader();
        lPlane.initializeUniforms();
        rPlane.initialize(2);
        rPlane.createTexture(rightFrame);
        rPlane.createShader();
        rPlane.initializeUniforms();
        //ros::Duration(1).sleep();
        activo=true;
    }
    //leftFrame = cv_bridge::toCvCopy( msg->leftimage, "bgr8")->image;
    //lPlane.updateTexture(cv_bridge::toCvCopy( msg->leftimage, "bgr8")->image);
    //rightFrame = cv_bridge::toCvCopy( msg->rightimage, "bgr8")->image;
    //rPlane.updateTexture(cv_bridge::toCvCopy( msg->rightimage, "bgr8")->image);
    /*if(!activoLeft)
    {
        lPlane.initialize(1);
        lPlane.createTexture(leftFrame);
        lPlane.createShader();
        activoLeft=true;
        rPlane.initialize(2);
        rPlane.createTexture(rightFrame);
        rPlane.createShader();
        activoRight=true;
    }*/
    //cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void drawFaces(const augmentedVision::rectanglesVector::ConstPtr& msg)
{
    leftFaces.clear();
    for (int i = 0;i < msg->rectanglesLeft.size(); i++) 
    {
        Rect face(msg->rectanglesLeft[i].x,msg->rectanglesLeft[i].y,msg->rectanglesLeft[i].width, msg->rectanglesLeft[i].height);
        leftFaces.push_back(face);
    }

    rightFaces.clear();
    for (int i = 0;i < msg->rectanglesRight.size(); i++) 
    {
        Rect face(msg->rectanglesRight[i].x,msg->rectanglesRight[i].y,msg->rectanglesRight[i].width, msg->rectanglesRight[i].height);
        rightFaces.push_back(face);
    }
}

/*void imageRight(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    rightFrame = cv_bridge::toCvShare(msg, "bgr8")->image;
    if(!activo)
    {
        glfwInit();

        createWindow();
        activateOpenGL();
        usleep(1000);
        activo=true;
    }
    if(!activoRight)
    {
        rPlane.initialize(2);
        rPlane.createTexture(rightFrame);
        rPlane.createShader();
        activoRight=true;
    }
    //cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}*/


int main(int argc, char** argv)
{
    ros::init(argc, argv, "screenOculus_nodo");
    ros::NodeHandle nodo;
    ROS_INFO("Nodo screenOculus creado y registrado");
    //image_transport::ImageTransport it(nodo);
    //image_transport::ImageTransport it2(nodo);
    //image_transport::Subscriber sub = it.subscribe("imagenesCapturadas", 0, recortarImagen);
    ros::Subscriber subscriptor = nodo.subscribe("imagenesCapturadas", 0, updateImages);
    ros::Subscriber subscriptorFaces = nodo.subscribe("faces", 0, drawFaces);
    //image_transport::Subscriber sub2 = it.subscribe("imageRight", 0, imageRight);
    /*Mat leftFrame,rightFrame;
    VideoCapture videoLeft(0);
    VideoCapture videoRight(1);

    if (!videoLeft.isOpened())
    {
        printf("Error al inicializar cámara left");
        return 1;
    }
    if (!videoRight.isOpened())
    {
        printf("Error al inicializar cámara right");
        return 1;
    }

    videoLeft >> leftFrame;
    videoRight >> rightFrame;*/

    /*glfwInit();

    createWindow();
    activateOpenGL();

    lPlane.initialize(1);
    lPlane.createTexture(leftFrame);
    lPlane.createShader();

    rPlane.initialize(2);
    rPlane.createTexture(rightFrame);
    rPlane.createShader();*/

    //glEnable(GL_DEPTH_TEST);

    /*ros::init(argc, argv, "screenOculus_nodo");
    ros::NodeHandle nodo;
    ROS_INFO("Nodo screenOculus creado y registrado");*/

    ros::Duration rate(0.02);

    while(ros::ok() && !cerrar)
    {
        if(activo)
        //while (activo && !glfwWindowShouldClose(window))
        {
            /*videoLeft.grab();
            videoLeft.retrieve(leftFrame, CV_CAP_OPENNI_BGR_IMAGE);

            videoRight.grab();
            videoRight.retrieve(rightFrame, CV_CAP_OPENNI_BGR_IMAGE);*/

            /*videoLeft >> leftFrame;
            videoRight >> rightFrame;*/

            for (int i = 0;i < leftFaces.size(); i++) 
            {
                rectangle(leftFrame, Point(leftFaces[i].x, leftFaces[i].y),
                    Point(leftFaces[i].width, leftFaces[i].height),
                    Scalar(0, 255, 0), 2);
            }
            for (int i = 0;i < rightFaces.size(); i++) 
            {
                rectangle(rightFrame, Point(rightFaces[i].x, rightFaces[i].y),
                    Point(rightFaces[i].width, rightFaces[i].height),
                    Scalar(0, 255, 0), 2);
            }

            lPlane.updateTexture(leftFrame);
            rPlane.updateTexture(rightFrame);

            glfwSwapBuffers(window);
            glfwPollEvents();

            if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            {
                cerrar=true;
                glfwSetWindowShouldClose(window, GL_TRUE);
            }

            drawScene();

            /*ros::spinOnce();

            rate.sleep();*/
        }

        ros::spinOnce();

        rate.sleep();
    }
    close();
    return 0;
}
