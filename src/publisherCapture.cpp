#include "ros/ros.h"

#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>

#include "augmentedVision/twoImages.h"

using namespace cv;

Mat3b leftFrame,rightFrame, firstLeft, firstRight;

bool salir=false;

void * functionCapture(void * var)
{
    //VideoCapture videoLeft(0);
    VideoCapture videoRight(1);

    /*if (!videoLeft.isOpened())
    {
        printf("Error al inicializar cámara left");
        salir=true;
    }*/
    if (!videoRight.isOpened())
    {
        printf("Error al inicializar cámara right");
        salir=true;
    }

    while(!salir)
    {
        //videoLeft >> leftFrame;
        videoRight >> firstRight;
        transpose(firstRight, rightFrame);
        //imshow("bla", leftFrame);
    }

    //videoLeft.release();
    videoRight.release();
    
    return NULL;
}

void * functionCapture2(void * var)
{
    VideoCapture videoLeft(2);
    //VideoCapture videoRight(1);

    if (!videoLeft.isOpened())
    {
        printf("Error al inicializar cámara left");
        salir=true;
    }
    /*if (!videoRight.isOpened())
    {
        printf("Error al inicializar cámara right");
        salir=true;
    }*/

    while(!salir)
    {
        videoLeft >> firstLeft;
        transpose(firstLeft, leftFrame);
        //flip(leftFrame, leftFrame, 90);
        //videoRight >> rightFrame;

        //imshow("bla", leftFrame);
    }

    videoLeft.release();
    //videoRight.release();
    
    return NULL;
}


int main(int argc, char** argv)
{
    pthread_t mythread, mythread2;
    pthread_create(&mythread, NULL, functionCapture, NULL);
    pthread_create(&mythread2, NULL, functionCapture2, NULL);

    ros::init(argc, argv, "publisherCapture");
    ros::NodeHandle nodo;
    ROS_INFO("Nodo que captura las imágenes creado y registrado");


    image_transport::ImageTransport it(nodo);
	//image_transport::Publisher pubLeft = it.advertise("imageLeft", 0);
  	//image_transport::Publisher pubRight = it.advertise("imageRight", 0);

    ros::Publisher publicador = nodo.advertise<augmentedVision::twoImages>("imagenesCapturadas", 0);

/*

    ros::Publisher publicadorImagenLeft = nodo.advertise<std_msgs::String>("imagenLeft_topic", 0);
    ros::Publisher publicadorImagenRight = nodo.advertise<std_msgs::String>("imagenRight_topic", 0);
*/
	/*Mat leftFrame,rightFrame, totalFrame;
    VideoCapture videoLeft(0);
    VideoCapture videoRight(1);

    if (!videoLeft.isOpened() && !videoRight.isOpened())
    {
        printf("Error al inicializar cámara left");
        return 1;
    }*/
    /*if (!videoRight.isOpened())
    {
        printf("Error al inicializar cámara right");
        return 1;
    }*/

    //usleep(50000);

    sensor_msgs::ImagePtr msgLeft;
    sensor_msgs::ImagePtr msgRight;

    augmentedVision::twoImages msg;
    

    ros::Duration rate(0.02);

    
    while(ros::ok())
    {
    	//videoLeft >> leftFrame;
    	//videoRight >> rightFrame;

        //usleep(50000);

    	if(!leftFrame.empty() && !rightFrame.empty())
    	{
            /*leftFrame.copyTo(totalImage(Rect(0,0,leftFrame.cols, leftFrame.rows)));
            rightFrame.copyTo(totalImage(Rect(leftFrame.cols, 0, leftFrame.cols, leftFrame.rows)));
*/
            //if(!leftFrame.empty() && !rightFrame.empty())
            //hconcat(leftFrame, rightFrame, totalFrame);
            //IplImage *img = new IplImage(totalFrame);

    		msgLeft = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftFrame).toImageMsg();
	    	//pubLeft.publish(msgLeft);

            msgRight = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightFrame).toImageMsg();
	    	//pubRight.publish(msgRight);

            msg.leftimage = *msgLeft;
            msg.rightimage = *msgRight;

            publicador.publish(msg);

            //printf("Paso\n");

            //imshow("bla", leftFrame);

	    	//cv::waitKey(1);
		}
    	/*publicadorImagenLeft.publish(leftFrame);
        ROS_INFO("Mensaje imagenLeft enviado");
        publicadorImagenRight.publish(rightFrame);
        ROS_INFO("Mensaje imagenLeft enviado");*/
        
        ros::spinOnce();

		rate.sleep();
    }

    salir=true;

    ros::Duration(1).sleep();

    leftFrame.release();
    rightFrame.release();

    //videoLeft.release();
    //videoRight.release();

    //pthread_exit(NULL);

    return 0;
}
