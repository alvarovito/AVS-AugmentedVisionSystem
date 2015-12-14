#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pruebaListener");
  ros::NodeHandle nodo;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nodo);
  image_transport::Subscriber sub = it.subscribe("camera/imageLeft", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}