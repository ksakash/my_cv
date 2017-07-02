#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <task_torpedo/torpedoConfig.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float32MultiArray.h"
#include <opencv/highgui.h>
//#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
//#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

int t1min, t1max, t2min, t2max, t3min, t3max; // values used for thresholding according to the object under detection

using namespace cv;
cv::Mat frame, newframe;
void balance_white(cv::Mat mat) {
  double discard_ratio = 0.05;
  int hists[3][256];
  memset(hists, 0, 3*256*sizeof(int));

  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        hists[j][ptr[x * 3 + j]] += 1;
      }
    }
  }

  // cumulative hist
  int total = mat.cols*mat.rows;
  int vmin[3], vmax[3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 255; ++j) {
      hists[i][j + 1] += hists[i][j];
    }
    vmin[i] = 0;
    vmax[i] = 255;
    while (hists[i][vmin[i]] < discard_ratio * total)
      vmin[i] += 1;
    while (hists[i][vmax[i]] > (1 - discard_ratio) * total)
      vmax[i] -= 1;
    if (vmax[i] < 255 - 1)
      vmax[i] += 1;
  }


  for (int y = 0; y < mat.rows; ++y) {
    uchar* ptr = mat.ptr<uchar>(y);
    for (int x = 0; x < mat.cols; ++x) {
      for (int j = 0; j < 3; ++j) {
        int val = ptr[x * 3 + j];
        if (val < vmin[j])
          val = vmin[j];
        if (val > vmax[j])
          val = vmax[j];
        ptr[x * 3 + j] = static_cast<uchar>((val - vmin[j]) * 255.0 / (vmax[j] - vmin[j]));
      }
    }
  }
}
void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    ///////////////////////////// DO NOT REMOVE THIS, IT COULD BE INGERIOUS TO HEALTH /////////////////////
    newframe.copyTo(frame);
    ////////////////////////// FATAL ///////////////////////////////////////////////////
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}
 
void callback(task_torpedo::torpedoConfig &config, uint32_t level) // called when the parameters are changed
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("Torpedo_Reconfigure Request : New parameters : %d %d %d %d %d %d ", t1min, t1max, t2min, t2max, t3min, t3max);
}
int main(int argc, char** argv)
{
    /*VideoCapture cap(1);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(1))
        return 0;*/
     
     /*cap.set(CV_CAP_PROP_BRIGHTNESS, 10);
     cap.set(CV_CAP_PROP_CONTRAST, 10);
     cap.set(CV_CAP_PROP_SATURATION, 10);
     cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0.2);
  //cap.set(CV_CAP_PROP_EXPOSURE, 0.1);*/
  
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  cv::Mat frame = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  //image_transport::Subscriber sub = it.subscribe("/varun/sensors/front_camera/image_raw",1000,imageCallback);
  image_transport::Publisher pub = it.advertise("/varun/sensors/first_camera/image_raw", 1);
  image_transport::Publisher pub1 = it.advertise("/varun/sensors/second_camera/image_raw", 1);
  image_transport::Publisher pub2 = it.advertise("/varun/sensors/third_camera/image_raw", 1);
  image_transport::Publisher pub3 = it.advertise("/varun/sensors/fourth_camera/image_raw", 1);
  image_transport::Publisher pub4 = it.advertise("/varun/sensors/fifth_camera/image_raw", 1);
  image_transport::Publisher pub5 = it.advertise("/varun/sensors/sixth_camera/image_raw", 1);
  image_transport::Publisher pub6 = it.advertise("/varun/sensors/seventh_camera/image_raw", 1);
  
  //cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  ros::Rate loop_rate(5);

  dynamic_reconfigure::Server<task_torpedo::torpedoConfig> server;
  dynamic_reconfigure::Server<task_torpedo::torpedoConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

    
    while(nh.ok())
    {
            //std::cout << exposure << std::endl;
   /* std::cout << cap.get(CV_CAP_PROP_BRIGHTNESS) << std::endl;
    std::cout << cap.get(CV_CAP_PROP_CONTRAST) << std::endl;
    std::cout << cap.get(CV_CAP_PROP_SATURATION) << std::endl;
    std::cout << cap.get(CV_CAP_PROP_AUTO_EXPOSURE) << std::endl;
    //std::cout << cap.get(CV_CAP_PROP_EXPOSURE) << std::endl;
    //std::cout << cap.get(CV_CAP_PROP_HUE) << std::endl;*/
     // Mat frame; 2017-07-02 21:57:20 ⌚  akash-X556UQK in ~/bagfiles

      //cap >> frame;
    if (frame.empty())
    {
     
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }
    cv::Scalar hsv_min = cv::Scalar(0, 0, 100, 0);
    cv::Scalar hsv_max = cv::Scalar(0, 0, 257, 0);
    cv::Mat thresholded;

cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
      cv::Mat lab_image;
      cv::Mat balanced_image;
      /*cv::Mat white_balance;
      balanceWhite(frame, white_balance, WHITE_BALANCE_SIMPLE, 0, 255, 0, 255);*/
      frame.copyTo(balanced_image);
      balance_white(balanced_image);
      cv::Mat dst1;
      fastNlMeansDenoisingColored(balanced_image, balanced_image, 3, 7, 21);
      bilateralFilter(balanced_image, dst1, 4, 8, 8);
      	
      cv::cvtColor(frame, lab_image, CV_BGR2Lab);

      // Extract the L channel
      std::vector<cv::Mat> lab_planes(3);
        cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

   // convert back to RGB
   Mat image_clahe;
   cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);
cv::Mat balanced_image1, dst2;
      
      fastNlMeansDenoisingColored(image_clahe, image_clahe, 3, 7, 21);
      bilateralFilter(image_clahe, dst2, 4, 8, 8);
      balance_white(dst2);
      image_clahe.copyTo(balanced_image1);
      balance_white(balanced_image1);
      cv::Mat dst3;
      fastNlMeansDenoisingColored(balanced_image1, balanced_image1, 3, 7, 21);
      inpaint(balanced_image1,  Mat::zeros(balanced_image1.size(), CV_8U), dst3, 3, INPAINT_TELEA);
    // Filter out colors which are out of range.

    cv::inRange(dst3, hsv_min, hsv_max, thresholded);
    //cv::Mat dst4;
   //bilateralFilter(thresholded, dst4, 4, 8, 8)


      //bilateralFilter(balanced_image1, dst2, 4, 8, 8);
      //balance_white(balanced_image1);
      //cv::Mat dst4;
   //bilateralFilter(thresholded, dst4, 20, 20, 20);
   //cv::Mat dst4;
   //bilateralFilter(thresholded, dst4, 4, 8, 8)
 //morphological opening (remove small objects from the foreground)
    //cv::erode(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
   cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    //morphological closing (fill small holes in the foreground)
    //cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    //cv::erode(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));


      
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_clahe).toImageMsg();
      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst1).toImageMsg();
      sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
      sensor_msgs::ImagePtr msg4 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst3).toImageMsg();
      sensor_msgs::ImagePtr msg5 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst2).toImageMsg();
      sensor_msgs::ImagePtr msg6 = cv_bridge::CvImage(std_msgs::Header(),"mono8" ,thresholded).toImageMsg();
      
      if( frame.empty() ) break; // end of video stream
      
      pub.publish(msg);
      pub1.publish(msg1);
      pub2.publish(msg2);
      pub3.publish(msg3);
      pub4.publish(msg4);
      pub5.publish(msg5);
      pub6.publish(msg6);
      
      ros::spinOnce();
      loop_rate.sleep();
       // stop capturing by pressing ESC 
    }
    // the camera will be closed automatically upon exit
     //cap.close();
    return 0;
}
