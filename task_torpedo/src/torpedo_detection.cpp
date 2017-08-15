// Copyright 2016 AUV-IITK
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <task_torpedo/torpedoConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

using namespace cv;


int w = -2, x = -2, y = -2, z = -2; // to change
bool IP = true; // indication for running the node or not
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max; // values used for thresholding according to the object under detection

cv::Mat frame; // takes the original image
cv::Mat newframe; // takes the original image from the topic
cv::Mat dst;
cv::Mat dst_array;

int count = 0, count_avg = 0, p = -1;

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

void torpedoListener(std_msgs::Bool msg)
{
  IP = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) // called everytime when something is published on the topic
{
  if (p == 32)
    return;
  try
  {
    count++;
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    ///////////////////////////// DO NOT REMOVE THIS, IT COULD BE INGERIOUS TO HEALTH /////////////////////
    newframe.copyTo(frame);
    ////////////////////////// FATAL ///////////////////////////////////////////////////
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

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

int main(int argc, char *argv[])
{
  int height, width, step, channels;  // parameters of the image we are working on

  ros::init(argc, argv, "torpedo_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/torpedo", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("torpedo_detection_switch", 1000, &torpedoListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub = it.advertise("/varun/sensors/first_camera/image_raw", 1);
  image_transport::Publisher pub1 = it.advertise("/varun/sensors/second_camera/image_raw", 1);
  image_transport::Publisher pub2 = it.advertise("/varun/sensors/third_camera/image_raw", 1);
  
  dynamic_reconfigure::Server<task_torpedo::torpedoConfig> server;
  dynamic_reconfigure::Server<task_torpedo::torpedoConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cvNamedWindow("TorpedoDetection:Original", CV_WINDOW_NORMAL);
  cvNamedWindow("TorpedoDetection:AfterEnhancing", CV_WINDOW_NORMAL);
  cvNamedWindow("TorpedoDetection:AfterThresholding", CV_WINDOW_NORMAL);
  // cvNamedWindow("TorpedoDetection:AfterHistogramEqualization", CV_WINDOW_NORMAL);
  // cvNamedWindow("TorpedoDetection:AfterMorphology",CV_WINDOW_NORMAL);

  // capture size -
  CvSize size = cvSize(width, height);
  std::vector<cv::Point2f> center_ideal(5);
  float side[5];

  for(int i = 0; i < 5; i++){
    sides[i] = 0;
  }

  // Initialize different images that are going to be used in the program
  cv::Mat cv::Mat lab_image, balanced_image1, dstx, thresholded, image_clahe, dst;
  std::vector<cv::Mat> lab_planes(3);
  // image converted to HSV plane
  // asking for the minimum distance where bwe fire torpedo

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
    // Get one frame
    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;

    cv::cvtColor(frame, lab_image, CV_BGR2Lab);

    // Extract the L channel
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    cv::Mat image_clahe;
    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);
    
    for (int i=0; i < 7; i++)
    {
      bilateralFilter(image_clahe, dstx, 6, 8, 8);
      bilateralFilter(dstx, image_clahe, 6, 8, 8);
    }
    // balance_white(dst2);
    
    image_clahe.copyTo(balanced_image1);
    balance_white(balanced_image1);
    
    for (int i=0; i < 2; i++)
    {
      bilateralFilter(balanced_image1, dstx, 6, 8, 8);
      bilateralFilter(dstx, balanced_image1, 6, 8, 8);
    }
        
    // Filter out colors which are out of range.

    cv::inRange(balanced_image1, hsv_min, hsv_max, thresholded);
   
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    cv::imshow("TorpedoDetection:AfterEnhancing", balanced_image1);
    cv::imshow("TorpedoDetection:AfterThresholding", thresholded);

    if ((cvWaitKey(10) & 255) == 27)
      break;

    if (!IP)
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours in the image
      double largest_area = 0, largest_contour_index = 0;
      double second_largest_area = 0, second_largest_contour_index = 0;

      if (contours.empty())
      {
        int x_cord = 320 - center_ideal[0].x;
        int y_cord = -240 + center_ideal[0].y;
        if (x_cord < -270)
        {
          array.data.push_back(-2);  // top
          array.data.push_back(-2);
          array.data.push_back(-2);
          array.data.push_back(-2);
          pub.publish(array);
        }
        else if (x_cord > 270)
        {
          array.data.push_back(-1);  // left_side
          array.data.push_back(-1);
          array.data.push_back(-1);
          array.data.push_back(-1);
          pub.publish(array);
        }
        else if (y_cord > 200)
        {
          array.data.push_back(-3);  // bottom
          array.data.push_back(-3);
          array.data.push_back(-3);
          array.data.push_back(-3);
          pub.publish(array);
        }
        else if (y_cord < -200)
        {
          array.data.push_back(-4);  // right_side
          array.data.push_back(-4);
          array.data.push_back(-4);
          array.data.push_back(-4);
          pub.publish(array);
        }
        // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        // remove higher bits using AND operator
        if ((cvWaitKey(10) & 255) == 27)
          break;
        continue;
      }

      for (int i = 0; i < contours.size(); i++)  // iterate through each contour.
      {
        double a = contourArea(contours[i], false);  //  Find the area of contour
        if (a > largest_area)
        {
          largest_area = a;
          largest_contour_index = i;  // Store the index of largest contour
        }
      }

      second_largest_contour_index = largest_contour_index;

      for (int i = contours.size()-1; i >= 0; i--)
      {
        double a = contourArea(contours[i], false);
        if (a <= 0.75*largest_area)
        {
          second_largest_area = a;
          second_largest_contour_index = i;
          break;
        }
      }
      // Convex HULL
      std::vector<std::vector<cv::Point> > hull(contours.size());
      convexHull(cv::Mat(contours[largest_contour_index]), hull[largest_contour_index], false);

      cv::Mat Drawing(thresholded_Mat.rows, thresholded_Mat.cols, CV_8UC1, cv::Scalar::all(0));
      std::vector<cv::Vec4i> hierarchy;
      cv::Scalar color(255, 255, 255);

      std::vector<cv::RotatedRect> boundRect(2);
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      

      boundRect[0] = MinAreaRect(cv::Mat(contours[largest_contour_index]));
      boundRect[1] = MinAreaRect(cv::Mat(contours[second_largest_contour_index])); // finding a min fitting rectangle around the heart shape

      Point2f rect_points0[4];
      Point2f rect_points1[4];

      boundRect[0].points(rect_points0); // get the points of the two rectangles formed
      boundRect[1].points(rect_points1);

      
      // not shown the Drawing Mat
      // rectangle(Drawing, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0); // drawing a rectangle in the image
      // rectangle(Drawing, boundRect[1].tl(), boundRect[1].br(), color, 2, 8, 0); // drawing a rectangle in the image 

      cv::Point2f center;
      center.x = (rect_points0[0].x + rect_points0[1].x + rect_points0[2].x + rect_points0[3].x) / 4; // center of the rectangle of the largest contour 
      center.y = (rect_points0[0].y + rect_points0[1].y + rect_points0[2].y + rect_points0[3].y) / 4;

      float sides0[4]; // length of the sides of the rectangle formed of the largest contour 

      for(int i = 0; i < 4; i++){

        float distance = (rect_points0[i].x - rect_points0[(i+1)%4])*(rect_points0[i].x - rect_points0[(i+1)%4]) + (rect_points0[i].y - rect_points0[(i+1)%4])*(rect_points0[i].y - rect_points0[(i+1)%4].y);
        sides0[i] = pow(disatnce,0.5);

      }


      // int side_x = (boundRect[0].br()).x - (boundRect[0].tl()).x; // change the sides
      // int side_y = -((boundRect[0].tl()).y - (boundRect[0].br()).y);
      drawContours(Drawing, contours, largest_contour_index, color, 2, 8, hierarchy);

      cv::Point2f heart_center; // center of the second largest rectangle
      heart_center.x = (rect_points1[0].x + rect_points1[1].x + rect_points1[2].x + rect_points1[3].x) / 4;
      heart_center.y = (rect_points1[0].x + rect_points1[1].x + rect_points1[2].x + rect_points1[3].x) / 4
      // int heart_side_x = (boundRect[1].br()).x - (boundRect[1].tl()).x; // change the sides
      // int heart_side_y = -((boundRect[1].tl()).y - (boundRect[1].br()).y);
      drawContours(Drawing, contours, second_largest_contour_index, color, 2, 8, hierarchy);

      cv::Mat frame_mat = frame;
      cv::Point2f screen_center;
      screen_center.x = 320;  // size of my screen
      screen_center.y = 240;

      circle(frame_mat, center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
      circle(frame_mat, heart_center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);

      for( int j = 0; j < 4; j++ ) // drawing the largest rectangle obtained 
          line( frame_mat, rect_points0[j], rect_points0[(j+1)%4], color, 1, 8 );
    
      for( int j = 0; j < 4; j++ ) // drawing the second largest rectangle obtained 
          line( frame_mat, rect_points1[j], rect_points1[(j+1)%4], color, 1, 8 );
      
      float sides1[4]; // contains the side of the rectangle around the torpedo

      for(int i = 0; i < 4; i++){

        float distance = (rect_points1[i].x - rect_points1[(i+1)%4])*(rect_points1[i].x - rect_points1[(i+1)%4]) + (rect_points1[i].y - rect_points1[(i+1)%4])*(rect_points1[i].y - rect_points1[(i+1)%4].y);
        sides1[i] = pow(disatnce,0.5);

      }

      float radius[1];

      radius[0] = (sides0[0] + sides0[1] + sides0[2] + sides0[3] + sides1[0] + sides1[1] + sides1[2] + sides1[3]) / 8 // average of all the sides of the rectangles to get a approx. radius to use
    
      // rectangle(frame_mat, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0);
      // rectangle(frame_mat, boundRect[1].tl(), boundRect[1].br(), color, 2, 8, 0);
      circle(frame_mat, screen_center, 4, cv::Scalar(150, 150, 150), -1, 8, 0);  // center of screen

      int x_cord_heart = -(heart_center.x - 320);
      int y_cord_heart = -240 + heart_center.y;

      int x_cord = -(center.x - 320);
      int y_cord = -240 + center.y;

      cv::Point2f center_avg;

      center_avg[0].x = (center_ideal[0].x + center_ideal[1].x + center_ideal[2].x + center_ideal[3].x + center_ideal[4].x) / 5;
      center_avg[0].y = (center_ideal[0].y + center_ideal[1].y + center_ideal[2].y + center_ideal[3].y + center_ideal[4].y) / 5;
      
      float side_avg = (side[0] + side[1] + side[2] + side[3] + side[4]) / 5;
      if ((radius[0] < (side_avg + 10)) && (count_avg >= 5))
      {
        side[4] = side[3];
        side[3] = side[2];
        side[2] = side[1];
        side[1] = side[0];
        side[0] = radius[0];
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = heart_center[0];
        count_avg++;
      }
      else if (count_avg <= 5)
      {
        side[count_avg] = radius[0];
        center_ideal[count_avg] = center[0];
        count_avg++;
      }
      else
      {
        count_avg = 0;
      }

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame).toImageMsg();

      pub.publish(msg);
      pub1.publish(msg1);
      pub2.publish(msg2);
      
      
      // cv::Mat circles = frame;
      // circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      // circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
      // circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

      int net_x_cord = 320 - center_ideal[0].x + radius[0]; // put the sides in place of radius
      int net_y_cord = -240 + center_ideal[0].y + radius[0];
      if (net_x_cord < -310)
      {
        array.data.push_back(-2);  // top
        array.data.push_back(-2);
        array.data.push_back(-2);
        array.data.push_back(-2);
        pub.publish(array);
      }
      else if (net_x_cord > 310)
      {
        array.data.push_back(-1);  // left_side
        array.data.push_back(-1);
        array.data.push_back(-1);
        array.data.push_back(-1);
        pub.publish(array);
        ros::spinOnce();
      }
      else if (net_y_cord > 230)
      {
        array.data.push_back(-3);  // bottom
        array.data.push_back(-3);
        array.data.push_back(-3);
        array.data.push_back(-3);
        pub.publish(array);
      }
      else if (net_y_cord < -230)
      {
        array.data.push_back(-4);  // right_side
        array.data.push_back(-4);
        array.data.push_back(-4);
        array.data.push_back(-4);
        pub.publish(array);
      }
      else
      {
        float distance;
        distance = pow(radius[0] / 7526.5, -.92678);  // function found using experiment
        array.data.push_back(r[0]);                   // publish radius
        array.data.push_back((320 - center_ideal[0].x));
        array.data.push_back(-(240 - center_ideal[0].y));
        array.data.push_back(distance);
        pub.publish(array);
      }

      pub.publish(array);
      ros::spinOnce();

      // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
      // remove higher bits using AND operator
      if ((cvWaitKey(10) & 255) == 27)
        break;
    }
    else
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      if ((cvWaitKey(10) & 255) == 32)
      {
        if (p == 32)
          p = -1;
        else
          p = 32;
      }
      if (p == 32)
        ROS_INFO("%s: PAUSED\n", ros::this_node::getName().c_str());
      ros::spinOnce();
    }
  }
  return 0;
}

