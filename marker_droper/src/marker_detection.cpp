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
#include <task_marker/markerConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

int w = -2, x = -2, y = -2, z = -2;
bool IP = true;
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max;

cv::Mat frame;
cv::Mat newframe;
int count = 0, count_avg = 0, p = -1;

void callback(task_marker::markerConfig &config, uint32_t level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  ROS_INFO("Marker_Reconfigure Request : New parameters : %d %d %d %d %d %d ", t1min, t1max, t2min, t2max, t3min, t3max);
}

void markerListener(std_msgs::Bool msg)
{
  IP = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
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
  
  ros::init(argc, argv, "marker_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/marker", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("marker_detection_switch", 1000, &markerListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);

  dynamic_reconfigure::Server<task_marker::markerConfig> server;
  dynamic_reconfigure::Server<task_marker::markerConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cvNamedWindow("MarkerDetection:OriginalImage", CV_WINDOW_NORMAL);
  cvNamedWindow("MarkerDetection:AfterEnhancing", CV_WINDOW_NORMAL);
  cvNamedWindow("MarkerDetection:AfterThrsholding", CV_WINDOW_NORMAL);
  
  // capture size -
  CvSize size = cvSize(width, height);
  std::vector<cv::Point2f> center_ideal(5);

  cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
  cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);


  cv::Mat lab_image, balanced_image1, dstx, thresholded, image_clahe, dst;
  std::vector<cv::Mat> lab_planes(3);


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

    cv::imshow("MarkerDetection:AfterEnhancing", balanced_image1);
    cv::imshow("MarkerDetection:AfterThresholding", thresholded);

    if ((cvWaitKey(10) & 255) == 27)
      break;

    if (!IP)
    {
      // find contours	
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded;
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
      if (contours.empty())
      {
        int x_cord = 320 - center_ideal[0].x;
        int y_cord = -240 + center_ideal[0].y;
        if (x_cord < -270)
        {
          array.data.push_back(-2);  // top
          array.data.push_back(-2);
          // array.data.push_back(-2);
          // array.data.push_back(-2);
          pub.publish(array);
        }
        else if (x_cord > 270)
        {
          array.data.push_back(-1);  // left_side
          array.data.push_back(-1);
          // array.data.push_back(-1);
          // array.data.push_back(-1);
          pub.publish(array);
        }
        else if (y_cord > 200)
        {
          array.data.push_back(-3);  // bottom
          array.data.push_back(-3);
          // array.data.push_back(-3);
          // array.data.push_back(-3);
          pub.publish(array);
        }
        else if (y_cord < -200)
        {
          array.data.push_back(-4);  // right_side
          array.data.push_back(-4);
          // array.data.push_back(-4);
          // array.data.push_back(-4);
          pub.publish(array);
        }
        ros::spinOnce();
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
      // Convex HULL
      std::vector<std::vector<cv::Point> > hull(contours.size());
      convexHull(cv::Mat(contours[largest_contour_index]), hull[largest_contour_index], false);

      std::vector<cv::Point2f> center(1);
      cv::minEnclosingCircle(contours[largest_contour_index], center[0], radius[0]);
      

      cv::Mat Drawing(thresholded_Mat.rows, thresholded_Mat.cols, CV_8UC1, cv::Scalar::all(0));
      std::vector<cv::Vec4i> hierarchy;
      cv::Scalar color(255, 255, 255);

      std::vector<cv::Rect> boundRect(1);

      boundRect[0] = boundingRect(cv::Mat(contours[largest_contour_index]));

      rectangle(Drawing, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0);

      // cv::Point center;
      center[0].x = ((boundRect[0].br()).x + (boundRect[0].tl()).x) / 2;
      center[0].y = ((boundRect[0].tl()).y + (boundRect[0].br()).y) / 2;
      int side_x = (boundRect.br()).x - (boundRect.tl()).x;
      int side_y = -((boundRect.tl()).y - (boundRect.br()).y);


      drawContours(Drawing, contours, largest_contour_index, color, 2, 8, hierarchy);

      cv::Mat frame_mat = frame;
      cv::Point2f screen_center;
      screen_center.x = 320;  // size of my screen
      screen_center.y = 240;

      circle(frame_mat, center[0], 5, cv::Scalar(0, 250, 0), -1, 8, 1);
      rectangle(frame_mat, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0);
      circle(frame_mat, screen_center, 4, cv::Scalar(150, 150, 150), -1, 8, 0);  // center of screen

      cv::imshow("MarkerDetection:Contours", Drawing);

      std::vector<cv::Point2f> center_avg(1)
      center_avg[0].x = (center_ideal[0].x + center_ideal[1].x + center_ideal[2].x + center_ideal[3].x + center_ideal[4].x) / 5;
      center_avg[0].y = (center_ideal[0].y + center_ideal[1].y + center_ideal[2].y + center_ideal[3].y + center_ideal[4].y) / 5;
      
      if ((center[0].x < (center_avg[0].x + 10)) && (center[0].y < (center_avg[0].y + 10)) && (count_avg >= 5))
      {
        center_ideal[4] = center_ideal[3];
        center_ideal[3] = center_ideal[2];
        center_ideal[2] = center_ideal[1];
        center_ideal[1] = center_ideal[0];
        center_ideal[0] = center[0];
        count_avg++;
      }
      else if (count_avg <= 5)
      {
        r[count_avg] = radius[0];
        center_ideal[count_avg] = center[0];
        count_avg++;
      }
      else
      {
        count_avg = 0;
      }

      // cv::Mat circles = frame;
      // circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      // circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
      // circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

      int net_x_cord = 320 - center_ideal[0].x + r[0]; // put the sides in place of radius
      int net_y_cord = -240 + center_ideal[0].y + r[0];
      if (net_x_cord < -310)
      {
        array.data.push_back(-2);  // top
        array.data.push_back(-2);
        // array.data.push_back(-2);
        // array.data.push_back(-2);
        pub.publish(array);
      }
      else if (net_x_cord > 310)
      {
        array.data.push_back(-1);  // left_side
        array.data.push_back(-1);
        // array.data.push_back(-1);
        // array.data.push_back(-1);
        pub.publish(array);
        ros::spinOnce();
      }
      else if (net_y_cord > 230)
      {
        array.data.push_back(-3);  // bottom
        array.data.push_back(-3);
        // array.data.push_back(-3);
        // array.data.push_back(-3);
        pub.publish(array);
      }
      else if (net_y_cord < -230)
      {
        array.data.push_back(-4);  // right_side
        array.data.push_back(-4);
        // array.data.push_back(-4);
        // array.data.push_back(-4);
        pub.publish(array);
      }
      else if (side_y > 110 || side_x > 110) // put the side of the square or rectangle which is obtained from the conours
      {
        array.data.push_back(-5);
        array.data.push_back(-5);
        // array.data.push_back(-5);
        // array.data.push_back(-5);
        pub.publish(array);
      }
      else
      {
        // float distance;
        // distance = pow(radius[0] / 7526.5, -.92678);  // function found using experiment
        // array.data.push_back(r[0]);                   // publish radius
        array.data.push_back((320 - center_ideal[0].x));
        array.data.push_back(-(240 - center_ideal[0].y));
        array.data.push_back(distance);
        pub.publish(array);
      }

      cv::imshow("MarkerDetection:Original", circles);  // Original stream with detected ball overlay

      if ((cvWaitKey(10) & 255) == 32)
      {
        if (x == 32)
          x = -1;
        else
          x = 32;
      }
      if (x == 32)
        ROS_INFO("%s: PAUSED\n", ros::this_node::getName().c_str());
      ros::spinOnce();
      // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
      // remove higher bits using AND operator
      if ((cvWaitKey(10) & 255) == 27)
        break;
    }
    else
    {
      if ((cvWaitKey(10) & 255) == 32)
      {
        if (x == 32)
          x = -1;
        else
          x = 32;
      }
      if (x == 32)
        ROS_INFO("%s: PAUSED\n", ros::this_node::getName().c_str());
      ros::spinOnce();
    }
  }
  // output_cap.release();
  return 0;
}
