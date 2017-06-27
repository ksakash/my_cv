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

void SimplestCB(cv::Mat& in, cv::Mat& out, float percent)
{
    assert(in.channels() == 3);
    assert(percent > 0 && percent < 100);
    float half_percent = percent / 200.0f;
    std::vector<cv::Mat> tmpsplit; split(in, tmpsplit);
    for ( int i = 0; i < 3; i++ )
    {
        // find the low and high precentile values (based on the input percentile)
        cv::Mat flat;
        tmpsplit[i].reshape(1, 1).copyTo(flat);
        cv::sort(flat, flat, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
        int lowval = flat.at<uchar>(cvFloor((static_cast<float>(flat.cols)) * half_percent));
        int highval = flat.at<uchar>(cvCeil((static_cast<float>(flat.cols)) * (1.0 - half_percent)));
        // saturate below the low percentile and above the high percentile
        tmpsplit[i].setTo(lowval, tmpsplit[i] < lowval);
        tmpsplit[i].setTo(highval, tmpsplit[i] > highval);
        // scale the channel
        cv::normalize(tmpsplit[i], tmpsplit[i], 0, 255, cv::NORM_MINMAX);
    }
    cv::merge(tmpsplit, out);
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

  dynamic_reconfigure::Server<task_torpedo::torpedoConfig> server;
  dynamic_reconfigure::Server<task_torpedo::torpedoConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cvNamedWindow("TorpedoDetection:AfterColorFiltering", CV_WINDOW_NORMAL);
  cvNamedWindow("TorpedoDetection:Contours", CV_WINDOW_NORMAL);
  cvNamedWindow("TorpedoDetection:AfterSimplestCB", CV_WINDOW_NORMAL);
  cvNamedWindow("TorpedoDetection:AfterHistogramEqualization", CV_WINDOW_NORMAL);
  cvNamedWindow("TorpedoDetection:AfterMorphology",CV_WINDOW_NORMAL);

  // capture size -
  CvSize size = cvSize(width, height);

  // Initialize different images that are going to be used in the program
  cv::Mat hsv_frame, thresholded, thresholded1, thresholded2, thresholded3, filtered;  // image converted to HSV plane
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

    SimplestCB(frame, dst, 1);
    cv::imshow("TorpedoDetection:AfterSimplestCB",dst);

    cv::Mat frame_array[3];

    cv::split(dst, frame_array);

    equalizeHist(frame_array[0], frame_array[0]);
    equalizeHist(frame_array[1], frame_array[1]);
    equalizeHist(frame_array[2], frame_array[2]);

    cv::merge(frame_array, 3, dst_array);
    cv::imshow("TorpedoDetection:AfterHistogramEqualization",dst_array);

    // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
    cv::cvtColor(frame, hsv_frame, CV_BGR2HSV);
    cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
    cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);
    // Filter out colors which are out of range.
    cv::inRange(hsv_frame, hsv_min, hsv_max, thresholded);

    //morphological opening (remove small objects from the foreground)
    erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    //morphological closing (fill small holes in the foreground)
    dilate(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    erode(thresholded, thresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    
    cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);
    cv::imshow("TorpedoDetection:AfterColorFiltering", thresholded);  // The stream after color filtering

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
        array.data.push_back(0);
        array.data.push_back(0);

        pub.publish(array);
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

      std::vector<cv::Rect> boundRect(2);

      boundRect[0] = boundingRect(cv::Mat(contours[largest_contour_index]));
      boundRect[1] = boundingRect(cv::Mat(contours[second_largest_contour_index])); // finding a min fitting rectangle around the heart shape

      // not shown the Drawing Mat
      rectangle(Drawing, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0); // drawing a rectangle in the image
      rectangle(Drawing, boundRect[1].tl(), boundRect[1].br(), color, 2, 8, 0); // drawing a rectangle in the image 

      cv::Point center;
      center.x = ((boundRect[0].br()).x + (boundRect[0].tl()).x) / 2;
      center.y = ((boundRect[0].tl()).y + (boundRect[0].br()).y) / 2;
      int side_x = (boundRect[0].br()).x - (boundRect[0].tl()).x;
      int side_y = -((boundRect[0].tl()).y - (boundRect[0].br()).y);
      drawContours(Drawing, contours, largest_contour_index, color, 2, 8, hierarchy);

      cv::Point heart_center;
      heart_center.x = ((boundRect[1].br()).x + (boundRect[1].tl()).x) / 2;
      int heart_side_x = (boundRect[1].br()).x - (boundRect[1].tl()).x;
      int heart_side_y = -((boundRect[1].tl()).y - (boundRect[1].br()).y);
      heart_center.y = (boundRect[1].br()).y + side_y / 3;
      drawContours(Drawing, contours, second_largest_contour_index, color, 2, 8, hierarchy);

      cv::Mat frame_mat = frame;
      cv::Point2f screen_center;
      screen_center.x = 320;  // size of my screen
      screen_center.y = 240;

      circle(frame_mat, center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
      circle(frame_mat, heart_center, 5, cv::Scalar(0, 250, 0), -1, 8, 1);
      rectangle(frame_mat, boundRect[0].tl(), boundRect[0].br(), color, 2, 8, 0);
      rectangle(frame_mat, boundRect[1].tl(), boundRect[1].br(), color, 2, 8, 0);
      circle(frame_mat, screen_center, 4, cv::Scalar(150, 150, 150), -1, 8, 0);  // center of screen

      int x_cord_heart = heart_center.x - 320;
      int y_cord_heart = 240 - heart_center.y;

      int x_cord = center.x - 320;
      int y_cord = 240 - center.y;
        

      cv::imshow("TorpedoDetection:Contours", Drawing);

      // have to make changes after here 
      if (x_cord < -270)
      {
        array.data.push_back(-2);  // top
        array.data.push_back(-2);
      }
      else if (x_cord > 270)
      {
        array.data.push_back(-1);  // left_side
        array.data.push_back(-1);
      }
      else if (y_cord > 200)
      {
        array.data.push_back(-3);  // bottom
        array.data.push_back(-3);
      }
      else if (y_cord < -200)
      {
        array.data.push_back(-4);  // right_side
        array.data.push_back(-4);
      }
      else
      {
        array.data.push_back(x_cord_heart); // x and y coordinate of the center of the heart
        array.data.push_back(y_cord_heart);
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

