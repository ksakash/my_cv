// Copyright 2016 AUV-IITKs
#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <task_buoy/buoyConfig.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float32MultiArray.h"
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

int flag =0; // 0 for red, 1 for green, 2 for blue
std::vector<cv::Mat> buoys(3); // for containing all the thresholded images of buoys separately

bool IP = true;
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max;  // Default Params
int r1min, r1max, r2min, r2max, r3min, r3max;
int g1min, g1max, g2min, g2max, g3min, g3max;
int b1min, b1max, b2min, b2max, b3min, b3max;

cv::Mat frame;
cv::Mat newframe;
// cv::Mat dst_array;
cv::Mat dst;

int count = 0, count_avg = 0, x = -1;

void callback(task_buoy::buoyConfig &config, uint32_t level)
{
  t1min = config.t1min_param;
  t1max = config.t1max_param;
  t2min = config.t2min_param;
  t2max = config.t2max_param;
  t3min = config.t3min_param;
  t3max = config.t3max_param;
  flag = config.flag_param;
  ROS_INFO("Buoy_Reconfigure Request:New params : %d %d %d %d %d %d %d", t1min, t1max, t2min, t2max, t3min, t3max, flag);
}

void lineDetectedListener(std_msgs::Bool msg)
{
  IP = msg.data;
}

void coloredBuoyListener(std_msgs::int64 msg)
{
  flag = msg.data;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  if (x == 32)
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
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}

void balance_white(cv::Mat mat)
{
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
  ros::init(argc, argv, "buoy_detection");
  int height, width, step, channels;  // parameters of the image we are working on
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("buoy_detection_switch", 1000, &lineDetectedListener);
  ros::Subscriber sub1 = n.subscribe<std_msgs::int64>("buoy_color_switch", 1000, &coloredBuoyListener); // for knowing which buoy is alreagy hit
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  dynamic_reconfigure::Server<task_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<task_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  // cvNamedWindow("BuoyDetection:circle", CV_WINDOW_NORMAL);
  // cvNamedWindow("BuoyDetection:AfterThresholding", CV_WINDOW_NORMAL);
  // cvNamedWindow("BuoyDetection:AfterEnhancing", CV_WINDOW_NORMAL);

  CvSize size = cvSize(width, height);
  std::vector<cv::Point2f> center_ideal(5);

  float r[5];

  for (int m = 0; m++; m < 5)
    r[m] = 0;

  // all the cv::Mat declared outside the loop to increase the speed


  cv::Mat lab_image, balanced_image1, dstx, image_clahe, dst, dst1, dst2;
  std::vector<cv::Mat> lab_planes(3);
  std::vector<cv::Mat> thresholded(3);

  while (ros::ok())
  {
    // std::cout << t1min << "  "<< t1max <<" "<< t2min << " " << t2max << " " << t3min <<" "<< t3max << std::endl;

    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
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

    for (int i = 0; i < 3; i++)
    {
      buoys[i] = balanced_image1; // till here the input will be same irrespective of the color
    }

    // no need to make 18 parameters dynamically configurable, only 6 parameters needed

    if (flag == 0)
    {
      r1min = t1min;
      r1max = t1max;
      r2min = t2min;
      r2max = t2max;
      r3min = t3min;
      r3max = t3max;
    }

    else if (flag == 1)
    {
      g1min = t1min;
      g2min = t2min;
      g3min = t3min;
      g1max = t1max;
      g2max = t2max;
      g3max = t3max;
    }

    else if (flag == 2)
    {
      b1min = t1min;
      b1max = t1max;
      b2min = t2min;
      b2max = t2max;
      b3min = t3min;
      b3max = t3max;
    }
    cv::Scalar r_min = cv::Scalar(r1min, r2min, r3min, 0);
    cv::Scalar r_max = cv::Scalar(r1max, r2max, r3max, 0);

    cv::Scalar b_min = cv::Scalar(b1min, b2min, b3min, 0);
    cv::Scalar b_max = cv::Scalar(b1max, b2max, b3max, 0);

    cv::Scalar g_min = cv::Scalar(g1min, g2min, g3min, 0);
    cv::Scalar g_max = cv::Scalar(g1max, g2max, g3max, 0);

    // thresholding all the colors according to their thresholding values
    cv::inRange(buoys[0], r_min, r_max, thresholded[0]);
    cv::inRange(bouys[1], g_min, g_max, thresholded[0]);
    cv::inRange(buoys[2], b_min, b_max, thresholded[0]);

    for (int i = 0; i < 3; i++)
    {
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    }

    // merging the result

    cv::addWeighted( thresholded[0], 1, thresholded[1], 1, 0.0, dst1);
    cv::addWeighted( dst1, 1, thresholded[2], 1, 0.0, dst2);

    // cv::imshow("BuoyDetection:AfterEnhancing", balanced_image1);
    // cv::imshow("BuoyDetection:AfterThresholding", thresholded);

    if ((cvWaitKey(10) & 255) == 27)
      break;

    if (1)
    {

      std::cout << "inside if" << std::endl;
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded[flag];
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;

      // std::cout << contours.size() << std::endl;

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
          array.data.push_back(flag);
          pub.publish(array);
        }
        else if (x_cord > 270)
        {
          array.data.push_back(-1);  // left_side
          array.data.push_back(-1);
          array.data.push_back(-1);
          array.data.push_back(-1);
          array.data.push_back(flag);
          pub.publish(array);
        }
        else if (y_cord > 200)
        {
          array.data.push_back(-3);  // bottom
          array.data.push_back(-3);
          array.data.push_back(-3);
          array.data.push_back(-3);
          array.data.push_back(flag);
          pub.publish(array);
        }
        else if (y_cord < -200)
        {
          array.data.push_back(-4);  // right_side
          array.data.push_back(-4);
          array.data.push_back(-4);
          array.data.push_back(-4);
          array.data.push_back(flag);
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
      std::vector<float> radius(1);
      cv::minEnclosingCircle(contours[largest_contour_index], center[0], radius[0]);
      cv::Point2f pt;
      pt.x = 320;  // size of my screen
      pt.y = 240;

      float r_avg = (r[0] + r[1] + r[2] + r[3] + r[4]) / 5;
      if ((radius[0] < (r_avg + 10)) && (count_avg >= 5))
      {
        r[4] = r[3];
        r[3] = r[2];
        r[2] = r[1];
        r[1] = r[0];array.data.push_back(flag);
        r[0] = radius[0];
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
      {array.data.push_back(flag);
        count_avg = 0;
      }

      std::cout << "here" << std::endl;


      cv::Mat circles = frame;
      circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
      circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", circles).toImageMsg();
      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
      sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded).toImageMsg();

      pub1.publish(msg1);
      pub2.publish(msg2);
      pub3.publish(msg3);

      int net_x_cord = 320 - center_ideal[0].x + r[0];
      int net_y_cord = -240 + center_ideal[0].y + r[0];
      if (net_x_cord < -310)
      {
        array.data.push_back(-2);  // top
        array.data.push_back(-2);
        array.data.push_back(-2);
        array.data.push_back(-2);
        array.data.push_back(flag);
        pub.publish(array);
      }
      else if (net_x_cord > 310)
      {
        array.data.push_back(-1);  // left_side
        array.data.push_back(-1);
        array.data.push_back(-1);
        array.data.push_back(-1);
        array.data.push_back(flag);
        pub.publish(array);
      }
      else if (net_y_cord > 230)
      {
        array.data.push_back(-3);  // bottom
        array.data.push_back(-3);
        array.data.push_back(-3);
        array.data.push_back(-3);
        array.data.push_back(flag);
        pub.publish(array);
      }
      else if (net_y_cord < -230)
      {
        array.data.push_back(-4);  // right_side
        array.data.push_back(-4);
        array.data.push_back(-4);
        array.data.push_back(-4);
        array.data.push_back(flag);
        pub.publish(array);
      }
      else if (r[0] > 110)
      {
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(flag);
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
        array.data.push_back(flag);
        pub.publish(array);
      }
      // cv::imshow("BuoyDetection:circle", circles);  // Original stream with detected ball overlay

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

