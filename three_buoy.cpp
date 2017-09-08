// Copyright 2016 AUV-IITK
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
#include "pre_processing.h"

#define SHELLSCRIPT_DUMP "\
#/bin/bash \n\
echo -e \"parameters dumped!!\" \n\
rosparam dump ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml \
"
#define SHELLSCRIPT_LOAD "\
#/bin/bash \n\
echo -e \"parameters loaded!!\" \n\
rosparam load ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml \
"

bool IP = true;
bool video = false;
bool save;
bool done;
bool threshold;
int flag;
int count = 0;
int BGR[3][2];  // rgb values for color filtering

int red_buoy[3][2], green_buoy[3][2], blue_buoy[3][2]; // rows for colors and colors for max and min ; 0, 1, 2 for blue, green and red ; 0 for min and 1 max
cv::Mat frame;
cv::Mat newframe;
cv::Mat dst;

int count_avg = 0;

void callback(task_buoy::buoyConfig &config, uint32_t level)
{
  flag = config.flag_param;

  if (count > 0)
  {
    if (flag == 0){
      config.t1min_param = blue_buoy[0][0];
      config.t1max_param = blue_buoy[0][1];
      config.t2min_param = blue_buoy[1][0];
      config.t2max_param = blue_buoy[1][1];
      config.t3min_param = blue_buoy[2][0];
      config.t3max_param = blue_buoy[2][1];
    }

    else if (flag == 1){
      config.t1min_param = green_buoy[0][0];
      config.t1max_param = green_buoy[0][1];
      config.t2min_param = green_buoy[1][0];
      config.t2max_param = green_buoy[1][1];
      config.t3min_param = green_buoy[2][0];
      config.t3max_param = green_buoy[2][1];
    }

    else if (flag == 2){
      config.t1min_param = red_buoy[0][0];
      config.t1max_param = red_buoy[0][1];
      config.t2min_param = red_buoy[1][0];
      config.t2max_param = red_buoy[1][1];
      config.t3min_param = red_buoy[2][0];
      config.t3max_param = red_buoy[2][1];
    }
  }

  /*BGR[0][0] = config.t1min_param;
  BGR[0][1] = config.t1max_param;
  BGR[1][0] = config.t2min_param;
  BGR[1][1] = config.t2max_param;
  BGR[2][0] = config.t3min_param;
  BGR[2][1] = config.t3max_param;*/

  //std::cout << "yo" << std::endl;

  pre_processing::threshold_values_update(BGR, config);

  //std::cout << "yo2" << std::endl;

  done = config.done_param;
  // flag = config.flag_param;
  threshold = config.threshold_param;

  if (done = true){
    config.save_param = false;
  }

  save = config.save_param;
  config.save_param = false;


  ROS_INFO("Buoy_Reconfigure Request:New params : %d %d %d %d %d %d %d %d %d %d", BGR[0][0], BGR[0][1], BGR[1][0], BGR[1][1], BGR[2][0], BGR[2][1], save, flag, done, threshold);
}

void lineDetectedListener(std_msgs::Bool msg)
{
  IP = msg.data;
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

/*void balance_white(cv::Mat mat)
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
}*/


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "buoy_detection");
  int height, width, step, channels;  // parameters of the image we are working on
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("buoy_detection_switch", 1000, &lineDetectedListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  system(SHELLSCRIPT_LOAD);

  n.getParam("buoy_detection/r1min", red_buoy[0][0]);
  n.getParam("buoy_detection/r1max", red_buoy[0][1]);
  n.getParam("buoy_detection/r2min", red_buoy[1][0]);
  n.getParam("buoy_detection/r2max", red_buoy[1][1]);
  n.getParam("buoy_detection/r3min", red_buoy[2][0]);
  n.getParam("buoy_detection/r3max", red_buoy[2][1]);

  n.getParam("buoy_detection/g1max", green_buoy[0][0]);
  n.getParam("buoy_detection/g1min", green_buoy[0][1]);
  n.getParam("buoy_detection/g2max", green_buoy[1][0]);
  n.getParam("buoy_detection/g2min", green_buoy[1][1]);
  n.getParam("buoy_detection/g3max", green_buoy[2][0]);
  n.getParam("buoy_detection/g3min", green_buoy[2][1]);

  n.getParam("buoy_detection/b1max", blue_buoy[0][0]);
  n.getParam("buoy_detection/b1min", blue_buoy[0][1]);
  n.getParam("buoy_detection/b2max", blue_buoy[1][0]);
  n.getParam("buoy_detection/b2min", blue_buoy[1][1]);
  n.getParam("buoy_detection/b3max", blue_buoy[2][0]);
  n.getParam("buoy_detection/b3min", blue_buoy[2][1]);

  std::cout << "after getParam" << std::endl;
  std::cout << red_buoy[0][0] << std::endl;

  dynamic_reconfigure::Server<task_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<task_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  std::cout << "after server setCallback" << std::endl;

  CvSize size = cvSize(width, height);
  std::vector<cv::Point2f> center_ideal(5);

  float r[5];

  for (int m = 0; m++; m < 5)
    r[m] = 0;

  cv::Mat lab_image, balanced_image1, dstx, image_clahe, dst, dst1;
  std::vector<cv::Mat> lab_planes(3);
  std::vector<cv::Mat> buoys(3);
  std::vector<cv::Mat> thresholded(3);
  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
    if (! ros::param::has("/buoy_detection/b2min"))
      std::cout << "exist" << std::endl;
    if (threshold == true){
      if (flag == 0)
      {
        for (int i = 0; i < 3; i++){
          for (int j = 0; j < 2; j++){
            blue_buoy[i][j] = BGR[i][j];
          }
        }
      }

      else if (flag == 1)
      {
        for (int i = 0; i < 3; i++){
          for (int j = 0; j < 2; j++){
            green_buoy[i][j] = BGR[i][j];
          }
        }
      }

      else if (flag == 2)
      {
        for (int i = 0; i < 3; i++){
          for (int j = 0; j < 2; j++){
            red_buoy[i][j] = BGR[i][j];
          }
        }
      }
    }

    if (save == true){

      n.setParam("buoy_detection/r1min", red_buoy[0][0]);
      n.setParam("buoy_detection/r1max", red_buoy[0][1]);
      n.setParam("buoy_detection/r2min", red_buoy[1][0]);
      n.setParam("buoy_detection/r2max", red_buoy[1][1]);
      n.setParam("buoy_detection/r3min", red_buoy[2][0]);
      n.setParam("buoy_detection/r3max", red_buoy[2][1]);

      n.setParam("buoy_detection/g1max", green_buoy[0][0]);
      n.setParam("buoy_detection/g1min", green_buoy[0][1]);
      n.setParam("buoy_detection/g2max", green_buoy[1][0]);
      n.setParam("buoy_detection/g2min", green_buoy[1][1]);
      n.setParam("buoy_detection/g3max", green_buoy[2][0]);
      n.setParam("buoy_detection/g3min", green_buoy[2][1]);

      n.setParam("buoy_detection/b1max", blue_buoy[0][0]);
      n.setParam("buoy_detection/b1min", blue_buoy[0][1]);
      n.setParam("buoy_detection/b2max", blue_buoy[1][0]);
      n.setParam("buoy_detection/b2min", blue_buoy[1][1]);
      n.setParam("buoy_detection/b3max", blue_buoy[2][0]);
      n.setParam("buoy_detection/b3min", blue_buoy[2][1]);

      save = false;
      std::cout << "save false" << std::endl;

      system(SHELLSCRIPT_DUMP);
      std::cout << "dumped" << std::endl;

    }

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
    dst1 = pre_processing::balance_white(balanced_image1, 0.05);

    for (int i=0; i < 2; i++)
    {
      bilateralFilter(balanced_image1, dstx, 6, 8, 8);
      bilateralFilter(dstx, balanced_image1, 6, 8, 8);
    }

    for (int i = 0; i < 3; i++)
      buoys[i] = dst1;

    cv::Scalar red_buoy_min = cv::Scalar(red_buoy[0][0], red_buoy[1][0], red_buoy[2][0], 0);
    cv::Scalar red_buoy_max = cv::Scalar(red_buoy[0][1], red_buoy[1][1], red_buoy[2][1], 0);

    cv::Scalar blue_buoy_min = cv::Scalar(blue_buoy[0][0], blue_buoy[1][0], blue_buoy[2][0], 0);
    cv::Scalar blue_buoy_max = cv::Scalar(blue_buoy[0][1], blue_buoy[1][1], blue_buoy[2][1], 0);

    cv::Scalar green_buoy_min = cv::Scalar(green_buoy[0][0], green_buoy[1][0], green_buoy[2][0], 0);
    cv::Scalar green_buoy_max = cv::Scalar(green_buoy[0][1], green_buoy[1][1], green_buoy[2][1], 0);

    // thresholding all the colors according to their thresholding values
    cv::inRange(buoys[0], red_buoy_min, red_buoy_max, thresholded[0]);
    cv::inRange(buoys[1], green_buoy_min, green_buoy_max, thresholded[1]);
    cv::inRange(buoys[2], blue_buoy_min, blue_buoy_max, thresholded[2]);


    // Filter out colors which are out of range.

    for (int i = 0; i < 3; i++)
    {
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
      cv::dilate(thresholded[i], thresholded[i], getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    }

    // if ((cvWaitKey(10) & 255) == 27)
    //   break;

    if (1)
    {
      // find contours
      std::vector<std::vector<cv::Point> > contours;
      cv::Mat thresholded_Mat = thresholded[0];
      findContours(thresholded_Mat, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);  // Find the contours
      double largest_area = 0, largest_contour_index = 0;
      std::cout << "inside if" << std::endl;
      sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", balanced_image1).toImageMsg();
      sensor_msgs::ImagePtr msg3 = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresholded_Mat).toImageMsg();

      pub2.publish(msg2);
      pub3.publish(msg3);

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
        ros::spinOnce();
        // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        // remove higher bits using AND operator
        // if ((cvWaitKey(10) & 255) == 27)
        //   break;
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
        r[1] = r[0];
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
      {
        count_avg = 0;
      }

      cv::Mat circles = frame;
      circle(circles, center_ideal[0], r[0], cv::Scalar(0, 250, 0), 1, 8, 0);  // minenclosing circle
      circle(circles, center_ideal[0], 4, cv::Scalar(0, 250, 0), -1, 8, 0);    // center is made on the screen
      circle(circles, pt, 4, cv::Scalar(150, 150, 150), -1, 8, 0);             // center of screen

      sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", circles).toImageMsg();
      pub1.publish(msg1);

      int net_x_cord = 320 - center_ideal[0].x + r[0];
      int net_y_cord = -240 + center_ideal[0].y + r[0];
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
      else if (r[0] > 110)
      {
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(-5);
        array.data.push_back(-5);
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
      // cv::imshow("BuoyDetection:circle", circles);  // Original stream with detected ball overlay

      /*if ((cvWaitKey(10) & 255) == 32)
      {
        if (x == 32)
          x = -1;
        else
          x = 32;
      }
      if (x == 32)
        ROS_INFO("%s: PAUSED\n", ros::this_node::getName().c_str());*/
      ros::spinOnce();
      // If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
      // remove higher bits using AND operator
      // if ((cvWaitKey(10) & 255) == 27)
      //   break;
    }
    else
    {
      /*if ((cvWaitKey(10) & 255) == 32)
      {
        if (x == 32)
          x = -1;
        else
          x = 32;
      }
      if (x == 32)
        ROS_INFO("%s: PAUSED\n", ros::this_node::getName().c_str());*/
      ros::spinOnce();
    }
  }
  // output_cap.release();
  return 0;
}
