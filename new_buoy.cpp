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

bool IP = true;
bool flag = false;
bool video = false;
int t1min, t1max, t2min, t2max, t3min, t3max;  // Default Params

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
  ROS_INFO("Buoy_Reconfigure Request:New params : %d %d %d %d %d %d", t1min, t1max, t2min, t2max, t3min, t3max);
}

void lineDetectedListener(std_msgs::Bool msg)
{
  IP = msg.data;
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


// void SimplestCB(cv::Mat& in, cv::Mat& out, float percent)
// {
//    assert(in.channels() == 3);
//    assert(percent > 0 && percent < 100);
//    float half_percent = percent / 200.0f;
//    std::vector<cv::Mat> tmpsplit; split(in, tmpsplit);
//    for ( int i = 0; i < 3; i++ )
//    {
//        // find the low and high precentile values (based on the input percentile)
//        cv::Mat flat;
//        tmpsplit[i].reshape(1, 1).copyTo(flat);
//        cv::sort(flat, flat, CV_SORT_EVERY_ROW + CV_SORT_ASCENDING);
//        int lowval = flat.at<uchar>(cvFloor((static_cast<float>(flat.cols)) * half_percent));
//        int highval = flat.at<uchar>(cvCeil((static_cast<float>(flat.cols)) * (1.0 - half_percent)));
//        // saturate below the low percentile and above the high percentile
//        tmpsplit[i].setTo(lowval, tmpsplit[i] < lowval);
//        tmpsplit[i].setTo(highval, tmpsplit[i] > highval);
//        // scale the channel
//        cv::normalize(tmpsplit[i], tmpsplit[i], 0, 255, cv::NORM_MINMAX);
//    }
//    cv::merge(tmpsplit, out);
//    cout << "inside simplestcb : no problem here" << endl;
// }

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
  // std::string Video_Name = "Random_Video";
  // if (argc >= 2)
  //  flag = true;
  // if (argc == 3)
  // {
  //  video = true;
  //  std::string avi = ".avi";
  //  Video_Name = (argv[2]) + avi;
  // }

  // cv::VideoWriter output_cap(Video_Name, CV_FOURCC('D', 'I', 'V', 'X'), 9, cv::Size(640, 480));

  ros::init(argc, argv, "buoy_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/buoy", 1000);
  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("buoy_detection_switch", 1000, &lineDetectedListener);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);

  dynamic_reconfigure::Server<task_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<task_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  cvNamedWindow("BuoyDetection:circle", CV_WINDOW_NORMAL);
  cvNamedWindow("BuoyDetection:AfterThresholding", CV_WINDOW_NORMAL);
  cvNamedWindow("BuoyDetection:AfterEnhancing",CV_WINDOW_NORMAL);
  // cvNamedWindow("BuoyDetection:AfterSimplestCB",CV_WINDOW_NORMAL);
  // capture size -
  CvSize size = cvSize(width, height);
  std::vector<cv::Point2f> center_ideal(5);

  cv::Mat hsv_frame, thresholded, filtered;  // image converted to HSV plane
  float r[5];

  for (int m = 0; m++; m < 5)
    r[m] = 0;


  // all the cv::Mat declared outside the loop to increase the speed
  cv::Mat lab_image;
  cv::Mat balanced_image;
  cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
  cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);
  cv::Mat balanced_image1;
  cv::Mat dstx;
  std::vector<cv::Mat> lab_planes(3);

  while (ros::ok())
  {
    std_msgs::Float64MultiArray array;
    loop_rate.sleep();
    if (frame.empty())
    {
      ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
      ros::spinOnce();
      continue;
    }

    // if (video)
    //  output_cap.write(frame);
    
    // get the image data
    height = frame.rows;
    width = frame.cols;
    step = frame.step;

    // SimplestCB(frame, dst, 1);
    // cv::imshow("BuoyDetection:AfterColorFiltering",dst);

    // cv::Mat frame_array[3];

    // cv::split(dst, frame_array);

    // cv::equalizeHist(frame_array[0], frame_array[0]);
    // cv::equalizeHist(frame_array[1], frame_array[1]);
    // cv::equalizeHist(frame_array[2], frame_array[2]);

    // cv::merge(frame_array, 3, dst_array);*/

    // cv::Mat thresholded;

    // cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    
    // frame.copyTo(balanced_image);
    // balance_white(balanced_image);
    // cv::Mat dst1;
    // fastNlMeansDenoisingColored(balanced_image, balanced_image, 3, 7, 21);
    // bilateralFilter(balanced_image, dst1, 4, 8, 8);
        
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
        
    // fastNlMeansDenoisingColored(image_clahe, image_clahe, 3, 7, 21);
    // bilateralFilter(image_clahe, dst2, 4, 8, 8);
    // balance_white(dst2);
    // image_clahe.copyTo(balanced_image1);
    // balance_white(balanced_image1);
    // cv::Mat dst3;
    // fastNlMeansDenoisingColored(balanced_image1, balanced_image1, 3, 7, 21);
    // inpaint(balanced_image1,  Mat::zeros(balanced_image1.size(), CV_8U), dst3, 3, INPAINT_TELEA);
    // Filter out colors which are out of range.

    cv::inRange(balanced_image1, hsv_min, hsv_max, thresholded);
    // cv::Mat dst4;
    // bilateralFilter(thresholded, dst4, 4, 8, 8)


    // bilateralFilter(balanced_image1, dst2, 4, 8, 8);
    // balance_white(balanced_image1);
    // cv::Mat dst4;
    // bilateralFilter(thresholded, dst4, 20, 20, 20);
    // cv::Mat dst4;
    // bilateralFilter(thresholded, dst4, 4, 8, 8)
    // morphological opening (remove small objects from the foreground)
    // cv::erode(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

    cv::imshow("BuoyDetection:AfterEnhancing", balanced_image1);
    cv::imshow("BuoyDetection:AfterThresholding", thresholded);


    // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
    // cv::cvtColor(dst_array, hsv_frame, CV_BGR2HSV);
    
    // cv::Scalar hsv_min = cv::Scalar(t1min, t2min, t3min, 0);
    // cv::Scalar hsv_max = cv::Scalar(t1max, t2max, t3max, 0);
    // Filter out colors which are out of range.
    // cv::inRange(hsv_frame, hsv_min, hsv_max, thresholded);

    // morphological opening (remove small objects from the foreground)
    // cv::erode(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    // cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // morphological closing (fill small holes in the foreground)
    // cv::dilate(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    // cv::erode(thresholded, thresholded, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // cv::GaussianBlur(thresholded, thresholded, cv::Size(9, 9), 0, 0, 0);
    // cv::imshow("BuoyDetection:AfterColorFiltering", thresholded);  // The stream after color filtering

    if ((cvWaitKey(10) & 255) == 27)
      break;

    if (1)
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
      cv::imshow("BuoyDetection:circle", circles);  // Original stream with detected ball overlay

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
