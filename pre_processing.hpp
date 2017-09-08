// Copyright 2017 AUV-IITK
#ifndef PRE_PROCESSING_H
#define PRE_PROCESSING_H

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

namespace pre_processing{

  cv::Mat balance_white(cv::Mat src, float parameter); // for removing the color cast
  cv::Mat color_correction(cv::Mat src, float parameter); // apply the CLAHE algorithm here
  cv::Mat denoise(cv::Mat src, int i); // applying bilateralFilter 2*i times
  std_msgs::Float64MultiArray empty_frame_handler(cv::Point2f); // gives the last coordinates of the center of the buoy if the present frame is empty
  std_msgs::Float64MultiArray edge_case_handler(cv::Point2f, int radius); // tells if buoy is at the edges and which edge
  void get_buoys_params(ros::NodeHandle &nh, int** r, int** b, int** g); // get the parameter values from the parameter server running in the ros master
  void threshold(int** BGR, int** red_buoy, int** blue_buoy, int** green_buoy, int flag); // for thresholding three buoys without using 18 variables
  void set_buoy_params(ros::NodeHandle n, int** red_buoy, int** blue_buoy, int** green_buoy); // for setting the parameter values in the parameter server
  void update_values(task_buoy::buoyConfig &config, int flag);
  void threshold_values_update(int BGR[][2], task_buoy::buoyConfig &config);
}

#endif
