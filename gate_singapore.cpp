// Copyright 2017 AUV-IITK
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
#include <task_gate/gateConfig.h>
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include "std_msgs/Float64MultiArray.h"
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <string>

#define SHELLSCRIPT_DUMP "\
#/bin/bash \n\
echo -e \"parameters dumped!!\" \n\
rosparam dump ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml /buoy_detection\
"
#define SHELLSCRIPT_LOAD "\
#/bin/bash \n\
echo -e \"parameters loaded!!\" \n\
rosparam load ~/catkin_ws/src/auv/task_handler_layer/task_buoy/launch/dump.yaml /buoy_detection\
"

cv::Mat frame, thresholded, newframe;
int red_min[3], red_max[3], green_min[3], green_max[3], blue_min[3], blue_max[3];
int flag = 0;
bool threshold = false;
bool save = false;
int count = 0;
bool task_gate_done;
bool gate_found;

void callback(task_buoy::buoyConfig &config, unit32_t){

  flag = config.flag_param;
  threshold = config.threshold;

  if(!threshold){

    // update the values of the trackbars used in thresholding the image

    if (flag == 0){
      config.t1min_param = red_min[0];
      config.t1max_param = red_max[0];
      config.t2min_param = blue_min[0];
      config.t2max_param = blue_max[0];
      config.t3min_param = green_min[0];
      config.t3max_param = green_max[0];
    }
    else if (flag == 1){
      config.t1min_param = red_min[1];
      config.t1max_param = red_max[1];
      config.t2min_param = blue_min[1];
      config.t2max_param = blue_max[1];
      config.t3min_param = green_min[1];
      config.t3max_param = green_max[1];
    }
    else if (flag == 2){
      config.t1min_param = red_min[2];
      config.t1max_param = red_max[2];
      config.t2min_param = blue_min[2];
      config.t2max_param = blue_max[2];
      config.t3min_param = green_min[2];
      config.t3max_param = green_max[2];
    }

  }

  // update the values in the red, green, blue matrix

  if (flag == 0){

    red_min[0] = config.t1min_param;
    red_max[0] = config.t1max_param;
    blue_min[0] = config.t2min_param;
    blue_max[0] = config.t2max_param;
    green_min[0] = config.t3min_param;
    green_max[0] = config.t3max_param;

  }

  else if (flag == 1){

    red_min[1] = config.t1min_param;
    red_max[1] = config.t1max_param;
    blue_min[1] = config.t2min_param;
    blue_max[1] = config.t2max_param;
    green_min[1] = config.t3min_param;
    green_max[1] = config.t3max_param;

  }

  else if (flag == 2){

    red_min[1] = config.t1min_param;
    red_max[1] = config.t1max_param;
    blue_min[1] = config.t2min_param;
    blue_max[1] = config.t2max_param;
    green_min[1] = config.t3min_param;
    green_max[1] = config.t3max_param;

  }


  if (!count){

    config.save_param = false; // just for making the radio button for save parameter false at the start of thresholding
    count++;

  }

  save = config.save_param;

  // for informing about the changing parameters
  ROS_INFO("Gate_Reconfigure Request:Red_Pole_params : %d %d %d %d %d %d %d %d %d", red_min[0], red_max[0], blue_min[0], blue_max[0], green_min[0], green_max[0], save, flag, threshold);
  ROS_INFO("Gate_Reconfigure Request:Green_Pole_params : %d %d %d %d %d %d %d %d %d", red_min[1], red_max[1], blue_min[1], blue_max[1], green_min[1], green_max[1]);

}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    newframe = cv_bridge::toCvShare(msg, "bgr8")->image;
    newframe.copyTo(frame);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("%s: Could not convert from '%s' to 'bgr8'.", ros::this_node::getName().c_str(), msg->encoding.c_str());
  }
}

bool isGateDectected(std::vector<std::vector<cv::Point> > contour0, std::vector<std::vector<cv::Point> > contour0, std::vector<std::vector<cv::Point> > contour0){

  return ((!contour0.empty())||(!contour1.empty())||(!contour2.empty());

}

int main(int argc, char **argv){

  ros::init(argc, argv, "gate_detection");
  int length, breadth, step, channels;
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/varun/ip/gate", 1000);
  ros::Rate loop_rate(10);

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub1 = it.subscribe("/varun/sensors/front_camera/image_raw", 1, imageCallback);
  image_transport::Publisher pub1 = it.advertise("/first_picture", 1);
  image_transport::Publisher pub2 = it.advertise("/second_picture", 1);
  image_transport::Publisher pub3 = it.advertise("/third_picture", 1);

  // to load all the parameters used in the node from the yaml file on the parameter server
  system(SHELLSCRIPT_LOAD);

  // to get those parameters from the parameter srver and set it in the variables declared here

  nh.getParam("gate_detection/r1min", red_min[0]);
  nh.getParam("gate_detection/r1max", red_max[0]);
  nh.getParam("gate_detection/r2min", blue_min[0]);
  nh.getParam("gate_detection/r2max", blue_max[0]);
  nh.getParam("gate_detection/r3min", green_min[0]);
  nh.getParam("gate_detection/r3max", green_max[0]);

  nh.getParam("gate_detection/g1max", red_min[1]);
  nh.getParam("gate_detection/g1min", red_max[1]);
  nh.getParam("gate_detection/g2max", blue_min[1]);
  nh.getParam("gate_detection/g2min", blue_max[1]);
  nh.getParam("gate_detection/g3max", green_min[1]);
  nh.getParam("gate_detection/g3min", green_max[1]);

  nh.getParam("gate_detection/b1max", red_min[2]);
  nh.getParam("gate_detection/b1min", red_max[2]);
  nh.getParam("gate_detection/b2max", blue_min[2]);
  nh.getParam("gate_detection/b2min", blue_max[2]);
  nh.getParam("gate_detection/b3max", green_min[2]);
  nh.getParam("gate_detection/b3min", green_max[2]);

  dynamic_reconfigure::Server<task_buoy::buoyConfig> server;
  dynamic_reconfigure::Server<task_buoy::buoyConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  while(ros::ok()){

    if (!task_gate_done){

      loop_rate.sleep();

      if (save == true){

        nh.setParam("gate_detection/r1min", red_min[0]);
        nh.setParam("gate_detection/r1max", red_max[0]);
        nh.setParam("gate_detection/r2min", blue_min[0]);
        nh.setParam("gate_detection/r2max", blue_max[0]);
        nh.setParam("gate_detection/r3min", green_min[0]);
        nh.setParam("gate_detection/r3max", green_max[0]);

        nh.setParam("gate_detection/g1min", red_min[1]);
        nh.setParam("gate_detection/g1max", red_max[1]);
        nh.setParam("gate_detection/g2min", blue_min[1]);
        nh.setParam("gate_detection/g2max", blue_max[1]);
        nh.setParam("gate_detection/g3min", green_min[1]);
        nh.setParam("gate_detection/g3max", green_max[1]);

        nh.setParam("gate_detection/b1min", red_min[2]);
        nh.setParam("gate_detection/b1max", red_max[2]);
        nh.setParam("gate_detection/b2min", blue_min[2]);
        nh.setParam("gate_detection/b2max", blue_max[2]);
        nh.setParam("gate_detection/b3min", green_min[2]);
        nh.setParam("gate_detection/b3max", green_max[2]);

        // dump the values in the yaml file
        save = false;
        system(SHELLSCRIPT_DUMP); // all the parameters saved in the yaml file

      }

      if (frame.empty())
      {
        ROS_INFO("%s: empty frame", ros::this_node::getName().c_str());
        ros::spinOnce();
        continue;
      }

      // all the image proecssing till the threshold

      pre_processing::color_correction();
      pre_processing::balance_white();
      pre_processing::denoise();

      std::vector<cv::Mat> thresholded(3);

      cv::Scalar red_rod_min = cv::Scalar(blue_min[0], green_min[0], red_min[0], 0);
      cv::Scalar red_rod_max = cv::Scalar(blue_max[0], green_max[0], red_max[0], 0);

      cv::Scalar black_rod_min = cv::Scalar(blue_min[1], green_min[1], red_min[1], 0);
      cv::Scalar black_rod_max = cv::Scalar(blue_max[1], green_max[1], red_max[1], 0);

      cv::Scalar green_rod_min = cv::Scalar(blue_min[2], green_min[2], red_min[2], 0);
      cv::Scalar green_rod_max = cv::Scalar(blue_max[2], green_max[2], red_max[2], 0);

      // thresholding all the colors according to their thresholding values
      cv::inRange(dst1, red_rod_min, red_rod_max, thresholded[0]);
      cv::inRange(dst1, green_rod_min, green_rod_max, thresholded[1]);
      cv::inRange(dst1, blue_rod_min, blue_rod_max, thresholded[2]);

      std::vector<std::vector<cv::Point> > contour0;
      std::vector<std::vector<cv::Point> > contour1;
      std::vector<std::vector<cv::Point> > contour2;

      cv::Mat Drawing0(thresholded.rows, thresholded.cols, CV_8UC1, cv::Scalar::all(0));
      cv::Mat Drawing1(thresholded.rows, thresholded.cols, CV_8UC1, cv::Scalar::all(0));
      cv::Mat Drawing2(thresholded.rows, thresholded.cols, CV_8UC1, cv::Scalar::all(0));

      findContours(thresholded[0], contour0, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      findContours(thresholded[1], contour1, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      findContours(thresholded[2], contour2, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

      // get the largest contour in the group of every group of contours
      // get the center of the largest contour
      // then use the contour center to know the center of the gate
      // the convexHull can be handy here

      drawContours(Drawing0, contour0, largest_contour_index0, color, 2, 8, hierarchy);
      drawContours(Drawing1, contour1, largest_contour_index1, color, 2, 8, hierarchy);
      drawContours(Drawing2, contour2, largest_contour_index2, color, 2, 8, hierarchy);

      cv::Mat combined_contour = Drawing0 + Drawing1 + Drawing2;

      if (!gate_detection){

        gate_detection = isGateDectected(contour0, contour1, contour2);

      }
      else {

        continue;

      }

      if (gate_detection){

        // if contour empty
        if (!(contour0.empty()||contour1.empty()||contour2.empty())){ // if there is even one contour

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

        }

        // if contour not empty

        else {

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

        }

      }

      else {
        // contour is always empty
        continue;
      }

    } // if ends

  } // while ends

}
