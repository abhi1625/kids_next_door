/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

/**
 * @file        ToyDetection.hpp
 * @author      Abhinav Modi
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam
 * @date        Dec 1, 2019
 * @brief       Header file for ToyDetection module to detect and locate toys in the
 *              robot's base frame using ArUco markers present on the toys.
 */

#ifndef INCLUDE_TOYDETECTION_HPP_
#define INCLUDE_TOYDETECTION_HPP_

#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include "../include/ROSModule.hpp"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "kids_next_door/toyFound.h"

class ToyDetection : public ROSModule {
 public:
  /**
   * @brief Default Constructor
   */
  ToyDetection();

  /**
   * @brief Method to initialize Subscribers, inherited from ROSModule
   *
   * @param None
   *
   * @return None
   */
  void initializeSubscribers();

  /**
   * @brief Method to initialize Service Servers, inherited from ROSModule
   *
   * @param None
   *
   * @return None
   */
  void initializeServiceServers();

  /**
   * @brief Method to check of ArUco is detected and storing the pose
   *        by calling the /knd/foundToy service
   * @param None
   * @return int - 1 if ArUco is detected 0 otherwise
   */
  int detectArUco();

  /**
   * @brief Callback method for detection flag.
   * @param detectionFlag - const reference to a detection flag bool
   * @return None
   */
  void detectionCb(const std_msgs::Bool::ConstPtr &detectionFlag);

  /**
   * @brief ROS serice to check and return goal Pose if toy is found
   * @param req - service request uses integer id for a tag
   * @param resp - service response object contains bool for detection and
   *               geometry_msgs::PoseStamped msg for target Pose
   */
  bool findToySrv(kids_next_door::toyFound::Request &req,           //NOLINT
      kids_next_door::toyFound::Response &resp);  //NOLINT

  ~ToyDetection();

 private:
  /**
   * @brief tf listener to listen to target pose of the detected
   *        ArUco marker in the world frame
   */
  tf::TransformListener listener;

  /**
   * @brief rosservice server object for find toy service
   */
  ros::ServiceServer server;

  /**
   * @brief ros subscriber object for aruco tag detection flag
   */
  ros::Subscriber arucoSub;

  /**
   * @brief tf lookup to change the coordinate frame of the detected toy
   */
  tf::StampedTransform transform;

  /**
   * @brief tag ID of the current toy being searched for
   */
  double currToyID;

  /**
   * @brief Node handler object for the ToyDetection class
   */
  ros::NodeHandle nh;

  /**
   * @brief detection Flag bool msg for ArUco detection
   */
  std_msgs::Bool detectionFlag;

  /**
   * @brief PoseStamped msg for the detected ArUco tag(toy)
   */
  geometry_msgs::PoseStamped toyPose;
};

#endif  // INCLUDE_TOYDETECTION_HPP_
