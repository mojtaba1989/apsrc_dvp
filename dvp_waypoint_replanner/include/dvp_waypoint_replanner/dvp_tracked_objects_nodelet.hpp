/*
 * AutonomouStuff, LLC ("COMPANY") CONFIDENTIAL
 * Unpublished Copyright (c) 2009-2021 AutonomouStuff, LLC, All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of COMPANY. The intellectual and technical
 * concepts contained herein are proprietary to COMPANY and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is obtained from COMPANY.  Access to the source
 * code contained herein is hereby forbidden to anyone except current COMPANY employees, managers or contractors who
 * have executed Confidentiality and Non-disclosure agreements explicitly covering such access.
 *
 * The copyright notice above does not evidence any actual or intended publication or disclosure  of  this source code,
 * which includes information that is confidential and/or proprietary, and is a trade secret, of  COMPANY.   ANY
 * REPRODUCTION, MODIFICATION, DISTRIBUTION, PUBLIC  PERFORMANCE, OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS  SOURCE
 * CODE  WITHOUT  THE EXPRESS WRITTEN CONSENT OF COMPANY IS STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND
 * INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR
 * IMPLY ANY RIGHTS TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE, USE, OR SELL ANYTHING THAT IT
 * MAY DESCRIBE, IN WHOLE OR IN PART.
 */

#ifndef DVP_WAYPOINT_REPLANNER_DVPTRACKEDOBJECTSNL_H
#define DVP_WAYPOINT_REPLANNER_DVPTRACKEDOBJECTSNL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <network_interface/network_interface.h>

#include <autoware_msgs/DetectedObjectArray.h>

#include "dvp_waypoint_replanner/packet_definitions.hpp"

namespace dvp_tracked_objects
{
class DvpTrackedObjectsNl : public nodelet::Nodelet
{
private:
  // Init
  virtual void onInit();
  void loadParams();

  // Subscriber callbacks
  void trackedObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& tracked_objects);

  // Util functions
  uint8_t convertClassification(const std::string& classification);
  void openConnection();

  // Nodehandles
  ros::NodeHandle nh_, pnh_;

  // Subscribers
  ros::Subscriber tracked_objects_sub_;

  // Internal state
  AS::Network::UDPInterface udp_interface_;

  // Parameters
  std::string destination_ip_;
  int destination_port_;

  // Cutoff angle in radians, any object falling outside of +-cutoff angle is not included
  double cutoff_angle_ = 0.7854;

  // Objects with a classification score below the threshold will be excluded
  double score_threshold_ = 0.8;
};

}  // namespace dvp_tracked_objects
#endif  // DVP_WAYPOINT_REPLANNER_DVPTRACKEDOBJECTSNL_H
