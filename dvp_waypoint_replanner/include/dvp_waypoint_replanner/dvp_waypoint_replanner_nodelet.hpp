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

#ifndef DVP_WAYPOINT_REPLANNER_DVPWAYPOINTREPLANNERNL_H
#define DVP_WAYPOINT_REPLANNER_DVPWAYPOINTREPLANNERNL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <network_interface/udp_server.h>
#include <thread>
#include <mutex>

#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/VehicleStatus.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>

#include "dvp_waypoint_replanner/packet_definitions.hpp"

const ros::Duration OUTDATED_DATA_TIMEOUT(0.5);

namespace dvp_waypoint_replanner
{
class DvpWaypointReplannerNl : public nodelet::Nodelet
{
public:
  DvpWaypointReplannerNl();
  ~DvpWaypointReplannerNl();

private:
  // Init
  virtual void onInit();
  void loadParams();

  // Subscriber callbacks
  void rawWaypointsCallback(const autoware_msgs::LaneArray::ConstPtr& raw_waypoints);
  void envelopeWaypointsCallback(const autoware_msgs::Lane::ConstPtr& envelope_waypoints);
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity);
  void vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status);
  void closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id);

  // UDP server callback for new data received
  std::vector<uint8_t> handleServerResponse(const std::vector<uint8_t>& received_payload);

  // Util functions
  bool startServer();
  void generateGlobalPath(DVPMod::VelocityProfile velocity_profile);

  // Nodehandles
  ros::NodeHandle nh_, pnh_;

  // Publishers
  ros::Publisher max_waypoints_pub_, mod_waypoints_pub_;

  // Subscribers
  ros::Subscriber raw_waypoints_sub_, envelope_waypoints_sub_;
  ros::Subscriber current_velocity_sub_, vehicle_status_sub_, closest_waypoint_sub_;

  // Internal state
  AS::Network::UDPServer udp_server_;
  std::thread udp_server_thread_;
  std::mutex waypoints_mtx_;
  std::mutex status_data_mtx_;
  bool udp_server_running_ = false;
  bool received_raw_waypoints_ = false;
  bool received_envelope_waypoints_ = false;
  autoware_msgs::Lane raw_waypoints_;
  autoware_msgs::Lane envelope_waypoints_;
  bool velocity_profile_enabled_ = false;

  // Current velocity of the vehicle (mm/s)
  uint16_t current_velocity_ = 0;
  ros::Time current_velocity_rcvd_time_;

  // DBW in manual or autonomy
  bool dbw_engaged_ = false;
  ros::Time dbw_engaged_rcvd_time_;

  // Closest global waypoint id
  int32_t closest_waypoint_id_ = 0;
  ros::Time closest_waypoint_id_rcvd_time_;

  // Velocity at the closest global waypoint (mm/s)
  uint16_t target_velocity_ = 0;

  // Parameters
  std::string server_ip_;
  int server_port_;

  // Highest allowed speed of any waypoint on the global path (km/h)
  double max_speed_;
};

}  // namespace dvp_waypoint_replanner
#endif  // DVP_WAYPOINT_REPLANNER_DVPWAYPOINTREPLANNERNL_H
