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

#include <string>
#include <vector>

#include "dvp_waypoint_replanner/dvp_waypoint_replanner_nodelet.hpp"
#include "dvp_waypoint_replanner/packet_definitions.hpp"

namespace dvp_waypoint_replanner
{
DvpWaypointReplannerNl::DvpWaypointReplannerNl()
{
}

DvpWaypointReplannerNl::~DvpWaypointReplannerNl()
{
  if (udp_server_running_)
  {
    udp_server_.stop();
    udp_server_thread_.join();
  }
}

void DvpWaypointReplannerNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Publishers
  max_waypoints_pub_ = nh_.advertise<autoware_msgs::LaneArray>("based/lane_waypoints_raw", 10, true);
  mod_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("base_waypoints", 10, true);

  // Subscribers
  raw_waypoints_sub_ =
      nh_.subscribe("based/lane_waypoints_file", 1, &DvpWaypointReplannerNl::rawWaypointsCallback, this);
  envelope_waypoints_sub_ =
      nh_.subscribe("base_waypoints_envelope", 1, &DvpWaypointReplannerNl::envelopeWaypointsCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &DvpWaypointReplannerNl::velocityCallback, this);
  vehicle_status_sub_ = nh_.subscribe("vehicle_status", 1, &DvpWaypointReplannerNl::vehicleStatusCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &DvpWaypointReplannerNl::closestWaypointCallback, this);

  if (startServer())
  {
    udp_server_running_ = true;
    udp_server_thread_ = std::thread(std::bind(&AS::Network::UDPServer::run, &udp_server_));
  }
  else
  {
    udp_server_running_ = false;
    ros::requestShutdown();
  }
}

void DvpWaypointReplannerNl::loadParams()
{
  pnh_.param<std::string>("server_ip", server_ip_, "127.0.0.1");
  pnh_.param("server_port", server_port_, 1551);
  pnh_.param("max_speed", max_speed_, 100.0);

  ROS_INFO("Parameters Loaded");
}

std::vector<uint8_t> DvpWaypointReplannerNl::handleServerResponse(const std::vector<uint8_t>& received_payload)
{
  DVPMod::VelocityProfile velocity_profile;
  if (velocity_profile.unpack(received_payload))
  {
    generateGlobalPath(velocity_profile);
  }
  else
  {
    ROS_ERROR("Checksum doesn't match! dropping packet");
  }

  // Create reply message
  bool data_ready = false;
  ros::Time now = ros::Time::now();

  std::unique_lock<std::mutex> lock(status_data_mtx_);
  if ((now - current_velocity_rcvd_time_) <= OUTDATED_DATA_TIMEOUT &&
      (now - dbw_engaged_rcvd_time_) <= OUTDATED_DATA_TIMEOUT &&
      (now - closest_waypoint_id_rcvd_time_) <= OUTDATED_DATA_TIMEOUT)
  {
    data_ready = true;
  }

  DVPMod::StatusReply reply = {};
  reply.msg_id = velocity_profile.msg_id;
  reply.data_valid = data_ready;
  reply.path_tracking_enabled = dbw_engaged_;
  reply.velocity_profile_enabled = velocity_profile_enabled_;

  reply.closest_global_waypoint_id = closest_waypoint_id_;
  reply.target_global_velocity = target_velocity_;
  reply.current_velocity = current_velocity_;
  reply.crc = 0;

  return reply.pack();
}

void DvpWaypointReplannerNl::generateGlobalPath(DVPMod::VelocityProfile velocity_profile)
{
  std::unique_lock<std::mutex> wp_lock(waypoints_mtx_);

  if (received_raw_waypoints_ && received_envelope_waypoints_)
  {
    autoware_msgs::Lane modified_waypoints = envelope_waypoints_;
    wp_lock.unlock();

    uint num_waypoints = modified_waypoints.waypoints.size();

    if (num_waypoints != raw_waypoints_.waypoints.size())
    {
      ROS_ERROR("Waypoints don't match! unable to modify velocities");
    }
    else
    {
      // First set all waypoints to original velocities, never exceeding envelope
      for (uint i = 0; i < num_waypoints; ++i)
      {
        modified_waypoints.waypoints[i].twist.twist.linear.x = std::min(
            raw_waypoints_.waypoints[i].twist.twist.linear.x, envelope_waypoints_.waypoints[i].twist.twist.linear.x);
      }

      // Now modify based on UDP packet profile, never exceeding envelope
      if (velocity_profile.status == 1)
      {
        for (uint i = 0; i < 50; ++i)
        {
          uint wp_id = i + velocity_profile.first_global_waypoint_id;
          if (wp_id < num_waypoints)
          {
            double profile_vel = static_cast<double>(velocity_profile.target_velocities[i]) / 1000.0;

            modified_waypoints.waypoints[wp_id].twist.twist.linear.x =
                std::min(profile_vel, envelope_waypoints_.waypoints[wp_id].twist.twist.linear.x);
          }
        }
        std::unique_lock<std::mutex> status_lock(status_data_mtx_);
        velocity_profile_enabled_ = true;
      }
      else
      {
        std::unique_lock<std::mutex> status_lock(status_data_mtx_);
        velocity_profile_enabled_ = false;
      }

      std::unique_lock<std::mutex> status_lock(status_data_mtx_);
      target_velocity_ = static_cast<uint16_t>(
          std::round(std::abs(modified_waypoints.waypoints[closest_waypoint_id_].twist.twist.linear.x) * 1000.0));
    }

    mod_waypoints_pub_.publish(modified_waypoints);
  }
  else
  {
    ROS_WARN_THROTTLE(2.0, "Waiting for waypoints");
  }
}

bool DvpWaypointReplannerNl::startServer()
{
  AS::Network::ReturnStatuses status = udp_server_.open(server_ip_, server_port_);

  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not start UDP server: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
    return false;
  }
  else
  {
    ROS_INFO("UDP server started");

    udp_server_.registerReceiveHandler(
        std::bind(&DvpWaypointReplannerNl::handleServerResponse, this, std::placeholders::_1));

    return true;
  }
}

void DvpWaypointReplannerNl::rawWaypointsCallback(const autoware_msgs::LaneArray::ConstPtr& raw_waypoints)
{
  if (raw_waypoints->lanes.size() > 1)
  {
    ROS_ERROR("Dynamic velocity profile replanning feature does not support multiple lanes");
    return;
  }

  raw_waypoints_ = raw_waypoints->lanes[0];

  // Set all waypoints to max_speed, to determine global velocity envelope with
  // stop signs/lights taken into account
  autoware_msgs::LaneArray max_waypoints = *raw_waypoints;
  for (auto& waypoint : max_waypoints.lanes[0].waypoints)
  {
    waypoint.twist.twist.linear.x = max_speed_ / 3.6;
  }

  max_waypoints_pub_.publish(max_waypoints);

  std::unique_lock<std::mutex> lock(waypoints_mtx_);
  received_raw_waypoints_ = true;
}

void DvpWaypointReplannerNl::envelopeWaypointsCallback(const autoware_msgs::Lane::ConstPtr& envelope_waypoints)
{
  std::unique_lock<std::mutex> lock(waypoints_mtx_);
  envelope_waypoints_ = *envelope_waypoints;
  received_envelope_waypoints_ = true;
  lock.unlock();

  // Publish global path using empty profile to publish original path
  DVPMod::VelocityProfile empty_profile;
  generateGlobalPath(empty_profile);
}

void DvpWaypointReplannerNl::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& current_velocity)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  current_velocity_ = static_cast<uint16_t>(std::round(std::abs(current_velocity->twist.linear.x) * 1000.0));
  current_velocity_rcvd_time_ = ros::Time::now();
}

void DvpWaypointReplannerNl::vehicleStatusCallback(const autoware_msgs::VehicleStatus::ConstPtr& vehicle_status)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  dbw_engaged_ = (vehicle_status->drivemode == autoware_msgs::VehicleStatus::MODE_AUTO ||
                  vehicle_status->steeringmode == autoware_msgs::VehicleStatus::MODE_AUTO);
  dbw_engaged_rcvd_time_ = ros::Time::now();
}

void DvpWaypointReplannerNl::closestWaypointCallback(const std_msgs::Int32::ConstPtr& closest_waypoint_id)
{
  std::unique_lock<std::mutex> lock(status_data_mtx_);
  if (!received_raw_waypoints_)
  {
    return;
  }

  if (closest_waypoint_id->data >= 0 && closest_waypoint_id->data < (int32_t)raw_waypoints_.waypoints.size())
  {
    closest_waypoint_id_ = closest_waypoint_id->data;
    closest_waypoint_id_rcvd_time_ = ros::Time::now();
  }
}

}  // namespace dvp_waypoint_replanner

PLUGINLIB_EXPORT_CLASS(dvp_waypoint_replanner::DvpWaypointReplannerNl, nodelet::Nodelet);
