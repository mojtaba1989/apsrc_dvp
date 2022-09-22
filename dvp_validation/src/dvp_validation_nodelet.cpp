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

#include "dvp_validation/dvp_validation_nodelet.hpp"

#include <string>
#include <vector>
#include <math.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <dvp_waypoint_replanner/packet_definitions.hpp>

namespace dvp_validation
{
DvpValidationNl::DvpValidationNl()
{
}

DvpValidationNl::~DvpValidationNl()
{
  if (udp_server_running_)
  {
    udp_server_.stop();
    udp_server_thread_.join();
  }
}

void DvpValidationNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Publishers
  objects_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/tracked_objects_udp_viz", 10, false);

  // Subscribers
  joy_sub_ = nh_.subscribe("joy", 1000, &DvpValidationNl::joyCallback, this);

  // Timers
  udp_poll_timer_ = nh_.createTimer(ros::Duration(0.1), &DvpValidationNl::processNewVelProfile, this);

  // Open connection to objects publisher:
  if (openObjectsListener())
  {
    udp_server_running_ = true;
    udp_server_thread_ = std::thread(std::bind(&AS::Network::UDPServer::run, &udp_server_));
  }
}

void DvpValidationNl::loadParams()
{
  pnh_.param<std::string>("server_ip", server_ip_, "127.0.0.1");
  pnh_.param("server_port", server_port_, 1551);
  pnh_.param("receive_buffer_size", receive_buffer_size_, 24);
  pnh_.param<std::string>("objects_ip", objects_ip_, "127.0.0.1");
  pnh_.param("objects_port", objects_port_, 1552);

  vehicle_color_.r = 0.0;
  vehicle_color_.g = 0.4;
  vehicle_color_.b = 1.0;
  vehicle_color_.a = 1.0;

  unknown_color_.r = 0.5;
  unknown_color_.g = 0.5;
  unknown_color_.b = 0.5;
  unknown_color_.a = 1.0;

  pedestrian_color_.r = 1.0;
  pedestrian_color_.g = 0.0;
  pedestrian_color_.b = 0.0;
  pedestrian_color_.a = 1.0;

  bicycle_color_.r = 0.0;
  bicycle_color_.g = 1.0;
  bicycle_color_.b = 0.0;
  bicycle_color_.a = 1.0;

  ROS_INFO("Parameters Loaded");
}

void DvpValidationNl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (!initialized_)
  {
    ROS_WARN("Not yet initialized");
    return;
  }

  try
  {
    // Engage dynamic velocity planning
    if (joy_msg->buttons[START_BUTTON] == 1 && !engaged_)
    {
      engaged_ = true;
      ROS_INFO("Enabling dvp");
      profile_speed_ = current_speed_;
      return;
    }

    // Disengage dynamic velocity planning
    if (joy_msg->buttons[SELECT_BUTTON] == 1 && engaged_)
    {
      engaged_ = false;
      ROS_INFO("Disabling dvp");
      return;
    }

    // Adjust profile speed
    if (engaged_)
    {
      if (joy_msg->axes[DPAD_UD] == 1.0)
      {
        profile_speed_ += SPEED_ADJUST;
        ROS_INFO("Increase speed to %d", profile_speed_);
        return;
      }
      else if (joy_msg->axes[DPAD_UD] == -1.0)
      {
        // Prevent wrap-around
        if (profile_speed_ >= SPEED_ADJUST)
        {
          profile_speed_ -= SPEED_ADJUST;
        }
        ROS_INFO("Decrease speed to %d", profile_speed_);
        return;
      }
    }
  }
  catch (const std::out_of_range& oor)
  {
    ROS_ERROR("An out-of-range exception was caught. This probably means you selected the wrong controller type.");
  }
}

void DvpValidationNl::processNewVelProfile(const ros::TimerEvent& event)
{
  // Unused
  (void) event;

  if (!udp_interface_.is_open())
  {
    openConnection();
  }
  else
  {
    DVPMod::VelocityProfile velocity_profile = {};
    velocity_profile.msg_id = msg_id_++;

    // Don't actively modify path until we are initialized
    if (initialized_)
    {
      velocity_profile.status = engaged_;
    }
    else
    {
      velocity_profile.status = 0;
    }

    velocity_profile.first_global_waypoint_id = closest_wp_id_;

    for (int i = 0; i < 50; ++i)
    {
      velocity_profile.target_velocities[i] = profile_speed_;
    }
    velocity_profile.crc = 4;
    udp_interface_.write(velocity_profile.pack());

    // Wait for/receive response from UDP server
    std::vector<uint8_t> payload;
    AS::Network::ReturnStatuses rs = udp_interface_.read(&payload);

    if (rs != AS::Network::ReturnStatuses::OK)
    {
      ROS_ERROR("UDP read failed");
      return;
    }

    if (payload.size() == 0)
    {
      ROS_INFO("No packet received");
    }
    else
    {
      DVPMod::StatusReply reply;
      if (reply.unpack(payload))
      {
        ROS_INFO_THROTTLE(1.0, "id: %d, dv: %d, pt: %d, vp: %d, wp: %d, tvel: %d, cvel: %d", reply.msg_id,
                          reply.data_valid, reply.path_tracking_enabled, reply.velocity_profile_enabled,
                          reply.closest_global_waypoint_id, reply.target_global_velocity, reply.current_velocity);
      }
      else
      {
        ROS_ERROR("Checksum doesn't match! dropping packet");
        return;
      }

      if (reply.data_valid)
      {
        initialized_ = true;
        closest_wp_id_ = reply.closest_global_waypoint_id;
        current_speed_ = reply.current_velocity;
      }
      else
      {
        ROS_INFO("Received reply, but data not yet valid");
      }
    }
  }
}

void DvpValidationNl::openConnection()
{
  AS::Network::ReturnStatuses status = udp_interface_.open(server_ip_, server_port_, receive_buffer_size_);

  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not open UDP interface: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
  }
  else
  {
    ROS_INFO("UDP interface opened");
  }
}

bool DvpValidationNl::openObjectsListener()
{
  AS::Network::ReturnStatuses status = udp_server_.open(objects_ip_, objects_port_);

  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not start UDP listener: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
    return false;
  }

  ROS_INFO("UDP listener started");
  udp_server_.registerReceiveHandler(std::bind(&DvpValidationNl::handleServerResponse, this, std::placeholders::_1));

  return true;
}

std::vector<uint8_t> DvpValidationNl::handleServerResponse(const std::vector<uint8_t>& received_payload)
{
  std::vector<uint8_t> empty_reply;

  DVPMod::TrackedObjects tracked_objects;
  if (!tracked_objects.unpack(received_payload))
  {
    ROS_ERROR("Checksum mismatch! dropping packet");
    return empty_reply;
  }

  // First delete all existing/previous markers
  visualization_msgs::MarkerArray viz_markers;
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  viz_markers.markers.push_back(delete_marker);

  if (tracked_objects.num_objects > 0)
  {
    for (uint8_t i = 0; i < tracked_objects.num_objects; i++)
    {
      geometry_msgs::Point position;
      position.x = static_cast<double>(tracked_objects.x_pos[i]) / 100.0;
      position.y = static_cast<double>(tracked_objects.y_pos[i]) / 100.0;
      position.z = 0.0;

      viz_markers.markers.push_back(
          buildArrowMarker(i, position, static_cast<double>(tracked_objects.heading[i]) / 100.0,
                           tracked_objects.classification[i], static_cast<double>(tracked_objects.size[i]) / 1000.0));

      viz_markers.markers.push_back(
          buildLabelMarker(i, position, static_cast<double>(tracked_objects.speed[i]) / 1000.0, tracked_objects.id[i]));
    }

    objects_pub_.publish(viz_markers);
  }

  return empty_reply;
}

visualization_msgs::Marker DvpValidationNl::buildArrowMarker(uint8_t id, geometry_msgs::Point position, double heading,
                                                             uint8_t classification, double size)
{
  visualization_msgs::Marker arrow_marker;

  arrow_marker.header.frame_id = "lidar";
  arrow_marker.ns = "arrow_markers";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.type = visualization_msgs::Marker::ARROW;

  switch (classification)
  {
    case 1:
    {
      arrow_marker.color = unknown_color_;
      break;
    }
    case 2:
    {
      arrow_marker.color = vehicle_color_;
      break;
    }
    case 3:
    {
      arrow_marker.color = pedestrian_color_;
      break;
    }
    case 4:
    {
      arrow_marker.color = bicycle_color_;
      break;
    }
    default:
    {
      arrow_marker.color = unknown_color_;
      break;
    }
  }

  arrow_marker.id = id;
  arrow_marker.pose.position = position;

  tf2::Matrix3x3 rotation_matrix;
  tf2::Quaternion q;

  rotation_matrix.setEulerYPR(heading, 0, 0);
  rotation_matrix.getRotation(q);

  arrow_marker.pose.orientation.x = q.getX();
  arrow_marker.pose.orientation.y = q.getY();
  arrow_marker.pose.orientation.z = q.getZ();
  arrow_marker.pose.orientation.w = q.getW();

  // Set the scale of the arrow -- 1x1x1 here means 1m on a side
  arrow_marker.scale.x = size;
  arrow_marker.scale.y = 0.1;
  arrow_marker.scale.z = 0.1;

  return arrow_marker;
}

visualization_msgs::Marker DvpValidationNl::buildLabelMarker(uint8_t id, geometry_msgs::Point position, double speed,
                                                             uint32_t global_id)
{
  visualization_msgs::Marker label_marker;

  label_marker.header.frame_id = "lidar";
  label_marker.ns = "label_markers";
  label_marker.action = visualization_msgs::Marker::ADD;
  label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  label_marker.scale.x = 1.5;
  label_marker.scale.y = 1.5;
  label_marker.scale.z = 1.5;

  std_msgs::ColorRGBA label_color;
  label_color.r = 1.0;
  label_color.g = 1.0;
  label_color.b = 1.0;
  label_color.a = 1.0;

  label_marker.color = label_color;
  label_marker.id = id;

  // convert m/s to km/h
  std::stringstream kmh_velocity_stream;
  kmh_velocity_stream << std::fixed << std::setprecision(1) << (speed * 3.6);
  std::string text = "[" + std::to_string(global_id) + "]\n" + kmh_velocity_stream.str() + " km/h";

  label_marker.text += text;
  label_marker.pose.position = position;
  label_marker.scale.z = 1.0;

  return label_marker;
}

}  // namespace dvp_validation

PLUGINLIB_EXPORT_CLASS(dvp_validation::DvpValidationNl, nodelet::Nodelet);
