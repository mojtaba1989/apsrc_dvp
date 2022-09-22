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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "dvp_waypoint_replanner/dvp_tracked_objects_nodelet.hpp"
#include "dvp_waypoint_replanner/packet_definitions.hpp"

namespace dvp_tracked_objects
{
void DvpTrackedObjectsNl::onInit()
{
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();
  loadParams();

  // Subscribers
  tracked_objects_sub_ = nh_.subscribe("tracking/objects", 1, &DvpTrackedObjectsNl::trackedObjectsCallback, this);
}

void DvpTrackedObjectsNl::loadParams()
{
  pnh_.param<std::string>("destination_ip", destination_ip_, "127.0.0.1");
  pnh_.param("destination_port", destination_port_, 1552);
  pnh_.param("cutoff_angle", cutoff_angle_, 0.7854);
  pnh_.param("score_threshold", score_threshold_, 0.8);

  ROS_INFO("Parameters Loaded");
}

void DvpTrackedObjectsNl::trackedObjectsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr& tracked_objects)
{
  if (!udp_interface_.is_open())
  {
    openConnection();
  }
  else
  {
    if (tracked_objects->header.frame_id != "lidar" && tracked_objects->header.frame_id != "velodyne")
    {
      ROS_ERROR("Tracked objects not in lidar frame!");
      return;
    }

    std::vector<autoware_msgs::DetectedObject> filtered_objects;
    for (auto& object : tracked_objects->objects)
    {
      double angle = std::atan2(object.pose.position.y, object.pose.position.x);

      if (std::abs(angle) <= cutoff_angle_ && object.score >= score_threshold_)
      {
        filtered_objects.push_back(object);
      }
    }

    if (filtered_objects.size() > 30)
    {
      // Sort and remove the objects furthest away
      std::sort(filtered_objects.begin(), filtered_objects.end(),
                [](const autoware_msgs::DetectedObject& a, const autoware_msgs::DetectedObject& b) {
                  double dist_a = sqrt(a.pose.position.x * a.pose.position.x + a.pose.position.y * a.pose.position.y);
                  double dist_b = sqrt(b.pose.position.x * b.pose.position.x + b.pose.position.y * b.pose.position.y);
                  return (dist_a < dist_b);
                });

      filtered_objects.resize(30);
    }

    // Build TrackedObjects packet
    DVPMod::TrackedObjects tracked_objects = {};
    tracked_objects.num_objects = static_cast<uint8_t>(filtered_objects.size());

    for (size_t i = 0; i < filtered_objects.size(); i++)
    {
      // Convert positions from (m) to (cm)
      tracked_objects.x_pos[i] = static_cast<int16_t>(std::round(filtered_objects[i].pose.position.x * 100.0));
      tracked_objects.y_pos[i] = static_cast<int16_t>(std::round(filtered_objects[i].pose.position.y * 100.0));

      // Convert speed from (m/s) to (mm/s)
      tracked_objects.speed[i] = static_cast<int16_t>(std::round(filtered_objects[i].velocity.linear.x * 1000.0));

      // Convert Quaternion to heading (yaw)
      tf2::Quaternion q(filtered_objects[i].pose.orientation.x, filtered_objects[i].pose.orientation.y,
                        filtered_objects[i].pose.orientation.z, filtered_objects[i].pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      tracked_objects.heading[i] = static_cast<int16_t>(std::round(yaw * 100.0));

      tracked_objects.classification[i] = convertClassification(filtered_objects[i].label);

      // Convert size from (m) to (mm)
      tracked_objects.size[i] = static_cast<uint16_t>(
          std::round(std::max(filtered_objects[i].dimensions.x, filtered_objects[i].dimensions.y) * 1000.0));
      tracked_objects.id[i] = static_cast<uint32_t>(filtered_objects[i].id);
    }

    ROS_DEBUG("Sending %lu objects", filtered_objects.size());

    udp_interface_.write(tracked_objects.pack());
  }
}

uint8_t DvpTrackedObjectsNl::convertClassification(const std::string& classification)
{
  if (classification == "car")
  {
    return 2;
  }
  else if (classification == "pedestrian")
  {
    return 3;
  }
  else if (classification == "bike")
  {
    return 4;
  }
  else
  {
    return 1;
  }
}

void DvpTrackedObjectsNl::openConnection()
{
  AS::Network::ReturnStatuses status = udp_interface_.open(destination_ip_, destination_port_);

  if (status != AS::Network::ReturnStatuses::OK)
  {
    ROS_ERROR("Could not open UDP interface: %d - %s", static_cast<int>(status), return_status_desc(status).c_str());
  }
  else
  {
    ROS_INFO("UDP interface opened, sending to %s:%d", destination_ip_.c_str(), destination_port_);
  }
}

}  // namespace dvp_tracked_objects

PLUGINLIB_EXPORT_CLASS(dvp_tracked_objects::DvpTrackedObjectsNl, nodelet::Nodelet);
