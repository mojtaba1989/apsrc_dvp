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

#ifndef DVP_VALIDATION_DVPVALIDATIONNL_H
#define DVP_VALIDATION_DVPVALIDATIONNL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <network_interface/network_interface.h>
#include <network_interface/udp_server.h>
#include <thread>

#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

const uint8_t START_BUTTON = 7;
const uint8_t SELECT_BUTTON = 6;
const uint8_t DPAD_LR = 6;
const uint8_t DPAD_UD = 7;
const uint16_t SPEED_ADJUST = 1000;

namespace dvp_validation
{
class DvpValidationNl : public nodelet::Nodelet
{
public:
  DvpValidationNl();
  ~DvpValidationNl();

private:
  // Init
  virtual void onInit();
  void loadParams();

  // Subscriber callbacks
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);

  // Timer callbacks
  void processNewVelProfile(const ros::TimerEvent& event);

  // Util functions
  void openConnection();
  bool openObjectsListener();
  std::vector<uint8_t> handleServerResponse(const std::vector<uint8_t>& received_payload);

  visualization_msgs::Marker buildArrowMarker(uint8_t id, geometry_msgs::Point position, double heading,
                                              uint8_t classification, double size);
  visualization_msgs::Marker buildLabelMarker(uint8_t id, geometry_msgs::Point position, double speed,
                                              uint32_t global_id);

  // Nodehandles, both public and private
  ros::NodeHandle nh_, pnh_;

  // Publishers
  ros::Publisher objects_pub_;

  // Subscribers
  ros::Subscriber joy_sub_;

  // Timers
  ros::Timer udp_poll_timer_;

  // Internal state
  AS::Network::UDPInterface udp_interface_;
  AS::Network::UDPServer udp_server_;
  std::thread udp_server_thread_;
  bool udp_server_running_ = false;
  uint8_t msg_id_ = 0;
  uint16_t closest_wp_id_ = 0;
  bool initialized_ = false;
  bool engaged_ = false;
  uint16_t current_speed_ = 0;
  uint16_t profile_speed_ = 0;

  // Parameters
  std::string server_ip_;
  int server_port_;
  int receive_buffer_size_;
  std::string objects_ip_;
  int objects_port_;

  // Colors for visualization
  std_msgs::ColorRGBA vehicle_color_;
  std_msgs::ColorRGBA unknown_color_;
  std_msgs::ColorRGBA pedestrian_color_;
  std_msgs::ColorRGBA bicycle_color_;
};

}  // namespace dvp_validation
#endif  // DVP_VALIDATION_DVPVALIDATIONNL_H
