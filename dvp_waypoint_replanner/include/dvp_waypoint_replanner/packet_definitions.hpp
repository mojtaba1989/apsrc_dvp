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

#ifndef PACKET_DEFINITIONS_H
#define PACKET_DEFINITIONS_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <boost/crc.hpp>

namespace DVPMod
{
class VelocityProfile
{
public:
  uint8_t msg_id;
  uint8_t status;
  uint16_t first_global_waypoint_id;
  uint16_t target_velocities[50];
  uint8_t reserved[10];
  uint32_t crc;

  std::vector<uint8_t> pack()
  {
    std::vector<uint8_t> buffer(118);
    buffer[0] = msg_id;
    buffer[1] = status;
    std::memcpy(&buffer[2], &first_global_waypoint_id, sizeof(first_global_waypoint_id));
    std::memcpy(&buffer[4], target_velocities, sizeof(target_velocities));
    std::memcpy(&buffer[104], reserved, sizeof(reserved));

    // Calculate CRC
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 114);
    crc = msg_crc.checksum();

    std::memcpy(&buffer[114], &crc, sizeof(crc));

    return buffer;
  }

  bool unpack(const std::vector<uint8_t>& buffer)
  {
    msg_id = buffer[0];
    status = buffer[1];
    std::memcpy(&first_global_waypoint_id, &buffer[2], sizeof(first_global_waypoint_id));
    std::memcpy(target_velocities, &buffer[4], sizeof(target_velocities));
    std::memcpy(reserved, &buffer[104], sizeof(reserved));
    std::memcpy(&crc, &buffer[114], sizeof(crc));

    // Confirm CRC
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 114);
    if (crc == msg_crc.checksum())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

class StatusReply
{
public:
  uint8_t msg_id;
  bool data_valid;
  bool path_tracking_enabled;
  bool velocity_profile_enabled;
  uint16_t closest_global_waypoint_id;
  uint16_t target_global_velocity;
  uint16_t current_velocity;
  uint8_t reserved[10];
  uint32_t crc;

  std::vector<uint8_t> pack()
  {
    std::vector<uint8_t> buffer(24);
    buffer[0] = msg_id;
    buffer[1] = static_cast<uint8_t>(data_valid);
    buffer[2] = static_cast<uint8_t>(path_tracking_enabled);
    buffer[3] = static_cast<uint8_t>(velocity_profile_enabled);
    std::memcpy(&buffer[4], &closest_global_waypoint_id, sizeof(closest_global_waypoint_id));
    std::memcpy(&buffer[6], &target_global_velocity, sizeof(target_global_velocity));
    std::memcpy(&buffer[8], &current_velocity, sizeof(current_velocity));
    std::memcpy(&buffer[10], reserved, sizeof(reserved));

    // Calculate CRC
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 20);
    crc = msg_crc.checksum();

    std::memcpy(&buffer[20], &crc, sizeof(crc));

    return buffer;
  }

  bool unpack(const std::vector<uint8_t>& buffer)
  {
    msg_id = buffer[0];
    data_valid = static_cast<bool>(buffer[1] & 0x01);
    path_tracking_enabled = static_cast<bool>(buffer[2] & 0x01);
    velocity_profile_enabled = static_cast<bool>(buffer[3] & 0x01);
    std::memcpy(&closest_global_waypoint_id, &buffer[4], sizeof(closest_global_waypoint_id));
    std::memcpy(&target_global_velocity, &buffer[6], sizeof(target_global_velocity));
    std::memcpy(&current_velocity, &buffer[8], sizeof(current_velocity));
    std::memcpy(reserved, &buffer[10], sizeof(reserved));
    std::memcpy(&crc, &buffer[20], sizeof(crc));

    // Confirm CRC
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 20);
    if (crc == msg_crc.checksum())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

class TrackedObjects
{
public:
  uint8_t num_objects;
  int16_t x_pos[30];
  int16_t y_pos[30];
  int16_t speed[30];
  int16_t heading[30];
  uint8_t classification[30];
  uint16_t size[30];
  uint32_t id[30];
  uint8_t reserved[10];
  uint32_t crc;

  std::vector<uint8_t> pack()
  {
    std::vector<uint8_t> buffer(465);
    buffer[0] = num_objects;
    std::memcpy(&buffer[1], x_pos, sizeof(x_pos));
    std::memcpy(&buffer[61], y_pos, sizeof(y_pos));
    std::memcpy(&buffer[121], speed, sizeof(speed));
    std::memcpy(&buffer[181], heading, sizeof(heading));
    std::memcpy(&buffer[241], classification, sizeof(classification));
    std::memcpy(&buffer[271], size, sizeof(size));
    std::memcpy(&buffer[331], id, sizeof(id));
    std::memcpy(&buffer[451], reserved, sizeof(reserved));

    // Calculate CRC
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 461);
    crc = msg_crc.checksum();

    std::memcpy(&buffer[461], &crc, sizeof(crc));

    return buffer;
  }

  bool unpack(const std::vector<uint8_t>& buffer)
  {
    num_objects = buffer[0];
    std::memcpy(x_pos, &buffer[1], sizeof(x_pos));
    std::memcpy(y_pos, &buffer[61], sizeof(y_pos));
    std::memcpy(speed, &buffer[121], sizeof(speed));
    std::memcpy(heading, &buffer[181], sizeof(heading));
    std::memcpy(classification, &buffer[241], sizeof(classification));
    std::memcpy(size, &buffer[271], sizeof(size));
    std::memcpy(id, &buffer[331], sizeof(id));
    std::memcpy(reserved, &buffer[451], sizeof(reserved));
    std::memcpy(&crc, &buffer[461], sizeof(crc));

    // Confirm CRC
    boost::crc_32_type msg_crc;
    msg_crc.process_bytes(&buffer[0], 461);
    if (crc == msg_crc.checksum())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
};

}  // namespace DVPMod

#endif  // PACKET_DEFINITIONS_H
