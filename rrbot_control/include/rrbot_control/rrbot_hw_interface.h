/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#ifndef RRBOT_CONTROL__RRBOT_HW_INTERFACE_H
#define RRBOT_CONTROL__RRBOT_HW_INTERFACE_H

#include <ros_control_boilerplate/generic_hw_interface.h>
#include <math.h>
#include "serial/serial.h"


using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

namespace rrbot_control
{

#pragma pack(1)
struct InputMessageFormat {
    unsigned char header1; //fixed header
    unsigned char header2; //fixed header
    double unused1;
    double unused2;
    double unused3;
    double velocity_control_pid_output;
    int unused4;
    int joint_1_encoder_position;
    int joint_1_speed;
    int unused5;
    int joint_2_encoder_position;
    int joint_2_speed;
    int joint_1_error;
    int encoder_1_index_count;
    int encoder_2_index_count;
    int reset_state;
    int unused6;
    int unused7;
    int unused8;
    int loopback;
};

struct OutputMessageFormat {
    unsigned char header1; //fixed header
    unsigned char header2; //fixed header
    double joint_1_direct_speed_set; //directly sets the speed if not 0 (overrides pid loop and so the setpoint)
    double joint_2_direct_speed_set;
    double p; //continuously set the pid values
    double i;
    double d;
    int joint_1_speed_setpoint; //this ist the actual setpoint!!!
    int joint_2_speed_setpoint;
    int home_axis; //not implemented yet?
    int reset_encoders; //set to 1 to reset and then back to 0
    int loopback;
    unsigned char terminator1;
    unsigned char terminator2;
};
#pragma pack(0)

/// \brief Hardware interface for a robot
class RRBotHWInterface : public ros_control_boilerplate::GenericHWInterface
{
public:
  /**
   * \brief Constructor
   * \param nh - Node handle for topics.
   */
  RRBotHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Read the state from the robot hardware. */
  virtual void read(ros::Duration &elapsed_time);

  /** \brief Write the command to the robot hardware. */
  virtual void write(ros::Duration &elapsed_time);

  /** \breif Enforce limits for all values before writing */
  virtual void enforceLimits(ros::Duration &period);

  /** \brief prints a std::string as hexadecimal string */
  std::string stringToHexStream(std::string &string);

  serial::Serial my_serial;

  /** \brief on test motor one rotation is 7000 encoder impulses */
  int encoder_steps_per_rotation = 7000;

  double radians_per_encoder_step = M_PI/encoder_steps_per_rotation;

};  // class

}  // namespace

#endif
