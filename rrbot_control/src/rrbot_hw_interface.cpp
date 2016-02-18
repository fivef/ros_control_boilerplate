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

/* Author: Steffen Pfiffner based on boilerplate example of Dave Coleman
   Desc:   Hardware interface for accessing the scara controller via serial messages
           (binary as produced/received by matlab)
*/

#include <rrbot_control/rrbot_hw_interface.h>

namespace rrbot_control
{

RRBotHWInterface::RRBotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
  : ros_control_boilerplate::GenericHWInterface(nh, urdf_model)
{

  //TODO: make rosparam
  string port("/dev/ttyACM0");

  my_serial.setPort(port);
  my_serial.open();

    cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;



  ROS_INFO_NAMED("rrbot_hw_interface", "RRBotHWInterface Ready.");
}



void RRBotHWInterface::read(ros::Duration &elapsed_time)
{

  std::string message = my_serial.readline();

  InputMessageFormat input_message;

  memcpy(&input_message, message.data(), sizeof(InputMessageFormat));


  ROS_DEBUG_STREAM_NAMED("received_serial_msgs", std::endl

                                 << "\nlen message:"
                                << message.length()
                                << "\nsturct size:"
                                << sizeof(InputMessageFormat)
                                << "\nheader1: "
                                << input_message.header1
                                << "\nheader2: "
                                << input_message.header2
                                << "\npid output: "
                                << input_message.pid_output
                                << "\naxis_1_direction: "
                                << input_message.axis_1_direction
                                << "\naxis_1_position: "
                                << input_message.axis_1_position


                                );

                                /*  
                                
                                << "axis_1_pid_output: "
                                << input_message->axis_1_pid_output
                                << "axis_1_direction: "
                                << input_message->axis_1_direction);
*/
  printStdStringAsHex(message);


  //TODO: for each joint:
    /*
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    joint_position_[joint_id] += joint_position_command_[joint_id];
  */
  joint_position_[0] = input_message.axis_1_position;

  ROS_INFO_STREAM_THROTTLE(0.1, std::endl
                                 << printStateHelper());


}



void RRBotHWInterface::write(ros::Duration &elapsed_time)
{
  // Safety
  enforceLimits(elapsed_time);

  ROS_INFO_STREAM_THROTTLE(0.1, std::endl
                                  << printCommandHelper());

  OutputMessageFormat output_message;

  output_message.header1 = 14; //fixed header
  output_message.header2 = 14; //fixed header
  output_message.joint_1_speed = joint_velocity_command_[0];
  output_message.linefeed = '\n';


  char output_message_string[sizeof(OutputMessageFormat)]; //(reinterpret_cast<char const*>(output_message), sizeof(OutputMessageFormat));
  //output_message_string.resize(sizeof(OutputMessageFormat));

  memcpy(output_message_string, (unsigned char *) &output_message, sizeof(OutputMessageFormat));


  std::string out_string(output_message_string, sizeof(OutputMessageFormat));

  printStdStringAsHex(out_string);


  size_t bytes_written = my_serial.write(out_string);

  ROS_INFO_STREAM_NAMED("sent_serial_msg_bytes_number", std::endl
                                  << "out sting len: "
                                  << out_string.length()
                                  << "\noutput_message_struct_size: "
                                  << sizeof(OutputMessageFormat)
                                  << "\n joint_1 velocity: "
                                  << joint_velocity_command_[0]
                                  << "\n# bytes written: "
                                  << bytes_written);

  /*TODO Exception handling
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   * \throw serial::IOException
   */


  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // FILL IN YOUR WRITE COMMAND TO USB/ETHERNET/ETHERCAT/SERIAL ETC HERE
  //
  // FOR A EASY SIMULATION EXAMPLE, OR FOR CODE TO CALCULATE
  // VELOCITY FROM POSITION WITH SMOOTHING, SEE
  // sim_hw_interface.cpp IN THIS PACKAGE
  //
  // DUMMY PASS-THROUGH CODE
  /*
  for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    joint_position_[joint_id] += joint_position_command_[joint_id];
  */
  // END DUMMY CODE
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void RRBotHWInterface::enforceLimits(ros::Duration &period)
{
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
  //
  // CHOOSE THE TYPE OF JOINT LIMITS INTERFACE YOU WANT TO USE
  // YOU SHOULD ONLY NEED TO USE ONE SATURATION INTERFACE,
  // DEPENDING ON YOUR CONTROL METHOD
  //
  // EXAMPLES:
  //
  // Saturation Limits ---------------------------
  //
  // Enforces position and velocity
  pos_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces velocity and acceleration limits
  vel_jnt_sat_interface_.enforceLimits(period);
  //
  // Enforces position, velocity, and effort
  // eff_jnt_sat_interface_.enforceLimits(period);

  // Soft limits ---------------------------------
  //
  // pos_jnt_soft_limits_.enforceLimits(period);
   //vel_jnt_soft_limits_.enforceLimits(period);
   //eff_jnt_soft_limits_.enforceLimits(period);
  //
  // ----------------------------------------------------
  // ----------------------------------------------------
  // ----------------------------------------------------
}

void RRBotHWInterface::printStdStringAsHex(std::string &string){

    //print as hex
  std::stringstream ss;
  
  ss << std::hex << std::setfill('0');
  for (size_t i = 0; string.length() > i; ++i) {
      ss << std::setw(2) << static_cast<unsigned int>(static_cast<unsigned char>(string[i]));
  }

  ROS_INFO_STREAM_NAMED("bytes_in_hex", std::endl << "Bytes in hex: "
                                  << ss.str());

}

}  // namespace
