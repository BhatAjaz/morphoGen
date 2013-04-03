/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef CTRL_CONST
#define CTRL_CONST

namespace iCub
{

namespace torqueCtrlTest
{

// the last element of the enum (SFC_COMMAND_COUNT) represents the total number of commands accepted by this module
enum TorqueCtrlTestCommand{ 
	no_ctrl,            pid_ctrl,           open_ctrl,     
    set_kp,             get_kp,
    set_kd,             get_kd,
    set_ki,             get_ki,
    set_taod,           get_taod,           get_tao,
    set_pwm,            get_pwm,
    set_alpha,          get_alpha,
    sim_on,             sim_off,
    set_joint,          get_joint,
    reset_pid,
    help,				quit,
    SFC_COMMAND_COUNT} ;

// the order of the command in this list MUST correspond to the order of the enum SkinForceCtrlCommand
const std::string TorqueCtrlTestCommand_s[]  = {
    "stop",                 "pid ctrl",             "open ctrl",
    "set kp",               "get kp",
    "set kd",               "get kd",
    "set ki",               "get ki",
    "set taod",             "get taod",             "get tao",
    "set pwm",              "get pwm",
    "set alpha",            "get alpha",
    "sim on",               "sim off",
    "set jnt",              "get jnt",
    "reset",
    "help",                 "quit"};

// the order in SkinForceCtrlCommand_desc must correspond to the order in SkinForceCtrlCommand_s
const std::string TorqueCtrlTestCommand_desc[]  = {
    "stop the controller",
	"set the control mode to pid", 
	"set the control mode to open loop", 
	"set the proportional gains (double)",
    "get the proportional gains (double)",
	"set the derivative gains (double)",
    "get the derivative gains (double)",
    "set the integral gains (double)",
    "get the integral gains (double)",
    "set the desired joint torques",
    "get the desired joint torques",
    "get the current joint torques",
    "set the desired pwm",
    "get the desired pwm",
	"set the intensity of the low pass filter (in [0,1])",
    "get the intensity of the low pass filter (in [0,1])",
    "activate the simulation mode (the computed torques are not commanded to the motors)",
    "deactivate the simulation mode (the computed torques are not commanded to the motors)",
    "set the joint to control",
    "get the joint to control",
    "reset the torque pid integral error",
    "get this list", 
	"quit the module"};

enum ControlThreadStatus { STATUS_OK=0, STATUS_DISCONNECTED };

enum ControlMode { NO_CONTROL, PID_CTRL, OPEN_CTRL, CONTROL_MODE_SIZE };
const std::string ControlMode_desc[] = { "NO_CONTROL", "PID_CTRL", "OPEN_CTRL"};

enum CommandMode { SIMULATION, REAL };              // SIMULATION: it doesn't send torques to the motors, but just to the output port

static const float  DEFAULT_ALPHA = 0.97f;             // contact force low pass filter intensity (in [0, 1])
static const int    DEFAULT_JOINT_ID = 3;              // id of the joint to control
static const int    DEFAULT_SAT_LIM_UP = 500;
static const int    DEFAULT_SAT_LIM_DOWN = -500;

// DEFAULT CONTROLLER GAINS
static const double DEFAULT_PID_KP[]   = { 100.0 };
static const double DEFAULT_PID_KI[]   = { 000.0 };
static const double DEFAULT_PID_KD[]   = { 000.0};
static const int DEFAULT_PID_KP_SIZE   = sizeof(DEFAULT_PID_KP)/sizeof(double);
static const int DEFAULT_PID_KI_SIZE   = sizeof(DEFAULT_PID_KI)/sizeof(double);
static const int DEFAULT_PID_KD_SIZE   = sizeof(DEFAULT_PID_KD)/sizeof(double);



// DEFAULT CONTROLLER SET POINTS
static const double DEFAULT_TAOD[]        = { 000.3 };

static const double GRAV_ACC = 9.81;

}

} // end namespace



#endif
