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

#ifndef CTRL_THREAD
#define CTRL_THREAD

#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/ctrl/pids.h>
#include "iCub/torqueCtrlTest/controlConstants.h"
#include "iCub/torqueCtrlTest/robot_interfaces.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

namespace iCub
{

namespace torqueCtrlTest
{

class controlThread: public yarp::os::RateThread
{
private:

    VerbosityLevel          verbose;
    std::string             name;               // thread name
    ControlThreadStatus     thread_status;      // the status of the thread (OK, DISCONNECTED)
                                                // as soon as the status is DISCONNECTED, the module calls stop() on this thread
    ControlMode             ctrlMode;           // the type of control used
    bool                    ctrlModeChanged;    // true iff the ctrl mode has just been changed
    Semaphore               ctrlModeSem;        // semaphore managing the access to ctrlMode and ctrlModeChanged
    BodyPart                bodyPart;           // the body part of the robot to control
    CommandMode             cmdMode;            // SIMULATION: no torques are sent to the motors

    parallelPID*            torquePid;

    // NETWORK DELAY MANAGEMENT
    double                  timePre;
    vector<double>          durations;
    vector<double>          timestamps;
    vector<string>          durationDesc;

    // PORTS
    BufferedPort<Vector>    *port_torques;      // output port sending the same PWM that is commanded to the motor
    BufferedPort<Vector>    *port_monitor;      // output port sending data for monitoring the controller

    Vector                  monitorData;        // vector sent on the monitor port

    // CONTROL LOOP 
	robot_interfaces*   interfaces;
    int                 jointId;        // index of the controlled joint
    float               alpha;          // low pass filter intensity (in [0, 1])
    Vector              tao, taod;      // current and desired joint torques
    double              pwm, pwmD;      // pwm commanded to the robot joint motor, pwm desired (open ctrl mode)

    bool sanityCheck();
    void updateRobotStatus();

    void initCtrlMode(const ControlMode &cm);
    void computePwm(const ControlMode &cm);
    void sendPwm(const ControlMode &cm);
    void prepareMonitorData(const ControlMode &cm);
    void updateDurations(int index);

    inline Vector getParameter(string key){
        Bottle b;
        torquePid->getOptions(b);
        int size=1;
        Vector v(size);
        helperPID::getVectorFromOption(b, key.c_str(), v, size);
        return v; 
    }

    inline void setParameter(string key, Vector value, ControlMode cm){
        /*if(cm==NO_CONTROL)
            cm = getCtrlMode();*/
        Bottle b;
        helperPID::addVectorToOption(b, key.c_str(), value);
        torquePid->setOptions(b);
    }

public:	

    controlThread(std::string _moduleName, std::string _robotName, int _period, 
        iCub::skinDynLib::BodyPart _bodyPart, VerbosityLevel _verbose=NO_VERBOSE) ;
	
    bool threadInit();	
    void run();
    void threadRelease();

    // SET METHODS
    inline void setKp(Vector _kp, ControlMode cm=NO_CONTROL) { setParameter("Kp", _kp, cm); }
    inline void setKp(double _kp, ControlMode cm=NO_CONTROL) { setParameter("Kp", Vector(1, &_kp), cm); }
    inline void setKi(Vector _ki, ControlMode cm=NO_CONTROL) { setParameter("Ki", _ki, cm); }
    inline void setKi(double _ki, ControlMode cm=NO_CONTROL) { setParameter("Ki", Vector(1, &_ki), cm); }
    inline void setKd(Vector _kd, ControlMode cm=NO_CONTROL) { setParameter("Kd", _kd, cm); }
    inline void setKd(double _kd, ControlMode cm=NO_CONTROL) { setParameter("Kd", Vector(1, &_kd), cm);}
    inline void setTaod(Vector _taod) {
        if(_taod.size() != taod.size()){
            stringstream ss;
            ss<<"Taod wrong size "<< _taod.size();
            throw runtime_error(ss.str().c_str());
        }
        taod = _taod;
    }
    inline void setTaod(double _taod) { taod = _taod; }
    inline void setPwmD(double _pwmD) { pwmD = _pwmD; }
    inline void setAlpha(float _alpha) {
        if(_alpha<0.0 || alpha>1.0){
            stringstream ss;
            ss<<"Alpha out of range [0,1]: "<< alpha;
            throw runtime_error(ss.str().c_str());
        }
        alpha = _alpha;
    }

    inline void setCtrlMode(const ControlMode &cm){
        if(ctrlMode != cm){
            ctrlModeSem.wait();
            ctrlModeChanged = true;
            ctrlMode = cm;
            ctrlModeSem.post();
        }
    }

    // reset the integral error of the torque PID
    inline void resetTorquePid(){
        torquePid->reset(zeros(1));
    }

    inline void setSimMode(bool on){
        if(on)
            cmdMode = SIMULATION;
        else
            cmdMode = REAL;
    }

    inline void setJoint(unsigned int joint){
        if(ctrlMode != NO_CONTROL){
            throw runtime_error("It is not allowed to change joint while control loop is active.");
        }
        jointId = joint;
    }
    // GET METHODS
    inline ControlMode getCtrlMode(){ return ctrlMode; }
	inline ControlThreadStatus getThreadStatus(){ return thread_status; }
    inline Vector getKp(ControlMode cm=NO_CONTROL){ 
        if(cm==NO_CONTROL)
            cm = getCtrlMode();
        return getParameter("Kp");
    }
    inline Vector getKi(ControlMode cm=NO_CONTROL){ 
        if(cm==NO_CONTROL)
            cm = getCtrlMode();
        return getParameter("Ki");
    }
    inline Vector getKd(ControlMode cm=NO_CONTROL){ 
        if(cm==NO_CONTROL)
            cm = getCtrlMode();
        return getParameter("Kd");
    }    
    inline Vector getTao(){ return tao; }
    inline Vector getTaod(){ return taod; }
    inline double getPwmD(){ return pwmD; }
    inline float getalpha(){ return alpha; }
    inline int getJoint(){ return jointId; }

};


}

} // end namespace

#endif
