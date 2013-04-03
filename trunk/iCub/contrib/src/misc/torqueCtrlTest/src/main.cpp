/**
@ingroup icub_module
\defgroup torqueCtrlTest torqueCtrlTest
Test different kind of low level torque control using the open loop control interface to set the motor PWM.
Copyright (C) 2008 RobotCub Consortium
Author: Andrea Del Prete
Date: first release 08/2011
CopyPolicy: Released under the terms of the GNU GPL v2.0.

\author Andrea Del Prete
*/ 

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/common.h>

#include <stdexcept>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "iCub/torqueCtrlTest/controlConstants.h"
#include "iCub/torqueCtrlTest/controlThread.h"

YARP_DECLARE_DEVICES(icubmod)


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

namespace iCub
{

namespace torqueCtrlTest
{

class controlModule: public RFModule
{
private:
    
	int period;
	controlThread *contrThread;	
    BodyPart bodyPart;    
    Port rpcPort;        	

public:
    controlModule()
    {
		bodyPart            = UNKNOWN_BODY_PART;
        contrThread         = 0;
        period              = 10;
    }
   
    bool configure(ResourceFinder &rf)
    {		
		string fwdSlash = "/";

        //-----------------GET THE MODULE NAME-------------------//
        string name = "torqueCtrlTest";
        if (rf.check("name"))
            name = rf.find("name").asString().c_str();
        setName(name.c_str());
        
        //-----------------GET THE PERIOD-------------------//
        int period = 10;
        if (rf.check("period"))
            period = rf.find("period").asInt();

		//-----------------GET THE ROBOT NAME-------------------//
		string robot_name = "icub";
		if (rf.check("robot"))
            robot_name = rf.find("robot").asString().c_str();

		//------------------CHECK WHICH ARM IS ENABLED-----------//
		if (rf.check("left_arm")){
			bodyPart = LEFT_ARM;
			fprintf(stderr,"'left_arm' option found. Left arm will be enabled.\n");
		}
        else if (rf.check("right_arm")){
			bodyPart = RIGHT_ARM;
			fprintf(stderr,"'right_arm' option found. Right arm will be enabled.\n");
		}
        else{
            bodyPart = LEFT_ARM;
            fprintf(stderr, "No arm specified. Left arm will be used by default.\n");
        }

        //---------------------RPC PORT--------------------------//
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);


        //--------------------------THREAD--------------------------
        contrThread = new controlThread(name, robot_name, period, bodyPart, VERBOSE);
        fprintf(stderr,"control thread istantiated...\n");
        contrThread->start();
        fprintf(stderr,"control thread started\n");
        return true;
    }


    bool respond(const Bottle& command, Bottle& reply) 
    {
	    stringstream temp;
	    string helpMessage =  string(getName().c_str()) + " commands are: ";
	    reply.clear();

	    TorqueCtrlTestCommand com;
        Bottle param;
	    if(!identifyCommand(command, com, param)){
		    reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		    return true;
	    }

	    switch( com ){
		    case quit:          reply.addString("quitting");    return false;
		    case help:
                reply.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
                reply.addString(helpMessage.c_str());
			    for(unsigned int i=0; i< SFC_COMMAND_COUNT; i++){
				    reply.addString( ("- "+TorqueCtrlTestCommand_s[i]+": "+TorqueCtrlTestCommand_desc[i]).c_str() );
			    }
			    return true;

            case no_ctrl:       contrThread->setCtrlMode(NO_CONTROL);   break;
            case pid_ctrl:    contrThread->setCtrlMode(PID_CTRL);   break;
            case open_ctrl:   contrThread->setCtrlMode(OPEN_CTRL);   break;

            case get_kp:
                {
                    Vector kp = contrThread->getKp();
                    reply.addString(kp.toString(3).c_str());
                    return true;
                }
            case get_kd:
                {
                    Vector kd = contrThread->getKd();
                    reply.addString(kd.toString(3).c_str());
                    return true;
                }
            case get_ki:
                {
                    Vector ki = contrThread->getKi();
                    reply.addString(ki.toString(3).c_str());
                    return true;
                }           
            case get_taod:
                {
                    Vector taod = contrThread->getTaod();
                    reply.addString(taod.toString(3).c_str());
                    return true;
                }
            case get_tao:
                {
                    Vector tao = contrThread->getTao();
                    reply.addString(tao.toString(3).c_str());
                    return true;
                }           
            case get_alpha:
                {
                    float alpha = contrThread->getalpha();
                    reply.addDouble(alpha);
                    return true;
                }
            case get_joint:
                {
                    reply.addInt(contrThread->getJoint());
                    return true;
                }
            case get_pwm:
                {
                    reply.addInt(contrThread->getPwmD());
                    return true;
                }

            case set_kp:
                {
                    try{
                        if(param.size()>1){
                            Vector kp = bottle2vector(param);
                            contrThread->setKp(kp);
                        }else{
                            contrThread->setKp(param.get(0).asDouble());
                        }
                        reply.addString("kp set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kp failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_kd:
                {
                    try{
                        if(param.size()>1){
                            Vector kd = bottle2vector(param);
                            contrThread->setKd(kd);
                        }else{
                            contrThread->setKd(param.get(0).asDouble());
                        }
                        reply.addString("kd set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set kd failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }
            case set_ki:
                {
                    try{
                        if(param.size()>1){
                            Vector ki = bottle2vector(param);
                            contrThread->setKi(ki);
                        }else{
                            contrThread->setKi(param.get(0).asDouble());
                        }
                        reply.addString("ki set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set ki failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }            
            case set_taod:
                {
                    try{
                        if(param.size()>1){
                            contrThread->setTaod(bottle2vector(param));
                        }else{
                            contrThread->setTaod(param.get(0).asDouble());
                        }
                        reply.addString("taod set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set taod failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }           
            case set_alpha:
                {
                    try{
                        float alpha = (float)(bottle2vector(param))[0];
                        contrThread->setAlpha(alpha);
                        reply.addString("alpha set successfully.");
                    }catch(runtime_error &e){
                        reply.addString("set alpha failed: ");
                        reply.addString(e.what());
                    }
                    return true;
                }    
            case set_joint:
                try{
                    contrThread->setJoint(param.get(0).asInt());
                    reply.addString("joint set successfully.");
                }catch(runtime_error &e){
                    reply.addString("set joint failed: ");
                    reply.addString(e.what());
                }
                return true;
            case set_pwm:
                try{
                    contrThread->setPwmD(param.get(0).asDouble());
                    reply.addString("pwm set successfully.");
                }catch(runtime_error &e){
                    reply.addString("set pwm failed: ");
                    reply.addString(e.what());
                }
                return true;
            case reset_pid:
                {
                    contrThread->resetTorquePid();
                    break;
                }

            case sim_on:    contrThread->setSimMode(true);  break;
            case sim_off:   contrThread->setSimMode(false); break;        
		    default: reply.addString("ERROR: This command is known but it is not managed in the code."); return true;
	    }

	    reply.addString( (TorqueCtrlTestCommand_s[com]+" command received.").c_str());
	    return true;	
    }


    /**
      * Identify the command in the bottle and return the correspondent enum value.
      * All the elements of the Bottle that are after the identified command are inserted 
      * in the param bottle.
      */
    bool identifyCommand(const Bottle &commandBot, TorqueCtrlTestCommand &com, Bottle &param){
	    for(unsigned int i=0; i<SFC_COMMAND_COUNT; i++){
		    stringstream stream(TorqueCtrlTestCommand_s[i]);
		    string word;
		    int wordCounter=0;
		    bool found = true;

		    while(stream>>word){
			    if (commandBot.get(wordCounter).asString() != word.c_str()){
				    found=false;
				    break;
			    }
			    wordCounter++;
		    }
		    if(found){
			    com = (TorqueCtrlTestCommand)i;
                for(int k=wordCounter; k<commandBot.size(); k++)
                    param.add(commandBot.get(k));
			    return true;
		    }
	    }

	    return false;
    }


    bool close(){
		//stop thread 
		if(contrThread){
            contrThread->stop();
            delete contrThread;
            contrThread = 0;
        }
				
		//closing ports
        rpcPort.interrupt();
		rpcPort.close();

        return true;
    }

    double getPeriod()  { return 0.3;  }
    bool updateModule()
	{
        if (contrThread==0) 
            return false;

        double avgTime, stdDev, period;
        period = contrThread->getRate();
        //contrThread->getEstUsed(avgTime, stdDev);     // real duration of run()
        contrThread->getEstPeriod(avgTime, stdDev);
        if(avgTime > 1.3 * period){
            printf("WARNING: Control loop is too slow (real period: %3.3f+/-%3.3f; expected period %3.3f)\n", avgTime, stdDev, period);
        }
        
        ControlThreadStatus thread_status = contrThread->getThreadStatus();

		if (thread_status==STATUS_OK)
            return true;
		else if (thread_status==STATUS_DISCONNECTED){
			printf ("torqueCtrlTest module lost connection with iCubInterface or wholeBodyDynamics, now closing...\n");
			return false;
		}else{
			fprintf(stderr,"torqueCtrlTest module was closed successfully! \n");
			return true;
		}        
	}   
    Vector bottle2vector(const Bottle &b) {
        Vector v(b.size());
        for(int i=0; i<b.size();i++){
            if(!b.get(i).isInt() && !b.get(i).isDouble()){
                throw runtime_error("One of the values is not a number");
            }
            v(i) = b.get(i).asDouble();
        }
        return v;
    }
};

}
} // end namespace

int main(int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app: default wrechObserver/conf)" << endl;
        cout << "\t--from       from: the name of the file.ini to be used for calibration"                                        << endl;
        cout << "\t--name       name: the name of the module used for the port names. default torqueCtrlTest"	                  << endl;
        cout << "\t--robot      robot: the name of the robot. default icub"	                                					  << endl;
        cout << "\t--period     period: the period used by the module. default 10ms (not less than 10ms)"						  << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    iCub::torqueCtrlTest::controlModule module;
    module.runModule(rf);

    //cin.get();
    return 0;
}

