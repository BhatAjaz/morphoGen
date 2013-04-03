/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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

/** 
\defgroup pmpExample pmpExample
 
@ingroup icub_module  
 
Example module for the use of \ref pmp "Pmp Library".

\section intro_sec Description 
This simple module steers the arm to a starting pose and then 
creates one pmp-target and one pmp-obstacle; thereby it enables 
the pmp field that in turn lets the robot end-effector attain 
the target avoiding the obstacle. 
 
It requires the \ref pmpServer running. 
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref pmp "Pmp" library.

\section parameters_sec Parameters 
--robot \e robot
- select the robot to connect to.

--part \e part
- select the part to control. 

--remote \e name
- specify the pmp server name to connect to.
 
--local \e name
- specify the pmp client stem-name.
 
--verbosity \e level
- specify the verbosity level of the pmp client print-outs.
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori, Ugo Pattacini
*/ 

#include <string>
#include <stdio.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/ctrl/math.h>
#include <iCub/pmp/pmp_client.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::pmp;


/************************************************************************/
class ClientModule: public RFModule
{
protected:
    PmpClient clientR;
    bool init;
    bool closing;
    yarp::dev::PolyDriver         dCtrlR;
    yarp::dev::ICartesianControl *iCtrlR;
    int store_context_idR;
    int targetR;
    int obstacleR;
    Vector xR;
    Vector oR;
    double distR;
    string armR;

    PmpClient clientL;
    yarp::dev::PolyDriver         dCtrlL;
    yarp::dev::ICartesianControl *iCtrlL;
    int store_context_idL;
    int targetL;
    int obstacleL;
    Vector xL;
    Vector oL;
    double distL;
    string armL;

public:

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        int verbosity = rf.check("verbosity",Value(0)).asInt();
        string remote = "/right_pmp_server";
        string local  = "/right_pmp_client";
        string robot  = rf.check("robot",Value("icubSim")).asString().c_str();
        string part   = "right_arm";
        armR          = "right";

        Property options;
        options.put("verbosity",verbosity);
        options.put("remote",remote.c_str());
        options.put("local",local.c_str());

        Property optCtrl;
        optCtrl.put("device","cartesiancontrollerclient");
        optCtrl.put("remote",("/"+robot+"/cartesianController/"+part).c_str());
        optCtrl.put("local",(local+"/cartesian").c_str());

        init=true;
        closing=false;

        if (dCtrlR.open(optCtrl))
            dCtrlR.view(iCtrlR);
        else
            return false;

        iCtrlR->storeContext(&store_context_idR);

        Vector dof;
        iCtrlR->getDOF(dof);
        Vector newDof=dof;
        newDof[0]=1.0;
        newDof[2]=1.0;
        
        iCtrlR->setDOF(newDof,dof);
        iCtrlR->setLimits(7,-70.0,70.0);

        int res = clientR.open(options);

        remote = "/left_pmp_server";
        local  = "/left_pmp_client";
        part   = "left_arm";
        armR   = "left";

        options.put("remote",remote.c_str());
        options.put("local",local.c_str());

        optCtrl.put("remote",("/"+robot+"/cartesianController/"+part).c_str());
        optCtrl.put("local",(local+"/cartesian").c_str());

        if (dCtrlL.open(optCtrl))
            dCtrlL.view(iCtrlL);
        else
            return false;

        iCtrlL->storeContext(&store_context_idL);

        iCtrlL->getDOF(dof);
        newDof=dof;
        newDof[0]=1.0;
        newDof[2]=1.0;
        
        iCtrlL->setDOF(newDof,dof);
        iCtrlL->setLimits(7,-70.0,70.0);

        res = res && clientL.open(options);

        // get the torso dofs
        Vector newDof_l, curDof_l;
        Vector newDof_r, curDof_r;
        iCtrlL  -> getDOF(curDof_l);
        iCtrlR -> getDOF(curDof_r);
        
        newDof_l=curDof_l;
        newDof_r=curDof_r;

        newDof_l[0] = 0;  newDof_l[1] = 0;  newDof_l[2] = 0;
        newDof_r[0] = 0;  newDof_r[1] = 0;  newDof_r[2] = 0;

        iCtrlL -> setDOF(newDof_l,curDof_l);
        iCtrlR -> setDOF(newDof_r,curDof_r);
        
        return res;
    }

    /************************************************************************/
    bool close()
    {
        if (!closing) 
        {
            clientR.disableControl();                
            clientR.disableField();
            
            iCtrlR->restoreContext(store_context_idR);
            dCtrlR.close();

            clientR.clearItems();

            clientL.disableControl();                
            clientL.disableField();
            
            iCtrlL->restoreContext(store_context_idL);
            dCtrlL.close();

            clientL.clearItems();
        }

        clientR.close();
        clientL.close();
        return true;
    }

    /************************************************************************/
    bool updateModule()
    {
        if (init)
        {
            Vector xdR(3,0.0), xdL(3,0.0);
            Vector odR(4,0.0), odL(4,0.0);
            xdR[0] = -0.3157; xdR[1] =  0.1; xdR[2] =  0.0422;
            odR[0] = 0.0; odR[1] = 1.0; odR[2] = 0.0; odR[3] = M_PI;
            xdL[0] = -0.3157; xdL[1] = -0.1; xdL[2] =  0.0422;
            odL[0] = 0.0; odL[1] = 0.0; odL[2] = 1.0; odL[3] = M_PI;

            iCtrlR->goToPoseSync(xdR,odR,2.0);
            iCtrlR->waitMotionDone();
            iCtrlR->getPose(xR,oR);
            iCtrlR->setTrajTime(1.0);
            iCtrlR->setInTargetTol(1e-3);

            clientR.setActiveIF(armR);
            clientR.setPointStateToTool();
            clientR.enableControl();
            clientR.enableField();

            iCtrlL->goToPoseSync(xdL,odL,2.0);
            iCtrlL->waitMotionDone();
            iCtrlL->getPose(xL,oL);
            iCtrlL->setTrajTime(1.0);
            iCtrlL->setInTargetTol(1e-3);

            clientL.setActiveIF(armL);
            clientL.setPointStateToTool();
            clientL.enableControl();
            clientL.enableField();  

            init=false;
            return true;
        }
        else
        {
            Time::delay(20);
        }
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.1;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    ClientModule mod;
    return mod.runModule(rf);
}



