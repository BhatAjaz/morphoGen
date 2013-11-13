// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon 
 * (based on code written by Alberto Bietti, Logan Niehaus, and Gionvanni Saponaro for the autoAssociativeMemory module)
 * email:   david.vernon@robotcub.org
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

/* yarp */
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

/* iCub */
#include <iCub/attentionNetworkTest.h>

using namespace yarp::os;

int main(int argc, char *argv[])
{
    /* 
     * create a specific Network object, rather than using Network::init().
     * this way, automatic cleanup by the means of Network::fini() will be smoother
     */

    Network network;

    AttentionNetworkTest attentionNetworkTestModule;	// instantiate module
 
    /* prepare and configure the resource finder */

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("attentionNetworkTest.ini");       //overridden by --from parameter
    rf.setDefaultContext("attentionNetworkTest/conf");         //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */

    attentionNetworkTestModule.runModule(rf);

    return 0;
}