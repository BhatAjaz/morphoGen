/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <map>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/PortReport.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

#include "iCub/DMP/DMPstructure.h"

class Manager;  //forward declaration

class ObjectPropertiesCollectorPort: public yarp::os::RpcClient, public yarp::os::PortReport
{
private:
    bool scheduleUpdate;
    std::map<std::string,int> memoryIds;
    bool checkConnection();

public:
    ObjectPropertiesCollectorPort()
        :scheduleUpdate(false)
    {
        this->setReporter(*this);
    }

    void report(const yarp::os::PortInfo &info)
    {
        if(info.created && !info.incoming)
            scheduleUpdate=true;
    }

    bool addActionTarget(int id, yarp::sig::Vector targetPosition);
    int  createActionTarget(std::string actionName, yarp::sig::Vector targetPosition);
    bool get2DPositionFromMemory(const std::string &object, yarp::sig::Vector &position);
    DMPstructure get_information_for(int32_t id);

    bool isUpdateNeeded()
    {
        if(scheduleUpdate)
        {
            scheduleUpdate=false;
            return true;
        }

        return false;
    }
};

#endif
