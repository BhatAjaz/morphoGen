/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 * email:  elena.ceseracciu@iit.it
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

#include <yarp/os/Time.h>
#include <list>

#include "iCub/utils.h"
#include "iCub/module.h"

using namespace std;
using namespace yarp::os;

/**********************************************************/
bool ObjectPropertiesCollectorPort::checkConnection()
{
    return (this->getOutputCount()>0 ? true : false);
}
/**********************************************************/
std::vector<int32_t> ObjectPropertiesCollectorPort::get_all_actions_with_traj_endeff()
{
    std::vector<int32_t> result(0);
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('a','s','k'));
    Bottle &conditions=cmdOpc.addList();
    Bottle &entity= conditions.addList();
    entity.addString("entity");
    entity.addString("==");
    entity.addString("action");
    conditions.addString("&&");
    Bottle &trajectory= conditions.addList();
    trajectory.addString("traj_endeff");
    Bottle bReply;
    if (!checkConnection())
         return result;
    this->write(cmdOpc, bReply);
    // [ack] ("id" (<num0> <num1> ...)) 
    if (bReply.size()<1 || bReply.get(0).asVocab()==Vocab::encode("nack") || !bReply.get(1).check("id"))
        return result;
    Bottle* listIds=bReply.get(1).asList()->find("id").asList();
    result.reserve(listIds->size());
    for (int idCt=0; idCt<listIds->size(); ++idCt)
    {
       result.push_back(listIds->get(idCt).asInt());   
    }
    return result;
}

/**********************************************************/
bool ObjectPropertiesCollectorPort::addActionDMP(int32_t id, DMPstructure &dmp)
{
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('s','e','t'));
    Bottle& newProp=cmdOpc.addList();
    Bottle& dmpId=newProp.addList();
    dmpId.addString("id");
    dmpId.addInt(id);
    Bottle& bDmpElement=newProp.addList();
    bDmpElement.addString("DMP");
    Bottle& bDmp=bDmpElement.addList();
    Bottle& N=bDmp.addList();
    N.addString("N");
    N.addInt(dmp.get_num_basis());
    Bottle& dof=bDmp.addList();
    dof.addString("dof");
    dof.addInt(dmp.get_dof());
    Bottle& alpha_x=bDmp.addList();
    alpha_x.addString("alphax");
    alpha_x.addInt(dmp.get_alphax());
    Bottle& alpha_z=bDmp.addList();
    alpha_z.addString("alphaz");
    alpha_z.addInt(dmp.get_alphaz());
    Bottle& beta_z=bDmp.addList();
    beta_z.addString("betaz");
    beta_z.addInt(dmp.get_betaz());
    Bottle& tau=bDmp.addList();
    tau.addString("tau");
    tau.addDouble(dmp.get_tau());
    Bottle& weights=bDmp.addList();
    weights.addString("weights");
    Bottle & weightRows=weights.addList();
    for (int i1=0; i1<dmp.get_dof(); ++i1)
    {
        yarp::sig::Vector weightsVector=dmp.get_weights(i1);
        Bottle& weightsColumn=weightRows.addList();
        for (int i2=0; i2<weightsVector.length(); ++i2)
        {
            weightsColumn.addDouble(weightsVector(i2));
        }
    }
    Bottle& goal=bDmp.addList();
    goal.addString("goal");
    Bottle& goalData=goal.addList();
    yarp::sig::Vector goalVector=dmp.get_goal();
    for(int i1=0; i1<goalVector.length(); ++i1)
        goalData.addDouble(goalVector(i1));
        
    Bottle bReply;
    if (!checkConnection())
        return false;
    this->write(cmdOpc, bReply);
    return bReply.size()!=0 && bReply.get(0).asVocab()==Vocab::encode("ack");
}
/**********************************************************/
bool ObjectPropertiesCollectorPort::get_information_for(int32_t id)
{
    if (!learnerModule)
        return false;
    learnerModule-> resetCurrent();
    
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('g','e','t'));
    Bottle &trajId=cmdOpc.addList();
    trajId.addString("id");
    trajId.addInt(id);
    Bottle bReply;
    if (!checkConnection())
        return false;
    this->write(cmdOpc, bReply);
    if (bReply.size()<2 || bReply.get(0).asVocab()==Vocab::encode("nack") || !bReply.get(1).check("entity") || bReply.get(1).find("entity").asString()!="action")
         return false;
    learnerModule->currentMutex.wait();
    bool foundSmt=false;
    Bottle* trajectory=bReply.get(1).find("traj_endeff").asList();
    if (trajectory!=NULL)
    {
        int noSamples=trajectory->size();
        if (noSamples>0)
        {
            foundSmt=true;
            Bottle* sample = trajectory->get(0).asList();
            int dof=sample->size()-1;    //last element is the timestamp
            learnerModule->dof=dof;            
            yarp::sig::Matrix& trajectoryMatrix=learnerModule->currentTrajectory;
            trajectoryMatrix.resize(noSamples, dof+1);
            for (int ct=0; ct<noSamples; ++ct)     
            {
                sample= trajectory->get(ct).asList();
                for(int ctDof=0; ctDof<dof; ++ctDof)
                {  
                    trajectoryMatrix[ct][ctDof+1]=sample->get(ctDof).asDouble(); // fist column is for time
                }
                    
                trajectoryMatrix[ct][0]=sample->get(dof).asDouble();          
            }
        }
     }
     
     Bottle *targetPosition=bReply.get(1).find("targetPosition").asList();
     if(targetPosition)
     {
        foundSmt=true;
        yarp::sig::Vector& targetVector=learnerModule->currentTarget;
        targetVector.resize(targetPosition->size());
        for (int i=0; i<targetPosition->size(); ++i)
        {
          targetVector(i)=targetPosition->get(i).asDouble();
        }
     }
     if (bReply.get(1).check("name"))
         learnerModule->actionName=bReply.get(1).find("name").asString().c_str();
     
     Bottle* dmp =bReply.get(1).find("DMP").asList();
     if(dmp)
     {
         foundSmt=true;
         int N=dmp->find("N").asInt();
         int dof=dmp->find("dof").asInt();
         double alpha_x=dmp->find("alphax").asDouble();
         double alpha_z=dmp->find("alphaz").asDouble();
         double beta_z=dmp->find("betaz").asDouble();
         learnerModule->currentDMP=DMPstructure(dof,N,alpha_x,alpha_z,beta_z);
         DMPstructure &learnerDmp=learnerModule->currentDMP;
         double tau=dmp->find("tau").asDouble();
         learnerDmp.set_tau(tau);
         Bottle* weights=dmp->find("weights").asList(); //they are serialized row by row
         if (weights)
         {
             for (int i1=0; i1<weights->size(); ++i1)
             {
                 Bottle* weightRow=weights->get(i1).asList();
                 if (weightRow)
                 {
                     yarp::sig::Vector weightRowVector(weightRow->size());
                     for (int i2=0; i2<weightRow->size(); ++i2)
                         weightRowVector(i2)=weightRow->get(i2).asDouble();
                     learnerDmp.set_weights(i1, weightRowVector);
                 }
            }
         }

         Bottle* goal=dmp->find("goal").asList();
         if(goal)
         {
             yarp::sig::Vector goalVector(goal->size());
             for(int i1=0; i1<goal->size(); ++i1)
                 goalVector(i1)=goal->get(i1).asDouble();
             learnerDmp.set_goal(goalVector);
         }
     }
     
     learnerModule->currentMutex.post();
     return true;     
}