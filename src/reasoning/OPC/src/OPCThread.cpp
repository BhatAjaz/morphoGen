
#include <OPCThread.h>
#include <cstring>
#include <string>
#include<time.h>
#include <math.h>
#include <iostream>
#include <fstream>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#include "MessageFormats/DarwinMessages.h"
using namespace darwin::msg;

OPCThread::OPCThread() {
    robot = "icub";        
}

OPCThread::OPCThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

OPCThread::~OPCThread() {
    // do nothing
}

bool OPCThread::threadInit() {
   	   
		if (!OPCServer.open(getName("/OPCServer:io").c_str())) {
			cout << ": unable to open port to send unmasked events "  << endl;
			return false;  // unable to open; let RFModule know so that it won't run
		}  

    	if (!WorldSnap.open(getName("/input/snapshot:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
       }  
		if (!SendScene.open(getName("/output/scene:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
       }
		//OPC server listens to the observer and posts snapshots when triggered
   
	 return true;
}

void OPCThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string OPCThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void OPCThread::setInputPortName(string InpPort) {
    
}

void OPCThread::run()
{   
	 while (isStopping() != true) {

        if (OPCServer.getInputCount()) {        
			Bottle OPCReq, OPCResp;
         
			OPCServer.read(OPCReq,true);
			
			if(!OPCReq.isNull()) {
				printf("%s \n",OPCReq.toString().c_str());
				//cout<<"OPCReq!=NULL"<<endl;
				// request present
				//reading
				int cmd = OPCReq.get(0).asVocab();
				cout<< "Received microgoal FIND from Client:Observer" <<cmd<<endl;
				Network::connect("/vision/objects:o", "/input/snapshot:i"); // this is now a connection to the new CVUT+FORTH vision system
				Network::connect("/output/scene:o", "/observer/VisionScene:i");
                //===========================================================================
        for(int i=0;i<10;i++)
					{
						for(int j=0;j<18;j++)
							{
							  Eighteens[i][j]=0;
							}
					  }
				 for(int i=0;i<10;i++)
					{
						IOD[i]=0;
					    Graspability[i]=0;
					}
				  NumObject=0;
				 int ctrr=0;
				 //===========================================================================
					if (WorldSnap.getInputCount())
							{            
							  					
								//	  Bottle* ObjIdd = WorldSnap.read(true);
								//	  NumObject = (int) ObjIdd->get(ctrr).asDouble();
								//	  ctrr=ctrr+1;
								//	  for(int i=0;i<NumObject;i++)
								//			 {
								//			  IOD[i]=(int) ObjIdd->get(ctrr).asDouble();
								//			  ctrr=ctrr+1; 
								//			 }
								//		for(int i=0;i<NumObject;i++)
								//			{
								//				 Graspability[i]=(int) ObjIdd->get(ctrr).asDouble();
								//				 ctrr=ctrr+1;
								//		    }
        //                     for(int i=0;i<NumObject;i++)
								//			{
								//				 /*if(Graspability[i]==0)
								//								  {
       	//															  for(int j=0;j<7;j++)
								//											{
								//												Eighteens[i][j]=0;
								//						   
								//											}	
								//									  }*/

								//				  if(Graspability[i]==1)
								//									{
       	//															  for(int j=0;j<7;j++)
								//											{
								//												Eighteens[i][j]=ObjIdd->get(ctrr).asDouble();
								//												ctrr=ctrr+1; 
								//											}	
								//									  }
							 //}
							
								VisualScene *scene;
								scene=WorldSnap.read();
								Time::delay(0.1);
								VisualScene &sendingscene = SendScene.prepare();

								sendingscene = *scene;
								/*std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;
								std::cout<<sendingscene.toString()<<std::endl;
								std::cout<<"::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"<<std::endl;*/
								SendScene.write(false);


								std::cout<<"contents of the scene:  "<<scene->toString()<<std::endl;
								std::cout << "Got " << scene->size() << " objects" << std::endl; 
								NumObject=(int) scene->size();
								for(int i=0; i<scene->size(); ++i){

								     VisualObject& obj = (*scene)[i]; 
								     //std::cout << obj.toString() << std::endl; 
									 IOD[i]=obj.ID();
									 }

								for(int i=0; i<scene->size(); ++i){
									ctrr=0;
									VisualObject& obj = (*scene)[i]; 
								    //std::cout << obj.toString() << std::endl; 
									if(IOD[i]==101){
										int nInsert=obj.InsertTip().size();
										Graspability[i]=nInsert;
										for(int j=0; j<nInsert; j++)
											{
											Eighteens[i][ctrr]=obj.InsertTip()[j].x();
											Eighteens[i][ctrr+1]=obj.InsertTip()[j].y();
											Eighteens[i][ctrr+2]=obj.InsertTip()[j].z();
											ctrr=ctrr+3;
											}
									}
									if(IOD[i]==100){
										//int nInsert=0;
										Graspability[i]=1;
										
											Eighteens[i][ctrr]=obj.Hole()[0].Center().x();
											Eighteens[i][ctrr+1]=obj.Hole()[0].Center().y();
											Eighteens[i][ctrr+2]=obj.Hole()[0].Center().z();
											Eighteens[i][ctrr+3]=obj.Hole()[1].Center().x();
											Eighteens[i][ctrr+4]=obj.Hole()[1].Center().y();
											Eighteens[i][ctrr+5]=obj.Hole()[1].Center().z();
											Eighteens[i][ctrr+6]=obj.Hole()[2].Center().x();
											Eighteens[i][ctrr+7]=obj.Hole()[2].Center().y();
											Eighteens[i][ctrr+8]=obj.Hole()[2].Center().z();
											ctrr=ctrr+9;
											
									}

								 }

      						}

			 //============================================================================
			  cout << "Sending out Visual Snapshot to Client Observer" << endl;

     		  OPCResp.addInt(428);
			  OPCResp.addInt(NumObject);
				
								for(int i=0;i<NumObject;i++)
									{
									  OPCResp.addInt(IOD[i]);
									}
								for(int i=0;i<NumObject;i++)
									{
									  OPCResp.addDouble(Graspability[i]);
									}
								for(int i=0;i<NumObject;i++)
									{
                                       for(int j=0;j<Graspability[i]*9;j++)
										   { 
									             OPCResp.addDouble(Eighteens[i][j]);
									       }
								}
								
				
				OPCServer.reply(OPCResp);
				//Time::delay(3);
			}
			else {
				cout<<"null request"<<endl;
			}
//========================================================================================  
		}

		//Time::delay(5);          	     
	} 
}

void OPCThread::threadRelease() {
    // nothing
	//EpimCtrlPort.close();
     
}


void OPCThread::onStop() {
    
//    outputPort.interrupt();
  //  outputPort.close();
}


