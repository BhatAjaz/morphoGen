
#include <PMPThread.h>
#include <cstring>
#include <string>
#include <time.h>
#include <math.h>
#include <vector>
#include <iostream>
#include "MessageFormats/VocabDefinitions.h"
#include "MessageFormats/DarwinMessages.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

using namespace darwin::reach;

PMPThread::PMPThread() {
	robot = "icub";        
}

PMPThread::PMPThread(string _robot, string _configFile){
	robot = _robot;
	configFile = _configFile;
}

PMPThread::~PMPThread() {
	// do nothing
}

bool PMPThread::threadInit() {


	if (!Inp3D.open(getName("/input/coordinates:i").c_str())) {
		cout << ": unable to open port to send unmasked events "  << endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}   

	if (!MotCom.open(getName("/v:o").c_str())) {
		cout << ": unable to open port to send unmasked events "  << endl;
		return false;  // unable to open; let RFModule know so that it won't run
	} 
	
	if (!PMPResponse.open(getName("/PMPreply:io").c_str())) {
		cout << ": unable to open port to send unmasked events "  << endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}

	if (!Inpjoints.open(getName("/input/joints:i").c_str())) {
		cout << ": unable to open port /input/joints to send unmasked events "  << endl;
		return false;  // unable to open; let RFModule know so that it won't run
	}

	if (!activationsPort.open(getName("/activations:o").c_str())) {
        cout << ": unable to open port to send activations "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!MotComRX.open(getName("/vRX:o").c_str())) {
        cout << ": unable to open port to send activations "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!MotComTX.open(getName("/vTX:o").c_str())) {
        cout << ": unable to open port to send activations "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	Kompliance(0);
	flag_openfile = false;
	//LoadANN();

	//MotCom.open("/v");


	if (Network::exists("/pmpRX/vRX:o"))			
	{
		Network::connect("/pmpRX/vRX:o", "/robotcontroller/jointsRX:i");  //check this
	}


	if (Network::exists("/pmpRX/vTX:o"))			
	{
		Network::connect("/pmpRX/vTX:o", "/robotcontroller/jointsTX:i");  //check this
	}


	if (Network::exists("/pmpRX/v:o"))			
	{
		Network::connect("/pmpRX/v:o", "/robotcontroller/joints:i");  //check this
	}

	if (Network::exists("/pmpRX/input/joints:i"))
	{
		Network::connect("/robotcontroller/joints:o","/pmpRX/input/joints:i");
	}

	return true;
}

void PMPThread::setName(string str) {
	this->name=str;
	printf("name: %s", name.c_str());
}


void PMPThread::setRobotName(string robName) {

    this->robot = robName;
    printf("Robot name: %s", robot.c_str());
}


void PMPThread::setWeightsPath(std::string s, int i) {
    switch(i){
    case 1: {weights1Path = s;} break;
    case 2: {weights2Path = s;} break;
    case 3: {weights3Path = s;} break;
	case 4: {weights4Path = s;} break;
    case 5: {weights5Path = s;} break;
    case 6: {weights6Path = s;} break;
    }

}

void PMPThread::setBiasesPath(std::string s, int i) {
    switch(i){
    case 1: {biases1Path = s;} break;
    case 2: {biases2Path = s;} break;
    case 3: {biases3Path = s;} break; 
	case 4: {biases4Path = s;} break;
    case 5: {biases5Path = s;} break;
    case 6: {biases6Path = s;} break; 
    }

}
std::string PMPThread::getName(const char* p) {
	string str(name);
	str.append(p);
	return str;
}

void PMPThread::setInputPortName(string InpPort) {

}

void PMPThread::run() { 
	
	while (isStopping() != true) {

		if (PMPResponse.getInputCount()) {

			if (PMPResponse.read(bottleReq,true)) {
				//if read happened successfully it returns true

				printf("%s \n",bottleReq.toString().c_str());
				std::cout << bottleReq.toString() << std::endl;

				try {
					//the bottle sent should be of type PMPKINCommand
					PMPKINCommand& ObsReq = TypeSafeCast<PMPKINCommand>(bottleReq);

					switch (ObsReq.subID()) {
					case PMPKINCommand::kinID::value: 
						handleKinematicsCommand(ObsReq.kin());
						break;
					case PMPKINCommand::pmpID::value:
						handlePMPCommand(ObsReq.pmp());
						break;
					default:
						std::cout << "Unknown PMPKINCommand" << std::endl;
					}
					cout << ObsResp.toString();
					//the result is now ready to be returned to the Observer
					PMPResponse.reply(ObsResp);
					Time::delay(0.0);
				}
				catch (std::exception e) {
					std::cout << e.what();
				}

				bottleReq.clear();
			}	
		}
	}
}
 
void PMPThread::setKinematicsAccess(KinematicsAccess* pkin) {
	_kinematicsAccess = pkin;
}

void PMPThread::writeReachResult(ReachResult& r,Stamp& s) {
	--_kinematicsMovesStarted;
	ObsResp.setresult(r.result());
}

void PMPThread::handleKinematicsCommand(ReachCommand& reach) {
	_kinematicsStamp.update();
	_kinematicsMovesStarted = _kinematicsAccess->writeReachCommand(reach,_kinematicsStamp);
	while (_kinematicsMovesStarted) {
		Time::delay(0.01);
	}
}

void PMPThread::handlePMPCommand(PMPCommand& ObsReq) {
				

	//this will be replaced with a client request from Observer that is responsible for executing
	//a plan given by reasoning system, at present it is takign 3D coordiantes from FORTH module for testing purposes
	//========================================================================================

	//Obsreq will be used int eh future to get events formt eh Observer
	// commented VM 27 feb to close the loop for TX via Observer Client and not through FORTH port
	//Bottle* inPC = Inp3D.read(true);

	//double x = inPC->get(0).asDouble();
	//double y = inPC->get(1).asDouble();
	//double z = inPC->get(2).asDouble();
	//=====================================================================================================
	// request present
	//reading
	robCmd=100;
	wrioTx=1.34;
	AvoidTray=-1;
	int XmitGreen; //Change 2005
	double FuseIndicator=-1;
	 //for loading ANN
	//parse the PMPCommand bottle
	int GoalCodePMP = ObsReq.GoalCodePMP();
	cout<< "GoalCodePMP from Observer is: " <<GoalCodePMP<<endl;
	MSFlag = ObsReq.MSFlag();
	//MSFlag=0; // For testing.......
	cout<< "MENTAL SIMULATION REQUEST from Observer is: " <<MSFlag<<endl;
	Wr_Iorient = ObsReq.Wr_Iorient();
	cout<< "WRIST ORIENTATION from Observer is: " <<Wr_Iorient<<endl;	
	int TRAJTYPE = ObsReq.TRAJTYPE();
	cout<< "TRAJECTORY TYPE from Observer is: "<<TRAJTYPE<<endl;
	ResPM=2;
	int oidpmap = ObsReq.oidpmap();
	int rmPMP = ObsReq.rmPMP();
	for(int i=0; i<6; i++) 
	{
		VTGSIN[i] = ObsReq.VTGSIN()[i];
		std::cout << "Receiving micro goal from the Observer client" <<  VTGSIN[i] << endl;
		ColAvoidOld[i]=VTGSIN[i];
	}
							
	if((VTGSIN[0]<50)&&(VTGSIN[1]>330))
	{
		wrioTx=wrioTx-(0.20*(VTGSIN[1]-330)/70);
	}
		
	FuseIndicator=VTGSIN[2];

	if (GoalCodePMP==111)
	{
		Context=500;
	}
	if (GoalCodePMP==95)
	{
		Context=320;
	}
	if(GoalCodePMP==19)
	{
		if(TRAJTYPE==1)
		{
			Context=410;
		}
		else if(TRAJTYPE==0)
		{
			Context=680;
		}
	}
	if(GoalCodePMP==1)
	{
		if(FuseIndicator==44){
			Context=320;
		}
		else
		{
			Context=410;
		}
	}
				
				
	if(GoalCodePMP==122) {
		std::cout << "Paralell use of two robots" << endl; 
		// VTGSIN[2]=VTGSIN[2]+60;
		//VTGSIN[5]=VTGSIN[5]+60;   //removed this too
		Paraleelism(); 

		if(ResPM==1 && !detectCollision()) // reaches +60 in paraleell and then goes down in paralell
		{
			//sendParalell();
			//Time::delay(3);
			VTGSIN[0]=ColAvoidOld[0];
			VTGSIN[1]=ColAvoidOld[1];
			VTGSIN[2]=ColAvoidOld[2]+60;
			VTGSIN[5]=ColAvoidOld[5]+60;
			Paraleelism(); 
			sendParalell();
			Time::delay(3);
			VTGSIN[0]=ColAvoidOld[0];
			VTGSIN[1]=ColAvoidOld[1];
			VTGSIN[2]=ColAvoidOld[2];
			VTGSIN[5]=ColAvoidOld[5];
			Paraleelism(); 
			sendParalell();
			Time::delay(3);
		}
                
		if(ResPM==1 && detectCollision())
		{
			VTGSIN[0]=ColAvoidOld[0];
			VTGSIN[1]=-200; //Just added VM 2011
			VTGSIN[2]=ColAvoidOld[2]+300; 
			VTGSIN[5]=ColAvoidOld[5]+60;
			Paraleelism(); 
			bool detcol=detectCollision();
			if (!detcol) {
				sendParalell();
				Time::delay(3);
				VTGSIN[0]=ColAvoidOld[0];
				VTGSIN[1]=-200; //Just added VM 2011
				VTGSIN[2]=ColAvoidOld[2]+300; 
				VTGSIN[5]=ColAvoidOld[5];
				Paraleelism(); 
				sendParalell();
				Time::delay(3);
				ResPM=14;
			}
		}
	}	

	//placing the arm close to the destination...
	if((GoalCodePMP==1)||(GoalCodePMP==21)||(GoalCodePMP==95)||(GoalCodePMP==111)){
		ReadCurrentJoints();
		ChooseANN(rmPMP);
		std::cout << "Goal REA Target recd from Client" << endl;
		InitializeJan();

		if((GoalCodePMP==95)||(GoalCodePMP==111)) {
			//This is the offset taking into account RX Robot
			ResPM=VTGS(VTGSIN[0],VTGSIN[1],VTGSIN[2],VTGSIN[3],VTGSIN[4],VTGSIN[5],0,0,0);  
		}
		else {
			if((AvoidTray==32)||(AvoidTray==14)){			
				TrayAvoidanceON();
			}

			if(FuseIndicator==44)
			{ //Reachign the fuse in iterations
				ResPM=VTGS(VTGSIN[0],VTGSIN[1],VTGSIN[2],VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
				XmitGreen=0;
				//MSFlag=1;
				if(ResPM==1)
				{
					std::cout << "PMP Simulation converged" << endl;
					// cin >> XmitGreen;
					XmitGreen=1;
					if(MSFlag==1) {
						std::cout << "Executing Movement::Transmiting motor commands" << endl;
						VariableSpeedCtrl(robCmd); 
						MessagePassR(robCmd);
						IndReachCheck(robCmd);
					}
					//  ResPM=2;
				}
				XmitGreen=0;
			ResPM=VTGS(VTGSIN[0],VTGSIN[1],VTGSIN[2]+30,VTGSIN[3],VTGSIN[4],VTGSIN[5]+20,0,0,0);  // Changes 2005
			XmitGreen=0;
			//MSFlag=1;
			if(ResPM==1)
				{
					std::cout << "PMP Simulation converged" << endl;
					// cin >> XmitGreen;
					XmitGreen=1;
					if(MSFlag==1) {
						std::cout << "Executing Movement::Transmiting motor commands" << endl;
						VariableSpeedCtrl(robCmd); 
						MessagePassR(robCmd);
						IndReachCheck(robCmd);
					}
					//  ResPM=2;
				}
			} //FuseIndicator loop
			ResPM=VTGS(VTGSIN[0],VTGSIN[1],VTGSIN[2],VTGSIN[3],VTGSIN[4],VTGSIN[5],0,0,0);  // Changes 2005
			XmitGreen=0;
			//Changes 2005
		}
		std::cout << "ResPM" << ResPM << endl;
		XmitGreen=0;
		//MSFlag=1;
        if(ResPM==1) {
			std::cout << "PMP Simulation converged" << endl;
			// cin >> XmitGreen;
			XmitGreen=1;
			if(MSFlag==1) {
				std::cout << "Executing Movement::Transmiting motor commands" << endl;
				VariableSpeedCtrl(robCmd); 
				MessagePassR(robCmd);
				IndReachCheck(robCmd);
			}
            //  ResPM=2;
		}
		XmitGreen=0;
	}//GoalCode loop
	//Time::delay(20);
	
	if(GoalCodePMP==19){

		std::cout << "After ResPM0 " << endl;
		if (TRAJTYPE==1) //Changes2005..	Continuous motion ObstAC
		{
			double NewViaX,NewViaY,NewViaZ;
			ChooseANN(rmPMP); //choose rmPMP
			NewViaZ=VTGSIN[5]+(VTGSIN[2]-VTGSIN[5])*0.33;
			////================================Moving through a Via Point Multi Fuse Box================================
			ResPM=VTGS(VTGSIN[0],VTGSIN[1],NewViaZ,VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
			XmitGreen=0;
			if(ResPM==1)
			{
				std::cout << "PMP Simulation converged" << endl;
				// cin >> XmitGreen;
				XmitGreen=1;
				if(MSFlag==1){
					std::cout << "Executing Movement::Transmiting motor commands" << endl;
					VariableSpeedCtrl(robCmd); 
					//MessagePassR(robCmd);
					IndReachCheck(robCmd);
				}
				//  ResPM=2;
			}
			XmitGreen=0;
			//-------------------------------------
			NewViaZ=VTGSIN[5]+(VTGSIN[2]-VTGSIN[5])*0.66;
			ResPM=VTGS(VTGSIN[0],VTGSIN[1],NewViaZ,VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
			XmitGreen=0;
			if(ResPM==1)
			{
				std::cout << "PMP Simulation converged" << endl;
				// cin >> XmitGreen;
				XmitGreen=1;
				if(MSFlag==1){
					std::cout << "Executing Movement::Transmiting motor commands" << endl;
						VariableSpeedCtrl(robCmd); 
					//MessagePassR(robCmd);
					IndReachCheck(robCmd);
				}
				//  ResPM=2;
			}
			XmitGreen=0;
			//-------------------------------------
			ResPM=VTGS(VTGSIN[0],VTGSIN[1],VTGSIN[2],VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
			XmitGreen=0;
			if(ResPM==1)
			{
				std::cout << "PMP Simulation converged" << endl;
				// cin >> XmitGreen;
				XmitGreen=1;
				if(MSFlag==1){
					std::cout << "Executing Movement::Transmiting motor commands" << endl;
					VariableSpeedCtrl(robCmd); 
					//MessagePassR(robCmd);
					IndReachCheck(robCmd);
				}
				//  ResPM=2;
			}
			XmitGreen=0;
			//================================Moving through a Via Point Single Fuse================================
			//double NewViaX,NewViaY,NewViaZ;
			//NewViaX=VTGSIN[0]+(VTGSIN[3]-VTGSIN[0])*0.33;
//                  NewViaY=VTGSIN[1]+(VTGSIN[4]-VTGSIN[1])*0.33;
			//NewViaZ=VTGSIN[2]+(VTGSIN[5]-VTGSIN[2])*0.33;
			// 	 ResPM=VTGS(NewViaX,NewViaY,NewViaZ,VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
			// XmitGreen=0;
			// if(ResPM==1)
			//	   {
			//		   std::cout << "PMP Simulation converged" << endl;
			//		  // cin >> XmitGreen;
			//		   XmitGreen=1;
			//		   if(MSFlag==1){
			//			std::cout << "Executing Movement::Transmiting motor commands" << endl;
			//		   MessagePassR(robCmd);
			//			ReachCheck();
			//		   }
			//		 //  ResPM=2;
			//	   }
		//		XmitGreen=0;
			////-------------------------------------
			//NewViaX=VTGSIN[0]+(VTGSIN[3]-VTGSIN[0])*0.66;
//                  NewViaY=VTGSIN[1]+(VTGSIN[4]-VTGSIN[1])*0.66;
			//NewViaZ=VTGSIN[2]+(VTGSIN[5]-VTGSIN[2])*0.66;
			//ResPM=VTGS(NewViaX,NewViaY,NewViaZ,VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
			// XmitGreen=0;
			// if(ResPM==1)
			//	   {
			//		   std::cout << "PMP Simulation converged" << endl;
			//		  // cin >> XmitGreen;
			//		   XmitGreen=1;
			//		   if(MSFlag==1){
			//			std::cout << "Executing Movement::Transmiting motor commands" << endl;
			//		   MessagePassR(robCmd);
			//			ReachCheck();
			//		   }
			//		 //  ResPM=2;
			//	   }
		//		XmitGreen=0;
			//	//-------------------------------------
			//	ResPM=VTGS(VTGSIN[3],VTGSIN[4],VTGSIN[5],VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
			// XmitGreen=0;
			// if(ResPM==1)
			//	   {
			//		   std::cout << "PMP Simulation converged" << endl;
			//		  // cin >> XmitGreen;
			//		   XmitGreen=1;
			//		   if(MSFlag==1){
			//			std::cout << "Executing Movement::Transmiting motor commands" << endl;
			//		   MessagePassR(robCmd);
			//			ReachCheck();
			//		   }
			//		 //  ResPM=2;
			//	   }
		//		XmitGreen=0;
		}
		if (TRAJTYPE==0){
			//commented OCT 2014 VM...........................................;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	//               if((X_posIniObRx[1]>-260)&&(X_posIniObRx[1]<=-90))
			//{
	//                  ResPM=VTGS(X_posIniObRx[0],X_posIniObRx[1],250,VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
			//   XmitGreen=0;
			//		 if(ResPM==1)
			//			   {
			//				   std::cout << "PMP Simulation converged" << endl;
			//				  // cin >> XmitGreen;
			//				   XmitGreen=1;
			//				   if(MSFlag==1){
			//					std::cout << "Executing Movement::Transmiting motor commands" << endl;
			//					VariableSpeedCtrl(robCmd); 
			//					MessagePassR(robCmd);
			//					ReachCheck();
			//				   }
			//			  }
			//}
					//commented OCT 2014 VM...........................................;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
			if(MSFlag==1){
				if(rmPMP==-1)
				{
					initRX();
					std::cout << "After INITRX " << endl;
					VariableSpeedCtrl(32); 
					MessagePassR(32);
					IndReachCheck(32);
				}
				if(rmPMP==1){
					initTX();
					std::cout << "After INITTX " << endl;
					VariableSpeedCtrl(14); 
					MessagePassR(14);
					IndReachCheck(14);
					//Time::delay(4);
					std::cout << "After MessagePassR2 " << endl;
				}
				//	ReachCheck();
				std::cout << "After Reach Check 2" << endl;
			}
		}
		ResPM=1;
	}

	std::cout << "Sending out Result of requested Primitive behaviour to Client Observer" << endl;


	ObsResp.setflag(221.0);
	ObsResp.setresult(ResPM);
	ObsResp.location().setx(X_pos[0]).sety(X_pos[1]).setz(X_pos[2]);
	ObsResp.joints().clear();
	ObsResp.joints().add(ang1);
	ObsResp.joints().add(ang2);
	ObsResp.joints().add(ang3);
	ObsResp.joints().add(ang4);
	ObsResp.joints().add(ang5);
	ObsResp.joints().add(ang6);
}

void PMPThread::Paraleelism(){

	              NsetJ=0;
				  AvoidTray=-1;
				  ReadCurrentJoints();
				 MSFlag=0; // temporarilly set MSFlag as 0..just to capture sets of joint angles
				 for(int i=0;i<2;i++){
					 if(i==0){
						 ChooseANN(-1);
						 //VTGSIN[2]=VTGSIN[2];  Removed offset RX Paralell was 55
					}
					  if(i==1){
				 ChooseANN(1);
				 VTGSIN[0]=VTGSIN[3];//make this proper
				 VTGSIN[1]=VTGSIN[4];
				 VTGSIN[2]=VTGSIN[5];
					 }
				 InitializeJan();
				 if(((AvoidTray==32)||(AvoidTray==14))&&(i==0)){
							
								TrayAvoidanceON();
							}
				  ResPM=VTGS(VTGSIN[0],VTGSIN[1],VTGSIN[2],VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
							int XmitGreen=0;
							 if(robCmd==32)
								  {
								    JStoreRx[NsetJ][0]=ang1;
									JStoreRx[NsetJ][1]=ang2;
									JStoreRx[NsetJ][2]=ang3;
									JStoreRx[NsetJ][3]=ang4;
									JStoreRx[NsetJ][4]=ang5;
									JStoreRx[NsetJ][5]=ang6;
									
									std::cout<< "Joints of RX :"<< JStoreRx[NsetJ][0]<<" , " <<JStoreRx[NsetJ][1]<<" , " <<JStoreRx[NsetJ][2]<<" , "
										<<JStoreRx[NsetJ][3]<<" , " <<JStoreRx[NsetJ][4]<<" , " <<JStoreRx[NsetJ][5]<<endl;
									NsetJ=NsetJ+1;

								  }
							 if(robCmd==14)
								  {
								    JStoreTx[0][0]=ang1;
									JStoreTx[0][1]=ang2;
									JStoreTx[0][2]=ang3;
									JStoreTx[0][3]=ang4;
									JStoreTx[0][4]=ang5;
									JStoreTx[0][5]=ang6;

									std::cout<< "Joints of TX :"<< JStoreTx[0][0]<<" , " <<JStoreTx[0][1]<<" , " <<JStoreTx[0][2]<<" , "
										<<JStoreTx[0][3]<<" , " <<JStoreTx[0][4]<<" , " <<JStoreTx[0][5]<<endl;
									//NsetJ=NsetJ+1;
								  }
				
				 }		

}

void  PMPThread::sendParalell()
{
for(int i=0;i<NsetJ;i++){
										   ang1=JStoreRx[i][0];
										   ang2=JStoreRx[i][1];
										   ang3=JStoreRx[i][2];
										   ang4=JStoreRx[i][3];
										   ang5=JStoreRx[i][4];
										   ang6=JStoreRx[i][5];
									   cout << "PMP Simulation converged" << endl;
									   MSFlag=1;
									  int XmitGreen=1;
									   if(MSFlag==1){
										cout << "Executing Movement::Transmiting motor commands" << endl;
										VariableSpeedCtrl(32); 
								
									   }

									   }
										   ang1=JStoreTx[0][0];
										   ang2=JStoreTx[0][1];
										   ang3=JStoreTx[0][2];
										   ang4=JStoreTx[0][3];
										   ang5=JStoreTx[0][4];
										   ang6=JStoreTx[0][5];
										   cout << "PMP Simulation converged" << endl;
									   MSFlag=1;
									  // cin >> XmitGreen;
									   int XmitGreen=1;
									   if(MSFlag==1){
										cout << "Executing Movement::Transmiting motor commands" << endl;
										VariableSpeedCtrl(14); 
										//MessagePassR(robCmd);
									//	ReachCheck();
									   }

IndReachCheck(32);
IndReachCheck(14);
}


void PMPThread::ChooseANN(double Thrish)
{

  if(Thrish > 0){
					LoadANN();
					initializeANN(32);
					Time::delay(0.2);
					CloseANN();
					robCmd=14;
					cout << "Simulating movememnt with Robot TX90 "<< endl;
					CJATx[0]=CJA[6];
					CJATx[1]=CJA[7];
					CJATx[2]=CJA[8];
					CJATx[3]=CJA[9];
					CJATx[4]=CJA[10];
					CJATx[5]=CJA[11];
					double *angIniObRx = CJATx;
					double *nFK = forward_Kinematics(angIniObRx,1); // Joint Angles to Positions 3>>>>>2
					int i;
						for(i=0;i<3;i++)
						{
							X_posIniObRx[i]=*(nFK+i);
						}
						if(X_posIniObRx[1]<-260)
						{
						  cout<<"Need to avoid Trays:::near Tx"<< endl;
						//  AvoidTray=32;
						}
						cout<<"FK computed by PMP TX is"<<X_posIniObRx[0]<< X_posIniObRx[1]<<endl;
						

				}
				else{
					LoadANN();
					initializeANN(14);
					Time::delay(0.2);
					CloseANN();
					robCmd=32;
					cout << "Simulating movememnt with Robot RX130 "<< endl;
					CJARx[0]=CJA[0];
					CJARx[1]=CJA[1];
					CJARx[2]=CJA[2];
					CJARx[3]=CJA[3];
					CJARx[4]=CJA[4];
					CJARx[5]=CJA[5];
					double *angIniObRx = CJARx;
					double *nFK = forward_Kinematics(angIniObRx,1); // Joint Angles to Positions 3>>>>>2
					int i;
						for(i=0;i<3;i++)
						{
							X_posIniObRx[i]=*(nFK+i);
						}
						if(X_posIniObRx[1]<-260)
						{
						  cout<<"Need to avoid Trays:::near Rx"<< X_posIniObRx[1]<<endl;
						 // AvoidTray=32;
						}
						cout<<"FK computed by PMP is  "<<X_posIniObRx[0]<< " , "<<X_posIniObRx[1]<<" , "<<X_posIniObRx[2]<<endl;
						
				}
				
};

void PMPThread::TrayAvoidanceON()
{
	int XmitGreen;
	if(X_posIniObRx[2]<260)
								{
                                   ResPM=VTGS(X_posIniObRx[0],X_posIniObRx[1],380,VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
								  XmitGreen=0;
								  if(robCmd==32)
								  {
								    JStoreRx[NsetJ][0]=ang1;
									JStoreRx[NsetJ][1]=ang2;
									JStoreRx[NsetJ][2]=ang3;
									JStoreRx[NsetJ][3]=ang4;
									JStoreRx[NsetJ][4]=ang5;
									JStoreRx[NsetJ][0]=ang6;
									NsetJ=NsetJ+1;
								  }
				//MSFlag=1;
							 if(ResPM==1)
								   {
									   cout << "PMP Simulation converged" << endl;
									  // cin >> XmitGreen;
									   XmitGreen=1;
									   if(MSFlag==1){
										cout << "Executing Movement::Transmiting motor commands" << endl;
										VariableSpeedCtrl(robCmd); 
										MessagePassR(robCmd);
						//				ReachCheck();
									   }
									 //  ResPM=2;
								   }

								}
							//--------------------------------------------------
							 ResPM=VTGS(VTGSIN[0],VTGSIN[1],320,VTGSIN[3],VTGSIN[4],VTGSIN[5]+40,0,0,0);  // Changes 2005
							 XmitGreen=0;
							 if(robCmd==32)
								  {
								    JStoreRx[NsetJ][0]=ang1;
									JStoreRx[NsetJ][1]=ang2;
									JStoreRx[NsetJ][2]=ang3;
									JStoreRx[NsetJ][3]=ang4;
									JStoreRx[NsetJ][4]=ang5;
									JStoreRx[NsetJ][0]=ang6;
									NsetJ=NsetJ+1;
								  }
				//MSFlag=1;
							 if(ResPM==1)
								   {
									   cout << "PMP Simulation converged" << endl;
									  // cin >> XmitGreen;
									   XmitGreen=1;
									   if(MSFlag==1){
										cout << "Executing Movement::Transmiting motor commands" << endl;
										VariableSpeedCtrl(robCmd); 
										MessagePassR(robCmd);
							//			ReachCheck();
									   }
									 //  ResPM=2;
								   }


};
 bool PMPThread::detectCollision()
 {
  


	std::vector<float> jointTx(6,0);
	std::vector<float> jointRx(6,0);
	int collisionFlag=0;
	for(int j=0;j<NsetJ;j++)
	{
	for (int i=0;i<6;i++){
		jointRx[i]=JStoreRx[j][i];
		jointTx[i]=JStoreTx[0][i];
	}


			std::cout<<"Joints of TX before position calculation "<<jointTx[0] <<" , "<<jointTx[1] <<" , "<<jointTx[2] <<" , "
			<<jointTx[3] <<" , "<<jointTx[4] <<" , "<<jointTx[5] <<endl;

		std::vector<LibRoboMove::Kinematics::vecType> testTx = getBodyConfig(jointTx,1);

		for(int i= 0; i< testTx.size();++i)
		{
			std::cout<< " The TX position of Joint "<<i+1<< "  is at :" <<testTx[i].getX() <<" , "<<testTx[i].getY() <<" , "<<testTx[i].getZ() <<std::endl<<std::endl;

		}
				std::cout<<"Joints of RX before position calculation "<<jointRx[0] <<" , "<<jointRx[1] <<" , "<<jointRx[2] <<" , "
			<<jointRx[3] <<" , "<<jointRx[4] <<" , "<<jointRx[5] <<endl;


		std::vector<LibRoboMove::Kinematics::vecType> testRx = getBodyConfig(jointRx,2);

		for(int i= 0; i< testRx.size();++i)
		{
			std::cout<< " The RX position of Joint "<<i+1<< "  is at :" <<testRx[i].getX() <<" , "<<testRx[i].getY() <<" , "<<testRx[i].getZ() <<std::endl;

		}

		float xdist4 = testTx[4].getX()-testRx[4].getX();
		float ydist4 = testTx[4].getY()-testRx[4].getY();
		float zdist4 = testTx[4].getZ()-testRx[4].getZ();
		float netDiff4 = sqrt (pow(xdist4,2)+pow(ydist4,2)+pow(zdist4,2));

		float xdist5 = testTx[5].getX()-testRx[5].getX();
		float ydist5 = testTx[5].getY()-testRx[5].getY();
		float zdist5 = testTx[5].getZ()-testRx[5].getZ();
		float netDiff5 = sqrt (pow(xdist5,2)+pow(ydist5,2)+pow(zdist5,2));
		std::cout<< " distance between joint 5 of two robots "<<netDiff4<<endl;
		std::cout<< " distance between joint 6 of two robots "<<netDiff5<<endl;
		if (netDiff4 <300 || netDiff5 < 300){
			collisionFlag=1;
		}
		if (collisionFlag==1)
			break;
	}
	if (collisionFlag==1){
			std::cout<< " Possible collisions detected "<<endl;
			return true;
		}
		else {
			std::cout<< "Movement possible without collisions  "<<endl;
			return false;
		}


 }

void PMPThread::ReadCurrentJoints()
{
  Bottle* jointsin = Inpjoints.read(false);

		std::vector<double> currposition(12,0);
		std::vector<float> jointTx(6,0);
		std::vector<float> jointRx(6,0);


		if(jointsin!=NULL )
		{
			for(int i=0;i<6;i++)
			{
				CJA[i] = jointsin->get(i).asDouble();
				jointRx[i] = (float) CJA[i];
				CJA[i] = CJA[i]*(3.14/180);
			}

			for(int i=6;i<12;i++)
			{
				CJA[i] = jointsin->get(i).asDouble();
				jointTx[i-6] = (float) CJA[i];
				CJA[i] = CJA[i]*(3.14/180);
			}
		}


		std::vector<LibRoboMove::Kinematics::vecType> testTx = getBodyConfig(jointTx,1);

		for(int i= 0; i< testTx.size();++i)
		{
			std::cout<< " The TX position of Joint "<<i+1<< "  is at :" <<testTx[i].getX() <<" , "<<testTx[i].getY() <<" , "<<testTx[i].getZ() <<std::endl<<std::endl;

		}

		std::vector<LibRoboMove::Kinematics::vecType> testRx = getBodyConfig(jointRx,1);

		for(int i= 0; i< testRx.size();++i)
		{
			std::cout<< " The RX position of Joint "<<i+1<< "  is at :" <<testRx[i].getX() <<" , "<<testRx[i].getY() <<" , "<<testRx[i].getZ() <<std::endl;

		}
}

void PMPThread::AngUpStore(){
   
	AngUp[0]=ang1;
	AngUp[1]=ang2;
	AngUp[2]=ang3;
	AngUp[3]=ang4;
	AngUp[4]=ang5;
	AngUp[5]=ang6;
}

void PMPThread::AngUpLoad(){

    ang1=AngUp[0];
	ang2=AngUp[1];
	ang3=AngUp[2];
	ang4=AngUp[3];
	ang5=AngUp[4];
	ang6=AngUp[5];

}

//void PMPThread::ReachCheck(int robcmd)
//{
// std::cout<<"Reach checking .. "<<std::endl;
// bool jointmoveflag =true;
//
// while(jointmoveflag)
// {
//  Bottle* jointsin = Inpjoints.read(false);
//
//  std::vector<double> currposition(6,0);
//  if(jointsin!=NULL )
//  {
//   if(robcmd == 14)
//   {
//    for(int i=0;i<6;i++)
//    {
//     currposition[i] = jointsin->get(i+6).asDouble();
//    }
//   }
//
//   else if(robcmd == 32)
//   {
//    for(int i=0;i<6;i++)
//    {
//     currposition[i] = jointsin->get(i).asDouble();
//    }
//   }
//
//   double squaredDiff = 0;
//
//   if(robcmd == 14)
//   {
//    for (int i = 0; i < currposition.size(); i++)
//    {
//     double d = currposition[i] - movingjoints[i+6];
//     squaredDiff += d * d;
//    }
//   }
//
//   else if(robcmd == 32)
//   {
//    for (int i = 0; i < currposition.size(); i++)
//    {
//     double d = currposition[i] - movingjoints[i];
//     squaredDiff += d * d;
//    }
//   }
//
//   if (sqrt(squaredDiff) < 0.1)
//   {
//    jointmoveflag = false;
//
//    std::cout<<"The joint angles compairison: "<<std::endl;
//
//    if(robcmd == 14)
//    {
//     for (int i = 0; i < currposition.size(); i++)
//     {
//      std::cout<<" movingjoint["<<i<<"]  , currjoint["<<i<<"]:  "<<movingjoints[i+6]<<"  ,  "<<currposition[i]<<std::endl; 
//     }
//    }
//
//    else if(robcmd == 32)
//    {
//     for (int i = 0; i < currposition.size(); i++)
//     {
//      std::cout<<" movingjoint["<<i<<"]  , currjoint["<<i<<"]:  "<<movingjoints[i]<<"  ,  "<<currposition[i]<<std::endl; 
//     }
//    }
//
//   }
//
//  }
// }
//
//};


void PMPThread::ReachCheck()
{
	bool jointmoveflag =true;

	while(jointmoveflag)
	{
		Bottle* jointsin = Inpjoints.read(false);

		std::vector<double> currposition(12,0);
		if(jointsin!=NULL )
		{
			for(int i=0;i<12;i++)
			{
				currposition[i] = jointsin->get(i).asDouble();
			}
			double squaredDiff = 0;
			for (int i = 0; i < currposition.size(); i++)
			{
				double d = currposition[i] - movingjoints[i];
				squaredDiff += d * d;
			}
			if (sqrt(squaredDiff) < 0.1)
			{
				jointmoveflag = false;

				std::cout<<"The joint angles compairison: "<<std::endl;

				for(int i=0;i<12;i++)
				{
					std::cout<<" movingjoint["<<i<<"]  , currjoint["<<i<<"]:  "<<movingjoints[i]<<"  ,  "<<currposition[i]<<std::endl;	
				}
			}



		}
	}


}

void PMPThread::threadRelease() {
	// nothing

}

void PMPThread::onStop() {

	//    outputPort.interrupt();


	//  outputPort.close();
}

void PMPThread::initTX() {

	ang1 = -12.28;
	ang2 = -2.84;
	ang3 = 83.07;
	ang4 = 1.1;
	ang5 = 8.87;
	ang6 = 4.01;

}

void PMPThread::initRX() {

	ang1 = 5;
	ang2 = -14;
	ang3 = 90;
	ang4 = 0;
	ang5 = 90;
	ang6 = -23;

}

double* PMPThread::forward_Kinematics(double *u , int l)
{
	//double *p;
	//double a[3];
	//double T_Len=0; double T_Ori=0;
	////modified kinem eqns VM 21 Jan, old params in PMOTX90LSeb
	//a[0]=2.08970*sin(u[4])*sin(u[3])*cos(u[0])-269.490*sin(u[4])*sin(u[3])*sin(u[0])+549.970*sin(u[2])*cos(u[1])*cos(u[0])+4.26480*sin(u[2])*cos(u[1])*sin(u[0])+0.002*sin(u[2])*sin(u[1])*sin(u[0])-0.000015*sin(u[2])*sin(u[1])*cos(u[0])+549.970*cos(u[2])*sin(u[1])*cos(u[0])+4.26480*cos(u[2])*sin(u[1])*sin(u[0])-0.002*cos(u[2])*cos(u[1])*sin(u[0])+0.000015*cos(u[2])*cos(u[1])*cos(u[0])+1.68030*cos(u[4])*cos(u[2])*cos(u[1])-1.68030*cos(u[4])*sin(u[2])*sin(u[1])+0.000007*cos(u[4])*cos(u[3])*cos(u[0])-0.00098*cos(u[4])*cos(u[3])*sin(u[0])-0.000004*sin(u[3])*sin(u[2])*cos(u[1])+0.000007*sin(u[4])*sin(u[3])*cos(u[2])*sin(u[1])*sin(u[0])+0.00098*sin(u[4])*sin(u[3])*cos(u[2])*sin(u[1])*cos(u[0])+269.490*cos(u[4])*cos(u[2])*sin(u[1])*cos(u[0])+2.08970*cos(u[4])*sin(u[2])*cos(u[1])*sin(u[0])+0.00098*cos(u[4])*sin(u[2])*sin(u[1])*sin(u[0])-0.000007*cos(u[4])*sin(u[2])*sin(u[1])*cos(u[0])+269.490*cos(u[4])*sin(u[2])*cos(u[1])*cos(u[0])-0.00098*cos(u[4])*sin(u[3])*cos(u[2])*cos(u[1])*cos(u[0])-1.68030*sin(u[4])*cos(u[3])*cos(u[2])*sin(u[1])-0.000006*sin(u[4])*sin(u[3])-0.000007*sin(u[4])*cos(u[3])*cos(u[2])*sin(u[1])*cos(u[0])+3.87700*sin(u[1])*sin(u[0])-0.000006*sin(u[4])*sin(u[3])*sin(u[2])*sin(u[1])-0.000007*sin(u[4])*cos(u[3])*sin(u[2])*cos(u[1])*cos(u[0])+50.3850*cos(u[0])-49.6080*sin(u[0])+3.11750*cos(u[1])+2.08970*cos(u[4])*cos(u[2])*sin(u[1])*sin(u[0])+499.980*sin(u[1])*cos(u[0])-2.08970*sin(u[4])*cos(u[3])*sin(u[2])*sin(u[1])*sin(u[0])+0.00098*sin(u[4])*cos(u[3])*cos(u[2])*sin(u[1])*sin(u[0])-3.42920*sin(u[2])*sin(u[1])+0.000007*cos(u[4])*cos(u[2])*cos(u[1])*cos(u[0])+3.42920*cos(u[2])*cos(u[1])-1.68030*sin(u[4])*cos(u[3])*sin(u[2])*cos(u[1])-0.000007*cos(u[4])*cos(u[0])+0.000007*cos(u[4])*sin(u[3])*sin(u[2])*sin(u[1])*sin(u[0])+2.08970*sin(u[4])*cos(u[3])*cos(u[2])*cos(u[1])*sin(u[0])-0.00098*cos(u[4])*cos(u[2])*cos(u[1])*sin(u[0])+0.00066*sin(u[3])*cos(u[2])*cos(u[1])*cos(u[0])+0.000005*sin(u[3])*cos(u[2])*cos(u[1])*sin(u[0])+0.00098*sin(u[4])*cos(u[3])*sin(u[2])*cos(u[1])*sin(u[0])-0.000007*cos(u[4])*sin(u[3])*cos(u[2])*cos(u[1])*sin(u[0])+0.00098*cos(u[4])*sin(u[0])-0.000005*sin(u[3])*sin(u[2])*sin(u[1])*sin(u[0])+0.000006*cos(u[4])*sin(u[3])*sin(u[2])*cos(u[1])-0.00066*sin(u[3])*sin(u[2])*sin(u[1])*cos(u[0])-0.000005*cos(u[3])*cos(u[0])+0.000006*cos(u[4])*sin(u[3])*cos(u[2])*sin(u[1])+0.00098*sin(u[4])*sin(u[3])*sin(u[2])*cos(u[1])*cos(u[0])+0.00066*cos(u[3])*sin(u[0])+0.000014*cos(u[1])*cos(u[0])+269.490*sin(u[4])*cos(u[3])*cos(u[2])*cos(u[1])*cos(u[0])+0.000007*sin(u[4])*sin(u[3])*sin(u[2])*cos(u[1])*sin(u[0])-0.0018*cos(u[1])*sin(u[0])+0.00098*cos(u[4])*sin(u[3])*sin(u[2])*sin(u[1])*cos(u[0])+0.000006*sin(u[4])*sin(u[3])*cos(u[2])*cos(u[1])-269.490*sin(u[4])*cos(u[3])*sin(u[2])*sin(u[1])*cos(u[0])-409.680-0.000004*sin(u[3])*cos(u[2])*sin(u[1]);
	//a[1]=269.490*sin(u[4])*sin(u[3])*cos(u[0])+2.08680*sin(u[4])*sin(u[3])*sin(u[0])-4.25880*sin(u[2])*cos(u[1])*cos(u[0])+549.980*sin(u[2])*cos(u[1])*sin(u[0])-0.000015*sin(u[2])*sin(u[1])*sin(u[0])-0.0020*sin(u[2])*sin(u[1])*cos(u[0])-4.25880*cos(u[2])*sin(u[1])*cos(u[0])+549.980*cos(u[2])*sin(u[1])*sin(u[0])+0.000015*cos(u[2])*cos(u[1])*sin(u[0])+0.0020*cos(u[2])*cos(u[1])*cos(u[0])-0.477*cos(u[4])*cos(u[2])*cos(u[1])+0.477*cos(u[4])*sin(u[2])*sin(u[1])+0.00098*cos(u[4])*cos(u[3])*cos(u[0])+0.000007*cos(u[4])*cos(u[3])*sin(u[0])+0.000001*sin(u[3])*sin(u[2])*cos(u[1])+0.00098*sin(u[4])*sin(u[3])*cos(u[2])*sin(u[1])*sin(u[0])-0.000007*sin(u[4])*sin(u[3])*cos(u[2])*sin(u[1])*cos(u[0])-2.08680*cos(u[4])*cos(u[2])*sin(u[1])*cos(u[0])+269.490*cos(u[4])*sin(u[2])*cos(u[1])*sin(u[0])-0.000007*cos(u[4])*sin(u[2])*sin(u[1])*sin(u[0])-0.00098*cos(u[4])*sin(u[2])*sin(u[1])*cos(u[0])-2.08680*cos(u[4])*sin(u[2])*cos(u[1])*cos(u[0])+0.000007*cos(u[4])*sin(u[3])*cos(u[2])*cos(u[1])*cos(u[0])+0.477*sin(u[4])*cos(u[3])*cos(u[2])*sin(u[1])+0.000001*sin(u[4])*sin(u[3])+799.630-0.00098*sin(u[4])*cos(u[3])*cos(u[2])*sin(u[1])*cos(u[0])+499.980*sin(u[1])*sin(u[0])+0.000001*sin(u[4])*sin(u[3])*sin(u[2])*sin(u[1])-0.00098*sin(u[4])*cos(u[3])*sin(u[2])*cos(u[1])*cos(u[0])+49.6100*cos(u[0])+50.3860*sin(u[0])-0.8855*cos(u[1])+269.490*cos(u[4])*cos(u[2])*sin(u[1])*sin(u[0])-3.87160*sin(u[1])*cos(u[0])-269.490*sin(u[4])*cos(u[3])*sin(u[2])*sin(u[1])*sin(u[0])-0.000007*sin(u[4])*cos(u[3])*cos(u[2])*sin(u[1])*sin(u[0])+00.97410*sin(u[2])*sin(u[1])+0.00098*cos(u[4])*cos(u[2])*cos(u[1])*cos(u[0])-00.97410*cos(u[2])*cos(u[1])+0.477*sin(u[4])*cos(u[3])*sin(u[2])*cos(u[1])-0.00098*cos(u[4])*cos(u[0])+0.00098*cos(u[4])*sin(u[3])*sin(u[2])*sin(u[1])*sin(u[0])+269.490*sin(u[4])*cos(u[3])*cos(u[2])*cos(u[1])*sin(u[0])+0.000007*cos(u[4])*cos(u[2])*cos(u[1])*sin(u[0])-0.000005*sin(u[3])*cos(u[2])*cos(u[1])*cos(u[0])+0.00066*sin(u[3])*cos(u[2])*cos(u[1])*sin(u[0])-0.000007*sin(u[4])*cos(u[3])*sin(u[2])*cos(u[1])*sin(u[0])-0.00098*cos(u[4])*sin(u[3])*cos(u[2])*cos(u[1])*sin(u[0])-0.000007*cos(u[4])*sin(u[0])-0.00066*sin(u[3])*sin(u[2])*sin(u[1])*sin(u[0])-0.000001*cos(u[4])*sin(u[3])*sin(u[2])*cos(u[1])+0.000005*sin(u[3])*sin(u[2])*sin(u[1])*cos(u[0])-0.00066*cos(u[3])*cos(u[0])-0.000001*cos(u[4])*sin(u[3])*cos(u[2])*sin(u[1])-0.000007*sin(u[4])*sin(u[3])*sin(u[2])*cos(u[1])*cos(u[0])-0.000005*cos(u[3])*sin(u[0])+0.0018*cos(u[1])*cos(u[0])-2.08680*sin(u[4])*cos(u[3])*cos(u[2])*cos(u[1])*cos(u[0])+0.00098*sin(u[4])*sin(u[3])*sin(u[2])*cos(u[1])*sin(u[0])+0.000014*cos(u[1])*sin(u[0])-0.000007*cos(u[4])*sin(u[3])*sin(u[2])*sin(u[1])*cos(u[0])-0.000001*sin(u[4])*sin(u[3])*cos(u[2])*cos(u[1])+2.08680*sin(u[4])*cos(u[3])*sin(u[2])*sin(u[1])*cos(u[0])+0.000001*sin(u[3])*cos(u[2])*sin(u[1]);
	//a[2]=173.68+269.50*((((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*cos(u[2])+(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*sin(u[2]))*cos(u[3])+(-0.000003+0.000003*((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*sin(u[2])-0.000003*(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*cos(u[2])+0.0062*sin(u[0])+0.0017*cos(u[0]))*sin(u[3]))*sin(u[4])-269.50*(-0.0+0.000003*(((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*cos(u[2])+(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*sin(u[2]))*sin(u[3])-0.000003*(-0.000003+0.000003*((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*sin(u[2])-0.000003*(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*cos(u[2])+0.0062*sin(u[0])+0.0017*cos(u[0]))*cos(u[3])-1.0000*((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*sin(u[2])+1.0000*(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*cos(u[2])+0.0*sin(u[0])+0.0*cos(u[0]))*cos(u[4])+0.00066*(((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*cos(u[2])+(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*sin(u[2]))*sin(u[3])-0.00066*(-0.000003+0.000003*((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*sin(u[2])-0.000003*(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*cos(u[2])+0.0062*sin(u[0])+0.0017*cos(u[0]))*cos(u[3])+550.00*((-0.0062*cos(u[0])+0.0017*sin(u[0]))*cos(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*sin(u[1]))*sin(u[2])-550.00*(-1.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])+(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]))*cos(u[2])+0.39856*sin(u[0])-0.22629*cos(u[0])+500.0*(-0.0062*cos(u[0])+0.0017*sin(u[0]))*sin(u[1])-500.0*(-0.9999-0.0*sin(u[0])-0.0*cos(u[0]))*cos(u[1]);
	//p=a;
	//return p;
	double *p;
   double a[3];
   double T_Len=0; double T_Ori=0;
    inputL=6;
	hiddenL1=32;
	hiddenL2=41;
	outputL=3;
   	double h1[32],h2[41];
   	for(int v=0;v<hiddenL1;v++) {
		double sum = 0;
		for(int i=0;i<inputL;i++) {
      		sum = sum + w1[v][i]*u[i];   
	    }
		h1[v] = sum + b1[v][0];
		h1[v] = tanh(h1[v]);
	}

	for(int v=0;v<hiddenL2;v++) {
		double sum = 0;
		for(int i=0;i<hiddenL1;i++) {
      		sum = sum + w2[v][i]*h1[i];   
	    }
		h2[v] = sum + b2[v][0];
		h2[v] = tanh(h2[v]);
	}

	for(int v=0;v<outputL;v++) {
		double sum = 0;
		for(int i=0;i<hiddenL2;i++) {
      		sum = sum + w3[v][i]*h2[i];   
	    }
		a[v] = sum + b3[v][0];
		//a[v] = tanh(a[v]);
	}
   p=a;
   return p;
};


double* PMPThread::forcefield(double *w, double*v)
{
	int j;
	//double *ptr;
	double pos[3], tar[3],res[3];
	for(j=0;j<3;j++)
	{
		pos[j]=*(w+j);
		tar[j]=*(v+j);
		res[j]=KFORCE*(tar[j]-pos[j]);// Virtual stiffness is multiplied here //0.019 0.8 
	}
	ptr=res;
	return ptr;
};



double* PMPThread::PMPJack(double *force)
{
	int i,u;
    double pi=3.14;
    double ff[3],Joint_Field[10],Jvel[6];
	double *foof;

    for(i=0;i<3;i++)
	{
	ff[i]=*(force+i);
   	}
    ff[0]=ff[0]*1;
	ff[1]=ff[1]*1;
	ff[2]=ff[2]*1;

	inputL=6;
	hiddenL1=32;
	hiddenL2=41;
	outputL=3;
//==========================ANN implementation =====//
	
	double h[32][1],z[41][1],p[41][1],hinter[32][1],p1[41][1],pinter[41][1],Jack[3][10],JacT[10][3];
//==================== Loading Weights, Biases for the ANN ====================

	for(u=0;u<hiddenL1;u++) {
		double sum=0;
		for(i=0;i<inputL;i++){
       		sum=sum+(w1[u][i]*Jan[i]);
	   	}
		h[u][0]=sum+b1[u][0]; // Inner variable of Layer 1
		z[u][0]=tanh(h[u][0]);// output of layer 1
	hinter[u][0]=1-(pow(z[u][0],2));    //(1-tanh(h(b,1))^2)
	}
//================W1/B1work over here, u have 'h' and 'z'===================

	for(u=0;u<hiddenL2;u++) {
		double sum2=0;
		for(i=0;i<hiddenL1;i++) {
      		
       		sum2=sum2+w2[u][i]*z[i][0];
		}
		p[u][0]=sum2+b2[u][0]; // here u get p int variable of layer 2 ////////
		p1[u][0]=tanh(p[u][0]);// output of layer 1
		pinter[u][0]=1-(pow(p1[u][0],2));
		
	}

	/////////////////////////////////////////////////////////Added SEND ACTIVATIONS To PORT FOR DISPLAY
    if (activationsPort.getOutputCount()) {
        Bottle& actv =activationsPort.prepare();
    	actv.clear();
		for(int i=0; i<hiddenL1; i++)
            actv.addDouble(hinter[i][0]); //added layer1 neuron activations

		for(int i=0; i<16; i++)
            actv.addDouble(0); // added required gap values to that of icub

		for(int i=0; i<hiddenL2; i++)
            actv.addDouble(pinter[i][0]); // added layer2 neuron activations

		for(int i=0; i<14; i++)
            actv.addDouble(0); // added required gap values to that of icub
	    //cout<<"Sending Activations for Display"<<endl;
       	activationsPort.write();
        Time::delay(0.03);
    }
////////////////////////////////////////////////////////////////////////////////////


	//int l=hiddenL2,j=hiddenL1,k,n,a,b;

	for(int k=0;k<outputL;k++) {
		for(int n=0;n<inputL;n++) {
			double inter1=0;
			for(int a=0;a<hiddenL2;a++) {
				for(int b=0;b<hiddenL1;b++) {
    				inter1=inter1 +((w3[k][a]*pinter[a][0])*((w2[a][b]*hinter[b][0])*w1[b][n]));
				}
			}
			Jack[k][n]=inter1;
			//fprintf(writeJ,"\n \t  %f",Jack[k][n]);
		}	
	}	
	    Joint_Field[0]=(0-Jan[0])*J2H; // Multiply by Joint compliance 
		Joint_Field[1]=(0-Jan[1])*J2H; //0.52 / Modified in June at Crete
		Joint_Field[2]=(0-Jan[2])*J2H;  //1.8
		Joint_Field[3]=(0-Jan[3])*500;  //4.5 //0.95
		Joint_Field[4]=(wrioTx-Jan[4])*900; 
		if (robCmd==14)
		{
		Joint_Field[5]=(2.09-Jan[5])*110; // Multiply by Joint compliance //was 410
		}
		if (robCmd==32)
		{
		Joint_Field[5]=(-1.41-Jan[5])*110; // Multiply by Joint compliance //was 410
		}
      //  Joint_Field[5]=( Wr_Iorient-Jan[5])*410; //for pushing
//=======================================================================================

		for(int a=0;a<inputL;a++) {
		double jvelo=0;
    	for(int n=0;n<outputL;n++) {
	   		JacT[a][n]=Jack[n][a];
       		jvelo=jvelo+(JacT[a][n]*ff[n]);
		}
    	Jvel[a]=0.002*(jvelo+Joint_Field[a]);
	} 

    foof=Jvel;
	return foof;
	//int i;
	//double pi=3.14;
	//double ff[3],Jacob[30],Joint_Field[10],Jvel[20];
	//double visco[5]={1,1,1,1,1};
	//double *foof;

	//for(i=0;i<3;i++)
	//{
	//	ff[i]=*(force+i);
	//}
	//ff[0]=ff[0]*1;
	//ff[1]=ff[1]*1;
	//ff[2]=ff[2]*1;

	////=======================================================================================
	//// Jacobian for the arm
	////=======================================================================================
	//Jacob[0]=-269.490*sin(Jan[4])*sin(Jan[3])*cos(Jan[0])-2.08970*sin(Jan[4])*sin(Jan[3])*sin(Jan[0])+4.26480*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-549.970*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000015*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.002*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+4.26480*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-549.970*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000015*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.002*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[3])*cos(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[3])*sin(Jan[0])+0.00066*cos(Jan[3])*cos(Jan[0])-269.490*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+2.08970*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-499.980*sin(Jan[1])*sin(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-49.6080*cos(Jan[0])-50.3850*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+269.490*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+2.08970*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000014*cos(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+2.08970*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+3.87700*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000005*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00066*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000005*cos(Jan[3])*sin(Jan[0])-269.490*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-2.08970*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000005*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[0])-0.0018*cos(Jan[1])*cos(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*cos(Jan[0]);
	//Jacob[1]=-0.000015*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.002*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-4.26480*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-549.970*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000015*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.002*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+4.26480*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+549.970*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000006*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])+0.00098*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-1.68030*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])+0.0018*sin(Jan[1])*sin(Jan[0])-1.68030*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])-2.08970*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-269.490*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-3.11750*sin(Jan[1])-0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-2.08970*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-1.68030*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])+0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000005*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000006*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])+3.87700*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000005*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000014*sin(Jan[1])*cos(Jan[0])-2.08970*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000004*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])+0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+269.490*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000006*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])-0.000006*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])-269.490*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-3.42920*sin(Jan[2])*cos(Jan[1])-3.42920*cos(Jan[2])*sin(Jan[1])+1.68030*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])+499.980*cos(Jan[1])*cos(Jan[0])+2.08970*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000004*sin(Jan[3])*cos(Jan[2])*cos(Jan[1]);
	//Jacob[2]=-0.000015*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.002*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-4.26480*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-549.970*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000015*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.002*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+4.26480*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+549.970*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000006*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])+0.00098*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-1.68030*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])-1.68030*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])-2.08970*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-269.490*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-2.08970*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-1.68030*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])+0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000005*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000006*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])+0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000005*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-2.08970*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000004*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])+0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+269.490*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000006*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])-0.000006*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])-269.490*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-3.42920*sin(Jan[2])*cos(Jan[1])-3.42920*cos(Jan[2])*sin(Jan[1])+1.68030*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])+2.08970*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000004*sin(Jan[3])*cos(Jan[2])*cos(Jan[1]);
	//Jacob[3]=1.68030*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])-0.000006*sin(Jan[4])*cos(Jan[3])+0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000006*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])+2.08970*sin(Jan[4])*cos(Jan[3])*cos(Jan[0])+2.08970*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-2.08970*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[0])+0.000005*sin(Jan[3])*cos(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+269.490*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*sin(Jan[0])+0.000005*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000006*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])-0.00066*sin(Jan[3])*sin(Jan[0])+0.00066*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[0])-0.000004*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])+0.000007*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000004*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])+1.68030*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])+0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000006*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])+0.00098*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00066*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000005*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000006*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1]);
	//Jacob[4]=-0.000006*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])-0.000006*cos(Jan[4])*sin(Jan[3])-2.08970*sin(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-269.490*sin(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[0])+2.08970*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+2.08970*cos(Jan[4])*sin(Jan[3])*cos(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000006*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])+0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[0])-1.68030*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])-269.490*cos(Jan[4])*sin(Jan[3])*sin(Jan[0])-1.68030*sin(Jan[4])*cos(Jan[2])*cos(Jan[1])+1.68030*sin(Jan[4])*sin(Jan[2])*sin(Jan[1])-2.08970*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+269.490*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000006*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])-0.000006*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])-0.00098*sin(Jan[4])*sin(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-2.08970*sin(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-269.490*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-1.68030*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1]);
	//Jacob[5]=0;

	//Jacob[6]=2.08680*sin(Jan[4])*sin(Jan[3])*cos(Jan[0])-269.490*sin(Jan[4])*sin(Jan[3])*sin(Jan[0])+549.980*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+4.25880*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.002*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000015*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+549.980*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+4.25880*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.002*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000015*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000007*cos(Jan[4])*cos(Jan[3])*cos(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[3])*sin(Jan[0])-0.000005*cos(Jan[3])*cos(Jan[0])+2.08680*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+269.490*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+3.87160*sin(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+50.3860*cos(Jan[0])-49.6100*sin(Jan[0])-0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000005*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-2.08680*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+269.490*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.0018*cos(Jan[1])*sin(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+269.490*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+499.980*sin(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+2.08680*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00066*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000005*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00066*cos(Jan[3])*sin(Jan[0])+2.08680*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[0])+0.000014*cos(Jan[1])*cos(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[0]);
	//Jacob[7]=-0.002*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000015*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-549.980*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+4.25880*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.002*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000015*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+549.980*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-4.25880*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000001*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])-0.000007*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.477*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])-0.000014*sin(Jan[1])*sin(Jan[0])+0.477*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])-269.490*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+2.08680*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.8855*sin(Jan[1])+0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.477*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000005*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000005*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000001*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])+499.980*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.0018*sin(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+2.08680*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000001*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])-0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-2.08680*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000001*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])+0.000001*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])+2.08680*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.97410*sin(Jan[2])*cos(Jan[1])+0.97410*cos(Jan[2])*sin(Jan[1])-0.477*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])-3.87160*cos(Jan[1])*cos(Jan[0])+269.490*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000001*sin(Jan[3])*cos(Jan[2])*cos(Jan[1]);
	//Jacob[8]=-0.002*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000015*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-549.980*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+4.25880*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.002*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000015*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+549.980*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-4.25880*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000001*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])-0.000007*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.477*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])+0.477*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])-269.490*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+2.08680*cos(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.477*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000005*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000005*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000001*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])-0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.00066*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+2.08680*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000001*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])-0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-2.08680*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000001*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])+0.000001*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])+2.08680*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.97410*sin(Jan[2])*cos(Jan[1])+0.97410*cos(Jan[2])*sin(Jan[1])-0.477*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])+269.490*cos(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000001*sin(Jan[3])*cos(Jan[2])*cos(Jan[1]);
	//Jacob[9]=-0.477*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])+0.000001*sin(Jan[4])*cos(Jan[3])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000001*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])+269.490*sin(Jan[4])*cos(Jan[3])*cos(Jan[0])+269.490*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-269.490*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[0])+0.00066*sin(Jan[3])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-2.08680*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+2.08680*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+2.08680*sin(Jan[4])*cos(Jan[3])*sin(Jan[0])+0.00066*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000001*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])+0.000005*sin(Jan[3])*sin(Jan[0])-0.000005*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[0])+0.000001*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])+0.00098*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000007*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000001*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])-0.477*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])-0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.000001*sin(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])-0.000007*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000005*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00066*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000001*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1]);
	//Jacob[10]=0.000001*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])+0.000001*cos(Jan[4])*sin(Jan[3])-269.490*sin(Jan[4])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])+2.08680*sin(Jan[4])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+2.08680*sin(Jan[4])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+0.00098*sin(Jan[4])*cos(Jan[0])+269.490*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.00098*sin(Jan[4])*cos(Jan[3])*cos(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.00098*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.00098*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])*cos(Jan[0])+269.490*cos(Jan[4])*sin(Jan[3])*cos(Jan[0])+0.00098*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+0.000001*cos(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])-0.000007*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.000007*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])-0.000007*sin(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000007*sin(Jan[4])*cos(Jan[3])*sin(Jan[0])+0.477*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])+2.08680*cos(Jan[4])*sin(Jan[3])*sin(Jan[0])+0.477*sin(Jan[4])*cos(Jan[2])*cos(Jan[1])-0.477*sin(Jan[4])*sin(Jan[2])*sin(Jan[1])-269.490*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*sin(Jan[0])-2.08680*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-0.000001*cos(Jan[4])*sin(Jan[3])*cos(Jan[2])*cos(Jan[1])+0.000001*sin(Jan[4])*sin(Jan[3])*sin(Jan[2])*cos(Jan[1])+0.000007*sin(Jan[4])*sin(Jan[0])-0.00098*sin(Jan[4])*cos(Jan[2])*cos(Jan[1])*cos(Jan[0])-269.490*sin(Jan[4])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])-0.000007*cos(Jan[4])*cos(Jan[3])*cos(Jan[2])*sin(Jan[1])*sin(Jan[0])+2.08680*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*sin(Jan[1])*cos(Jan[0])+0.477*cos(Jan[4])*cos(Jan[3])*sin(Jan[2])*cos(Jan[1]);
	//Jacob[11]=0;


	//Jacob[12]=(269.500*((0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*cos(Jan[2])+(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*sin(Jan[2]))*cos(Jan[3])+269.500*(0.000003*(0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*sin(Jan[2])-0.000003*(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*cos(Jan[2])+0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[3]))*sin(Jan[4])-1.0*(269.500*(0.000003*(0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*cos(Jan[2])+0.000003*(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*sin(Jan[2]))*sin(Jan[3])-269.500*(0.0*(0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*sin(Jan[2])-0.0*(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*cos(Jan[2])+0.0*cos(Jan[0])-0.0*sin(Jan[0]))*cos(Jan[3])-269.500*(0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*sin(Jan[2])+269.500*(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*cos(Jan[2]))*cos(Jan[4])+(0.00066*(0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*cos(Jan[2])+0.00066*(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*sin(Jan[2]))*sin(Jan[3])-1.0*(0.0*(0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*sin(Jan[2])-0.0*(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*cos(Jan[2])+0.000004*cos(Jan[0])-0.000001*sin(Jan[0]))*cos(Jan[3])+550*(0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[1])*sin(Jan[2])-550*(-0.0062*sin(Jan[0])-0.0017*cos(Jan[0]))*sin(Jan[1])*cos(Jan[2])+0.39856*cos(Jan[0])+0.226290*sin(Jan[0])+(3.10000*sin(Jan[0])+0.850000*cos(Jan[0]))*sin(Jan[1]);
	//Jacob[13]=(269.500*((-1.0*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*cos(Jan[2])+((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+0.999900*sin(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+269.500*((-0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*sin(Jan[2])-1.0*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+0.000002*sin(Jan[1]))*cos(Jan[2]))*sin(Jan[3]))*sin(Jan[4])-1.0*(269.500*(0.000003*(-1.0*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*cos(Jan[2])+0.000003*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+0.999900*sin(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-269.500*(0.000003*(-0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*sin(Jan[2])-0.000003*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+0.000002*sin(Jan[1]))*cos(Jan[2]))*cos(Jan[3])-269.500*(-1.0*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*sin(Jan[2])+269.500*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+0.999900*sin(Jan[1]))*cos(Jan[2]))*cos(Jan[4])+(0.00066*(-1.0*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*cos(Jan[2])+0.00066*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+0.999900*sin(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-1.0*(0.00066*(-0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*sin(Jan[2])-0.00066*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+0.000002*sin(Jan[1]))*cos(Jan[2]))*cos(Jan[3])+(-550*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*sin(Jan[1])-549.945*cos(Jan[1]))*sin(Jan[2])-1.0*(550*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*cos(Jan[1])+549.945*sin(Jan[1]))*cos(Jan[2])+(-3.10000*cos(Jan[0])+0.850000*sin(Jan[0]))*cos(Jan[1])-499.950*sin(Jan[1]);
	//Jacob[14]=(269.500*(-1.0*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*sin(Jan[2])+((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*cos(Jan[2]))*cos(Jan[3])+269.500*((0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*cos(Jan[2])+(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3]))*sin(Jan[4])-1.0*(269.500*(-0.000003*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*sin(Jan[2])+0.000003*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*cos(Jan[2]))*sin(Jan[3])-269.500*(0.000003*(0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*cos(Jan[2])+0.000003*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])-269.500*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*cos(Jan[2])-269.500*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[4])+(-0.00066*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*sin(Jan[2])+0.00066*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*cos(Jan[2]))*sin(Jan[3])-1.0*(0.00066*(0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*cos(Jan[2])+0.00066*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(550*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-549.945*sin(Jan[1]))*cos(Jan[2])+(550*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-549.945*cos(Jan[1]))*sin(Jan[2]);
	//Jacob[15]=(-269.500*(((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*cos(Jan[2])+((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])+269.500*(-0.000003+(0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*sin(Jan[2])-1.0*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*cos(Jan[2])+0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*cos(Jan[3]))*sin(Jan[4])-1.0*(269.500*(0.000003*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*cos(Jan[2])+0.000003*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+269.500*(-0.0+0.000003*(0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*sin(Jan[2])-0.000003*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*cos(Jan[2])+0.0*sin(Jan[0])+0.0*cos(Jan[0]))*sin(Jan[3]))*cos(Jan[4])+(0.00066*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*cos(Jan[2])+0.00066*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(-0.0+0.00066*(0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*sin(Jan[2])-0.00066*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*cos(Jan[2])+0.000004*sin(Jan[0])+0.000001*cos(Jan[0]))*sin(Jan[3]);
	//Jacob[16]=(269.500*(((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*cos(Jan[2])+((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+269.500*(-0.000003+(0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*sin(Jan[2])-1.0*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*cos(Jan[2])+0.0062*sin(Jan[0])+0.0017*cos(Jan[0]))*sin(Jan[3]))*cos(Jan[4])+(269.500*(0.000003*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*cos(Jan[2])+0.000003*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-269.500*(-0.0+0.000003*(0.000003*(-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.000002*sin(Jan[1]))*sin(Jan[2])-0.000003*(0.000003*(0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.000002*cos(Jan[1]))*cos(Jan[2])+0.0*sin(Jan[0])+0.0*cos(Jan[0]))*cos(Jan[3])-269.500*((-0.0062*cos(Jan[0])+0.0017*sin(Jan[0]))*cos(Jan[1])-0.999900*sin(Jan[1]))*sin(Jan[2])+269.500*((0.0062*cos(Jan[0])-0.0017*sin(Jan[0]))*sin(Jan[1])-0.999900*cos(Jan[1]))*cos(Jan[2]))*sin(Jan[4]);
	//Jacob[17]=0;


	//Joint_Field[0]=(0-Jan[0])*J2H; // Multiply by Joint compliance 
	//Joint_Field[1]=(0-Jan[1])*J2H; //0.52 / Modified in June at Crete
	//Joint_Field[2]=(0-Jan[2])*J2H;  //1.8
	//Joint_Field[3]=(0-Jan[3])*J2H;  //4.5 //0.95
	//Joint_Field[4]=(1.67-Jan[4])*900; // this should be the decised wrist pose computed coming from VTGS..FORTH
	//Joint_Field[5]=(0-Jan[5])*J2H; // Multiply by Joint compliance 

	////=======================================================================================
	////double arig=0.00061;

	//Jvel[0]= 1*KOMP_JANG*((ff[0]*Jacob[0])+(ff[1]*Jacob[6])+(ff[2]*Jacob[12])+Joint_Field[0]);
	//Jvel[1]= 1*KOMP_JANG*((ff[0]*Jacob[1])+(ff[1]*Jacob[7])+(ff[2]*Jacob[13])+Joint_Field[1]);
	//Jvel[2]= 1*KOMP_JANG*((ff[0]*Jacob[2])+(ff[1]*Jacob[8])+(ff[2]*Jacob[14])+Joint_Field[2]);
	//Jvel[3]= 1*KOMP_JANG*((ff[0]*Jacob[3])+(ff[1]*Jacob[9])+(ff[2]*Jacob[15])+Joint_Field[3]);
	//Jvel[4]= 1*KOMP_JANG*((ff[0]*Jacob[4])+(ff[1]*Jacob[10])+(ff[2]*Jacob[16])+Joint_Field[4]);
	//Jvel[5]= 1*KOMP_JANG*((ff[0]*Jacob[5])+(ff[1]*Jacob[11])+(ff[2]*Jacob[17])+Joint_Field[5]);

	//foof=Jvel;
	//return foof;
};

double PMPThread::Gamma_Int(double *Gar,int n)
{
	int k=1;        /* Counters in the algorithm */
	double a=0;
	double h,sum,fk;

	// Simpsons 1/3 rule for Integration

	sum=*(Gar);             /* Initial function value */
	int c=2;           
	h=1;                /*Step Size*/
	while (k <= n-1)   /* Steps through the iteration */
	{
		fk=*(Gar+k);
		c=6-c;       /* gives the 4,2,4,2,... */
		sum = (sum + c*fk);  /* Adds on the next area */
		k++;         /* Increases k value by +1 */
	}
	sum=RAMP_KONSTANT*sum/3; // changed 0.0025 to

	return sum;

};

double PMPThread::Gamma_IntDisc(double *Gar,int n)
 {
	 int k=1;        /* Counters in the algorithm */
    double a=0;
    double h,sum,fk;
	
 // Simpsons 1/3 rule for Integration

  sum=*(Gar);             /* Initial function value */
       int c=2;           
       h=1;                /*Step Size*/
	   while (k <= n-1)   /* Steps through the iteration */
           {
		     fk=*(Gar+k);
             c=6-c;       /* gives the 4,2,4,2,... */
             sum = (sum + c*fk);  /* Adds on the next area */
             k++;         /* Increases k value by +1 */
           }
	   sum=RAMP_KONSTANT*sum/3; // changed 0.0025 to
	 
	 return sum;
	
 };

void PMPThread::IndReachCheck(int robcmd)
{
 std::cout<<"Reach checking .. "<<std::endl;
 bool jointmoveflag =true;

 while(jointmoveflag)
 {
  Bottle* jointsin = Inpjoints.read(false);

  std::vector<double> currposition(6,0);
  if(jointsin!=NULL )
  {
   if(robcmd == 14)
   {
    for(int i=0;i<6;i++)
    {
     currposition[i] = jointsin->get(i+6).asDouble();
    }
   }

   else if(robcmd == 32)
   {
    for(int i=0;i<6;i++)
    {
     currposition[i] = jointsin->get(i).asDouble();
    }
   }

   double squaredDiff = 0;

   if(robcmd == 14)
   {
    for (int i = 0; i < currposition.size(); i++)
    {
     double d = currposition[i] - movingjointsTX[i];
     squaredDiff += d * d;
    }
   }

   else if(robcmd == 32)
   {
    for (int i = 0; i < currposition.size(); i++)
    {
     double d = currposition[i] - movingjointsRX[i];
     squaredDiff += d * d;
    }
   }

   if (sqrt(squaredDiff) < 0.1)
   {
    jointmoveflag = false;

    std::cout<<"The joint angles compairison: "<<std::endl;

    if(robcmd == 14)
    {
     for (int i = 0; i < currposition.size(); i++)
     {
      std::cout<<" movingjointTX["<<i<<"]  , currjoint["<<i<<"]:  "<<movingjointsTX[i]<<"  ,  "<<currposition[i]<<std::endl; 
     }
    }

    else if(robcmd == 32)
    {
     for (int i = 0; i < currposition.size(); i++)
     {
      std::cout<<" movingjointRX["<<i<<"]  , currjoint["<<i<<"]:  "<<movingjointsRX[i]<<"  ,  "<<currposition[i]<<std::endl; 
     }
    }

   }

  }
 }

}

double PMPThread::Gamma1(int _Time1)
	{
			double t_ramp1=(_Time1)*0.001;
			double t_init1=0,t_dur1=2,z1,t_win1,t_window1,csi1,prod11,prod21,Gamma1;

			z1=(t_ramp1-t_init1)/t_dur1;
			t_win1=(t_init1+t_dur1)-t_ramp1;
				if(t_win1>0)
				{
					t_window1=1;}
				else 
				{ t_window1=0;}
			csi1=(6*pow(z1,5))-(15*pow(z1,4))+(10*pow(z1,3));  //6z^5-15z^4+10z^3
			csi_dot1=(30*pow(z1,4))-(60*pow(z1,3))+(30*pow(z1,2)); //csi_dot=30z^4-60z^3+30z^2
			prod11=(1/(1.0001-(csi1*t_window1)));
			prod21=(csi_dot1*0.3333*t_window1);
			Gamma1=(prod11*prod21);
			return Gamma1;
       };

 double PMPThread::GammaDisc(int _Time)
 {
	double t_ramp=(_Time)*RAMP_KONSTANT; //0.0025 to
double t_init=0.1,z,t_win,t_window,csi,prod1,prod2,Gamma;

z=(t_ramp-t_init)/t_dur;
t_win=(t_init+t_dur)-t_ramp;
	if(t_win>0)
	{
		t_window=1;}
	else 
	{ t_window=0;}
csi=(6*pow(z,5))-(15*pow(z,4))+(10*pow(z,3));  //6z^5-15z^4+10z^3
csi_dot=(30*pow(z,4))-(60*pow(z,3))+(30*pow(z,2)); //csi_dot=30z^4-60z^3+30z^2
//fprintf(wrL,"\n  %f     %f \t  %f",csi,csi_dot);
prod1=(1/(1.0001-(csi*t_window)));
prod2=(csi_dot*0.3333*t_window);
Gamma=prod1*prod2;
return Gamma;
            
 };


double PMPThread::Gamma(int _Time)
{
	double t_ramp=(_Time)*RAMP_KONSTANT; //0.0025 to
	double t_init=0.1,z,t_win,t_window,csi,prod1,prod2,Gamma;

	z=(t_ramp-t_init)/t_dur;
	t_win=(t_init+t_dur)-t_ramp;
	if(t_win>0)
	{
		t_window=1;}
	else 
	{ t_window=0;}
	csi=(6*pow(z,5))-(15*pow(z,4))+(10*pow(z,3));  //6z^5-15z^4+10z^3
	csi_dot=(30*pow(z,4))-(60*pow(z,3))+(30*pow(z,2)); //csi_dot=30z^4-60z^3+30z^2
	//fprintf(wrL,"\n  %f     %f \t  %f",csi,csi_dot);
	prod1=(1/(1.0001-(csi*t_window)));
	prod2=(csi_dot*0.3333*t_window);
	Gamma=prod1*prod2;
	return Gamma;

};

void PMPThread::LoadANN()
{
	
	

		printf("\n loading Neural Network.... \n");
    Weight1=fopen(weights1Path.c_str(),"r");//input file to be given to the program	
  	Weight2=fopen(weights2Path.c_str(),"r");
  	Weight3=fopen(weights3Path.c_str(),"r");
  	bias1=fopen(biases1Path.c_str(),"r");
  	bias2=fopen(biases2Path.c_str(),"r");
  	bias3=fopen(biases3Path.c_str(),"r");
    Weight4=fopen(weights4Path.c_str(),"r");//input file to be given to the program	
  	Weight5=fopen(weights5Path.c_str(),"r");
  	Weight6=fopen(weights6Path.c_str(),"r");
  	bias4=fopen(biases4Path.c_str(),"r");
  	bias5=fopen(biases5Path.c_str(),"r");
  	bias6=fopen(biases6Path.c_str(),"r");



 
}

void PMPThread::CloseANN()
{
	
	printf("\n Closing ANN ... \n");
    fclose(Weight1);//input file to be given to the program	
	fclose(Weight2);
	fclose(Weight3);
	fclose(Weight4);
	fclose(Weight5);
	fclose(Weight6);
	fclose(bias1);
	fclose(bias2);
	fclose(bias3);
	fclose(bias4);
	fclose(bias5);
	fclose(bias6);





 
}


void PMPThread::initializeANN(int ANNswitch)
{

	int m,n;
	inputL=6;
	hiddenL1=32;
	hiddenL2=41;
	outputL=3;
	printf("\n Initializing Neural Network.... \n");
    int u,i;
    float s=0;
    int dec;

   if(ANNswitch==14){///Loading RX
    if(!Weight1)
           { cout << "Error opening W1" << endl; }
    else
           { cout << "Loading W1" << endl;}
		for (m =0; m<hiddenL1; m++)
				{
					for (n=0; n<inputL; n++)
					{
						 fscanf(Weight1,"%f",&s);
						 w1[m][n] = s;
						 
						//  cout << Episodes[m][n] << endl ;
					}
				}

	
    if(!bias1)
           { cout << "Error opening b1" << endl; }
    else
           { cout << "Loading b1" << endl;}
		for (m =0; m<hiddenL1; m++)
				{
					for (n=0; n<1; n++)
					{
						  
						 fscanf(bias1,"%f",&s);
						 b1[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}

		//-------------------------------------------------------

    if(!Weight2)
           { cout << "Error opening W2" << endl; }
    else
           { cout << "Loading W2" << endl;}
		for (m =0; m<hiddenL2; m++)
				{
					for (n=0; n<hiddenL1; n++)
					{
						 //Weight2 >> w2[m][n];
						fscanf(Weight2,"%f",&s);
						 w2[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}

	
    if(!bias2)
           { cout << "Error opening b2" << endl; }
    else
           { cout << "Loading b2" << endl;}
		for (m =0; m<hiddenL2; m++)
				{
					for (n=0; n<1; n++)
					{
						 //bias2 >> b2[m][n];
						 fscanf(bias2,"%f",&s);
						 b2[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}
		//-------------------------------------------------------------------
		   
    if(!Weight3)
           { cout << "Error opening W3" << endl; }
    else
           { cout << "Loading W3" << endl;}
		for (m =0; m<outputL; m++)
				{
					for (n=0; n<hiddenL2; n++)
					{
						 //Weight3 >> w3[m][n];
						fscanf(Weight3,"%f",&s);
						 w3[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}

	
    if(!bias3)
           { cout << "Error opening b3" << endl; }
    else
           { cout << "Loading b3" << endl;}
		for (m =0; m<outputL;m++)
				{
					for (n=0; n<1; n++)
					{
						// bias3 >> b3[m][n
						 fscanf(bias3,"%f",&s);
						 b3[m][n] = s;
					//	 cout << b3[m][n] << endl ;
					}
				}
		cout << b3[0][0] <<"  " << b2[40][0] << "  " << b1[31][0]<< "  " << w3[2][40]<< "  " << w2[40][31]<< "  " << w1[30][5]<< "  " << endl ;
   }

    if(ANNswitch==32){
    if(!Weight4)
           { cout << "Error opening W4" << endl; }
    else
           { cout << "Loading W4" << endl;}
		for (m =0; m<hiddenL1; m++)
				{
					for (n=0; n<inputL; n++)
					{
						 fscanf(Weight4,"%f",&s);
						 w1[m][n] = s;
						 
						//  cout << Episodes[m][n] << endl ;
					}
				}

	
    if(!bias4)
           { cout << "Error opening b4" << endl; }
    else
           { cout << "Loading b4" << endl;}
		for (m =0; m<hiddenL1; m++)
				{
					for (n=0; n<1; n++)
					{
						  
						 fscanf(bias4,"%f",&s);
						 b1[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}

		//-------------------------------------------------------

    if(!Weight5)
           { cout << "Error opening W5" << endl; }
    else
           { cout << "Loading W5" << endl;}
		for (m =0; m<hiddenL2; m++)
				{
					for (n=0; n<hiddenL1; n++)
					{
						 //Weight2 >> w2[m][n];
						fscanf(Weight5,"%f",&s);
						 w2[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}

	
    if(!bias5)
           { cout << "Error opening b5" << endl; }
    else
           { cout << "Loading b5" << endl;}
		for (m =0; m<hiddenL2; m++)
				{
					for (n=0; n<1; n++)
					{
						 //bias2 >> b2[m][n];
						 fscanf(bias5,"%f",&s);
						 b2[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}
		//-------------------------------------------------------------------
		   
    if(!Weight6)
           { cout << "Error opening W6" << endl; }
    else
           { cout << "Loading W6" << endl;}
		for (m =0; m<outputL; m++)
				{
					for (n=0; n<hiddenL2; n++)
					{
						 //Weight3 >> w3[m][n];
						fscanf(Weight6,"%f",&s);
						 w3[m][n] = s;
						//  cout << Episodes[m][n] << endl ;
					}
				}

	
    if(!bias6)
           { cout << "Error opening b6" << endl; }
    else
           { cout << "Loading b6" << endl;}
		for (m =0; m<outputL;m++)
				{
					for (n=0; n<1; n++)
					{
						// bias3 >> b3[m][n
						 fscanf(bias6,"%f",&s);
						 b3[m][n] = s;
					//	 cout << b3[m][n] << endl ;
					}
				}
		cout << b3[0][0] <<"  " << b2[40][0] << "  " << b1[31][0]<< "  " << w3[2][40]<< "  " << w2[40][31]<< "  " << w1[30][5]<< "  " << endl ;
   }
};

void PMPThread::InitializeJan()// u need to enter the right values..
{
	//Jan[0]=0;
	//Jan[1]=0;
	//Jan[2]=0;
	//Jan[3]=0;
	//Jan[4]=0;
	//Jan[5]=0;
	//janini0=0;
	//janini1=0;
	//janini2=0;
	//janini3=0;// -0.6981
	//janini4=0;
	//janini5=0;

	//x_iniIC=455.5;
	//y_iniIC=-743;
	//z_iniIC=2076;
	//x_ini=455.5;
	//y_ini=-743;
	//z_ini=2076;

	        Jan[0]=0.7;
			Jan[1]=0.7;
			Jan[2]=1.39;
			Jan[3]=-1.89;
			Jan[4]=-0.63;
            Jan[5]=0;
			janini0=0.7;
			janini1=0.7;
			janini2=1.39;
			janini3=-1.89;// -0.6981
	        janini4=-0.63;
			janini5=0;
			
    		x_iniIC=35;
			y_iniIC=-25;
			z_iniIC=223;
			x_ini=35;
			y_ini=-25;
			z_ini=223;

};

void PMPThread::InitializeJanObst()// u need to enter the right values..
		  {
            Jan[0]=-1.06;
			Jan[1]=0.95;
			Jan[2]=0.80;
			Jan[3]=0.06;
			Jan[4]=1.65;
            Jan[5]=0;
			janini0=-1.06;
			janini1=0.95;
			janini2=0.80;
			janini3=0.06;// -0.6981
	        janini4=1.65;
			janini5=0;
			
    		x_iniIC=50;
			y_iniIC=1;
			z_iniIC=92;
			x_ini=50;
			y_ini=1;
			z_ini=92;

	 };


  int PMPThread::VTGS(double XT1, double YT2, double ZT3,double XO1, double YO2, double ZO3,int ChoiceAct,int MentalSim, double WristGraspPose)
  {
	  //Kompliance(0);
	  
	  // Stores Output Gamma Function of terminal attractor dynamics
	//  ofstream wr("Gamma.txt");
	  // Output of Target Generator
	  //ofstream wr1("target.txt");
	  //ofstream wr_Gam("resultL.txt");
	   int time;
	   double Gam;
	   double fin[3];
	   int n=3;
	   int retvalue=0;
	   int sizzSecurity=0;
	   int sizz=0;

	   if(ChoiceAct==0){
	    Kompliance(0);
		if((XT1==0)&&(YT2==0)&&(ZT3==0)) //note this is for iCub , for iCubSim we need to divide by 1000
		{
			XT1=482;
			YT2=102;
			ZT3=200;
		}

		fin[0]=XT1; //Final Position X
		fin[1]=YT2;
		fin[2]=ZT3;
		/*if (robCmd == 32){ //removed offset for RX
			fin[2]=fin[2]+60;
		}*/
		double xoffs=0,yoffs=0,zoffs=0;
		int replan=0;
	   	
		x_fin=fin[0]; //Final Position X
		y_fin=fin[1];
		z_fin=fin[2];

    printf("\n Targets");
	printf("\n \n %f, \t  %f, \t %f\n\n ",x_fin,y_fin,z_fin);
	
    for(time=0;time<ITERATION;time++) // 2000 incremental steps of delta 0.005
		{
     
			Gam=GammaDisc(time);
        //	 wr << time << "    " << Gam << endl; 

		//  ====================Target Generation //=========================

			double inter_x=(x_fin-x_ini)*Gam;
			Gam_Arr[time]=inter_x;
			double *Gar=Gam_Arr;
			x_ini=Gamma_IntDisc(Gar,time)+x_iniIC;
			
			double inter_y=(y_fin-y_ini)*Gam;
			Gam_Arry[time]=inter_y;
			double *Gary=Gam_Arry;
			y_ini=Gamma_IntDisc(Gary,time)+y_iniIC;
			
			double inter_z=(z_fin-z_ini)*Gam;
			Gam_Arrz[time]=inter_z;
			double *Garz=Gam_Arrz;
			z_ini=Gamma_IntDisc(Garz,time)+z_iniIC;
			
//			wr1 << x_ini << "    " << y_ini << "    " << z_ini << endl;
				
			MotCon(x_ini,y_ini,z_ini,time,Gam);
//			wr_Gam << Jan[0] << "  " << Jan[1]<< "  " << Jan[2]<< "  " << Jan[3]<< "  " << Jan[4]<< "  " << Jan[5]<< "  " << Jan[6] <<endl;
   			
	}
			konst=(180/3.14159);
			ang1=konst*Jan[0];
			ang2=konst*Jan[1];
			ang3=konst*Jan[2];
			ang4=konst*Jan[3];
			ang5=konst*Jan[4];
			//ang5=90;
			//ang6=ComputeTheta5(VTGSIN[6],VTGSIN[7],VTGSIN[8],VTGSIN[9]) ; //should be modifed to ComputeTheta5(VTGSIN[0],VTGSIN[0],VTGSIN[0],VTGSIN[0])
			ang6=konst*Jan[5];
			printf("\n  %f, \t  %f, \t %f ,\t %f ,\t %f ,\t %f",ang1,ang2,ang3,ang4,ang5,ang6);
			printf("\n\n FINAL SOLUTION  %f, \t  %f, \t %f \t ",X_pos[0],X_pos[1],X_pos[2]);
		   
			if((sqrt(pow(X_pos[0]-fin[0],2)+ pow(X_pos[1]-fin[1],2)+ pow(X_pos[2]-fin[2],2))<=40))
				{
				   sizz=1;
				}
          
			if(sizz==0)
				{
				 cout << "Test Test Test" << endl;
				}	
	}

		if(ChoiceAct==1)
		  {
			  //Imitation system to synthesize moving point attractors: Future work
			  InitializeJanObst();						 
						  //Imitation system to synthesize moving point attractors: Future work

					  if((XT1==0)&&(YT2==0)&&(ZT3==0)) //note this is for iCub , for iCubSim we need to divide by 1000
						{
						 XT1=482;
						 YT2=102;
						 ZT3=305;
						 XO1=482;
						 YO2=102;
						 ZO3=305;
						}
						janini0=Jan[0];
						janini1=Jan[1];
						janini2=Jan[2];//-0.9425
						janini3=Jan[3];
						janini4=Jan[4];
						janini5=Jan[5];  //CHECK THE INITIALIZATION
						x_iniIC=x_ini;
						y_iniIC=y_ini;
						z_iniIC=z_ini;


					double xoffs=0,yoffs=0,zoffs=0;
					int replan=0;
					x_fin1=XT1; //Final Position X
					y_fin1=YT2;
					z_fin1=ZT3;
					x_fin2=XO1; //Final Position X
					y_fin2=YO2;
					z_fin2=ZO3;
					  printf("\n Targets");
					  printf("\n \n %f, \t  %f, \t %f \t %f, \t  %f, \t %f ",x_fin1,y_fin1,z_fin1,x_fin2,y_fin2,z_fin2);

					  for(time=0;time<4000;time++) // 2000 incremental steps of delta 0.005
						{
					//KXA=1;	KXB=1; 	KYA=1; 	KYB=1; 	TSEC=1500; STARTini=1;dividen=300;
					KXA=1;	KXB=10; 	KYA=1; 	KYB=10; TSEC=1000;STARTini=2; dividen=300; // bump
					//KXA=10;	KXB=1; 	KYA=1; 	KYB=10; TSEC=1000; STARTini=1; dividen=300;//cusp
				double GamA,GamB;
				GamA=Gamma(time);
				GamB=Gamma1(time-TSEC);
				if(time<=TSEC)
				 {
				  GamB=0;
				  csi_dot1=0;
				 } 
				if(time>=1500)
				 {
				  GamA=0;
				  csi_dot=0;
				 }
				if(time>=2500)
				 {
				  csi_dot1=0;
				 }
				   Gam=GamA+GamB;		
//				   wr << time << "    " << Gam << endl; 
					//fprintf(wr,"\n  %d \t  %f \t  %f \t  %f \t  %f \t  %f",time,GamA,GamB,Gam,csi_dot,csi_dot1); /// v.txt has values of Gamma and time
			   
				double inter_x1=((x_fin1-x_ini)*GamA);
				double inter_x2=((x_fin2-x_ini)*GamB);
				Gam_Arr1[time]=inter_x1;
				Gam_Arr2[time]=inter_x2;
				double *Gar1=Gam_Arr1;
				double *Gar2=Gam_Arr2;
 				x_ini=KXA*Gamma_Int(Gar1,time)+ KXB*Gamma_Int(Gar2,time)+x_iniIC;
				
				double inter_y1=((y_fin1-y_ini)*GamA);
				double inter_y2=((y_fin2-y_ini)*GamB);
				Gam_Arry1[time]=inter_y1;
				Gam_Arry2[time]=inter_y2;
				double *Gary1=Gam_Arry1;
				double *Gary2=Gam_Arry2;
				y_ini=KYA*Gamma_Int(Gary1,time)+KYB*Gamma_Int(Gary2,time)+y_iniIC;
					
				double inter_z1=((z_fin1-z_ini)*GamA);
				double inter_z2=((z_fin2-z_ini)*GamB);
				Gam_Arrz1[time]=inter_z1;
				Gam_Arrz2[time]=inter_z2;
				double *Garz1=Gam_Arrz1;
				double *Garz2=Gam_Arrz2;
				z_ini=KYA*Gamma_Int(Garz1,time)+KYB*Gamma_Int(Garz2,time)+z_iniIC;

				MotCon(x_ini,y_ini,z_ini,time,Gam);
				if(((X_pos[0]>0)&&(X_pos[0]<320))&&((X_pos[1]>0)&&(X_pos[1]<210))&&((X_pos[2]>90)&&(X_pos[2]<360)))
					{
						//posiL << X_pos[0] << "    " << X_pos[1] << "    " << X_pos[2] << endl;
						//wr1 << x_ini << "    " << y_ini << "    " << z_ini << " " <<endl;
					}

   				   }

		  }

    //wr.close();
    return sizz; 
  };

//double PMPThread::ComputeTheta5(double Fx, double Fy,double Bx, double By)
//{
	

  double PMPThread::ComputeTheta5(double Fx, double Fy,double Bx, double By)
	  {
       double XGrip=(((((1*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*sin(Jan[3]))*cos(Jan[4])+(0.000003*(((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.000003*(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*cos(Jan[3])-1.00000*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+1.00000*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.000003*sin(Jan[0])+0.00000002*cos(Jan[0]))*sin(Jan[4]))*cos(Jan[5])+(0.000003*((((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*sin(Jan[3]))*sin(Jan[4])-0.000003*(0.000003*(((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.000003*(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*cos(Jan[3])-1.00000*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+1.00000*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.000003*sin(Jan[0])+0.00000002*cos(Jan[0]))*cos(Jan[4])-1.00000*(((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])+1.00000*(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*cos(Jan[3])-0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.0*sin(Jan[0])+0.0*cos(Jan[0]))*sin(Jan[5])-409.679+269.500*((((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*sin(Jan[3]))*sin(Jan[4])-269.500*(0.000003*(((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.000003*(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*cos(Jan[3])-1.00000*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+1.00000*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.000003*sin(Jan[0])+0.00000002*cos(Jan[0]))*cos(Jan[4])+0.0006*(((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.0006*(-0.0+0.000003*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-0.999*sin(Jan[0])+0.0077*cos(Jan[0]))*cos(Jan[3])+550.000*((0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*cos(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-550.000*(-1.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])+(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])-49.6083*sin(Jan[0])+50.3852*cos(Jan[0])+500.0*(0.999*cos(Jan[0])+0.0077*sin(Jan[0]))*sin(Jan[1])-500.0*(-0.0062+0.000003*sin(Jan[0])-0.00000002*cos(Jan[0]))*cos(Jan[1]);
       double YGrip=(((((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*sin(Jan[3]))*cos(Jan[4])+(0.000003*(((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.000003*(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*cos(Jan[3])-1.00000*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+1.00000*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0*sin(Jan[0])+0.000003*cos(Jan[0]))*sin(Jan[4]))*cos(Jan[5])+(0.000003*((((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*sin(Jan[3]))*sin(Jan[4])-0.000003*(0.000003*(((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.000003*(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*cos(Jan[3])-1.00000*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+1.00000*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0*sin(Jan[0])+0.000003*cos(Jan[0]))*cos(Jan[4])-1.00000*(((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])+1.00000*(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*cos(Jan[3])-0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0*sin(Jan[0])+0*cos(Jan[0]))*sin(Jan[5])+799.627+269.500*((((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*cos(Jan[3])+(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*sin(Jan[3]))*sin(Jan[4])-269.500*(0.000003*(((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.000003*(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*cos(Jan[3])-1.00000*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])+1.00000*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0*sin(Jan[0])+0.000003*cos(Jan[0]))*cos(Jan[4])+0.0006*(((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*cos(Jan[2])+(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*sin(Jan[2]))*sin(Jan[3])-0.0006*(0.0+0.000003*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-0.000003*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+0.0077*sin(Jan[0])+1.0*cos(Jan[0]))*cos(Jan[3])+550.000*((-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*cos(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*sin(Jan[1]))*sin(Jan[2])-550.000*(-1.0*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])+(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]))*cos(Jan[2])+50.3856*sin(Jan[0])+49.6098*cos(Jan[0])+500*(-0.0077*cos(Jan[0])+1.0*sin(Jan[0]))*sin(Jan[1])-500*(0.0017-0.0*sin(Jan[0])-0.000003*cos(Jan[0]))*cos(Jan[1]);
// Current Gripper orientation: Red Line 
	   double magTricky, dotTricky, GripTricky,DP,MP;
       magTricky=sqrt(pow(Bx-Fx,2)+pow(By-Fy,2)); //Blue line magnitide
       GripTricky=sqrt(pow(XGrip-X_pos[0],2)+pow(YGrip-X_pos[1],2));//Red line magnitide
	   MP=magTricky*GripTricky;
       DP=((Bx-Fx)*(XGrip-X_pos[0]))+ ((By-Fy)*(YGrip-X_pos[1]));
	   dotTricky=(acos(DP/MP))*57.2958; 

	cout<<"MAG TRICKY IS "<<magTricky<<std::endl;

	cout<<"DOT TRICKY IS "<<dotTricky<<std::endl;

	if(dotTricky * 1 == dotTricky)
	{
		dotTricky = dotTricky;
	}
	else
	{
		dotTricky = 0;
	}

	return dotTricky;
};

  void PMPThread::VariableSpeedCtrl(int roboname) {
		 if(roboname==32)
				{

			//DELETE This commented code
			///////////////////////////////////////////////////////////////////////////
					//BufferedPort<JointsContext>* MotComRX = new BufferedPort<JointsContext>();

					////Test Port – create one for each TX and RX
					//MotComRX->open("/vRX:o");
					//
					//if (Network::exists("/robotcontroller/jointsRX:i"))			
					//Network::connect("/vRX:o", "/robotcontroller/jointsRX:i");  //check this
			/////////////////////////////////////////////////////////////////////////
					movingjointsRX.clear();
					movingjointsRX.push_back(ang1);
					movingjointsRX.push_back(ang2);
					movingjointsRX.push_back(ang3);
					movingjointsRX.push_back(ang4);
					movingjointsRX.push_back(ang5);
					movingjointsRX.push_back(ang6);

					JointsContext &testwrite = MotComRX.prepare();      

					//Set context here
					//320 		Reaching fuse
					//410 		Reaching Fusebox
					//500 		Inserting fuse
					//680 		Initializing

					switch(Context)
					{
					case 320:
						testwrite.setContext(CONTEXT_RFUSE);
						break;

					case 410: 
						testwrite.setContext(CONTEXT_RBOX);
						break;

					case 500:
						testwrite.setContext(CONTEXT_INSERT);
						break;

					case 680:
						testwrite.setContext(CONTEXT_INIT);
						break;

					default:
						testwrite.setContext(CONTEXT_INIT);
						break;
					}

					

					//Do not forget to clear the joint angles before filling them
						testwrite.JointAngles().clear();

						testwrite.JointAngles().add(ang1);
						testwrite.JointAngles().add(ang2);
						testwrite.JointAngles().add(ang3);
						testwrite.JointAngles().add(ang4);
						testwrite.JointAngles().add(ang5);
						testwrite.JointAngles().add(ang6);
						MotComRX.write(true);
						Time::delay(0.1);
		
				}

		if(roboname==14)
		{ 
			
			//DELETE This commented code
			///////////////////////////////////////////////////////////////////////////
			//BufferedPort<JointsContext>* MotComTX = new BufferedPort<JointsContext>();
					//Test Port – create one for each TX and RX
					//MotComTX->open("/vTX:o");
					//if (Network::exists("/robotcontroller/jointsTX:i"))			
					//	Network::connect("/vTX:o", "/robotcontroller/jointsTX:i");  //check this
			////////////////////////////////////////////////////////////////////////////////////////
					movingjointsTX.clear();
					movingjointsTX.push_back(ang1);
					movingjointsTX.push_back(ang2);
					movingjointsTX.push_back(ang3);
					movingjointsTX.push_back(ang4);
					movingjointsTX.push_back(ang5);
					movingjointsTX.push_back(ang6);


						JointsContext &testwrite = MotComTX.prepare();      

					//Set context here
						
					//Set context here
					//320 		Reaching fuse
					//410 		Reaching Fusebox
					//500 		Inserting fuse
					//680 		Initializing

					switch(Context)
					{
					case 320:
						testwrite.setContext(CONTEXT_RFUSE);
						break;

					case 410: 
						testwrite.setContext(CONTEXT_RBOX);
						break;

					case 500:
						testwrite.setContext(CONTEXT_INSERT);
						break;

					case 680:
						testwrite.setContext(CONTEXT_INIT);
						break;

					default:
						testwrite.setContext(CONTEXT_INIT);
						break;
					}


					//Do not forget to clear the joint angles before filling them
						testwrite.JointAngles().clear();

						testwrite.JointAngles().add(ang1);
						testwrite.JointAngles().add(ang2);
						testwrite.JointAngles().add(ang3);
						testwrite.JointAngles().add(ang4);
						testwrite.JointAngles().add(ang5);
						testwrite.JointAngles().add(ang6);
						MotComTX.write(true);
						Time::delay(0.1);
		
			
		}
};


void PMPThread::MessagePassR(int roboname) {

	if (MotCom.getOutputCount()) { 
		Bottle& B = MotCom.prepare();	
		B.clear();
		if(roboname==14)
		{
			B.addDouble(5);
			B.addDouble(-14);
			B.addDouble(90);
			B.addDouble(0);
			B.addDouble(90);
			B.addDouble(-23);
			B.addDouble(ang1);
			B.addDouble(ang2);
			B.addDouble(ang3);
			B.addDouble(ang4);
			B.addDouble(ang5);
			B.addDouble(ang6);

            movingjoints.clear();
			movingjoints.push_back(5);
			movingjoints.push_back(-14);
			movingjoints.push_back(90);
			movingjoints.push_back(0);
			movingjoints.push_back(90);
			movingjoints.push_back(-23);
			movingjoints.push_back(ang1);
			movingjoints.push_back(ang2);
			movingjoints.push_back(ang3);
			movingjoints.push_back(ang4);
			movingjoints.push_back(ang5);
			movingjoints.push_back(ang6);
		}
		if(roboname==32)
		{
			B.addDouble(ang1);
			B.addDouble(ang2);
			B.addDouble(ang3);
			B.addDouble(ang4);
			B.addDouble(ang5);
			B.addDouble(ang6);	
			B.addDouble(0);
			B.addDouble(0);
			B.addDouble(0);
			B.addDouble(0);
			B.addDouble(0);
			B.addDouble(0);
			//movingjoints.resize(12);
			movingjoints.clear();
			//for(int i=0;i<12;i++)
			//{
			//	movingjoints[i] = 0;
			//}
			movingjoints.push_back(ang1);
			movingjoints.push_back(ang2);
			movingjoints.push_back(ang3);
			movingjoints.push_back(ang4);
			movingjoints.push_back(ang5);
			movingjoints.push_back(ang6);
			movingjoints.push_back(0);
			movingjoints.push_back(0);
			movingjoints.push_back(0);
			movingjoints.push_back(0);
			movingjoints.push_back(0);
			movingjoints.push_back(0);
//
//movingjoints.push_back(ang1);
//			movingjoints.push_back(ang1);
//			movingjoints[0] = ang1;
//			movingjoints[1] = ang2;
//			movingjoints[2] = ang3;
//			movingjoints[3] = ang4;
//			movingjoints[4] = ang5;
//			movingjoints[5] = ang6;
//			movingjoints[6] = 0;
//			movingjoints[7] = 0;
//			movingjoints[8] = 0;
//			movingjoints[9] = 0;
//			movingjoints[10] = 0;
//			movingjoints[11] = 0;
			}
		/*movingjoints.resize(12);
		movingjoints.clear();
		for(int i=0;i<6;i++)
		{
			movingjoints[i] = 0;
		}

		movingjoints[6] = ang1;
		movingjoints[7] = ang2;
		movingjoints[8] = ang3;
		movingjoints[9] = ang4;
		movingjoints[10] = ang5;
		movingjoints[11] = ang6;

		std::cout<<std::endl<<"Contents of Moving joints are: "<<std::endl<<std::endl;

		for(int i=0;i<12;i++)
		{
			std::cout<<movingjoints[i]<<" , "; 
		}*/
		std::cout<<std::endl<<std::endl<<" WRiting these ::: "<<B.toString().c_str()<<std::endl<<std::endl;
		MotCom.write();
	}
};



void PMPThread::MotCon(double T1, double T2, double T3, int time, double Gam)

{
	double *ang = Jan;
	int len=1,i;

	//===============================================================

	double *nFK = forward_Kinematics(ang,len); // Joint Angles to Positions 3>>>>>2

	for(i=0;i<3;i++)
	{
		X_pos[i]=*(nFK+i);
	}

	//	posiL << X_pos[0] << "  " << X_pos[1] << "  " << X_pos[2] << "  " << endl;
	//cout << X_pos[0] << "  " << X_pos[1] << "  " << X_pos[2] << "  " << endl;
	double *po=X_pos;

	//===================================================================
	target[0]=T1;
	target[1]=T2;
	target[2]=T3;

	double *ta=target;
	double *force=forcefield(po,ta); // Position to Force 3>>>>>3
	for(i=0;i<3;i++)
	{
		ffield[i]=*(force+i);
	}
	double *topmp= ffield;
	//===================================================================
	//                  PASSIVE MOTION PARADIGM
	//===================================================================
	double *Q_Dot=PMPJack(topmp);  //Force to Torque to Q_dot 2>>>>>3
	//===================================================================

	for(i=0;i<6;i++)          //MODIFY
	{
		JoVel[i]=(*(Q_Dot+i))*Gam;
	}

	// From Q_dots to Q >>>>>
	q1[time]=JoVel[0];
	double *j1=q1;
	double joi1=Gamma_IntDisc(j1,time);
	Jan[0]=joi1+janini0;

	q2[time]=JoVel[1];
	double *j2=q2;
	double joi2=Gamma_IntDisc(j2,time);
	Jan[1]=joi2+janini1;


	q3[time]=JoVel[2];
	double *j3=q3;
	double joi3=Gamma_IntDisc(j3,time);
	Jan[2]=joi3+janini2;

	q4[time]=JoVel[3];
	double *j4=q4;
	double joi4=Gamma_IntDisc(j4,time);
	Jan[3]=joi4+janini3;

	q5[time]=JoVel[4];
	double *j5=q5;
	double joi5=Gamma_IntDisc(j5,time);
	Jan[4]=joi5+janini4;

	q6[time]=JoVel[5];
	double *j6=q6;
	double joi6=Gamma_IntDisc(j6,time);
	Jan[5]=joi6+janini5;

}



 void PMPThread::Kompliance(int TagK)
 {
   //  if (TagK==0)
		 //{
			//printf ("Adjusting Compliances 1 \n");
   // 		KFORCE=0.14;  //was 0.07 earlier
			//ITERATION=1000;              
			//RAMP_KONSTANT=0.005;
			//t_dur=5;
			//KOMP_JANG=0.0001;
			//J2H=0.1; 
			//printf ("\n Initiating System Dynamics \n");
		 //} 

	 if (TagK==0)
		 {
			printf ("Adjusting Compliances 1 \n");
    		KFORCE=0.005;
			ITERATION=1000;              
			RAMP_KONSTANT=0.005;
			t_dur=5;
			KOMP_JANG=0.0001; //0.0001
			J2H=0.01; 
			printf ("\n Initiating System Dynamics \n");
		 } 
 
 };
 
 int PMPThread::FindSmoothPath(std::vector<float>* PreviousPosition, std::vector<std::vector<float>>* NextPosition)
{

	// The Location of joint angles with minimum error
	int index = -1;
	float minerror = 999999;

	for(int next = 0; next < NextPosition->size(); next++)
	{		
		float testerror = 0;

			for(int posin = 0; posin < PreviousPosition->size(); posin++)
			{

				testerror += pow((PreviousPosition->at(posin) - NextPosition->at(next).at(posin)),2) ;

			}

			if(testerror < minerror)
			{
				index = next;

				minerror = testerror;
			}
			
	}

	return index;
}
 		/// @brief Get the vector that represents the position of all the joints of the robot w.r.t world coordinate frame
		/// 
		///
		/// @param joints joint angles of the robot
		/// @param robot - for a specific robot TX == 1 and RX ==2
		/// @return Returns the transformation from the origin of the world coordinate system to the 
		///         robot flange (according to specified joint angles).
 std::vector<LibRoboMove::Kinematics::vecType> PMPThread::getBodyConfig(std::vector<float> joints, int robot)
 { 
	 
	float konst=(180/3.14159);
	
	if(joints.size() == 6 )
	{
		for(int i=0;i<joints.size();i++)
		{
			joints[i] = joints[i]/konst;
		}
	}


	 LibRoboMove::Kinematics::matType m_transformCheckerboardToRobotBase;
	 LibRoboMove::Kinematics::matType m_inverseTransform;
	 LibRoboMove::KinematicsRX130 m_kRX;
	 LibRoboMove::KinematicsTX90 m_kTX;


	 ///initializing values for TX 
	if(robot == 1)
	{
		
		m_kTX = LibRoboMove::KinematicsTX90(TX90L, 2.38509, -2.19225, 220);
		m_transformCheckerboardToRobotBase.set(0.9998, -0.0154, 0.0095, 0.0, 0.0155, 0.9998, -0.00925, 0.0, -0.0094, 0.0094, 0.9999, 0.0, 554.67, -678.41, 352.18, 1.0); // Calibration 29-07-2014 - TX
		m_inverseTransform.set(0.998, 0.0155, -0.0094, 0.0, -0.0154, 0.9998, 0.0094, 0.0,  0.0095, -0.0093, 0.9999, 0.0, -568.3946, 672.9615, -340.5584, 1.000); // Inverse transformation of TX 29-07-2014
	}


		 ///initializing values for RX 
	else if(robot == 2)
	{
		
		m_kRX = LibRoboMove::KinematicsRX130(RX130B, 0, 0, 284);
		m_transformCheckerboardToRobotBase.set(0.99996, -0.0084, 0.0030, 0.0,  0.0084, 0.99996,  0.0026, 0.0, -0.0030, -0.0026, 0.99999, 0.0, 554.4600, 905.95, 352.48, 1.0);
		m_inverseTransform.set(0.99996, 0.0084, -0.0029, 0.0, -0.0084, 0.99996, -0.0026, 0.0, 0.0029, 0.0026, 0.99999, 0.0, -547.87, -911.49, -348.45, 1.0);
	}

	std::vector<LibRoboMove::Kinematics::vecType> positions;
	positions.resize(joints.size());

	for(int i=0;i<joints.size();i++)
	{

		LibRoboMove::Kinematics::vecType temp;
		if(robot ==1)
		{
			/// Return the position w.r.t robot base coordinate frame of TX90
			temp = m_kTX.JointAnglesToCartesianPerJoint(joints,i);
		}

		else
		{
			/// Return the position w.r.t robot base coordinate frame of RX130
			temp = m_kRX.JointAnglesToCartesianPerJoint(joints,i);
		}
		
		/// Results in position w.r.t world coordinate frame
		LibRoboMove::Kinematics::vecType temp_1 = temp*m_inverseTransform;
		positions[i] = temp_1;
	}

		return positions;

 }