
#include <ObserverThread.h>
#include <cstring>
#include <string>
#include<time.h>
#include <math.h>
#include <windows.h>
#include <iostream>
#include <fstream>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

using namespace darwin::msg;

ObserverThread::ObserverThread() {
    robot = "icub";
}

ObserverThread::ObserverThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

ObserverThread::~ObserverThread() {
    // do nothing
}

bool ObserverThread::threadInit() {

	if (!QueryEPIM.open(getName("/what-to-do:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!PlanEx.open(getName("/Strategy:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!EpimCtrlPort.open(getName("/EpimCtrl:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!UserServices.open(getName("/UserServer:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!OPCCtrlPort.open(getName("/SmallWorldsOPC:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!BodySchemaCtrlPort.open(getName("/BodySchemaSim:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!GraspPort.open(getName("/GraspCtrl:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!bodyPlot.open(getName("/bodyPlot:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!objectPlot.open(getName("/objectPlot:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!actionPlot.open(getName("/actionPlot:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!cwsPlot.open(getName("/cwsPlot:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!alignPlot.open(getName("/alignPlot:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	if (!commandRobot.open(getName("/commandRobot:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

	InitializeSW();
	 //Network::connect("/hubPlot:o", "/emPlotter/observerHub:i");
	PrimPushSideFlag=0;

	fileName = pathPrefix;
	fileName.append("Report.txt");
	Report.open(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("HubA.txt");
	HubA.open(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("MapA.txt");
	MapA.open(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("AcInitActiv.txt");
	WorABin.open(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("ThePresent.txt");
	Present.open(fileName.c_str());
	 return true;
}

void ObserverThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string ObserverThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void ObserverThread::setPath(string inS) {
    pathPrefix	= inS.c_str();
}

void ObserverThread::setInputPortName(string InpPort) {

}

void ObserverThread::run()
{
	KeySend=0;
	int AckMesEp=0;
	//state = 1; //VM made state as 2 for checking with TX190L 14 feb 2013
   state = 0;
	while (isStopping() != true) {

		switch(state) {
			case 0 : {         //Now retrocatives the small world: with learnt proto language: Tallest stack is retained/integrated :)
						int iCo;
						cout<<" Waiting for user goal " <<endl;
						Report<<" Waiting for user goal " <<endl;
						GContext=50;
						HubID_EPIM=50;
						GoalPointer=0;
						iterMicro=0;
						iterMicroN=0;
						TerminateFlag=50;
						RootGoalFlag=14;
						MergePlans=0;
						int interfaceRes=UserInterface(14);
						// to test communication with prim body schema industrial
						//int PMPIT=MicroMonitor(0);
						//==============================================
						if(interfaceRes==1)
							{
								  PrimBodySchema(19,0,1,1,0,0);
								  initGR();
								  initGL();  
								  RefreshPlacemap();
								  cout<<" From User goal to anticipated Neural Hub activations " <<endl;
							      GetLocalAct(NumWords);
								  int PropWeightC=1; // this can come as a result of the elimination/growth rule
								  Retroactivate(PropWeightC);
								  cout<<"provH"<<ProvHub[0]<<endl;
								  cout<<ProvHub[20]<<endl;
								  //RetroactivateBodyHub(1);  //not needed here will become active based on failiures
                                  cout<<" Initializing DARWIN Working memory " <<endl;
								  InitializeWorkingMemory(GoalPointer);
								  if(Goal_AcID==4)
									  {
								         cout<<"Enter the Object on which to Place " <<endl;
								         LoadGWSArgument();
									  }
								  GContext=14;
								  Replan=0;
								  pointIntersect=0; // this helps thread/plan binding in time
								  //change
								  if(Goal_AcID==3){
								  UGPush=1;
								  }
								  //change
								  
								  //////////////////////////////////////////////////////////////////////////////////////////
								 // GWSPtr=0; // this is not needed
								//  PrimPush(0);
							}

						GiD=100;  //made this 100 to divert the loop
						// This is to close the loop without micromonitoring.........................
						// testing pushing stand alone

						//PosHandOcc[0]=55;
					 //   PosHandOcc[1]=289;
						//PosHandOcc[2]=55;
						//PlaceMapPos=0;
						//ObjIDEE[PlaceMapPos]=100;
						////PrimPush(-90,110); 
												
						state=1; // was state 1, made to 0 for testing opc vision loop
						// This is to close the loop without micromonitoring.........................

						//cin >> GiD; //not distrubing this for the time being
						if(GiD==14){
							Replan=0;
							//RefreshPlacemap(); // need to Switch this on so as to initialize both the Global workspace and Placemap
							//	state=1; // commented this but u need to take actions: Very OK!
						}

						if(GiD==32)
							  {
									NPiCs=1;
									Replan=0;
									for(iCo=0; iCo<10; iCo++)
										{
										  SeqAcP[iCo]=50;
										}
									RefreshPlacemap(); // commented for testing 10-03-2014 ********************************
									for(iCo=0; iCo<NumberofObs; iCo++)
										{
										  SeqAcP[iCo]=ObjIDEE[iCo];
										  cout<<"Oject id is "<<SeqAcP[iCo]<<endl;
										  NPiCs=NPiCs+1;
										}
										state = 5;
								}
			}
			break;


			case 1: {
						 Network::connect("/EpimCtrl:io", "/strategy:io");
						   for(int i=0;i<1000;i++)
							  {
								 Strata[i]= 0;
							  }
							cout << "Sending snapshot to EPIM "<< endl;
							Report << "Sending snapshot to EPIM "<< endl;

			// for tests with Xlate and Inv-Xlate// this should come from Refresh Place Map that issues OPC the microgoal to FORTH

					    for(int i=0;i<10;i++)
							{
								ObjIDEE[i]=50; //null object
								XlatTrackP[i]=50;
								XlatTrackPl[i]=50;
							}
					if(Replan==0){
					   /* NumberofObs=3;
						NumberofObsE=2*/;

						NumberofObsE=NumberofObs;
                        largeness=0;
      //                  ObjIDEE[0]=5; //mush
	     //				ObjIDEE[1]=0;
						//ObjIDEE[2]=3; //cyli

						for(int i=0;i<NumberofObs;i++)
							{
								XlatTrackP[i]=ObjIDEE[i];
								XlatTrackPl[i]=ObjIDEE[i];
						    }
					//	Xlator(); //*******************************************************************************
						}

						if(Replan==1){
							cout<<"Micromonitoring Failed: Transimiting Body Hub activations to the Episodic memory"<<endl;
							NumberofObsE=42;
							largeness=0;
							GContext=50;
							HubID_EPIM=2;
						}

						if(Replan==140){
							cout<<"Transimiting Object Hub activations to the Episodic memory"<<endl;
							NumberofObsE=42;
							largeness=0;
							GContext=50;
							HubID_EPIM=1;
						} 
						//==========================================================================================
						Bottle cmd, response;
						cmd.addVocab(COMMAND_VOCAB_REQ);
						cmd.addInt(GContext);
						cmd.addInt(HubID_EPIM);
						cmd.addInt(Goal_AcID+1);
						cout<<"Goal Context  "<<GContext << "with Hub Query  "<< HubID_EPIM << "and Action Hubactivation" <<Goal_AcID+1<< endl;

                        cmd.addDouble(NumberofObsE);
						cmd.addInt(largeness);
						cmd.addInt(Replan);
						for(int i=0;i<NumberofObsE;i++)
							{
								if(Replan==0)
									{
									  cmd.addDouble(ObjIDEEEpim[i]); //this should be replaced with ObjIDEE[i] coming from OPC server
									}
								if(Replan==1)
									{
										cmd.addDouble(BodyHub[i]);
									}
								if(Replan==140)
									{
										cmd.addDouble(OCHub[i]); 
									}
							}

						EpimCtrlPort.write(cmd,response);
						printf("%s \n",response.toString().c_str());

					int responsecode = response.get(0).asVocab();
					cout<<responsecode<<endl;

					if(responsecode == 123 /*COMMAND_VOCAB_ACK*/) {
						fileName = pathPrefix;
						fileName.append("PXper.txt");
						ofstream PXper(fileName.c_str());
						 cout << "Receiving Plan from server" << endl;
						 Report <<"Receiving Plan from server" << endl;
						 for(int i=0;i<1000;i++)
							{
							 Strata[i]= response.get(i+1).asInt();
							 PXper << Strata[i] <<endl;
							}
						 pointRew=response.get(1001).asInt();
						 cout<<"Point of Chunk Termination is"<<pointRew<<endl;
						 cout << "Minimal energy Plan or MemoChunk recd sucessfully: Requesting EPIM to wait for next event" << endl;
						 Report << "Minimal energy Plan or MemoChunk recd sucessfully: Requesting EPIM to wait for next event" << endl;

						 state=6;
					}
						if(responsecode == 41 /*COMMAND_VOCAB_ACK*/) {
						 cout << "No relevant past experience exists in the Neural Episodic memory: Triggering exploration" << endl;
						 Report << "No relevant past experience exists in the Neural Episodic memory: Triggering exploration" << endl;
						 state=10;
					}
			}
			break;

			case 2:
				{
					int iCo,jCo;
					int Pctrr=0;
					int AcSeq[20][50];
					for(iCo=0; iCo<6; iCo++){
					  SeqAc[iCo]=50;
					  }
					for(iCo=0; iCo<10; iCo++){
					  SeqAcP[iCo]=50;
					//  PastPlan[iCo]=50;
					  }
					//===================================================
					fileName = pathPrefix;
					fileName.append("PXper.txt");
					ifstream PlnW(fileName.c_str());
					if(!PlnW)
						   { cout << "Error opening plan" << endl; }
					else
						   { cout << "Loading plan" << endl;
					         Report<< "Loading plan" << endl;}
					for (iCo=0; iCo<1000; iCo++)
						   {
							PlnW >> Strata[iCo];
						   }
					//==============Just for testing, this will be usally fed from EPIM=======
					for(iCo=0; iCo<20; iCo++)
							 {
								for(jCo=0; jCo<50; jCo++)
										{
							  				 AcSeq[iCo][jCo]=Strata[Pctrr];
											 Pctrr=Pctrr+1;
										 }
							 }

					int NObjs=0;
                    int ctr=1;
					for(iCo=0; iCo<20; iCo++)
							 {
                                if(AcSeq[iCo][42]==1)
                                  {
									    NObjs=NObjs+1;
										for(jCo=0; jCo<36; jCo++)
												{
							  						if(AcSeq[iCo][jCo]==1)
														{
														 SeqAc[ctr]=jCo;
														 cout << "found object" << SeqAc[ctr] << endl;
														 ctr=ctr+1;
														}
												}
									}
							 }
					  SeqAc[0]=NObjs;
					  cout<< "there are " << SeqAc[0] << "object shapes to stack: Interpreting.." << endl;
                      Interpret();  // Interprets plan  in shapes to plan in FORTH representations
					   if(Replan==0){
							 state = 5;
						 }
					    if(Replan==1){
							 state = 4;
						 }
				}
			break;

			case 4 : {
			  cout<<"Merging the incompletely executed past plan with the new one from EPIM..."<<endl;
			  Report<<"Merging the incompletely executed past plan with the new one from EPIM..."<<endl;
              Mergence();
			  cout<<"Mergence synthesized a posisble plan"<<endl;
			  Report<<"Mergence synthesized a posisble plan"<<endl;
			  state=5;
			}
			break;

			case 11 : {
			  cout<<"Exploring possibilities with Objects: Sending Object Hub activations to EPIM"<<endl;
			  Report<<"Exploring possibilities with Objects: Sending Object Hub activations to EPIM"<<endl;
              Replan=140;
			  OCHub[32]=1;
			  state=1;
			  UGPush=0;
			  PushIntersect=14;
			  //this seems to work:Now u need to Micromonitor/Execute this properly....
			}
			break;

			case 5:
				  {
					  int iCo,jCo;
					  int StakSuc=0;
					  if(Replan==1){
					  Replan=0; //when ever it enters pick and place loop there are chances of failing and replanning
					  //so this is implemented such that the system calls itself recurrsively ..... VM1203
					  cout<<"Building up on a past plan that was not sucessfull: Trying Again...."<<endl;
					  Report<<"Building up on a past plan that was not sucessfull: Trying Again...."<<endl;
				     }
					  PtOfReplan=0;//***************************************************


					 // for(iCo=0; iCo<NPiCs-1; iCo++)
						//{
						//	cout<< "Proposed Plan is as follows:" << endl;
						//	cout << "Picking : " << SeqAcP[iCo+1]  << " and placing it on : " << SeqAcP[iCo] << endl;
						//}
					 //
					 // Time::delay(20);


					  for(iCo=0; iCo<NPiCs-1; iCo++)
							 {
							   int pickk=50;
							   int Placc=50;
							   if(GiD==14){
							       cout << "Picking" << SeqAcP[iCo+1]  << "and placing it on" << SeqAcP[iCo] << endl;
                                   Report<< "Picking" << SeqAcP[iCo+1]  << "and placing it on" << SeqAcP[iCo] << endl;
								   pickk=SeqAcP[iCo+1];
								   Placc=SeqAcP[iCo];
							   }
							   if(GiD==32){
							       cout << "Transporting Object " << SeqAcP[iCo] << "in the box" << endl;
								   Report << "Transporting Object " << SeqAcP[iCo] << "in the box" << endl;
								    pickk=SeqAcP[iCo];
								   Placc=SeqAcP[iCo+1];
							   }

								  int picks = PickandPlace(pickk,Placc,iCo); //**************************************
							//   int picks=1; // loop break just for testing //***********************************
                            cout<<"PICKS Result"<< picks<<endl;
							if(picks==4) //VM Made change here 23/03
							   {
							      cout << "PMP target Unreachable or Object slippage: Attention:" << endl;
								  Report << "PMP target Unreachable or Object slippage: Attention:" << endl;
								  SeqAcP[iCo+1]=SeqAcP[iCo];
							   }

								 if(picks==0)
								   {
									   cout << "Sense failiures: Attention" << endl;
									   Report << "Sense failiures: Attention" << endl;
									   PtOfReplan=iCo;
                                       cout << "Failure sensed when picking" <<SeqAcP[PtOfReplan+1] << "and placing it on"<< SeqAcP[PtOfReplan]<< endl;
									   Report << "Failure sensed when picking" <<SeqAcP[PtOfReplan+1] << "and placing it on"<< SeqAcP[PtOfReplan]<< endl;
									   Replan=1;; //to exit loop
                                       iCo=NPiCs;
									   //state=3; // go back to refresh and contact reasoning
								    }
							   if(picks==1){
                               StakSuc=StakSuc+picks;
							   cout << "Seems fine: Going ahead with the next micro sequence !!" << endl;
							   Report << "Seems fine: Going ahead with the next micro sequence !!" << endl;
							   }
							 }
					  //MAY BE WE MUST TAKE A SNAPSHOT AND MEASURE THE TOP MOST POINT: IN CASE OF STACK DESTRUCTION
					   cout << "Finished the Goal!! Anticipated reward from past experience is" << StakSuc+1 <<endl;
					   Report << "Finished the Goal!! Anticipated reward from past experience is" << StakSuc+1 <<endl;
					  //have to read the right value from Strata...
					  //Time::delay(60);
					   if(Replan==1){
					   state =3;
					   }
						if(Replan==0){
					   state = 0;
					   }

			    }

			break;

			case 3 : {
				cout<<" Sensed Unexpected hurdles: Monitoring where the past plan failed and contacting EPIM " <<endl;
				Report<<" Sensed Unexpected hurdles: Monitoring where the past plan failed and contacting EPIM " <<endl;
				    int iCo,jCo;
					for(iCo=0; iCo<10; iCo++){
					//  SeqAcP[iCo]=50;
					  PastPlan[iCo]=50;
					  }
					numberpast=0;
				//	findsuccess=0; //********************Need to uncomment this********VM1203
				    if((findsuccess==0)||(CannotFind==1)) //or grasp slippage or PMP fialiure
						{
							numberpast=PtOfReplan;
							for(iCo=0; iCo<PtOfReplan+1; iCo++){
							PastPlan[iCo]=SeqAcP[iCo];
							cout<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
							Report<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
							PPiCo=PastPlan[iCo];
							}
						}
					if((findsuccess2==0)&&(findsuccess!=0)) //failed during palceing operations
						{
							numberpast=PtOfReplan-1;
							for(iCo=0; iCo<PtOfReplan; iCo++){
							PastPlan[iCo]=SeqAcP[iCo];
							cout<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
							Report<< "Transfering half finished plan to Working memory...."<<PastPlan[iCo]<<endl;
						   }
                        }
					cout<< "Peceiving the messed up situation and contacting Reasoning" <<PastPlan[numberpast] << endl;

					// A time delay may be needed here
					Time::delay(12);
				  	RefreshPlacemap(); //****** Need to Uncomment this for real testing: VM1203
					//***************************************************************************

					//***************************************************************************
					state=1;
					}
			break;

			//====================Decoding Minimal action Plan and Micromonitoring
			case 6:
				{
					int iCo,jCo;
					int Pctrr=0;
					int AcSeqe[20][50];
                    if((RootGoalFlag==14)||(Replan==1)) // there is a need to interpret
					{
						for(iCo=0; iCo<20; iCo++){
						  SeqAcInterp[iCo]=50;
						  }
						for(iCo=0; iCo<20; iCo++){
						  SeqAcNum[iCo]=50;
						//  PastPlan[iCo]=50;
						  }

						for(iCo=0; iCo<20; iCo++)
								 {
									for(jCo=0; jCo<50; jCo++)
											{
							  					 AcSeqe[iCo][jCo]=Strata[Pctrr];
												 Pctrr=Pctrr+1;
											 }
								 }

						NActs=0;
						int ctr=1;
						for(iCo=0; iCo<pointRew; iCo++)// this can be limited to the chunk that is determined by the point of intersection and min energy of the returned plan
								 {
									if((AcSeqe[iCo][43]==1)&&(AcSeqe[iCo][49]==0)) //not the goal representation but the action that leads to the goal
									  {
											NActs=NActs+1;
											for(jCo=0; jCo<12; jCo++)
													{
							  							if(AcSeqe[iCo][jCo]==1)// this may be an analog number based on the rememberd memory activity
															{
															 SeqAcInterp[ctr]=jCo;
															 Encapsulate[ctr]=0;
															 SeqAcNum[ctr]=iCo;
			 //NOTE: CHANGED SeqAcNum[iCo] to SeqAcNum[ctr] everywhere..this is to find the correct seq no in case of encapsulation: Replanning seems to work still
															 cout << "found micro action::" << SeqAcInterp[ctr] << "micro sequnce::"<<SeqAcNum[ctr] << endl;
															 ctr=ctr+1;
															}
													}
										}
									if((AcSeqe[iCo][49]==0)&&(AcSeqe[iCo][48]==1)&&(AcSeqe[iCo][46]==0)) //an encalpulated Goal
									  {
										NActs=NActs+1;
										cout << "An encapusalted Goal::" << endl;
										for(jCo=0; jCo<12; jCo++)
													{
														if(AcSeqe[iCo][jCo]==1)// this may be an analog number based on the rememberd memory activity
															{
															 SeqAcInterp[ctr]=jCo;
															 SeqAcNum[ctr]=iCo;
															 Encapsulate[ctr]=1;
															 cout << "found micro goal::" << SeqAcInterp[ctr] << "micro sequnce::"<<SeqAcNum[ctr] << endl;
															 ctr=ctr+1;
															}
													}

									  }
								 }
						  SeqAcInterp[0]=NActs;
						  Encapsulate[0]=NActs;
				}//State7Flag
					  int PastplanCtr=0;
					  if(Replan==1)
						  {
                            pointIntersect=PtOfReplann+1;
							cout<<"Intersecting element is "<<pointIntersect << endl;
					  	  }
					 	 if(MergePlans==14)
						  {
                            pointIntersect=iterMicro-2;
							cout<<"Intersecting element is "<<pointIntersect << endl;
					  	  }

					  cout<< "there are " << SeqAcInterp[0] << "Micro sequences leading to goal:" << Goal_AcID+1 << "Interpreting.." << endl;
					     for(iCo=pointIntersect; iCo<SeqAcInterp[0]; iCo++)
							{
								if(Encapsulate[iCo+1]==0)
								{
								      CMicroSub=MicroMonitor(SeqAcInterp[iCo+1]);  //Am commenting Micromonitor to test encapsulation
									//  CMicroSub=50; //Commented out to test online learning
									  if(CMicroSub==1){
										 cout<<"Top Down and Bottom up activations Do not resonate: retriggereing reasoning/Xploration"<<endl;
										 PtOfReplann=SeqAcNum[iCo+1];
										 ActionPointer=SeqAcInterp[iCo+1];
										 int lastelem=MaintainTrace(PtOfReplann);
										 iterMicro=lastelem+1; // this is the place for chunking experience from another memory, iterMicro-1 is the frame of partial cue
										 //going back to epim:if no plan recd..u cna retroactivate body hub to Autonomously explore through OPC: this must be a new state
										 break;
									  }

									   if(CMicroSub==50){
										  cout<<"Top Down and Bottom up activations resonate: moving to next microsequnce"<<endl;
										}
								}
								if(Encapsulate[iCo+1]==1)
									{
                                        cout<<"Encapsulated Micro Goal:::::"<< SeqAcInterp[iCo+1] << "in micro sequnce::"<<SeqAcNum[iCo+1] << endl;
										RootGoalFlag=14;
										GoalStack[GoalPointer][0]=Goal_AcID;
										GoalStack[GoalPointer][1]=NActs;
										GoalStack[GoalPointer][2]=iCo+1; //point of encountering an encapsulated goal
										int pCo;
										for(pCo=0; pCo<NActs; pCo++)
											 {
										       GoalStack[GoalPointer][pCo+3]=SeqAcInterp[pCo+1];
										 	 }
										for(pCo=0; pCo<NActs; pCo++)
											 {
										       GoalStack[GoalPointer][pCo+3+NActs]=Encapsulate[pCo+1];
										 	 } // I stored the intrepreted plan that was being executed: Now transfering control to the encapsulated goal
										GoalPointer=GoalPointer+1;
										Goal_AcID=SeqAcInterp[iCo+1];
										GContext=14;
										Replan=0;
										pointIntersect=0;
										cout<<" Initializing DARWIN Working memory for new Subgoal " <<endl;
								       // InitializeWorkingMemory(GoalPointer);
										//THIS MAY NOT BE NEEDED AS WORKIGN MEMORY IS INITIALIZED THROUGH USER INTERFACE: Comment on 14th Jan
										CMicroSub=14;
										HubID_EPIM=50;
										break;
									}
						 }
						 if(CMicroSub==50){
							 /*if(TerminateFlag==50)
								 {
								   state = 0;
								 }*/
							 if(TerminateFlag==1)
								 {
                                   cout<<" MicroGoal with ID::::: " << GoalPointer << "is terminated" <<endl;
								   if(GoalPointer==0){
								   state = 0;
								   }
								   if(GoalPointer!=0){
								   state = 7;
								   }
								 }
							 }
						  if(CMicroSub==1){
							 state = 1;
							 Replan=1;
						 }
						  if((CMicroSub==1)&&(UGPush==1)&&(NullObj==1)){
							 state = 11;
							// Replan=122;
						 }
						  if(CMicroSub==14){
							 state = 1;
							 //Replan=1;
						 }
                   /*    Interpret();  // Interprets plan  in shapes to plan in FORTH representations
					  if(Replan==0){
							 state = 5;
						 }
					    if(Replan==1){
							 state = 4;
						 }*/

				}
			break;


       case 7 : {
			  cout<<" MicroGoal accomplished, Going back to root goal ID:::::"<<GoalStack[GoalPointer-1][0] << endl;
			  GoalPointer=GoalPointer-1;
			  Goal_AcID= GoalStack[GoalPointer][0];
			  NActs=GoalStack[GoalPointer][1];
			  pointIntersect= GoalStack[GoalPointer][2]; //point of encountering an encapsulated goal
			  if(PushIntersect==14)
			  {
			    GoalStack[0][2]=0;
			  }
			  int pCo;
			  for(pCo=0; pCo<NActs; pCo++)
						{
						   SeqAcInterp[pCo+1]=GoalStack[GoalPointer][pCo+3];
						}
			 for(pCo=0; pCo<NActs; pCo++)
						{
						  Encapsulate[pCo+1]=GoalStack[GoalPointer][pCo+3+NActs];
						} // I stored the intrepreted plan that was being executed: Now transfering control to the encapsulated goal
			  SeqAcInterp[0]=NActs;
			  Encapsulate[0]=NActs;
			  Replan=0;
			  RootGoalFlag=23;
			  state=6;//Going back to mincromonitor
			}
			break;

			 case 10 : {
			  cout<<" Combining past experience with Explorative actions/User guidance" << endl;
			  int iCo,jCo;
			  MergePlans=0;
			  NXploreAct=0;
			//   PtOfReplann=SeqAcNum[iCo+1];
			  // ActionPointer=SeqAcInterp[iCo+1];  //U already get action pointer becasue u go through Micromonitoring
			   int ExplorePt=MaintainTrace(PtOfReplann);
			   iterMicro=ExplorePt+1;
			   cout<<" Retroactivating Bodh Hub-Action Hub" << endl;
			   int ActPlay=RetroactivateBodyHub(0);
			   int ActionPresent =0;
			   while(NXploreAct !=0){
			   if(ActPlay!=50)
				 {
					int PMicroSub=MicroMonitor(ActionHubXplore[ActionPresent]);
					if(PMicroSub==50){
										 cout<<"Merging the explorative action with the Previous plan"<<endl;
										 NXploreAct=0;
										 if(ActionHubXplore[ActionPresent]==6){
										 state=0;
										 }
										 if(ActionHubXplore[ActionPresent]!=6){  //tThe robot did not choose to terminate the goal but tried soemthing else that succeded
										     RootGoalFlag=23;
											 MergePlans=14;
											 Replan=0;
											 state=6;
											 for(jCo=0; jCo<20; jCo++)
													{
													   SeqAcInterp[jCo]=SeqAcInterpR[jCo];
													}
										 }
									  }

									   if(PMicroSub==1){
										  cout<<"This did nto work out: Exploring again"<<endl;
										 // ActionHubXplore[ActPlay]=1;
										  ActionPresent=ActionPresent+1;
										  NXploreAct=NXploreAct-1;
										 // ActPlay=RetroactivateBodyHub(0);
										}
			  }
			 }
			break;

		}

		//Time::delay(5);
	}
}
}


void ObserverThread::threadRelease() {
    // nothing
	EpimCtrlPort.close();
	WorABin.close();
	HubA.close();
	MapA.close();

}

int ObserverThread::MaintainTrace(int PtRe)
	{
		int iCo,jCo,Pctrr=0;
		for(iCo=iterMicro; iCo<PtRe+1; iCo++)
							 {
								for(jCo=0; jCo<50; jCo++)
										{
							  				 BottomUPTrace[iCo][jCo]=Strata[Pctrr];
											 Pctrr=Pctrr+1;
										 }
							 }

		for(jCo=0; jCo<42; jCo++)
				{
				   BottomUPTrace[PtRe+1][jCo]=BodyHub[jCo];
				}

		for(jCo=0; jCo<20; jCo++)
				{
				   SeqAcInterpR[jCo]=SeqAcInterp[jCo];
				}

	return PtRe+1;
	};

int ObserverThread::MicroMonitor(int stateMicormonitor)
	{
		int iCo,jCo,ConsMicro,ActionPlotter[12];
		ConsMicro=0;
      	 stateMM=stateMicormonitor;
					 for(jCo=0; jCo<42; jCo++)
									{
									  BodyHub[jCo]=0;
									}
					  for(jCo=0; jCo<12; jCo++)
									{
									  ActionPlotter[jCo]=0;
									}
				 switch(stateMM)
					 {
						case 2:
							{
							   cout<<"Initialializing search primitive for Goal pointer::"<<  Goal_AcID << endl;
							   ActionPlotter[2]=1;
							    if((GoalStack[0][0]==0)||(GoalStack[0][0]==1)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=0;
							   	   }
							   if(((GoalStack[0][0]==4)&&(GoalPointer==2))||((GoalStack[0][0]==9)&&(GoalPointer==3)))//the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=0;
							   	   }
							    if((GoalStack[0][0]==4)&&(GoalPointer==1)||((GoalStack[0][0]==9)&&(GoalPointer==2))) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=1;
							   	   }
								if((GoalStack[0][0]==3)&&(GoalPointer==1)&&(PushOID==5)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=0;
							   	   }
								if((GoalStack[0][0]==3)&&(GoalPointer==4)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=1;
							   	   }
								if((GoalStack[0][0]==3)&&(GoalPointer==3)) //the roor goal issued by the user in reference to the global work space
								   {
							         GWSPtr=2;
							   	   }

							   double Fres=PrimSearch(0,0,95);

							   //cout << "Input number goal arguement words " << endl;
				      //          cin >> Fres; //this is just for checking

							   if(Fres<=0)
								   {
							         cout<<"Analyzing anticipated consequnce in the body hub"<<endl;
									 BodyHub[18]=1;
									 BodyHub[19]=1;
									 BodyHub[23]=1;
									 ConsMicro=50;
								    }
							    if(Fres>0.1)
								   {
							         cout<<"Analyzing anticipated consequnce in the body hub"<<endl;
									 initBodyHub();
									 BodyHub[20]=1;
									 BodyHub[21]=1;
									 BodyHub[22]=1;
									 ConsMicro=1;
									 NullObj=1;
								   }

							 }
		                break;

						case 0:
							{
							   cout<<"Initialializing Body Schema primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[0]=1;
								PrimPushExecFlag=0;

								 if((GoalStack[0][0]==4)&&(GoalPointer==1)||((GoalStack[0][0]==9)&&(GoalPointer==2))) ////the roor goal issued by the user in reference to the global work space
								   {
										if(PeriPersonalFB-PeriPersonalF!=0)
										 {
										     if(PosHandOcc[1]<-robToCamY)
												 {
											       PrimPush(-140,PosHandOcc[1]+140); 
													cout << "Pushing the Fuse Box towards the right PPS" << endl;
												   // PrimPush(104,-77); 
												 }
											 if(PosHandOcc[1]>=-robToCamY)
												 {
											      PrimPush(140,PosHandOcc[1]-140); 
													 //PrimPush(-104,77); // towards Tx 
												 }
											}
							   	   }

							//	int rres=PrimBodySchemaIndustrial(1,0,1,0,0,0); this was for the industrial robot
								
								 int rres=PrimBodySchema(1,0,1,1,76,0); 

								//PrimBodySchema(1,0,1,0,0,1); Commented out for testing 7/02/14
							   rres=1;
							   if(rres<=0) // This implies failiure to reach that will be addressed soon, then the if loop must be different
								   {
							         cout<<"Analyzing anticipated consequnce in the body hub"<<endl;
									 BodyHub[3]=1;
									 BodyHub[4]=1;
									 ConsMicro=50;
									}
							    if(rres>0.1)
								   {
							           cout<<"Reach Succesfull"<< endl;
									   BodyHub[0]=1;
									   BodyHub[1]=1;
									   BodyHub[24]=1; // 24:Arm Occ R, 25-L, 26-Both: this is based on the 3D location of the target
									   ConsMicro=50;
									   
								   

								  // AlignHand(); //for testing alignment..
								   }

							}
		                break;

						case 1:
							{
							   cout<<"Initialializing Grasp primitive for Goal pointer::"<<  Goal_AcID << endl;
							   ActionPlotter[0]=2;
							 //  PrimGrasp();  //Not triggered presently

							 //  PrimGraspIndustrial(GRIP_CLOSE,BODY_SIDE_LEFT);
							   //PlaceMap[PlaceMapPos][1]
							    if((robToCamY+PlaceMap[PlaceMapPos][1])<0){ //using the correct value of y axis in robot frame..check in PrimBodySchema
									
									PrimGrasp(GRASP_PINCH,0);//O for left arm
								 }
								 else
								 {
									
								   PrimGrasp(GRASP_PINCH,1);//1 for right arm
								 }
							   int Gres=1;
							   if(Gres==1) // This implies failiure to reach that will be addressed soon, then the if loop must be different
								   {
							         cout<<"Grasp Done:Analyzing anticipated consequnce in the body hub"<<endl;
									 BodyHub[6]=1;
									 BodyHub[7]=1;
									 BodyHub[11]=1;
									 BodyHub[28]=1;
									 BodyHub[29]=1;
									 BodyHub[30]=1;
									//Finger Occupied must also be represented in the future.
									 Time::delay(5);
									 PrimBodySchema(19,0,0,1,76,0); 
									 ConsMicro=50;
									}
							    if(Gres==0)
								   {
							          cout<<"Grasping Failed"<< endl;
									   BodyHub[8]=1;
									   BodyHub[9]=1;
									   BodyHub[10]=1; // 24:Arm Occ R, 25-L, 26-Both: this is based on the 3D location of the target
									   ConsMicro=1;
									}

							}
		                break;

						case 6:
							{
							   cout<<"Initialializing Interrupt primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[6]=1;
							    cout<<"Goal Terminate Sucesfull: Reinitializing working memory and waiting for user goal"<< endl;
								BodyHub[16]=1;
							    BodyHub[17]=1;
								TerminateFlag=1;
							    ConsMicro=50;
							 }
		                break;

						case 5:
							{
							   cout<<"Initialializing 'Seek User help and learn new experience' primitive"<< endl;
							   ActionPlotter[5]=1;
							   int ActionUser=UserInterface(32);

							   if(ActionUser==1)
								   { // user is willing to help, puts the object in the scene..., ideally u must refresh placemap
									/*   for(jCo=0; jCo<42; jCo++)
											{
											  GlobalWorkSpace[0][jCo]=PlaceMapHub[0][jCo];
											}
									    */
									   ConsMicro=50;
								   }
							     if(ActionUser==0)
								   {
									   /*BodyHub[16]=1;
									   BodyHub[17]=1;*/
									  ConsMicro=1;
								   }

							}

						break;
						case 11:
							{
							   cout<<"Initialializing Release primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[11]=1;
								   AlignFlag=1; //for testing alignment..
								   RefreshPlacemap(); //added on 28/10/2014 ajaz
								   //================================Code for Alignment=======================================
								   		//double AlignF = 99999;

										//while(AlignF > 10)
										//{
										//	AlignF=Align();
										//	std::cout<<"Distance alignment: "<<AlignF<<std::endl;
										//}
										//LoCAlign[2]=86;
										////double alex=PrimBodySchemaIndustrial(95,0,0,0,0,0);  //This was for the industrial robot
										//double alex=PrimBodySchema(1,0,1,0,0,0);
								   	   double AlignF = 99999;
               
									// Check if Vision can find the objects after certain trial, check SceneChanged function
									bool flag_exit = false;

									// If distance is greater than a threshold, do alignment
													while(AlignF > 10)
													{
									  // Calculating the distance and moving the robot to do alignment
														bool flag_exit = Align(AlignF);
									                    double alex=PrimBodySchema(95,0,0,1,76,0);
														std::cout<<"Distance alignment: "<<AlignF<<std::endl;

									  //if(AlignF == -1)
									  //{
									  //  flag_exit = true;                   
									  //}
													}

									if(!flag_exit)
									{
										// LoCAlign[2]=70;
										//PrimBodySchema(95,0,0,1,76,0);
										Insert(3);
										Time::delay(3);
										Insert(5);
										Time::delay(2);
										//Insert(7);
										//Time::delay(2);
										//Insert(9);
										//Time::delay(3);
										//Insert(10);
										//Time::delay(3);
									}

									if((robToCamY+PlaceMap[PlaceMapPos][1])<0){ //using the correct value of y axis in robot frame..check in PrimBodySchema

										PrimGrasp(GRASP_RELEASE,0);//O for left arm
													}
									else
									{
										PrimGrasp(GRASP_RELEASE,1);//1 for right arm
									}
									Time::delay(4);
								PrimBodySchema(19,0,0,1,-20,0); 
								Time::delay(2);
								 cout<<"Object released"<< endl;
								 BodyHub[28]=0;
								 BodyHub[29]=0;
								 BodyHub[30]=0;
								TerminateFlag=1;
								//=========================================
								//===================================================

								if(PushOIDAsm==14)
										{
											 for(jCo=0; jCo<42; jCo++)
												 {
													  GlobalWorkSpace[0][jCo]=0;
													  GlobalWorkSpace[1][jCo]=0;
													  GlobalWorkSpace[2][jCo]=0;
												}
											  GlobalWorkSpace[0][9]=1;
											  /*GlobalWorkSpace[1][8]=1;
											  GlobalWorkSpace[2][9]=1;*/
										  }
								//===================================================
							    ConsMicro=50;
							 }
		                break;
						//change
						 case 3:
							{
							   cout<<"Initialializing Push primitive for Goal pointer::"<<  Goal_AcID << endl;
							    ActionPlotter[4]=1;
								
								          if(PosHandOcc[1]<-robToCamY)
												 {
											       PrimPush(-90,PosHandOcc[1]+140); 
													// PrimPush(104,-77); 
												 }
											 if(PosHandOcc[1]>-robToCamY)
												 {
											      PrimPush(90,PosHandOcc[1]-140); 
													 //PrimPush(-104,77); // towards Tx 
												 }
							    cout<<"Push Sucesfull: Reinitializing working memory and waiting for user goal"<< endl;
								BodyHub[13]=1;
							    BodyHub[14]=1;
								ConsMicro=50;
							 }
		                break;  


						case 10:
							{
							   cout<<"Initialializing Observe primitive for Goal pointer::"<<  Goal_AcID << endl;
							   ConsMicro=50;
							 }
		                break;  
						//change
		 			} //micromonitor loop ends


				 // It will be interesting to plot the comparison between the present behavior in comparison with the plan proposed by reasoning:?????
				 //this has to take into account Encapsulation...to enable reconstruction..
				    Behavior[iterMicroN][stateMicormonitor]=1;
                    Behavior[iterMicroN][43]=1;
					 for(iCo=0; iCo<50; iCo++)
						 {
						 Present<< Behavior[iterMicroN][iCo]<< "    ";
						 }
						 Present << "    " << endl;
				    iterMicroN=iterMicroN+1;
					if(stateMicormonitor!=5)
						{
							for(jCo=0; jCo<42; jCo++)
											{
											  Behavior[iterMicroN][jCo]=BodyHub[jCo];
											}
							Behavior[iterMicroN][46]=1;
								 for(iCo=0; iCo<50; iCo++)
									 {
									 Present<< Behavior[iterMicroN][iCo]<< "    ";
									 }
						 Present << "    " << endl;
						 iterMicroN=iterMicroN+1;
						}
			if (actionPlot.getOutputCount()) {	
				 //////////////////changes to add visualization ACTION GUI Commented out
				Bottle& actionBot =actionPlot.prepare();
				actionBot.clear();
				actionBot.addString("action");
				Bottle& actionBotAll = actionBot.addList();
				actionBotAll.clear();
				for(int i=0; i<12; i++)
					{

                     actionBotAll.addDouble(ActionPlotter[i]);
				}
			    cout<<"Sending Action hub Matrix to DARWIN GUI"<<endl;
             	actionPlot.write();
				Sleep(2000);
			}
			if (bodyPlot.getOutputCount()) {	
				////////////////changes to add visualization BODY
				Bottle& bodyBot =bodyPlot.prepare();
				bodyBot.clear();
				bodyBot.addString("body");
				Bottle& bodyBotAll = bodyBot.addList();
				bodyBotAll.clear();
				for(int i=0; i<42; i++)
					{

                     bodyBotAll.addDouble(BodyHub[i]);
				}
			    cout<<"Sending Body hub Matrix to DARWIN GUI"<<endl;
             	bodyPlot.write();
				Sleep(3000);
			}

		return ConsMicro;
		
	};

	void ObserverThread::Insert(int i)
	{
		if(!(Network::isConnected("/commandRobot:o", "/psControl/interface:i"))){
			Network::connect("/commandRobot:o", "/psControl/interface:i");
		}
		Bottle& outBot = commandRobot.prepare();   // Get the object
		outBot.clear();
		outBot.addVocab(CMD_TORSO); // put "pos" command in the bottle
		Bottle& listBot2 = outBot.addList();
		listBot2.addDouble(PMPresp[4]);
		listBot2.addDouble(PMPresp[5]);
		listBot2.addDouble(PMPresp[6]+i);
		printf("\n\nSending bottle torso (%s)\n",outBot.toString().c_str());
		commandRobot.write();
		};

void ObserverThread::initBodyHub()
{
	int iCo;
	 for(iCo=0; iCo<42;iCo++)
	 {
	   BodyHub[iCo]=0;
	 }
};


double ObserverThread::PrimPush(int ReachSide, int PushTarget)
{
	//int Pushres=PrimSearch(0,0,95);
	PrimPushFlag=1;
	PrimPushExecFlag=0;
	PrimPushSideFlag=0;
	PlaceMap[PlaceMapPos][0]=PosHandOcc[0];
	PlaceMap[PlaceMapPos][1]=PosHandOcc[1]+ReachSide;
	PlaceMap[PlaceMapPos][2]=PosHandOcc[2];
	if(PosHandOcc[1]<-robToCamY)
	{		
			initGPL();
			Time::delay(4);
	}
	if(PosHandOcc[1]>=-robToCamY)
	{		
			initGPR();
			Time::delay(4);
	}
	int pushreach=PrimBodySchema(1,0,1,1,-20,0);
	Time::delay(3);
	PrimPushExecFlag=1;
	PlaceMap[PlaceMapPos][0]=PosHandOcc[0];
	//PlaceMap[PlaceMapPos][1]=-1*(PosHandOcc[1]+ReachSide-PushTarget); /right arm
	if(PosHandOcc[1]<-robToCamY)
	 {
		// PlaceMap[PlaceMapPos][1]=PosHandOcc[1]+ReachSide-PushTarget;
		 PlaceMap[PlaceMapPos][1]=-10;
	}
	if(PosHandOcc[1]>=-robToCamY)
	 {
		// PlaceMap[PlaceMapPos][1]=-1*(PosHandOcc[1]+ReachSide-PushTarget);
		 PlaceMap[PlaceMapPos][1]=10;
	}
	//PlaceMap[PlaceMapPos][2]=PlaceMap[PlaceMapPos][2];
	PrimPushFlag=1;
	pushreach=PrimBodySchema(1,0,1,0,-20,0);

	//Time::delay(3);
	cout<<" Predicted object location is    " << XPosition[0] << XPosition[1] <<XPosition[2]<< endl;
	//if(pushreach==1)
			//{
	          cout<<" Simulation of pushing predicts sucessful spatial reorganization to afford inserting "<< endl;
			PlaceMap[PlaceMapPos][0]=PosHandOcc[0];
			PrimPushExecFlag=1;
			if(PosHandOcc[1]<-robToCamY)
				 {
					 //PlaceMap[PlaceMapPos][1]=PosHandOcc[1]+ReachSide-PushTarget;
					  PlaceMap[PlaceMapPos][1]=-50;
					  //initGPL();
					  //Time::delay(4);
				}
				if(PosHandOcc[1]>=-robToCamY)
				 {
					 //PlaceMap[PlaceMapPos][1]=-1*(PosHandOcc[1]+ReachSide-PushTarget);
					 PlaceMap[PlaceMapPos][1]=40;
					 //initGPR();
					 //Time::delay(4);
				}
	       // PlaceMap[PlaceMapPos][2]=PlaceMap[PlaceMapPos][2];
				PrimPushFlag=1;
				if(!(Network::isConnected("/commandRobot:o", "/psControl/interface:i"))){
					Network::connect("/commandRobot:o", "/psControl/interface:i");
				}
				Bottle& outBot = commandRobot.prepare();   // Get the object
				outBot.clear();
				outBot.addVocab(COMMAND_SPEED);
				outBot.addVocab(COMMAND_FAST);
				printf("\n\nSending speed fast request (%s)\n",outBot.toString().c_str());
				commandRobot.write();
				Time::delay(4);
				pushreach=PrimBodySchema(1,0,1,1,-20,0);
				Time::delay(4);

				outBot.clear();
				outBot.addVocab(COMMAND_SPEED);
				outBot.addVocab(COMMAND_SLOW);
				printf("\n\nSending speed slow request (%s)\n",outBot.toString().c_str());
				commandRobot.write();
			//}
	PrimPushFlag=0;
	PrimPushExecFlag=0;
	Time::delay(5);
	pushreach=PrimBodySchema(19,0,1,1,0,0);
	AlignFlag=1;
	//Sleep(5000);
	//RefreshPlacemap();
	PrimPushSideFlag=1;
	if(PosHandOcc[1]<-robToCamY)
				 {
					 PlaceMap[PlaceMapPos][1]=50;
				}
	if(PosHandOcc[1]>=-robToCamY)
				 {
					 PlaceMap[PlaceMapPos][1]=-50;
				}
	cout<<" New object location is    "<<LoCAlign[0] << LoCAlign[1] <<LoCAlign[2]<< endl;
	pushreach=1;
	//PrimPushSideFlag=0;
	return pushreach;
};


bool ObserverThread::SceneChanged(double threshold)
{
  // Varaibles to store the previous values of fuse and hole
  double prev_fuse[3];
  double prev_hole[3];

  // Loading the previous values of fuse and hole
  for(int i=0;i<3;++i)
  {
    prev_fuse[i] = LoCAlignFusee[i];
    prev_hole[i] = LoCAlign[i];
  } 

  // Refreshing the placemap to get the current position of the fuse and hole
  RefreshPlacemap();

  // Check if atleast two objects are found
  if(NumberofObs>=2)
  {

    bool detected_hole = false;
    bool detected_fuse = false;

    for(int i=0;i<NumberofObs;++i)
    {
      // If hole is found
      if(ObjIDEE[i] == 100)
      {
        detected_hole = true;
      }

      // If fuse is found
      if(ObjIDEE[i] == 101)
      {
        
        detected_fuse = true;

      }
    }

    // Proceed only if both hole and fuse are found, if not exit the function
    if(!(detected_fuse && detected_hole)){
      std::cout<<"DID NOT find fuse and/or  Hole "<<std::endl;
        return true;
    }
  }

  // If less than two objects are found, exit the function
  else{
    std::cout<<"No.of objects less than 2"<<std::endl;
    return true;
  }

  // Find the distance of the fuse from the previous and the current position
  double xdiff_fuse = (prev_fuse[0]-LoCAlignFusee[0]);
	double ydiff_fuse = (prev_fuse[1]-LoCAlignFusee[1]);
  double zdiff_fuse = (prev_fuse[2]-LoCAlignFusee[2]);
	double distance_fuse = (sqrt(pow(xdiff_fuse,2)+ pow(ydiff_fuse,2)+pow(zdiff_fuse,2)));

  std::cout<<std::endl;
  std::cout<<"Distance of the fuse from the last two snapshots: "<<distance_fuse<<std::endl;

  // Find the distance of the hole from the previous and the current position
  double  xdiff_hole = (prev_hole[0]-LoCAlign[0]);
	double ydiff_hole = (prev_hole[1]-LoCAlign[1]);
    double zdiff_hole = (prev_hole[2]-LoCAlign[2]);
	double distance_hole = (sqrt(pow(xdiff_hole,2)+ pow(ydiff_hole,2)+pow(zdiff_hole,2)));

  std::cout<<std::endl;
  std::cout<<"Distance of the hole from the last two snapshots: "<<distance_hole<<std::endl;
  std::cout<<std::endl;

  // If any one of the two distances calculated above are greater than the threshold, 
  // then return true to calculate distance again
  if(distance_fuse > threshold || distance_hole > threshold)
  {
    return true;
  }

  // If both distances are below the threshold, return false and continue to the Align function
  else
  {
    return false;
  }

}

// This function Aligns the fuse to the opening and facilitates insertion into the hole
bool ObserverThread::Align(double& AlignF)
{
	AlignFlag=1;
	//Sleep(30000); // Static Sleep - not used any more

  std::cout<<"Entered Align Function"<<std::endl;

  bool Scene_flag = true;
  int counter = 0;

  while(Scene_flag)
  {
    //Distance threshold is set to 15
    ++counter;
    std::cout<<"Calling the SceneChanged Function for  "<<counter<<" time"<<std::endl;
    Scene_flag =  SceneChanged(15);

    // If vision cannot find the required objects after number of tries, this case 10, then exit
    if(counter >= 50)
    {
      std::cout<<"Vision cannot find required objects in the scene, Please check"<<std::endl;
     return true;
    }

  }
  //Add new function to check the distance between current and previous fuse cap position

	//RefreshPlacemap();

  // Variables to store difference
  double xdiff = 0;
  double ydiff = 0;

	/*LoCAlign[0]=(XPosition[0]+LoCAlign[0])/2;
	LoCAlign[1]=(XPosition[1]+LoCAlign[1])/2;
	LoCAlign[2]=XPosition[2]-40;*/
	//LoCAlign[0]=((LoCAlignFusee[0]+LoCAlign[0])/2);
	//LoCAlign[1]=(LoCAlignFusee[1]+LoCAlign[1])/2;
	//LoCAlign[2]=205;
  // Finding the distance between positions of the fuse and the hole given by vision
	  xdiff = (LoCAlign[0]-LoCAlignFusee[0]);
	  ydiff = (LoCAlign[1]-LoCAlignFusee[1]);
	  AlignF = (sqrt(pow(xdiff,2)+ pow(ydiff,2)));

	std::cout<<"Distance : "<<AlignF<<std::endl;

	///GUI yarpscope for Aligment
	if(alignPlot.getOutputCount()>0){
		Bottle & PlotDistance = alignPlot.prepare();
		PlotDistance.clear();
		PlotDistance.addDouble(AlignF);
		alignPlot.write(true);
	}
  // This difference in X and Y are added to the current position of the robot (calculated from the encoder values)
  // to get the next position to which the robot should move
		LoCAlign[0]=XPosition[0]+xdiff;
		LoCAlign[1]=XPosition[1]+ydiff;
		LoCAlign[2]=125;

	return false;
	};

//double ObserverThread::Align()
//{
//	AlignFlag=1;
//	Sleep(30000);
//	RefreshPlacemap();
//	/*LoCAlign[0]=(XPosition[0]+LoCAlign[0])/2;
//	LoCAlign[1]=(XPosition[1]+LoCAlign[1])/2;
//	LoCAlign[2]=XPosition[2]-40;*/
//	//LoCAlign[0]=((LoCAlignFusee[0]+LoCAlign[0])/2);
//	//LoCAlign[1]=(LoCAlignFusee[1]+LoCAlign[1])/2;
//	//LoCAlign[2]=205;
//	double  xdiff = (LoCAlign[0]-LoCAlignFusee[0]);
//	double ydiff = (LoCAlign[1]-LoCAlignFusee[1]);
//	double distance = (sqrt(pow(xdiff,2)+ pow(ydiff,2)));
//	std::cout<<"Distance : "<<distance<<std::endl;
//		LoCAlign[0]=XPosition[0]+xdiff;
//		LoCAlign[1]=XPosition[1]+ydiff;
//		LoCAlign[2]=145;
//		//double alex=PrimBodySchemaIndustrial(95,0,0,0,0,0);
//
//		double alex=PrimBodySchema(95,0,0,0,0,0);
//	return distance;
//	};

double ObserverThread::AlignHand()
{
	                                double AlignF = 99999;
               
									// Check if Vision can find the objects after certain trial, check SceneChanged function
									bool flag_exit = false;

									// If distance is greater than a threshold, do alignment
													while(AlignF > 10)
													{
									  // Calculating the distance and moving the robot to do alignment
														bool flag_exit = Align(AlignF);
									                    double alex=PrimBodySchema(95,0,0,0,76,0);
														std::cout<<"Distance alignment: "<<AlignF<<std::endl;

									  //if(AlignF == -1)
									  //{
									  //  flag_exit = true;                   
									  //}
													}

									if(!flag_exit)
									{
									  // Changing the height to do the insertion after alignment
									  LoCAlign[2]=86;
									  double alex=PrimBodySchema(95,0,0,0,76,0);
									}
	/*AlignHandFlag=1;
	Sleep(30000);
	RefreshPlacemap();

	double xdiff = (XPosition[0]-LoCAlignFusee[0]);
	double ydiff = (XPosition[1]-LoCAlignFusee[1]);
	double distance = (sqrt(pow(xdiff,2)+ pow(ydiff,2)));
	std::cout<<"Distance : "<<distance<<std::endl;
		LoCAlignHand[0]=XPosition[0]+xdiff;
		LoCAlignHand[1]=XPosition[1]+ydiff;
		LoCAlign[2]=145;
		double alex=PrimBodySchema(95,0,0,0,0,0);
	return distance;*/
									return 1;
	};

void ObserverThread::LoadGWSArgument()
	{
		UserInterface(41);
		GetLocalAct(NumWords);
		int PropWeightC=1; // this can come as a result of the elimination/growth rule
		Retroactivate(PropWeightC);
		cout<<" Initializing DARWIN Working memory " <<endl;
		InitializeWorkingMemory(1);
	}

void ObserverThread::Mergence(){
    int iCo, jCo, NoBReplan=0;
	int TempSeqAcP[10];
//	if(CannotFind==1){
//		NPiCs=NPiCs-1;
//		}
	for(iCo=0; iCo<NPiCs; iCo++)
			{
               TempSeqAcP[iCo]=SeqAcP[iCo];
			   cout<< "Interpreting New plan"<< TempSeqAcP[iCo] <<endl;
			   Report<< "Mergence Interpreting New plan"<< TempSeqAcP[iCo] <<endl;
			   SeqAcP[iCo]=50;
       	 }
        SeqAcP[0]=PPiCo;
		NoBReplan=NoBReplan+1;
		cout<< "Connecting element of past plan"<< SeqAcP[0] <<endl;
		Report<< "Connecting element of past plan"<< SeqAcP[0] <<endl;
		int present;
        for(iCo=0; iCo<NPiCs; iCo++)
			{
				present=0;
				for(jCo=0; jCo<numberpast+1; jCo++){
					if(PastPlan[jCo]==TempSeqAcP[iCo])
					{
					 cout<<"Object" << TempSeqAcP[iCo] << "is common"<< endl;
					 Report<<"Object" << TempSeqAcP[iCo] << "is common"<< endl;
					 present=1;
					}
				}
				if((present==0)&&(TempSeqAcP[iCo]!=SeqAcP[0])){
                    cout<<"Using Object"<<TempSeqAcP[iCo]<<"in the new plan" <<endl;
					SeqAcP[NoBReplan]=TempSeqAcP[iCo];
					NoBReplan=NoBReplan+1;
				}
		    }
		NPiCs=NoBReplan;

}

int ObserverThread::PickandPlace(int pick, int place, int seqNumber) {

	int resPandP=0;
	double PMPRepl=0, PMPReplPlace=0,GraspO=-1, GraspC=-1, GraspR=-1,PMPReplInit=0,PMPReplMicro=0;
	CannotFind=-1, Cumulate=0;
	findsuccess=0;
	findsuccess2=0;
	if(GiD==14){
	/*findsuccess=PrimSearch(pick,place,5);
		if(findsuccess == 0)
		{
			findsuccess=PrimSearch(pick,place,5);
			if(findsuccess == 0)
			{
				findsuccess=PrimSearch(pick,place,5);
			}
		}*/
        cout<<" Waiting for findsuccess " <<endl;
		cin >> findsuccess;  //VM extended test 0104

	}
	if(GiD==32){
	findsuccess=PrimSearch(pick,place,68);
	}
	cout<<"FIND SUCCESS IS : "<<findsuccess<<endl;
	Report<<"FIND SUCCESS IS : "<<findsuccess<<endl;

//    int findsuccess=1;
	if (findsuccess==1)
		{
		  //
			if(GiD==14){
			cout<< "Both Objects involved in the present micro sequnce are there " << endl;
			Report<< "Both Objects involved in the present micro sequnce are there "<<pick << place << endl;
			}
			cout<< "Contacting Grasp server to open gripper " << endl; //open
			Report<< "Contacting Grasp server to open gripper " << endl;
			Cumulate=Cumulate+1	;

    /*        GraspO=PrimGrasp(2); //commented 0603 // opening of gripper */

			GraspO = 5;//commented 0603 *************************VM01/04
			if(GraspO==5)
				{
					cout<< "Contacting PMP server ........VOCAB REA " << endl;
					Report<< "Contacting PMP server ........VOCAB REA " << endl;
                    Cumulate=Cumulate+1	;
					cout<<"THE OBJECT ID:  "<<GetObjIDs[0]<<std::endl;

				//	PMPRepl=PrimBodySchema(1,GetObjIDs[0],1,pick);// contains arguements for reach object pick;
                    cout<<" Waiting for PMPRepl " <<endl;
					cin >>PMPRepl;

					cout<<"PMPRepl: "<<PMPRepl<<std::endl;
					Report<<"PMPRepl received from Prim Body schema is: "<<PMPRepl<<std::endl;
					//if this is sucessful, grasp the object, init the arm, find if the object is still there in the scene
					//if all these actions are sucessful
				}
			if(PMPRepl==1)
				{
                 cout<< "Contacting Grasp server to Grip the object " << endl; //open
				 Report<< "Contacting Grasp server to Grip the object " << endl; //open
				 Cumulate=Cumulate+1;
            /*     GraspC=PrimGrasp(1); //commented 0603 */
                  cout<< "Waiting for GraspC " << endl;
                  cin>>GraspC;
			    }
		////commented 0603
			//GraspC = 5;
			if(GraspC==5)
				{
                 GraspC=4;// added VM 23/03
                 cout<< "Contacting PMP server to init and Check sucess of Pick " << endl; //open
				 Report<< "Contacting PMP server to init and Check sucess of Pick " << endl; //open
				 Cumulate=Cumulate+1;
				 //Time::delay(10);

             /*    PMPReplInit=PrimBodySchema(19,GetObjIDs[0],1,pick); */ //**********VM01/04

				 //Time::delay(10);
//*****************************23/03/: Modified this part so as not not route through vision..but Gripper instead.......
				 //**My also have to add one line in PMP Module..if necessary...
	/*			 PrimSearch(pick,place,68);
				 int checkDisp=GetObjIDs[0];
                 if ((checkDisp != 50) && (PlaceMap[checkDisp][cannotfindXLoc]!=cannotfindX)&&(GraspC==5))
				 {
				  CannotFind=0;
				 }
                 if(checkDisp==50)
                 {
				  CannotFind=0;
				 } */

			//	 GraspC=PrimGrasp(1);  //checking closing again after arm has initialized..VM 23/03

				 GraspC=5; //VM01/04
				 if(GraspC==5){
                 CannotFind=0;
				 cout<<"Object is coupled to the gripper : "<<CannotFind<<endl;
				 Report<<"Object is coupled to the gripper : "<<CannotFind<<endl;
				 }

				 //you need to refresh placemap here and check if the picked object ID is no longer there
			    }

			//============================bug correct
			if(GraspC==4){
                 CannotFind=1;
				 cout<<"Object has slipped from the fingers : "<<CannotFind<<endl;
                 Report<<"Object has slipped from the fingers : "<<CannotFind<<endl;
				 }
			//===========================================================

			//CannotFind = 0;
			if (CannotFind==0){
			//RefreshPlacemap();
            Cumulate=Cumulate+1	;
			if(GiD==14){
			//Time::delay(10);
		/*	findsuccess2=PrimSearch(place,pick,68);
			if(findsuccess2 == 0)
			{
				findsuccess2=PrimSearch(place,pick,68);
				if(findsuccess2 == 0)
				{
					findsuccess2=PrimSearch(place,pick,68);
				}
			}*/
            cout<<" Waiting for findsucess2 " <<endl;
            cin>>findsuccess2; //VM04/01
			cout<< "reaching object " << GetObjIDs[1]<<" , " << GetObjIDs[0] << endl;
			Report<< "reaching object to place " << GetObjIDs[1]<<" , " << GetObjIDs[0] << endl;
			}
			if(GiD==32){
			findsuccess2=1;
			cout<< "Placing object "<< endl;
			Report<< "Placing object "<< endl;
			}
				if(findsuccess2==1){
						 if(GiD==14){

					//	PMPReplPlace=PrimBodySchema(1,GetObjIDs[0],3,pick);// contains arguements for reach object Place;	//was place before
					    cout<<" Waiting for PMPReplPlace " <<endl;
						cin>>PMPReplPlace;
						//VM01/04
							 //if this is sucessful, Release the object, init the arm, find if the object with ID pick is now there in the scene
						//and approximately allined in the z-dimension with object place
						 }
						 if(GiD==32){

							StaticLoc[0]=550;  //Check the start position to place
							StaticLoc[1]= 450;
							if(pick == 2)
							{
								StaticLoc[2]=125+90;
							}
							else
							{
								StaticLoc[2]=95+90;
							}

						  PMPReplPlace=PrimBodySchema(21,GetObjIDs[0],1,GetObjIDs[0],0,1);// contains arguements for reach object Place;
						//if this is sucessful, Release the object, init the arm, find if the object with ID pick is now there in the scene
						//and approximately allined in the z-dimension with object place


							//cout<<"LOCATION where I am placing object "<<pick<<
								//"is"<<StaticLoc[0]<<StaticLoc[1]<<StaticLoc[2]<<endl;
						 }
				   }
			}

            if(PMPReplPlace==1)
			    {
					cout<< "Contacting Grasp server to stack/release " << endl; //open
					Report<< "Contacting Grasp server to stack/release " << endl; //open
					Cumulate=Cumulate+1	;
                   // GraspR=PrimGrasp(2); // releasing opening gripper
                   GraspR=5;//
			    }
			//GraspR = 1;
            if(GraspR==5)
			    {

                  Cumulate=Cumulate+1;
				  Report<<"Entered initializing loop : "<<GraspR<<endl;
				   cout<<"Entered initializing loop : "<<GraspR<<endl;
			//	  PMPReplMicro=PrimBodySchema(19,GetObjIDs[1],1,GetObjIDs[1]); //init the arm and go back to the master to see what next
				  //VM01/04

				   Report<<"out of PMPREPLMICRO : "<<PMPReplMicro<<endl;
				  cout<<"out of PMPREPLMICRO : "<<PMPReplMicro<<endl;
				  //Time::delay(10);
			    }
        // here you get a cumulative score of sucess of vision, PMP and Grasp,
		//this has to be fed back to the Observer to execute the next Usequnce
		}

	if (findsuccess==0)
		{
		cout<< "Sense failure in finding object to pick up" << endl;
		Report<< "Sense failure in finding object to pick up" << endl;
		Cumulate=0;
		}

	if ((findsuccess2==0)&&(findsuccess!=0))
		{
		 cout<<"Sense failure in finding object on which I need to place the picked object"<<endl;
		 Report<<"Sense failure in finding object on which I need to place the picked object"<<endl;
		Cumulate=0;
		}

	if ((PMPRepl==2)&&(findsuccess==1))
		{
		cout<<"Sense failure in reaching the object I need to pick up"<<endl;
		Report<<"Sense failure in reaching the object I need to pick up"<<endl;
		Cumulate=4;  // object to be picked is not reachable: Comment VM 23/03^
		}

	if ((PMPReplPlace==2)&&(PMPRepl==1)&&((CannotFind!=1)))
		{
		cout<<"Sense failure in Placing"<<endl;
		Report<<"Sense failure in Placing"<<endl;
		Cumulate=0;
		}

	if ((CannotFind==1)&&(PMPRepl==1))
		{
			cout<<"Sense failure: The scene is not refreshed/Object has slipped our: CannotFind"<<CannotFind<<endl;
		Report<<"Sense failure: The scene is not refreshed/Object has slipped our"<<endl;
		Cumulate=0; // object picked up has slipped  Comment VM/23/03
		}

	if ((PMPRepl==1)&&(PMPReplPlace==1)&&(CannotFind==0)&&(findsuccess==1)&&(findsuccess2==1)){
		Cumulate=1;
	}
    cout<<"CUMULATE SCORE"<<Cumulate<<endl;

	if(GiD==32){
 Cumulate=findsuccess;
 }

return Cumulate; // VM transformed the findsucess score into a cumualtive score that relates to all micro events
}

//This is for the industrial robot
//int PMPGoalCode, int OIDinPM,int PIdentifier,int ObjectIDPMP

double ObserverThread::PrimBodySchemaIndustrial(int PMPGoalCode,int OIDinPM,int PIdentifier, int MsimFlag, int WristOrient, int TrajType){

                    	Network::connect("/BodySchemaSim:io", "/pmpRX/PMPreply:io");
						cout << "Inside the PrimBodySchema : " << endl;
						Report << "Inside the PrimBodySchema : "  << endl;
                        int iCo=0, iCocord=-1,cannotfindXLoc=100;
						cannotfindX=0;
                        for(int i=2;i<18;i=i+3)
							{
							  if((PlaceMap[OIDinPM][i])>iCocord) //checks 2-5-8-11-14..
								  {
								     iCocord=PlaceMap[OIDinPM][i]; //z coordinate
                                     iCo=i;
								  }
							}

						Bottle BodySchema_cmd, BodySchema_response;
						BodySchema_cmd.addVocab(COMMAND_VOCAB_REA);
						BodySchema_cmd.addInt(PMPGoalCode);
						BodySchema_cmd.addInt(MsimFlag);
						BodySchema_cmd.addInt(WristOrient);
						BodySchema_cmd.addInt(TrajType);
						BodySchema_cmd.addInt(ObjIDEE[PlaceMapPos]);
						//the rest of information will be replaced by correct numbers coming from Place Map (that is a event driven
						//working memory keeping track of what thngs are there and where they are in the world)
						if(PMPGoalCode==1){
							BodySchema_cmd.addDouble(PlaceMap[PlaceMapPos][0]);
							cannotfindX=PlaceMap[PlaceMapPos][1];
							cannotfindXLoc=iCo-2;
							BodySchema_cmd.addDouble(PlaceMap[PlaceMapPos][1]);
							if(PIdentifier==1)
							{
								BodySchema_cmd.addDouble(PlaceMap[PlaceMapPos][2]);
							}
							/*if(PIdentifier==3)
							{
								if(OIDinPM == 2)
								{
									BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+123);
								}
								else
								{
									BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+93);
								}
							}*/
						}
						if(PMPGoalCode==95){
							BodySchema_cmd.addDouble(LoCAlign[0]);
							BodySchema_cmd.addDouble(LoCAlign[1]);
							BodySchema_cmd.addDouble(LoCAlign[2]);
											}

						if(PMPGoalCode==19){

						cout<<"Entered Goal of Initialization " <<endl;
						Report<<"Entered Goal of Initialization " <<endl;
						BodySchema_cmd.addDouble(XPosition[0]);
						BodySchema_cmd.addDouble(XPosition[1]);
						BodySchema_cmd.addDouble(XPosition[2]);
						cout<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
						Report<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
						}
						if(PMPGoalCode==21){
						BodySchema_cmd.addDouble(StaticLoc[0]);
						BodySchema_cmd.addDouble(StaticLoc[1]);
						BodySchema_cmd.addDouble(StaticLoc[2]);
						}
						BodySchema_cmd.addDouble(0);
						BodySchema_cmd.addDouble(0);
						BodySchema_cmd.addDouble(0);
						// for ComputeTheta
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][0]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][1]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][3]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][4]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][6]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][7]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][9]);
						BodySchema_cmd.addDouble(PlaceMap[OIDinPM][10]);
						BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

						cout<<"Sent a request to PMP Server with PMPGoalCode : "<<PMPGoalCode<<endl;
						Report<<"Sent a request to PMP Server with PMPGoal Code : "<<PMPGoalCode<<endl;
						printf("%s \n",BodySchema_response.toString().c_str());

						double Bresponsecode = BodySchema_response.get(0).asDouble();
					    cout<<Bresponsecode<<endl;

					if(Bresponsecode == 221 /*COMMAND_VOCAB_REACH*/) {
						 cout << "Receiving status from Body Schema PMP server" << endl;
						 Report << "Receiving status from Body Schema PMP server" << endl;
						 for(int i=0;i<10;i++)
							{
							 PMPresp[i]= BodySchema_response.get(i+1).asDouble();
							 cout << "Resp from PMP server" << PMPresp[i]<< endl;
							}
						 XPosition[0]=PMPresp[1];
						 XPosition[1]=PMPresp[2];
						 XPosition[2]=PMPresp[3];
						 cout<<" Forward model output of arm position"<< endl;
						 cout<< XPosition[0] << XPosition[1] << XPosition[2] << endl;
						 if(PMPresp[0]==1){
						 cout << "Reached goal object sucessfully: wait for next goal from client" << endl;
						 Report << "Reached goal object sucessfully: wait for next goal from client" << endl;
						 }

						 //if(PMPresp[0]==0){
							// cout << "Goal is not doable: need to form a updated plan with help of EPIM" << endl;
						 //}
					}
					Network::disconnect("/BodySchemaSim:io", "/pmpRX/PMPreply:io");
return PMPresp[0];
}

double ObserverThread::PrimBodySchema(int PMPGoalCode, int OIDinPM,int PIdentifier,int MsimFlag, int WristOrient, int TrajType){

    if(!(Network::isConnected("/BodySchemaSim:io", "/PMP/PMPreply:io"))){
			Network::connect("/BodySchemaSim:io", "/PMP/PMPreply:io");
		}    				
	//Network::connect("/BodySchemaSim:io", "/PMP/PMPreply:io");
						cout << "Inside the PrimBodySchema : " << endl;
						Report << "Inside the PrimBodySchema : "  << endl;

                        int iCo=0, iCocord=-1,cannotfindXLoc=100;
						cannotfindX=0;
						robToCamX=-480,robToCamY=-180;
						offsetX=0,offsetY=0; //offset may be 30 to 40 positive or negative

           /*             for(int i=2;i<18;i=i+3)
							{
							  if((PlaceMap[OIDinPM][i])>iCocord) //checks 2-5-8-11-14..
								  {
								     iCocord=PlaceMap[OIDinPM][i]; //z coordinate
                                     iCo=i;
								  }
							}  */  //This is commented as I dont have 18 3D positions

						Bottle BodySchema_cmd, BodySchema_response;
						BodySchema_cmd.addVocab(COMMAND_VOCAB_CACT);
						if(PMPGoalCode==95){
						BodySchema_cmd.addInt(1); // Reach 1 or Initialize 19 or 21 Conitnuous reaching
						}
						else
						{
						BodySchema_cmd.addInt(PMPGoalCode);
						}
						BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);
						//the rest of information will be replaced by correct numbers coming from Place Map (that is a event driven
						//working memory keeping track of what thngs are there and where they are in the world)
						BodySchema_cmd.clear();
						BodySchema_response.clear();
						if((PMPGoalCode==1)||(PMPGoalCode==95)){
							BodySchema_cmd.addVocab(COMMAND_VOCAB_MICG);
							Bottle& Coordinates = BodySchema_cmd.addList();
							if(PMPGoalCode==1){
								Coordinates.addDouble(robToCamX+PlaceMap[PlaceMapPos][0]-5); 
								if((PrimPushExecFlag==0)&&(PrimPushSideFlag==0)){
									if(PlaceMap[PlaceMapPos][1]<-robToCamY){
										Coordinates.addDouble(robToCamY+PlaceMap[PlaceMapPos][1]+10);
									}
									else{
										Coordinates.addDouble(robToCamY+PlaceMap[PlaceMapPos][1]+10);  //219 offset before demo
									}//should be 168 but put a constant offset in x       //192 on 15 feb
								}
								if(PrimPushExecFlag==1)
								{
							  		Coordinates.addDouble(PlaceMap[PlaceMapPos][1]);
								}
								if(PrimPushSideFlag==1)
								{
							  		Coordinates.addDouble(PlaceMap[PlaceMapPos][1]);
									PrimPushSideFlag=0;
								}
								
								if(ObjIDEE[PlaceMapPos]==101)
								{
									Coordinates.addDouble(55); //PlaceMap[PlaceMapPos][2];//35 on 15 feb //50 on 18 feb
								}

								if(ObjIDEE[PlaceMapPos]==100)
								{	
									if(PrimPushFlag==1)
									{
										Coordinates.addDouble(-15);
										PrimPushFlag=0;
									}
									else
									{
										Coordinates.addDouble(150);
									}

								}
							}
							if(PMPGoalCode==95){
								Coordinates.addDouble(robToCamX+LoCAlign[0]);
								Coordinates.addDouble(robToCamY+LoCAlign[1]);//192
								Coordinates.addDouble(LoCAlign[2]+14); //has to be checked
							}
							BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

							BodySchema_cmd.clear();
						    BodySchema_response.clear();
							BodySchema_cmd.addVocab(COMMAND_VOCAB_MSIM);
							BodySchema_cmd.addInt(MsimFlag);
							BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

							BodySchema_cmd.clear();
						    BodySchema_response.clear();
							BodySchema_cmd.addVocab(COMMAND_VOCAB_WRIO);
							BodySchema_cmd.addInt(WristOrient);
							BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

							BodySchema_cmd.clear();
						    BodySchema_response.clear();
							BodySchema_cmd.addVocab(COMMAND_VOCAB_TRAT);
							BodySchema_cmd.addInt(TrajType);
							BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);


							BodySchema_cmd.clear();
						    BodySchema_response.clear();
							BodySchema_cmd.addVocab(COMMAND_VOCAB_REA);
						    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);
							/*	if(PIdentifier==3)
								{
									if(ObjectIDPMP == 2)
									{
										BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+123);
									}
									else
									{
										BodySchema_cmd.addDouble(PlaceMap[OIDinPM][iCo]+93);
									}
								}*/ //This was for the mushroom Offset

						}

						if(PMPGoalCode==19){

							cout<<"Entered Goal of Initialization " <<endl;
							Report<<"Entered Goal of Initialization " <<endl;

						    BodySchema_cmd.clear();
						    BodySchema_response.clear();
							BodySchema_cmd.addVocab(COMMAND_VOCAB_INIT);
						    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

							BodySchema_cmd.clear();
						    BodySchema_response.clear();
							BodySchema_cmd.addVocab(COMMAND_VOCAB_MSIM);
							BodySchema_cmd.addInt(MsimFlag);
							BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);

						    BodySchema_cmd.clear();
						    BodySchema_response.clear();
							BodySchema_cmd.addVocab(COMMAND_VOCAB_REA);
						    BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);


						/*BodySchema_cmd.addDouble(XPosition[0]);
						BodySchema_cmd.addDouble(XPosition[1]);
						BodySchema_cmd.addDouble(XPosition[2]);
						cout<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;
						Report<<"Entered Goal of Initialization X1,X2,X3 : "<<XPosition[0]<<" , "<<XPosition[1]<<" , "<<XPosition[2] <<endl;*/
						}



						//continuous reachign through a via point
						if(PMPGoalCode==21){
							BodySchema_cmd.addDouble(StaticLoc[0]);
							BodySchema_cmd.addDouble(StaticLoc[1]);
							BodySchema_cmd.addDouble(StaticLoc[2]);
							BodySchema_cmd.addDouble(StaticLoc[3]);
							BodySchema_cmd.addDouble(StaticLoc[4]);
							BodySchema_cmd.addDouble(StaticLoc[5]);
						}
						//BodySchema_cmd.addDouble(0);
						//BodySchema_cmd.addDouble(0);
						//BodySchema_cmd.addDouble(0);
						//// for ComputeTheta
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][0]);
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][1]);
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][3]);
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][4]);
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][6]);
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][7]);
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][9]);
						//BodySchema_cmd.addDouble(PlaceMap[OIDinPM][10]);
						//BodySchemaCtrlPort.write(BodySchema_cmd,BodySchema_response);
						cout<<"Sent a request to PMP Server with PMPGoalCode : "<<PMPGoalCode<<endl;
						Report<<"Sent a request to PMP Server with PMPGoal Code : "<<PMPGoalCode<<endl;
						printf("%s \n",BodySchema_response.toString().c_str());

						double Bresponsecode = BodySchema_response.get(0).asDouble();
					    cout<<Bresponsecode<<endl;

					if(Bresponsecode == 221 /*COMMAND_VOCAB_REACH*/) {
						 cout << "Receiving status from Body Schema PMP server" << endl;
						 Report << "Receiving status from Body Schema PMP server" << endl;
						 for(int i=0;i<14;i++)
							{
							 PMPresp[i]= BodySchema_response.get(i+1).asDouble();
							 cout << "Resp from PMP server" << PMPresp[i]<< endl;
							}
						 XPosition[0]=PMPresp[1]-robToCamX;//robToCamY+PlaceMap[PlaceMapPos][0]+20
						 XPosition[1]=PMPresp[2]-robToCamY;
						 XPosition[2]=PMPresp[3];
						 cout<<" Forward model output of arm position"<< endl;
						 cout<< XPosition[0] << XPosition[1] << XPosition[2] << endl;
						 if(PMPresp[0]==1){
						 cout << "Reached goal object sucessfully: wait for next goal from client" << endl;
						 Report << "Reached goal object sucessfully: wait for next goal from client" << endl;
						 }

						// if(PMPresp[0]==0){
						//	 cout << "Goal is not doable: need to form a updated plan with help of EPIM" << endl;
						// }
					}
return PMPresp[0];
}


double ObserverThread::PrimSearch(int obj1, int obj2, int goalidentity){

	RefreshPlacemap(); // connect to the bottle that brings in visual information and refresh the place map
	int iCo, jCo, niterFind=1, goalFind[2];
	double FindResp=0;
	if(goalidentity==68){
	  goalFind[0]=obj1;
	}
	if (goalidentity==5){
		niterFind=2;
		goalFind[0]=obj1;
		goalFind[1]=obj2;
	}

	if((goalidentity==5)||(goalidentity==68)){
    for(iCo=0; iCo<niterFind; iCo++)
			{
				GetObjIDs[iCo]=50;
				for(jCo=0; jCo<NumObjectsinScene; jCo++)
						 {
						   if(ObjIDEE[jCo]==goalFind[iCo])
						   GetObjIDs[iCo]=jCo;
						 }
				if(niterFind==1)
				{
				  if(GetObjIDs[iCo]!= 50)
					  {
					   FindResp=1;
					   cout <<"object is there" << endl;
					  }
				}

				if(niterFind==2)
				{
				  if(GetObjIDs[iCo]!= 50)
					  {
					   FindResp=FindResp+0.5;
					   cout <<"object" << iCo << "is there" << endl;
					  }
				}

			}
	}

	if(goalidentity==95)
			{
              // u now need to transform all object ID's into learnt object hub activations to bring compositionlity and flexibility
            PlaceMapPos=10;
			//PlaceMapHub[0][0]=1; //this must come due to top down retroactivation from user goal given in proto language: now is blue driver
			//PlaceMapHub[0][6]=1;
			  int SumOfGWS=0;
			   double sumPh;
			  for(iCo=0; iCo<NumObjectsinScene; iCo++)
				   		 {
							sumPh=0;
							for(jCo=0; jCo<42; jCo++)
									 {
									   sumPh=sumPh+((GlobalWorkSpace[GWSPtr][jCo]-PlaceMapHub[iCo][jCo])*(GlobalWorkSpace[GWSPtr][jCo]-PlaceMapHub[iCo][jCo]));
									 }
							if(sumPh==0)
							{
							  cout <<"object is there at PlaceMap Location" << iCo << endl;
							  PlaceMapPos=iCo;
							  break;
							}
			  }
			  if(NumObjectsinScene==0)
				                    {
										for(jCo=0; jCo<42; jCo++)
											 {
											   SumOfGWS=SumOfGWS+(GlobalWorkSpace[GWSPtr][jCo]);
											 }
										if(SumOfGWS>0)
											 {
											   sumPh=1;
											 }
									 }

		     FindResp=sumPh;

			}
   return FindResp;
}

int ObserverThread::PrimGrasp(GraspTypeType GraspReq,int BodyChain){
						return 1;
}

int ObserverThread::PrimGraspIndustrial(GraspTypeType GraspReq,int BodyChain){
   return 1;
}

double ObserverThread::RefreshPlacemap(){
	//bool flag = true;

	//while(flag)
	//{
  Network::connect("/SmallWorldsOPC:io", "/OPCServer:io");
  Bottle OPC_cmd, OPC_Response;

			 OPC_cmd.addVocab(COMMAND_VOCAB_FIND);
			 OPCCtrlPort.write(OPC_cmd,OPC_Response);
			 printf("%s \n",OPC_Response.toString().c_str());

			 for(int i=0;i<10;i++)
					{
						for(int j=0;j<18;j++)
							{
							  PlaceMap[i][j]=0;
							}
					  }
				 for(int i=0;i<10;i++)
					{
						ObjIDEE[i]=50; //null object
						XlatTrackP[i]=50;
						XlatTrackPl[i]=50;
					}
				 NumberofObs=0;
				 int ctrr=0;
	//==========================================================================================
              int OPCresponsecode = OPC_Response.get(0).asVocab();
			   ctrr=ctrr+1;
			  cout<<OPCresponsecode<<endl;

					if(OPCresponsecode == 428 /*COMMAND_VOCAB_FIND*/) {
						 cout << "Receiving status from OPC server" << endl;
						 Report << "Receiving status from OPC server" << endl;

//=============================================================================================
						 NumberofObs = OPC_Response.get(ctrr).asInt();
						/* if(NumberofObs > 0)
						 {
							 flag = false;
						 }*/
						 NumObjectsinScene=NumberofObs;
							   ctrr=ctrr+1;
							   for(int i=0;i<NumberofObs;i++)
									{
										ObjIDEE[i]=OPC_Response.get(ctrr).asDouble();
										XlatTrackP[i]=ObjIDEE[i];
										XlatTrackPl[i]=ObjIDEE[i]; //buggy things are there
										cout << "Object ID received from OPC server" << ObjIDEE[i] << endl;
										Report << "Object ID received from OPC server" << ObjIDEE[i] <<endl;
									//	Graspability[i]=OPC_Response.get(ctrr).asDouble();
										ctrr=ctrr+1;
									}

							   for(int i=0;i<NumberofObs;i++)
									{
										cout << "Graspability status received from OPC server" << ObjIDEE[i] << endl;
										Report << "Graspability status  received from OPC server" << ObjIDEE[i] <<endl;
										Graspability[i]=OPC_Response.get(ctrr).asDouble();
										ctrr=ctrr+1;
									}

								for(int i=0;i<NumberofObs;i++)
									{
                                        Report << "place map coordinates of object " << i << endl;
										for(int j=0;j<Graspability[i]*3;j++)
											{
												PlaceMap[i][j]=OPC_Response.get(ctrr).asDouble();
												Report << PlaceMap[i][j]<< endl;
												ctrr=ctrr+1;
												if((ObjIDEE[i]==100))  //at present we are only taking care of pushing while reaching and not aligning: this can be added by making some changes to scene change function
													{
												       if(PlaceMap[i][1]<-robToCamY){
														   PeriPersonalFB=4;
													   }
													   if(PlaceMap[i][1]>=-robToCamY){
														    PeriPersonalFB=5;  //trigger RX for fusebox
													   }
													   //change pushh
													   PosHandOcc[0]=PlaceMap[i][0];
													   PosHandOcc[1]=PlaceMap[i][1];
													   PosHandOcc[2]=55;
													}
												if((ObjIDEE[i]==101))
													{
												       if(PlaceMap[i][1]<-robToCamY){
														 PeriPersonalF=4;
													   }
													   if(PlaceMap[i][1]>=-robToCamY){
														 PeriPersonalF=5;  //trigger RX for fuse
													   }
												
													}
											}
										if((AlignFlag==1)&&(ObjIDEE[i]==100))
											{
												LoCAlign[0]=PlaceMap[i][0];
												LoCAlign[1]=PlaceMap[i][1];
												LoCAlign[2]=PlaceMap[i][2];
											}
										if((AlignFlag==1)&&(ObjIDEE[i]==101))
											{
												LoCAlignFusee[0]=PlaceMap[i][0];
												LoCAlignFusee[1]=PlaceMap[i][1];
												LoCAlignFusee[2]=PlaceMap[i][2];
											}

								     }
//==============================================================================================

					}
					Xlator();
	//}
   Network::disconnect("/SmallWorldsOPC:io", "/OPCServer:io");
	return NumberofObs;

}

void ObserverThread::Interpret(){

	NPiCs=0;
	for(int iCo=1; iCo<SeqAc[0]+1; iCo++)	{

				if((SeqAc[iCo]==20))
							   {
								   for(int jCo=0; jCo<numcu; jCo++){
									SeqAcP[NPiCs]=NumCubID[jCo];
									cout<< "found cube" <<SeqAcP[NPiCs] <<endl;
									NPiCs=NPiCs+1;
				                     }
							   }
				if((SeqAc[iCo]==13))
							   {
								   for(int jCo=0; jCo<numcy; jCo++){
									SeqAcP[NPiCs]=NumCylID[jCo];
									cout<< "found cyli" <<SeqAcP[NPiCs] <<endl;
									NPiCs=NPiCs+1;
				                     }
							   }
				if((SeqAc[iCo]==18))
								{
								 SeqAcP[NPiCs]=7;
								 NPiCs=NPiCs+1;
								}
				if((SeqAc[iCo]==35))
								{
								 SeqAcP[NPiCs]=NumMushID[0];
								 NPiCs=NPiCs+1;
								}

	}
	cout<<"NPiCs Interpreter"<<NPiCs<<endl;

}



void ObserverThread::Xlator(){
	        int m,n,
            largeness=0;
			numcu=0;
			numcy=0;
			nummu=0;
			NumCylID[0]=50;
			NumCylID[1]=50;
			NumCubID[0]=50;
			NumCubID[1]=50;
			NumCubID[2]=50;
			NumberofObsE=0;
			for (m =0; m<10; m++)
				{
					for (n=0; n<42; n++)
					{
						 PlaceMapHub[m][n]=0;
					}
				}

			int delnumcy=0,delnumcu=0;
			int CountEP=0;
			for(int i=0;i<NumberofObs;i++)
				{
					if ((ObjIDEE[i]==2)||(ObjIDEE[i]==5))
					   {
						   if(nummu==0){
							ObjIDEEEpim[CountEP]=35; //Mushroom
							CountEP=CountEP+1;
							}
						   XlatTrackP[i]=ObjIDEE[i];
						   XlatTrackPl[i]=ObjIDEE[i];
						   NumMushID[nummu]=ObjIDEE[i];
                           nummu=nummu+1;
							if(nummu==1)
							{
								NumberofObsE=NumberofObsE+1;
							}
					   }
					if ((ObjIDEE[i]==0))
					   {
						ObjIDEEEpim[CountEP]=7; //fuse
						//ObjIDEEEpim[NumberofObs+1]=37;
						CountEP=CountEP+1;
						XlatTrackP[i]=ObjIDEE[i];
						XlatTrackPl[i]=ObjIDEE[i];
						largeness=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"Large object"<<endl;
					   }
					if ((ObjIDEE[i]==3)||(ObjIDEE[i]==6))
					   {
							if(numcy==0){
							ObjIDEEEpim[CountEP]=13; //Cylinder
							CountEP=CountEP+1;
							}
						XlatTrackP[i]=ObjIDEE[i];
						XlatTrackPl[i]=ObjIDEE[i];
						NumCylID[numcy]=ObjIDEE[i];
                        numcy=numcy+1;
							if(numcy==1)
							{
								NumberofObsE=NumberofObsE+1;
							}
					   }

					if ((ObjIDEE[i]==15)||(ObjIDEE[i]==1)||(ObjIDEE[i]==4))
					   {
						   if(numcu==0){
							ObjIDEEEpim[CountEP]=20; //Cube
							CountEP=CountEP+1;
						   }
						XlatTrackP[i]=ObjIDEE[i];
						XlatTrackPl[i]=ObjIDEE[i];
						NumCubID[numcu]=ObjIDEE[i];
                        numcu=numcu+1;
								if(numcu==1)
								{
								   NumberofObsE=NumberofObsE+1;
								}
						cout<<"Numcu"<< numcu << "NumCubID[0]" << NumCubID[0]<< endl;
					   }

					if ((ObjIDEE[i]==100))
					   {
					    PlaceMapHub[i][9]=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"Fuse box found"<<endl;
					   }

					if ((ObjIDEE[i]==101))
					   {
					    PlaceMapHub[i][8]=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"Fuse found"<<endl;
					   }

					if ((ObjIDEE[i]==102))
					   {
					    PlaceMapHub[i][32]=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"Composite Object: FUSE BOX SET UP found"<<endl;
					   }

					if ((ObjIDEE[i]==120))
					   {
					    PlaceMapHub[i][14]=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"MECCANO Block found"<<endl;
					   }

					if ((ObjIDEE[i]==121))
					   {
					    PlaceMapHub[i][6]=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"MECCANO screwdriver found"<<endl;
					   }

					if ((ObjIDEE[i]==122))
					   {
					    PlaceMapHub[i][33]=1;
						NumberofObsE=NumberofObsE+1;
						cout<<"MECCANO Composite object found"<<endl;
					   }

			}
			cout<<"No of objects in the abstract neural representation:" << NumberofObsE << endl;

	}

void ObserverThread::InvXlator(int pi, int pl){

	PickMicro=50;
    PlaceMicro=50;
	if(pi==35){
	for(int i=0;i<NumberofObs;i++)
				{
					if (XlatTrackP[i]==2)
						{
						  PickMicro=2;
						  XlatTrackP[i]=50;
						}
	            }
	}

	if(pi==18){
		{
           PickMicro=6;
	     }
	}

	if(pi==13){
	for(int i=0;i<NumberofObs;i++)
				{
                   if ((XlatTrackP[i]==3)||(XlatTrackP[i]==5))
						{
						  PickMicro=XlatTrackP[i];
						  XlatTrackP[i]=50;
						  i=10;
						}
	            }
	}

	if(pi==20){
	for(int i=0;i<NumberofObs;i++)
				{
                  if ((XlatTrackP[i]==0)||(XlatTrackP[i]==1)||(XlatTrackP[i]==4))
						{
						  PickMicro=XlatTrackP[i];
						  XlatTrackP[i]=50;
						  i=10;
						}

	            }
	}
//================================================================================
	if(pl==35){
	for(int i=0;i<NumberofObs;i++)
				{
					if (XlatTrackPl[i]==2)
						{
						  PlaceMicro=2;
						  XlatTrackPl[i]=50;
						}
	            }
	}

	if(pl==18){
		{
           PlaceMicro=6;
	     }
	}

	if(pl==13){
	for(int i=0;i<NumberofObs;i++)
				{
                   if ((XlatTrackPl[i]==3)||(XlatTrackPl[i]==5))
						{
						  PlaceMicro=XlatTrackPl[i];
						  XlatTrackPl[i]=50;
						  i=10;
						}
	            }
	}

	if(pl==20){
	for(int i=0;i<NumberofObs;i++)
				{
                  if ((XlatTrackPl[i]==0)||(XlatTrackPl[i]==1)||(XlatTrackPl[i]==4))
						{
						  PlaceMicro=XlatTrackPl[i];
						  XlatTrackPl[i]=50;
						  i=10;
						}

	            }
	}

}

int ObserverThread::RetroactivateBodyHub(int PropWB)
	{
		//BodyHub[34]=1; //testing
		//BodyHub[35]=1;
		int iCo,jCo,pr_Bo;
		ActionChoice=50;
		for(iCo=0; iCo<12; iCo++)
				{
				     pr_Bo=0;
					 for(jCo=0; jCo<42; jCo++)
						{
							pr_Bo = pr_Bo + BodyHub2Acn[jCo][iCo]*BodyHub[jCo];
						}
					 ActionHub[iCo]=pr_Bo/(pr_Bo+0.001);
					 if(ActionHub[iCo]>0.50){
					 cout << "Xplorative Action:::::" << ActionHub[iCo] << "possible in relation to the present situation" <<endl;
					 ActionHubXplore[NXploreAct]=iCo;
					 NXploreAct=NXploreAct+1;
					 }
		}

		for(iCo=0; iCo<12; iCo++)
				{
					if(ActionHub[iCo]>0.5){
						if(ActionHub[iCo]-ActionHubXplore[iCo]>0.5){

							ActionChoice=iCo;
							break;
						}
					}
		}
		return ActionChoice;
	//	Sleep(15000);
	};

void ObserverThread::Retroactivate(int PropWC)
	{
      int iCo, jCo, mCo, sumWHM;

      if(PropWC==0)
		  {
		     for(iCo=0; iCo<36; iCo++)
				 {
					sumWHM=0;
					for(jCo=0; jCo<60; jCo++)
									 {
                                        sumWHM=sumWHM+MapstoProvHub[iCo][jCo];
									 }
                     if(sumWHM==2)
						 {
						    for(mCo=0; mCo<60; mCo++)
								 {
                                   MapstoProvHub[iCo][mCo]=0;
							     }
						 }
			     }

		  }

      for(iCo=0; iCo<500; iCo++)
		 {
			//WProvColShapRev*LocalAct': MapstoProvHub[36][90] Bottom up====================================
			 double maxlocal=0;
			 for(jCo=0; jCo<36; jCo++)
					 {
                       double pr_Co=0;
						 for(mCo=0; mCo<90; mCo++)
							{
                              pr_Co = pr_Co + MapstoProvHub[jCo][mCo]*LocalMapAct[mCo];
							}

						 ProvHub[jCo]=ProvHub[jCo]+0.015*((-0.01*ProvHub[jCo])+pr_Co);
						 if(ProvHub[jCo]<0.0001)
							 {
								 ProvHub[jCo]=0;
							 }


    				 }

             //==================================== WProvColShapRev'*u ===Top Down==============;

             for(jCo=0; jCo<90; jCo++)
					 {
                       double pr_CoT=0;
						 for(mCo=0; mCo<36; mCo++)
							{
                              pr_CoT = pr_CoT + ProvHubtoMaps[jCo][mCo]*ProvHub[mCo];
							}
						 LocalMapAct[jCo]=LocalMapAct[jCo]+1.3*((-0.01*LocalMapAct[jCo])+pr_CoT);
						 if(LocalMapAct[jCo]<0.0001)
							 {
								 LocalMapAct[jCo]=0;
							 }if(LocalMapAct[jCo]>maxlocal)
							 {
								 maxlocal=LocalMapAct[jCo];
							 }

					}

//GUI Commented out
	if (cwsPlot.getOutputCount()) {	
			if (iCo%50 == 0) {
			Bottle& cwsBot =cwsPlot.prepare();
			cwsBot.clear();
			cwsBot.addString("CWS");
			Bottle& cwsBotAll = cwsBot.addList();
			cwsBotAll.clear();
			for(int i=0; i<90; i++)
			{
				cwsBotAll.addDouble(LocalMapAct[i]/maxlocal);
			}
		//	cout<<"Sending  Color Word Shape hubs to DARWIN GUI"<<endl;
			cwsPlot.write();
			 Sleep(100);

			 }
	  }
    	 }

	  PushOID=0;
	   if(ProvHub[32]>=0.9)
	   {
	    PushOID=5;
		PushOIDAsm=14; //made change in night
	   }

	  for(iCo=0; iCo<42; iCo++)
		 {
          HubA << ProvHub[iCo]<< endl;
	     }
	   for(iCo=0; iCo<90; iCo++)
		 {
          MapA << LocalMapAct[iCo]<< endl;
	     }

	   //DARWIN GUI
		if (objectPlot.getOutputCount()) {	
		Bottle& objectBot =objectPlot.prepare();
		objectBot.clear();
		objectBot.addString("object");
		Bottle& objectBotAll = objectBot.addList();
		objectBotAll.clear();
		for(int i=0; i<42; i++)
		{
            objectBotAll.addDouble(ProvHub[i]);
		}
		cout<<"Sending Weight Matrix to DARWIN GUI"<<endl;
        objectPlot.write();
		}



	};

void ObserverThread::InitializeWorkingMemory(int GoalPointer)
	{
		int jCo;
		 for(jCo=0; jCo<42; jCo++)
		 {
			 if(ProvHub[jCo]>0){
				 GlobalWorkSpace[GoalPointer][jCo]=1;
			 }
			 if(ProvHub[jCo]<=0){
				 GlobalWorkSpace[GoalPointer][jCo]=0;
			 }

	     }
		 if((Goal_AcID==9))
				{
					 for(jCo=0; jCo<42; jCo++)
						 {
							  GlobalWorkSpace[0][jCo]=0;
							  GlobalWorkSpace[1][jCo]=0;
				        }
					  GlobalWorkSpace[0][8]=1;
					  GlobalWorkSpace[1][9]=1;
				  }

			if((Goal_AcID==3)&&(PushOID==5))
				{
					 for(jCo=0; jCo<42; jCo++)
						 {
							  GlobalWorkSpace[0][jCo]=0;
							  GlobalWorkSpace[1][jCo]=0;
							  GlobalWorkSpace[1][jCo]=0;
				        }
					  GlobalWorkSpace[0][32]=1;
					  GlobalWorkSpace[1][8]=1;
					  GlobalWorkSpace[2][9]=1;
					  //PushOID=14;
				  }
		 //this stores the anticipated object activation, placemap is initialized through refresh placemap-opc connection
	};

void ObserverThread:: GetLocalAct(int numW)
{
 int iCo, jCo, m,n, clocal=0;
  fileName = pathPrefix;
  fileName.append("MapI.txt");
  ofstream MapAini(fileName.c_str());
  MaxiAct=0.0001;
  MaxiActS=0.0001;
  MaxiActW=0.0001;
  /*for (n=0; n<30; n++)
		{
			 Col[n]=0;
			 Word[n]=0;
			 Shape[n]=0;
		}*/
	  	  for (n=0; n<90; n++)
					{
					 LocalMapAct[n]=0;
					}
		  for (n=0; n<42; n++)
					{
					 ProvHub[n]=0;
					}

  for(iCo=0; iCo<30; iCo++)
	 {
       Rdis=sqrt(pow(ColW[iCo][0]-ipCol[0],2)+pow(ColW[iCo][1]-ipCol[1],2)+pow(ColW[iCo][2]-ipCol[2],2));
	   RdisS=sqrt(pow(ShapW[iCo][0]-ipShap[0],2)+pow(ShapW[iCo][1]-ipShap[1],2)+pow(ShapW[iCo][2]-ipShap[2],2));
       RdisAct[iCo]=1*exp(-(Rdis)/40);
	   RdisActS[iCo]=1*exp(-(RdisS)/40);
       if(RdisAct[iCo]>MaxiAct)
		   {
			 MaxiAct=RdisAct[iCo];
			 // u can also have a indicatior here of which neuron is most active in col som
		   }
	   if(RdisActS[iCo]>MaxiActS)
		   {
			 MaxiActS=RdisActS[iCo];
			 // u can also have a indicatior here of which neuron is most active in col som
		   }
	   }

 for(iCo=0; iCo<30; iCo++)
				{
				     pr_Co=0;
					 pr_CoS=0;
					 for(jCo=0; jCo<30; jCo++)
						{
							pr_Co = pr_Co + inhib[iCo][jCo]*RdisAct[jCo];
							pr_CoS = pr_CoS + inhib[iCo][jCo]*RdisActS[jCo];
						}

                    Col[iCo]=(RdisAct[iCo]-(0.035*pr_Co))/MaxiAct;
					Shape[iCo]=(RdisActS[iCo]-(0.035*pr_CoS))/MaxiActS;
                    LocalMapAct[clocal]=Col[iCo];
					LocalMapAct[60+clocal]=Shape[iCo];
                    clocal=clocal+1;
				}

 //========================================================
//Words act
for(m=0; m<numW; m++)
	     {
			//RdistempW=0;
			MaxiActW=0.0001;
			for(iCo=0; iCo<30; iCo++)
				 {
                     RdistempW=0;
					 RdisW=0;
					 for(jCo=0; jCo<120; jCo++)
						{
							RdistempW = RdistempW + ((WorW[iCo][jCo]-WordIn[m][jCo])*(WorW[iCo][jCo]-WordIn[m][jCo])); //pow(WorW[iCo][jCo]-WordIn[m][jCo],2)
						}
				   RdisW=sqrt(RdistempW);
				   RdisActW[m][iCo]=5.64*exp(-1*(RdisW/0.25));
				   if(RdisActW[m][iCo]>MaxiActW)
					   {
						 MaxiActW=RdisActW[m][iCo];
					   }
				 }
			cout << MaxiActW << endl;
			//Sleep(20000);
			//===============================================================
       for(iCo=0; iCo<30; iCo++)
				{
				     pr_CoW=0;
					for(jCo=0; jCo<30; jCo++)
						{
							pr_CoW = pr_CoW + inhib[iCo][jCo]*RdisActW[m][jCo];
						}
                    WorActiv[m][iCo]=(RdisActW[m][iCo]-(0.03*pr_CoW))/MaxiActW;
					//WorActiv[m][iCo]=RdisActW[m][iCo]*10;
                 }

       } // m loop of words to be projected to the word SOM

   for(iCo=0; iCo<30; iCo++)
				{
					if(numW==1)
					{
					 LocalMapAct[30+iCo]=WorActiv[0][iCo];
					}
					if(numW==2)
					{
					 LocalMapAct[30+iCo]=WorActiv[0][iCo]+WorActiv[1][iCo];
					}
                 }

 //===== here u get the net initial bottom up neural activity: col, shap: by vision and word through keyboard ========
   for(iCo=0; iCo<90; iCo++)
		 {
          //MapAini <<WorActiv[0][iCo]<<endl;
		  MapAini << LocalMapAct[iCo]<< endl;
	     }
};

int ObserverThread::RetroactivateAcHub()
	{

   return 0;
	};


void ObserverThread::InitializeSW()
 {
    int m=0,n=0;
	//================== Initialize connectivity ==========================
    fileName = pathPrefix;
	fileName.append("WProvCSWRev.txt");
    ifstream WCSW(fileName.c_str());
	printf("%s\n",fileName.c_str());

	fileName = pathPrefix;
	fileName.append("wordWRev.txt");
	ifstream WorN(fileName.c_str());
	printf("%s\n",fileName.c_str());

	fileName = pathPrefix;
	fileName.append("colNeuronsW.txt");
	ifstream ColN(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("WHTM.txt");
	ifstream WHMW(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("ShapeNeuronsWRev.txt");
	ifstream SHWN(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("WBHubA.txt");
	ifstream WBHA(fileName.c_str());

	fileName = pathPrefix;
	fileName.append("WAP.txt");
	ifstream WAPR(fileName.c_str());

    for (m =0; m<36; m++)
				{
					for (n=0; n<90; n++)
					{
						 WCSW >> MapstoProvHub[m][n];
					}
				}

	for (m =0; m<42; m++)
				{
					for (n=0; n<12; n++)
					{
						 WBHA >> BodyHub2Acn[m][n];
					}
				}

	for (m =0; m<90; m++)
				{
					for (n=0; n<36; n++)
					{
						 WHMW >> ProvHubtoMaps[m][n];
					}
				}

	for (m =0; m<30; m++)
				{
					for (n=0; n<120; n++)
					{
						 WorN >> WorW[m][n];
					}
				}
cout << WorW[0][8]<< endl;

for (m =0; m<12; m++)
				{
					for (n=0; n<120; n++)
					{
						 WAPR >> WActionPrim[m][n];
					}
				}

cout << WActionPrim[0][8]<< endl;

for (m =0; m<30; m++)
				{
					for (n=0; n<3; n++)
					{
						 ColN >> ColW[m][n];
					}
				}

for (m =0; m<30; m++)
				{
					for (n=0; n<3; n++)
					{
						 SHWN >>	ShapW[m][n];
					}
				}

		for (m =0; m<10; m++)
				{
					for (n=0; n<42; n++)
					{
						 PlaceMapHub[m][n]=0;
					}
				}

	for (m =0; m<10; m++)
				{
					for (n=0; n<50; n++)
					{
						 GlobalWorkSpace[m][n]=0;
					}
				}
	for (m =0; m<10; m++)
				{
					for (n=0; n<50; n++)
					{
						 GoalStack[m][n]=50;
					}
				}
	for (n=0; n<20; n++)
					{
						 Encapsulate[n]=50;
					}

	for (m =0; m<20; m++)
				{
					for (n=0; n<50; n++)
					{
						 BottomUPTrace[m][n]=0;
						 Behavior[m][n]=0;
					}
				}


	for (n=0; n<120; n++)
					{
					 WordAIn[n]=0;
					}
	for (m =0; m<30; m++)
				{
					for (n=0; n<30; n++)
					{
						 inhib[m][n]=1;
						 if(m==n)
						 {
						   inhib[m][n]=0;
						 }
					}
				}
	 for (n=0; n<12; n++)
		{
			ActionHubXplore[n]==0;
	    }
	  for (n=0; n<30; n++)
		{
			 Col[n]=0;
			 Word[n]=0;
			 Shape[n]=0;
		}
	      ipCol[0]=0;
	      ipCol[1]=0;
	      ipCol[2]=0;
		  ipShap[0]=0;
		  ipShap[1]=0;
		  ipShap[2]=0;


	  for (n=0; n<42; n++)
					{
						ProvHub[n]=0;
						BodyHub[n]=0;
						OCHub[n]=0;
						BodyHubTD[n]=0;
						BodyHubBU[n]=0;
					}
  };

void ObserverThread::WordEncode(int numu, int hubenc)
{

	string mycol;
    int sizz,m,n;
    	for (n=0; n<120; n++)
					{
						 WordIn[numu][n]=0;
					}

    cout << "Input word " << endl;
    cin >> mycol;
    sizz=mycol.size();
    cout << sizz << endl;
    for (n=0; n<sizz; n++)
		{
		 cout << mycol[n] << endl;
		 if (mycol[n] ==  'r') { WordIn[numu][(8+(n*20))]=1;}
		 if (mycol[n] ==  'e') { WordIn[numu][(0+(n*20))]=1;}
		 if (mycol[n] ==  'd') { WordIn[numu][(9+(n*20))]=1;}
		 if (mycol[n] ==  't') { WordIn[numu][(1+(n*20))]=1;}
		 if (mycol[n] ==  'a') { WordIn[numu][(2+(n*20))]=1;}
		 if (mycol[n] ==  'o') { WordIn[numu][(3+(n*20))]=1;}
		 if (mycol[n] ==  'i') { WordIn[numu][(4+(n*20))]=1;}
		 if (mycol[n] ==  'n') { WordIn[numu][(5+(n*20))]=1;}
		 if (mycol[n] ==  's') { WordIn[numu][(6+(n*20))]=1;}
		 if (mycol[n] ==  'h') { WordIn[numu][(7+(n*20))]=1;}
		 if (mycol[n] ==  'l') { WordIn[numu][(10+(n*20))]=1;}
		 if (mycol[n] ==  'c') { WordIn[numu][(11+(n*20))]=1;}
		 if (mycol[n] ==  'u') { WordIn[numu][(12+(n*20))]=1;}
		 if (mycol[n] ==  'm') { WordIn[numu][(13+(n*20))]=1;}
		 if (mycol[n] ==  'w') { WordIn[numu][(14+(n*20))]=1;}
		 if (mycol[n] ==  'f') { WordIn[numu][(15+(n*20))]=1;}
		 if (mycol[n] ==  'g') { WordIn[numu][(16+(n*20))]=1;}
		 if (mycol[n] ==  'y') { WordIn[numu][(17+(n*20))]=1;}
		 if (mycol[n] ==  'p') { WordIn[numu][(18+(n*20))]=1;}
		 if (mycol[n] ==  'j') { WordIn[numu][(19+(n*20))]=1;}

	     }
    };


void ObserverThread::WordEncodeA(int numu, int hubenc)
{

	string mycol;

	
	int sizz,n;

   };


int ObserverThread::UserInterface(int GoalLearn)
{
	  int m,n,userIntervention;
	  if((GoalLearn==14)||(GoalLearn==41))
		  {
			if(GoalLearn==14)
			  {
				  cout << "Input Root Goal " << endl;
				//==============================================================
				  cout << "Input word " << endl;
				  string mycoll;
				  int sizze;
					cin >> mycoll;
					sizze=mycoll.size();
					cout << sizze << endl;
					for (n=0; n<120; n++)
					{
					WordAIn[n]=0;
					}

			for (n=0; n<sizze; n++)
				 {
	   			 cout << mycoll[n] << endl;
				 if (mycoll[n] ==  'r') { WordAIn[(8+(n*20))]=1;}
				 if (mycoll[n] ==  'e') { WordAIn[(0+(n*20))]=1;}
				 if (mycoll[n] ==  'd') { WordAIn[(9+(n*20))]=1;}
				 if (mycoll[n] ==  't') { WordAIn[(1+(n*20))]=1;}
				 if (mycoll[n] ==  'a') { WordAIn[(2+(n*20))]=1;}
				 if (mycoll[n] ==  'o') { WordAIn[(3+(n*20))]=1;}
				 if (mycoll[n] ==  'i') { WordAIn[(4+(n*20))]=1;}
				 if (mycoll[n] ==  'n') { WordAIn[(5+(n*20))]=1;}
				 if (mycoll[n] ==  's') { WordAIn[(6+(n*20))]=1;}
				 if (mycoll[n] ==  'h') { WordAIn[(7+(n*20))]=1;}
				 if (mycoll[n] ==  'l') { WordAIn[(10+(n*20))]=1;}
				 if (mycoll[n] ==  'c') { WordAIn[(11+(n*20))]=1;}
				 if (mycoll[n] ==  'u') { WordAIn[(12+(n*20))]=1;}
				 if (mycoll[n] ==  'm') { WordAIn[(13+(n*20))]=1;}
				 if (mycoll[n] ==  'w') { WordAIn[(14+(n*20))]=1;}
				 if (mycoll[n] ==  'f') { WordAIn[(15+(n*20))]=1;}
				 if (mycoll[n] ==  'g') { WordAIn[(16+(n*20))]=1;}
				 if (mycoll[n] ==  'y') { WordAIn[(17+(n*20))]=1;}
				 if (mycoll[n] ==  'p') { WordAIn[(18+(n*20))]=1;}
				 if (mycoll[n] ==  'j') { WordAIn[(19+(n*20))]=1;}
				 }

				int iCo,jCo,ahactiv=0;
				int maxA=0.01;
					for(iCo=0; iCo<12; iCo++)
											{
												int pr_Coo=0;
												 for(jCo=0; jCo<120; jCo++)
														{
															pr_Coo = pr_Coo + WActionPrim[iCo][jCo]*WordAIn[jCo];
														}

												ActH[iCo]=pr_Coo;
												if(ActH[iCo]>maxA)
													{
													   maxA=ActH[iCo];
													   ahactiv=iCo;
													}
											}
			   for(iCo=0; iCo<12; iCo++)
					   {
							 ActH[iCo]=ActH[iCo]/maxA;
						 WorABin <<ActH[iCo] << endl;
						// cout<<ActH[iCo]<<endl;
					   }
				cout<<"ahaactive"<<ahactiv<<endl;
				Goal_AcID=ahactiv;
				GoalStack[GoalPointer][0]=Goal_AcID;
	  }

  //========================================================

				   int sizz;
				   /*cout << "Input number goal arguement words " << endl;
				   cin >> sizz;*/
				    sizz=1;
                   NumWords=sizz;
				   fileName = pathPrefix;
				   fileName.append("WorBin.txt");
				   ofstream WorBin(fileName.c_str());
  					for (n=0; n<sizz; n++)
						{
						  WordEncode(n,2);
						}

					for (m =0; m<sizz; m++)
								{
									for (n=0; n<120; n++)
									{
										 WorBin << WordIn[m][n] << "    ";
									}
									WorBin << "    " << endl;
								}
					userIntervention=1;
			  }

	  if(GoalLearn==32) // is for the purpose oc communicating with the user in the online Xperience gaining mechanism
		  {
	         cout << "Error ID " << "cannot find the object requested: need help " <<  endl;
	         cout << "Reinitate and Retry or Learn experience directly " << endl;
		     cin >> userIntervention;
	  	  }

	  return userIntervention;
	};


void ObserverThread::onStop() {

//    outputPort.interrupt();
  //  outputPort.close();
}

void ObserverThread::initGR()
 {

    if(!(Network::isConnected("/commandRobot:o", "/psControl/interface:i"))){
			Network::connect("/commandRobot:o", "/psControl/interface:i");
		}
		Bottle& outBot = commandRobot.prepare();   // Get the object
		outBot.clear();
		outBot.addVocab(CMD_RIGHT_HAND); // put "p
    Bottle& listBot = outBot.addList();
    listBot.addDouble(33);
    listBot.addDouble(90);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    printf("Sending bottle right hand (%s)\n",outBot.toString().c_str());
    commandRobot.writeStrict();
    //Time::delay(2);
};

void ObserverThread::initGL()
 {
    if(!(Network::isConnected("/commandRobot:o", "/psControl/interface:i"))){
			Network::connect("/commandRobot:o", "/psControl/interface:i");
		}
    Bottle& outBot = commandRobot.prepare();   // Get the object
    outBot.clear();
    outBot.addVocab(CMD_LEFT_HAND); // put "pos" command in the bottle
    Bottle& listBot = outBot.addList();
    listBot.addDouble(33);
    listBot.addDouble(90);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    printf("Sending bottle right hand (%s)\n",outBot.toString().c_str());
    commandRobot.writeStrict();
    //Time::delay(2);
};


void ObserverThread::initGPR()
 {

    if(!(Network::isConnected("/commandRobot:o", "/psControl/interface:i"))){
			Network::connect("/commandRobot:o", "/psControl/interface:i");
		}
		Bottle& outBot = commandRobot.prepare();   // Get the object
		outBot.clear();
		outBot.addVocab(CMD_RIGHT_HAND); // put "p
    Bottle& listBot = outBot.addList();
    listBot.addDouble(33);
    listBot.addDouble(10);
    listBot.addDouble(80);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    printf("Sending bottle right hand (%s)\n",outBot.toString().c_str());
    commandRobot.writeStrict();
    //Time::delay(2);
};

void ObserverThread::initGPL()
 {
	if(!(Network::isConnected("/commandRobot:o", "/psControl/interface:i"))){
			Network::connect("/commandRobot:o", "/psControl/interface:i");
		}
    Bottle& outBot = commandRobot.prepare();   // Get the object
    outBot.clear();
    outBot.addVocab(CMD_LEFT_HAND); // put "pos" command in the bottle
    Bottle& listBot = outBot.addList();
    listBot.addDouble(33);
    listBot.addDouble(10);
    listBot.addDouble(80);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    listBot.addDouble(0);
    printf("Sending bottle right hand (%s)\n",outBot.toString().c_str());
    commandRobot.writeStrict();
    //Time::delay(2);
};

