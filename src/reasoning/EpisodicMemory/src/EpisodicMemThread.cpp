
#include <EpisodicMemThread.h>
#include <cstring>
#include <string>
#include<time.h>
#include <math.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

EpisodicMemThread::EpisodicMemThread() {
    robot = "icub";        
}

EpisodicMemThread::EpisodicMemThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

EpisodicMemThread::~EpisodicMemThread() {
    // do nothing
}

bool EpisodicMemThread::threadInit() {
   
   
	if (!WorldSnap.open(getName("/world/analysis:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
	
	if (!PlanF.open(getName("/Strategy:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   

	if (!ObserverResponse.open(getName("/strategy:io").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   

	if (!RememberedMemories.open(getName("/MyRemembered:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   

	if (!HubObject.open(getName("/HubObject:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
  if (!HubBody.open(getName("/HubBody:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!UsefulPastXperiences.open(getName("/Useful/PastXperiences:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
	if (!HumTopDownCompete.open(getName("/Hub:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
	if (!PlanorXplore.open(getName("/PlanXplore:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   
	
	N=1000;
	PrevSentPlan=50;
	Network::connect("/what-to-do:o", "/world/analysis:i"); 
	InitializeAM();
	//Network::connect("/Strategy:o", "/strategy:i");  //check this
	//Network::connect("/AckObserver:o","/EpimAck:i"); 
   
    return true;
}

void EpisodicMemThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string EpisodicMemThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void EpisodicMemThread::setInputPortName(string InpPort) {
    
}

void EpisodicMemThread::setPath(string inS) {
    pathPrefix	= inS.c_str();
}

void EpisodicMemThread::run()
{    
    while (isStopping() != true)
	 {
	   //response section
		if(ObserverResponse.getInputCount())
		{
			Bottle request, response;
		
			ObserverResponse.read(request,true);
			
			if(!request.isNull()) {
				printf("%s \n",request.toString().c_str());
				//cout<<"request!=NULL"<<endl;
				// request present
				//reading
				int cmd = request.get(0).asVocab();
				//cout<<cmd<<endl;
				Goal_Context=request.get(1).asInt();
                HubQuery=request.get(2).asInt();
				//cout<<"Goal Context is " << Goal_Context << "Hub context is" << HubQuery <<endl;
				GoalID=request.get(3).asInt();
				GoalIDRef=GoalID;
				//cout<<"ActionHUB ID is"<<GoalID<<endl;
                NoB=request.get(4).asDouble();
				//cout<<"There are " << NoB << "neural activations in Hub" << HubQuery <<endl;
				Largeness=request.get(5).asInt();

                //cout << Largeness << endl;
				ReplanStatus=request.get(6).asInt();
				//cout<< "Replan Status:::::" << ReplanStatus <<endl;
			   //=================================================================================

				 for (int m =0; m<42; m++)
					   {
						OHub[m]=0;
						VSSP[m]=0;
						BHub[m]=0;
					   }
				SumVSSP=0;
				iSPlan=0;
    			//initialization can from from bottom up perception
				int xid=0;
					for(int i=0;i<NoB;i++)
						{
						  xid = request.get(i+7).asInt();
						  if((HubQuery==1))
						  {
							 OHub[i]=xid;
							 cout << "Object HUB ID received" << xid << endl;
							 if(Goal_Context!=1400){
							 Goal_Context=50;
							 }
						  }
						   if(HubQuery==2)
						  {
							 BHub[i]=xid;
							 cout << "BODY HUB ID received" << xid << endl;
							 HubQuery=2;
							 Goal_Context=50;
						  }
						  
						}
                     if(Largeness == 1)
						  {
							 OHub[37]=1;  //size related neural activation
							 //cout<<"Largeness"<<Largeness<<endl;
						  }  
					 if(Largeness == 0)
						  {
							 OHub[36]=0;  //size related neural activation
						  }  
					  if(ReplanStatus == 0)
						  {
							 PrevSentPlan=50;  //size related neural activation
						  }  
					
      				for (int m =0; m<42; m++)
						{
							VSSP[m]=OHub[m];
							if(m<36)  // note that u only take shape into account assuming 2 shapes of different sizes do not come into picture
								{
							   SumVSSP=SumVSSP+OHub[m];
							}
					   }
					//cout << "Visuo Spatial sketch pad cumulative activity" << "\t" << SumVSSP << endl ;
			        //cout << "Remembering and Planning" << endl ;

					//========================================================
        /*if( HubQuery ==1) {
					  Bottle& HBU = HubObject.prepare();
					  HBU.clear();
					  HBU.addString("hubObject");
					  Bottle& HBUVals = HBU.addList();
					  HBUVals.clear();
					  for(int i=0; i<42; i++)
						  {
						  HBUVals.addInt(VSSP[i]);
						  }
					
					  cout<<"Sending Plan to DARWIN GUI related to object hub"<<endl;
            HubObject.write();
          }
          if( HubQuery ==2) {
              Bottle& HBU = HubBody.prepare();
					    HBU.clear();
					    HBU.addString("hubBody");
					    Bottle& HBUVals = HBU.addList();
					    HBUVals.clear();
					    for(int i=0; i<42; i++)
						    {
						    HBUVals.addInt(BHub[i]);
						    }
					
					    cout<<"Sending Plan to DARWIN GUI related to body hub"<<endl;
             	HubBody.write();
          }*/
					//========================================================

					//make bottle to command VSSP bottom up activations to GUI here... 

				//	Time::delay(30); Just To check...Xlator...VM 04 March 2013
					cout<<"Recalling past experiences in relation to Goal Context " << Goal_Context << " and/or neural activations in Hub" << HubQuery <<endl;
					MemControl(HubQuery,Goal_Context); 
            //=================================================================================
				//processing here
				//processPlan();
               if(iSPlan!=0)
				 {
					response.addInt(123);
				 }
			   if(iSPlan==0)
				 {
					response.addInt(41);
					ChunkTerminate=iSPlan;
				 }
			   //cout << "Sending out plan to Client" << endl;
					for(int i=0;i<1000;i++)
						{
						  response.addInt(PlanPastExp[i]);
						}
					//cout<<"Point of Chunk Termination is"<<ChunkTerminate<<endl;
					response.addInt(ChunkTerminate);
					ObserverResponse.reply(response);
					
					fileName = pathPrefix;
					fileName.append("PXper.txt");
					ofstream PXper(fileName.c_str());	
					for(int i=0; i<1000; i++)
					{
					 PXper<< PlanPastExp[i] <<endl;
                     //planvalues.addInt(PlanPastExp[i]);
					}
					PXper.close();
               // ofstream PXper("PXper.txt");

				/*Bottle& Plan = PlanorXplore.prepare();
				Plan.clear();
				Plan.addString("plan1");
				Bottle& planvalues = Plan.addList();
				planvalues.clear();
				for(int i=0; i<1000; i++)
					{
					 PXper<< PlanPastExp[i] <<endl;
                     planvalues.addInt(PlanPastExp[i]);
					}
				
			    //cout<<"Sending Plan to DARWIN GUI"<<endl;
             	PlanorXplore.write();*/
				//printf("%s \n",PlanResponse.toString().c_str());

				//int Planresponsecode = PlanResponse.get(0).asInt();
					

				yarp::os::Semaphore mutex;
				mutex.wait();	
				idle = true;
				mutex.post();
				
			}
			else {
				//cout<<"null requst"<<endl;
			}
		}
	
		//Time::delay(1);

	}
	
}

void EpisodicMemThread::threadRelease() {
    // nothing
	ObserverResponse.close();
	PlanF.close();
    WorldSnap.close();
	AckObs.close();
}

void EpisodicMemThread::onStop() {
    
//    outputPort.interrupt();
  //  outputPort.close();
}

void EpisodicMemThread:: MemControl(int HubQuery, int GoalContext)
	{
		 int iCo;
		 //ofstream PXper("PXper.txt");
		 fileName = pathPrefix;
		 fileName.append("PXper.txt");
		 ofstream PXper(fileName.c_str());
		 int Nrelevent=RememberPast(HubQuery,GoalContext);  //remember my past episodic experiences based on the present context/hubs active/user goal
		 if((GoalContext==4)) //activations in object or action hubs relayed by Observer or User
			 {
			 int nwinner=TDMemCompHub(Nrelevent); // find which memories out of all those rememebred are msot valuable	 
			 int noverl=FindOverlap(nwinner); // find how various rememebred expereinces overlap in the knowledge they encode
			 RipRealSequence(nwinner); // Take out useful object action sequnces that can be enacted in the present from these memories
			 PlanFromPastXP(nwinner,noverl);   // synthesize a novel behaviour or combine past experience with exploration
			 }
		  if((HubQuery==1)||(HubQuery==2)||(GoalContext=14)) //activations in object or action hubs relayed by Observer or User
			 {
             if(Nrelevent!=0)
			 {
				 int MemEnergy=TDMemCompBodyHub(Nrelevent); // here u will get a remembered plan/combination of plans with minimal energy in the context of the goal
				 // extracting relevant sequnce chunk...not all of the plan is needed, what you need is only the relavant memory chunk that connects with observer req
				 iSPlan=1;
			 }
			 	
    //Make bottle to command to GUI about plotting remembered experiences /VM
				/*Bottle& Plan = PlanorXplore.prepare();
				Plan.clear();
				Plan.addString("plan1");
				Bottle& planvalues = Plan.addList();
				planvalues.clear();
				for(int i=0; i<1000; i++)
					{
					 //PXper<< PlanPastExp[i] <<endl;
                     planvalues.addInt(PlanPastExp[i]);
					}
				
			    //cout<<"Sending Plan to DARWIN GUI"<<endl;
             	PlanorXplore.write();
				*/

			 }

		/* for(iCo=0; iCo<1000; iCo++)  //this is for plotting..do this later
			{
			 PXper<< PlanPastExp[iCo] <<endl;
			}*/  
};

void EpisodicMemThread:: PlanFromPastXP(int NWine, int Noverlapp)
	{
      int iCo,jCo,Olala=0,Pctrr,composi=0,Pctrr2;
	    	if(Noverlapp==0)
					{
						if(NWine==1)
							{
							  Pctrr=0;
						      //cout << "Past Exp sequnce is directly available without  recombination/chunking" << endl;
							   for(iCo=0; iCo<IndexM[0]; iCo++)
										 {
										 for(jCo=0; jCo<50; jCo++)
											 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[0][Pctrr];
							    				  Pctrr=Pctrr+1;
											 }
										 }
							   PEnd=Pctrr;
								   if((SumVSSP-SumTopDown)<0.8)
									  {
										  //cout << "Full knowledge exists in past experiences: Plan Synthesized! " << endl;
									  } 
								   if((SumVSSP-SumTopDown)>0.75)
									  {
										  //cout << "Combining past experience with exploration: On novel objects " << endl;
										  Olala=NovelObID[0];
										  XploreCombact(Olala); 
									  } 
						      }

						if(NWine==2)
							{
						      //cout << "need to combine nonoverlapping chunks of knowledge " << endl;
								// here u have to copy paste first action seq in usefulacseq as the plan
                            //=================================================================
				for(composi=Largeness; composi<2; composi++)
						{
							Pctrr=0;
							Pctrr2=0;
							if(composi==0)
							{
								for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
									{
										  if(iCo<IndexM[0])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[0][Pctrr];
							    				  Pctrr=Pctrr+1;
												 }
										  }
										  if(iCo>=IndexM[0])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[1][Pctrr2];
							    				  Pctrr=Pctrr+1;
												  Pctrr2=Pctrr2+1;
												 }
										  }
									  }
								PEnd=Pctrr;
                                if((SumVSSP-SumTopDown)<0.5)
									  {
										  //cout << "Full knowledge exists in past experiences: Plan Synthesized! " << endl;
										  //cout << "Trigger explorative action sequence 1 ? " << endl;
									  }

								   if((SumVSSP-SumTopDown)>0.5)
									  {
										  cout << "Combining past experience with exploration: On novel objects " << endl;
										  Olala=NovelObID[0];
										  XploreCombact(Olala); 
								      } 
                            					
                            }

							//---------------------------------------------------------------------------
							if(composi==1)
							{
								for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
									{
										  if(iCo<IndexM[1])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[1][Pctrr];
							    				  Pctrr=Pctrr+1;
												 }
										  }
										  if(iCo>=IndexM[1])
										   {
											 for(jCo=0; jCo<50; jCo++)
												 {
							  					  PlanPastExp[Pctrr]=NowObAcSeq[0][Pctrr2];
							    				  Pctrr=Pctrr+1;
												  Pctrr2=Pctrr2+1;
												 }
										  }
									  }
                             PEnd=Pctrr;
                                if((SumVSSP-SumTopDown)<0.5)
									  {
										  //cout << "Full knowledge exists in past experiences: Plan Synthesized! " << endl;
										//  cout << "Trigger explorative action sequence 2 ? " << endl;
									  } 
								   if((SumVSSP-SumTopDown)>0.5)
									  {
										  cout << "Combining past experience with exploration: On novel objects " << endl;
										  Olala=NovelObID[0];
										  XploreCombact(Olala); 
								      }     
			
							}//composi=1 loop
						}

//=======================================================================
						    }
					}

				if(Noverlapp > 0)
					{
					  MemComb(NWine);	
					   if((SumVSSP-SumTopDown)<0.5)
						  {
							  //cout << "Full knowledge exists the novel action sequence synthesized by recombinign memories! " << endl;
						  } 
					  if((SumVSSP-SumTopDown)>0.5)
						  {
							  //cout << "Combining past experience with exploration: On novel objects " << endl;
							  Olala=NovelObID[0];
							  XploreCombact(Olala); 
						  }     

					} 	   
		
	};

	void EpisodicMemThread::XploreCombact(int Ola)
		{
		  int NovObID, iCo,jCo;
		  NovObID=Ola;
		  //cout<<"entered Xplore mode: New object"<< Ola <<endl;
		  for(jCo=0; jCo<42; jCo++)
				{
					if((jCo==NovObID)||(jCo==36)){
						PlanPastExp[PEnd]=1;
						PEnd=PEnd+1;
					}
					else {
					  PlanPastExp[PEnd]=0;
					  PEnd=PEnd+1;
					}
				}
            PlanPastExp[PEnd]=1; //42
			PlanPastExp[PEnd+13]=1; 
			PlanPastExp[PEnd+51]=1; 
			
		  // make sure u now say to Observer this is not a plan but exploration with a new object detected and wait for reward
		};


void EpisodicMemThread::MemComb(int NWiner)
	{
		int iCo,jCo,dis[2],disDiff[2],ConcMem[3][1000],conccnt,coun2,compo;
		//===========================================================================================================
      cout << "Making a new plan combining multiple past Xpereinces" << endl;
                if(NWiner==2) // if there are many u need to remove this loop and iterate more
					{
						//put M1 before M2
						for (compo=0;compo<2; compo++)
						{
						 conccnt=0;
                         coun2=0;
                         for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
						 {
							
							 for(jCo=0; jCo<50; jCo++)
							 {
								 if(compo==0)
								 {
									   if(iCo<IndexM[0])
									   {
										   ConcMem[compo][conccnt]=NowObAcSeq[0][conccnt];
										   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
										   {
											dis[0]=iCo;
											//cout << "Olap Element Mem1" << dis[0]<< endl;
										   }
									   }
										if(iCo >=IndexM[0])
										   {
											   ConcMem[compo][conccnt]=NowObAcSeq[1][coun2];
											   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
											   {
												dis[1]=iCo;
												//cout << "Olap Element Mem2" << dis[1]<< endl;
											   }
											   coun2=coun2+1;
											}
								 } // if compo =0 

								 if(compo==1)
								 {
									   if(iCo<IndexM[1])
									   {
										   ConcMem[compo][conccnt]=NowObAcSeq[1][conccnt];
										   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
										   {
											dis[0]=iCo;
											//cout << "Olap Element Mem1" << dis[0]<< endl;
										   }
									   }
										if(iCo >=IndexM[1])
										   {
											   ConcMem[compo][conccnt]=NowObAcSeq[0][coun2];
											   if((jCo==OLap[0])&&(ConcMem[compo][conccnt]==1))
											   {
												dis[1]=iCo;
												//cout << "Olap Element Mem2" << dis[1]<< endl;
											   }
											   coun2=coun2+1;
											}
								 } // if compo =1

									conccnt=conccnt+1;
							 }// end of jCo loop

						 }// end of iCo loop

						 disDiff[compo]=dis[1]-dis[0];
						 //cout << "Compo diss diff" << compo << "   " << disDiff[compo]<< endl;

						}// end of compo loop 
							
				    }// end of if Winer loop
    
//============ Removing redundant overlapps and synthesizing novel action sequnce ====================
  int Pctr;
	if(disDiff[0]<disDiff[1])
	  {
          conccnt=0;
		  Pctr=0;
		  for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
				 {
					 if(iCo==dis[1])
						 {
						  iCo=iCo+2;
                          conccnt=conccnt+100; 
						 } 
					 if((iCo !=dis[1])||(iCo !=dis[1]+1))
						 {
						 for(jCo=0; jCo<50; jCo++)
							 {
							   PlanPastExp[Pctr]=ConcMem[0][conccnt];
							   conccnt=conccnt+1;
							   Pctr=Pctr+1;
							 }
					     }
		         }//iCo loop ends here
		  PEnd=Pctr;
	    }
 
  if(disDiff[0]>disDiff[1])
	  {
	  conccnt=0;
		  Pctr=0;
		  for(iCo=0; iCo<IndexM[0]+IndexM[1]; iCo++)
				 {
					 if(iCo==dis[0])
						 {
						  iCo=iCo+2;
                          conccnt=conccnt+100; 
						 } 
					 if((iCo !=dis[0])||(iCo !=dis[0]+1))
						 {
						 for(jCo=0; jCo<50; jCo++)
							 {
							   PlanPastExp[Pctr]=ConcMem[1][conccnt];
							   conccnt=conccnt+1;
							   Pctr=Pctr+1;
							  }
					     }
		         }//iCo loop ends here
	    PEnd=Pctr;
	  }
  if(disDiff[0]==disDiff[1])
	  {
	   //Need to explore further both action sequences in ConcMem....
	  
	  }

	};


void EpisodicMemThread:: RipRealSequence(int nme)
	{
	 int iCo,jCo,jCoo,MCnt,UnfM[20][50], UseSeq[20][50];
					fileName = pathPrefix;
					fileName.append("UsefulAcSeqs.txt");
					ofstream UsePCue(fileName.c_str());
	 //ofstream UsePCue("UsefulAcSeqs.txt");
     for(MCnt=0; MCnt<nme; MCnt++)
	 {
//====================================================================================
		 int counUnf=0;
		 for(iCo=0; iCo<20; iCo++)
				{
				  for(jCo=0; jCo<50; jCo++)
						{
	      					UseSeq[iCo][jCo]=0;
							UnfM[iCo][jCo]=NRelPast[MemID[MCnt]][counUnf];
							if(UnfM[iCo][jCo]>0.75)
									{
									  UnfM[iCo][jCo]=1;
									}
							counUnf=counUnf+1;
							if((jCo==44)&&(UnfM[iCo][jCo]==1))
								{
								 IndexM[MCnt]=iCo;
								}
						}
				}  
                // Here u have the index of the useful part of the useful memories
                int ICT=IndexM[MCnt];
				int CntUse=0;
                for(iCo=0; iCo<ICT; iCo=iCo+2)
				  {
                    int SuVssp=0; 
                    for(jCo=0; jCo<36; jCo++)
						{
                         SuVssp=SuVssp+UnfM[iCo][jCo]*VSSP[jCo];
					    }
					if(SuVssp != 0)
						{
						 for(jCoo=0; jCoo<50; jCoo=jCoo+1)
							  {
                               UseSeq[CntUse][jCoo]=UnfM[iCo][jCoo];
							   UseSeq[CntUse+1][jCoo]=UnfM[iCo+1][jCoo];
                               }
						 CntUse=CntUse+2;
						} //ripped the object action sequnce
					if(SuVssp == 0)
						{
                         IndexM[MCnt]=IndexM[MCnt]-2; //because one object remembered of the past is not there in the now
						 // hence cannot be acted upon
					    }
				  }
  /////// Reunfold the object-action sequnce relevant in the now emerging from this past experience
     int countRel=0;
				for(iCo=0; iCo<20; iCo++)
					{
					  for(jCo=0; jCo<50; jCo++)
							{
								NowObAcSeq[MCnt][countRel]=UseSeq[iCo][jCo];
								UsePCue << NowObAcSeq[MCnt][countRel]<< "    ";
								countRel=countRel+1;
							}
				}
            UsePCue << "    " << endl;
  // ========================================================================================================        
  //for MCnt ends below
	 }
	 //for(iCo=0; iCo<nme; iCo++)
	  //{

		//cout<<"Sending useful experiences DARWIN GUI: "<<iCo<<endl;
		/*Bottle& Useful = UsefulPastXperiences.prepare();	
		Useful.clear();

		 string str;

		  if(iCo == 0)
		  {
			  str = "cue0";
		  }
		  else if(iCo == 1)
		  {
			  str = "cue1";
		  }
		  else if(iCo == 2)
		  {
			  str = "cue2";
		  }
		  else if(iCo == 3)
		  {
			  str = "cue3";
		  }
		  else if(iCo == 4)
		  {
			  str = "cue4";
		  }		 
		 
		//string name("rem");
		//sprintf((char*)name.c_str(),"%d",iCo);
		printf("%s \n",str.c_str()); // rem0, rem1, rem2, rem3
		Useful.addString(str.c_str());

		Bottle& usevalues = Useful.addList();
		usevalues.clear();

		for(jCo=0; jCo<1000; jCo++)
		{					
			usevalues.addInt(NowObAcSeq[iCo][jCo]);
		}
	 
	UsefulPastXperiences.write();*/  
	//RememberedMemories.write();
	//Time::delay(2);

	//}
   	};

int EpisodicMemThread::FindOverlap(int NW)
	{
      int iCo,  Overl=0;
	  double memul, vssptddiff=0,vsspThresh=0.5;
	 // if(Largeness==1)
	  //{
	    //vsspThresh=1.1;
	  //}
	  NNovobs=0;
	  if(NW==1)
		  {
		  //cout << "There is a single winner" << endl;
           for(iCo=0; iCo<36; iCo++)
				{
					vssptddiff=VSSP[iCo]-HubTopDown[MemID[0]][iCo];
                 
					if(vssptddiff>vsspThresh)
						{
							//cout << "There is a novel object in VSSP that is not there in TD Hub activity" << endl;
							NovelObID[NNovobs]=iCo;
							NNovobs=NNovobs+1; //size issue is here : to solve it make VSSP neural activity 
							//a unique ID that codes for both size and shape, only large objects need to be reassigned a new neuron
						}
				}
		   //cout << "Novel obejcts" << NNovobs <<endl;
		  }

      if(NW==2)
		  {
		    for(iCo=0; iCo<36; iCo++)
				{
                  memul=0;
			      memul=HubTopDown[MemID[0]][iCo]*HubTopDown[MemID[1]][iCo];
			      if(memul > 0.1) 
					  {
					   Overl=Overl+1;
					   OLap[Overl-1]=iCo;
					  }
                   vssptddiff=VSSP[iCo]-(HubTopDown[MemID[0]][iCo]+HubTopDown[MemID[1]][iCo]);
					if(vssptddiff>0.2)
						{
							//cout << "There is a novel object in VSSP that is not there in TD Hub activity" << endl;
							NovelObID[NNovobs]=iCo;
							NNovobs=NNovobs+1; //size issue is here : to solve it make VSSP neural activity 
							//a unique ID that codes for both size and shape, only large objects need to be reassigned a new neuron
						} 
			    }
			//cout << "No of overlaps" << "   " <<  Overl <<endl;
			//cout << "Novel obejcts" << NNovobs <<endl;
		   }
return Overl;
 };

int EpisodicMemThread:: TDMemCompBodyHub(int Nrelevant) // this deals with top down modultion in the body and goal hubs, formation of subsidiary plans under failure
{
        int InstMk=0,iCo,jCo,Nmemos=0,Hubu[42],nom,comprel=0,UnfR[20][50],rewno,MaxEnergy;
		int SunHIB[6];
		SumTopDownB=0;
		ChunkTerminate=0;
					fileName = pathPrefix;
					fileName.append("MinEnergyPlan.txt");
					ofstream HeeHuBod(fileName.c_str());
		//ofstream HeeHuBod("MinEnergyPlan.txt");

		for(jCo=0; jCo<6; jCo++)
						{
							SunHIB[jCo]=0;
							MemIDBhub[jCo]=0;
						}
		 
		MaxEnergy=36;
		//===============Anticipating the energy of the memory=========================
		for(comprel=0; comprel<Nrelevant; comprel++)
				{ 
					int counUnf=0;
					for(iCo=0; iCo<20; iCo++)
							{
							  for(jCo=0; jCo<50; jCo++)
									{
										UnfR[iCo][jCo]=NRelPast[comprel][counUnf];
										counUnf=counUnf+1;
									}
							}
                     for(iCo=0; iCo<20; iCo++)
							{
								if(UnfR[iCo][44]>0.8)
									{
									  InstMk=iCo;
									}
					        }
					 SunHIB[comprel]=InstMk;
	                 for(jCo=0; jCo<36; jCo++)
									{
										if(UnfR[InstMk][jCo]>0.8)
											{
											    rewno=jCo;
												//nom=(jCo+1)/(0.5*InstMk);
											}
									}  
        
                   //cout << "Anticipated Energy of memory"  << "\t" << comprel+1 << "is"  << "\t" << rewno  << endl;
					   if(rewno<MaxEnergy)
					   {
                          MaxEnergy=rewno;
						  MemIDBhub[0]=comprel;
					   }

		}
//Recreating the memory with minimum energy (NOTE: this can be >1 if hub is controled by a team of memories...)
		int counUnf=0;
		int memindexy= MemIDBhub[0];
		
		//cout << "MemindexY"  << "\t" <<  memindexy  << endl;
					for(iCo=0; iCo<1000; iCo++)
							{
								 HeeHuBod<< NRelPast[memindexy][iCo]<< "    ";
								 PlanPastExp[iCo]=NRelPast[memindexy][iCo];
							 }
						 HeeHuBod << "    " << endl; 

						ChunkTerminate=SunHIB[memindexy];
						if((Goal_Context==1400)&&(ChunkTerminate==0))
						{
						ChunkTerminate=6;
						}
  int MaxPlan=0;
  int tempMaxPlan=0;
   for(iCo=0; iCo<9; iCo++)
	   {
		  int ScorePlanMem=0;
		  for(jCo=0; jCo<1000; jCo++)
						{
						   ScorePlanMem=ScorePlanMem +(PlanPastExp[jCo]-Episodes[iCo][jCo]);
						}
		  if(ScorePlanMem==0)
			  {
              // MaxPlan=ScorePlanMem;
		       PrevSentPlan=iCo;
		  	  }
		   }
   //cout << "Sending plan:::::"  << "\t" <<  PrevSentPlan  << endl;
   return MaxEnergy;
   };

int EpisodicMemThread:: TDMemCompHub(int Nrelev)
	{
		int InstMk=0,iCo,jCo,Nmemos=0,Hubu[42],nom,comprel=0,UnfR[20][50],rewno;
		double SunHI[6];
	    SunhiDiff=0.4;
					 
		SumTopDown=0;
		fileName = pathPrefix;
		fileName.append("HubTDownCont.txt");
		ofstream HeeHu(fileName.c_str());
		//ofstream HeeHu("HubTDownCont.txt");

		fileName = pathPrefix;
		fileName.append("HubTDCom.txt");
		ofstream HeeHoo(fileName.c_str());
		//ofstream HeeHoo("HubTDCom.txt");
       
				   for(jCo=0; jCo<6; jCo++)
						{
							SunHI[jCo]=0;
							MemID[jCo]=0;
						}
                
		for(iCo=0; iCo<5; iCo++)
				{
				   for(jCo=0; jCo<42; jCo++)
						{
							HubTopDown[iCo][jCo]=0;
						}
                }  
	
        for(jCo=0; jCo<42; jCo++)
						{
						Hubu[jCo]=OHub[jCo];
    					}
         Hubu[36]=0; Hubu[37]=0; 

//=================== Init done, Now find the anticipated reward fetched by each remembered memory and objects they know about=============
		for(comprel=0; comprel<Nrelev; comprel++)
				{ 
					int counUnf=0;
					for(iCo=0; iCo<20; iCo++)
							{
							  for(jCo=0; jCo<50; jCo++)
									{
										UnfR[iCo][jCo]=NRelPast[comprel][counUnf];
										counUnf=counUnf+1;
									}
							}
                     for(iCo=0; iCo<20; iCo++)
							{
								if(UnfR[iCo][44]>0.8)
									{
									  InstMk=iCo;
									}
					        }
	                 for(jCo=0; jCo<36; jCo++)
									{
										if(UnfR[InstMk][jCo]>0.8)
											{
											    rewno=jCo;
												nom=(jCo+1)/(0.5*InstMk);
											}
									}  
                   
                   //cout << "Anticipated reward for memory"  << "\t" << comprel+1 << "is" << "\t" << nom << "\t"  << InstMk << "\t" << rewno  << endl;
				//   HubActNote[comprel][6]=nom;
				  				   
	//====================== here u extract the top down influence of this experience =============================
          	for(iCo=0; iCo<42; iCo++)
				{
				    int pr_CooC=0;
					 for(jCo=0; jCo<1000; jCo++)
						{
							pr_CooC = pr_CooC + WHub2EpimT[iCo][jCo]*NRelPast[comprel][jCo];
						}
                   HubTopDown[comprel][iCo]= pr_CooC*Hubu[iCo]*nom;
				}
            
			SunHI[comprel]=0; 
			for(iCo=0; iCo<42; iCo++)
				 {
				 HeeHu<< HubTopDown[comprel][iCo]<< "    ";
                 SunHI[comprel]=SunHI[comprel]+HubTopDown[comprel][iCo];  //Sum HubTD
				 }
			 HeeHu << "    " << endl;

				// loop ends here
			}
       //    if(Largeness==1)
		//			 {
		//			   SunhiDiff=0.95;
		//			   //cout<<"Setting sunhiDiff"<<SunhiDiff<<endl;
		//			 }
if(Nrelev>1)
		{
			 int iterComp, iterHub,iterHubM;
			 for(iterComp=0; iterComp<5; iterComp++)
					 {
						 for(iCo=0; iCo<Nrelev; iCo++)
							 {
								 double MaxHu=0.0001;
								 for(jCo=0; jCo<Nrelev; jCo++)
									 {
										if(iCo != jCo)
											{
											  for(iterHub=0; iterHub<42; iterHub++)
												 {
													HubTopDown[iCo][iterHub]=HubTopDown[iCo][iterHub]-(0.1*(HubTopDown[iCo][iterHub]*HubTopDown[jCo][iterHub]*SunHI[jCo]));
													if(HubTopDown[iCo][iterHub] < 0.25)
													{
                                                    HubTopDown[iCo][iterHub]=0;
													}
													if(HubTopDown[iCo][iterHub]>MaxHu)
														{
														MaxHu=HubTopDown[iCo][iterHub];
														}
											    }
											  
											//here u get all 42 numbers, u need to find max, normalize and go to next iteration
										}
										for(iterHubM=0; iterHubM<42; iterHubM++)
												{
													HubTopDown[iCo][iterHubM]=HubTopDown[iCo][iterHubM]/MaxHu;										
												}
                                        //jCo loop of reciveing inhibition from all memories competing is over
                                        
									  }
							 }
					  }
		//compet if loop ends below 
		}
for(iCo=0; iCo<Nrelev; iCo++)
				 {
                    SunHI[Nrelev]=0;
					for(jCo=0; jCo<42; jCo++)
					 {
					   HeeHoo<< HubTopDown[iCo][jCo] << "    ";
					   SunHI[Nrelev]=SunHI[Nrelev]+HubTopDown[iCo][jCo];
					 }
					 HeeHoo << "    " << endl;
					
					 if(SunHI[Nrelev]<SunhiDiff) // was 0.9
					 {
					  SunHI[Nrelev]=0;
					 }
                     if(SunHI[Nrelev]>SunhiDiff)
					 {
					  Nmemos=Nmemos+1;
					  MemID[Nmemos-1]=iCo;
					  //cout << "Winning MemID"  << "  " << MemID[Nmemos-1] << " power " << SunHI[Nrelev] << " No of winners so far " << Nmemos  <<endl ;
                      //cout<<SunhiDiff<<endl; 					
					 }
					 SumTopDown=SumTopDown+SunHI[Nrelev];
    }
//here we get the final result of competing memories, know how many survived Nmemos, who all are they...
//cout << "Top Down Sum"  << "  " << SumTopDown << endl ;

//for(iCo=0; iCo<Nrelev; iCo++)
	  //{

		//cout<<"Sending Hub Top Down Competition components to DARWIN GUI: "<<iCo<<endl;
		/*Bottle& HubTDOWN = HumTopDownCompete.prepare();	
		HubTDOWN.clear();

		 string str;

		  if(iCo == 0)
		  {
			  str = "hub0";
		  }
		  else if(iCo == 1)
		  {
			  str = "hub1";
		  }
		  else if(iCo == 2)
		  {
			  str = "hub2";
		  }
		  else if(iCo == 3)
		  {
			  str = "hub3";
		  }
		  else if(iCo == 4)
		  {
			  str = "hub4";
		  }		 
		 
		//string name("rem");
		//sprintf((char*)name.c_str(),"%d",iCo);
		printf("%s \n",str.c_str()); // rem0, rem1, rem2, rem3
		HubTDOWN.addString(str.c_str());

		Bottle& hubvalues = HubTDOWN.addList();
		hubvalues.clear();

		for(jCo=0; jCo<42; jCo++)
		{					
			hubvalues.addDouble(HubTopDown[iCo][jCo]);
		}
	 
	HumTopDownCompete.write();*/  
	//RememberedMemories.write();
	//Time::delay(0.2);

	//}

return Nmemos;
	};



int EpisodicMemThread::RememberPast(int HubID, int GoalRoot) // loops from the present to partial cue to remembered past
{
	int icn,HE[1000],AE[1000],sumhe,tempHE,iCo,jCo,VunF[20][50],TagPQ,ActivIndex;  //OHub WHub2Epim Episodes
	kk=0;
		fileName = pathPrefix;
		fileName.append("PCue.txt");
		ofstream PCue(fileName.c_str());
	//ofstream PCue("PCue.txt");

		fileName = pathPrefix;
		fileName.append("HeEpMult.txt");
		ofstream Hee(fileName.c_str());
	//ofstream Hee("HeEpMult.txt");	 
		int sttr=0; 
		int endtr=0;
		if(Goal_Context==1400)
		{
		  sttr=7;
		}
	for(icn=sttr; icn<NumEpi; icn++) //NumEpi //VM Made this change to test online learning
			{
				if((icn!=PrevSentPlan)&&(icn!=6)) //VM made change here to take into account word:Assemble
				{
				for(iCo=0; iCo<20; iCo++)
							{
							  for(jCo=0; jCo<50; jCo++)
									{
										VunF[iCo][jCo]=0;
									}
								}
				TagPQ=0;
				sumhe=0;
				tempHE=0;
				if((HubID==1)||(HubID==2))
				{
	   			//==================== Instantaneous Hub (1x42)* WHubEpim (42x1000)========================================
           					for(iCo=0; iCo<1000; iCo++)
								{
									int pr_Coo=0;
									if(HubID==1) //Object Hub
										{
										 for(jCo=0; jCo<42; jCo++)
											{
												pr_Coo = pr_Coo + WHub2Epim[iCo][jCo]*OHub[jCo];
											}
										}
									if(HubID==2)//Body Hub
										{
										 for(jCo=0; jCo<42; jCo++)
											{
												pr_Coo = pr_Coo + WBody2Epim[iCo][jCo]*BHub[jCo];
											}
										}

									HE[iCo]=pr_Coo*Episodes[icn][iCo]; //bloody
									sumhe=tempHE+ HE[iCo];
									tempHE=sumhe;
								}
					//==========There is potential for remembering here ========================
					if((tempHE>0))
						{
							TagPQ=1;
							kk=kk+1;
							cout<<"generating Partial cue"<<kk<<"  "<<icn<< tempHE <<endl;

						//=====================Add Acn==============================================
							//for(iCo=0; iCo<1000; iCo++)   //commented VM 12 12
							//		{
							//			int pr_CooA=0;
							//			 for(jCo=0; jCo<9; jCo++)
							//				{
							//					pr_CooA = pr_CooA + WAct2Epim[iCo][jCo]*Action[jCo];
							//				}

							//			AE[iCo]=(pr_CooA+HE[iCo])*Episodes[icn][iCo]; //bloody
							//		}
						// Increment no of past experiences, Add tags
						// Retrieve full memo
						// Store experience in WM
						//====================================Unfold2by2==================================
							int counUnf=0;
							for(iCo=0; iCo<20; iCo++)
									{
									  for(jCo=0; jCo<50; jCo++)
											{
												//VunF[iCo][jCo]=AE[counUnf]; //converted to HE
												VunF[iCo][jCo]=HE[counUnf];
												counUnf=counUnf+1;
											}
										}

					//==================================Add Hub Tags===================     
							if(HubID==2) //Body
							{
								 for(iCo=0; iCo<20; iCo=iCo+1)
								{
								  int sumUnf=0;
								  for(jCo=0; jCo<50; jCo++)
									  {
									   sumUnf=sumUnf+VunF[iCo][jCo];
									  }
								  if(sumUnf>0)
								  {
								  VunF[iCo][46]=1;
								  }
								} //this is for body hub tag
							}
							//=================================
							if(HubID==1) //Object
							{
								 for(iCo=0; iCo<20; iCo=iCo+1)
								{
								  int sumUnf=0;
								  for(jCo=0; jCo<50; jCo++)
									  {
									   sumUnf=sumUnf+VunF[iCo][jCo];
									  }
								  if(sumUnf>0)
								  {
								  VunF[iCo][49]=1;
								  VunF[iCo][47]=1;
								//  VunF[iCo][42]=1;
								  }
								} //this is for body hub tag
								 if(GoalID==10)
								 {
								   VunF[0][49]=1;
								   VunF[0][48]=1;
								   VunF[0][9]=1;
								   VunF[0][21]=1;

								 }
							}
					}
			
				}

		if((GoalRoot==14)&&(GoalNAact[icn][1]==GoalIDRef))
					  {
					  kk=kk+1;
					  //NRelEp=NRelEpi;
				//	  //cout<<"generating Partial cue for Goal"<<endl;
					  TagPQ=1;
						  ActivIndex=GoalNAact[icn][0];
						  VunF[ActivIndex-1][GoalIDRef-1]=1; 
						  VunF[ActivIndex-1][48]=1; 
						  VunF[ActivIndex-1][49]=1; 
						  VunF[ActivIndex][47]=1; 
						  VunF[ActivIndex][49]=1; 
						  if(GoalIDRef!=10){
						 //Pointer to the Global workspace activation
                          VunF[ActivIndex-1][GoalIDRef+11]=1; 
						  VunF[ActivIndex-1][GoalIDRef+23]=1; 
						  VunF[ActivIndex][38]=1; 
						  VunF[ActivIndex][39]=1; 
						  VunF[ActivIndex][40]=1; 
						  VunF[ActivIndex][41]=1; 
						  }
						  if(GoalIDRef==10){
						  VunF[ActivIndex][32]=1; 
						  }
					  }
				// //cout<<"Goal Partial Cue"<<VunF[6][0]<<VunF[6][49]<<endl;
				// printf("%d" "%d" "%d \n",kk,VunF[6][0],VunF[6][49]);
				 //=======================================================================================
			 if(TagPQ==1)
						{
					 int counti=0;
								for(iCo=0; iCo<20; iCo++)
									{
									  for(jCo=0; jCo<50; jCo++)
											{
												PartCue[kk-1][counti]=VunF[iCo][jCo];
												Uini[counti]=VunF[iCo][jCo];
												PCue << PartCue[kk-1][counti]<< "    ";
												counti=counti+1;
											}
									 }
							PCue << "    " << endl;
				//========================Partical cue is generated and written==========================================
				// Retrieve full experience from partial cue in the present context, store it in appropriate location in NrELeP
		     RetrievalFromCue(kk-1);
                for(iCo=0; iCo<1000; iCo++)
						 {
						 Hee<< NRelPast[kk-1][iCo]<< "    ";
						 }
						 Hee << "    " << endl; 
	     	 // //cout<<"KK Val"<<kk<<endl; 
				}

				 //=======================================================================================
			 } //icn not equal to previously sent plan 
		} //icn loop
			
	
	//===================================================================

//GUI Commented out
	////cout<<"Sending Remembered past experiences to DARWIN GUI"<<endl;
	//Bottle& Remembered = RememberedMemories.prepare();	
	//Remembered.clear();
	//Remembered.addInt(NRelEp-1);
	//	// The GUI Stuff is commented //==============================================
 //   for(iCo=0; iCo<kk; iCo++)
	//  {
	//			//cout<<"Sending Remembered past experiences to DARWIN GUI: "<<iCo<<endl;
	//			Bottle& Remembered = RememberedMemories.prepare();	
	//			Remembered.clear();
	//			string str;
	//			 if(iCo == 0)
	//			  {
	//				  str = "rem0";
	//			  }
	//			  else if(iCo == 1)
	//			  {
	//				  str = "rem1";
	//			  }
	//			  else if(iCo == 2)
	//			  {
	//				  str = "rem2";
	//			  }
	//			  else if(iCo == 3)
	//			  {
	//				  str = "rem3";
	//			  }
	//			  else if(iCo == 4)
	//			  {
	//				  str = "rem4";
	//			  }		 
	//			printf("%s \n",str.c_str()); // rem0, rem1, rem2, rem3
	//			Remembered.addString(str.c_str());

	//			Bottle& remvalues = Remembered.addList();
	//			remvalues.clear();

	//			for(jCo=0; jCo<1000; jCo++)
	//			{					
	//				remvalues.addInt(NRelPast[iCo][jCo]);
	//			}
	// 
	//		RememberedMemories.write();
	//		//Time::delay(2);
 //   }
		PCue.close();
	return kk;
};


void EpisodicMemThread::RetrievalFromCue(int Nrelpos)
{
double delt=0.0015,inhibi=0,sumV=0;
int j=0, k=0;
double VinstT[1000],TempAct[1000],Vsig[1000];

		for (j=0; j<1000; j++) 
				 {
					Vsig[j]=Uini[j];	//Uini stores the partial cue, Vnet is the final activity			
				 }
		           					
for (k=1; k<500; k++)
	 {
		    sumV=0;
			for (j=0; j<1000; j++) //summation of init cond
				 {
					sumV=sumV+Vsig[j];				
				 }
			inhibi=-30+(3.5*sumV); //net inhibitory network current
			if(inhibi<=0) 
			{
			 inhibi=0;
			}
			////cout << inhibi << endl ;
			//==================== Instantaneous VT 1000*1000 x 1000*1========================================
            int iCo,jCo;
			for(iCo=0; iCo<N; iCo++)
				{
				    double pr_Co=0;
					 for(jCo=0; jCo<N; jCo++)
						{
							pr_Co = pr_Co + data[iCo][jCo]*Vsig[jCo];
						}

                    VinstT[iCo]=pr_Co; 
				}
			//====================================================================
          for(iCo=0; iCo<N; iCo++)
			   {
               TempAct[iCo]= delt*((-0.01*Uini[iCo])+VinstT[iCo]-inhibi);
			   Uini[iCo]=Uini[iCo]+TempAct[iCo];
			   if(Uini[iCo]<=0) 
				   {
				     Vsig[iCo]=0;
				   }
				  if(Uini[iCo]>0) 
				   {
				      Vsig[iCo]= Uini[iCo];
				   }
			   } 
   
	  }
               				
//=====================================================================
      int iCo;
		fileName = pathPrefix;
		fileName.append("Remembered.txt");
		ofstream RecMem(fileName.c_str());
      //ofstream RecMem("Remembered.txt");
    			for (iCo=0; iCo<1000; iCo++)
				    	{
                         if(Vsig[iCo]>0) 
						   {
							  VnetAct[iCo]=1;
						   }
						 if(Vsig[iCo]<=0) 
						   {
							  VnetAct[iCo]=0;
						   }
						 RecMem << Vsig[iCo]<< endl;
						 NRelPast[Nrelpos][iCo]=VnetAct[iCo];
						}
};


void EpisodicMemThread::MemMaint()
{
	int T[1000][1000],m,n,j;
    for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						T[m][n]=0;
					}
				}

	for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						for (j=0; j<NumEpi; j++)
						{
						T[m][n]=T[m][n]+ (Episodes[j][m]*Episodes[j][n]);
						}
						if(m==n)
						{
						T[m][n]=0;
						}
					}
				}

	for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						if(T[m][n]>0)
						{
						T[m][n]=1;
						}
						else
						{
						T[m][n]=0;
						}
                     data[m][n]=T[m][n];
					}
	}
		
};



void EpisodicMemThread::InitializeAM()
 {
	int m=0,n=0;
    fileName = pathPrefix;
	fileName.append("Numepi.txt");
	//ifstream NEp(fileName.c_str()); //Commented it 07/12
   // NEp>> NumEpi;
	NumEpi=10;//just for testing
	//GoalID=1;//just for testing..this must come from the User-Observer loop
	cout << "Number of episodes experiences in memory" << "\t" <<NumEpi << endl ;
/*	if(Largeness==0)
		{
		  NumEpi=3;
		}*/  //This is related to stacking goal commented 07/12
  	fileName = pathPrefix;
	//fileName.append("Episodes77N.txt");
	fileName.append("E770.txt");  //append
	ifstream EpiW(fileName.c_str());//loaded the weights for Reivesd Body Hub-Goal-Object Hub network , need to swich to the right neural net in the future based on Context
    if(!EpiW)     
           { cout << "Error opening Network weight matrix" << endl; }
    else
           { cout << "Loading Existing expereince" << endl;}
		for (m =0; m<25; m++)
				{
					for (n=0; n<1000; n++)
					{
						 EpiW >> Episodes[m][n];
						//  cout << Episodes[m][n] << endl ;
					}
				}

		cout << "Loading Episodes" <<Episodes[6][0] << endl ;
//===================================================================================

	fileName = pathPrefix;
	//fileName.append("WMems77N.txt");
	fileName.append("WeightNOPres.txt");
	ifstream TCo(fileName.c_str());  //Episodic Patch new

data = (int **) malloc(1000 * sizeof(int)); 
for (int h = 0; h < 1000; h++)  
         data[h] = (int*) malloc(1000*sizeof(int));
   
 if(!TCo)
           { cout << "Error opening Network weight matrix" << endl; }
    else
           { cout << "Loading Neural connectivity of AssocMem" << endl;}

 for (m =0; m<1000; m++){
	 for (n =0; n<1000; n++)
				{
						 TCo >> data[m][n];
					    // cout << data[m][n] ;
					}
 }

	//cout << "Loaded Neural connectivity" << endl;

 /////////////////////////////////////////////////////////////////////////////////////////////////Change to visualize weight matrix
		
 /*Bottle& weight = PlanorXplore.prepare();
				weight.clear();
				weight.addString("weight");
				Bottle& weightValues = weight.addList();
				weightValues.clear();
				for(int i=0; i<1000; i++)
					{
					 //PXper<< PlanPastExp[i] <<endl;
					for(int j=0; j<1000; j++)
					{
                     weightValues.addInt(data[i][j]);
					}
				}
			    //cout<<"Sending Weight Matrix to DARWIN GUI"<<endl;
             	PlanorXplore.write();*/

////////////////////////////////////////////////////////////////////////////////////

 
		cout << "Loading Episodes" <<data[100][100] << endl ;
	//DumpAM();
    //===================================================================================

 	fileName = pathPrefix;
	fileName.append("cue.txt");
 	ifstream Cu(fileName.c_str());
    { //cout << "Loading partial cue" << endl;
	}
					for (n=0; n<1000; n++)
					{
						 Cu >> Uini[n];
				    }
                    //cout << Uini[42] << endl ;
					//cout << Uini[55] << endl ;
//===================================================================================
					// RetrievalFromCue();
//===================================================================================
fileName = pathPrefix;
//fileName.append("whepisN.txt");
fileName.append("WHubEpNew.txt");
ifstream Whub(fileName.c_str());
  for (m =0; m<1000; m++)
				{
					for (n=0; n<42; n++)
					{
						 Whub >> WHub2Epim[m][n];
					}
				}
cout << "Loading Hub Epim Net" <<WHub2Epim[73][23] << endl ;

//===================================================================================
fileName = pathPrefix;
fileName.append("WHubToEpisN.txt");
ifstream WhubTt(fileName.c_str());
  for (m =0; m<42; m++)
				{
					for (n=0; n<1000; n++)
					{
						 WhubTt >> WHub2EpimT[m][n];
					}
				}
//cout << "Loading Hub Epim NetT" <<WHub2EpimT[18][18] << endl ;

//===================================================================================
   	fileName = pathPrefix;
	fileName.append("WActToEpis.txt");
   	ifstream WAcE(fileName.c_str());
    if(!WAcE)
           { //cout << "Error opening Hub Epim Net" << endl;
	}
    else
           { //cout << "Loading Act Epim Net" << endl;
	}
		for (m =0; m<1000; m++)
				{
					for (n=0; n<9; n++)
					{
						 WAcE >> WAct2Epim[m][n];
					}
				}
					//cout << WAct2Epim[55][5] << endl ;
					
//===================================================================================
   
//===================================================================================
   for (m =0; m<42; m++)
	   {
	//	OHub[m]=0;
		VSSP[m]=0;
		//BHub[m]=0;
	   }
//OHub[20]=1;  // This initialization can from from bottom up perception
//OHub[13]=1;
//OHub[35]=1;
//OHub[18]=1;
//OHub[37]=1;
//BHub[20]=1;
//BHub[21]=1;
//BHub[22]=1;
SumVSSP=0;
for (m =0; m<42; m++)
	   {
		VSSP[m]=OHub[m];
		if(m<36)  // note that u only take shape into account assuming 2 shapes of different sizes do not come into picture
			{
			   SumVSSP=SumVSSP+OHub[m];
			}
	   }
//cout << "Visuo Spatial sketch pad cumulative activity" << "\t" << SumVSSP << endl ;
//===================================================================================
for (m =0; m<9; m++)
	   {
		Action[m]=0;
	    }
	//  Action[5]=1;   //this initialization can come through user goal, all set

//===================================================================================

for (m =0; m<15; m++)
				{
					for (n=0; n<1000; n++)
					{
						NRelPast[m][n]=0;
						NowObAcSeq[m][n]=0;
						PartCue[m][n]=0;
					}
				}
for (n=0; n<1000; n++)
	{
	 PlanPastExp[n]=0;
	}

//===================================================================================

fileName = pathPrefix;
fileName.append("GoalNact.txt");
ifstream GActivateW(fileName.c_str()); //loaded the weights for Reivesd Body Hub-Goal-Object Hub network , need to swich to the right neural net in the future based on Context
    if(!GActivateW)
           { cout << "Error opening weight matrix" << endl; }
    else
           { cout << "Loading Existing expereince" << endl;}
		for (m =0; m<9; m++)
				{
					for (n=0; n<2; n++)
					{
						 GActivateW >> GoalNAact[m][n];
						//  cout << Episodes[m][n] << endl ;
					}
				}


		//===================================================================================
fileName = pathPrefix;
fileName.append("WBodytoEpimT.txt");
ifstream WBodhub(fileName.c_str());
  for (m =0; m<1000; m++)
				{
					for (n=0; n<42; n++)
					{
						 WBodhub >> WBody2Epim[m][n];
					}
				}
//cout << "Loading Body to Hub Epim Net" <<WBody2Epim[18][18] << endl ;

//===================================================================================
};


void EpisodicMemThread::DumpAM()
{
	int m=0,n=0;
	fileName = pathPrefix;
  	fileName.append("WeightN.txt");
	ofstream NetWO(fileName.c_str());
    		for (m =0; m<1000; m++)
				{
					for (n=0; n<1000; n++)
					{
						 NetWO << data[m][n] << "    ";
					}
					NetWO << "    " << endl;
				}
			free(data); 

//====================Updating new episodes learnt in the now ===============================
			fileName = pathPrefix;
  			fileName.append("EpisodesNew.txt");
			ofstream EpiWO(fileName.c_str());
    		for (m =0; m<25; m++)
				{
					for (n=0; n<1000; n++)
				    	{
						 EpiWO << Episodes[m][n]<< "    ";
						}
					EpiWO << "    " << endl;
				}
//======================== Updating number of episdoes of experiences in the memory at present =========
    fileName = pathPrefix;
  	fileName.append("Numepi.txt");
    ofstream NEpo(fileName.c_str());
    NEpo<< NumEpi;
	cout << "Number of episodes experiences in memory" << "\t" <<NumEpi << endl ;

}


int EpisodicMemThread::Random_Zel(int lim)
{
   srand( (unsigned)time( NULL ) );
   int addn=rand();
   static long a = 100001+addn; 
   int p;
   a = (a * 125) % 2796203;   
   p= ((a % lim));
   //cout << "\n\n Random Select %d" << "\t" << p << endl;
   return p;
};

 
