void PassiveMotionParadigm::MessageDevDriverR()
 {
    
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/test/client");   //local port names
    options.put("remote", "icub/right_arm");         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }

	Property optionsL;
    optionsL.put("device", "remote_controlboard");
    optionsL.put("local", "/test/clientL");   //local port names
    optionsL.put("remote", "icub/left_arm");         //where we connect to

    // create a device
    PolyDriver robotDeviceL(optionsL);
    if (!robotDeviceL.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }

	Property optionsT;
    optionsT.put("device", "remote_controlboard");
    optionsT.put("local", "/test/clientT");   //local port names
    optionsT.put("remote", "icub/torso");         //where we connect to

    // create a device
    PolyDriver robotDeviceT(optionsT);
    if (!robotDeviceT.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }
	
    // create a device
 

    IPositionControl *pos;
    IEncoders *encs;

	IPositionControl *posL;
    IEncoders *encsL;

	IPositionControl *posT;
    IEncoders *encsT;


    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

	 bool okL;
    okL = robotDeviceL.view(posL);
    okL = okL && robotDeviceL.view(encsL);

	bool okT;
    okT = robotDeviceT.view(posT);
    okT = okT && robotDeviceT.view(encsT);

	
    if (!ok) {
        printf("Problems acquiring RIGHT ARM interfaces\n");
        //return 0;
    }

	 if (!okL) {
        printf("Problems acquiring LEFT ARM interfaces\n");
        //return 0;
    }

 if (!okT) {
        printf("Problems acquiring Torso interfaces\n");
        //return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
	//printf("Joints %d \n", nj);

	int njL=0;
    posL->getAxes(&njL);
	//printf("Joints Left %d \n", njL);

	int njT=0;
    posT->getAxes(&njT);
	//printf("Joints Torso %d \n", njT);


    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);

	 Vector encodersL;
    Vector commandL;
    Vector tmpL;
    encodersL.resize(njL);
    tmpL.resize(njL);
    commandL.resize(njL);

Vector encodersT;
    Vector commandT;
    Vector tmpT;
    encodersT.resize(njT);
    tmpT.resize(njT);
    commandT.resize(njT);


    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 60.0;
    }
    pos->setRefAccelerations(tmp.data());

	for (i = 0; i < njL; i++) {
         tmpL[i] = 60.0;
    }
    posL->setRefAccelerations(tmpL.data());

	for (i = 0; i < njT; i++) {
         tmpT[i] = 60.0;
    }
    posT->setRefAccelerations(tmpT.data()); 

	/*
	if needed we can check the position of our axes by doing:

		\code
		enc->getEncoders(encoders.data());
		\endcode
	*/

    for (i = 0; i < nj; i++) {
        tmp[i] = 30.0;
        pos->setRefSpeed(i, tmp[i]);
    }

	 for (i = 0; i < njL; i++) {
        tmpL[i] = 30.0;
        posL->setRefSpeed(i, tmpL[i]);
    }

	 for (i = 0; i < njT; i++) {
        tmpT[i] = 30.0;
        posT->setRefSpeed(i, tmpT[i]);
    } 

    //pos->setRefSpeeds(tmp.data()))
    
    //fisrst zero all joints
    //
    command=0;
	commandL=0;
	commandT=0;
	

//for (i = 0; i < 5; i++) 
//{

	 /*        command[0]=-50;
             command[1]=20;
             command[2]=-10;
             command[3]=50;
		     pos->positionMove(command.data());

			 commandL[0]=-50;
             commandL[1]=20;
             commandL[2]=-10;
             commandL[3]=50;
		     posL->positionMove(commandL.data());

		      
			  commandT[0]=25;
			   commandT[1]=10;
			    commandT[2]=25;

			   posT->positionMove(commandT.data());

			 bool done2=false;
		while(!done2)
				{
					pos->checkMotionDone(&done2);
					Time::delay(0.1);
				}
				*/ 

			command[0]=ang4;
			command[1]=ang5;
			command[2]=ang6;
			command[3]=ang7;
			command[4]=ang8;
			command[5]=ang9;
			command[6]=ang10;
			command[7]=angCup;
			command[8]=angT1;
			command[9]=angT2;
			command[10]=angT3;
			command[11]=angI1;
			command[12]=angI2;
			command[13]=angM1;
			command[14]=angM2;
			command[15]=angRP;

			commandL[0]=ang4L;
			commandL[1]=ang5L;
			commandL[2]=ang6L;
			commandL[3]=ang7L;
			commandL[4]=ang8L;
			commandL[5]=ang9L;
			commandL[6]=ang10L;
			commandL[7]=angCupL;
			commandL[8]=angTL1;
			commandL[9]=angTL2;
			commandL[10]=angTL3;
			commandL[11]=angIL1;
			commandL[12]=angIL2;
			commandL[13]=angML1;
			commandL[14]=angML2;
			commandL[15]=angRPL;

			pos->positionMove(command.data());

			posL->positionMove(commandL.data());


			 commandT[0]=(-1*ang3);
			 commandT[1]=(1*ang2);
			 commandT[2]=(1*ang1);

			   posT->positionMove(commandT.data());
	//bool ok = pos->positionMove(command_position.data());
    
			bool done=false;

			while(!done)
			{
				pos->checkMotionDone(&done);
				Time::delay(0.1);
			}

			bool doneL=false;

			while(!doneL)
			{
				posL->checkMotionDone(&doneL);
				Time::delay(0.1);
			}

			bool doneT=false;

			while(!doneT)
			{
				posT->checkMotionDone(&doneT);
				Time::delay(0.1);
			}
//	}

    robotDevice.close();
	robotDeviceL.close();
	robotDeviceT.close();
    
    //return 0;

 }

 void PassiveMotionParadigm::MessageDevDriverR()
 {
    
    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", "/test/client");   //local port names
    options.put("remote", "icub/right_arm");         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }

	IPositionControl *pos;
    IEncoders *encs;

	bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

	if (!ok) {
        printf("Problems acquiring RIGHT ARM interfaces\n");
        //return 0;
    }
 
    int nj=0;
    pos->getAxes(&nj);
	//printf("Joints %d \n", nj);

    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
	
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 60.0;
    }
    pos->setRefAccelerations(tmp.data());

	/*
	if needed we can check the position of our axes by doing:

		\code
		enc->getEncoders(encoders.data());
		\endcode
	*/

    for (i = 0; i < nj; i++) {
        tmp[i] = 30.0;
        pos->setRefSpeed(i, tmp[i]);
    }
    command=0;
	
	    	command[0]=ang4;
			command[1]=ang5;
			command[2]=ang6;
			command[3]=ang7;
			command[4]=ang8;
			command[5]=ang9;
			command[6]=ang10;
			command[7]=angCup;
			command[8]=angT1;
			command[9]=angT2;
			command[10]=angT3;
			command[11]=angI1;
			command[12]=angI2;
			command[13]=angM1;
			command[14]=angM2;
			command[15]=angRP;

			
			pos->positionMove(command.data());

			bool done=false;

			while(!done)
			{
				pos->checkMotionDone(&done);
				Time::delay(0.1);
			}

			
    robotDevice.close();
	
 }


void PassiveMotionParadigm::MessageDevDriverL()
 {
   

	Property optionsL;
    optionsL.put("device", "remote_controlboard");
    optionsL.put("local", "/test/clientL");   //local port names
    optionsL.put("remote", "icub/left_arm");         //where we connect to

    // create a device
    PolyDriver robotDeviceL(optionsL);
    if (!robotDeviceL.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }


	IPositionControl *posL;
    IEncoders *encsL;

	bool okL;
    okL = robotDeviceL.view(posL);
    okL = okL && robotDeviceL.view(encsL);

	if (!okL) {
        printf("Problems acquiring LEFT ARM interfaces\n");
        //return 0;
    }

    int njL=0;
    posL->getAxes(&njL);
	//printf("Joints Left %d \n", njL);

	 Vector encodersL;
    Vector commandL;
    Vector tmpL;
    encodersL.resize(njL);
    tmpL.resize(njL);
    commandL.resize(njL);


	for (i = 0; i < njL; i++) {
         tmpL[i] = 60.0;
    }
    posL->setRefAccelerations(tmpL.data());

	
	 for (i = 0; i < njL; i++) {
        tmpL[i] = 30.0;
        posL->setRefSpeed(i, tmpL[i]);
    }

	 
	commandL=0;
	
			commandL[0]=ang4L;
			commandL[1]=ang5L;
			commandL[2]=ang6L;
			commandL[3]=ang7L;
			commandL[4]=ang8L;
			commandL[5]=ang9L;
			commandL[6]=ang10L;
			commandL[7]=angCupL;
			commandL[8]=angTL1;
			commandL[9]=angTL2;
			commandL[10]=angTL3;
			commandL[11]=angIL1;
			commandL[12]=angIL2;
			commandL[13]=angML1;
			commandL[14]=angML2;
			commandL[15]=angRPL;

			posL->positionMove(commandL.data());


			bool doneL=false;

			while(!doneL)
			{
				posL->checkMotionDone(&doneL);
				Time::delay(0.1);
			}

	robotDeviceL.close();
 }

void PassiveMotionParadigm::MessageDevDriverT()
 {
    
	Property optionsT;
    optionsT.put("device", "remote_controlboard");
    optionsT.put("local", "/test/clientT");   //local port names
    optionsT.put("remote", "icub/torso");         //where we connect to

    // create a device
    PolyDriver robotDeviceT(optionsT);
    if (!robotDeviceT.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        //return 0;
    }
	
	IPositionControl *posT;
    IEncoders *encsT;

	bool okT;
    okT = robotDeviceT.view(posT);
    okT = okT && robotDeviceT.view(encsT);

 if (!okT) {
        printf("Problems acquiring Torso interfaces\n");
        //return 0;
    }

	int njT=0;
    posT->getAxes(&njT);
	//printf("Joints Torso %d \n", njT);

   Vector encodersT;
    Vector commandT;
    Vector tmpT;
    encodersT.resize(njT);
    tmpT.resize(njT);
    commandT.resize(njT);

	for (i = 0; i < njT; i++) {
         tmpT[i] = 60.0;
    }
    posT->setRefAccelerations(tmpT.data()); 


	 for (i = 0; i < njT; i++) {
        tmpT[i] = 30.0;
        posT->setRefSpeed(i, tmpT[i]);
    } 

	commandT=0;
			 commandT[0]=(-1*ang3);
			 commandT[1]=(1*ang2);
			 commandT[2]=(1*ang1);

			   posT->positionMove(commandT.data());


			bool doneT=false;

			while(!doneT)
			{
				posT->checkMotionDone(&doneT);
				Time::delay(0.1);
			}

	robotDeviceT.close();
 
    //return 0;

 }

