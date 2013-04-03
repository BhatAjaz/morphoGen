#include <string.h>
#include<time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
// #include <windows.h> //Rea change 10/1/13
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include "pmp.h"
#include "vision.h"
#include "segment.h"

#include <yarp/sig/Vector.h>

 //
 using namespace yarp::os;
 using namespace yarp::sig;
 using namespace yarp::sig::draw;
 using namespace yarp::sig::file;
 using namespace yarp;

 int main(int argc, char **argv)

/* Passes the appropriate coordinates and other arguements to the MotCon function 
for the pushing task*/
{    
        Network yarp;
        Network::init();
	PassiveMotionParadigm P1;
	P1.Reason(1);
	//	Sleep(10000);
	Network::fini();
	return 0;
}
