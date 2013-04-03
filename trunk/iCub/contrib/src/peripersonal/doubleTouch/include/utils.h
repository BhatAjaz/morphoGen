#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynTransform.h>
#include <iCub/pmp/pmp_client.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>

#include <gsl/gsl_math.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::pmp;

using namespace std;

YARP_DECLARE_DEVICES(icubmod)

#ifndef __UTILS__
#define __UTILS__
//******************************************************

class genericHandler {
public:
	genericHandler(string _name, string _robot, string _part, int _verbosity);

	string name;
	string robot;
	string part;       // short name (i.e. "right")
    string ppart;      // short name (i.e. "right_arm")
	int verbosity;

	int printMessage(const int level, const char *format, ...) const;
};

//******************************************************

class limbHandler: public genericHandler {
private:
	// Driver for "classical" interfaces
	PolyDriver 		   dd;
public:
	// "Classical" interfaces
	IEncoders  		  *iencs;
	IPositionControl  *ipos;
	IControlMode      *ictrl;
	IImpedanceControl *iimp;

    Vector encoders;		// Encoders
    int nj;					// Number of joints

	limbHandler(string _name, string _robot, string _part, int _verbosity);

	bool 	init();
	Vector 	getEncoders();
	int 	getAxes() {return nj;}

	~limbHandler();
};

//******************************************************

class armDynHandler: public genericHandler {
private:
    limbHandler *arm;
    limbHandler *torso;

	Vector encodersA, encodersT;

    // Stuff related to iDyn
    iDynLimb      *limb;
    iDynChain     *chain;
    iDynInvSensor *sens;

    // Joint variables:
    Vector q;
    Vector dq;
    Vector d2q;

    Vector *ft, F_measured, F_iDyn;
    Vector F_offset, FT, *FT_ref;

    AWLinEstimator  *linEst;
    AWQuadEstimator *quadEst;

    Vector w0,dw0,d2p0,Fend,Mend;

    BufferedPort<Vector> *port_FT;

    Vector evalVel(const Vector &x);
    Vector evalAcc(const Vector &x);

    // Cartesian controller
    PolyDriver        cc;
    ICartesianControl *icart;
    int store_context_id;

    // PMP client
    PmpClient *pmp;
public:

	armDynHandler(string _name, string _robot, string _part, int _verbosity, limbHandler *_torso);

	bool 		       init();
    void 		       updateState();
    void 		       compute_F_iDyn();
    iDynLimb*          getLimb()      { return limb; };
    limbHandler*       getArm()       { return arm; };
    limbHandler*       getTorso()     { return torso; };
    ICartesianControl* getCartCtrl()  { return icart; };
    PmpClient*         getPmpClient() { return pmp; };
    Vector             getQ()         { return q; };

    ~armDynHandler();
};

#endif
// empty line to make gcc happy