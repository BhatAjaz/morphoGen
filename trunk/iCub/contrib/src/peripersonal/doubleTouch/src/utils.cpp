#include "utils.h"

genericHandler::genericHandler(string _name, string _robot, string _part, int _verbosity): name(_name), robot(_robot), part(_part), verbosity(_verbosity) {}

int genericHandler::printMessage(const int level, const char *format, ...) const {

  if (verbosity>=level)  {

    fprintf(stdout,"*** %s: ",name.c_str());

    va_list ap;
    va_start(ap,format);
    int ret=vfprintf(stdout,format,ap);
    va_end(ap);
    
    return ret;
  }
  else  return -1;
}

/******************************************/
/******************************************/
/******************************************/

limbHandler::limbHandler(string _name, string _robot, string _part, int _verbosity): genericHandler(_name,_robot,_part,_verbosity) {

	if 		(part == "right_arm") part = "right";
	else if (part == "left_arm")  part = "left";

	if 		(part == "right") ppart = "right_arm";
	else if (part == "left")  ppart = "left_arm";
	else if (part == "torso") ppart = "torso";
}

bool limbHandler::init() {
	Property Opt;
	Opt.put("robot",  robot.c_str());
	Opt.put("part",   ppart.c_str());
	Opt.put("device", "remote_controlboard");
	Opt.put("remote", ("/"+robot+"/"+ppart).c_str());
	Opt.put("local",  ("/"+name+"/posCtrl/"+ppart).c_str());

	if (!dd.open(Opt)){
	printMessage(0,"ERROR: could not open %s PolyDriver!\n",ppart.c_str());
	return false;
	}

	// open the view
	bool ok = 1;

	if (dd.isValid()) {
		ok = ok && dd.view(iencs);
	
		if (part != "torso") {			// We don't need these for the torso
			ok = ok && dd.view(ipos);
			ok = ok && dd.view(ictrl);
			ok = ok && dd.view(iimp);
		}
	}

	if (!ok) {
		printMessage(0,"\nERROR: Problems acquiring PosCtrl %s interfaces!!!!\n\n",part.c_str());
		return false;
	}

	nj=0;

	if (part != "torso") {
		
		ipos->getAxes(&nj);
		Vector tmp(nj,0.0);

		for (int i = 0; i < nj; i++) {
			tmp[i] = 50.0;
			ipos  -> setRefSpeed (i, 20.0);
			iimp  -> setImpedance(i, 0.111, 0.014);
		}
		ipos -> setRefAccelerations(tmp.data());
	}

    iencs->getAxes(&nj);
    encoders.resize(nj,0.0);

    return true;
}

Vector limbHandler::getEncoders() {
	iencs->getEncoders(encoders.data());	
	return encoders;
}

limbHandler::~limbHandler() {
	dd.close();
}

/******************************************/
/******************************************/
/******************************************/

Vector armDynHandler::evalVel(const Vector &x) {
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return linEst->estimate(el);
}

Vector armDynHandler::evalAcc(const Vector &x) {
    AWPolyElement el;
    el.data=x;
    el.time=Time::now();

    return quadEst->estimate(el);
}

armDynHandler::armDynHandler(string _name, string _robot, string _part, int _verbosity, limbHandler *_torso): genericHandler(_name,_robot,_part,_verbosity), torso(_torso) {

	if 		(part == "right_arm") part = "right";
	else if (part == "left_arm")  part = "left";
	if 		(part == "right") ppart = "right_arm";
	else if (part == "left")  ppart = "left_arm";

	port_FT = new BufferedPort<Vector>;

    q.resize(10,0.0);           dq.resize(10,0.0);      d2q.resize(10,0.0);
    w0.resize(3,0.0);           dw0.resize(3,0.0);      d2p0.resize(3,0.0);
    Fend.resize(3,0.0);         Mend.resize(3,0.0);
    F_measured.resize(6,0.0);   F_iDyn.resize(6,0.0);   F_offset.resize(6,0.0);
    FT.resize(6,0.0);
    d2p0[2]=9.81; // frame 0 acceleration
    store_context_id = -1;

	arm   = new limbHandler(name,robot, part  ,verbosity);

    limb = new iCubArmDyn(part.c_str());
    limb 		 -> releaseLink(0);
    limb 		 -> releaseLink(1);
    limb         -> releaseLink(2);
    chain = limb -> asChain();
	sens = new iDynInvSensorArm(chain,part,DYNAMIC);

    // I parametri sono la lunghezza massima della finestra e la soglia
    linEst  = new AWLinEstimator(16,1.0);
    quadEst = new AWQuadEstimator(25,1.0);

    limb->setAng(q);
    limb->setDAng(dq);
    limb->setD2Ang(d2q);
    limb->prepareNewtonEuler(DYNAMIC);
    limb->initNewtonEuler(w0,dw0,d2p0,Fend,Mend);  

    pmp = new PmpClient();
}

bool armDynHandler::init() {
	if (!arm   -> init()) return false;

    encodersA.resize(arm  ->getAxes(),0.0);
    encodersT.resize(torso->getAxes(),0.0);

    /******************************************/

    Property Opt;
    Opt.put("robot", robot.c_str());
    Opt.put("part", ppart.c_str());
    Opt.put("device","cartesiancontrollerclient");
    Opt.put("remote", ("/"+robot+"/cartesianController/"+ppart).c_str());
    Opt.put("local",  ("/"+name+"/cartesianClient/"+ppart).c_str());

    if ((!cc.open(Opt)) || (!cc.view(icart))){
        printMessage(0,"Error: could not open %s CartesianController!\n",ppart.c_str());
        return false;
    }

    icart -> storeContext(&store_context_id);

    // set trajectory time
    if (robot == "icubSim") {
        icart  -> setTrajTime(1);
        icart  -> setInTargetTol(0.005);
    }
    else if (robot == "icub") {
        icart  -> setTrajTime(0.75);
        icart  -> setInTargetTol(0.01);
    }

    icart  -> setTrackingMode(true);

    // limit the torso pitch (20 degrees forward, 10 backward)
    // icart  -> setLimits(0,-10,20);   // default is -10,70
    // icart  -> setLimits(1,-20,20);   // default is -30,30
    // icart  -> setLimits(2,-30,30);   // default is -50,50

    /******************************************/
    Property OptPmp;
    OptPmp.put("verbosity", verbosity); // somehow prevent pmp client to print unwanted stuff
    OptPmp.put("remote", ("/"+part+"_pmp_server").c_str());
    OptPmp.put("local",  ("/"+name+"/"+part+"_pmp_client").c_str());

    if (!pmp -> open(OptPmp)) {
        printMessage(0,"Error: could not open %s Pmp Client!\n",ppart.c_str());
        return false;
    }
    /******************************************/

    if (robot == "icub") {
    	port_FT -> open(("/"+name+"/"+ppart+"/FT:i").c_str());
    	ft = port_FT -> read(true);

        Vector F_sum(6,0.0);
        
        printMessage(2,"*****************************************");
        for (int i=0; i<10; i++) {
            compute_F_iDyn();
            ft = port_FT->read(true);
            F_sum = F_sum + *ft - F_iDyn;
            
            printMessage(3,"FT_offset %s: \t%s\n",ppart.c_str(),(*ft-F_iDyn).toString().c_str());
        }
        
        F_offset = F_sum/10;
        
        printMessage(2,"FT_offset_avg %s: \t%s\n",ppart.c_str(),F_offset.toString().c_str());
        printMessage(2,"*****************************************");
    }

    return true;
}

void armDynHandler::updateState() {
    encodersA = arm   -> getEncoders();
    encodersT = torso -> getEncoders();

    for (int i=0;i<3;i++)             q(i) = encodersT(2-i);
    for (int i=3;i<q.length();i++)    q(i) = encodersA(i-3);

    dq  = evalVel(q);
    d2q = evalAcc(q);

    limb->setAng  (CTRL_DEG2RAD * q);
    limb->setDAng (CTRL_DEG2RAD * dq);
    limb->setD2Ang(CTRL_DEG2RAD * d2q);
}

void armDynHandler::compute_F_iDyn() {

	updateState();

    limb->computeNewtonEuler(w0,dw0,d2p0,Fend,Mend);
    sens->computeSensorForceMoment();
    
    F_iDyn = -1.0 * sens->getSensorForceMoment(); 
}

armDynHandler::~armDynHandler() {

    if (arm) { delete arm; arm = 0; }
    // Torso is deleted by the doubleTouchThread that instantiated it

    icart -> restoreContext(store_context_id);
    icart -> stopControl();
    cc.close();
    printMessage(1,"%s dynHandler CartCtrl successfully closed!\n",part.c_str());
    if (port_FT) {
    	port_FT -> interrupt();
    	port_FT -> close();
    	// delete port_FT;
    	port_FT = 0;
    }
	printMessage(1,"%s dynHandler port successfully closed!\n",part.c_str());

    if (sens)   { delete sens;    sens = 0;    }
    if (limb)   { delete limb;    limb = 0;    }
    if (linEst) { delete linEst;  linEst = 0;  }
    if (quadEst){ delete quadEst; quadEst = 0; }
    if (ft)     { delete ft;      ft = 0;      }

    printMessage(0,"%s dynHandler successfully closed!\n",part.c_str());
}
// empty line to make gcc happy