#include "doubleTouchThread.h"

doubleTouchThread::doubleTouchThread(int _cd_rate, const string &_name, const string &_robot, int _v, bool _m, int _K, int _D, int _G) : RateThread(_cd_rate), name(_name), robot(_robot), verbosity(_v), m(_m), K(_K), D(_D), G(_G) {

  mov = 0; step = 0; unsuccess = 0; stab = 0; attempt = 0;

  d1 = 0; d2 = 0; d1_old = 0; d2_old = 0;

	cntRdr      = new BufferedPort<iCub::skinDynLib::skinContactList>;
  iCubGuiObjs = new BufferedPort<Bottle>;

  fal = 2; ual = -1;
  far = 5; uar = -1;

  trgt1 = 0; trgt2 = 0;
  obst1 = 0; obst2 = 0;

  q.resize(10,0.0);
  Twl.resize(4,4);

  cntctPosLink.resize(3,0.0);
  cntctPosWRF.resize(3,0.0);
  cntctPosEE.resize(3,0.0);
  cntctNormDir.resize(3,0.0);
  cntctPressure = -1;
  cntctLinkNum = -1;
  cntctArm = "";

  torso_handler = new limbHandler(name,robot,"torso",verbosity);
  right_handler = new armDynHandler(name,robot,"right",verbosity,torso_handler);
  left_handler  = new armDynHandler(name,robot,"left", verbosity,torso_handler);
}

bool doubleTouchThread::threadInit() {
  cntRdr      -> open(("/"+name+"/contacts:i").c_str());
  iCubGuiObjs -> open(("/"+name+"/icubgui_objects:o").c_str());

  // Property OptGaze;
  //   OptGaze.put("device","gazecontrollerclient");
  //   OptGaze.put("remote","/iKinGazeCtrl");
  //   OptGaze.put("local",("/"+name+"/gaze").c_str());
 
  // if ((!client_gaze.open(OptGaze)) || (!client_gaze.view(igaze))){
  //   printMessage(0,"Error: could not open the Gaze Controller!\n");
  //   return false;
  // }

  if (!torso_handler -> init()) {
    printMessage(0,"ERROR! torso_handler failed somewhere!\n");
    return false;
  }
  if (!right_handler -> init()) {
    printMessage(0,"ERROR! right_handler failed somewhere!\n");
    return false;
  }
  if (!left_handler -> init()) {
    printMessage(0,"ERROR! left_handler failed somewhere!\n");
    return false;
  }

  right_arm = right_handler -> getCartCtrl();
  left_arm  = left_handler  -> getCartCtrl();

  pmp_right = right_handler -> getPmpClient();
  pmp_left  = left_handler  -> getPmpClient();

  toggleJoints("disable_torso");
    
	return true;
}

void doubleTouchThread::run(){

  skinContactList *skinContacts  = cntRdr -> read(false);

  if (step != 0 && step != 1) drawTool();

  // If there's any safety risk, return back to resting position.
  if (!checkSafety()) handleIssue();
  // If robot is moving, wait for movement to end
  else if (checkMotion()) {
    cout << "step: " << step << endl;
    if (step == 0) {
      if (m) {

        printMessage(1,"dblTchThrd: moving to impedance position mode..\n");
        
        for (int i = 0; i < 5; ++i) {
          right_handler -> getArm() -> ictrl -> setImpedancePositionMode(i);
          left_handler  -> getArm() -> ictrl -> setImpedancePositionMode(i);
        }
        
        printMessage(0,"MOVING TO REST...\n");
        goToRest();
        // move the thumbs close to the hand
        right_handler -> getArm() -> ipos -> positionMove(9,70);
        left_handler  -> getArm() -> ipos -> positionMove(9,70);
      }
      step++;
      printMessage(0,"WAITING FOR CONTACT...\n");
      if (robot == "icub") Time::delay(1);
    }
    else if (step == 1) { 
     	if(skinContacts) {

        detectContact(skinContacts); // READ A CONTACT ON THE SKIN

        if (cntctArm != "") {
          printMessage(0,"CONTACT!!! Arm: %s Position: %s\n\t\t NormDir: %s Pressure: %g\n",cntctArm.c_str(),
                       cntctPosWRF.toString().c_str(),cntctNormDir.toString().c_str(),cntctPressure);

          // If motor modality is activated, go to the next step
          if (m) step++;
          // if (robot == "icub") step = 6;
          // cout << "step: " << step << endl;
          // If motor modality is not activated, simply
          // cycle back to this step (and skip anything else)
          if (robot == "icub") Time::delay(1);
        }
      }
    }
    else if (step == 2 || step == 22) {
      printMessage(0,"DOUBLE TOUCH...\n");
      printMessage(1,"Accomodating %s...\n", cntctArm.c_str());
      if (robot != "icub" && robot != "icubSim") normalAccomodation();
      
      // touched_pmp -> setPointStateToTool();
      touched_pmp -> enableField();
      touched_pmp -> enableControl();

      step++;
      if (step == 23) step = 33;
      Time::delay(0.2);
      if (robot == "icub" || robot == "icubSim") {
        Time::delay(1);
        step = 1;
      }
    }
    // STEP = 3 -> 
    else if (step == 3 || step == 33) {
      printMessage(1,"Going to taxel...\n");
      if (robot != "icub" && robot != "icubSim") goToTaxel();

      touching_pmp -> setPointStateToTool();
      touching_pmp -> enableField();
      touching_pmp -> enableControl();
      Time::delay(1);

      if (step == 33) step = 3;
      step++;
      if (robot == "icub") Time::delay(1);
    }
    // STEP = 4 -> no movement, no resting position, an event should be ended -> move back
    else if (step == 4) {
      unsuccess = 0;
      stab = 0;
      pmp_right -> disableField();
      pmp_right -> disableControl();
      pmp_right -> removeToolFrame();  
      pmp_right -> clearItems();
      // disablePmp(pmp_right);
      
      pmp_left  -> disableField();
      pmp_left  -> disableControl();
      pmp_left  -> removeToolFrame();
      pmp_left  -> clearItems();
      // disablePmp(pmp_left);
      
      step++;
      if (robot == "icub") Time::delay(1);
    }
    else if (step == 5 || step == 55) {
      printMessage(0,"MOVING TO REST...step=%i\n",step);
      goToRest();
      if (step == 55) step = 66;
      else            step++;
      if (robot == "icub") Time::delay(1);
    }
    else if (step == 6 || step == 66) {
      pmp_right -> disableField();
      pmp_right -> disableControl();
      pmp_right -> clearItems();
      // disablePmp(pmp_right);
      
      pmp_left  -> disableField();
      pmp_left  -> disableControl();
      pmp_left  -> clearItems();
      // disablePmp(pmp_right);

      if (step == 66) step = 22;
      else step = 1;
      printMessage(0,"WAITING FOR CONTACT...\n");
      if (robot == "icub") Time::delay(1);
    }
    else {
      printMessage(0,"ERROR!!! doubleTouchThread should never be here!!!\n");
      printMessage(0,"Step: %d",step);
      Time::delay(1);
    }
  }
}

bool doubleTouchThread::checkSafety() {
  return true;
}

void doubleTouchThread::handleIssue() {

}

bool doubleTouchThread::checkMotion() {

  if (step < 4) {
    bool done_l=0, done_r=0;

    left_arm  -> checkMotionDone(&done_l);
    right_arm -> checkMotionDone(&done_r);

    if (done_l && done_r) return true;
  }
  else if (step == 4) {
    Vector t(3,0.0);
    Vector x,o;

    if (Bottle *v=targetOpt2.find("center").asList()) {
  
      t.resize(v->size());
      for (size_t i=0; i<t.length(); i++)
        t[i]=v->get(i).asDouble();
    }

    touching_pmp -> getTool(x,o);
    d1 = norm(t-x);

    touching_arm -> getPose(x,o);
    d2 = norm(t-x);
    
    double diff1 = fabs(d1-d1_old);
    double diff2 = fabs(d2-d2_old);
    d1_old = d1;
    d2_old = d2;
    printMessage(1,"step=%i,\t|xTg-x|=%g [m],\t|x-xee|=%g[m],\tdiff1=%g [m],\tdiff2=%g [m],\tunsuccess=%i\n",step,d1,d2,diff1,diff2,unsuccess);
    
    if ((d1<2e-2) && (d2<2e-2)) {
      if (attempt != 0 ) attempt = 0; 
      printMessage(0,"SUCCESSFUL CONTACT!\n");
      return true;
    }
    else if ((diff1<1e-3) && (diff2<1e-3) && (d1<0.25) && (d2<0.25)) {
      if (unsuccess == 30) {
        step = 55;
        unsuccess = 0;
        d1 = 0; d1_old = 0;
        d2 = 0; d2_old = 0;

        disablePmp(touching_pmp);
        disablePmp(touched_pmp);

        attempt++;
        if (attempt == 3) {
          attempt = 0;
          printMessage(0,"Third attempt. DoubleTouch failed!\n");
          step = 5;
          return true;
        }
        printMessage(0,"Trying Again.. Attempt #%i\n", attempt);
        Time::delay(2);
        locateContact();
      }
      else unsuccess++;
    }
    else unsuccess = 0;
    return false;
  }
  else if (step == 5 || step == 55) {
    return true;
  }
  else {
    Vector t(3,0.0);
    Vector x,o;

    if (Bottle *v=targetOpt2.find("center").asList()) {
    
      t.resize(v->size());
      for (size_t i=0; i<t.length(); i++)
        t[i]=v->get(i).asDouble();
    }

    if (step == 22)      touched_pmp  -> getTool(x,o);
    else if (step == 33 || step == 6 || step == 66) {
      touching_pmp -> getTool(x,o);
    }
    d1 = norm(t-x);

    if (step == 22)                    touched_arm  -> getPose(x,o);
    else if (step == 33 || step == 6)  touched_pmp  -> getTool(x,o);
    d2 = norm(t-x);

    double diff1 = fabs(d1-d1_old);
    double diff2 = fabs(d2-d2_old);
    printMessage(1,"step=%i,\t|d1-d1_old|=%g [m],\t|d2-d2_old|=%g[m],\tstab=%i\n",step,diff1,diff2,stab);

    if ((diff1<1e-3) && (diff2<1e-3)) stab++;
    else                              stab = 0;

    d1_old = d1;
    d2_old = d2;

    if (stab == 30) {
      stab = 0;
      d1 = 0; d1_old = 0;
      d2 = 0; d2_old = 0;
      return true;
    }
  }
  
  return false;
}

void doubleTouchThread::goToTaxel() {
  // After the accomodation, the contralateral arm is free to touch the taxel.
  // We need to set an obstacle and the target (ie the taxel itself).
  // The iCub's head will move accordingly

  Vector p(3,0.0), o(4,0.0);

  // Set the target for the touching arm (i.e. the touched taxel)
    touched_pmp -> getTool(p,o);
    configureTarget(targetOpt2,p);
    touching_pmp -> addItem(targetOpt2,trgt2);

  // Set the obstacle for the touching arm (i.e. the touched_arm end-eff)
    touched_arm -> getPose(p,o);
    configureObstacle(obsOpt2,p);
    touching_pmp -> addItem(obsOpt2,obst2);

    printMessage(2,"GOTOTAXEL ENDED.\n");
}

void doubleTouchThread::configureObstacle(Property &prop,Vector p) {
  Value centerOb; centerOb.fromString(("("+string(p.toString().c_str())+")").c_str());
  Value radiusOb; radiusOb.fromString("(0.1 0.1 0.1)");

  prop.put("type","obstacle_gaussian");
  prop.put("active","on");
  prop.put("G",G);
  prop.put("name","obstacle");
  prop.put("center",centerOb);
  prop.put("radius",radiusOb);
  prop.put("cut_tails","on");
}

void doubleTouchThread::configureTarget(Property &prop,Vector p) {
  Value centerTg; centerTg.fromString(("("+string(p.toString().c_str())+")").c_str());
  Value radiusTg; radiusTg.fromString("(0.01 0.01 0.01)");

  prop.put("type","target_msd");
  prop.put("active","on");
  prop.put("K",K);
  prop.put("D",D);
  prop.put("name","target");
  prop.put("center",centerTg);
  prop.put("radius",radiusTg);
}

void doubleTouchThread::normalAccomodation() {
  if (robot == "icub" || robot == "icubSim") cout << "NORMALACCOMODATION!!!\n\n";
  // The ipsilateral arm should rotate in order to show the taxel to the
  // contralateral approaching arm. Thus, the taxel should be normal to the line
  // joining the taxel itself and the contralateral end-effector
  Vector join(3,0.0);   // Vector related to the joining line
  Vector xee(3,0.0);    // End-Effector Position
  Vector oee(4,0.0);    // End-Effector Orientation

  touching_arm -> getPose(xee, oee);
  // Contact position + joiningLine = endEffPose
  join =  xee - cntctPosWRF;

  // AttachTool:
    // p -> position of the taxel wrt the end eff (ie cnctPosEE, already found)
    // o -> rtmatrix from WRF to i-th link (i.e. Twl) * rotation in order to
    //      point z-axis parallel to cntctNormDir - a bit more tricky :(

  // FIRST: create a reference frame solidal with the taxel:
    //   x -> parallel to the x-axis
    //   z -> parallel to cntctNormDir (it's already an unitary vector)
    //   y -> ortonormal to the two

    Matrix R(4,4);
    Vector o(4,0.0);
    Vector x(3,0.0), z(3,0.0), y(3,0.0);
    z = cntctNormDir;
    // z[0] * x + z[1] * y + z[2] * z = 0
    // x = - (z[1] / z[0]) * y - (z[2] / z[0]) * z
    x[0] = -z[2]/z[0]; x[2] = 1;
    y = -1*(cross(x,z));
    // Let's make them unitary vectors:
    x = x / norm(x);
    y = y / norm(y);
    z = z / norm(z);

    R.zero();
    R(3,3) = 1;
    R.setSubcol(x,0,0);
    R.setSubcol(y,0,1);
    R.setSubcol(z,0,2);
    R.setSubcol(cntctPosWRF,0,3);

    o = dcm2axis(R);

    // Now R is the rototranslation (i.e. change of reference frame) from WRF
    // to the taxel. Let's switch it in end-eff reference coordinates:

    R = SE3inv(Twe) * R;

    printMessage(2,"Rpre:\n%s\n",R.toString().c_str());

    o = dcm2axis(R);
    Vector toolp = R.subcol(0,3,3);

  // SECOND: disable joints from taxel to end-eff + attachToolFrame
    toggleJoints("disable_wrist");

    touched_pmp -> attachToolFrame(toolp,o);

  // THIRD: setPointOrientation should rotate the tool properly
    Vector odot(4,0.0);
    o.resize(4,0.0);
    x.resize(3,0.0); z.resize(3,0.0); y.resize(3,0.0);
    z = join;
    // z[0] * x + z[1] * y + z[2] * z = 0
    // x = - (z[1] / z[0]) * y - (z[2] / z[0]) * z
    // (y,z) = (0,1) -> V2 : (-z[2]/z[0]; 0; 1)
    // (y,z) = (1,0) -> V3 : (-z[1]/z[0]; 1; 0)
    x[0] = -z[2]/z[0]; x[2] = 1;
    y = -1*(cross(x,z));
    // Let's make them unitary vectors:
    x = x / norm(x);
    y = y / norm(y);
    z = z / norm(z);

    R.zero();
    R(3,3) = 1;
    R.setSubcol(x,0,0);
    R.setSubcol(y,0,1);
    R.setSubcol(z,0,2);
    R.setSubcol(cntctPosWRF,0,3);

    o = dcm2axis(R);    // from rotation matrix to axis/angle notation

    printMessage(2,"R:\n%s\n",R.toString().c_str());
    printMessage(2,"o:  %s\n",o.toString().c_str());

    Vector xdot(3,0.0);

    touched_pmp -> setPointStateToTool();
    touched_pmp -> setPointOrientation(o,odot);

  // FOURTH: targetopt1 -> the final position of the tool will be in the direction of join
    
    Vector t(3,0.0);
    
    // t = (norm(x)>0.1?cntctPosWRF+0.1*z:cntctPosWRF+0.05*z);
    // if (step == 22) t = (norm(x)>0.2?cntctPosWRF+0.2*z:cntctPosWRF+0.1*z);
    t = cntctPosWRF+0.45*join;
    if (step == 22) t = cntctPosWRF+0.55*join;

    configureTarget(targetOpt1,t);
    touched_pmp -> addItem(targetOpt1,trgt1);
}

void doubleTouchThread::drawTool() {

  Vector p(3,0.0), o(4,0.0), o_euler(3,0.0);
  Matrix R(3,3);
  string t = "tool";

  yarp::os::Bottle &obj = iCubGuiObjs -> prepare();

  touched_pmp -> getTool(p,o);
  R = axis2dcm(o);
  o_euler = CTRL_RAD2DEG * dcm2euler(R);

  if (step == 6 || step == 5) {
    obj.addString("delete");
    obj.addString(t.c_str());
  }
  if (step == 2 || step == 22 || step == 3 || step == 33 || step == 55 || step == 66) {

    obj.clear();
    obj.addString("object"); // command to add/update an object
    obj.addString(t.c_str());
    // object dimensions in millimiters 
    // (it will be displayed as an ellipsoid with the tag "my_object_name")
    obj.addDouble(15);
    obj.addDouble(15);
    obj.addDouble(40);
    // object position in millimiters
    // reference frame: X=fwd, Y=left, Z=up
    obj.addDouble(p[0]*1000);
    obj.addDouble(p[1]*1000);
    obj.addDouble(p[2]*1000);
    // object orientation (roll, pitch, yaw) in degrees
    obj.addDouble(o_euler[2]);
    obj.addDouble(o_euler[1]);
    obj.addDouble(o_euler[0]);
    // object color (0-255)
    obj.addInt(150);
    obj.addInt(50);
    obj.addInt(255);
    // transparency (0.0=invisible 1.0=solid)
    obj.addDouble(0.7);

    // Move the head toward the affected taxel
    if (step == 55 || step == 66) {
      Vector ang(3);
      ang[0]=+0.0;                   // azimuth-component [deg]
      ang[1]=+0.0;                   // elevation-component [deg]
      ang[2]=+0.0;                   // vergence-component [deg]

      // igaze -> lookAtAbsAngles(ang);    // move the gaze
    }
    // else igaze -> lookAtFixationPoint(p);    // move the gaze
  }
  iCubGuiObjs -> write();
}

void doubleTouchThread::toggleJoints(string toggle) {

  if (toggle == "disable_torso" || toggle == "enable_torso") {
    // get the torso dofs
    Vector newDof_l, curDof_l;
    Vector newDof_r, curDof_r;
    left_arm  -> getDOF(curDof_l);
    right_arm -> getDOF(curDof_r);
    
    newDof_l=curDof_l;
    newDof_r=curDof_r;

    if (toggle == "disable_torso") {
      printMessage(2,"Disabling torso..\n");
      newDof_l[0] = 0;  newDof_l[1] = 0;  newDof_l[2] = 0;
      newDof_r[0] = 0;  newDof_r[1] = 0;  newDof_r[2] = 0;
    }
    else if (toggle == "enable_torso") {
      printMessage(2,"Endabling torso..\n");
      newDof_l[0] = 1;  newDof_l[1] = 1;  newDof_l[2] = 1;
      newDof_r[0] = 1;  newDof_r[1] = 1;  newDof_r[2] = 1;
    }

    // send the request for dofs reconfiguration
    left_arm  -> setDOF(newDof_l,curDof_l);
    right_arm -> setDOF(newDof_r,curDof_r);
  }
  else if (toggle == "disable_wrist" || toggle == "enable_wrist") { 
    Vector newDof, curDof;

    if      (cntctArm == "right_arm") right_arm -> getDOF(curDof);
    else if (cntctArm == "left_arm")  left_arm  -> getDOF(curDof);

    newDof = curDof;

    if (toggle == "disable_wrist") {
      printMessage(2,"Disabling wrist..\n");
      newDof[7]=0;  newDof[8]=0;  newDof[9]=0;
    }
    else if (toggle == "enable_wrist") {
      printMessage(2,"Enabling wrist..\n");
      newDof[7]=1;  newDof[8]=1;  newDof[9]=1;
    }

    // send the request for dofs reconfiguration
    if      (cntctArm == "right_arm") right_arm -> setDOF(newDof,curDof);
    else if (cntctArm == "left_arm")  left_arm  -> setDOF(newDof,curDof);
  }
}

void doubleTouchThread::goToRest() {
  goToRest("both_arms");
}

// the usage of the input string is deprecated (actually, was never implemented, 
// but the architecture permains in order to be able to fit further extensions)
void doubleTouchThread::goToRest(string s) {   

  Vector xdr(3,0.0), xdl(3,0.0);
  Vector odr(4,0.0), odl(4,0.0);
  // RIGHT ARM POSITIONS:
  xdr[0] = -0.3157; xdr[1] =  0.1; xdr[2] =  0.0422;
  // x axis forward, y axis rightward, z axis downward
  odr[0] = 0.0; odr[1] = 1.0; odr[2] = 0.0; odr[3] = M_PI;
  // LEFT ARM POSITIONS:
  xdl[0] = -0.3157; xdl[1] = -0.1; xdl[2] =  0.0422;
  // x axis forward, y axis leftward, z axis upward
  odl[0] = 0.0; odl[1] = 0.0; odl[2] = 1.0; odl[3] = M_PI;

  if (1) { // There's no reason for it to run whenever goToRest is called
    Vector ang(3);
    ang[0]=+0.0;                   // azimuth-component   [deg]
    ang[1]=+0.0;                   // elevation-component [deg]
    ang[2]=+0.0;                   // vergence-component  [deg]

    // igaze -> lookAtAbsAngles(ang);    // move the gaze
  }

  if (step == 5 || step == 55) {
    Vector p(3,0.0), o(4,0.0), odot(4,0.0);
    Value centerTg, radiusTg;

    // 
      if      (cntctArm == "right_arm") configureTarget(targetOpt2,xdl);
      else if (cntctArm == "left_arm")  configureTarget(targetOpt2,xdr);
      touching_pmp -> addItem(targetOpt2,trgt2);

      touched_arm -> getPose(p,o);
      configureObstacle(obsOpt2,p);
      touching_pmp -> addItem(obsOpt2,obst2);

      touching_pmp -> setPointStateToTool();
      if      (cntctArm == "right_arm") touching_pmp -> setPointOrientation(odl,odot);
      else if (cntctArm == "left_arm")  touching_pmp -> setPointOrientation(odr,odot);
      touching_pmp -> enableField();
      touching_pmp -> enableControl();

      Time::delay(2);

    if (step == 5) {
      toggleJoints("enable_wrist");
      if (cntctArm == "right_arm")     configureTarget(targetOpt1,xdr);
      else if (cntctArm == "left_arm") configureTarget(targetOpt1,xdl);
      touched_pmp -> addItem(targetOpt1,trgt1);

      touching_arm -> getPose(p,o);
      configureObstacle(obsOpt1,p);
      touched_pmp -> addItem(obsOpt1,obst1);

      touched_pmp -> setPointStateToTool();
      if (cntctArm == "right_arm")     touched_pmp -> setPointOrientation(odr,odot);
      else if (cntctArm == "left_arm") touched_pmp -> setPointOrientation(odl,odot);
      touched_pmp -> enableField();
      touched_pmp -> enableControl();
      Time::delay(0.5);
    }
  }
  else {
    right_arm -> goToPoseSync(xdr,odr); //let's use the blocking call
    Time::delay(1.5);
    left_arm -> goToPoseSync(xdl,odl); //let's use the blocking call
    Time::delay(0.5);
  }

  while(!checkMotion()) {
    Time::delay(1);
  }
}

void doubleTouchThread::detectContact(skinContactList *_sCL) {
  // Reset variables:
  cntctPosLink.resize(3,0.0);
  cntctPosWRF.resize(3,0.0);
  cntctNormDir.resize(3,0.0);
  cntctPressure = -1;
  cntctLinkNum = -1;
  cntctArm = "";

  // Search for a suitable contact:
  for(skinContactList::iterator it=_sCL->begin(); it!=_sCL->end(); it++) {
    int skinPart = it -> getSkinPart(); // Retrieve the skinPart of the skinContact

    if(skinPart == fal || skinPart == ual || skinPart == far || skinPart == uar) {
      // Convert the skincontact to a vector:
      Vector cnt = it -> toVector();
      // Store the skincontact for eventual future use
      cntctSkin.fromVector(cnt);
      // Get the position of the contact:
      cntctPosLink = it -> getGeoCenter();
      // Retrieve the link number in order to find the origin of the reference frame
      // NB 3 is added since limbL/R doesn't deal with the torso
      cntctLinkNum = it -> getLinkNumber() + 3;
      // Retrieve the normal direction of the contact
      cntctNormDir = it -> getNormalDir();
      // Retrieve the pressure of the contact
      cntctPressure = it -> getPressure();
      
      if(skinPart == fal || skinPart == ual) {
        cntctArm = "left_arm";
        touched_pmp  = pmp_left;  touching_pmp = pmp_right;
        touched_arm  = left_arm;  touching_arm = right_arm;
      }
      else if(skinPart == far || skinPart == uar) {
        cntctArm = "right_arm";
        touched_pmp  = pmp_right; touching_pmp = pmp_left;
        touched_arm  = right_arm; touching_arm = left_arm;
      }

      locateContact();             // LOCATE THE CONTACT IN WRF
      break;
    }
  }
}

void doubleTouchThread::locateContact() {

  // The position of the contact will be translated both in World Reference
  // Frame's coordinates, and in end-effector coordinates. This in order to
  // cope with the attachTool() function in pmp_client
  Vector result(4,0.0);
  Twl.zero();
  Twe.zero();
  Tel.zero();
  iDynLimb *lmb;

  if ( cntctArm == "right_arm" ) {
    right_handler -> updateState();
    q   = right_handler -> getQ();
    lmb = right_handler -> getLimb();
  }
  else if ( cntctArm == "left_arm") {
    left_handler -> updateState();
    q   = left_handler -> getQ();
    lmb = left_handler -> getLimb();
  }

  // REMEMBER THAT iDyn SUCKS! setAng with blocked
  // joints is different from setAng with no joint blocked!
  // and no documentation is provided :O
  Twl = lmb -> getH(cntctLinkNum, true);
  Twe = lmb -> getH();
  cntctPosLink.push_back(1);
  result = Twl * cntctPosLink;
  result.pop_back();

  cntctPosWRF = result;

  /********************************************************************/
  // Now, let's find the taxel position wrt the end-effector. Quick reminder:
  // A = rt matrix from WRF to i-th link (ie Twl)
  // B = rt matrix from WRF to end-effector (ie Twe)
  // C = rt matrix from end-effector to i-th link (ie Tel)
  // B*C = A -> C?
  // B^-1 * B * C = B^-1 * A  ->  C = B^-1 * A -> Tel = Twe^-1 * Twl

  result.resize(4,0.0);
  Tel = SE3inv(Twe) * Twl;
  result = Tel * cntctPosLink;
  result.pop_back();

  cntctPosEE = result;

  cntctPosLink.pop_back();
 
  printMessage(2,"Twl:\n%s\n",Twl.toString().c_str());
  printMessage(2,"Twe:\n%s\n",Twe.toString().c_str());
  printMessage(2,"Tel:\n%s\n",Tel.toString().c_str());

  printMessage(1,"q: %s\n",q.toString().c_str());
  printMessage(1,"linknumber: %d\n",cntctLinkNum);
  printMessage(1,"PosLink: %s\n",cntctPosLink.toString().c_str());
  printMessage(1,"PosEE:   %s\n",cntctPosEE.toString().c_str());
}

void doubleTouchThread::closePort(Contactable *_port) {
  if (_port)
  {
      _port -> interrupt();
      _port -> close();

      delete _port;
      _port = 0;
  }
}

int doubleTouchThread::printMessage(const int level, const char *format, ...) const {

  if (verbosity>=level)  {

    fprintf(stdout,"*** %s: ",name.c_str());

    va_list ap;
    va_start(ap,format);
    int ret=vfprintf(stdout,format,ap);
    va_end(ap);
    
    return ret;
  }
  else
      return -1;
}

void doubleTouchThread::threadRelease() {

  yarp::os::Bottle &obj = iCubGuiObjs -> prepare();
  obj.addString("delete");
  obj.addString("tool");  
  iCubGuiObjs -> write();

  cout<<"dblTchThrd: returning to position mode..\n";
    
    for (int i = 0; i < 5; ++i) {
      right_handler -> getArm() -> ictrl -> setPositionMode(i);
      left_handler  -> getArm() -> ictrl -> setPositionMode(i);
    }

  cout<<"dblTchThrd: closing ports..\n";

    closePort(cntRdr);
    printMessage(1,"skin port successfully closed!\n");
    closePort(iCubGuiObjs);
    printMessage(1,"icubGuiobjs port successfully closed!\n");

  cout<<"dblTchThrd: closing controllers..\n";

    if (torso_handler) {
      delete torso_handler;
      torso_handler = 0;
    }
    if (left_handler) {
      delete left_handler;
      left_handler = 0;
    }
    if (right_handler) {
      delete right_handler;
      right_handler = 0;
    }

    // igaze -> stopControl();
    // client_gaze.close();

    disablePmp(pmp_right);
    pmp_right -> close();
    disablePmp(pmp_left);
    pmp_left  -> close();
}

void doubleTouchThread::enablePmp(PmpClient *p) {
  p -> setPointStateToTool();
  p -> enableField();
  p -> enableControl();
}

void doubleTouchThread::disablePmp(PmpClient *p) {
  p -> disableField();
  p -> disableControl();
  p -> removeToolFrame();
  p -> clearItems();
}

// empty line to make gcc happy