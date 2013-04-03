#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynTransform.h>
#include <iCub/pmp/pmp_client.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>

#include "utils.h"

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::skinDynLib;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace iCub::pmp;

using namespace std;

/*****************************************************************************/
// This thread detects a touched taxel on the skin (through readings from the
// skinContactList port), and projects it in a 3D position in World Reference
// Frame. Then, it moves the "controlateral" limb toward the affected taxel.
// 
// Three conditions: 
// 
// 1. NORMALITY: the ipsilateral hand should accomodate the movement
// by automatically rotate itself in order to facilitate the contralateral arm
// 
// 2. REACHABILITY/MANIPOLABILTY: the proper orientation must be found among
// those which maximize the reachability/manipolability task
// 
// 3. PMP: to avoid unexpected contacts between arms, the last condition is to
// think the arms themselves as mutual obstacles.

class doubleTouchThread: public RateThread {
protected:
  /***************************************************************************/
  // EXTERNAL VARIABLES: change them from command line or through .ini file
  // Flag that manages verbosity (v=1 -> more text printed out):
  int verbosity;
  // Flag that manages motor modality (m=0 -> the thread will only locate a
  // contact and the robot will never move):
  bool m;
  // Name of the module (to change port names accordingly):
  string name;
  // Name of the robot (to address the module toward icub or icubSim):
  string robot;
  // Parameters for the PMP
  double K;
  double D;
  double G;
  /***************************************************************************/
  // CONTACT-RELATED VARIABLES:
  Vector cntctPosLink;     // Position in i-th link RF
  Vector cntctPosWRF;      // Position in WRF
  Vector cntctPosEE;       // Position in end-eff RF
  Vector cntctNormDir;     // Normal Direction
  int cntctLinkNum;        // Link number
  double cntctPressure;    // Pressure
  string cntctArm;         // Affected Arm
  skinContact cntctSkin;   // SkinContact
  /***************************************************************************/
  // INTERNAL VARIABLES:
  // Flag to know in which step the thread is
  int step;
  // Flag to know if the robot is moving or not:
  bool mov;
  // Flag to know if the touch was successful:
  int unsuccess;
  int attempt;
  // Flag to know if the pmp is gone in a stable state:
  int stab;

  double d1, d2;
  double d1_old, d2_old;

  // Upper arm and forearm indexes for the SkinPart enum
  // (eventually, there will be easy to change the skinparts to monitor):
  int ual, fal, uar, far;

  // Ports:
  BufferedPort<iCub::skinDynLib::skinContactList> *cntRdr;
  BufferedPort<Bottle> *iCubGuiObjs;

  // Joint variables for right and left arm:
  Vector q;

  // Rototranslation Matrices:
  Matrix Twl; // From WRF to i-th link
  Matrix Twe; // From WRF to end-effector
  Matrix Tel; // From end-effector to i-th link

  // Resting positions for right and left arm
  // restPosition *strtPos;

  // Polydriver & stuff for cartesian controller:
  PolyDriver          client_gaze;
  ICartesianControl  *left_arm;
  ICartesianControl  *right_arm;
  ICartesianControl  *touched_arm;
  ICartesianControl  *touching_arm;
  IGazeControl       *igaze;

  // armDynHandler has a proper limbHandler as data member. It's only a wrapper that includes more
  // data members and methods (basically, the dynamic part in order to process FT data).
  // Both right and left handler has a pointer to torso handler in order to retrieve information
  // from torso.
  armDynHandler *left_handler;
  armDynHandler *right_handler;
  limbHandler   *torso_handler;

  // PMP stuff:
  PmpClient *pmp_left;
  PmpClient *pmp_right;
  PmpClient *touched_pmp;
  PmpClient *touching_pmp;

  // items with "1" are related to the ipsilateral arm,
  // otherwise they're related to the contralateral
  Property targetOpt1;  // Same position, but rotate the taxel
                        // toward the contralateral end-eff
  Property targetOpt2;  // The taxel on the ipsilateral arm
  Property obsOpt1;     // contralateral end-eff
  Property obsOpt2;     // ipsilateral end-eff

  int trgt1, trgt2;
  int obst1, obst2;

  void normalAccomodation();

  void drawTool();

  void closePort(Contactable *_port);

  void toggleJoints(string toggle);

  int printMessage(const int level, const char *format, ...) const;

  // check for motion
  bool checkMotion();
  // check for unexpected contacts 
  // trough FT sensors (not implemented yet)
  bool checkSafety();
  // handle issues related to safety:
  void handleIssue();

  void  enablePmp(PmpClient *p);
  void disablePmp(PmpClient *p);


  // moves arms to starting position:
  void goToRest();
  void goToRest(string s);    // either "both_arms", "right_arm",or "left_arm"
  // use the contralateral hand to touch the affected taxel
  void goToTaxel();
  // read the contact from either /skinManager/skin_events:o or
  // /wholeBodyDynamics/contacts:o , and retrieve the coordinates
  // of the contacts in i-th link Reference Frame:
  void detectContact(skinContactList *_sCL);
  // translate the contact into WRF for both right and left arm:
  void locateContact();

  void configureTarget(Property &prop,Vector p);
  void configureObstacle(Property &prop,Vector p);

public:
  // CONSTRUCTOR
  doubleTouchThread(int _cd_rate, const string &_name, const string &_robot, int _v, bool _m, int _K, int _D, int _G);
  // INIT
  virtual bool threadInit();
  // RUN
  virtual void run();
  // RELEASE
  virtual void threadRelease();
};

// PERSONAL NOTES (remember to remove them sooner or later!)
// iCub/main/src/tools/skinManagerGui
// iCub/main/src/modules/skinManager
// iCub/main/src/libraries/skinDynLib
//
// A skinContactList is represented as a list of lists
// where each list is a skinContact
// basically, it's a vector of skinContact
//
// A skinContact is a list of 8 elements that are:
// - a list of 4 int, i.e. contactId, bodyPart, linkNumber, skinPart
// - a list of 3 double, i.e. the CoP
// - a list of 3 double, i.e. the force
// - a list of 3 double, i.e. the moment
// - a list of 3 double, i.e. the geometric center
// - a list of 3 double, i.e. the normal direction
// - a list of N int, i.e. the active taxel ids
// - a double, i.e. the pressure
// 
// ((48725 4 4 5) (-0.017 0.062 -0.036) (0.476424 0.109944 0.611614)
// (0.0 0.0 0.0) (-0.017 0.062 -0.036) (-0.585 -0.135 -0.751) (134) 16.288001)

