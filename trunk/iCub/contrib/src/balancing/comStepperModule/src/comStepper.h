#ifndef COM_BALANCING
#define COM_BALANCING

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <iCub/iKin/iKinFwd.h>
//#include <iCub/iKin/iKinBody.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>


#include <iostream>
#include <iomanip>
#include <string.h>
#include <queue>
#include <list>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iKin;
using namespace std;


class comStepperThread: public RateThread
{
public:

    //****************** POLYDRIVERS AND INTERFACES ******************
    PolyDriver *dd_torso;
    PolyDriver *dd_rightLeg;
    PolyDriver *dd_leftLeg;
    
    IPositionControl *Ipos_TO;
    IPositionControl *Ipos_RL;
    IPositionControl *Ipos_LL;
    IControlMode     *Ictrl_TO;
    IControlMode     *Ictrl_LL;
    IControlMode     *Ictrl_RL;
    IVelocityControl *Ivel_TO;
    IVelocityControl *Ivel_LL;
    IVelocityControl *Ivel_RL;
    IEncoders        *Ienc_TO;
    IEncoders        *Ienc_LL;
    IEncoders        *Ienc_RL;
    IPidControl      *Ipid_TO;
    IPidControl      *Ipid_LL;
    IPidControl      *Ipid_RL;
    //****************************************************************

    int njHD;   int njTO;   int njRL;  int njLL; int njRA;  int njLA;

    Vector command;
    Vector command_RL;
    Vector command_LL;
    Vector zmp_xy;
    Vector zmp_xy_vel;
    Vector zmp_des;
    iCub::ctrl::AWLinEstimator      *zmp_xy_vel_estimator;

    //matrices involved in computing Jca
    Matrix Tba, Tbc, Tab, Tac;
    Matrix Rba, Rbc, Rac;
    Matrix pba, pbc, pac;
    Matrix pab, pcb, rca_b;
    Matrix Spac, Srca, Rrab;
    Matrix Jba, Jbc, Jac;
    
    //matrices involved in the computation of eac
    Matrix  pac_d,  Rac_d;
    Matrix dpac_d, dRac_d;
    
    Matrix nd,         sd,     ad,     ne,     se, ae;
    Matrix nd_hat, sd_hat, ad_hat, ne_hat, se_hat, ae_hat;
    
    Matrix ep, eo;
    Matrix L;
    
    
    //Matrices involved in the computation of the center of mass projection
    Matrix Jb_p, p_b, Jb_com           ;   //position of the center of mass and its jacobian
    Matrix PI                          ;   //projection matrix
    Matrix pi_a, pi_b, pi_a_d          ;   //projection of p_b and disred value expressed in r.f. 'a'
    Matrix Spab, SRab_pb, Srba_pi      ;   //varius matrices used in the computations
    Matrix Jab                         ;   //Jabian from of 'b' with respect to 'a'
    Matrix Jab_t, Jba_t                ;   //translated version of Jab and Jba
    Matrix Jpi_b, Jpi_b_com            ;   //jacobian of the projection on 'b'
    Matrix Jpi_a, Jpi_a_com            ;   //jacobian of the projection on 'a'
      
    
private:
    string robot_name;
    string local_name;
    string wbsName;
    bool springs;
    bool torso;
    bool verbose;

    //PORTS
    //input ports
    BufferedPort<Vector> *EEWRightLeg;        //EE Wrench Right Leg
    BufferedPort<Vector> *EEWLeftLeg;         //EE Wrench Left Leg
    BufferedPort<Vector> *EEWRightAnkle;      //EE Wrench Right Ankle Sensor
    BufferedPort<Vector> *EEWLeftAnkle;       //EE Wrench Left Ankle Sensor

    BufferedPort<Vector> *objPort;
    BufferedPort<Vector> *objPort2;
    BufferedPort<Matrix> *EEPRightLeg;        //EE Pose Right Leg
    BufferedPort<Matrix> *EEPLeftLeg;         //EE Pose Left Leg
    BufferedPort<Vector> *desired_zmp;        //varying set point.
    BufferedPort<Matrix> *COM_Jacob_port;
    BufferedPort<Vector> *COM_Posit_port;
    BufferedPort<Vector> *ankle_angle;        //Commanded ankle angle
    BufferedPort<Vector> *COM_ref_port;       //COM_ref 
    BufferedPort<Vector> *port_ft_foot_left;  //Left foot f/t sensor reading
    BufferedPort<Vector> *port_ft_foot_right; //Right foot f/t sensor reading

    double *angle;

    Matrix rot_f;

    //controller gains 
    double Kp_zmp_x, Kp_zmp_y;
    double Kd_zmp_x, Kd_zmp_y;
    double Kp_theta, Kp_phi;
    double Kd_theta, Kd_phi;

    //com ports
    string comPosPortString, comJacPortString;
    
    //compute ZMP variables
    double xLL, yLL, xDS, yDS, yRL, xRL;
    Vector *F_ext_RL;
    Vector *F_ext_LL;

    //For plots only
    Vector *F_ext_rf;
    Vector *F_ext_lf;

    //Sensors not available in the simulator
    bool Opt_nosens;
    
    //display
    bool Opt_display;

    //Left and Right leg pose.
    Vector PoseLeftLeg;
    Vector PoseRightLeg;

    // To read from Inertial sensor. 
    Vector *inrtl_reading;
    // To read HEAD, RIGHT AND LEFT poses
    Vector *head_pose;

    //Right and left leg and torso encoders
    Vector encs_r, encs_r_rad;
    Vector encs_l, encs_l_rad;

    Matrix Jac_FR;             //Jacobian matrix for right FOOT from ROOT.
    iCubLeg *Right_Leg;
    iCubLeg *Left_Leg;
    iCubInertialSensor *Inertia_Sens;

    Matrix Hright;
    Matrix Hleft;

    bool Opt_ankles_sens;
    
    Stamp zmp_time;

    Vector Offset_Lfoot, Offset_Rfoot;
    

public:
    comStepperThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_lefLeg,\
                       string _robot_name, string _local_name, string _wbs_name, bool display, bool noSens, \
                       bool ankles_sens, bool springs, bool torso, bool verbose, double Kp_zmp_x, double Kd_zmp_x,\
                       double Kp_zmp_y, double Kd_zmp_y, double Kp_theta, double Kd_theta, double Kp_phi, double Kd_phi, string comPosPortName, string comJacPortName);
    void setRefAcc(IEncoders* iencs, IVelocityControl* ivel);
    bool threadInit();
    void run();
    bool onStop();
    void threadRelease();
    void computeZMP(Vector* zmp_xy, Vector *F_ext_LL, Vector *F_ext_RL, Vector encs_l, Vector encs_r);
    double separation(Vector encs_r, Vector encs_l);
    void closePort(Contactable *_port);
    void velTranslator(Matrix &out, Matrix p);
    void rotTranslator(Matrix &out, Matrix R);
    void hat(Matrix &p_hat, Matrix p);
    Vector computeDeltaError(Vector q0R, Vector q0L, Vector q1R, Vector q1L);
    Vector computeDeltaProjB(Vector q0R, Vector q0L, Vector q1R, Vector q1L);
    Vector computeDeltaProjA(Vector q0R, Vector q0L, Vector q1R, Vector q1L);
    void computeDeltaProjReal(Vector dqR, Vector dqL, Vector dqT, Vector &delta_b, Vector &delta_pi_a, Vector &delta_pi_b);
};

#endif
