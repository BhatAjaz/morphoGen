#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Stamp.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/iKin/iKinFwd.h>
#include <math.h>
#include <stdio.h>


#include <iostream>
#include <iomanip>
#include <fstream>
#include <string.h>
#include "comStepper.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

#define FOR_PLOTS_ONLY
#define TH0_LEGS
// #define COMPUTE_FINITE_DIFF

comStepperThread::comStepperThread(int _rate, PolyDriver *_ddTor, PolyDriver *_dd_rightLeg, PolyDriver *_dd_leftLeg, string _robot_name, string _local_name, string _wbs_name, bool display, bool noSens, bool ankles_sens, bool _springs, bool _torso, bool _verbose, double Kp_zmp_x, double Kd_zmp_x, double Kp_zmp_y, double Kd_zmp_y, double Kp_theta, double Kd_theta, double Kp_phi, double Kd_phi, string comPosPort, string comJacPort) : RateThread(_rate), dd_torso(_ddTor), dd_rightLeg(_dd_rightLeg),dd_leftLeg(_dd_leftLeg), robot_name(_robot_name), local_name(_local_name), wbsName(_wbs_name), springs(_springs), torso(_torso), verbose(_verbose), Kp_zmp_x(Kp_zmp_x), Kd_zmp_x(Kd_zmp_x), Kp_zmp_y(Kp_zmp_y), Kd_zmp_y(Kd_zmp_y), Kp_theta(Kp_theta), Kd_theta(Kd_theta), Kp_phi(Kp_phi), Kd_phi(Kd_phi), comPosPortString(comPosPort), comJacPortString(comJacPort)
{
    //------------------ INTERFACE INITIALIZATION ----------------------------------
    printf("rate: %d\n",_rate);
    Ipos_TO     = 0;
    Ipos_RL     = 0;
    Ipos_LL     = 0;
    Ictrl_TO    = 0;
    Ictrl_LL    = 0;
    Ictrl_RL    = 0;
    Ivel_TO     = 0;
    Ivel_LL     = 0;
    Ivel_RL     = 0;
    Ienc_TO     = 0;
    Ienc_LL     = 0;
    Ienc_RL     = 0;
    Ipid_TO     = 0;
    Ipid_LL     = 0;
    Ipid_RL     = 0;
    
    Opt_display = display;
    Opt_nosens = noSens;
    Opt_ankles_sens = ankles_sens;
    
    zmp_xy_vel_estimator = new iCub::ctrl::AWLinEstimator(1,0.0);
    
    //---------------------------- PORTS ---------------------------------------------
    if (!noSens)
    {
        EEWRightLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
        EEWRightLeg->open(string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/right_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/right_leg/endEffectorWrench:i").c_str(),"tcp",false);
        
        EEWLeftLeg = new BufferedPort<Vector>; //Creating, opening and connecting PORT
        EEWLeftLeg->open(string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/left_leg/endEffectorWrench:o").c_str(), string("/"+local_name+"/left_leg/endEffectorWrench:i").c_str(),"tcp",false);
        
        EEWRightAnkle = new BufferedPort<Vector>;
        EEWRightAnkle->open(string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/right_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/right_ankle/endEffectorWrench:i").c_str());
        
        EEWLeftAnkle = new BufferedPort<Vector>;
        EEWLeftAnkle->open(string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());
        Network::connect(string(wbsName+"/left_foot/endEffectorWrench:o").c_str(),string("/"+local_name+"/left_ankle/endEffectorWrench:i").c_str());
        
        objPort = new BufferedPort<Vector>;
        objPort->open(string("/"+local_name+"/DSPzmp:o").c_str());
        Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/myiCubGui/objects").c_str());
        
        objPort2 = new BufferedPort<Vector>;
        objPort2->open(string("/"+local_name+"/DSPzmp2iCubGui:o").c_str());
        
        //system output
        desired_zmp = new BufferedPort<Vector>;
        desired_zmp->open(string("/"+local_name+"/desired_zmp:o").c_str());
        
        //Connecting output of BalancerModule's zmp TO velocityObserver input port to get derivatives.
        Network::connect(string("/"+local_name+"/DSPzmp:o").c_str(),string("/zmpVel/pos:i").c_str());
        
        //COM Jacobian
        //COM_Jacob_port = new BufferedPort<Matrix>;
        //COM_Jacob_port->open(string("/"+local_name+"/COM_Jacob_port:i").c_str());
        //Network::connect(string(wbsName+"/com_jacobian:o").c_str(),string("/"+local_name+"/COM_Jacob_port:i").c_str());
        
        port_ft_foot_left = new BufferedPort<Vector>;
        port_ft_foot_left->open(string("/"+local_name+"/left_foot/FT:i").c_str());
        Network::connect(string("/"+robot_name+"/left_foot/analog:o").c_str(), string("/"+local_name+"/left_foot/FT:i").c_str(), "tcp", false);
        
        port_ft_foot_right = new BufferedPort<Vector>;
        port_ft_foot_right->open(string("/"+local_name+"/right_foot/FT:i").c_str());
        Network::connect(string("/"+robot_name+"/right_foot/analog:o").c_str(), string("/"+local_name+"/right_foot/FT:i").c_str(),"tcp",false);
    }

    COM_ref_port = new BufferedPort<Vector>;
    COM_ref_port->open(string("/"+local_name+"/com_ref:o").c_str());

    ankle_angle = new BufferedPort<Vector>;
    ankle_angle->open(string("/"+local_name+"/commanded_ankle_ang:o").c_str());
    
    if (!comPosPort.empty())
    {
        fprintf(stderr, "Opening the port the COM position input!\n");
        COM_Posit_port = new BufferedPort<Vector>;
        COM_Posit_port->open(comPosPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port the COM position input!\n");

    if (!comJacPort.empty())
    {
        fprintf(stderr, "Opening a port for the COM Jacobian input!\n");
        COM_Jacob_port = new BufferedPort<Matrix>;
        COM_Jacob_port->open(comJacPort.c_str());
    }
    else
        fprintf(stderr, "Skipping the port the COM Jacobian input!\n");
    
    
    Right_Leg    = new iCubLeg("right");
    Left_Leg     = new iCubLeg("left");
    Inertia_Sens = new  iCubInertialSensor("v2");
}

bool comStepperThread::threadInit()
{
    fprintf(stderr, "STEP1: initializing the device drivers \n");
    F_ext_LL = 0;
    F_ext_RL = 0;
    
    //POLIDRIVERS AND INTERFACES
    
    // Torso Interface
    dd_torso->view(Ienc_TO);
    dd_torso->view(Ictrl_TO);
    dd_torso->view(Ivel_TO);
    dd_torso->view(Ipid_TO);
    dd_torso->view(Ipos_TO);
    if((!dd_torso) || (!Ienc_TO) || (!Ictrl_TO) || (!Ivel_TO) || (!Ipid_TO) || (!Ipos_TO))
    {
        printf("ERROR acquiring torso interfaces\n");
        return false;
    }
    
    //Right leg interfaces
    dd_rightLeg->view(Ienc_RL);
    dd_rightLeg->view(Ictrl_RL);
    dd_rightLeg->view(Ivel_RL);
    dd_rightLeg->view(Ipid_RL);
    dd_rightLeg->view(Ipos_RL);
    if((!dd_rightLeg) || (!Ienc_RL) || (!Ictrl_RL) || (!Ivel_RL) || (!Ipid_RL) || (!Ipos_RL))
    {
        printf("ERROR acquiring right leg interfaces\n");
        return false;
    }
    
    
    //Left leg interfaces
    dd_leftLeg->view(Ienc_LL);
    dd_leftLeg->view(Ictrl_LL);
    dd_leftLeg->view(Ivel_LL);
    dd_leftLeg->view(Ipid_LL);
    dd_leftLeg->view(Ipid_LL);
    dd_leftLeg->view(Ipos_LL);
    if((!dd_leftLeg) || (!Ienc_LL) || (!Ictrl_LL) || (!Ivel_LL) || (!Ipid_LL) || (!Ipos_LL))
    {
        printf("ERROR acquiring left leg interfaces\n");
        return false;
    }
    
    fprintf(stderr, "STEP2: moving torso to a given configuration \n");
    //FOR Sending commands to the TORSO
    Ienc_TO->getAxes(&njTO);
    command.resize(njTO);
    command[0] = 0.0;
    command[1] = 0.0;
    command[2] = 0.0;
    
    
    //FOR sending commands to the legs
    Ienc_LL->getAxes(&njLL); //Legs have 6 joints from 0 to 5.
    command_LL.resize(njLL);
    command_LL = 0;
    command_LL[0] = 0.0;
    
    Ienc_RL->getAxes(&njRL);
    command_RL.resize(njRL);
    command_RL = 0;
    command_RL[0] = 1.0;
    
    //Setting Reference Accelerations
    setRefAcc(Ienc_TO, Ivel_TO);
    setRefAcc(Ienc_RL, Ivel_RL);
    setRefAcc(Ienc_LL, Ivel_LL);
    
    
    //INITIALIZING thread variables.
    rot_f = zeros(3,3);
    
    // rot_f(0,0) = rot_f(1,1) = -1;
    // rot_f(2,2) = 1;
    rot_f(1,1) = -1;
    rot_f(2,0) = rot_f(0,2) = 1;
    
    fprintf(stderr, "STEP3: inizializing the ZMP \n");
    //INITIALIZING zmp vector
    zmp_xy = zeros(2);
    
    //INITIALIZING zmpXd desired
    zmp_des.resize(2);
    zmp_des[0] = 0.08;
    zmp_des[1] = 0.0;
    
    zmp_xy_vel.resize(2);
    zmp_xy_vel[0] = 0.0;
    zmp_xy_vel[1] = 0.0;
    
    // #ifdef STIFF_JNTS
    //     Kp_zmp_x = 0.40; Kp_zmp_y = 0;
    //     Kd_zmp_x = 0.005; Kd_zmp_y = 0;
    
    // //  Kp_theta = 80.0;  Kp_phi   = 0.0;
    // //  Kd_theta =  1.8;  Kd_phi   = 0.0;
    
    //     Kp_theta = 0.0;  Kp_phi   = 0.0;
    //     Kd_theta = 0.0;  Kd_phi   = 0.0;
    // #else
    //     Kp_zmp_x = 0.30; Kp_zmp_y = 0;      //Good values with spings 0.45 is original value with 4 springs
    //     Kd_zmp_x = 0.08; Kd_zmp_y = 0;      //Good values with springs 0.018
    
    //     Kp_theta = 80.0;  Kp_phi   = 0.0;   //Good values with springs
    //     Kd_theta =  1.0;  Kd_phi   = 0.0;   //Good values with springs
    // #endif
    
    //FOR SAFETY
    if(Kd_zmp_x>0.025){
        Kd_zmp_x=0.025;
    }
    if(Kp_zmp_x>0.9){
        Kp_zmp_x=0.9;
    }
    if(Kp_theta>100.0){
        Kp_theta=100;
    }
    if(Kd_theta>2.5){
        Kd_theta=2.5;
    }
    
    encs_r.resize(njRL);      encs_l.resize(njLL);
    encs_r_rad.resize(njRL);  encs_l_rad.resize(njLL);  
    
    Jac_FR.resize(6,32);
    Jac_FR.zero();
    
    Hright.resize(4,4);
    Hright.zero();
    Hright(0,0) = Hright(1,2) = Hright(3,3) = 1;
    Hright(2,1) = -1; Hright(1,3) = 0.0681; Hright(2,3) = -0.1199;
    
    Hleft.resize(4,4);
    Hleft.zero();
    Hleft(0,0) = Hleft(1,2) = Hleft(3,3) = 1;
    Hleft(2,1) = -1; Hleft(1,3) = -0.0681; Hleft(2,3) = -0.1199;
    
    fprintf(stderr, "STEP4: inizializing the desired movements \n");
#ifdef FOR_PLOTS_ONLY
    if (!Opt_nosens)
    {
        Vector* tmp;
        tmp = port_ft_foot_right->read(true);
        Offset_Rfoot = *tmp;
        tmp =  port_ft_foot_left->read(true);
        Offset_Lfoot = *tmp;
    }
    else
    {
        Offset_Lfoot.resize(6);
        Offset_Lfoot.zero();
        Offset_Rfoot.resize(6);
        Offset_Rfoot.zero();
    }
#endif
    
    // Setting initial configuration of entire robot.
    Vector tmp2;
    tmp2.resize(njRL);
    for(int i=0; i<njRL;i++){
        tmp2[i] = 10.0;
    }
    
    Ipos_RL->setRefSpeeds(tmp2.data());
    Ipos_LL->setRefSpeeds(tmp2.data());
    
    if (!Opt_display)
    {
        bool ok;
        ok = Ipos_RL->positionMove(command_RL.data());
        if(ok)
            printf("Right Leg set to initial position...\n");
        else
            return false;
        
        ok = Ipos_LL->positionMove(command_LL.data());
        if(ok)
            printf("Left Leg set to initial position...\n");
        else
            return false;
        
        ok = Ipos_TO->positionMove(command.data());
        if(ok)
            printf("Torso set to initial position...\n");
        else
            return false;
    }
    
    //resizing matrices involved in computing Jca
    Tba.resize(4,4);  Tbc.resize(4,4);  Tab.resize(4,4);  Tac.resize(4,4);
    Rba.resize(3,3);  Rbc.resize(3,3);  Rac.resize(3,3);
    pba.resize(3,1);  pbc.resize(3,1);  pac.resize(3,1);
    pab.resize(3,1);  pcb.resize(3,1);  rca_b.resize(3,1);

    Jba.resize(6, njRL); Jbc.resize(6, njLL); Jac.resize(6, njLL+njRL);
    Spac.resize(6,6); Srca.resize(6,6); Rrab.resize(6,6);
    
    //resizing matrices involved in the computation of eac
     pac_d.resize(3, 1);   Rac_d.resize(3, 3);
    dpac_d.resize(3, 1);  dRac_d.resize(3, 1);
    
    nd.resize(3, 1);     sd.resize(3, 1);     ad.resize(3, 1);
    ne.resize(3, 1);     se.resize(3, 1);     ae.resize(3, 1);
    nd_hat.resize(3, 3); sd_hat.resize(3, 3); ad_hat.resize(3, 3);
    ne_hat.resize(3, 3); se_hat.resize(3, 3); ae_hat.resize(3, 3);
    
    ep.resize(3, 1); eo.resize(3, 1);
    L.resize(3, 3);
    
    //resizing matrices for the computation of the center of mass projection
    PI = eye(3)                ; PI(0,0) = 0                ; Jb_p.resize(3, Jba.cols());
    p_b.resize(3,1)            ; pi_a.resize(3,1)           ; pi_b.resize(3,1);
    Spab.resize(6,6)           ; SRab_pb.resize(6,6)        ; Srba_pi.resize(6,6);
    Jab.resize(6, Jba.cols())  ; Jab_t.resize(3, Jba.cols()); pi_a_d.resize(3,1);
    Jba_t.resize(3, Jba.cols()); Jpi_b.resize(3, Jba.cols()); Jpi_a.resize(3, Jba.cols());
    
    //resizing matrices given the dimensions of the com_jac read from port
    if (!comPosPortString.empty())
    {
        fprintf(stderr, "Waiting for incoming COM position from port %s...\n", comPosPortString.c_str());
        Vector comPosition = *COM_Posit_port->read();
    }
    
    if (!comJacPortString.empty())
    {
        fprintf(stderr, "Waiting for incoming COM jacobian from port %s...\n", comJacPortString.c_str());
        Jb_com = (*COM_Jacob_port->read());
        Jb_com = Jb_com.removeRows(3, 3);
        
        njHD = 3;
        njRA = (Jb_com.cols() - njTO - njRL -njLL -njHD)/2;
        njLA = (Jb_com.cols() - njTO - njRL -njLL -njHD)/2;
        Jpi_b_com.resize(3, Jb_com.cols());
        Jpi_a_com.resize(3, Jb_com.cols());
    }
    
    return true;
}

void comStepperThread::setRefAcc(IEncoders* iencs, IVelocityControl* ivel)
{
    Vector tmp;  int nj;
    iencs->getAxes(&nj);
    
    tmp.resize(nj);
    int i;
    for (i=0;i<=nj;i++){
        tmp[i]=800;
    }
    ivel->setRefAccelerations(tmp.data());
}

void comStepperThread::run()
{
    //updating Time Stamp
    static Stamp timeStamp;
    timeStamp.update();
    
    //***************************** Reading F/T measurements and encoders ****************
    
#ifdef FOR_PLOTS_ONLY
    
    Vector* F_ext_RLt;
    Vector* F_ext_LLt;
    if (!Opt_nosens)
    {
        F_ext_RLt = port_ft_foot_right->read(true);
        F_ext_LLt = port_ft_foot_left->read(true);
        
        if (F_ext_LL==0) F_ext_LL= new Vector(6,0.0);
        if (F_ext_RL==0) F_ext_RL = new Vector(6,0.0);
    }
    else
    {
        F_ext_LLt= new Vector(6);    F_ext_LLt->zero();
        F_ext_RLt= new Vector(6);    F_ext_RLt->zero();
        
        if (F_ext_LL==0)
        {
            F_ext_LL= new Vector(6);
            F_ext_LL->zero();
        }
        if (F_ext_RL==0)
        {
            F_ext_RL= new Vector(6);
            F_ext_RL->zero();
        }
    }
    
    //fprintf(stderr, "Vector is: %s", Offset_Lfoot.toString().c_str());
    *F_ext_LL = -1.0*((*F_ext_LLt) - (Offset_Lfoot));
    *F_ext_RL = -1.0*((*F_ext_RLt) - (Offset_Rfoot));
    
    //Transformation from ankle to F/T sensor
    Matrix foot_hn(4,4); foot_hn.zero();
    foot_hn(0,2)=1; foot_hn(0,3)=-7.75;
    foot_hn(1,1)=-1;
    foot_hn(2,0)=-1;
    foot_hn(3,3)=1;
    
    Vector tmp1, tmp2;
    tmp1 = (*F_ext_RL).subVector(0,2); tmp1.push_back(0.0); tmp1 = foot_hn * tmp1;
    tmp2 = (*F_ext_RL).subVector(3,5); tmp2.push_back(0.0); tmp2 = foot_hn * tmp2;
    
    for (int i=0; i<3; i++) (*F_ext_RL)[i] = tmp1[i]; 
    for (int i=3; i<6; i++) (*F_ext_RL)[i] = tmp2[i-3];
    
    tmp1 = (*F_ext_LL).subVector(0,2); tmp1.push_back(0.0); tmp1 = foot_hn * tmp1;
    tmp2 = (*F_ext_LL).subVector(3,5); tmp2.push_back(0.0); tmp2 = foot_hn * tmp2;
    
    for (int i=0; i<3; i++) (*F_ext_LL)[i] = tmp1[i];
    for (int i=3; i<6; i++) (*F_ext_LL)[i] = tmp2[i-3];
    
#else
    if(Opt_ankles_sens && !Opt_nosens)
    {
        F_ext_RL = EEWRightAnkle->read(true);
        F_ext_LL = EEWLeftAnkle->read(true);
        
    }
    else{
        if (!Opt_nosens)
        {
            F_ext_RL = EEWRightLeg->read(true);
            F_ext_LL = EEWLeftLeg->read(true);
        }
    }
#endif
    
    
   
    // check if the robot is not in contact with the ground
    static bool on_ground = true;
    if ((*F_ext_LL)[0] > 50 &&
        (*F_ext_RL)[0] > 50  )
    {
        on_ground = true;
    }
    else
    {
        on_ground = false;
        for (int jj=0; jj<6; jj++) (*F_ext_LL)[jj] =  (*F_ext_RL)[jj] = 1e-20;
        (*F_ext_LL)[0] =  (*F_ext_RL)[0] = 1e+20;
    }
    
    //***************************** Computing ZMP ****************************************
    
    computeZMP(&zmp_xy, F_ext_LL, F_ext_RL, encs_l, encs_r);
    
    if(verbose){
        fprintf(stderr, "ZMP coordinates: %f %f       %d \n",zmp_xy[0],zmp_xy[1],(int)(torso));
        fprintf(stderr, "ZMP desired: %f %f\n",zmp_des[0], zmp_des[1]);
    }
    
    //*********************** SENDING ZMP COORDINATES TO GuiBalancer *********************
    if (!Opt_nosens)
        if (objPort->getOutputCount() > 0 && !Opt_nosens)
        {
            objPort->prepare() = zmp_xy;
            objPort->setEnvelope(timeStamp);
            objPort->write();
        }
    
    //********************** SENDING ZMP COORDS TO BE PLOT BY ICUBGUI *****************************
    if (!Opt_nosens)
        if (objPort2->getOutputCount() > 0)
        {
            Matrix Trans_lastRot(4,4); Trans_lastRot.zero();
            Trans_lastRot.setSubmatrix(rot_f,0,0);
            Trans_lastRot(3,3) = 1;
            Trans_lastRot(1,3) = -separation(encs_r,encs_l)/2;
            Vector zmp_xy_trans(4); zmp_xy_trans.zero();
            zmp_xy_trans.setSubvector(0,zmp_xy);
            zmp_xy_trans(3)=1;
            zmp_xy_trans = Hright*(Right_Leg->getH())*SE3inv(Trans_lastRot)*zmp_xy_trans;
            objPort2->prepare() = zmp_xy_trans.subVector(0,1);
            objPort2->write();
        }
    //*********************** Reading ZMP derivative **********************************************
    
    iCub::ctrl::AWPolyElement el;
    el.data=zmp_xy;
    el.time=Time::now();
    zmp_xy_vel = zmp_xy_vel_estimator->estimate(el);
    
    //*********************** Obtaining Transformation Matrix from Root to WRF ********************
    
    Matrix RLeg_RotTrans(4,4); RLeg_RotTrans.zero();
    RLeg_RotTrans = Hright * Right_Leg->getH();
    
    Matrix lastRotTrans(4,4);
    lastRotTrans.zero();
    lastRotTrans(0,2) = lastRotTrans(2,0) = lastRotTrans(3,3) = 1;
    lastRotTrans(1,1) = -1;
    lastRotTrans(1,3) = -1*RLeg_RotTrans(1,3);
    
    Matrix H_r_f(4,4); H_r_f.zero();
    H_r_f = RLeg_RotTrans * lastRotTrans;
    Matrix H_f_r = SE3inv(H_r_f);
    
    //********************** SUPPORT POLYGON DEFINITION ****************************
    /*        double bound_fw =  0.10;
     double bound_bw = -0.05;
     double security_offset = 0.02;
     
     //zmp support polygon
     if((zmp_xy[0]>bound_fw) || (zmp_xy[0]<bound_bw)){
     if(zmp_xy[0]<bound_bw){
     zmp_des[0] = bound_bw + security_offset;
     }
     else{
     if(zmp_xy[0]>bound_fw){
     zmp_des[0] = bound_fw - security_offset;
     }
     }
     }*/
    //********************** CONTROL BLOCK 1 **************************************
    double delta_zmp [2];
    delta_zmp [0] = zmp_xy[0] - zmp_des[0];
    delta_zmp [1] = zmp_xy[1] - zmp_des[1];
    double delta_zmp_vel [2];
    delta_zmp_vel[0] = zmp_xy_vel[0] - 0.0;
    delta_zmp_vel[1] = zmp_xy_vel[1] - 0.0;
    double delta_com [2];
    double delta_theta;
    double delta_phi;
    delta_com [0] = + Kp_zmp_x * delta_zmp[0] - Kd_zmp_x * delta_zmp_vel[0];
    delta_com [1] = + Kp_zmp_y * delta_zmp[1] - Kd_zmp_y * delta_zmp_vel[1];
    delta_theta   = - Kp_theta * delta_zmp[0] - Kd_theta * delta_zmp_vel[0];
    delta_phi     = - Kp_phi   * delta_zmp[1] - Kd_phi   * delta_zmp_vel[1];
    
    double der_part =  Kd_zmp_x * delta_zmp_vel[0];
    if(verbose)
        fprintf(stderr, "\n Kd*delta_zmp_vel[0] =  %f , %f \n", der_part, zmp_xy_vel[0]);
    
    //********************** CONTROL BLOCK 2 **************************************
    double COM_des[2];
    double phi_des;
    double theta_des;
    double COM_ref[2];
    double phi_ref;
    double theta_ref;
    //From initial standing position
    COM_des[0] = 0.0;
    COM_des[1] = 0.0;
    theta_des  = 0;
    phi_des   = 0;
    COM_ref[0] = delta_com[0] + COM_des[0];
    COM_ref[1] = delta_com[1] + COM_des[1];
    theta_ref  = delta_theta  + theta_des;
    phi_ref    = delta_phi    + phi_des;
    
    Vector COM_r(2); COM_r[0] = COM_ref[0]; COM_r[1] = COM_ref[1];
    COM_ref_port->prepare() = COM_r;
    COM_ref_port->setEnvelope(timeStamp);
    COM_ref_port->write();
    
    //********************** CONTROL BLOCK 3 **************************************
    double q_torso_theta        = 0.0;
    double q_torso_phi          = 0.0;
    double q_left_ankle_theta   = 0.0;
    double q_left_ankle_phi     = 0.0;
    double q_right_ankle_theta  = 0.0;
    double q_right_ankle_phi    = 0.0;
    
    double q0_right_leg         = 0.0;
    double q0_left_leg          = 0.0;
    
    double length_leg = 0.47;       //empirically taken this value from robot standing. COM z coord. from w.r.f.
    if(verbose)
        fprintf(stderr, "COM_ref = %+6.6f\n",COM_ref[0]);
    
    q_right_ankle_theta = asin(-COM_ref[0]/length_leg)*CTRL_RAD2DEG;
    
    //q_right_ankle_phi   = -asin(-COM_ref[1]/length_leg)*CTRL_RAD2DEG;
    q_right_ankle_phi = 0.0;
    q_left_ankle_phi  = 0.0;
    
    if(!torso){
        q_torso_theta = 2.0;
    }
    else{
        q_torso_theta = theta_ref;
    }
    
    //********************** COMPUTE JACOBIANS ************************************
    // In this notation we define a,b,c as follows:
    // right_foot_reference_frame = 'a';
    // base_reference_frame       = 'b';
    // left_foot_reference_frame  = 'c';
    
    Ienc_RL->getEncoders(encs_r.data());      encs_r_rad = CTRL_DEG2RAD * encs_r;       Right_Leg->setAng(encs_r_rad);
    Ienc_LL->getEncoders(encs_l.data());      encs_l_rad = CTRL_DEG2RAD * encs_l;       Left_Leg->setAng(encs_l_rad);
    
    Tba = Right_Leg->getH();
    Tbc =  Left_Leg->getH();
    
#ifdef COMPUTE_FINITE_DIFF
    Vector q0R(6);    Vector q1R(6);
    q0R(0) = 0.3;  q0R(1) = 0.2;  q0R(2) = 0.2;  q0R(3) = 0.1;  q0R(4) = 0.0;  q0R(5) = 0.1;
    q1R(0) = 0.2;  q1R(1) = 0.0;  q1R(2) = 0.0;  q1R(3) = 0.0;  q1R(4) = 0.1;  q1R(5) = 0.1;
    Vector q0L(6);    Vector q1L(6);
    q0L(0) = 0.1;  q0L(1) = 0.1;  q0L(2) = 0.2;  q0L(3) = 0.0;  q0L(4) = 0.1;  q0L(5) = 0.0;
    q1L(0) = 0.1;  q1L(1) = 0.2;  q1L(2) = 0.3;  q1L(3) = 0.4;  q1L(4) = 0.5;  q1L(5) = 0.6;
    Vector delta_e = computeDeltaError(q0R, q0L, q1R, q1L);
    Tba = Right_Leg->getH(q0R*CTRL_DEG2RAD);
    Tbc =  Left_Leg->getH(q0L*CTRL_DEG2RAD);
#endif
    
    Tab = iCub::ctrl::SE3inv(Tba);
    Tac = Tab * Tbc;
    
    Jba = Right_Leg->GeoJacobian();
    Jbc = Left_Leg->GeoJacobian();
    
    pba = Tba.submatrix(0, 2, 3, 3);
    pbc = Tbc.submatrix(0, 2, 3, 3);
    pac = Tac.submatrix(0, 2, 3, 3);
    
    Rba = Tba.submatrix(0, 2, 0, 2);
    Rbc = Tbc.submatrix(0, 2, 0, 2);
    Rac = Tac.submatrix(0, 2, 0, 2);

    // We compute the velocity of 'c' with respect to 'a'. Linear velocity is dpac, angular is doac.
    // We assume that we can compute the velocity of 'a' and 'c' with respct to 'b'.
    //
    //   | I    S(pac)  |  | dpac |   |  Rab    S(pab)*Rab |  ( | I    S(pbc)  |  | dpbc |   | I    S(pba)  |  | dpba | )
    //   |              |  |      | = |                    |  ( |              |  |      | - |              |  |      | )
    //   | 0       I    |  | doac |   |   0          Rab   |  ( | 0       I    |  | dobc |   | 0       I    |  | doba | )
    //
    // which becomes:
    //
    //   | I    S(pac)  |  | dpac |   |  Rab          0    |  ( | I    S(rca_b)|  | dpbc |    | dpba | )
    //   |              |  |      | = |                    |  ( |              |  |      | -  |      | )
    //   | 0       I    |  | doac |   |   0          Rab   |  ( | 0       I    |  | dobc |    | doba | )
    //
    // with rca_b the vector from 'c' to 'a' in 'b', i.e. rca_b = -R_ab^T * pac = -Rba*pac. The above can be written:
    //
    //   | dpac |   | I    -S(pac) | |  Rab          0    |  ( | I   -S(rca_b) |  | dpbc |    | dpba | )
    //   |      | = |              | |                    |  ( |               |  |      | -  |      | )
    //   | doac |   | 0       I    | |   0          Rab   |  ( | 0       I     |  | dobc |    | doba | )
    //
    // In particular, as expected the velcoity of 'c' with repsect to 'a' is zeros iff:
    //
    //   | I  -S(rca_b) |  | dpbc |    | dpba |
    //   |              |  |      | -  |      | = 0
    //   | 0       I    |  | dobc |    | doba |
    //
    // where:
    //
    //    | dpba |               | dpbc |
    //    |      | = Jba * dqa,  |      | = Jbc * dqc,
    //    | doba |               | dobc |
    //
    // and:
    //                                                                |                                    |
    //   | dpac |            | I    -S(pac) | |  Rab          0    |  |          ( I   -S(rca_b) )         |
    //   |      | = Jac dq = |              | |                    |  | -Jba   | (               ) *Jbc    |  dq
    //   | doac |            | 0       I    | |   0          Rab   |  |          ( 0       I     )         |  
    //                                                                |                                    |
    
    pab =   -1.0 * Rba.transposed() * pba;
    pcb =   -1.0 * Rbc.transposed() * pbc;
    rca_b = -1.0 * Rba * pac;
    
    velTranslator(Spac, pac);
    velTranslator(Srca, rca_b);
    rotTranslator(Rrab, Rba.transposed());
    
    Jac.setSubmatrix(zeros(Jba.rows(), Jba.cols()) - Jba, 0, 0);
    Jac.setSubmatrix(Srca * Jbc, 0, Jbc.cols());
    Jac = Spac * Rrab * Jac;
    
    // Let's now implement the control strategy for keeping the relative left-right foot constant. Using the same
    // notation used in Sciavicco-Siciliano (Section Orientation error, axis angle notation) we have:
    //
    // The position (ep) and orientation (eo) error are defined as:
    //
    //                             1
    // ep = pac_d - pac,     eo =  - (S(ne)nd + S(se)sd + S(ae)ad)
    //                             2
    //
    // where pac is the ac translation, pac_d its desired value and:
    //
    // Rac_d = [nd sd ad],    Rac = [ne se ae].
    //
    // The control loop is based on the following equation:
    //
    //   | I   0 |           |    dpac_d     |   | dep |   | Kp * ep |
    //  -|       |  Jac dq + |               | = |     | = |         |
    //   | 0   L |           |  L^T (dRac_d) |   | deo |   | Ko * eo |
    //
    // where:
    //         1 (                                               )
    //   L = - - ( S(nd) * S(ne) + S(sd) * S(se) + S(ad) * S(ae) )
    //         2 (                                               )
    //
    // Rearranging we have:
    //
    //            |    dpac_d    +      Kp * ep   |
    //   Jac dq = |                               |
    //            |  L^-1 (L^T dRac_d + Ko * eo)  |
    //
    // The current implementation computes dqc (left leg velocities) assuming dqr = 0:
    //
    //                       |    dpac_d    +      Kp * ep   |
    //    dqc= pinv(Jac_qc)  |                               |
    //                       |  L^-1 (L^T dRac_d + Ko * eo)  |
    //
    // where Jac = [ Jac_qa Jac_qc ].
    
    // Desired configuration and its derivative (currently zero)
    Matrix Tac0 = SE3inv(Right_Leg->getH(zeros(njRL))) * Left_Leg->getH(zeros(njLL));
    Rac_d = Tac0.submatrix(0, 2, 0, 2);
    pac_d = Tac0.submatrix(0, 2, 3, 3);
    dpac_d = zeros(3, 1);
    dRac_d = zeros(3, 1);
    if (verbose)
    {
        fprintf(stderr, "pac_d: %s\n", pac_d.toString().c_str());
        fprintf(stderr, "Rac_d: %s\n", iCub::ctrl::dcm2axis(Rac_d).toString().c_str());
    }
    
    nd = Rac_d.submatrix(0, 2, 0, 0);   ne = Rac.submatrix(0, 2, 0, 0);
    sd = Rac_d.submatrix(0, 2, 1, 1);   se = Rac.submatrix(0, 2, 1, 1);
    ad = Rac_d.submatrix(0, 2, 2, 2);   ae = Rac.submatrix(0, 2, 2, 2);

    hat(ne_hat, ne);    hat(se_hat, se);    hat(ae_hat, ae);
    hat(nd_hat, nd);    hat(sd_hat, sd);    hat(ad_hat, ad);
    
    ep = pac_d - pac;
    eo = 0.5*(ne_hat * nd     + se_hat * sd     + ae_hat * ad);
    L  =-0.5*(nd_hat * ne_hat + sd_hat * se_hat + ad_hat * ae_hat);
        
    Matrix ded(6,1);    Matrix Kp = eye(3, 3);   Matrix Ko = eye(3, 3);
    ded.setSubmatrix(dpac_d + Kp * (pac_d-pac),                  0, 0);
    ded.setSubmatrix(luinv(L)*(L.transposed()*dRac_d + Ko * eo), 3, 0);

    Matrix Jac_qa = Jac.submatrix(0, 5, 0, njRL-1);
    Matrix Jac_qc = Jac.submatrix(0, 5, njRL, njRL+njLL-1);
     
    Vector dqLL = zeros(njLL);
    dqLL = (100.0*pinvDamped(Jac_qc, 0.01)*ded).getCol(0);
    Ivel_LL->velocityMove(dqLL.data());
    if (verbose)
    {
        fprintf(stderr, "Position    error is: %s\n", ep.transposed().toString().c_str());
        fprintf(stderr, "Orientation error is: %s\n", eo.transposed().toString().c_str());
    }
    
    
#ifdef COMPUTE_FINITE_DIFF
    
    Matrix tmpM(6,6);
    tmpM.setSubmatrix(eye(3)     , 0, 0);   tmpM.setSubmatrix(zeros(3, 3), 0, 3);
    tmpM.setSubmatrix(zeros(3, 3), 3, 0);   tmpM.setSubmatrix(L,           3, 3);

    Vector dq(12);
    dq.setSubvector(0, q1R-q0R);
    dq.setSubvector(6, q1L-q0L);
    Vector de  = zeros(6) - tmpM*Jac*dq;
    
    if (verbose)
    {
        fprintf(stderr, "delta_e: %s\n", delta_e.toString().c_str());
        fprintf(stderr, "de:      %s\n", de.toString().c_str());
        // fprintf(stderr, "Velocity right foot in b: %s\n", (Jba*(q1R-q0R)).toString().c_str());
        // fprintf(stderr, "Velocity left  foot in b: %s\n", (Spac*Rrab*Srca*Jbc*(q1L-q0L)).toString().c_str());
        // fprintf(stderr, "Res: %s\n", L.toString().c_str());
    }
#endif
    //
    // Here we compute the projection of the center of mass on the ground
    // and the associated Jacobian. First we compute an axis zg which is
    // orthogonal to the ground plane. Three cases are possible:
    //
    // right foot single support phase: zg = zr
    //
    // left foot single support phase : zg = zf
    //                                                 || z - z_r ||   ||  | I |     | z_r | ||
    // double support phase           : zg = arg min_z ||         || = ||  |   | z - |     | ||
    //                                                 || z - z_l ||   ||  | I |     | z_l | ||
    //
    // Everything is computed directly with respect to the coordinate frame b.
    
    enum phase { LEFT_SUPPORT, RIGHT_SUPPORT, BOTH_SUPPORT };
    phase current_phase = BOTH_SUPPORT;
    
    Matrix zg_b(3,1);
    Matrix zl_b = Rbc.submatrix(0, 2, 2, 2);
    Matrix zr_b = Rba.submatrix(0, 2, 2, 2);

    switch (current_phase)
    {
        case LEFT_SUPPORT:
            zg_b = zl_b;
            break;
            
        case RIGHT_SUPPORT:
            zg_b = zr_b;
            break;

        case BOTH_SUPPORT:
        {
            Matrix A(6,3);
            A.setSubmatrix(eye(3), 0, 0);
            A.setSubmatrix(eye(3), 3, 0);
            Matrix b(6,1);
            b.setSubmatrix(zr_b, 0, 0);
            b.setSubmatrix(zl_b, 3, 0);
            zg_b = pinv(A)*b;
            zg_b = zg_b / norm(zg_b.getCol(0));
        }
            break;
    }
    
    // Let's now compute the projection of a point p_b (eventually the COM)
    // passing trought the right foot (pba) and orthogonal to zg_b. All these
    // computations are in 'b'. The plane equation is:
    //
    // P_b : zg_b^T (x_b - pba) = 0
    //
    // The projection point belongs to a line parallel to zg_b and passing through p
    //
    // r_b : x_b = zg_b * v + p_b      for all v in R
    //
    // Intersecting plane and line and using zg_b^T * zg_b = 1 we have:
    //
    // zg_b^T (zg_b * v + p_b - pba) = 0
    //
    // v = zg_b^T (pba - p_b)
    //
    // which corresponds to the point:
    //
    // pi_b = zg_b zg_b^T (pba - p_b) + p_b
    //
    // or equivalently:
    //
    // pi_b = (I - zg_b zg_b^T) p_b + zg_b zg_b^T pba
    //
    // which gives the point x_b projection of p on the plane. We can also
    // consider referring all quantities to the reference frame 'a' (right foot)
    // with the advantage that zg will be constant during the right foot support.
    // In this case we have:
    //
    //             | 0 |                  | px_a |
    // pi_a = (I - | 0 | | 0 0 1 |) p_a = | py_a | = PI*p_a
    //             | 1 |                  |  0   |
    //
    // The point p_a is the center of mass, which is however expressed in
    // the reference frame 'b'. Therefore:
    //
    // pi_a = PI * (Rab * p_b + pab)    and pi_b = Rba * PI * (Rab * p_b + pab) + pba
    //
    // In homogenous coordinates:
    //
    // pi_b = Tba * PI2 * Tab * p_b
    //
    //          | 1  0  0  0 |
    //    PI2 = | 0  1  0  0 |
    //          | 0  0  0  0 |
    //          | 0  0  0  1 |
    //
    // Let's now compute the associated Jacobian. We have:
    //
    // dpi_a = PI * d(Rab * p_b + pab) = PI * (Rab dp_b(t) + [ I -S(Rab p_b) ] Jab * dq)
    //
    // where Jab is the 'a'-'b' Jacobian.
    //
    // dpi_b = d(Rba * pi_a + pba) = Rba * dpi_a + [ I -S(Rba pi_a) ] Jba*dq
    //
    // Finally, the velocity dp_b(t) is computed as:
    //
    // dp_b(t) = Jb_p*dq
    //
    // so that we have:
    //
    // dpi_b = Rba * PI * (Rab dp_b(t) + [ I -S(Rab p_b) ] Jab * dq) + [ I -S(Rba pi_a) ] Jba*dq
    //
    // dpi_b = Rba * PI * (Rab Jb_p*dq + [ I -S(Rab p_b) ] Jab * dq) + [ I -S(Rba pi_a) ] Jba*dq
    //
    // dpi_b = [Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba] dq
    //
    //               (                                  )
    // dpi_a = Rab * (dpi_b - [ I -S(Rba pi_a) ] Jba*dq )
    //               (                                  )
    //
    //               (                                               )
    // dpi_a = Rab * (Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab )
    //               (                                               )
    //
    //               (                                               )
    // dpi_a =       (    PI Rab Jb_p +     PI [ I -S(Rab p_b) ] Jab )
    //               (                                               )
    //
    // and therefore we have the following Jacobians:
    //
    // Jpi_b = Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba
    //
    // Jpi_a =     PI Rab Jb_p +     PI [ I -S(Rab p_b) ] Jab 
    //
    // where as usual we can compute Jab as a function of Jba
    //
    //   | dpab |             | Rab  -S(pab)Rab |
    //   |      | = Jab dq = -|                 |  Jba dq
    //   | doab |             | 0       Rab     |
    //
    // Considering all the joints involve in the COM, we can consider the following
    // ordering that is equivalent to the one used in the definition of the
    // COM Jacobian Jb_com by the wholeBodyDynamics module.
    //
    //        q = [q_left_arm,  q_right_arm, q_torso, q_left_leg,  q_right_leg]
    //
    // Thus in the above formulas, Jb_p refers only to the last columns of Jb_com:
    //
    //   Jb_com = [Jb_com_left_arm,  Jb_com_right_arm, Jb_com_torso, Jb_com_left_leg,  Jb_com_right_leg]
    //
    // or in other words we used Jb_com_right_leg = Jb_p. Therefore the projection:
    //
    //   Jpi_b_com = [Rba PI Rab Jb_com_left_arm,  Rba PI Rab Jb_com_right_arm, Rba PI Rab Jb_com_torso, Rba PI Rab Jb_com_left_leg,  Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba]
    //
    //   Jpi_b_com = Rba PI Rab Jb_com + [0,  0, 0, 0,  Rba PI [ I -S(Rab p_b) ] Jab + [ I -S(Rba pi_a) ] Jba]
    
  
    if (comPosPortString.empty())
        p_b  = zeros(3,1);
    else
        p_b.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
        
    if (comJacPortString.empty())
        Jb_p = zeros(3, Jba.cols());
    else
    {
        Jb_com   = (*COM_Jacob_port->read()).removeRows(3, 3);
        Jb_p     = Jb_com.submatrix(0, 2, njLL, njLL+njRL-1);
    }
    
    pi_a  =       PI * (Rba.transposed() * p_b + pab)      ;
    pi_b  = Rba * PI * (Rba.transposed() * p_b + pab) + pba;
    
    velTranslator(Spab, pab);
    Jab   = (-1.0) * Spab * Rrab * Jba;

    velTranslator(SRab_pb, Rba.transposed()*p_b);
    Jab_t = SRab_pb.submatrix(0, 2, 0, 5) * Jab;
    
    velTranslator(Srba_pi, Rba*pi_a);
    Jba_t = Srba_pi.submatrix(0, 2, 0, 5) * Jba;
    
    Jpi_b     = Rba * PI * Rba.transposed() * Jb_p   + Rba * PI * Jab_t + Jba_t;
    Jpi_a     =       PI * Rba.transposed() * Jb_p   +       PI * Jab_t ;
    
#ifdef COMPUTE_FINITE_DIFF
        Vector delta_pi_b = computeDeltaProjB(q0R, q0L, q1R, q1L);
        // fprintf(stderr, "delta_pi_b is:\n %s\n", delta_pi_b.toString().c_str());
        // fprintf(stderr, "dpi_b is:     \n %s\n", (Jpi_b*(q1R - q0R)).toString().c_str());
        Vector delta_pi_a = computeDeltaProjA(q0R, q0L, q1R, q1L);
        // fprintf(stderr, "delta_pi_a is:\n %s\n", delta_pi_a.toString().c_str());
        // fprintf(stderr, "dpi_a is:     \n %s\n", (Jpi_a*(q1R - q0R)).toString().c_str());
#endif
    
    // Finally let's compute the control loop that brings the robot in a desired configuration
    // with respect to a reference position. The best is to force the dynamics of the COM
    // projection in the reference frame 'a', i.e. to impose:
    //
    //           dpi_a = Kpi_a * (pi_a_d - pi_a)
    //
    // To do so:
    //                          |                                   |
    //          dpi_a = Rab  *  | dpi_b - [ I -S(Rba pi_a) ] Jba*dq |
    //                          |                                   |
    //
    //                      |                                                |
    //      dpi_a = Rab  *  | Rba PI Rab Jb_p + Rba PI [ I -S(Rab p_b) ] Jab |  = Jpi_a * dq
    //                      |                                                |
    //
    //     dq = inv(Jpi_a_com) * Kpi_a * (pi_a_d - pi_a)
    //
    // Assuming that we vant to use only the torso, we have:
    //
    //     Jpi_a_com_torso * dqtorso +  Jpi_a_com_left_leg * dqLL = Kpi_a * (pi_a_d - pi_a)
    //
    // and therefore:
    //
    //     dqtorso = inv(Jpi_a_com_torso) * (Kpi_a * (pi_a_d - pi_a) -  Jpi_a_com_left_leg * dqLL)
 
    if (!comJacPortString.empty())
    {
        double Kpi_a = 100.0;
        pi_a_d(0,0) = 0.0;    pi_a_d(1,0) = -0.12;    pi_a_d(2,0) =  0.02;
    
        Jpi_b_com = Rba * PI * Rba.transposed() * Jb_com;
        Jpi_a_com =       PI * Rba.transposed() * Jb_com;
        Jpi_b_com.setSubmatrix(Jpi_b_com.submatrix(0, 2, njLL, njLL+njRL-1) + Rba * PI * Jab_t + Jba_t, 0, njLL);
        Jpi_a_com.setSubmatrix(Jpi_a_com.submatrix(0, 2, njLL, njLL+njRL-1) +       PI * Jab_t        , 0, njLL);

        Matrix Jpi_a_com_torso    = Jpi_a_com.submatrix(0, 2, njLL+njRL,  njLL+njRL+njTO-1);
        Matrix Jpi_a_com_left_leg = Jpi_a_com.submatrix(0, 2,         0,  njLL-1);
        
        Vector de_pi_a = Kpi_a* (pi_a_d - pi_a).getCol(0) -  Jpi_a_com_left_leg * dqLL;
         
        //Vector dqRL = pinvDamped(Jpi_a_com.submatrix(0, 2, njLL, njLL+njRL-1), 0.01)*de_pi_a;
        //Ivel_RL->velocityMove(dqRL.data());

        Vector dqTO = pinvDamped(Jpi_a_com_torso, 0.0001)*de_pi_a;
        double swap = dqTO(0);
        dqTO(0) = dqTO(2);
        dqTO(2) = swap;
        Ivel_TO->velocityMove(dqTO.data());
        
        if (verbose)
        {
            // fprintf(stderr, "njHD: %d, njRA: %d, njLA: %d, njTO: %d, njRL: %d, njLL: %d, tot: %d\n", njHD, njRA, njLA, njTO, njRL, njLL, Jb_com.cols());
            // fprintf(stderr, "p_b is               :  \n%s\n", p_b.toString().c_str());
            // fprintf(stderr, "pi_a is              :  \n%s\n", (pi_a).toString().c_str());
            fprintf(stderr, "COM Position error is:  \n%s\n", (pi_a_d - pi_a).toString().c_str());
            // fprintf(stderr, "COM Jacobian        :\n%s\n", (Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)).toString().c_str());
            // fprintf(stderr, "Sent velocities     :\n%s\n",  dqTO.toString().c_str());
        }
#ifdef COMPUTE_FINITE_DIFF
        Vector dqTO(3);
        dqTO = zeros(3);
        dqTO(0) =  0;     dqTO(1) = 0;     dqTO(2) = -0.3;
        Vector dqTOrev(3);
        dqTOrev(0) = dqTO(2);     dqTOrev(1) = dqTO(1);     dqTOrev(2) = dqTO(0);
        
        Vector dqRL(6);
        dqRL = zeros(6);
        // dqRL(0) =  -1;     dqRL(1) = -0.5;     dqRL(2) = -0.3;
        // dqRL(3) =  -1;     dqRL(4) = -0.5;     dqRL(5) = -0.3;
        
        Vector dqLL(6);
        dqLL = zeros(6);
        // dqLL(0) =  -1;     dqLL(1) = -0.5;     dqLL(2) = -0.3;
        // dqLL(3) =  -1;     dqLL(4) = -0.5;     dqLL(5) = -0.3;
        
        
        Vector delta_b(3), delta_pi_a(3), delta_pi_b(3);
        computeDeltaProjReal(dqRL, dqLL, dqTO, delta_b, delta_pi_a, delta_pi_b);
        
        dqTOrev = dqTOrev * CTRL_DEG2RAD;
        dqRL    = dqRL    * CTRL_DEG2RAD;
        dqLL    = dqLL    * CTRL_DEG2RAD;
        
        // fprintf(stderr, "Real delta_b is:     \n %s\n", delta_b.toString().c_str());
        // fprintf(stderr, "Real db is:          \n %s\n", (Jb_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)*dqTOrev).toString().c_str());
        // fprintf(stderr, "Real PI*Rba^T*db is:       \n %s\n", (PI*Rba.transposed()*((Jb_com.submatrix(0, 2,     njLL, njLL+njRL-1)*dqRL) +
        //                                                  (Jb_com.submatrix(0, 2,         0,      njLL-1)*dqLL) +
        //                                                  (Jb_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)*dqTOrev))).toString().c_str());
        
        // fprintf(stderr, "Real delta_pi_a is:  \n %s\n", delta_pi_a.toString().c_str());
        // fprintf(stderr, "Real dpi_a is:       \n %s\n", ((Jpi_a_com.submatrix(0, 2,     njLL, njLL+njRL-1)*dqRL) +
        // (Jpi_a_com.submatrix(0, 2,         0,      njLL-1)*dqLL) +
        // (Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)*dqTOrev)).toString().c_str());
        
        // fprintf(stderr, "Real delta_pi_b is:  \n %s\n", delta_pi_b.toString().c_str());
        // fprintf(stderr, "Real dpi_b is:       \n %s\n", ((Jpi_b_com.submatrix(0, 2, njLL, njLL+njRL-1)*dqRL)+(Jpi_b_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)*dqTOrev)).toString().c_str());
        // fprintf(stderr, "Jpi_b_com_torso is: \n %s\n", Jpi_b_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1).toString().c_str());
        // fprintf(stderr, "Rba  is:            \n %s\n", Rba.toString().c_str());
        // fprintf(stderr, "Rba*Jpi_a_com  is:  \n %s\n", (Rba*Jpi_a_com.submatrix(0, 2, njLL+njRL, njLL+njRL+njTO-1)).toString().c_str());
#endif
    }
    //********************** SEND COMMANDS   **************************************
    // SATURATORS
    //torso forward/backward
    double limit = 20;
    if (q_torso_theta>limit)  q_torso_theta=limit;
    if (q_torso_theta<-limit) q_torso_theta=-limit;
    //torso right/left
    if (q_torso_phi>limit)  q_torso_phi=limit;
    if (q_torso_phi<-limit) q_torso_phi=-limit;
    
    //ankle backward/forward
    if (q_right_ankle_theta>limit)  q_right_ankle_theta=limit;
    if (q_right_ankle_theta<-limit) q_right_ankle_theta=-limit;
    
    //ankle right/left
    if (q_right_ankle_phi>10)  q_right_ankle_phi=10;
    if (q_right_ankle_phi<-5) q_right_ankle_phi=-5;
    
    //copying movement
    q_left_ankle_theta = q_right_ankle_theta;
    //q_left_ankle_phi = q_right_ankle_phi;  //WE NEED TO CALIBRATE AGAIN THIS JOINT BECAUSE -2 DEG FOR RIGHT LEG ARE ACTUALLY 0 FOR THE LEFT ONE.
    
    if(verbose){
        fprintf(stderr, "q theta to be sent: %+6.6f ", q_right_ankle_theta);
        fprintf(stderr, "q phi   to be sent: %+6.6f ", q_right_ankle_phi);
    }
    
    Vector ankle_ang(2); ankle_ang.zero();
    ankle_ang[0]=q_right_ankle_theta;
    ankle_angle->prepare()=ankle_ang;
    ankle_angle->setEnvelope(timeStamp);
    ankle_angle->write();
    
    if(verbose)
        fprintf(stderr, "q torso to be sent: %+6.6f\n", q_torso_theta);
    
#ifdef TH0_LEGS
    q0_right_leg = theta_ref;
    q0_left_leg = q0_right_leg;
#endif
    
    if(!Opt_display)
    {
        if (on_ground)
        {
            if(torso){
                Ipid_TO->setReference(2,q_torso_theta);
                // Ipid_TO->setReference(1,q_torso_phi);
            }
            
            Ipid_RL->setReference(4,q_right_ankle_theta);
            Ipid_LL->setReference(4,q_left_ankle_theta);
            
            Ipid_RL->setReference(0,q0_right_leg);
            Ipid_LL->setReference(0,q0_left_leg);
            
            //open legs version
            Ipid_RL->setReference(5,q_right_ankle_phi);
            Ipid_LL->setReference(5,q_left_ankle_phi);
            
            //closed leg version
            //Ipid_RL->setReference(5,q_right_ankle_phi);
            //Ipid_RL->setReference(1,-q_right_ankle_phi);
            //Ipid_LL->setReference(5,-q_left_ankle_phi);
            //Ipid_LL->setReference(1,q_left_ankle_phi);
        }
    }
}


void comStepperThread::computeZMP(Vector* zmp_xy, Vector *F_ext_LL, Vector *F_ext_RL, Vector encs_l, Vector encs_r)
{
    //rotation of the measured force
    (*F_ext_RL).setSubvector(0,rot_f*((*F_ext_RL).subVector(0,2)));
    (*F_ext_LL).setSubvector(0,rot_f*((*F_ext_LL).subVector(0,2)));
    (*F_ext_RL).setSubvector(3,rot_f*((*F_ext_RL).subVector(3,5)));
    (*F_ext_LL).setSubvector(3,rot_f*((*F_ext_LL).subVector(3,5)));
    
    // CoP Right Leg
    yRL =  (*F_ext_RL)[3]/(*F_ext_RL)[2];
    xRL = -(*F_ext_RL)[4]/(*F_ext_RL)[2];
    
    double sep = separation(encs_r,encs_l);
    if(verbose)
        fprintf(stderr, "\n FEET SEPARATION: %f\n", sep);
    
    // yLL =  (*F_ext_LL)[3]/(*F_ext_LL)[2] - separation(encs_r, encs_l)/2;
    yLL =  (*F_ext_LL)[3]/(*F_ext_LL)[2];
    xLL = -(*F_ext_LL)[4]/(*F_ext_LL)[2];
    
    xDS =   ((*F_ext_RL)[2]*xRL + (*F_ext_LL)[2]*xLL)/((*F_ext_RL)[2] + (*F_ext_LL)[2]);
    yDS =  -((*F_ext_RL)[2]*yRL + (*F_ext_LL)[2]*yLL)/((*F_ext_RL)[2] + (*F_ext_LL)[2]);
    
    (*zmp_xy)[0] = xDS;
    (*zmp_xy)[1] = yDS;
}

double comStepperThread::separation(Vector encs_r, Vector encs_l){
    
    PoseRightLeg = Right_Leg->EndEffPose(CTRL_DEG2RAD*encs_r,false);
    PoseRightLeg.resize(3);
    PoseRightLeg.push_back(1);
    PoseRightLeg = Hright*PoseRightLeg;
    
    PoseLeftLeg = Left_Leg->EndEffPose(CTRL_DEG2RAD*encs_l,false);
    PoseLeftLeg.resize(3);
    PoseLeftLeg.push_back(1);
    PoseLeftLeg = Hleft*PoseLeftLeg;
    
    double sep;
    sep = fabs(PoseLeftLeg[1]) + fabs(PoseRightLeg[1]);
    return sep;
}

bool comStepperThread::onStop()
{
    fprintf(stderr, "Stopping the comStepperThread\n");
    return true;
}

void comStepperThread::threadRelease()
{    /* Should delete dynamically created data-structures*/
    
    Ivel_LL->stop();    Ipos_LL->setPositionMode();
    Ivel_RL->stop();    Ipos_RL->setPositionMode();
    Ivel_TO->stop();    Ipos_TO->setPositionMode();
    
    fprintf(stderr, "Releasing the comStepperThread\n");
    if(!Opt_nosens)
    {
        fprintf(stderr, "Closing EEWRightLeg port \n");
        closePort(EEWRightLeg);
        
        fprintf(stderr, "Closing EEWLeftLeg port \n");
        closePort(EEWLeftLeg);
        
        fprintf(stderr, "Closing EEWRightAnkle port\n");
        closePort(EEWRightAnkle);
        
        fprintf(stderr, "Closing EEWLeftAnkle port\n");
        closePort(EEWLeftAnkle);
        
        fprintf(stderr, "Closing Object Port \n");
        closePort(objPort);
        
        fprintf(stderr, "Closing Object2 Port \n");
        closePort(objPort2);
        
        fprintf(stderr, "Closing desired_zmp port\n");
        closePort(desired_zmp);
        
    }
    
    if(!comPosPortString.empty())
    {
        fprintf(stderr, "Closing COM_Posit_port\n");
        closePort(COM_Posit_port);
    }

    if(!comJacPortString.empty())
    {
        fprintf(stderr, "Closing COM_Jacob_port\n");
        closePort(COM_Jacob_port);
    }

    fprintf(stderr, "Closing commanded_ankle_port\n");
    closePort(ankle_angle);
    fprintf(stderr, "Closing COM_ref_port\n");
    closePort(COM_ref_port);    
    fprintf(stderr, "Deleting the right leg\n");
    delete Right_Leg;
    fprintf(stderr, "Deleting the left leg\n");
    delete Left_Leg;
    fprintf(stderr, "Deleting the velocity estimator\n");
    delete zmp_xy_vel_estimator;
    fprintf(stderr, "Exiting comStepperThread::threadRelease\n");
}

void comStepperThread::closePort(Contactable *_port)
{
    if(_port)
    {
        _port->interrupt();
        _port->close();
        
        delete _port;
        _port = 0;
    }
    
}

void comStepperThread::hat(Matrix &hat_p, Matrix p)
{
    
    hat_p(0,0)=       0;  hat_p(0,1)= -p(2,0);  hat_p(0,2)= +p(1,0);
    hat_p(1,0)= +p(2,0);  hat_p(1,1)=       0;  hat_p(1,2)= -p(0,0);
    hat_p(2,0)= -p(1,0);  hat_p(2,1)= +p(0,0);  hat_p(2,2)= 0;
    
}

void comStepperThread::velTranslator(Matrix &out, Matrix p)
{
    
    Matrix hat_p(3,3);
    hat(hat_p, p);
    out.setSubmatrix(eye(3,3)  , 0, 0);    out.setSubmatrix(zeros(3,3) - hat_p, 0, 3);
    out.setSubmatrix(zeros(3,3), 3, 0);    out.setSubmatrix(eye(3,3)          , 3, 3);
}

void comStepperThread::rotTranslator(Matrix &out, Matrix R)
{
    out.setSubmatrix(R         , 0, 0);    out.setSubmatrix(zeros(3,3), 0, 3);
    out.setSubmatrix(zeros(3,3), 3, 0);    out.setSubmatrix(R         , 3, 3);
}

Vector comStepperThread::computeDeltaError(Vector q0R, Vector q0L, Vector q1R, Vector q1L)
{
    Matrix Tba0 = Right_Leg->getH(q0R*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(q0L*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    Matrix ep0 = pac0 - pac0;
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Vector eo0_tmp = iCub::ctrl::dcm2axis(Rac0 * Rac0.transposed());
    Vector eo0 = eo0_tmp.subVector(0, 2) * sin(eo0_tmp(3));;
    
    Matrix Tba1 = Right_Leg->getH(q1R*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH(q1L*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    Matrix ep1 = pac0 - pac1;
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Vector eo1_tmp = iCub::ctrl::dcm2axis(Rac0 * Rac1.transposed());
    Vector eo1 = eo1_tmp.subVector(0, 2) * sin(eo1_tmp(3));;
    
    Matrix delta_ep = ep1 - ep0;
    Vector delta_eo = eo1 - eo0;
    
    Vector delta_e(6);
    delta_e(0) = delta_ep(0,0);  delta_e(1) = delta_ep(1,0);  delta_e(2) = delta_ep(2,0);
    delta_e.setSubvector(3, delta_eo);

    return delta_e;

}

Vector comStepperThread::computeDeltaProjB(Vector q0R, Vector q0L, Vector q1R, Vector q1L)
{
    
    Matrix Tba0 = Right_Leg->getH(q0R*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(q0L*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Matrix pab0 = -1.0*(Rba0.transposed()*pba0);
    Vector pi_b0 = Rba0 * PI * (Rba0.transposed() * zeros(3) + pab0.getCol(0)) + pba0.getCol(0);
    
    Matrix Tba1 = Right_Leg->getH(q1R*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH(q1L*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Matrix pab1 = -1.0*(Rba1.transposed()*pba1);
    Vector pi_b1 = Rba1 * PI * (Rba1.transposed() * zeros(3) + pab1.getCol(0)) + pba1.getCol(0);
    
    // fprintf(stderr, "pi_b0: %s\n", pi_b0.toString().c_str());
    // fprintf(stderr, "pab0 : %s\n",  pab0.toString().c_str());
    // fprintf(stderr, "pi_b1: %s\n", pi_b1.toString().c_str());
    // fprintf(stderr, "pab1 : %s\n",  pab1.toString().c_str());
    Vector delta_pi_b = pi_b1 - pi_b0;
    return delta_pi_b;
    
}

Vector comStepperThread::computeDeltaProjA(Vector q0R, Vector q0L, Vector q1R, Vector q1L)
{
    
    Matrix Tba0 = Right_Leg->getH(q0R*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(q0L*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Matrix pab0 = -1.0*(Rba0.transposed()*pba0);
    Vector pi_a0 = PI * (Rba0.transposed() * zeros(3) + pab0.getCol(0));
    
    Matrix Tba1 = Right_Leg->getH(q1R*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH(q1L*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Matrix pab1 = -1.0*(Rba1.transposed()*pba1);
    Vector pi_a1 = PI * (Rba1.transposed() * zeros(3) + pab1.getCol(0));
    
    // fprintf(stderr, "pi_b0: %s\n", pi_b0.toString().c_str());
    // fprintf(stderr, "pab0 : %s\n",  pab0.toString().c_str());
    // fprintf(stderr, "pi_b1: %s\n", pi_b1.toString().c_str());
    // fprintf(stderr, "pab1 : %s\n",  pab1.toString().c_str());
    //Vector delta_pi_b = pi_b1 - pi_b0;
    Vector delta_pi_a = pi_a1 - pi_a0;
    return delta_pi_a;
    
}

void comStepperThread::computeDeltaProjReal(Vector dqR, Vector dqL, Vector dqT, Vector &delta_b, Vector &delta_pi_a, Vector &delta_pi_b)
{
    Matrix p_b0(3,1);
    Matrix p_b1(3,1);
    Vector j0RL(6);
    Ienc_RL->getEncoders(j0RL.data());
    Vector j0LL(6);
    Ienc_LL->getEncoders(j0LL.data());
    Vector j0TO(3);
    Ienc_TO->getEncoders(j0TO.data());
    if (comPosPortString.empty())
        p_b0  = zeros(3,1);
    else
        p_b0.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
    
    Matrix Tba0 = Right_Leg->getH(j0RL*CTRL_DEG2RAD);
    Matrix Tbc0 =  Left_Leg->getH(j0LL*CTRL_DEG2RAD);
    Matrix Tab0 = iCub::ctrl::SE3inv(Tba0);
    Matrix Tac0 = Tab0 * Tbc0;
    
    Matrix pba0 = Tba0.submatrix(0, 2, 3, 3);
    Matrix pbc0 = Tbc0.submatrix(0, 2, 3, 3);
    Matrix pac0 = Tac0.submatrix(0, 2, 3, 3);
    
    Matrix Rba0 = Tba0.submatrix(0, 2, 0, 2);
    Matrix Rbc0 = Tbc0.submatrix(0, 2, 0, 2);
    Matrix Rac0 = Tac0.submatrix(0, 2, 0, 2);
    Matrix pab0 = -1.0*(Rba0.transposed()*pba0);
    Vector pi_a0 = PI * (Rba0.transposed() * p_b0.getCol(0) + pab0.getCol(0));
    Vector pi_b0 = Rba0 * PI * (Rba0.transposed() * p_b0.getCol(0) + pab0.getCol(0)) + pba0.getCol(0);

    Ipos_LL->setRefSpeed(0, 100);    Ipos_LL->setRefSpeed(3, 100);
    Ipos_LL->setRefSpeed(1, 100);    Ipos_LL->setRefSpeed(4, 100);
    Ipos_LL->setRefSpeed(2, 100);    Ipos_LL->setRefSpeed(5, 100);
    Ipos_LL->positionMove((j0LL+dqL).data());

    Ipos_RL->setRefSpeed(0, 100);    Ipos_RL->setRefSpeed(3, 100);
    Ipos_RL->setRefSpeed(1, 100);    Ipos_RL->setRefSpeed(4, 100);
    Ipos_RL->setRefSpeed(2, 100);    Ipos_RL->setRefSpeed(5, 100);
    Ipos_RL->positionMove((j0RL+dqR).data());

    Ipos_TO->setRefSpeed(0, 100);
    Ipos_TO->setRefSpeed(1, 100);
    Ipos_TO->setRefSpeed(2, 100);
    fprintf(stderr, "COM is    %s \n", p_b0.toString().c_str());
    Ipos_TO->positionMove((j0TO+dqT).data());
    
    Time::delay(4);
    if (comPosPortString.empty())
        p_b1  = zeros(3,1);
    else
        p_b1.setCol(0, (*COM_Posit_port->read()).subVector(0, 2)) ;
    fprintf(stderr, "COM is now %s \n", p_b1.toString().c_str());
    
    Matrix Tba1 = Right_Leg->getH((j0RL+dqR)*CTRL_DEG2RAD);
    Matrix Tbc1 = Left_Leg->getH((j0LL+dqL)*CTRL_DEG2RAD);
    Matrix Tab1 = iCub::ctrl::SE3inv(Tba1);
    Matrix Tac1 = Tab1 * Tbc1;
    
    Matrix pba1 = Tba1.submatrix(0, 2, 3, 3);
    Matrix pbc1 = Tbc1.submatrix(0, 2, 3, 3);
    Matrix pac1 = Tac1.submatrix(0, 2, 3, 3);
    
    Matrix Rba1 = Tba1.submatrix(0, 2, 0, 2);
    Matrix Rbc1 = Tbc1.submatrix(0, 2, 0, 2);
    Matrix Rac1 = Tac1.submatrix(0, 2, 0, 2);
    Matrix pab1 = -1.0*(Rba1.transposed()*pba1);
    Vector pi_a1 = PI * (Rba1.transposed() * p_b1.getCol(0) + pab1.getCol(0));
    Vector pi_b1 = Rba1 * PI * (Rba1.transposed() * p_b1.getCol(0) + pab1.getCol(0)) + pba1.getCol(0);

    // fprintf(stderr, "pi_b0: %s\n", pi_b0.toString().c_str());
    // fprintf(stderr, "pab0 : %s\n",  pab0.toString().c_str());
    // fprintf(stderr, "pi_b1: %s\n", pi_b1.toString().c_str());
    // fprintf(stderr, "pab1 : %s\n",  pab1.toString().c_str());

    delta_b = p_b1.getCol(0) - p_b0.getCol(0);
    delta_pi_a = pi_a1 - pi_a0;
    delta_pi_b = pi_b1 - pi_b0;
}

