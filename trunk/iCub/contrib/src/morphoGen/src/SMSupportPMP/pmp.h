class PassiveMotionParadigm
{
	private:
	
		double Jan[10];
		double JanL[10]; //initial Joint angles


		double x_ini; // Using PMP+ Symmetry So Right and Left Arm movements are
		// coordinated by the forward inverse model of the left arm only
		double y_ini;
		double z_ini; 
		double x_iniL; // Init Config / O/p of target generator
		double y_iniL;
		double z_iniL;

		double x_fin,y_fin,z_fin;
		double x_finL,y_finL,z_finL;

		double Gam_Arr[20000],Gam_Arry[20000],Gam_Arrz[20000],q1[20000],q2[20000],q3[20000],q4[20000],q5[20000];
		double q6[20000],q7[20000],q8[20000],q9[20000],q10[20000];

		double Gam_ArrL[20000],Gam_ArryL[20000],Gam_ArrzL[20000],q1L[20000],q2L[20000],q3L[20000],q4L[20000],q5L[20000];
		double q6L[20000],q7L[20000],q8L[20000],q9L[20000],q10L[20000];

		double ang1,ang2,ang3,ang4,ang5,ang6,ang7,ang8,ang9,ang10,konst;
		double ang1L,ang2L,ang3L,ang4L,ang5L,ang6L,ang7L,ang8L,ang9L,ang10L;

		double angCup,angT1,angT2,angT3,angI1,angI2,angM1,angM2,angRP,angCupL,angTL1,angTL2,angTL3,angIL1,angIL2,angML1,angML2,angRPL;
        
		double janini3,janini4,janini5,janini6,janini7,janini8,janini9,janini0,janini2;
		double janini3L,janini4L,janini5L,janini6L,janini7L,janini8L,janini9L;
		
		double x_iniIC,y_iniIC,z_iniIC,x_iniICL,y_iniICL,z_iniICL;
		double target[3],targetL[3];
		double X_pos[3],ffield[3],JoVel[20],X_posL[3],ffieldL[3];
		double *ptr;
		//char* objEcYar ="ball";
		double KFORCE,ITERATION,RAMP_KONSTANT,t_dur,KOMP_JANG,KOMP_WAISZT,KOMP_WAISZT2,J2H,J7H,J8H,J9H,J3H;
		float s[5000];
		int ParamStiff;
		double x_fin1,y_fin1,z_fin1,x_fin2,y_fin2,z_fin2,x_fin1L,y_fin1L,z_fin1L,x_fin2L,y_fin2L,z_fin2L;
        double Gam_Arr1[20000],Gam_Arry1[20000],Gam_Arrz1[20000];
		double Gam_Arr2[20000],Gam_Arry2[20000],Gam_Arrz2[20000];
		double Gam_Arr1L[20000],Gam_Arry1L[20000],Gam_Arrz1L[20000];
		double Gam_Arr2L[20000],Gam_Arry2L[20000],Gam_Arrz2L[20000];
		double KXA,KXB,KYA,KYB,KXAL,KXBL,KYAL,KYBL,x_off1,z_off1,x_off2,z_off2,TSEC,STARTini,FrameRef,XinitTarg,ZinitTarg,csi_dot1,csi_dot;
        int dividen,NoBjS,Obj1ID,Obj2ID;
      // Global information management
		double MiniGoal[20], PlaceMap[10][10],BodyTrack[2][15],GoalSpace[10][23],WristOrR,WristOrL;
		double PickX,PickY,PickZ,PlacX,PlacY,PlacZ;
		double Proprioceptive[2];
		
	public:
		PassiveMotionParadigm();
		~PassiveMotionParadigm();
		double* forward_Kinematics(double *u , int l);
		double* forward_KinematicsL(double *uL , int lef);
		double* forward_KinematicsLRH(double *uLRH , int lefRH);
		double* forcefield(double *w, double*v);
		double* forcefieldL(double *wL, double*vL);
		double* PMP(double *force,double *forceL);
		double Gamma_Int(double *Gar,int n);
		double Gamma(int _Time);
		double Gamma1(int _Time1);
		void Reason(int typeGoal);
		int VTGS(double *MiniGoal, int ChoiceAct,int HandAct,int MSim, double Wrist);
		void MotCon(double T1, double T2, double T3,double TL1, double TL2, double TL3,int time, double Gam,int HAct);
		void MessagePassR();
		void MessagePassT();
		void MessagePassL();
		int FrameGoal();
		void MessageDevDriverR();
		void MessageDevDriverL();
		void MessageDevDriverT();
		void initiCubUp();
		void InitializeJan();
		void InitializeWrld(int visiontype);
		void CubGrazp1();
		void CubGrazp2();
		void CubGrazp3();
		void CubGrazp4();
		void CubGrazpL3();
		void CubGrazpL4();
		void CubUp(int HandUp);
		void CubRelease();
		void CubReleaseSoft();
		void Grab();
		void Kompliance(int TagK);
		void PredictReachability(int whichOb);
		void GetObjID();
		void GraspDetectVisionR();
		void GraspDetectVisionL();
		void GraspR();
		void GraspL();
		void PandP();

}; 

class PMP
{
public:
	PMP(void);
	~PMP(void);
};
