#ifndef GRASPER_H
#define GRASPER_H

#include <string>

#include <yarp\sig\Matrix.h>
#include <yarp\sig\Vector.h>
#include <yarp\os\Port.h>

#include "PoseGenerator.h"


class Grasper{


	static const string JOINTS_LEFT;
	static const string TOUCH_LEFT;
	static const string JOINTS_RIGHT;
	static const string TOUCH_RIGHT;

	static const string CONFIG;
	static const string OBJ_FOLD;
	static const string CUBOID_FOLD;

	static const string U_FILE;
	static const string S_FILE;
	static const string V_FILE;

	static const unsigned int NUM_FINGS;
	static const int NUM_SENS;
	static const double FING_THREAS;
	static const double INCREMENT;


	static yarp::sig::Matrix readMatrix(const std::string& filename,int cols);
	static yarp::sig::Vector readConfig(const std::string& path);

	//static void predict(yarp::sig::Matrix& vPred,const yarp::sig::Matrix& cents);
	static void move_fingers(const int firstJoint, const int lastJoint, const yarp::sig::Vector& data, yarp::os::Port& hand);
	static void squeeze(yarp::os::Port& touch, yarp::os::Port& hand);

	Grasper();
	~Grasper();
public:

	enum Joints{
		shoulder_pitch,
		shoulder_roll,
		shoulder_yaw,
		elbow,
		wrist_prosup,
		wrist_pitch,
		wrist_yaw,
		finger_add_abd,
		thumb_oppose,
		thumb_proximal,
		thumb_distal,
		index_proximal,
		index_distal,
		middle_proximal,
		middle_distal,
		pinky_ring,
		JointsNum
	};


	static void grasp(PoseGenerator::Hands domHand,PoseGenerator::ObjectClass objType);
	//static void keep();
};

#endif