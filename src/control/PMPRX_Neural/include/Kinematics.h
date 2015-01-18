
#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "../include/vecmath3D.h"
#define ROBOMOVE_API 
namespace LibRoboMove
{
	class Kinematics
	{
	public:
		// The vector type that is used for vector/matrix operations
		typedef vecmath3D::Vector<double> vecType;
		// The matrix type that is used for vector/matrix operations
		typedef vecmath3D::Matrix<double> matType;

	protected:
		/// Transformation from Flange to TCP
		matType mFlangeToTcp;
		/// Transformation from TCP to Flange
		matType mTcpToFlange;
		
			/// length of forearm
		double mLengthForearm;
		/// length of upper arm
		double mLengthUpperArm;
	
		/// Min limits of joint angles
		std::vector<double> mAngleLimitsMin;
		/// Max limits of joint angles
		std::vector<double> mAngleLimitsMax;

		

		/// The distance from the last joint to the Tool Center Point
		float mLastJointToFlange;

		/// The rotations that describe the geometry of the robot. Each joint is located at the 
		/// origin of a coordinate system where the z-axis indicates the direction of the 
		/// joint's rotation axis.
		std::vector<matType> rotations;

		/// The translations from one joint coordinate system to the next. Translations are described
		/// in the orientation of the target joint (i.e. the joint to which the coordinate system is 
		/// translated).
		std::vector<matType> translations;

	public:
		double rad2deg(double rad) { return rad / M_PI * 180; }
		double deg2rad(double deg) { return deg / 180 * M_PI; }

		virtual void ROBOMOVE_API Init(int type, matType flangeToTcp, matType tcpToFlange) = 0;
		
		/// @brief Check whether the joint angles are within their software limits
		bool ROBOMOVE_API CheckLimits(std::vector<float> jointAngles)
		{
			for (unsigned int i = 0; i < jointAngles.size(); i++) 
			{
				if ((jointAngles[i] < mAngleLimitsMin[i]) || (jointAngles[i] > mAngleLimitsMax[i])) 
				{
					std::cout << "joint angle exceeds limit: value = " << jointAngles[i] << ", min/max = " << mAngleLimitsMin[i] << "/" << mAngleLimitsMax[i] << std::endl;
					return false;
				}
 			}
 			return true;
		}
		
		/// @brief Transform a point from the world coordinate system to the position corresponding to the specified joint angles
		///
		/// @param jointAngles joint angles (in rad) of the robot
		/// @param position position of the point with respect to robot base coordinate frame
		/// @return Returns the position of the point with respect to the flange coordinate frame
		vecType ROBOMOVE_API JointAnglesToCartesian(std::vector<float> jointAngles, vecType position = vecType(0, 0, 0))
		{
			matType m = GetCompleteTransformation(jointAngles);
			return position * m;
		}
		
		/// @brief Transform a point from the world coordinate system to the position corresponding to the specified joint angles
		///
		/// @param jointAngles joint angles (in rad) of the robot
		/// @param-interested joint number 
		/// @param position position of the point with respect to robot base coordinate frame
		/// @return Returns the position of the point with respect to the joint requested
		vecType ROBOMOVE_API JointAnglesToCartesianPerJoint(std::vector<float> jointAngles, int jointnum, vecType position = vecType(0, 0, 0))
		{
			matType m = GetCompleteTransformationPerJoint(jointAngles,jointnum);
			return position * m;
		}

				/// @brief Get the matrix that represents the complete transformation of the specified joint angles
		///
		/// @param jointAngles joint angles of the robot
		/// @return Returns the transformation from the origin of the world coordinate system to the 
		///         robot flange (according to specified joint angles).
		matType ROBOMOVE_API GetCompleteTransformation(std::vector<float> jointAngles, bool includeJointToTCP = true)
		{
			matType transform;
			transform.makeIdentity();
			int numJointAngles = jointAngles.size();
			if (numJointAngles == 0) {
				if (includeJointToTCP)
					return mFlangeToTcp * transform;

				return transform;
			} else {
				transform = rotations[0] * transform;
				transform = translations[0] * transform;
				for (std::vector<float>::size_type i = 0; i < jointAngles.size(); i++) {
					transform = RotationMatrixZ(jointAngles[i]) * transform;
					transform = rotations[i+1] * transform;
					transform = translations[i+1] * transform;
				}
			} 

			if (includeJointToTCP)
				return mFlangeToTcp * transform;

			return transform;
		}
	/// @brief Get the matrix that represents the complete transformation of the specified joint angles 
		///
		/// @param jointAngles joint angles of the robot
		/// @param interested joint number
		/// @return Returns the transformation from the origin of the world coordinate system to the 
		///         robot joint mentioned
	    matType ROBOMOVE_API GetCompleteTransformationPerJoint(std::vector<float> jointAngles, int jointnum, bool includeJointToTCP = true)
		{
			matType transform;
			transform.makeIdentity();
			int numJointAngles = jointAngles.size();
			if (numJointAngles == 0) {
				if (includeJointToTCP)
					return mFlangeToTcp * transform;

				return transform;
			} else {
				transform = rotations[0] * transform;
				transform = translations[0] * transform;
				for (std::vector<float>::size_type i = 0; i < jointnum; i++) {
					transform = RotationMatrixZ(jointAngles[i]) * transform;
					transform = rotations[i+1] * transform;
					transform = translations[i+1] * transform;
				}
			} 

			if (includeJointToTCP)
				return mFlangeToTcp * transform;

			return transform;
		}
		
		virtual std::vector<float> ROBOMOVE_API CartesianToJointAngles(vecType& position, vecType& view, vecType& up, int variation1, int variation2, int variation3, int variation4) = 0;

		/// @brief Calculate the multiple different solutions of joint angles (angles in rad)
		virtual std::vector<std::vector<float>> ROBOMOVE_API CartesianToJointAngles(vecType& position, vecType& view, vecType& up)
		{
			std::vector<std::vector<float>> result;
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 2; j++) {
					for (int k = 0; k < 2; k++) {
						for (int l = 0; l < 2; l++) {
							std::vector<float> jointAngles = CartesianToJointAngles(position, view, up, i, j, k, l);
							if ((jointAngles.size() != 0) /*&& CheckLimits(jointAngles)*/) {
								result.push_back(jointAngles);
							}
						}
					}
				}
			}
			return result;
		}

		/// @brief Calculate the multiple different solutions of joint angles (angles in rad)
		virtual std::vector<float> ROBOMOVE_API CartesianToJointAnglesOptimal(vecType& position, vecType& view, vecType& up)
		{
			std::vector<float> result;
			double squaredSum = 0;
			double minSquaredSum = DBL_MAX;
			for (int i = 0; i < 2; i++) {
				for (int j = 0; j < 2; j++) {
					for (int k = 0; k < 2; k++) {
						for (int l = 0; l < 2; l++) {
							std::vector<float> jointAngles = CartesianToJointAngles(position, view, up, i, j, k, l);
							if ((jointAngles.size() != 0) /*&& CheckLimits(jointAngles)*/) {
								double squaredSum = 0;
								for (size_t i = 0; i < jointAngles.size(); i++)
								{
									double w = jointAngles.size() - i;
									squaredSum += jointAngles[i] * jointAngles[i] * w * w;
								}
								if (squaredSum < minSquaredSum)
								{
									minSquaredSum = squaredSum;
									result = jointAngles;
								}
							}
						}
					}
				}
			}
			return result;
		}

		/// @brief Set the flange-to-TCP (and its inverse) transformation
		/// @param flangeToTcp transformation matrix for transformation from flange to TCP
		/// @param flangeToTcp transformation matrix for transformation from TCP to flange
		void ROBOMOVE_API SetFlangeToTCP(matType flangeToTcp, matType tcpToFlange)
		{
			mFlangeToTcp = flangeToTcp;
			mTcpToFlange = tcpToFlange;
		}

		/// @brief Get the flange-to-TCP transformation
		/// @return Returns the transformation matrix of transformation from flange to TCP
		matType ROBOMOVE_API GetFlangeToTCP()
		{
			return mFlangeToTcp;
		}

		/// @brief Get the TCP-to-flange transformation
		/// @return Returns the transformation matrix of transformation from TCP to flange
		matType ROBOMOVE_API GetTCPToFlange()
		{
			return mTcpToFlange;
		}
		
		/// @brief Transform a point from the world coordinate system to the position corresponding to the specified joint angles
		///
		/// @param jointAngles joint angles (in rad) of the robot
		/// @param position position of the point with respect to robot base coordinate frame
		/// @return Returns the position of the point with respect to the flange coordinate frame and the RPY angles
		std::vector<vecType> ROBOMOVE_API JointAnglesToCartesian6D(std::vector<float> jointAngles, vecType position = vecType(0, 0, 0))
		{
			matType m = GetCompleteTransformation(jointAngles);
			vecType coordinates;
			std::vector<vecType> coordinates6D;
			vecType rpy;

			double beta = atan2(-m[2], sqrt(m[0]*m[0] + m[1]*m[1])) * 180 / M_PI;
			double alpha = atan2(m[1] / cos(beta), m[0] / cos(beta)) * 180 / M_PI;
			double gamma = atan2(m[6] / cos(beta), m[10] / cos(beta)) * 180 / M_PI;

			coordinates = position * m;
			double h = coordinates[0];
			coordinates[0] = -coordinates[1];
			coordinates[1] = h;
			coordinates6D.push_back(coordinates);
			rpy[0] = -alpha;
			rpy[1] = gamma;
			rpy[2] = beta;
			coordinates6D.push_back(rpy);

			return coordinates6D;
		}

		/// @brief Create a rotation matrix that represents rotation around the x-axis
		/// @param angle rotation angle in radian
		/// @return Returns the matrix that represents the rotation by the specified angle around the x-axis
		matType ROBOMOVE_API RotationMatrixX(double angle) 
		{
			matType m;
			m.makeRotateX(angle);
			return m;
		}

		/// @brief Create a rotation matrix that represents rotation around the y-axis
		/// @param angle rotation angle in radian
		/// @return Returns the matrix that represents the rotation by the specified angle around the y-axis
		matType ROBOMOVE_API RotationMatrixY(double angle) 
		{
			matType m;
			m.makeRotateY(angle);
			return m;
		}

		/// @brief Create a rotation matrix that represents rotation around the z-axis
		/// @param angle rotation angle in radian
		/// @return Returns the matrix that represents the rotation by the specified angle around the y-axis
		matType ROBOMOVE_API RotationMatrixZ(double angle) 
		{
			matType m;
			m.makeRotateZ(angle);
			return m;
		}

		/// @brief Create a matrix that represents translation
		/// @param dx translation parallel to x-axis
		/// @param dy translation parallel to y-axis
		/// @param dz translation parallel to z-axis
		/// @return Returns the matrix that represents translation by (dx, dy, dz)
		matType ROBOMOVE_API TranslationMatrix(double dx, double dy, double dz) 
		{
			matType m;
			m.makeTranslate(dx, dy, dz);
			return m;
		}

		/// @brief Intersect two circles in 2D. The first circle has to have its center at the origin of the coordinate system.
		///
		/// @param r1 The radius of the first circle which has its center at the origin of the coordinate system
		/// @param cx The x-coordinate of the center point of the second circle
		/// @param cy The y-coordinate of the center point of the second circle
		/// @param r2 The radius of the second circle
		/// @param solutionIndex 0 or 1 depending on whether the first or the second intersection point shall be calculated
		/// @param x output: x-coordinate of the intersection point
		/// @param y output: y-coordinate of the intersection point
		/// @return Returns true if the two circles intersect. Returns false if the circles do not intersect.
		bool ROBOMOVE_API intersectCircles2D(double r1, double cx, double cy, double r2, int solutionIndex, double& x, double& y) 
		{
			// calculate the squared values
			double cx2 = cx*cx;
			double cy2 = cy*cy;
			double r12 = r1*r1;
			double r22 = r2*r2;
			double h = cx2 + cy2 + r12 - r22;
	
			double epsilon = 0.0001;
			if (abs(cy) > epsilon)
			{
				// calculate a, b and c for solving the quadratic equation
				double c = h*h - 4*cy2*r12;
				double b = -4*cx*h;
				double a = 4*cx2 + 4*cy2;

				// check whether a solution exists
				double t = b*b - 4*a*c;
				if (t < 0) {
					// no real (only complex) solutions possible! return false!
					x = 0;
					y = 0;
					return false;
				}

				// determine x by solving the quadratic equation; depending on solutionIndex calculate the first or the second solution
				// calculate one out of two possible solutions
				if (solutionIndex == 0)
				{
					x = (-b + sqrt(t)) / (2*a);
					y = ((cx2 + cy2 + r12 - r22) / 2 - cx*x) / cy;
					return true;
				}
				else
				{
					x = (-b - sqrt(t)) / (2*a);
					y = ((cx2 + cy2 + r12 - r22) / 2 - cx*x) / cy;
					return true;
				}
			}
			else
			{
				if (abs(cx) > 0)
				{
					x = h / (2*cx);
					if (solutionIndex == 0)
					{
						if (r12 - x*x > 0) y = sqrt(r12 - x*x);
						else y = 0;
						return true;
					}
					else
					{
						if (r12 - x*x > 0) y = -sqrt(r12 - x*x);
						else y = 0;
						return true;
					}
				}
				else return false;
			}
		}

		/// @brief Calculate the angle (rad) between the vector from p1 to p2 and the vector from p2 to p3
		///
		/// @param p1 point p1
		/// @param p2 point p2
		/// @param p3 point p3
		/// @param axis The vector that points into the direction of the rotation axis. Required to determine the direction of rotation.
		/// @return Returns the angle between the vector from p1 to p2 and the vector from p2 to p3
		double ROBOMOVE_API angle(vecType p1, vecType p2, vecType p3, vecType axis) {
			vecType v1 = p2 - p1;
			vecType v2 = p3 - p2;

			v1.normalize();
			v2.normalize();

			if (((v1 ^ v2) + axis).length() > ((v2 ^ v1) + axis).length()) {
				return acos((double)(v1 * v2));
			} else {
				return -acos((double)(v1 * v2));
			}
		}

		/// @brief transform position and view- and up-vector of TCP to corresponding point/vectors relative to flange
		/// @param position input: position of TCP, output: position of flange
		/// @param view input: view-vector of TCP, output: corresponding view vector of flange
		/// @param up input: up-vector of TCP, output: corresponding up vector of flange
		void TransformToFlange(vecType& position, vecType& view, vecType& up)
		{
			// calculate left vector
			vecType leftVec = up.crossProduct(view);
			leftVec.normalize();

			// make sure that up vector is orthogonal to view vector and has unit length
			up = view.crossProduct(leftVec);
			up.normalize();

			// make sure the view vector has unit length
			view.normalize();

			// Calculate rotation matrix that rotates TCP orientation to flange orientation (in TCP coordinate frame).
			// Two things are relevant for rotation matrix rot:
			// - For the TCP in the TCP coordinate frame we have: view_tcp = (0, 0, 1), up_tcp = (0, 1, 0)
			// - For the flange in the TCP coordinate frame we have: view_flange = (0, 0, 1) * rot, up_flange = (0, 1, 0) * rot
			matType rot = mTcpToFlange;
			rot[12] = 0;
			rot[13] = 0;
			rot[14] = 0;

			// Calculate the position of the flange in the world coordinate frame 
			// (= [position of TCP in world coordinate frame] + [offset converted from TCP coordinate frame to world coordinate frame])
			vecType transl = vecType(mTcpToFlange[12], mTcpToFlange[13], mTcpToFlange[14]);
			position = position + leftVec * transl[0] + up * transl[1] + view * transl[2];

			// rotation to world coordinate system (i.e. rotation of the orientation of the flange in terms 
			// of left/up/view of the TCP coordinate frame to orientation in terms of x/y/z of the 
			// world coordinate frame)
			matType rotToWorld;
			rotToWorld.makeIdentity();
			rotToWorld[0] = leftVec[0];
			rotToWorld[1] = leftVec[1];
			rotToWorld[2] = leftVec[2];
			rotToWorld[4] = up[0];
			rotToWorld[5] = up[1];
			rotToWorld[6] = up[2];
			rotToWorld[8] = view[0];
			rotToWorld[9] = view[1];
			rotToWorld[10] = view[2];

			// calculate view and up vector with respect to world coordinate frame
			// view vector of TCP in TCP coordinate frame = (0, 0, 1)
			// view vector of flange in TCP coordinate frame = (0, 0, 1) * rot
			// view vector of flange in world coordinate frame = (0, 0, 1) * rot * rotToWorld = (0, 0, 1) * rot
			view = vecType(0, 0, 1) * rot * rotToWorld;
			up = vecType(0, 1, 0) * rot * rotToWorld; 
		}
	};
};