#pragma once

#include <vector>
#include <iostream>

#include <../include/vecmath3D.h>
#include <../include/Kinematics.h>

#define TX90 0
#define TX90L 1
#define TX90XL 2

namespace LibRoboMove
{

	/// Class SimpleKinematics provides very simple functionality for (inverse) robot kinematics.
	///
	class KinematicsTX90 : public Kinematics
	{
	

	private:

		/// @brief Initialization (called internally by constructor)
		virtual void Init(int type, matType flangeToTcp, matType tcpToFlange)
		{
			mFlangeToTcp = flangeToTcp;
			mTcpToFlange = tcpToFlange;

			mLastJointToFlange = 100;

			switch(type) {
				case TX90: {
					mLengthForearm = 425;
					mLengthUpperArm = 425;
					break;
				}
				case TX90L: {
					mLengthForearm = 550;
					mLengthUpperArm = 500;
					break;
				}
				case TX90XL: {
					mLengthForearm = 650;
					mLengthUpperArm = 650;
					break;
				}
			}
	
			// For now hardcoded for Staubli TX90

			// translation is relative to origin of previous joint, but AFTER rotation!!!

			rotations.push_back(RotationMatrixX(0));
			translations.push_back(TranslationMatrix(0, 0, 0));

			rotations.push_back(RotationMatrixX(-M_PI / 2));
			translations.push_back(TranslationMatrix(50, -478, 50));
	
			rotations.push_back(RotationMatrixZ(0));
			translations.push_back(TranslationMatrix(0, -mLengthUpperArm, 0));

			rotations.push_back(RotationMatrixX(M_PI / 2));
			translations.push_back(TranslationMatrix(0, 0, 0));

			rotations.push_back(RotationMatrixX(-M_PI / 2));
			translations.push_back(TranslationMatrix(0, -mLengthForearm, 0));

			rotations.push_back(RotationMatrixX(M_PI / 2));
			translations.push_back(TranslationMatrix(0, 0, mLastJointToFlange));

			rotations.push_back(RotationMatrixZ(0));
			translations.push_back(TranslationMatrix(0, 0, 0));

			// Set the joint angle limits for TX90
			mAngleLimitsMin.push_back(deg2rad(-160.0));
			mAngleLimitsMax.push_back(deg2rad( 160.0));
			mAngleLimitsMin.push_back(deg2rad(-127));
			mAngleLimitsMax.push_back(deg2rad( 127));
			mAngleLimitsMin.push_back(deg2rad(-140));
			mAngleLimitsMax.push_back(deg2rad( 140));
			mAngleLimitsMin.push_back(deg2rad(-270.0));
			mAngleLimitsMax.push_back(deg2rad( 270.0));
			mAngleLimitsMin.push_back(deg2rad(-115.0));
			mAngleLimitsMax.push_back(deg2rad( 130.0));
			mAngleLimitsMin.push_back(deg2rad(-270.0));
			mAngleLimitsMax.push_back(deg2rad( 270.0));
		}

	public:
		/// @brief Constructor
		///
		/// @param type one out of the following constants: TX90, TX90L, TX90XL
		ROBOMOVE_API KinematicsTX90(int type = TX90L, float flangeToTCP_x = 0, float flangeToTCP_y = 0, float flangeToTCP_z = 0)
		{
			matType m;
			m.makeTranslate(flangeToTCP_x, flangeToTCP_y, flangeToTCP_z);
			matType mInv;
			mInv.makeTranslate(-flangeToTCP_x, -flangeToTCP_y, -flangeToTCP_z);
			Init(type, m, mInv);
		}
		ROBOMOVE_API KinematicsTX90(int type, matType flangeToTcp, matType tcpToFlange)
		{
			Init(type, flangeToTcp, tcpToFlange);
		}

		/// @brief Destructor
		ROBOMOVE_API ~KinematicsTX90(void)
		{
		}

		/// @brief Convert from cartesian coordinates to joint angles
		///
		/// @param position position of the robot flange
		/// @param view direction of the z-axis after transformation
		/// @param up direction of the y-axis after transformation
		/// @return Returns the joint angles (in rad) that correspond to the specified cartesian coordinates
		virtual std::vector<float> ROBOMOVE_API CartesianToJointAngles(vecType& position, vecType& view, vecType& up, int variation1, int variation2, int variation3, int variation4)
		{
			// copy vector
			vecType positionVec(position[0], position[1], position[2]);
			vecType viewVec(view[0], view[1], view[2]);
			vecType upVec(up[0], up[1], up[2]);

			// normalize view and up vector
			viewVec.normalize();
			upVec.normalize();

			// transform position/view/up from TCP to flange
			TransformToFlange(positionVec, viewVec, upVec);

			// initialize the vector that keeps the 6 joint angles
			std::vector<float> joints;
			joints.resize(6);

			// view has to be normalized!!!
			vecType q = positionVec - (viewVec * (double)mLastJointToFlange);
	
			double shiftX = 50;
			double shiftY = 50;
			double mx = q[0] / 2;
			double my = q[1] / 2;
	
			double tx;
			double ty;
			double r2 = sqrt(mx*mx + my*my);
			bool solutionExists1 = intersectCircles2D(abs(shiftY), mx, my, r2, variation1, tx, ty);
			if (!solutionExists1)
			{
				joints.clear();
				return joints;
			}
			double t_len = sqrt(tx*tx + ty*ty);

			// the points position, q, r, s, and t are coplanar (n is the normal vector of this plane)
			vecType n(tx / t_len, ty/ t_len, 0);

			// calculate v
			vecType v(-n[1], n[0], 0);

			vecType s(tx - v[0] * shiftX, ty - v[1] * shiftX, 478);

			// calculate position of R in v/z plane
			double q_coord_v = 0;
			if (abs(v[0]) > abs(v[1]))
				q_coord_v = (q[0] - s[0]) / v[0];
			else
				q_coord_v = (q[1] - s[1]) / v[1];
	
			double q_coord_z = q[2] - s[2];
			double rv = 0;
			double rz = 0;
			bool solutionExists2 = intersectCircles2D(mLengthUpperArm, q_coord_v, q_coord_z, mLengthForearm, variation2, rv, rz);
			if (!solutionExists2)
			{
				joints.clear();
				return joints;
			}

			// calculate position of R in x/y/z
			vecType r(s[0] + rv * v[0], s[1] + rv * v[1], rz + s[2]);

			// ==================================================================
			// now that we know all joint positions: determine the joint angles!
			// ==================================================================

			joints[0] = (float)acos(n[1]);
			if (tx > 0) joints[0] *= -1;

			double cosJoint1 = rz / sqrt(rv*rv + rz*rz);
			if (cosJoint1 > 1) cosJoint1 = 1;
			if (cosJoint1 < -1) cosJoint1 = -1;
			joints[1] = (float)acos(cosJoint1);
			if (rv > 0) joints[1] *= -1;

			joints[2] = (float)angle(s, r, q, n);	

			vecType axis = (r - q) ^ (positionVec - q);
			axis.normalize();
			double alpha = angle(r, q, positionVec, axis);

			vecType rq = q - r;
			rq.normalize();
			vecType qp = positionVec - q;
			vecType pProjected = q + rq * cos(alpha) * qp.length();
			vecType vec = positionVec - pProjected;
			vec.normalize();
			vecType compare = rq ^ n;
			compare.normalize();
			double cosVal = compare * vec;
			if (cosVal > 1) cosVal = 1;
			if (cosVal < -1) cosVal = -1;
			joints[3] = (float)acos(cosVal);
			if (((compare ^ vec) + rq).length() < ((vec ^ compare) + rq).length()) joints[3] *= -1;
			joints[4] = (float)alpha;
			if (variation3 == 1)
			{
				if (joints[3] < 0)
					joints[3] += (float)M_PI;
				else
					joints[3] -= (float)M_PI;

				joints[4] *= -1;
			}
	
	
			joints[5] = 0;

			// calculate the up-vector if the last joint angle would be zero (currUp)
			matType m = GetCompleteTransformation(joints, false);
			vecType p1(0, 0, 0);
			vecType p2(0, 1, 0);
			p1 = p1 * m;
			p2 = p2 * m;
			vecType currUp = p2 - p1;
			currUp.normalize();

			// determine the up vector that is closest to the specified up-vector but perpendicular to the view vector
			vecType right = upVec.crossProduct(viewVec);
			upVec = viewVec.crossProduct(right);
			upVec.normalize();

			// the last joint angle is the angle between currUp and upVec
			double cos = currUp * upVec;
			if (cos > 1) cos = 1;
			if (cos < -1) cos = -1;
			joints[5] = (float)acos(cos);
			vecType vec1 = currUp ^ upVec;
			vecType vec2 = upVec ^ currUp;
			if ((vec1 + viewVec).length() < (vec2 + viewVec).length()) joints[5] *= -1;

			if (variation4 == 1)
			{
				if (joints[5] > 0) joints[5] -= 2*(float)M_PI;
				else joints[5] += 2*(float)M_PI;
			}

			return joints;
		}
	};
};
