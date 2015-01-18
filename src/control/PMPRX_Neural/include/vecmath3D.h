#pragma once

#define CONVERT_TO_OSG_MATRIX(matrix) (osg::Matrix( \
		matrix[0], matrix[1], matrix[2], matrix[3], \
		matrix[4], matrix[5], matrix[6], matrix[7], \
		matrix[8], matrix[9], matrix[10], matrix[11], \
		matrix[12], matrix[13], matrix[14], matrix[15]))


#define CONVERT_TO_VECMATH_MATRIX(type, matrix) (type( \
		matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3), \
		matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3), \
		matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3), \
		matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3, 3)))

/*
#define CONVERT_TO_VECMATH_MATRIX(type, matrix) (type( \
		matrix(0, 0), matrix(1, 0), matrix(2, 0), matrix(3, 0), \
		matrix(0, 1), matrix(1, 1), matrix(2, 1), matrix(3, 1), \
		matrix(0, 2), matrix(1, 2), matrix(2, 2), matrix(3, 2), \
		matrix(0, 3), matrix(1, 3), matrix(2, 3), matrix(3, 3)))
*/

namespace vecmath3D {

	template<typename T> class Vector {

	private:

		T x;
		T y;
		T z;
		T w;

	public:

		Vector() {
			x = 0;
			y = 0; 
			z = 0; 
			w = 1;
		}

		Vector(T x, T y, T z) {
			this->x = x;
			this->y = y;
			this->z = z;
			w = 1;
		}

		T& operator[](int index) {
			switch(index) {
			case 0: return x;
			case 1: return y;
			case 2: return z;
			default: return w;
			}
		}

		void operator=(Vector<T> v) {
			x = v[0];
			y = v[1];
			z = v[2];
			w = v[3];
		}

		Vector<T> crossProduct(Vector<T> v) {
			Vector<T> result;
			result[0] = y * v[2] - z * v[1];
			result[1] = z * v[0] - x * v[2];
			result[2] = x * v[1] - y * v[0];
			result[3] = 1;
			return result;
		}

		void set(T x, T y, T z) {
			this->x = x;
			this->y = y;
			this->z = z;
		}

		void setX(T x) {
			this->x = x;
		}

		void setY(T y) {
			this->y = y;
		}

		void setZ(T z) {
			this->z = z;
		}

		void setW(T w) {
			this->w = w;
		}

		double getX() {
			return this->x;
		}

		double getY() {
			return this->y;
		}

		double getZ() {
			return this->z;
		}

		T length() {
			return sqrt(x*x+y*y+z*z);
		}

		void normalize() {
			T len = length();
		    x /= len;
			y /= len;
			z /= len;
		}
	};

	template<typename T> class Matrix {

	private:

		std::vector<T> values;

	public:

		Matrix() {
			for (int i = 0; i < 16; i++) {
				values.push_back(0);
			}
		}

		Matrix(T m00, T m01, T m02, T m03, T m10, T m11, T m12, T m13, T m20, T m21, T m22, T m23, T m30, T m31, T m32, T m33) {
			for (int i = 0; i < 16; i++) {
				values.push_back(0);
			}
			set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33);
		}

		void makeIdentity() {
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					if (i == j) values[i * 4 + j] = 1;
					else values[i * 4 + j] = 0;
				}
			}
		}

		void set(T m00, T m01, T m02, T m03, T m10, T m11, T m12, T m13, T m20, T m21, T m22, T m23, T m30, T m31, T m32, T m33) {
			values[0]  = m00; values[1]  = m01; values[2]  = m02; values[3]  = m03;
			values[4]  = m10; values[5]  = m11; values[6]  = m12; values[7]  = m13;
			values[8]  = m20; values[9]  = m21; values[10] = m22; values[11] = m23;
			values[12] = m30; values[13] = m31; values[14] = m32; values[15] = m33;
		}

		T& operator[](int index) {
			return values[index];
		}

		void operator=(Matrix<T> m) {
			values[0]  = m[0]; values[1]  = m[1]; values[2]  = m[2]; values[3]  = m[3];
			values[4]  = m[4]; values[5]  = m[5]; values[6]  = m[6]; values[7]  = m[7];
			values[8]  = m[8]; values[9]  = m[9]; values[10] = m[10]; values[11] = m[11];
			values[12] = m[12]; values[13] = m[13]; values[14] = m[14]; values[15] = m[15];
		}

		void transpose() {
			std::vector<T> buf;
			buf.resize(values.size());
			std::copy(values.begin(), values.end(), buf.begin());
			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++) {
					values[i * 4 + j] = buf[i + 4 * j];
				}
			}
		}

	    void makeRotateX(T angle) {
	        makeIdentity();
			values[5] = cos(angle);
		    values[6] = sin(angle);
		    values[9] = -sin(angle);
		    values[10] = cos(angle);
	    }

		void makeRotateY(T angle) {
	        makeIdentity();
	        values[0] = cos(angle);
		    values[2] = -sin(angle);
		    values[8] = sin(angle);
		    values[10] = cos(angle);

	    }

		void makeRotateZ(T angle) {
	        makeIdentity();
	        values[0] = cos(angle);
		    values[1] = sin(angle);
		    values[4] = -sin(angle);
		    values[5] = cos(angle);

	    }
	   
	    void makeTranslate(T x, T y, T z) {
			makeIdentity();
			values[12] = x;
	        values[13] = y;
		    values[14] = z;
 		}
	};


	template<typename T> Vector<T> operator+(Vector<T> vec1, Vector<T> vec2) {
		Vector<T> v;
		v.setX(vec1[0] + vec2[0]);
		v.setY(vec1[1] + vec2[1]);
		v.setZ(vec1[2] + vec2[2]);
		return v;
	}

	template<typename T> Vector<T> operator-(Vector<T> vec1, Vector<T> vec2) {
		Vector<T> v;
		v.setX(vec1[0] - vec2[0]);
		v.setY(vec1[1] - vec2[1]);
		v.setZ(vec1[2] - vec2[2]);
		return v;
	}

	template<typename T> Matrix<T> operator*(Matrix<T> m1, Matrix<T> m2) {
		Matrix<T> m;
		for (int row1 = 0; row1 < 4; row1++) {
			for (int col1 = 0; col1 < 4; col1++) {

				m[row1 * 4 + col1] = 0;

				for (int i = 0; i < 4; i++) {
					m[row1 * 4 + col1] += m1[row1 * 4 + i] * m2[i * 4 + col1];
				}
			}
		}
		return m;
	}

	template<typename T> Vector<T> operator*(Vector<T> v, Matrix<T> m) {
		Vector<T> result;

		for (int i = 0; i < 4; i++) {
			result[i] = 0;
			for (int row = 0; row < 4; row++) {
				result[i] += m[row * 4 + i] * v[row];
			}
		}

		for (int i = 0; i < 3; i++) {
			result[i] /= result[3];
		}

		return result;
	}

	template<typename T> T operator*(Vector<T> v1, Vector<T> v2) {
		double result = 0;
		for (int i = 0; i < 3; i++) {
			result += v1[i] * v2[i];
		}
		return result;
	}

	template<typename T> Vector<T> operator*(Vector<T> v, T val) {
		Vector<T> result;
		for (int i = 0; i < 3; i++) {
			result[i] = v[i] * val;
		}
		return result;
	}

	template<typename T> Vector<T> operator^(Vector<T> v1, Vector<T> v2) {
		return v1.crossProduct(v2);
	}

	template<typename T> void printVec(T v) {
		std::cout << v[0] << ", " << v[1] << ", " << v[2] << ", " << v[3] << std::endl;
	}

	template<typename T> void printMat(T m) {
		std::cout << m[0]  << ", " << m[1]  << ", " << m[2]  << ", " << m[3]  << std::endl;
		std::cout << m[4]  << ", " << m[5]  << ", " << m[6]  << ", " << m[7]  << std::endl;
		std::cout << m[8]  << ", " << m[9]  << ", " << m[10] << ", " << m[11] << std::endl;
		std::cout << m[12] << ", " << m[13] << ", " << m[14] << ", " << m[15] << std::endl;
	}
}

