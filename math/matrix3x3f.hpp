#pragma once

#include <algorithm> // swap
#include <assert.h>

#include "free_utils.hpp"

// class for operating with 3-by-3 matricies of floats
// assumes row-major ordering:
// 0  1  2
// 3  4  5
// 6  7  8

class Mat3x3F
{
public:
	Mat3x3F()
	{
		for (unsigned char i = 0; i < 9; i++) _v[i] = 0;
		_v[0] = _v[4] = _v[8] = 1.0f; // diagonal!
	}

	Mat3x3F(const float v[9])
	{
		for (unsigned char i = 0; i < 9; i++) _v[i] = v[i];
	}

	static Mat3x3F Zeros()
	{
		float v[9] = { 0,0,0,0,0,0,0,0,0 };
		return Mat3x3F(v);
	}

	static Mat3x3F Ident()
	{
		float v[9] = { 1,0,0,0,1,0,0,0,1 };
		return Mat3x3F(v);
	}

	void Transpose()
	{
		std::swap(_v[1], _v[3]);
		std::swap(_v[2], _v[6]);
		std::swap(_v[5], _v[7]);
	}

	Mat3x3F RetTranspose() //Returns transpose but leaves original matrix unchanged
	{
		Mat3x3F res = *this;
		std::swap(res._v[1], res._v[3]);
		std::swap(res._v[2], res._v[6]);
		std::swap(res._v[5], res._v[7]);

		return res;
	}

	float Trace() {
		float res = _v[0] + _v[4] + _v[8];
		return res;
	}

	Mat3x3F operator*(const Mat3x3F& b)
	{
		float res[9] = { 0,0,0,0,0,0,0,0,0 };
		for (unsigned char r = 0; r < 3; r++)
		{
			for (unsigned char c = 0; c < 3; c++)
			{
				res[r * 3 + c] += _v[r * 3] * b._v[0 * 3 + c] + _v[r * 3 + 1] * b._v[1 * 3 + c] + _v[r * 3 + 2] * b._v[2 * 3 + c];
			}
		}
		return Mat3x3F(res);
	}

	Mat3x3F operator*(const float& b)
	{
		float res[9] = { 0,0,0,0,0,0,0,0,0 };
		for (unsigned char i = 0; i < 9; i++) res[i] = b * _v[i];

		return Mat3x3F(res);
	}

	static Mat3x3F Rotation(point3D vec, float theta)
	{
        point3D tmp = vec.norm();
		vec = vec.norm();
		float c = cos(theta);
		float s = sin(theta);
		float t = (1.0f - c);
		float X = vec[0]; float X2 = X * X;
		float Y = vec[1]; float Y2 = Y * Y;
		float Z = vec[2]; float Z2 = Z * Z;

		float v[9];
		v[0] = t * X2 + c;
		v[1] = t * X * Y + s * Z;
		v[2] = t * X * Z - s * Y;
		v[3] = t * X * Y - s * Z;
		v[4] = t * Y2 + c;
		v[5] = t * Y * Z + s * X;
		v[6] = t * X * Z + s * Y; //Corrected from v[6] = t*X*Y + s*Y; Luzius; 7.10.2010
		v[7] = t * Y * Z - s * X;
		v[8] = t * Z2 + c;

		return Mat3x3F(v);
	}

	static Mat3x3F SkewSymmetric(const float a, const float b, const float c)
	{
		float v[9];
		v[0] = 0.0f;
		v[1] = -c;
		v[2] = b;

		v[3] = c;
		v[4] = 0.0f;
		v[5] = -a;

		v[6] = -b;
		v[7] = a;
		v[8] = 0.0f;

		return Mat3x3F(v);
	}

	static Mat3x3F SkewSymmetric(point3D& w) { return Mat3x3F::SkewSymmetric(w[0], w[1], w[2]); }

	static Mat3x3F OuterProduct(point3D a, point3D b)//a^T*b
	{
		float v[9];
		v[0] = a[0] * b[0];
		v[1] = a[0] * b[1];
		v[2] = a[0] * b[2];

		v[3] = a[1] * b[0];
		v[4] = a[1] * b[1];
		v[5] = a[1] * b[2];

		v[6] = a[2] * b[0];
		v[7] = a[2] * b[1];
		v[8] = a[2] * b[2];

		return Mat3x3F(v);
	}

	point3D operator*(point3D& a) const
	{
        point3D ret;
		ret[0] = a[0] * _v[0] + a[1] * _v[1] + a[2] * _v[2];
		ret[1] = a[0] * _v[3] + a[1] * _v[4] + a[2] * _v[5];
		ret[2] = a[0] * _v[6] + a[1] * _v[7] + a[2] * _v[8];
		return ret;
	}

	Mat3x3F operator+(const Mat3x3F& a)
	{
		Mat3x3F ret;
		for (unsigned char i = 0; i < 9; i++)
		{
			ret._v[i] = _v[i] + a._v[i];
		}
		return ret;
	}


	Mat3x3F operator-(const Mat3x3F& a)
	{
		Mat3x3F ret;
		for (unsigned char i = 0; i < 9; i++)
		{
			ret._v[i] = _v[i] - a._v[i];
		}
		return ret;
	}

	Mat3x3F operator/(const float a)
	{
		Mat3x3F ret;
		for (unsigned char i = 0; i < 9; i++)
		{
			ret._v[i] = _v[i] / a;
		}
		return ret;
	}

	float& operator()(unsigned char r, unsigned char c)
	{
		assert(r < 3 && c < 3);
		return _v[r * 3 + c];
	}

	float& operator[](int i)
	{
		assert(i >= 0 && i < 9);
		return _v[i];
	}

	const float& operator[](int i) const
	{
		assert(i >= 0 && i < 9);
		return _v[i];
	}

	void SetValues(float v[9])
	{
		for (unsigned char i = 0; i < 9; i++) _v[i] = v[i];
	}

	void SetZero()
	{
		for (unsigned char i = 0; i < 9; i++) _v[i] = 0;
	}

	Mat3x3F Pointdot(const Mat3x3F& b)
	{ //multiply matrices elementwise (Matlab: A.*B)
		float res[9] = { 0,0,0,0,0,0,0,0,0 };
		for (int i = 0; i < 9; i++) {
			res[i] = _v[i] * b._v[i];
		}
		return Mat3x3F(res);
	}
    
	//cross product of vector with each column of a matrix, returns a matrix (vec x Mat)
	// side = 0 <-> vec x Mat
	// side = 1 <-> Mat x vec
	Mat3x3F Cross(point3D vec, int side)
	{
        point3D tmp1({_v[0], _v[3], _v[6]});
        point3D tmp2({_v[1], _v[4], _v[7]});
        point3D tmp3({_v[2], _v[5], _v[8]});
		float v[9];
		if (side == 0) {
			tmp1 = vec.cross(tmp1);
			tmp2 = vec.cross(tmp2);
			tmp3 = vec.cross(tmp3);
		}
		else if (side == 1) {
			tmp1 = tmp1.cross(vec);
			tmp2 = tmp1.cross(vec);
			tmp3 = tmp1.cross(vec);
		}
		tmp1.get(v[0], v[3], v[6]);
		tmp2.get(v[1], v[4], v[7]);
		tmp3.get(v[2], v[5], v[8]);
		return Mat3x3F(v);
	}

	float Determinant()
	{
		float res = 0;
		res += _v[0] * (_v[4] * _v[8] - _v[7] * _v[5])
			- _v[3] * (_v[1] * _v[8] - _v[7] * _v[2])
			+ _v[6] * (_v[1] * _v[5] - _v[4] * _v[2]);

		return res;
	}

	Mat3x3F Inverse(void)
	{
		Mat3x3F res;
		float  det = this->Determinant();
		res[0] = (_v[4] * _v[8] - _v[7] * _v[5]) / det;
		res[1] = -(_v[1] * _v[8] - _v[7] * _v[2]) / det;
		res[2] = (_v[1] * _v[5] - _v[4] * _v[2]) / det;

		res[3] = -(_v[3] * _v[8] - _v[6] * _v[5]) / det;
		res[4] = (_v[0] * _v[8] - _v[6] * _v[2]) / det;
		res[5] = -(_v[0] * _v[5] - _v[3] * _v[2]) / det;

		res[6] = (_v[3] * _v[7] - _v[6] * _v[4]) / det;
		res[7] = -(_v[0] * _v[7] - _v[6] * _v[1]) / det;
		res[8] = (_v[0] * _v[4] - _v[3] * _v[1]) / det;

		return res;

	}


	// 0 1 2
	// 3 4 5
	// 6 7 8


protected:
	float _v[9];
};
