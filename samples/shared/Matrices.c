///////////////////////////////////////////////////////////////////////////////
// Matrice.cpp
// ===========
// NxN Matrix Math classes
//
// The elements of the matrix are stored as column major order.
// | 0 2 |    | 0 3 6 |    |  0  4  8 12 |
// | 1 3 |    | 1 4 7 |    |  1  5  9 13 |
//            | 2 5 8 |    |  2  6 10 14 |
//                         |  3  7 11 15 |
//
//  AUTHOR: Song Ho Ahn (song.ahn@gmail.com)
// CREATED: 2005-06-24
// UPDATED: 2014-09-21
//
// Copyright (C) 2005 Song Ho Ahn
///////////////////////////////////////////////////////////////////////////////

#include <math.h>
//#include <algorithm>
#include "Matrices.h"

const float DEG2RAD = 3.141593f / 180;
const float EPSILON = 0.00001f;



///////////////////////////////////////////////////////////////////////////////
// transpose 2x2 matrix
///////////////////////////////////////////////////////////////////////////////
//Matrix2& Matrix2::transpose()
//{
//    std::swap(m[1],  m[2]);
//    return *this;
//}
//
//
//
/////////////////////////////////////////////////////////////////////////////////
//// return the determinant of 2x2 matrix
/////////////////////////////////////////////////////////////////////////////////
//float Matrix2::getDeterminant()
//{
//    return m[0] * m[3] - m[1] * m[2];
//}
//
//
//
/////////////////////////////////////////////////////////////////////////////////
//// inverse of 2x2 matrix
//// If cannot find inverse, set identity matrix
/////////////////////////////////////////////////////////////////////////////////
//Matrix2& Matrix2::invert()
//{
//    float determinant = getDeterminant();
//    if(fabs(determinant) <= EPSILON)
//    {
//        return identity();
//    }
//
//    float tmp = m[0];   // copy the first element
//    float invDeterminant = 1.0f / determinant;
//    m[0] =  invDeterminant * m[3];
//    m[1] = -invDeterminant * m[1];
//    m[2] = -invDeterminant * m[2];
//    m[3] =  invDeterminant * tmp;
//
//    return *this;
//}



///////////////////////////////////////////////////////////////////////////////
// transpose 3x3 matrix
///////////////////////////////////////////////////////////////////////////////
//Matrix3& Matrix3::transpose()
//{
//    std::swap(m[1],  m[3]);
//    std::swap(m[2],  m[6]);
//    std::swap(m[5],  m[7]);
//
//    return *this;
//}
//
//
//
/////////////////////////////////////////////////////////////////////////////////
//// return determinant of 3x3 matrix
/////////////////////////////////////////////////////////////////////////////////
//float Matrix3::getDeterminant()
//{
//    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
//           m[1] * (m[3] * m[8] - m[5] * m[6]) +
//           m[2] * (m[3] * m[7] - m[4] * m[6]);
//}
//
//
//


///////////////////////////////////////////////////////////////////////////////
// transpose 4x4 matrix
///////////////////////////////////////////////////////////////////////////////
//Matrix4& Matrix4::transpose()
//{
//    std::swap(m[1],  m[4]);
//    std::swap(m[2],  m[8]);
//    std::swap(m[3],  m[12]);
//    std::swap(m[6],  m[9]);
//    std::swap(m[7],  m[13]);
//    std::swap(m[11], m[14]);
//
//    return *this;
//}
//
//
//
/////////////////////////////////////////////////////////////////////////////////
//// compute the inverse of 4x4 Euclidean transformation matrix
////
//// Euclidean transformation is translation, rotation, and reflection.
//// With Euclidean transform, only the position and orientation of the object
//// will be changed. Euclidean transform does not change the shape of an object
//// (no scaling). Length and angle are reserved.
////
//// Use inverseAffine() if the matrix has scale and shear transformation.
////
//// M = [ R | T ]
////     [ --+-- ]    (R denotes 3x3 rotation/reflection matrix)
////     [ 0 | 1 ]    (T denotes 1x3 translation matrix)
////
//// y = M*x  ->  y = R*x + T  ->  x = R^-1*(y - T)  ->  x = R^T*y - R^T*T
//// (R is orthogonal,  R^-1 = R^T)
////
////  [ R | T ]-1    [ R^T | -R^T * T ]    (R denotes 3x3 rotation matrix)
////  [ --+-- ]   =  [ ----+--------- ]    (T denotes 1x3 translation)
////  [ 0 | 1 ]      [  0  |     1    ]    (R^T denotes R-transpose)
/////////////////////////////////////////////////////////////////////////////////
//Matrix4& Matrix4::invertEuclidean()
//{
//    // transpose 3x3 rotation matrix part
//    // | R^T | 0 |
//    // | ----+-- |
//    // |  0  | 1 |
//    float tmp;
//    tmp = m[1];  m[1] = m[4];  m[4] = tmp;
//    tmp = m[2];  m[2] = m[8];  m[8] = tmp;
//    tmp = m[6];  m[6] = m[9];  m[9] = tmp;
//
//    // compute translation part -R^T * T
//    // | 0 | -R^T x |
//    // | --+------- |
//    // | 0 |   0    |
//    float x = m[12];
//    float y = m[13];
//    float z = m[14];
//    m[12] = -(m[0] * x + m[4] * y + m[8] * z);
//    m[13] = -(m[1] * x + m[5] * y + m[9] * z);
//    m[14] = -(m[2] * x + m[6] * y + m[10]* z);
//
//    // last row should be unchanged (0,0,0,1)
//
//    return *this;
//}
//
//
//
/////////////////////////////////////////////////////////////////////////////////
//// inverse matrix using matrix partitioning (blockwise inverse)
//// It devides a 4x4 matrix into 4 of 2x2 matrices. It works in case of where
//// det(A) != 0. If not, use the generic inverse method
//// inverse formula.
//// M = [ A | B ]    A, B, C, D are 2x2 matrix blocks
////     [ --+-- ]    det(M) = |A| * |D - ((C * A^-1) * B)|
////     [ C | D ]
////
//// M^-1 = [ A' | B' ]   A' = A^-1 - (A^-1 * B) * C'
////        [ ---+--- ]   B' = (A^-1 * B) * -D'
////        [ C' | D' ]   C' = -D' * (C * A^-1)
////                      D' = (D - ((C * A^-1) * B))^-1
////
//// NOTE: I wrap with () if it it used more than once.
////       The matrix is invertable even if det(A)=0, so must check det(A) before
////       calling this function, and use invertGeneric() instead.
/////////////////////////////////////////////////////////////////////////////////
//Matrix4& Matrix4::invertProjective()
//{
//    // partition
//    Matrix2 a(m[0], m[1], m[4], m[5]);
//    Matrix2 b(m[8], m[9], m[12], m[13]);
//    Matrix2 c(m[2], m[3], m[6], m[7]);
//    Matrix2 d(m[10], m[11], m[14], m[15]);
//
//    // pre-compute repeated parts
//    a.invert();             // A^-1
//    Matrix2 ab = a * b;     // A^-1 * B
//    Matrix2 ca = c * a;     // C * A^-1
//    Matrix2 cab = ca * b;   // C * A^-1 * B
//    Matrix2 dcab = d - cab; // D - C * A^-1 * B
//
//    // check determinant if |D - C * A^-1 * B| = 0
//    //NOTE: this function assumes det(A) is already checked. if |A|=0 then,
//    //      cannot use this function.
//    float determinant = dcab[0] * dcab[3] - dcab[1] * dcab[2];
//    if(fabs(determinant) <= EPSILON)
//    {
//        return identity();
//    }
//
//    // compute D' and -D'
//    Matrix2 d1 = dcab;      //  (D - C * A^-1 * B)
//    d1.invert();            //  (D - C * A^-1 * B)^-1
//    Matrix2 d2 = -d1;       // -(D - C * A^-1 * B)^-1
//
//    // compute C'
//    Matrix2 c1 = d2 * ca;   // -D' * (C * A^-1)
//
//    // compute B'
//    Matrix2 b1 = ab * d2;   // (A^-1 * B) * -D'
//
//    // compute A'
//    Matrix2 a1 = a - (ab * c1); // A^-1 - (A^-1 * B) * C'
//
//    // assemble inverse matrix
//    m[0] = a1[0];  m[4] = a1[2]; /*|*/ m[8] = b1[0];  m[12]= b1[2];
//    m[1] = a1[1];  m[5] = a1[3]; /*|*/ m[9] = b1[1];  m[13]= b1[3];
//    /*-----------------------------+-----------------------------*/
//    m[2] = c1[0];  m[6] = c1[2]; /*|*/ m[10]= d1[0];  m[14]= d1[2];
//    m[3] = c1[1];  m[7] = c1[3]; /*|*/ m[11]= d1[1];  m[15]= d1[3];
//
//    return *this;
//}
//
//
//
/////////////////////////////////////////////////////////////////////////////////
//// return determinant of 4x4 matrix
/////////////////////////////////////////////////////////////////////////////////
//float Matrix4::getDeterminant()
//{
//    return m[0] * getCofactor(m[5],m[6],m[7], m[9],m[10],m[11], m[13],m[14],m[15]) -
//           m[1] * getCofactor(m[4],m[6],m[7], m[8],m[10],m[11], m[12],m[14],m[15]) +
//           m[2] * getCofactor(m[4],m[5],m[7], m[8],m[9], m[11], m[12],m[13],m[15]) -
//           m[3] * getCofactor(m[4],m[5],m[6], m[8],m[9], m[10], m[12],m[13],m[14]);
//}
//
//
//
///////////////////////////////////////////////////////////////////////////////
// compute cofactor of 3x3 minor matrix without sign
// input params are 9 elements of the minor matrix
// NOTE: The caller must know its sign.
///////////////////////////////////////////////////////////////////////////////
static float getCofactor(float m0, float m1, float m2,
	float m3, float m4, float m5,
	float m6, float m7, float m8)
{
	return m0 * (m4 * m8 - m5 * m7) -
		m1 * (m3 * m8 - m5 * m6) +
		m2 * (m3 * m7 - m4 * m6);
}



///////////////////////////////////////////////////////////////////////////////
// translate this matrix by (x, y, z)
///////////////////////////////////////////////////////////////////////////////
void Matrix4_translate_v(Matrix4* mx, const Vector3* v)
{
   Matrix4_translate_xyz(mx, v->x, v->y, v->z);
}

void Matrix4_translate_xyz(Matrix4* mx, float x, float y, float z)
{
   mx->m[0] += mx->m[3] * x;   mx->m[4] += mx->m[7] * x;   mx->m[8] += mx->m[11]* x;   mx->m[12]+= mx->m[15]* x;
   mx->m[1] += mx->m[3] * y;   mx->m[5] += mx->m[7] * y;   mx->m[9] += mx->m[11]* y;   mx->m[13]+= mx->m[15]* y;
   mx->m[2] += mx->m[3] * z;   mx->m[6] += mx->m[7] * z;   mx->m[10]+= mx->m[11]* z;   mx->m[14]+= mx->m[15]* z;
}



///////////////////////////////////////////////////////////////////////////////
// uniform scale
///////////////////////////////////////////////////////////////////////////////
void Matrix4_scale_s(Matrix4* mx, float s)

{
   Matrix4_scale_xyz(mx, s, s, s);
}

void Matrix4_scale_xyz(Matrix4* mx, float x, float y, float z)
{
   mx->m[0] *= x;   mx->m[4] *= x;   mx->m[8] *= x;   mx->m[12] *= x;
   mx->m[1] *= y;   mx->m[5] *= y;   mx->m[9] *= y;   mx->m[13] *= y;
   mx->m[2] *= z;   mx->m[6] *= z;   mx->m[10]*= z;   mx->m[14] *= z;
}



/////////////////////////////////////////////////////////////////////////////////
//// build a rotation matrix with given angle(degree) and rotation axis, then
//// multiply it with this object
/////////////////////////////////////////////////////////////////////////////////
//Matrix4& Matrix4::rotate(float angle, const Vector3& axis)
//{
//    return rotate(angle, axis.x, axis.y, axis.z);
//}
//
//Matrix4& Matrix4::rotate(float angle, float x, float y, float z)
//{
//    float c = cosf(angle * DEG2RAD);    // cosine
//    float s = sinf(angle * DEG2RAD);    // sine
//    float c1 = 1.0f - c;                // 1 - c
//    float m0 = m[0],  m4 = m[4],  m8 = m[8],  m12= m[12],
//          m1 = m[1],  m5 = m[5],  m9 = m[9],  m13= m[13],
//          m2 = m[2],  m6 = m[6],  m10= m[10], m14= m[14];
//
//    // build rotation matrix
//    float r0 = x * x * c1 + c;
//    float r1 = x * y * c1 + z * s;
//    float r2 = x * z * c1 - y * s;
//    float r4 = x * y * c1 - z * s;
//    float r5 = y * y * c1 + c;
//    float r6 = y * z * c1 + x * s;
//    float r8 = x * z * c1 + y * s;
//    float r9 = y * z * c1 - x * s;
//    float r10= z * z * c1 + c;
//
//    // multiply rotation matrix
//    m[0] = r0 * m0 + r4 * m1 + r8 * m2;
//    m[1] = r1 * m0 + r5 * m1 + r9 * m2;
//    m[2] = r2 * m0 + r6 * m1 + r10* m2;
//    m[4] = r0 * m4 + r4 * m5 + r8 * m6;
//    m[5] = r1 * m4 + r5 * m5 + r9 * m6;
//    m[6] = r2 * m4 + r6 * m5 + r10* m6;
//    m[8] = r0 * m8 + r4 * m9 + r8 * m10;
//    m[9] = r1 * m8 + r5 * m9 + r9 * m10;
//    m[10]= r2 * m8 + r6 * m9 + r10* m10;
//    m[12]= r0 * m12+ r4 * m13+ r8 * m14;
//    m[13]= r1 * m12+ r5 * m13+ r9 * m14;
//    m[14]= r2 * m12+ r6 * m13+ r10* m14;
//
//    return *this;
//}
//
//Matrix4& Matrix4::rotateX(float angle)
//{
//    float c = cosf(angle * DEG2RAD);
//    float s = sinf(angle * DEG2RAD);
//    float m1 = m[1],  m2 = m[2],
//          m5 = m[5],  m6 = m[6],
//          m9 = m[9],  m10= m[10],
//          m13= m[13], m14= m[14];
//
//    m[1] = m1 * c + m2 *-s;
//    m[2] = m1 * s + m2 * c;
//    m[5] = m5 * c + m6 *-s;
//    m[6] = m5 * s + m6 * c;
//    m[9] = m9 * c + m10*-s;
//    m[10]= m9 * s + m10* c;
//    m[13]= m13* c + m14*-s;
//    m[14]= m13* s + m14* c;
//
//    return *this;
//}
//
//Matrix4& Matrix4::rotateY(float angle)
//{
//    float c = cosf(angle * DEG2RAD);
//    float s = sinf(angle * DEG2RAD);
//    float m0 = m[0],  m2 = m[2],
//          m4 = m[4],  m6 = m[6],
//          m8 = m[8],  m10= m[10],
//          m12= m[12], m14= m[14];
//
//    m[0] = m0 * c + m2 * s;
//    m[2] = m0 *-s + m2 * c;
//    m[4] = m4 * c + m6 * s;
//    m[6] = m4 *-s + m6 * c;
//    m[8] = m8 * c + m10* s;
//    m[10]= m8 *-s + m10* c;
//    m[12]= m12* c + m14* s;
//    m[14]= m12*-s + m14* c;
//
//    return *this;
//}
//
//Matrix4& Matrix4::rotateZ(float angle)
//{
//    float c = cosf(angle * DEG2RAD);
//    float s = sinf(angle * DEG2RAD);
//    float m0 = m[0],  m1 = m[1],
//          m4 = m[4],  m5 = m[5],
//          m8 = m[8],  m9 = m[9],
//          m12= m[12], m13= m[13];
//
//    m[0] = m0 * c + m1 *-s;
//    m[1] = m0 * s + m1 * c;
//    m[4] = m4 * c + m5 *-s;
//    m[5] = m4 * s + m5 * c;
//    m[8] = m8 * c + m9 *-s;
//    m[9] = m8 * s + m9 * c;
//    m[12]= m12* c + m13*-s;
//    m[13]= m12* s + m13* c;
//
//    return *this;
//}

Matrix3 Matrix3_identity()
{
	Matrix3 mx = {
		.m = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f,
		}
	};
	return mx;
}

///////////////////////////////////////////////////////////////////////////////
// inverse 3x3 matrix
// If cannot find inverse, set identity matrix
///////////////////////////////////////////////////////////////////////////////
void Matrix3_invert(Matrix3 *mx)
{
	float determinant, invDeterminant;
	float tmp[9];

	tmp[0] = mx->m[4] * mx->m[8] - mx->m[5] * mx->m[7];
	tmp[1] = mx->m[2] * mx->m[7] - mx->m[1] * mx->m[8];
	tmp[2] = mx->m[1] * mx->m[5] - mx->m[2] * mx->m[4];
	tmp[3] = mx->m[5] * mx->m[6] - mx->m[3] * mx->m[8];
	tmp[4] = mx->m[0] * mx->m[8] - mx->m[2] * mx->m[6];
	tmp[5] = mx->m[2] * mx->m[3] - mx->m[0] * mx->m[5];
	tmp[6] = mx->m[3] * mx->m[7] - mx->m[4] * mx->m[6];
	tmp[7] = mx->m[1] * mx->m[6] - mx->m[0] * mx->m[7];
	tmp[8] = mx->m[0] * mx->m[4] - mx->m[1] * mx->m[3];

	// check determinant if it is 0
	determinant = mx->m[0] * tmp[0] + mx->m[1] * tmp[3] + mx->m[2] * tmp[6];
	if (fabs(determinant) <= EPSILON)
	{
		*mx = Matrix3_identity(); // cannot inverse, make it identity matrix
		return;
	}

	// divide by the determinant
	invDeterminant = 1.0f / determinant;
	mx->m[0] = invDeterminant * tmp[0];
	mx->m[1] = invDeterminant * tmp[1];
	mx->m[2] = invDeterminant * tmp[2];
	mx->m[3] = invDeterminant * tmp[3];
	mx->m[4] = invDeterminant * tmp[4];
	mx->m[5] = invDeterminant * tmp[5];
	mx->m[6] = invDeterminant * tmp[6];
	mx->m[7] = invDeterminant * tmp[7];
	mx->m[8] = invDeterminant * tmp[8];
}

Matrix4 Matrix4_identity()
{
	Matrix4 mx = {
		.m = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		}
	};
	return mx;
}

///////////////////////////////////////////////////////////////////////////////
// inverse 4x4 matrix
///////////////////////////////////////////////////////////////////////////////
void Matrix4_invert(Matrix4 *mx)
{
	// If the 4th row is [0,0,0,1] then it is affine matrix and
	// it has no projective transformation.
	if (mx->m[3] == 0 && mx->m[7] == 0 && mx->m[11] == 0 && mx->m[15] == 1)
		Matrix4_invertAffine(mx);
	else
	{
		Matrix4_invertGeneral(mx);
		/*@@ invertProjective() is not optimized (slower than generic one)
		if(fabs(m[0]*m[5] - m[1]*m[4]) > EPSILON)
			this->invertProjective();   // inverse using matrix partition
		else
			this->invertGeneral();      // generalized inverse
		*/
	}
}

///////////////////////////////////////////////////////////////////////////////
// compute the inverse of a 4x4 affine transformation matrix
//
// Affine transformations are generalizations of Euclidean transformations.
// Affine transformation includes translation, rotation, reflection, scaling,
// and shearing. Length and angle are NOT preserved.
// M = [ R | T ]
//     [ --+-- ]    (R denotes 3x3 rotation/scale/shear matrix)
//     [ 0 | 1 ]    (T denotes 1x3 translation matrix)
//
// y = M*x  ->  y = R*x + T  ->  x = R^-1*(y - T)  ->  x = R^-1*y - R^-1*T
//
//  [ R | T ]-1   [ R^-1 | -R^-1 * T ]
//  [ --+-- ]   = [ -----+---------- ]
//  [ 0 | 1 ]     [  0   +     1     ]
///////////////////////////////////////////////////////////////////////////////
void Matrix4_invertAffine(Matrix4 *mx)
{
	// R^-1
	Matrix3 r = {
		.m = {
			mx->m[0], mx->m[1], mx->m[2], mx->m[4], mx->m[5], mx->m[6], mx->m[8], mx->m[9], mx->m[10]
		}
	};
	Matrix3_invert(&r);
	mx->m[0] = r.m[0];  mx->m[1] = r.m[1];  mx->m[2] = r.m[2];
	mx->m[4] = r.m[3];  mx->m[5] = r.m[4];  mx->m[6] = r.m[5];
	mx->m[8] = r.m[6];  mx->m[9] = r.m[7];  mx->m[10] = r.m[8];

	// -R^-1 * T
	float x = mx->m[12];
	float y = mx->m[13];
	float z = mx->m[14];
	mx->m[12] = -(r.m[0] * x + r.m[3] * y + r.m[6] * z);
	mx->m[13] = -(r.m[1] * x + r.m[4] * y + r.m[7] * z);
	mx->m[14] = -(r.m[2] * x + r.m[5] * y + r.m[8] * z);

	// last row should be unchanged (0,0,0,1)
	//mx->m[3] = mx->m[7] = mx->m[11] = 0.0f;
	//mx->m[15] = 1.0f;
}

///////////////////////////////////////////////////////////////////////////////
// compute the inverse of a general 4x4 matrix using Cramer's Rule
// If cannot find inverse, return indentity matrix
// M^-1 = adj(M) / det(M)
///////////////////////////////////////////////////////////////////////////////
void Matrix4_invertGeneral(Matrix4 *mx)
{
	// get cofactors of minor matrices
	float cofactor0 = getCofactor(mx->m[5], mx->m[6], mx->m[7], mx->m[9], mx->m[10], mx->m[11], mx->m[13], mx->m[14], mx->m[15]);
	float cofactor1 = getCofactor(mx->m[4], mx->m[6], mx->m[7], mx->m[8], mx->m[10], mx->m[11], mx->m[12], mx->m[14], mx->m[15]);
	float cofactor2 = getCofactor(mx->m[4], mx->m[5], mx->m[7], mx->m[8], mx->m[9], mx->m[11], mx->m[12], mx->m[13], mx->m[15]);
	float cofactor3 = getCofactor(mx->m[4], mx->m[5], mx->m[6], mx->m[8], mx->m[9], mx->m[10], mx->m[12], mx->m[13], mx->m[14]);

	// get determinant
	float determinant = mx->m[0] * cofactor0 - mx->m[1] * cofactor1 + mx->m[2] * cofactor2 - mx->m[3] * cofactor3;
	if (fabs(determinant) <= EPSILON)
	{
		*mx = Matrix4_identity();
		return;
	}

	// get rest of cofactors for adj(M)
	float cofactor4 = getCofactor(mx->m[1], mx->m[2], mx->m[3], mx->m[9], mx->m[10], mx->m[11], mx->m[13], mx->m[14], mx->m[15]);
	float cofactor5 = getCofactor(mx->m[0], mx->m[2], mx->m[3], mx->m[8], mx->m[10], mx->m[11], mx->m[12], mx->m[14], mx->m[15]);
	float cofactor6 = getCofactor(mx->m[0], mx->m[1], mx->m[3], mx->m[8], mx->m[9], mx->m[11], mx->m[12], mx->m[13], mx->m[15]);
	float cofactor7 = getCofactor(mx->m[0], mx->m[1], mx->m[2], mx->m[8], mx->m[9], mx->m[10], mx->m[12], mx->m[13], mx->m[14]);

	float cofactor8 = getCofactor(mx->m[1], mx->m[2], mx->m[3], mx->m[5], mx->m[6], mx->m[7], mx->m[13], mx->m[14], mx->m[15]);
	float cofactor9 = getCofactor(mx->m[0], mx->m[2], mx->m[3], mx->m[4], mx->m[6], mx->m[7], mx->m[12], mx->m[14], mx->m[15]);
	float cofactor10 = getCofactor(mx->m[0], mx->m[1], mx->m[3], mx->m[4], mx->m[5], mx->m[7], mx->m[12], mx->m[13], mx->m[15]);
	float cofactor11 = getCofactor(mx->m[0], mx->m[1], mx->m[2], mx->m[4], mx->m[5], mx->m[6], mx->m[12], mx->m[13], mx->m[14]);

	float cofactor12 = getCofactor(mx->m[1], mx->m[2], mx->m[3], mx->m[5], mx->m[6], mx->m[7], mx->m[9], mx->m[10], mx->m[11]);
	float cofactor13 = getCofactor(mx->m[0], mx->m[2], mx->m[3], mx->m[4], mx->m[6], mx->m[7], mx->m[8], mx->m[10], mx->m[11]);
	float cofactor14 = getCofactor(mx->m[0], mx->m[1], mx->m[3], mx->m[4], mx->m[5], mx->m[7], mx->m[8], mx->m[9], mx->m[11]);
	float cofactor15 = getCofactor(mx->m[0], mx->m[1], mx->m[2], mx->m[4], mx->m[5], mx->m[6], mx->m[8], mx->m[9], mx->m[10]);

	// build inverse matrix = adj(M) / det(M)
	// adjugate of M is the transpose of the cofactor matrix of M
	float invDeterminant = 1.0f / determinant;
	mx->m[0] = invDeterminant * cofactor0;
	mx->m[1] = -invDeterminant * cofactor4;
	mx->m[2] = invDeterminant * cofactor8;
	mx->m[3] = -invDeterminant * cofactor12;

	mx->m[4] = -invDeterminant * cofactor1;
	mx->m[5] = invDeterminant * cofactor5;
	mx->m[6] = -invDeterminant * cofactor9;
	mx->m[7] = invDeterminant * cofactor13;

	mx->m[8] = invDeterminant * cofactor2;
	mx->m[9] = -invDeterminant * cofactor6;
	mx->m[10] = invDeterminant * cofactor10;
	mx->m[11] = -invDeterminant * cofactor14;

	mx->m[12] = -invDeterminant * cofactor3;
	mx->m[13] = invDeterminant * cofactor7;
	mx->m[14] = -invDeterminant * cofactor11;
	mx->m[15] = invDeterminant * cofactor15;
}

Vector4 Matrix4_multiply_Vector4(const Matrix4* const lhs, const Vector4* const rhs)
{
	Vector4 v = {
		lhs->m[0] * rhs->x + lhs->m[4] * rhs->y + lhs->m[8] * rhs->z + lhs->m[12] * rhs->w,
		lhs->m[1] * rhs->x + lhs->m[5] * rhs->y + lhs->m[9] * rhs->z + lhs->m[13] * rhs->w,
		lhs->m[2] * rhs->x + lhs->m[6] * rhs->y + lhs->m[10] * rhs->z + lhs->m[14] * rhs->w,
		lhs->m[3] * rhs->x + lhs->m[7] * rhs->y + lhs->m[11] * rhs->z + lhs->m[15] * rhs->w
	};
	return v;
}

Matrix4 Matrix4_multiply_Matrix4(const Matrix4* const l, const Matrix4* const r)
{
	{
		Matrix4 mx = {
			.m = {
			l->m[0] * r->m[0] + l->m[4] * r->m[1] + l->m[8] * r->m[2] + l->m[12] * r->m[3], l->m[1] * r->m[0] + l->m[5] * r->m[1] + l->m[9] * r->m[2] + l->m[13] * r->m[3], l->m[2] * r->m[0] + l->m[6] * r->m[1] + l->m[10] * r->m[2] + l->m[14] * r->m[3], l->m[3] * r->m[0] + l->m[7] * r->m[1] + l->m[11] * r->m[2] + l->m[15] * r->m[3],
			l->m[0] * r->m[4] + l->m[4] * r->m[5] + l->m[8] * r->m[6] + l->m[12] * r->m[7], l->m[1] * r->m[4] + l->m[5] * r->m[5] + l->m[9] * r->m[6] + l->m[13] * r->m[7], l->m[2] * r->m[4] + l->m[6] * r->m[5] + l->m[10] * r->m[6] + l->m[14] * r->m[7], l->m[3] * r->m[4] + l->m[7] * r->m[5] + l->m[11] * r->m[6] + l->m[15] * r->m[7],
			l->m[0] * r->m[8] + l->m[4] * r->m[9] + l->m[8] * r->m[10] + l->m[12] * r->m[11], l->m[1] * r->m[8] + l->m[5] * r->m[9] + l->m[9] * r->m[10] + l->m[13] * r->m[11], l->m[2] * r->m[8] + l->m[6] * r->m[9] + l->m[10] * r->m[10] + l->m[14] * r->m[11], l->m[3] * r->m[8] + l->m[7] * r->m[9] + l->m[11] * r->m[10] + l->m[15] * r->m[11],
			l->m[0] * r->m[12] + l->m[4] * r->m[13] + l->m[8] * r->m[14] + l->m[12] * r->m[15], l->m[1] * r->m[12] + l->m[5] * r->m[13] + l->m[9] * r->m[14] + l->m[13] * r->m[15], l->m[2] * r->m[12] + l->m[6] * r->m[13] + l->m[10] * r->m[14] + l->m[14] * r->m[15], l->m[3] * r->m[12] + l->m[7] * r->m[13] + l->m[11] * r->m[14] + l->m[15] * r->m[15]
		}
		};
		return mx;
	}
}
