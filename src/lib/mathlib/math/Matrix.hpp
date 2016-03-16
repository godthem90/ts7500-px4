/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
 *           Pavel Kirienko <pavel.kirienko@gmail.com>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file Matrix.hpp
 *
 * Matrix class
 */

#ifndef MATRIX_HPP
#define MATRIX_HPP

#include <stdio.h>
#include <math.h>

#ifdef CONFIG_ARCH_ARM
#include "../CMSIS/Include/arm_math.h"
#else
#include "matrix/math.hpp"
#endif
#include <platforms/px4_defines.h>

namespace math
{

/*template<unsigned int M, unsigned int N>
class __EXPORT Matrix;*/

template <unsigned int M, unsigned int N>
class __EXPORT MatrixBase;

template <unsigned int M, unsigned int N>
class __EXPORT Matrix
{
public:
	/**
	 * matrix data[row][col]
	 */
	float data[M][N];

	/**
	 * struct for using arm_math functions
	 */
#ifdef CONFIG_ARCH_ARM
	arm_matrix_instance_f32 arm_mat;
#else
	eigen_matrix_instance arm_mat;
#endif

	/**
	 * trivial ctor
	 * Initializes the elements to zero.
	 */
	Matrix()
	{
		arm_mat.numRows = M;
		arm_mat.numCols = N;
		arm_mat.pData = &data[0][0];
	}

	virtual ~Matrix() {};

	/**
	 * copyt ctor
	 */
	Matrix(const Matrix<M, N> &m)
	{
		arm_mat.numRows = M;
		arm_mat.numCols = N;
		arm_mat.pData = &data[0][0];
		memcpy(data, m.data, sizeof(data));
	}

	Matrix(const float *d)
	{
		arm_mat.numRows = M;
		arm_mat.numCols = N;
		arm_mat.pData = &data[0][0];
		memcpy(data, d, sizeof(data));
	}

	Matrix(const float d[M][N])
	{
		arm_mat.numRows = M;
		arm_mat.numCols = N;
		arm_mat.pData = &data[0][0];
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float *d) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float d[M][N]) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set row from vector
	 */
	void set_row(unsigned int row, const Vector<N> v) {
		for (unsigned i = 0; i < N; i++) {
			data[row][i] = v.data[i];
		}
	}

	/**
	 * set column from vector
	 */
	void set_col(unsigned int col, const Vector<M> v) {
		for (unsigned i = 0; i < M; i++) {
			data[i][col] = v.data[i];
		}
	}

	/**
	 * access by index
	 */
	float &operator()(const unsigned int row, const unsigned int col) {
		return data[row][col];
	}

	/**
	 * access by index
	 */
	float operator()(const unsigned int row, const unsigned int col) const {
		return data[row][col];
	}

	/**
	 * get rows number
	 */
	unsigned int get_rows() const {
		return M;
	}

	/**
	 * get columns number
	 */
	unsigned int get_cols() const {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return true;

		return false;
	}

	/**
	 * set to value
	 */
	const Matrix<M, N> &operator =(const Matrix<M, N> &m) {
		memcpy(data, m.data, sizeof(data));
		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * negation
	 */
	Matrix<M, N> operator -(void) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = -data[i][j];

		return res;
	}

	/**
	 * addition
	 */
	Matrix<M, N> operator +(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] + m.data[i][j];

		return res;
	}

	Matrix<M, N> &operator +=(const Matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] += m.data[i][j];

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * subtraction
	 */
	Matrix<M, N> operator -(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] - m.data[i][j];

		return res;
	}

	Matrix<M, N> &operator -=(const Matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] -= m.data[i][j];

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	Matrix<M, N> operator *(const float num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] * num;

		return res;
	}

	Matrix<M, N> &operator *=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] *= num;

		return *static_cast<Matrix<M, N>*>(this);
	}

	Matrix<M, N> operator /(const float num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] / num;

		return res;
	}

	Matrix<M, N> &operator /=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] /= num;

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * multiplication by another matrix
	 */
	template <unsigned int P>
	Matrix<M, P> operator *(const Matrix<N, P> &m) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<M, P> res;
		arm_mat_mult_f32(&arm_mat, &m.arm_mat, &res.arm_mat);
		return res;
#else
		matrix::Matrix<float, M, N> Me(this->arm_mat.pData);
		matrix::Matrix<float, N, P> Him(m.arm_mat.pData);
		matrix::Matrix<float, M, P> Product = Me * Him;
		Matrix<M, P> res(Product.data());
		return res;
#endif
	}

	/**
	 * transpose the matrix
	 */
	Matrix<N, M> transposed(void) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<N, M> res;
		arm_mat_trans_f32(&this->arm_mat, &res.arm_mat);
		return res;
#else
		matrix::Matrix<float, N, M> Me(this->arm_mat.pData);
		Matrix<N, M> res(Me.transpose().data());
		return res;
#endif
	}

	/**
	 * invert the matrix
	 */
	Matrix<M, N> inversed(void) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<M, N> res;
		arm_mat_inverse_f32(&this->arm_mat, &res.arm_mat);
		return res;
#else
		matrix::SquareMatrix<float, M> Me = matrix::Matrix<float, M, N>(this->arm_mat.pData);
		Matrix<M, N> res(Me.I().data());
		return res;
#endif
	}

	/**
	 * set zero matrix
	 */
	void zero(void) {
		memset(data, 0, sizeof(data));
	}

	/**
	 * set identity matrix
	 */
	void identity(void) {
		memset(data, 0, sizeof(data));
		unsigned int n = (M < N) ? M : N;

		for (unsigned int i = 0; i < n; i++)
			data[i][i] = 1;
	}

	void print(void) {
		for (unsigned int i = 0; i < M; i++) {
			printf("[ ");

			for (unsigned int j = 0; j < N; j++)
				printf("%.3f\t", (double)data[i][j]);

			printf(" ]\n");
		}
	}

	Vector<M> operator *(const Vector<N> &v) const {
#ifdef CONFIG_ARCH_ARM
		Vector<M> res;
		arm_mat_mult_f32(&arm_mat, &v.arm_col, &res.arm_col);
#else
		matrix::Matrix<float, M, N> Me(arm_mat.pData);
		matrix::Matrix<float, N, 1> Vec(v.arm_col.pData);
		matrix::Matrix<float, M, 1> Product = Me * Vec;
		Vector<M> res(Product.data());
#endif
		return res;
	}
};

template <>
class __EXPORT Matrix<3, 3>
{
public:
	/**
	 * matrix data[row][col]
	 */
	float data[3][3];

	/**
	 * struct for using arm_math functions
	 */
#ifdef CONFIG_ARCH_ARM
	arm_matrix_instance_f32 arm_mat;
#else
	eigen_matrix_instance arm_mat;
#endif

	/**
	 * trivial ctor
	 * Initializes the elements to zero.
	 */
	Matrix()
	{
		arm_mat.numRows = 3;
		arm_mat.numCols = 3;
		arm_mat.pData = &data[0][0];
	}

	virtual ~Matrix() {};

	/**
	 * copyt ctor
	 */
	Matrix(const Matrix<3, 3> &m)
	{
		arm_mat.numRows = 3;
		arm_mat.numCols = 3;
		arm_mat.pData = &data[0][0];
		memcpy(data, m.data, sizeof(data));
	}

	Matrix(const float *d)
	{
		arm_mat.numRows = 3;
		arm_mat.numCols = 3;
		arm_mat.pData = &data[0][0];
		memcpy(data, d, sizeof(data));
	}

	Matrix(const float d[3][3])
	{
		arm_mat.numRows = 3;
		arm_mat.numCols = 3;
		arm_mat.pData = &data[0][0];
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float *d) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float d[3][3]) {
		memcpy(data, d, sizeof(data));
	}

	/*void set(const float d[9]) {
		memcpy(data, d, sizeof(data));
	}*/

#if defined(__PX4_ROS)
	void set(const boost::array<float, 9ul> d) {
	set(static_cast<const float*>(d.data()));
	}
#endif

	/**
	 * set row from vector
	 */
	void set_row(unsigned int row, const Vector<3> v) {
		for (unsigned i = 0; i < 3; i++) {
			data[row][i] = v.data[i];
		}
	}

	/**
	 * set column from vector
	 */
	void set_col(unsigned int col, const Vector<3> v) {
		for (unsigned i = 0; i < 3; i++) {
			data[i][col] = v.data[i];
		}
	}

	/**
	 * access by index
	 */
	float &operator()(const unsigned int row, const unsigned int col) {
		return data[row][col];
	}

	/**
	 * access by index
	 */
	float operator()(const unsigned int row, const unsigned int col) const {
		return data[row][col];
	}

	/**
	 * get rows number
	 */
	unsigned int get_rows() const {
		return 3;
	}

	/**
	 * get columns number
	 */
	unsigned int get_cols() const {
		return 3;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Matrix<3, 3> &m) const {
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				if (data[i][j] != m.data[i][j])
					return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Matrix<3, 3> &m) const {
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				if (data[i][j] != m.data[i][j])
					return true;

		return false;
	}

	/**
	 * set to value
	 */
	const Matrix<3, 3> &operator =(const Matrix<3, 3> &m) {
		memcpy(data, m.data, sizeof(data));
		return *static_cast<Matrix<3, 3>*>(this);
	}

	/**
	 * negation
	 */
	Matrix<3, 3> operator -(void) const {
		Matrix<3, 3> res;

		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				res.data[i][j] = -data[i][j];

		return res;
	}

	/**
	 * addition
	 */
	Matrix<3, 3> operator +(const Matrix<3, 3> &m) const {
		Matrix<3, 3> res;

		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				res.data[i][j] = data[i][j] + m.data[i][j];

		return res;
	}

	Matrix<3, 3> &operator +=(const Matrix<3, 3> &m) {
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				data[i][j] += m.data[i][j];

		return *static_cast<Matrix<3, 3>*>(this);
	}

	/**
	 * subtraction
	 */
	Matrix<3, 3> operator -(const Matrix<3, 3> &m) const {
		Matrix<3, 3> res;

		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				res.data[i][j] = data[i][j] - m.data[i][j];

		return res;
	}

	Matrix<3, 3> &operator -=(const Matrix<3, 3> &m) {
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				data[i][j] -= m.data[i][j];

		return *static_cast<Matrix<3, 3>*>(this);
	}

	/**
	 * uniform scaling
	 */
	Matrix<3, 3> operator *(const float num) const {
		Matrix<3, 3> res;

		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				res.data[i][j] = data[i][j] * num;

		return res;
	}

	Matrix<3, 3> &operator *=(const float num) {
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				data[i][j] *= num;

		return *static_cast<Matrix<3, 3>*>(this);
	}

	Matrix<3, 3> operator /(const float num) const {
		Matrix<3, 3> res;

		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				res.data[i][j] = data[i][j] / num;

		return res;
	}

	Matrix<3, 3> &operator /=(const float num) {
		for (unsigned int i = 0; i < 3; i++)
			for (unsigned int j = 0; j < 3; j++)
				data[i][j] /= num;

		return *static_cast<Matrix<3, 3>*>(this);
	}

	/**
	 * multiplication by another matrix
	 */
	template <unsigned int P>
	Matrix<3, P> operator *(const Matrix<3, P> &m) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<M, P> res;
		arm_mat_mult_f32(&arm_mat, &m.arm_mat, &res.arm_mat);
		return res;
#else
		matrix::Matrix<float, 3, 3> Me(this->arm_mat.pData);
		matrix::Matrix<float, 3, P> Him(m.arm_mat.pData);
		matrix::Matrix<float, 3, P> Product = Me * Him;
		Matrix<3, P> res(Product.data());
		return res;
#endif
	}

	/**
	 * transpose the matrix
	 */
	Matrix<3, 3> transposed(void) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<3, 3> res;
		arm_mat_trans_f32(&this->arm_mat, &res.arm_mat);
		return res;
#else
		matrix::Matrix<float, 3, 3> Me(this->arm_mat.pData);
		Matrix<3, 3> res(Me.transpose().data());
		return res;
#endif
	}

	/**
	 * invert the matrix
	 */
	Matrix<3, 3> inversed(void) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<3, 3> res;
		arm_mat_inverse_f32(&this->arm_mat, &res.arm_mat);
		return res;
#else
		matrix::SquareMatrix<float, 3> Me = matrix::Matrix<float, 3, 3>(this->arm_mat.pData);
		Matrix<3, 3> res(Me.I().data());
		return res;
#endif
	}

	/**
	 * set zero matrix
	 */
	void zero(void) {
		memset(data, 0, sizeof(data));
	}

	/**
	 * set identity matrix
	 */
	void identity(void) {
		memset(data, 0, sizeof(data));
		unsigned int n = (3 < 3) ? 3 : 3;

		for (unsigned int i = 0; i < n; i++)
			data[i][i] = 1;
	}

	void print(void) {
		for (unsigned int i = 0; i < 3; i++) {
			printf("[ ");

			for (unsigned int j = 0; j < 3; j++)
				printf("%.3f\t", (double)data[i][j]);

			printf(" ]\n");
		}
	}


	Vector<3> operator *(const Vector<3> &v) const {
		Vector<3> res(data[0][0] * v.data[0] + data[0][1] * v.data[1] + data[0][2] * v.data[2],
			      data[1][0] * v.data[0] + data[1][1] * v.data[1] + data[1][2] * v.data[2],
			      data[2][0] * v.data[0] + data[2][1] * v.data[1] + data[2][2] * v.data[2]);
		return res;
	}

	void from_euler(float roll, float pitch, float yaw) {
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		data[0][0] = cp * cy;
		data[0][1] = (sr * sp * cy) - (cr * sy);
		data[0][2] = (cr * sp * cy) + (sr * sy);
		data[1][0] = cp * sy;
		data[1][1] = (sr * sp * sy) + (cr * cy);
		data[1][2] = (cr * sp * sy) - (sr * cy);
		data[2][0] = -sp;
		data[2][1] = sr * cp;
		data[2][2] = cr * cp;
	}

	Vector<3> to_euler(void) const {
		Vector<3> euler;
		euler.data[1] = asinf(-data[2][0]);

		if (fabsf(euler.data[1] - M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) + euler.data[0];

		} else if (fabsf(euler.data[1] + M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) - euler.data[0];

		} else {
			euler.data[0] = atan2f(data[2][1], data[2][2]);
			euler.data[2] = atan2f(data[1][0], data[0][0]);
		}

		return euler;
	}
};


// MxN matrix with float elements
template <unsigned int M, unsigned int N>
class __EXPORT MatrixBase
{
public:
	/**
	 * matrix data[row][col]
	 */
	float data[M][N];

	/**
	 * struct for using arm_math functions
	 */
#ifdef CONFIG_ARCH_ARM
	arm_matrix_instance_f32 arm_mat;
#else
	eigen_matrix_instance arm_mat;
#endif

	/**
	 * trivial ctor
	 * Initializes the elements to zero.
	 */
	MatrixBase() : 
		data(),
		arm_mat(M, N, &data[0][0])
	{
	}

	virtual ~MatrixBase() {};

	/**
	 * copyt ctor
	 */
	MatrixBase(const MatrixBase<M, N> &m) :
		arm_mat(M, N, &data[0][0])
	{
		memcpy(data, m.data, sizeof(data));
	}

	MatrixBase(const float *d) :
		arm_mat(M, N, &data[0][0])
	{
		memcpy(data, d, sizeof(data));
	}

	MatrixBase(const float d[M][N]) : 
		arm_mat(M, N, &data[0][0])
	{
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float *d) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set data
	 */
	void set(const float d[M][N]) {
		memcpy(data, d, sizeof(data));
	}

	/**
	 * set row from vector
	 */
	void set_row(unsigned int row, const Vector<N> v) {
		for (unsigned i = 0; i < N; i++) {
			data[row][i] = v.data[i];
		}
	}

	/**
	 * set column from vector
	 */
	void set_col(unsigned int col, const Vector<M> v) {
		for (unsigned i = 0; i < M; i++) {
			data[i][col] = v.data[i];
		}
	}

	/**
	 * access by index
	 */
	float &operator()(const unsigned int row, const unsigned int col) {
		return data[row][col];
	}

	/**
	 * access by index
	 */
	float operator()(const unsigned int row, const unsigned int col) const {
		return data[row][col];
	}

	/**
	 * get rows number
	 */
	unsigned int get_rows() const {
		return M;
	}

	/**
	 * get columns number
	 */
	unsigned int get_cols() const {
		return N;
	}

	/**
	 * test for equality
	 */
	bool operator ==(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return false;

		return true;
	}

	/**
	 * test for inequality
	 */
	bool operator !=(const Matrix<M, N> &m) const {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				if (data[i][j] != m.data[i][j])
					return true;

		return false;
	}

	/**
	 * set to value
	 */
	const Matrix<M, N> &operator =(const Matrix<M, N> &m) {
		memcpy(data, m.data, sizeof(data));
		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * negation
	 */
	Matrix<M, N> operator -(void) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = -data[i][j];

		return res;
	}

	/**
	 * addition
	 */
	Matrix<M, N> operator +(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] + m.data[i][j];

		return res;
	}

	Matrix<M, N> &operator +=(const Matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] += m.data[i][j];

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * subtraction
	 */
	Matrix<M, N> operator -(const Matrix<M, N> &m) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] - m.data[i][j];

		return res;
	}

	Matrix<M, N> &operator -=(const Matrix<M, N> &m) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] -= m.data[i][j];

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * uniform scaling
	 */
	Matrix<M, N> operator *(const float num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] * num;

		return res;
	}

	Matrix<M, N> &operator *=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] *= num;

		return *static_cast<Matrix<M, N>*>(this);
	}

	Matrix<M, N> operator /(const float num) const {
		Matrix<M, N> res;

		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				res.data[i][j] = data[i][j] / num;

		return res;
	}

	Matrix<M, N> &operator /=(const float num) {
		for (unsigned int i = 0; i < M; i++)
			for (unsigned int j = 0; j < N; j++)
				data[i][j] /= num;

		return *static_cast<Matrix<M, N>*>(this);
	}

	/**
	 * multiplication by another matrix
	 */
	template <unsigned int P>
	Matrix<M, P> operator *(const Matrix<N, P> &m) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<M, P> res;
		arm_mat_mult_f32(&arm_mat, &m.arm_mat, &res.arm_mat);
		return res;
#else
		matrix::Matrix<float, M, N> Me(this->arm_mat.pData);
		matrix::Matrix<float, N, P> Him(m.arm_mat.pData);
		matrix::Matrix<float, M, P> Product = Me * Him;
		Matrix<M, P> res(Product.data());
		return res;
#endif
	}

	/**
	 * transpose the matrix
	 */
	Matrix<N, M> transposed(void) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<N, M> res;
		arm_mat_trans_f32(&this->arm_mat, &res.arm_mat);
		return res;
#else
		matrix::Matrix<float, N, M> Me(this->arm_mat.pData);
		Matrix<N, M> res(Me.transpose().data());
		return res;
#endif
	}

	/**
	 * invert the matrix
	 */
	Matrix<M, N> inversed(void) const {
#ifdef CONFIG_ARCH_ARM
		Matrix<M, N> res;
		arm_mat_inverse_f32(&this->arm_mat, &res.arm_mat);
		return res;
#else
		matrix::SquareMatrix<float, M> Me = matrix::Matrix<float, M, N>(this->arm_mat.pData);
		Matrix<M, N> res(Me.I().data());
		return res;
#endif
	}

	/**
	 * set zero matrix
	 */
	void zero(void) {
		memset(data, 0, sizeof(data));
	}

	/**
	 * set identity matrix
	 */
	void identity(void) {
		memset(data, 0, sizeof(data));
		unsigned int n = (M < N) ? M : N;

		for (unsigned int i = 0; i < n; i++)
			data[i][i] = 1;
	}

	void print(void) {
		for (unsigned int i = 0; i < M; i++) {
			printf("[ ");

			for (unsigned int j = 0; j < N; j++)
				printf("%.3f\t", (double)data[i][j]);

			printf(" ]\n");
		}
	}
};

/*
template <unsigned int M, unsigned int N>
class __EXPORT Matrix : public MatrixBase<M, N>
{
public:
	using MatrixBase<M, N>::operator *;

	Matrix() : MatrixBase<M, N>() {}

	Matrix(const Matrix<M, N> &m) : MatrixBase<M, N>(m) {}

	Matrix(const float *d) : MatrixBase<M, N>(d) {}

	Matrix(const float d[M][N]) : MatrixBase<M, N>(d) {}

	const Matrix<M, N> &operator =(const Matrix<M, N> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	Vector<M> operator *(const Vector<N> &v) const {
#ifdef CONFIG_ARCH_ARM
		Vector<M> res;
		arm_mat_mult_f32(&this->arm_mat, &v.arm_col, &res.arm_col);
#else
		matrix::Matrix<float, M, N> Me(this->arm_mat.pData);
		matrix::Matrix<float, N, 1> Vec(v.arm_col.pData);
		matrix::Matrix<float, M, 1> Product = Me * Vec;
		Vector<M> res(Product.data());
#endif
		return res;
	}
};

template <>
class __EXPORT Matrix : public MatrixBase<3, 3>
{
public:
	using MatrixBase<3, 3>::operator *;

	Matrix() : MatrixBase<3, 3>() {}

	Matrix(const Matrix<3, 3> &m) : MatrixBase<3, 3>(m) {}

	Matrix(const float *d) : MatrixBase<3, 3>(d) {}

	Matrix(const float d[3][3]) : MatrixBase<3, 3>(d) {}
	void set(const float d[9]) {
		memcpy(data, d, sizeof(data));
	}

#if defined(__PX4_ROS)
	void set(const boost::array<float, 9ul> d) {
	set(static_cast<const float*>(d.data()));
	}
#endif

	const Matrix<3, 3> &operator =(const Matrix<3, 3> &m) {
		memcpy(this->data, m.data, sizeof(this->data));
		return *this;
	}

	Vector<3> operator *(const Vector<3> &v) const {
		Vector<3> res(data[0][0] * v.data[0] + data[0][1] * v.data[1] + data[0][2] * v.data[2],
			      data[1][0] * v.data[0] + data[1][1] * v.data[1] + data[1][2] * v.data[2],
			      data[2][0] * v.data[0] + data[2][1] * v.data[1] + data[2][2] * v.data[2]);
		return res;
	}

	void from_euler(float roll, float pitch, float yaw) {
		float cp = cosf(pitch);
		float sp = sinf(pitch);
		float sr = sinf(roll);
		float cr = cosf(roll);
		float sy = sinf(yaw);
		float cy = cosf(yaw);

		data[0][0] = cp * cy;
		data[0][1] = (sr * sp * cy) - (cr * sy);
		data[0][2] = (cr * sp * cy) + (sr * sy);
		data[1][0] = cp * sy;
		data[1][1] = (sr * sp * sy) + (cr * cy);
		data[1][2] = (cr * sp * sy) - (sr * cy);
		data[2][0] = -sp;
		data[2][1] = sr * cp;
		data[2][2] = cr * cp;
	}

	Vector<3> to_euler(void) const {
		Vector<3> euler;
		euler.data[1] = asinf(-data[2][0]);

		if (fabsf(euler.data[1] - M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) + euler.data[0];

		} else if (fabsf(euler.data[1] + M_PI_2_F) < 1.0e-3f) {
			euler.data[0] = 0.0f;
			euler.data[2] = atan2f(data[1][2] - data[0][1], data[0][2] + data[1][1]) - euler.data[0];

		} else {
			euler.data[0] = atan2f(data[2][1], data[2][2]);
			euler.data[2] = atan2f(data[1][0], data[0][0]);
		}

		return euler;
	}
};
*/
}

#endif // MATRIX_HPP
