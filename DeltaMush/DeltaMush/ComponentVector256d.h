// Copyright 2018 Luca Di Sera
//		Contact: disera.luca@gmail.com
//				 https://github.com/diseraluca
//				 https://www.linkedin.com/in/luca-di-sera-200023167
//
// This code is licensed under the MIT License. 
// More informations can be found in the LICENSE file in the root folder of this repository
//
//
// File : ComponentVector256d.h
//
// ComponentVector256d is an helper class for the deltaMush deformer aimed to improve code readability.
// It provides an interface for SOA Vectors that can contain and process 4 doubles at once.

#pragma once

#include <immintrin.h>

class ComponentVector256d {
public:

	/// Constructors
	inline ComponentVector256d() : x(_mm256_setzero_pd()), y(_mm256_setzero_pd()), z(_mm256_setzero_pd()) {}
	inline ComponentVector256d(const __m256d xx, const __m256d yy, const __m256d zz) : x(xx), y(yy), z(zz) {}
	inline ComponentVector256d(const double* xx, const double* yy, const double* zz) : x(_mm256_load_pd(xx)), y(_mm256_load_pd(yy)), z(_mm256_load_pd(zz)) {}

	// Set the components of the vector to the specified vectors
	inline void set(const __m256d xx, const __m256d yy, const __m256d zz) {
		x = xx;
		y = yy;
		z = zz;
	}

	// Set the components of the vector to zero
	inline void setZero() { 
		x = _mm256_setzero_pd();
		y = _mm256_setzero_pd();
		z = _mm256_setzero_pd();
	}

	// Return a vector containing the lengths of the contained vectors calculated as sqrt( x^2 + y^2 + z^2)
	inline __m256d length() const { return _mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(x, x), _mm256_add_pd(_mm256_mul_pd(y, y), _mm256_mul_pd(z, z)))); }

	// In-place normalization of the contained vectors
	inline void normalize() {
		__m256d factor{ _mm256_div_pd(_mm256_set1_pd(1.0), this->length()) };

		*(this) *= factor;
	}

	// Matrix - vector product where this is treated as an Nx3 matrix row
	inline __m256d asMatrixRowProduct(const ComponentVector256d& vector ) const {
		return _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(this->x, vector.x), _mm256_mul_pd(this->y, vector.y)), _mm256_mul_pd(this->z, vector.z));
	}

	inline void store(double* out_x, double* out_y, double* out_z) const {
		_mm256_store_pd(out_x, x);
		_mm256_store_pd(out_y, y);
		_mm256_store_pd(out_z, z);
	}

	/// ComponentVector256d - ComponentVector256d operators
	inline ComponentVector256d operator+(const ComponentVector256d& other) const { return ComponentVector256d(_mm256_add_pd(x, other.x), _mm256_add_pd(y, other.y), _mm256_add_pd(z, other.z)); }
	inline ComponentVector256d& operator+=(const ComponentVector256d& other) {
		x = _mm256_add_pd(x, other.x);
		y = _mm256_add_pd(y, other.y);
		z = _mm256_add_pd(z, other.z);

		return *this;
	}

	inline ComponentVector256d operator-(const ComponentVector256d& other) const { return ComponentVector256d(_mm256_sub_pd(x, other.x), _mm256_sub_pd(y, other.y), _mm256_sub_pd(z, other.z)); }
	inline ComponentVector256d operator*(const ComponentVector256d& other) const { return ComponentVector256d(_mm256_mul_pd(x, other.x), _mm256_mul_pd(y, other.y), _mm256_mul_pd(z, other.z)); }

	// The cross product operator
	inline ComponentVector256d operator^(const ComponentVector256d& other) const {
		return ComponentVector256d(
			_mm256_sub_pd(_mm256_mul_pd(y, other.z), _mm256_mul_pd(z, other.y)),
			_mm256_sub_pd(_mm256_mul_pd(z, other.x), _mm256_mul_pd(x, other.z)),
			_mm256_sub_pd(_mm256_mul_pd(x, other.y), _mm256_mul_pd(y, other.x))
		);
	}

	/// ComponentVector256d - __m256d operators
	inline ComponentVector256d operator*(const __m256d multiplier) const { return ComponentVector256d(_mm256_mul_pd(x, multiplier), _mm256_mul_pd(y, multiplier), _mm256_mul_pd(z, multiplier)); }
	inline ComponentVector256d& operator*=(const __m256d multiplier) { 
		x = _mm256_mul_pd(x, multiplier);
		y = _mm256_mul_pd(y, multiplier);
		z = _mm256_mul_pd(z, multiplier);

		return *this;
	}

public:
	__m256d x;
	__m256d y;
	__m256d z;
};