// Copyright 2018 Luca Di Sera
//		Contact: disera.luca@gmail.com
//				 https://github.com/diseraluca
//				 https://www.linkedin.com/in/luca-di-sera-200023167
//
// This code is licensed under the MIT License. 
// More informations can be found in the LICENSE file in the root folder of this repository
//
//
// File : ComponentVector256dMatrix3x3.h
//
// ComponentVector256dMatrix3x3 is an helper class for the deltaMush deformer aimed to improve code readability.
// It provides an interface to a 4-in-1 3x3 matrix that uses three ComponentVector256d as its rows.

#pragma once

#include "ComponentVector256d.h"

class ComponentVector256dMatrix3x3 {
public:
	/// Constructors
	inline ComponentVector256dMatrix3x3() :tangent(), normal(), binormal() {}

	// The product between a vector and the inverse of the matrix
	inline ComponentVector256d inverseProduct(const ComponentVector256d& vector) {
		__m256d length = _mm256_add_pd(_mm256_mul_pd(normal.x, normal.x), _mm256_add_pd(_mm256_mul_pd(normal.y, normal.y), _mm256_mul_pd(normal.z, normal.z)));
		__m256d factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);

		normal *= factor;

		length = _mm256_add_pd(_mm256_mul_pd(binormal.x, binormal.x), _mm256_add_pd(_mm256_mul_pd(binormal.y, binormal.y), _mm256_mul_pd(binormal.z, binormal.z)));
		factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);

		binormal *= factor;

		return ComponentVector256d(
			_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(tangent.x, vector.x), _mm256_mul_pd(normal.x, vector.y)), _mm256_mul_pd(binormal.x, vector.z)),
			_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(tangent.y, vector.x), _mm256_mul_pd(normal.y, vector.y)), _mm256_mul_pd(binormal.y, vector.z)),
			_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(tangent.z, vector.x), _mm256_mul_pd(normal.z, vector.y)), _mm256_mul_pd(binormal.z, vector.z))
		);
	}

	/// Matrix - Vector operators
	inline ComponentVector256d operator*(const ComponentVector256d& vector) {
		return ComponentVector256d(
			_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(this->tangent.x, vector.x), _mm256_mul_pd(this->tangent.y, vector.y)), _mm256_mul_pd(this->tangent.z, vector.z)),
			_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(this->normal.x, vector.x), _mm256_mul_pd(this->normal.y, vector.y)), _mm256_mul_pd(this->normal.z, vector.z)),
			_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(this->binormal.x, vector.x), _mm256_mul_pd(this->binormal.y, vector.y)), _mm256_mul_pd(this->binormal.z, vector.z))
		);
	}

public:
	ComponentVector256d tangent;
	ComponentVector256d normal;
	ComponentVector256d binormal;
};