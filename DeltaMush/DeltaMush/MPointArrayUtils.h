// Copyright 2018 Luca Di Sera
//		Contact: disera.luca@gmail.com
//				 https://github.com/diseraluca
//				 https://www.linkedin.com/in/luca-di-sera-200023167
//
// This code is licensed under the MIT License. 
// More informations can be found in the LICENSE file in the root folder of this repository
//
//
// File : MPointArrayUtils.h
//
// The MPointArrayUtils namespace provides some utility functions for the deltaMush deformer.
// In particular the functions needed to move a MPointArray to SOA Form and back to an MPointArray.

#pragma once

#include <maya/MPointArray.h>
#include <immintrin.h>

namespace MPointArrayUtils {
	inline void decomposePointArray(MPointArray & points, double * out_x, double * out_y, double * out_z, unsigned int vertexCount)
	{
		double* pointsPtr{ &points[0][0] };

		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4, out_x += 4, out_y += 4, out_z += 4, pointsPtr += 16) {
			__m256d xx = _mm256_setr_pd(pointsPtr[0], pointsPtr[0 + 4], pointsPtr[0 + 8], pointsPtr[0 + 12]);
			__m256d yy = _mm256_setr_pd(pointsPtr[1], pointsPtr[1 + 4], pointsPtr[1 + 8], pointsPtr[1 + 12]);
			__m256d zz = _mm256_setr_pd(pointsPtr[2], pointsPtr[2 + 4], pointsPtr[2 + 8], pointsPtr[2 + 12]);

			_mm256_store_pd(out_x, xx);
			_mm256_store_pd(out_y, yy);
			_mm256_store_pd(out_z, zz);
		}
	}

	inline void composePointArray(double * x, double * y, double * z, MPointArray & out_points, unsigned int vertexCount)
	{
		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex, ++x, ++y, ++z) {
			out_points[vertexIndex].x = x[0];
			out_points[vertexIndex].y = y[0];
			out_points[vertexIndex].z = z[0];
			out_points[vertexIndex].w = 1.0;
		}
	}
}