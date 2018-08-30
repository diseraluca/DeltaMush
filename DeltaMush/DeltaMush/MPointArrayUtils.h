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

namespace MPointArrayUtils {
	inline void decomposePointArray(const MPointArray & points, double * out_x, double * out_y, double * out_z, unsigned int vertexCount)
	{
		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex, ++out_x, ++out_y, ++out_z) {
			out_x[0] = points[vertexIndex].x;
			out_y[0] = points[vertexIndex].y;
			out_z[0] = points[vertexIndex].z;
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