// Copyright 2018 Luca Di Sera
//		Contact: disera.luca@gmail.com
//				 https://github.com/diseraluca
//				 https://www.linkedin.com/in/luca-di-sera-200023167
//
// This code is licensed under the MIT License. 
// More informations can be found in the LICENSE file in the root folder of this repository
//
//
// File : DeltaMush.cpp

#include "DeltaMush.h"

void * DeltaMush::creator()
{
	return nullptr;
}

MStatus DeltaMush::initialize()
{
	return MStatus();
}

MStatus DeltaMush::deform(MDataBlock & block, MItGeometry & iterator, const MMatrix & matrix, unsigned int multiIndex)
{
	return MStatus();
}
