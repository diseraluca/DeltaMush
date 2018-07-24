// Copyright 2018 Luca Di Sera
//		Contact: disera.luca@gmail.com
//				 https://github.com/diseraluca
//				 https://www.linkedin.com/in/luca-di-sera-200023167
//
// This code is licensed under the MIT License. 
// More informations can be found in the LICENSE file in the root folder of this repository
//
//
// File : DeltaMush.h
//
// The DeltaMush class is a custom deformer for Autodesk Maya that implements
// the Delta Mush smoothing algorithm { "Delta Mush: smoothing deformations while preserving detail" - Joe Mancewicz, Matt L.Derksen, StudiosHans Rijpkema, StudiosCyrus A.Wilson }.
// This node will perform a Delta Mush smoothing that smooths a mesh while preventing the loss of volume and details.
// Used to help and speed up the skinning of rigs while giving high level and fast deformations.

#pragma once

#include <maya/MPxDeformerNode.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MVector.h>

#include <vector>

class DeltaMush : public MPxDeformerNode {
public:
	static void*    creator();
	static MStatus  initialize();
	virtual MStatus deform(MDataBlock & block, MItGeometry & iterator, const MMatrix & matrix, unsigned int multiIndex) override;

private:
	MStatus averageSmoothing(const MPointArray& verticesPositions, MPointArray& out_smoothedPositions, const std::vector<MIntArray>& neighbours, unsigned int iterations, double weight);
	MVector neighboursAveragePosition(const MPointArray& verticesPositions, const std::vector<MIntArray>& neighbours, unsigned int vertexIndex);

	MStatus buildTangentSpaceMatrix(MMatrix& out_TangetSpaceMatrix, const MVector& tangent, const MVector& normal, const MVector& binormal, const MVector& translation) const;

public:
	static MString typeName;
	static MTypeId typeId;

	static MObject referenceMesh;
	static MObject smoothingIterations;
	static MObject smoothWeight;
};