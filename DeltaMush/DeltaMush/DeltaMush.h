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
// This implementation of the deformer requires a reference mesh that is an exact rest-pose copy of the deformed mesh.

#pragma once

#include <maya/MPxDeformerNode.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MVector.h>
#include <maya/MFnMesh.h>

#include <vector>

class DeltaMush : public MPxDeformerNode {
public:
	DeltaMush();

	static void*    creator();
	static MStatus  initialize();
	virtual MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) override;
	virtual MStatus preEvaluation(const MDGContext & context, const MEvaluationNode & evaluationNode) override;
	virtual MStatus deform(MDataBlock & block, MItGeometry & iterator, const MMatrix & matrix, unsigned int multiIndex) override;

private:
	// Get the neighbours vertices per-vertex of mesh. Stores them in the this->neighbours
	MStatus getNeighbours(MObject& mesh, unsigned int vertexCount);

	// Perform an average neighbour smoothing on the vertices in vertices position and stores the smoothedPositions in out_smoothedPositions.
	MStatus averageSmoothing(const MPointArray& verticesPositions, MPointArray& out_smoothedPositions, unsigned int iterations, double weight);

	// Calculate the tangent space deltas between the smoothed positions and the original positions. Initializes this->deltas and stores
	// the resulting deltas in it. Furthermore it initializes this->magnitude and store the respective magnitude in it.
	MStatus cacheDeltas(const MPointArray& vertexPositions, const MPointArray& smoothedPositions, unsigned int vertexCount);
	MStatus buildTangentSpaceMatrix(MMatrix& out_TangetSpaceMatrix, const MVector& tangent, const MVector& normal, const MVector& binormal) const;

	// Retrieves the per-vertex weight of every vertex and stores them in this->perVertexWeight
	MStatus getPerVertexWeights(MDataBlock& block, unsigned int multiIndex, unsigned int vertexCount);

public:
	static MString typeName;
	static MTypeId typeId;

	static MObject referenceMesh;
	static MObject rebindMesh;
	static MObject smoothingIterations;
	static MObject smoothWeight;
	static MObject deltaWeight;

private:
	bool isInitialized;

	std::vector<MIntArray> neighbours;
	std::vector<MVectorArray> deltas;
	std::vector<double> deltaMagnitudes;
	std::vector<float> perVertexWeights;
};