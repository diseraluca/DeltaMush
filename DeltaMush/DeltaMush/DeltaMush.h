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
// This implementation of the deformer requires the meshes to have a correctly unwrapped UV set to function properly.

#pragma once

#include <maya/MPxDeformerNode.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MVector.h>
#include <maya/MFnMesh.h>

#include <vector>

class DeltaMush : public MPxDeformerNode {
public:
	static void*    creator();
	static MStatus  initialize();
	virtual MStatus deform(MDataBlock & block, MItGeometry & iterator, const MMatrix & matrix, unsigned int multiIndex) override;

private:
	// Get the neighbours vertices per-vertex of mesh. The neighbours indexes are stored into out_neighbours
	MStatus getNeighbours(MObject& mesh, std::vector<MIntArray>& out_neighbours, unsigned int vertexCount) const;

	// Perform an average neighbour smoothing on the vertices in vertices position and stores the smoothedPositions in out_smoothedPositions.
	MStatus averageSmoothing(const MPointArray& verticesPositions, MPointArray& out_smoothedPositions, const std::vector<MIntArray>& neighbours, unsigned int iterations, double weight) const;

	// Calculates and return an MVector representing the average positions of the neighbours vertices of the vertex with ID = vertexIndex 
	MVector neighboursAveragePosition(const MPointArray& verticesPositions, const std::vector<MIntArray>& neighbours, unsigned int vertexIndex) const;

	// Calculate the tangent space deltas between the smoothed positions and the original positions and stores them in out_deltas.
	MStatus cacheDeltas(const MPointArray& vertexPositions, const MPointArray& smoothedPositions, const std::vector<MIntArray>& neighbours, MVectorArray& out_deltas, unsigned int vertexCount) const;

	// Calculate a per-vertex tangent space and apply the delta per-vertex to find the final position that is stored in out_positions
	MStatus applyDeltas(const MFnMesh& meshFn, const MPointArray& smoothedPositions, MVectorArray& deltas, MPointArray& out_positions, double weight, unsigned int vertexCount) const;
	MStatus buildTangentSpaceMatrix(MMatrix& out_TangetSpaceMatrix, const MVector& tangent, const MVector& normal, const MVector& binormal) const;

	// Safely get and MObject handle to the inputGeom[multiIndex] attribute 
	MObject getInputGeom(MDataBlock& block, unsigned int multiIndex);

public:
	static MString typeName;
	static MTypeId typeId;

	static MObject referenceMesh;
	static MObject smoothingIterations;
	static MObject smoothWeight;
	static MObject deltaWeight;
};