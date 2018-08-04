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

#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshVertex.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MMatrix.h>
#include <maya/MEvaluationNode.h>

MString DeltaMush::typeName{ "ldsDeltaMush" };
MTypeId DeltaMush::typeId{ 0xd1230a };

MObject DeltaMush::referenceMesh;
MObject DeltaMush::rebindMesh;
MObject DeltaMush::smoothingIterations;
MObject DeltaMush::smoothWeight;
MObject DeltaMush::deltaWeight;

DeltaMush::DeltaMush()
	:isInitialized{ false },
	 neighbours{},
	 deltas{},
	 deltaMagnitudes{},
	 perVertexWeights{}
{
}

void * DeltaMush::creator()
{
	return new DeltaMush();
}

MStatus DeltaMush::initialize()
{
	MStatus status{};

	MFnTypedAttribute   tAttr;
	MFnNumericAttribute nAttr;

	referenceMesh = tAttr.create("referenceMesh", "ref", MFnData::kMesh, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS(addAttribute(referenceMesh));

	rebindMesh = nAttr.create("rebindMesh", "rbm", MFnNumericData::kBoolean, false, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS(nAttr.setKeyable(true));
	CHECK_MSTATUS(addAttribute(rebindMesh));

	smoothingIterations = nAttr.create("smoothingIterations", "smi", MFnNumericData::kInt, 1, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS(nAttr.setKeyable(true));
	CHECK_MSTATUS(nAttr.setMin(1));
	CHECK_MSTATUS(addAttribute(smoothingIterations));

	smoothWeight = nAttr.create("smoothWeight", "smw", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS(nAttr.setKeyable(true));
	CHECK_MSTATUS(nAttr.setMin(0.0));
	CHECK_MSTATUS(nAttr.setMax(1.0));
	CHECK_MSTATUS(addAttribute(smoothWeight));

	deltaWeight = nAttr.create("deltaWeight", "dlw", MFnNumericData::kDouble, 1.0, &status);
	CHECK_MSTATUS_AND_RETURN_IT(status);
	CHECK_MSTATUS(nAttr.setKeyable(true));
	CHECK_MSTATUS(nAttr.setMin(0.0));
	CHECK_MSTATUS(nAttr.setMax(1.0));
	CHECK_MSTATUS(addAttribute(deltaWeight));

	CHECK_MSTATUS(attributeAffects(referenceMesh, outputGeom));
	CHECK_MSTATUS(attributeAffects(rebindMesh, outputGeom));
	CHECK_MSTATUS(attributeAffects(smoothingIterations, outputGeom));
	CHECK_MSTATUS(attributeAffects(smoothWeight, outputGeom));
	CHECK_MSTATUS(attributeAffects(deltaWeight, outputGeom));

	MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer ldsDeltaMush weights");

	return MStatus::kSuccess;
}

// Set the data initialization to be rerun if the smoothingIteration or referenceMesh values change.
// Used when maya is in DG evaluation.
MStatus DeltaMush::setDependentsDirty(const MPlug & plug, MPlugArray & plugArray)
{
	if (plug == smoothingIterations || plug == referenceMesh) {
		isInitialized = false;
	}

	return MPxNode::setDependentsDirty(plug, plugArray);
}

// Set the data initialization to be rerun if the smoothingIteration or referenceMesh values change.
// Used when maya is in parallel or serial evaluation.
MStatus DeltaMush::preEvaluation(const MDGContext & context, const MEvaluationNode & evaluationNode)
{
	MStatus status{};

	if (!context.isNormal()) {
		return MStatus::kFailure;
	}

	if ((evaluationNode.dirtyPlugExists(smoothingIterations, &status) && status) ||
		(evaluationNode.dirtyPlugExists(referenceMesh, &status) && status)) 
	{
		isInitialized = false;
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::deform(MDataBlock & block, MItGeometry & iterator, const MMatrix & matrix, unsigned int multiIndex)
{
	bool rebindMeshValue{ block.inputValue(rebindMesh).asBool() };
	int smoothingIterationsValue{ block.inputValue(smoothingIterations).asInt() };
	double smoothWeightValue{ block.inputValue(smoothWeight).asDouble() };

	int vertexCount{ iterator.count() };
	if (rebindMeshValue || !isInitialized) {
		// If a referenceMesh is not provided we get out
		MPlug referenceMeshPlug{ thisMObject(), referenceMesh };
		if (!referenceMeshPlug.isConnected()) {
			MGlobal::displayWarning(this->name() + ": referenceMesh is not connected. Please connect a mesh");
			return MStatus::kUnknownParameter;
		}

		// Retrieves the positions for the reference mesh
		MObject referenceMeshValue{ block.inputValue(referenceMesh).asMesh() };
		MFnMesh referenceMeshFn{ referenceMeshValue };

		MPointArray referenceMeshVertexPositions{};
		referenceMeshVertexPositions.setLength(vertexCount);
		referenceMeshFn.getPoints(referenceMeshVertexPositions);

		// Build the neighbours array 
		getNeighbours(referenceMeshValue, vertexCount);

		// Calculate the smoothed positions for the reference mesh
		MPointArray referenceMeshSmoothedPositions{};
		averageSmoothing(referenceMeshVertexPositions, referenceMeshSmoothedPositions, smoothingIterationsValue, smoothWeightValue);

		// Calculate and cache the deltas
		cacheDeltas(referenceMeshVertexPositions, referenceMeshSmoothedPositions, vertexCount);

		isInitialized = true;
	}

	MPointArray meshVertexPositions{};
	iterator.allPositions(meshVertexPositions);

	// Caculate the smoothed positions for the deformed mesh
	MPointArray meshSmoothedPositions{};
	averageSmoothing(meshVertexPositions, meshSmoothedPositions, smoothingIterationsValue, smoothWeightValue);

	// Apply the deltas
	MPointArray resultPositions{};
	resultPositions.setLength(vertexCount);

	// Store the current values of the per-vertex weights
	getPerVertexWeights(block, multiIndex, vertexCount);

	// Declares the data needed for the loop
	MVector delta{};
	double *deltaPtr{ &delta.x };

	// We construct our TNB vectors directly on the matrix memory
	// to avoid the need of copying the values later and avoid using MVectors.
	// Default constructor initializes to 4x4 identity matrix meaning we do not have to
	// modify any part of the matrix apart from [1,2,3][0:2]
	MMatrix tangentSpaceMatrix{};
	double* tangentPtr{ &tangentSpaceMatrix.matrix[0][0] };
	double* normalPtr{ &tangentSpaceMatrix.matrix[1][0] };
	double* binormalPtr{ &tangentSpaceMatrix.matrix[2][0] };

	double* vertexPositionsPtr{ &meshVertexPositions[0].x };
	double *smoothedPositionsPtr{ &meshSmoothedPositions[0].x };
	double *resultPositionsPtr{ &resultPositions[0].x };
	
	double averageFactor{};
	unsigned int neighbourIterations{};

	float envelopeValue{ block.inputValue(envelope).asFloat() };
	double deltaWeightValue{ block.inputValue(deltaWeight).asDouble() };

	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++, resultPositionsPtr += 4, vertexPositionsPtr += 4) {
		// resetting the delta vector
		deltaPtr[0] = 0.0;
		deltaPtr[1] = 0.0;
		deltaPtr[2] = 0.0;

		neighbourIterations = neighbours[vertexIndex].length() - 1;
		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < neighbourIterations; neighbourIndex++) {

			// Calculate the vectors between the current vertex and two of its neighbours
			tangentPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			tangentPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			tangentPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			normalPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			normalPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			normalPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			// TODO : remove the unsafe casting and provide a custom normalization
			((MVector*)(tangentPtr))->normalize();
			((MVector*)(normalPtr))->normalize();

			// Ensures  axis orthogonality through cross product.
			// Cross product is calculated in the following code as:
			// crossVector = [(y1 * z2 - z1 * y2), (z1 * x2 - x1 * z2), (x1 * y2 - y1 * x2)]
			binormalPtr[0] = tangentPtr[1] * normalPtr[2] - tangentPtr[2] * normalPtr[1];
			binormalPtr[1] = tangentPtr[2] * normalPtr[0] - tangentPtr[0] * normalPtr[2];
			binormalPtr[2] = tangentPtr[0] * normalPtr[1] - tangentPtr[1] * normalPtr[0];

			normalPtr[0] = tangentPtr[1] * binormalPtr[2] - tangentPtr[2] * binormalPtr[1];
			normalPtr[1] = tangentPtr[2] * binormalPtr[0] - tangentPtr[0] * binormalPtr[2];
			normalPtr[2] = tangentPtr[0] * binormalPtr[1] - tangentPtr[1] * binormalPtr[0];

			// Accumulate the displacement Vectors
			// TODO : Provide a custom matrix*vector implementation to remove the intermediate MVector and transform the MMatrix into a simpler double[4][4]
			MVector tangentSpaceDelta{ tangentSpaceMatrix * deltas[vertexIndex][neighbourIndex] };
			deltaPtr[0] += tangentSpaceDelta.x;
			deltaPtr[1] += tangentSpaceDelta.y;
			deltaPtr[2] += tangentSpaceDelta.z;
		}

		// Averaging the delta
		averageFactor = (1.0 / neighbourIterations);
		deltaPtr[0] *= neighbourIterations;
		deltaPtr[1] *= neighbourIterations;
		deltaPtr[2] *= neighbourIterations;

		// Scaling the delta
		delta.normalize();
		deltaPtr[0] *= (deltaMagnitudes[vertexIndex] * deltaWeightValue);
		deltaPtr[1] *= (deltaMagnitudes[vertexIndex] * deltaWeightValue);
		deltaPtr[2] *= (deltaMagnitudes[vertexIndex] * deltaWeightValue);

		// Finding the final position
		resultPositionsPtr[0] = smoothedPositionsPtr[vertexIndex * 4] + deltaPtr[0];
		resultPositionsPtr[1] = smoothedPositionsPtr[vertexIndex * 4 + 1] + deltaPtr[1];
		resultPositionsPtr[2] = smoothedPositionsPtr[vertexIndex * 4 + 2] + deltaPtr[2];
		resultPositionsPtr[3] = 1.0;

		// We calculate the new definitive delta
		deltaPtr[0] = resultPositionsPtr[0] - vertexPositionsPtr[0];
		deltaPtr[1] = resultPositionsPtr[1] - vertexPositionsPtr[1];
		deltaPtr[2] = resultPositionsPtr[2] - vertexPositionsPtr[2];

		// Setting the weighted final position
		resultPositionsPtr[0] = vertexPositionsPtr[0] + (deltaPtr[0] * perVertexWeights[vertexIndex] * envelopeValue);
		resultPositionsPtr[1] = vertexPositionsPtr[1] + (deltaPtr[1] * perVertexWeights[vertexIndex] * envelopeValue);
		resultPositionsPtr[2] = vertexPositionsPtr[2] + (deltaPtr[2] * perVertexWeights[vertexIndex] * envelopeValue);
	}

	iterator.setAllPositions(resultPositions);

	return MStatus::kSuccess;
}

MStatus DeltaMush::getNeighbours(MObject & mesh, unsigned int vertexCount)
{
	neighbours.resize(vertexCount);

	MItMeshVertex meshVtxIt{ mesh };
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++, meshVtxIt.next()) {
		meshVtxIt.getConnectedVertices(neighbours[vertexIndex]);
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::averageSmoothing(const MPointArray & verticesPositions, MPointArray & out_smoothedPositions, unsigned int iterations, double weight)
{
	unsigned int vertexCount{ verticesPositions.length() };
	out_smoothedPositions.setLength(vertexCount);

	// A copy is necessary to avoid losing the original data trough the computations while working iteratively on the smoothed positions
	MPointArray verticesPositionsCopy{ verticesPositions };

	//Declaring the data needed by the loop
	MVector averagePosition{};
	unsigned int neighbourCount{};

	double* averagePtr{ &averagePosition.x };
	const double*  vertexPtr{};
	const int* neighbourPtr{};
	double averageFactor{};

	int currentVertex{};

	double* outSmoothedPositionsPtr{ &out_smoothedPositions[0].x };
	double* verticesPositionsCopyPtr{ &verticesPositionsCopy[0].x };

	for (unsigned int iterationIndex{ 0 }; iterationIndex < iterations; iterationIndex++) {

		// Inrementing the pointer by four makes us jumps four double ( x, y, z , w ) positioning us on the next MPoint members
		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
			neighbourCount = neighbours[vertexIndex].length();

			//resetting the vector
			averagePtr[0] = 0.0;
			averagePtr[1] = 0.0;
			averagePtr[2] = 0.0;

			neighbourPtr = &neighbours[vertexIndex][0];
			for (unsigned int neighbourIndex{ 0 }; neighbourIndex < neighbourCount; neighbourIndex++, neighbourPtr++) {
				vertexPtr = verticesPositionsCopyPtr + (neighbourPtr[0] * 4);

				averagePtr[0] += vertexPtr[0];
				averagePtr[1] += vertexPtr[1];
				averagePtr[2] += vertexPtr[2];
			}

			// Divides the accumulated vector to average it
			averageFactor = (1.0 / neighbourCount);
			averagePtr[0] *= averageFactor;
			averagePtr[1] *= averageFactor;
			averagePtr[2] *= averageFactor;

			// Store the final weighted position
			currentVertex = vertexIndex * 4;
			outSmoothedPositionsPtr[currentVertex] = ((averagePtr[0] - verticesPositionsCopyPtr[currentVertex]) * weight) + verticesPositionsCopyPtr[currentVertex];
			outSmoothedPositionsPtr[currentVertex + 1] = ((averagePtr[1] - verticesPositionsCopyPtr[currentVertex + 1]) * weight) + verticesPositionsCopyPtr[currentVertex + 1];
			outSmoothedPositionsPtr[currentVertex + 2] = ((averagePtr[2] - verticesPositionsCopyPtr[currentVertex + 2]) * weight) + verticesPositionsCopyPtr[currentVertex + 2];
			outSmoothedPositionsPtr[currentVertex + 3] = 1.0;
		}

		std::swap(outSmoothedPositionsPtr, verticesPositionsCopyPtr);
	}

	// If the number of iterations is even we have to copy the updated data in out__smoothed positions 
	if ((iterations % 2) == 0) {
		out_smoothedPositions.copy(verticesPositionsCopy);
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::cacheDeltas(const MPointArray & vertexPositions, const MPointArray & smoothedPositions, unsigned int vertexCount)
{
	deltas.resize(vertexCount);
	deltaMagnitudes.resize(vertexCount);

	// Declare the data needed by the loop
	MVector delta{};
	double *deltaPtr{ &delta.x };

	const double *vertexPositionsPtr{ &vertexPositions[0].x };
	const double *smoothedPositionsPtr{ &smoothedPositions[0].x };

	unsigned int neighbourIterations{};

	MMatrix tangentSpaceMatrix{};
	double* tangentPtr{ &tangentSpaceMatrix.matrix[0][0] };
	double* normalPtr{ &tangentSpaceMatrix.matrix[1][0] };
	double* binormalPtr{ &tangentSpaceMatrix.matrix[2][0] };

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++, vertexPositionsPtr += 4) {
		delta[0] = vertexPositionsPtr[0] - smoothedPositionsPtr[vertexIndex * 4];
		delta[1] = vertexPositionsPtr[1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
		delta[2] = vertexPositionsPtr[2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

		deltaMagnitudes[vertexIndex] = delta.length();

		neighbourIterations = neighbours[vertexIndex].length() - 1;
		deltas[vertexIndex].setLength(neighbourIterations);
		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < neighbourIterations; neighbourIndex++) {
			// Calculate the vectors between the current vertex and two of its neighbours
			tangentPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			tangentPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			tangentPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			normalPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			normalPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			normalPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			// TODO : remove the unsafe casting and provide a custom normalization
			((MVector*)(tangentPtr))->normalize();
			((MVector*)(normalPtr))->normalize();

			// Ensures  axis orthogonality through cross product.
			// Cross product is calculated in the following code as:
			// crossVector = [(y1 * z2 - z1 * y2), (z1 * x2 - x1 * z2), (x1 * y2 - y1 * x2)]
			binormalPtr[0] = tangentPtr[1] * normalPtr[2] - tangentPtr[2] * normalPtr[1];
			binormalPtr[1] = tangentPtr[2] * normalPtr[0] - tangentPtr[0] * normalPtr[2];
			binormalPtr[2] = tangentPtr[0] * normalPtr[1] - tangentPtr[1] * normalPtr[0];

			normalPtr[0] = tangentPtr[1] * binormalPtr[2] - tangentPtr[2] * binormalPtr[1];
			normalPtr[1] = tangentPtr[2] * binormalPtr[0] - tangentPtr[0] * binormalPtr[2];
			normalPtr[2] = tangentPtr[0] * binormalPtr[1] - tangentPtr[1] * binormalPtr[0];

			// Calculate the displacement Vector
			deltas[vertexIndex][neighbourIndex] = tangentSpaceMatrix.inverse() * delta;
		}
	}

	return MStatus::kSuccess;
}


MStatus DeltaMush::getPerVertexWeights(MDataBlock & block, unsigned int multiIndex, unsigned int vertexCount)
{
	// TODO : Get the values manually by handle
	perVertexWeights.resize(vertexCount);

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		perVertexWeights[vertexIndex] = weightValue(block, multiIndex, vertexIndex);
	}

	return MStatus::kSuccess;
}
