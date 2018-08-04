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
// Used when maya is in paraller or serial evaluation.
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

		// Calculate the deltas
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

	auto start = std::chrono::high_resolution_clock::now();
	// Declares the data needed for the loop
	MVector delta{};
	MVector tangent{};
	MVector neighbourVector{};
	MVector binormal{};
	MVector normal{};
	MMatrix tangentSpaceMatrix{};

	double *deltaPtr{ &delta.x };
	double* tangentPtr{ &tangent.x };
	double* neighbourVectorPtr{ &neighbourVector.x };

	double *smoothedPositionsPtr{ &meshSmoothedPositions[0].x };
	double averageFactor{};

	float envelopeValue{ block.inputValue(envelope).asFloat() };
	double deltaWeightValue{ block.inputValue(deltaWeight).asDouble() };

	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		// resetting the delta vector
		deltaPtr[0] = 0.0;
		deltaPtr[1] = 0.0;
		deltaPtr[2] = 0.0;

		unsigned int neighbourIterations{ neighbours[vertexIndex].length() - 1 };
		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < neighbourIterations; neighbourIndex++) {
			tangentPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			tangentPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			tangentPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			neighbourVectorPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			neighbourVectorPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			neighbourVectorPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex][neighbourIndex + 1] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			tangent.normalize();
			neighbourVector.normalize();

			// Ensures  axis orthogonality
			binormal = tangent ^ neighbourVector;
			normal = tangent ^ binormal;

			// Build Tangent Space Matrix
			tangentSpaceMatrix[0][0] = tangentPtr[0];
			tangentSpaceMatrix[0][1] = tangentPtr[1];
			tangentSpaceMatrix[0][2] = tangentPtr[2];
			tangentSpaceMatrix[0][3] = 0.0;

			tangentSpaceMatrix[1][0] = normal.x;
			tangentSpaceMatrix[1][1] = normal.y;
			tangentSpaceMatrix[1][2] = normal.z;
			tangentSpaceMatrix[1][3] = 0.0;

			tangentSpaceMatrix[2][0] = binormal.x;
			tangentSpaceMatrix[2][1] = binormal.y;
			tangentSpaceMatrix[2][2] = binormal.z;
			tangentSpaceMatrix[2][3] = 0.0;

			tangentSpaceMatrix[3][0] = 0.0;
			tangentSpaceMatrix[3][1] = 0.0;
			tangentSpaceMatrix[3][2] = 0.0;
			tangentSpaceMatrix[3][3] = 1.0;

			// Accumulate the displacement Vectors
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


		resultPositions[vertexIndex] = meshSmoothedPositions[vertexIndex] + delta;

		// We calculate the new definitive delta and apply the remaining scaling factors to it
		delta = resultPositions[vertexIndex] - meshVertexPositions[vertexIndex];

		resultPositions[vertexIndex] = meshVertexPositions[vertexIndex] + (delta * perVertexWeights[vertexIndex] * envelopeValue);
	}

	auto end = std::chrono::high_resolution_clock::now();
	time += (end - start);
	++counter;
	if (counter >= 100) {
		std::cout << this->name() << ":" << std::chrono::duration_cast<std::chrono::microseconds>(time).count() << std::endl;
		time = time.zero();
		counter = 0;
	}
	iterator.setAllPositions(resultPositions);

	return MStatus::kSuccess;
}

MStatus DeltaMush::getNeighbours(MObject & mesh, unsigned int vertexCount)
{
	neighbours.resize(vertexCount);

	MItMeshVertex meshVtxIt{ mesh };
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++, meshVtxIt.next()) {
		CHECK_MSTATUS_AND_RETURN_IT(meshVtxIt.getConnectedVertices(neighbours[vertexIndex]));
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

			int currentVertex = vertexIndex * 4;
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

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		MVector delta{ vertexPositions[vertexIndex] - smoothedPositions[vertexIndex] };
		deltaMagnitudes[vertexIndex] = delta.length();

		unsigned int neighbourIterations{ neighbours[vertexIndex].length() - 1 };
		deltas[vertexIndex].setLength(neighbourIterations);
		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < neighbourIterations; neighbourIndex++) {
			MVector tangent = smoothedPositions[neighbours[vertexIndex][neighbourIndex]] - smoothedPositions[vertexIndex];
			MVector neighbourVerctor = smoothedPositions[neighbours[vertexIndex][neighbourIndex + 1]] - smoothedPositions[vertexIndex];

			tangent.normalize();
			neighbourVerctor.normalize();

			MVector binormal{ tangent ^ neighbourVerctor };
			MVector normal{ tangent ^ binormal };

			// Build Tangent Space Matrix
			MMatrix tangentSpaceMatrix{};
			buildTangentSpaceMatrix(tangentSpaceMatrix, tangent, normal, binormal);

			// Calculate the displacement Vector
			deltas[vertexIndex][neighbourIndex] = tangentSpaceMatrix.inverse() * delta;
		}
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::buildTangentSpaceMatrix(MMatrix & out_TangetSpaceMatrix, const MVector & tangent, const MVector & normal, const MVector & binormal) const
{
	// M = [tangent, normal, bitangent, translation(smoothedPosition]]
	out_TangetSpaceMatrix[0][0] = tangent.x;
	out_TangetSpaceMatrix[0][1] = tangent.y;
	out_TangetSpaceMatrix[0][2] = tangent.z;
	out_TangetSpaceMatrix[0][3] = 0.0;

	out_TangetSpaceMatrix[1][0] = normal.x;
	out_TangetSpaceMatrix[1][1] = normal.y;
	out_TangetSpaceMatrix[1][2] = normal.z;
	out_TangetSpaceMatrix[1][3] = 0.0;

	out_TangetSpaceMatrix[2][0] = binormal.x;
	out_TangetSpaceMatrix[2][1] = binormal.y;
	out_TangetSpaceMatrix[2][2] = binormal.z;
	out_TangetSpaceMatrix[2][3] = 0.0;

	out_TangetSpaceMatrix[3][0] = 0.0;
	out_TangetSpaceMatrix[3][1] = 0.0;
	out_TangetSpaceMatrix[3][2] = 0.0;
	out_TangetSpaceMatrix[3][3] = 1.0;

	return MStatus::kSuccess;
}

MStatus DeltaMush::getPerVertexWeights(MDataBlock & block, unsigned int multiIndex, unsigned int vertexCount)
{
	perVertexWeights.resize(vertexCount);

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		perVertexWeights[vertexIndex] = weightValue(block, multiIndex, vertexIndex);
	}

	return MStatus::kSuccess;
}
