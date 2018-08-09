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
#define _SCL_SECURE_NO_WARNINGS
#include "DeltaMush.h"

#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshVertex.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MMatrix.h>
#include <maya/MEvaluationNode.h>

#include <immintrin.h>

MString DeltaMush::typeName{ "ldsDeltaMush" };
MTypeId DeltaMush::typeId{ 0xd1230a };

MObject DeltaMush::referenceMesh;
MObject DeltaMush::rebindMesh;
MObject DeltaMush::smoothingIterations;
MObject DeltaMush::smoothWeight;
MObject DeltaMush::deltaWeight;


const unsigned int DeltaMush::MAX_NEIGHBOURS;
const unsigned int DeltaMush::DELTA_COUNT;

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
	
	double length{};
	double factor{};

	float envelopeValue{ block.inputValue(envelope).asFloat() };
	double deltaWeightValue{ block.inputValue(deltaWeight).asDouble() };

	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex, resultPositionsPtr += 4, vertexPositionsPtr += 4) {
		// resetting the delta vector
		deltaPtr[0] = 0.0;
		deltaPtr[1] = 0.0;
		deltaPtr[2] = 0.0;

		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < DELTA_COUNT; ++neighbourIndex) {

			// Calculate the vectors between the current vertex and two of its neighbours
			tangentPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			tangentPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			tangentPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			normalPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			normalPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			normalPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			// Normalizes the two vectors.
			// Vector normalization is calculated as follows:
			// lenght of the vector -> sqrt(x*x + y*y + z*z)
		    // [x, y, z] / length
			length = std::sqrt(tangentPtr[0] * tangentPtr[0] + tangentPtr[1] * tangentPtr[1] + tangentPtr[2] * tangentPtr[2]);

			factor = 1.0 / length;
			tangentPtr[0] *= factor;
			tangentPtr[1] *= factor;
			tangentPtr[2] *= factor;

			length = std::sqrt(normalPtr[0] * normalPtr[0] + normalPtr[1] * normalPtr[1] + normalPtr[2] * normalPtr[2]);

			factor = 1.0 / length;
			normalPtr[0] *= factor;
			normalPtr[1] *= factor;
			normalPtr[2] *= factor;

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
		factor = (1.0 / DELTA_COUNT);
		deltaPtr[0] *= factor;
		deltaPtr[1] *= factor;
		deltaPtr[2] *= factor;

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

void DeltaMush::decomposePointArray(const MPointArray & points, double * out_x, double * out_y, double * out_z, unsigned int vertexCount)
{
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex, ++out_x, ++out_y, ++out_z) {
		out_x[0] = points[vertexIndex].x;
		out_y[0] = points[vertexIndex].y;
		out_z[0] = points[vertexIndex].z;
	}
}

void DeltaMush::composePointArray(double * x, double * y, double * z, MPointArray & out_points, unsigned int vertexCount)
{
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex, ++x, ++y, ++z) {
		out_points[vertexIndex].x = x[0];
		out_points[vertexIndex].y = y[0];
		out_points[vertexIndex].z = z[0];
		out_points[vertexIndex].w = 1.0;
	}
}

MStatus DeltaMush::getNeighbours(MObject & mesh, unsigned int vertexCount)
{
	neighbours.resize(vertexCount * MAX_NEIGHBOURS);

	MItMeshVertex meshVtxIt{ mesh };
	MIntArray temporaryNeighbours{};
	int currentVertex{};
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex, meshVtxIt.next()) {
		meshVtxIt.getConnectedVertices(temporaryNeighbours);

		currentVertex = vertexIndex * MAX_NEIGHBOURS;
		if (temporaryNeighbours.length() >= MAX_NEIGHBOURS) {
			neighbours[currentVertex + 0] = temporaryNeighbours[0];
			neighbours[currentVertex + 1] = temporaryNeighbours[1];
			neighbours[currentVertex + 2] = temporaryNeighbours[2];
			neighbours[currentVertex + 3] = temporaryNeighbours[3];
		}
		else {
			for (unsigned int neighbourIndex{ 0 }; neighbourIndex < MAX_NEIGHBOURS; ++neighbourIndex) {
				if (neighbourIndex < temporaryNeighbours.length()) {
					neighbours[currentVertex + neighbourIndex] = temporaryNeighbours[neighbourIndex];
				}
				else {
					// With this we expect every vertex to have at least two neighbours
					neighbours[currentVertex + neighbourIndex] = neighbours[currentVertex +neighbourIndex - 2];
				}
			}
		}
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::averageSmoothing(const MPointArray & verticesPositions, MPointArray & out_smoothedPositions, unsigned int iterations, double weight)
{
	// TODO : RESOLVE ERROR FOR NON /40 VERTEX COUNT
	unsigned int vertexCount{ verticesPositions.length() };
	out_smoothedPositions.setLength(vertexCount);

	double* verticesX = new double[vertexCount];
	double* verticesY = new double[vertexCount];
	double* verticesZ = new double[vertexCount];
	decomposePointArray(verticesPositions, verticesX, verticesY, verticesZ, vertexCount);

	// A copy is necessary to avoid losing the original data trough the computations while working iteratively on the smoothed positions
	MPointArray verticesPositionsCopy{ verticesPositions };
	double* verticesCopyX = new double[vertexCount];
	std::copy(verticesX, verticesX + vertexCount, verticesCopyX);

	double* verticesCopyY = new double[vertexCount];
	std::copy(verticesY, verticesY + vertexCount, verticesCopyY);

	double* verticesCopyZ = new double[vertexCount];
	std::copy(verticesZ, verticesZ + vertexCount, verticesCopyZ);

	//Declaring the data needed by the loop
	MVector averagePosition{};
	__m256d averageX;
	__m256d averageY;
	__m256d averageZ;

	__m256d weighVector{ _mm256_set1_pd(weight) };
	double* averagePtr{ &averagePosition.x };
	const double*  vertexPtr{};
	const int* neighbourPtr{ &neighbours[0]};
	double averageFactor{};

	int currentVertex{};

	double* outSmoothedPositionsPtr{ &out_smoothedPositions[0].x };
	double* verticesPositionsCopyPtr{ &verticesPositionsCopy[0].x };

	for (unsigned int iterationIndex{ 0 }; iterationIndex < iterations; ++iterationIndex) {
		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4) {
			averageX = _mm256_setzero_pd();
			averageY = _mm256_setzero_pd();
			averageZ = _mm256_setzero_pd();

			neighbourPtr = &neighbours[vertexIndex * 4];
			for (unsigned int neighbourIndex{ 0 }; neighbourIndex < MAX_NEIGHBOURS; ++neighbourIndex, ++neighbourPtr) {
				__m256d neighboursX = _mm256_setr_pd(verticesCopyX[neighbourPtr[0]], verticesCopyX[neighbourPtr[0 + 4]], verticesCopyX[neighbourPtr[0 + 8]], verticesCopyX[neighbourPtr[0 + 12]]);
				__m256d neighboursY = _mm256_setr_pd(verticesCopyY[neighbourPtr[0]], verticesCopyY[neighbourPtr[0 + 4]], verticesCopyY[neighbourPtr[0 + 8]], verticesCopyY[neighbourPtr[0 + 12]]);
				__m256d neighboursZ = _mm256_setr_pd(verticesCopyZ[neighbourPtr[0]], verticesCopyZ[neighbourPtr[0 + 4]], verticesCopyZ[neighbourPtr[0 + 8]], verticesCopyZ[neighbourPtr[0 + 12]]);

				averageX = _mm256_add_pd(averageX, neighboursX);
				averageY = _mm256_add_pd(averageY, neighboursY);
				averageZ = _mm256_add_pd(averageZ, neighboursZ);
			}

			// Divides the accumulated vector to average it
			averageFactor = (1.0 / MAX_NEIGHBOURS);
			__m256d averageFactorVec = _mm256_set1_pd(averageFactor);

			averageX = _mm256_mul_pd(averageX, averageFactorVec);
			averageY = _mm256_mul_pd(averageY, averageFactorVec);
			averageZ = _mm256_mul_pd(averageZ, averageFactorVec);

			__m256d verticesCopyXVector = _mm256_load_pd(verticesCopyX + vertexIndex);
			__m256d verticesCopyYVector = _mm256_load_pd(verticesCopyY + vertexIndex);
			__m256d verticesCopyZVector = _mm256_load_pd(verticesCopyZ + vertexIndex);

			averageX = _mm256_sub_pd(averageX, verticesCopyXVector);
			averageY = _mm256_sub_pd(averageY, verticesCopyYVector);
			averageZ = _mm256_sub_pd(averageZ, verticesCopyZVector);

			averageX = _mm256_mul_pd(averageX, weighVector);
			averageY = _mm256_mul_pd(averageY, weighVector);
			averageZ = _mm256_mul_pd(averageZ, weighVector);

			averageX = _mm256_add_pd(averageX, verticesCopyXVector);
			averageY = _mm256_add_pd(averageY, verticesCopyYVector);
			averageZ = _mm256_add_pd(averageZ, verticesCopyZVector);

			_mm256_store_pd(verticesX + vertexIndex, averageX);
			_mm256_store_pd(verticesY + vertexIndex, averageY);
			_mm256_store_pd(verticesZ + vertexIndex, averageZ);
		}

		std::swap(verticesX, verticesCopyX);
		std::swap(verticesY, verticesCopyY);
		std::swap(verticesZ, verticesCopyZ);
	}

	composePointArray(verticesCopyX, verticesCopyY, verticesCopyZ, out_smoothedPositions, vertexCount);

	delete[] verticesX;
	delete[] verticesY;
	delete[] verticesZ;
	delete[] verticesCopyX;
	delete[] verticesCopyY;
	delete[] verticesCopyZ;

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

	MMatrix tangentSpaceMatrix{};
	double* tangentPtr{ &tangentSpaceMatrix.matrix[0][0] };
	double* normalPtr{ &tangentSpaceMatrix.matrix[1][0] };
	double* binormalPtr{ &tangentSpaceMatrix.matrix[2][0] };

	double length{};
	double factor{};

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex, vertexPositionsPtr += 4) {
		delta[0] = vertexPositionsPtr[0] - smoothedPositionsPtr[vertexIndex * 4];
		delta[1] = vertexPositionsPtr[1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
		delta[2] = vertexPositionsPtr[2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

		deltaMagnitudes[vertexIndex] = delta.length();

		deltas[vertexIndex].setLength(DELTA_COUNT);
		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < DELTA_COUNT; ++neighbourIndex) {
			// Calculate the vectors between the current vertex and two of its neighbours
			tangentPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			tangentPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			tangentPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			normalPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			normalPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			normalPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];

			length = std::sqrt(tangentPtr[0] * tangentPtr[0] + tangentPtr[1] * tangentPtr[1] + tangentPtr[2] * tangentPtr[2]);

			factor = 1.0 / length;
			tangentPtr[0] *= factor;
			tangentPtr[1] *= factor;
			tangentPtr[2] *= factor;

			length = std::sqrt(normalPtr[0] * normalPtr[0] + normalPtr[1] * normalPtr[1] + normalPtr[2] * normalPtr[2]);

			factor = 1.0 / length;
			normalPtr[0] *= factor;
			normalPtr[1] *= factor;
			normalPtr[2] *= factor;

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

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; ++vertexIndex) {
		perVertexWeights[vertexIndex] = weightValue(block, multiIndex, vertexIndex);
	}

	return MStatus::kSuccess;
}
