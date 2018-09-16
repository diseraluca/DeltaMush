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

#include "ComponentVector256d.h"
#include "ComponentVector256dMatrix3x3.h"
#include "MPointArrayUtils.h"

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


const unsigned int DeltaMush::MAX_NEIGHBOURS{ 4 };
const unsigned int DeltaMush::DELTA_COUNT{ MAX_NEIGHBOURS - 1 };
const double DeltaMush::AVERAGE_FACTOR{ 1.0 / MAX_NEIGHBOURS };

constexpr double DELTA_AVERAGE_FACTOR{ 1.0 / DeltaMush::DELTA_COUNT };

DeltaMush::DeltaMush()
	:isInitialized{ false },
	 paddedCount{ 0 },
	 verticesX{},
	 verticesY{},
	 verticesZ{},
	 neighbours{},
	 deltasX{},
	 deltasY{},
	 deltasZ{},
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

		unsigned int countModulo{ vertexCount % MAX_NEIGHBOURS };
		if (countModulo != 0) {
			paddedCount = vertexCount + (MAX_NEIGHBOURS - countModulo);
		}
		else {
			paddedCount = vertexCount;
		}

		// Retrieves the positions for the reference mesh
		MObject referenceMeshValue{ block.inputValue(referenceMesh).asMesh() };
		MFnMesh referenceMeshFn{ referenceMeshValue };

		MPointArray referenceMeshVertexPositions{};
		referenceMeshVertexPositions.setLength(vertexCount);
		referenceMeshFn.getPoints(referenceMeshVertexPositions);

		verticesX.resize(paddedCount);
		verticesY.resize(paddedCount);
		verticesZ.resize(paddedCount);

		MPointArrayUtils::decomposePointArray(referenceMeshVertexPositions, verticesX.data(), verticesY.data(), verticesZ.data(), vertexCount);

		smoothedX.resize(paddedCount);
		smoothedY.resize(paddedCount);
		smoothedZ.resize(paddedCount);

		// Build the neighbours array 
		getNeighbours(referenceMeshValue, vertexCount);

		// Calculate the smoothed positions for the reference mesh
		averageSmoothing(smoothingIterationsValue, smoothWeightValue, vertexCount);

		// Calculate and cache the deltas
		cacheDeltas(vertexCount);

		isInitialized = true;
	}

	MPointArray meshVertexPositions{};
	iterator.allPositions(meshVertexPositions);
	MPointArrayUtils::decomposePointArray(meshVertexPositions, verticesX.data(), verticesY.data(), verticesZ.data(), vertexCount);

	// Caculate the smoothed positions for the deformed mesh
	averageSmoothing(smoothingIterationsValue, smoothWeightValue, vertexCount);

	// Apply the deltas
	std::vector<double> resultsX{};
	resultsX.resize(paddedCount);
	std::vector<double> resultsY{};
	resultsY.resize(paddedCount);
	std::vector<double> resultsZ{};
	resultsZ.resize(paddedCount);

	// Store the current values of the per-vertex weights
	getPerVertexWeights(block, multiIndex, vertexCount);

	// Declares the data needed for the loop
	double length{};
	double factor{};

	float envelopeValue{ block.inputValue(envelope).asFloat() };
	double deltaWeightValue{ block.inputValue(deltaWeight).asDouble() };
	ComponentVector256d delta{};
	ComponentVector256dMatrix3x3 tangentSpaceMatrix{};

	double * deltaXPtr{ &deltasX[0] };
	double * deltaYPtr{ &deltasY[0] };
	double * deltaZPtr{ &deltasZ[0] };

	int* neighbourPtr{ &neighbours[0] };

	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4, neighbourPtr += 13) {
		delta.setZero();

		ComponentVector256d smoothedPositions{ smoothedX.data() + vertexIndex, smoothedY.data() + vertexIndex, smoothedZ.data() + vertexIndex };

		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < DELTA_COUNT; ++neighbourIndex, ++neighbourPtr, deltaXPtr += 4, deltaYPtr += 4, deltaZPtr += 4) {
			ComponentVector256d neighbourPositions{
				_mm256_setr_pd(smoothedX[neighbourPtr[0]], smoothedX[neighbourPtr[0 + 4]], smoothedX[neighbourPtr[0 + 8]], smoothedX[neighbourPtr[0 + 12]]),
				_mm256_setr_pd(smoothedY[neighbourPtr[0]], smoothedY[neighbourPtr[0 + 4]], smoothedY[neighbourPtr[0 + 8]], smoothedY[neighbourPtr[0 + 12]]),
				_mm256_setr_pd(smoothedZ[neighbourPtr[0]], smoothedZ[neighbourPtr[0 + 4]], smoothedZ[neighbourPtr[0 + 8]], smoothedZ[neighbourPtr[0 + 12]])
			};

			tangentSpaceMatrix.tangent = neighbourPositions - smoothedPositions;
			tangentSpaceMatrix.tangent.normalize();

			neighbourPositions.set(
				_mm256_setr_pd(smoothedX[neighbourPtr[1]], smoothedX[neighbourPtr[1 + 4]], smoothedX[neighbourPtr[1 + 8]], smoothedX[neighbourPtr[1 + 12]]),
				_mm256_setr_pd(smoothedY[neighbourPtr[1]], smoothedY[neighbourPtr[1 + 4]], smoothedY[neighbourPtr[1 + 8]], smoothedY[neighbourPtr[1 + 12]]),
				_mm256_setr_pd(smoothedZ[neighbourPtr[1]], smoothedZ[neighbourPtr[1 + 4]], smoothedZ[neighbourPtr[1 + 8]], smoothedZ[neighbourPtr[1 + 12]])
			);

			tangentSpaceMatrix.normal = neighbourPositions - smoothedPositions;
			tangentSpaceMatrix.normal.normalize();

			tangentSpaceMatrix.binormal = tangentSpaceMatrix.tangent ^ tangentSpaceMatrix.normal;
			tangentSpaceMatrix.normal = tangentSpaceMatrix.tangent ^ tangentSpaceMatrix.binormal;

			ComponentVector256d cachedDelta{ deltaXPtr, deltaYPtr, deltaZPtr };
			ComponentVector256d tangentSpaceDelta{ tangentSpaceMatrix * cachedDelta };

			delta += tangentSpaceDelta;
		}

		delta *= _mm256_set1_pd(DELTA_AVERAGE_FACTOR);
		delta.normalize();

		delta *= _mm256_mul_pd(_mm256_load_pd(&deltaMagnitudes[vertexIndex]), _mm256_set1_pd(deltaWeightValue));

		ComponentVector256d resultPositions{ delta + smoothedPositions };
		ComponentVector256d vertexPositions{ verticesX.data() + vertexIndex, verticesY.data() + vertexIndex, verticesZ.data() + vertexIndex };

		delta = resultPositions - vertexPositions;

		__m128 globalWeightsF{ _mm_load_ps(&perVertexWeights[vertexIndex]) };
		globalWeightsF = _mm_mul_ps(globalWeightsF, _mm_set1_ps(envelopeValue));

		__m256d globalWeights = _mm256_cvtps_pd(globalWeightsF);

		resultPositions = vertexPositions + (delta * globalWeights);
		resultPositions.store(&resultsX[vertexIndex], &resultsY[vertexIndex], &resultsZ[vertexIndex]);
	}

	MPointArray resultPositions{};
	resultPositions.setLength(vertexCount);
	MPointArrayUtils::composePointArray(&resultsX[0], &resultsY[0], &resultsZ[0], resultPositions, vertexCount);
	iterator.setAllPositions(resultPositions);

	return MStatus::kSuccess;
}

MStatus DeltaMush::getNeighbours(MObject & mesh, unsigned int vertexCount)
{
	neighbours.resize(paddedCount * MAX_NEIGHBOURS);
	std::fill(neighbours.begin(), neighbours.end(), 0.0);

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

MStatus DeltaMush::averageSmoothing(unsigned int iterations, double weight, unsigned int vertexCount)
{
	std::vector<double> verticesCopyX{ verticesX };
	std::vector<double> verticesCopyY{ verticesY };
	std::vector<double> verticesCopyZ{ verticesZ };

	ComponentVector256d average{};

	__m256d weighVector{ _mm256_set1_pd(weight) };
	int* neighbourPtr{};

	for (unsigned int iterationIndex{ 0 }; iterationIndex < iterations; ++iterationIndex) {
		neighbourPtr = &neighbours[0];

		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4, neighbourPtr += 12) {
			average.setZero();

			for (unsigned int neighbourIndex{ 0 }; neighbourIndex < MAX_NEIGHBOURS; ++neighbourIndex, ++neighbourPtr) {
				ComponentVector256d neighbourPositions{
					_mm256_setr_pd(verticesCopyX[neighbourPtr[0]], verticesCopyX[neighbourPtr[0 + 4]], verticesCopyX[neighbourPtr[0 + 8]], verticesCopyX[neighbourPtr[0 + 12]]),
					_mm256_setr_pd(verticesCopyY[neighbourPtr[0]], verticesCopyY[neighbourPtr[0 + 4]], verticesCopyY[neighbourPtr[0 + 8]], verticesCopyY[neighbourPtr[0 + 12]]),
					_mm256_setr_pd(verticesCopyZ[neighbourPtr[0]], verticesCopyZ[neighbourPtr[0 + 4]], verticesCopyZ[neighbourPtr[0 + 8]], verticesCopyZ[neighbourPtr[0 + 12]])
				};

				average += neighbourPositions;
			}

			// Divides the accumulated vector to average it
			__m256d averageFactorVec = _mm256_set1_pd(AVERAGE_FACTOR);
			average *= averageFactorVec;

			ComponentVector256d verticesCopyPositions{ verticesCopyX.data() + vertexIndex, verticesCopyY.data() + vertexIndex, verticesCopyZ.data() + vertexIndex };

			average = (average - verticesCopyPositions) * weighVector + verticesCopyPositions;
			average.store(smoothedX.data() + vertexIndex, smoothedY.data() + vertexIndex, smoothedZ.data() + vertexIndex);

		}

		smoothedX.swap(verticesCopyX);
		smoothedY.swap(verticesCopyY);
		smoothedZ.swap(verticesCopyZ);
	}

	smoothedX.swap(verticesCopyX);
	smoothedY.swap(verticesCopyY);
	smoothedZ.swap(verticesCopyZ);

	return MStatus::kSuccess;
}

MStatus DeltaMush::cacheDeltas(unsigned int vertexCount)
{
	deltaMagnitudes.resize(paddedCount);

	deltasX.resize(paddedCount * MAX_NEIGHBOURS * DELTA_COUNT);
	deltasY.resize(paddedCount * MAX_NEIGHBOURS * DELTA_COUNT);
	deltasZ.resize(paddedCount * MAX_NEIGHBOURS * DELTA_COUNT);

	// Declare the data needed by the loop

	ComponentVector256d delta{};
	ComponentVector256dMatrix3x3 tangentSpaceMatrix{};

	int* neighbourPtr{ &neighbours[0] };

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4, neighbourPtr += 13) {

		ComponentVector256d smoothedPositions{ smoothedX.data() + vertexIndex, smoothedY.data() + vertexIndex, smoothedZ.data() + vertexIndex };
		delta = ComponentVector256d(verticesX.data() + vertexIndex, verticesY.data() + vertexIndex, verticesZ.data() + vertexIndex) - smoothedPositions;

		_mm256_store_pd(&deltaMagnitudes[vertexIndex], delta.length());

		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < DELTA_COUNT; ++neighbourIndex, ++neighbourPtr) {

			ComponentVector256d neighbourPositions{
				_mm256_setr_pd(smoothedX[neighbourPtr[0]], smoothedX[neighbourPtr[0 + 4]], smoothedX[neighbourPtr[0 + 8]], smoothedX[neighbourPtr[0 + 12]]),
				_mm256_setr_pd(smoothedY[neighbourPtr[0]], smoothedY[neighbourPtr[0 + 4]], smoothedY[neighbourPtr[0 + 8]], smoothedY[neighbourPtr[0 + 12]]),
				_mm256_setr_pd(smoothedZ[neighbourPtr[0]], smoothedZ[neighbourPtr[0 + 4]], smoothedZ[neighbourPtr[0 + 8]], smoothedZ[neighbourPtr[0 + 12]])
			};

			tangentSpaceMatrix.tangent = neighbourPositions - smoothedPositions;
			tangentSpaceMatrix.tangent.normalize();

			neighbourPositions.set(
				_mm256_setr_pd(smoothedX[neighbourPtr[1]], smoothedX[neighbourPtr[1 + 4]], smoothedX[neighbourPtr[1 + 8]], smoothedX[neighbourPtr[1 + 12]]),
				_mm256_setr_pd(smoothedY[neighbourPtr[1]], smoothedY[neighbourPtr[1 + 4]], smoothedY[neighbourPtr[1 + 8]], smoothedY[neighbourPtr[1 + 12]]),
				_mm256_setr_pd(smoothedZ[neighbourPtr[1]], smoothedZ[neighbourPtr[1 + 4]], smoothedZ[neighbourPtr[1 + 8]], smoothedZ[neighbourPtr[1 + 12]])
			);

			tangentSpaceMatrix.normal = neighbourPositions - smoothedPositions;
			tangentSpaceMatrix.normal.normalize();

			tangentSpaceMatrix.binormal = tangentSpaceMatrix.tangent ^ tangentSpaceMatrix.normal;
			tangentSpaceMatrix.normal = tangentSpaceMatrix.tangent ^ tangentSpaceMatrix.binormal;

			// Calculate the displacement Vector
			ComponentVector256d result{ tangentSpaceMatrix.inverseProduct(delta) };
			result.store(&deltasX[0] + (vertexIndex * 3) + (neighbourIndex * 4), &deltasY[0] + (vertexIndex * 3) + (neighbourIndex * 4), &deltasZ[0] + (vertexIndex * 3) + (neighbourIndex * 4));
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