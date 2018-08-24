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


const unsigned int DeltaMush::MAX_NEIGHBOURS{ 4 };
const unsigned int DeltaMush::DELTA_COUNT{ MAX_NEIGHBOURS - 1 };
const double DeltaMush::AVERAGE_FACTOR{ 1.0 / MAX_NEIGHBOURS };

DeltaMush::DeltaMush()
	:isInitialized{ false },
	 paddedCount{ 0 },
	 verticesX{ nullptr },
	 verticesY{ nullptr },
	 verticesZ {nullptr },
	 neighbours{},
	 deltasX{},
	 deltasY{},
	 deltasZ{},
	 deltas{},
	 deltaMagnitudes{},
	 perVertexWeights{}
{
}

DeltaMush::~DeltaMush()
{
	if (verticesX) {
		delete[] verticesX;
		delete[] verticesY;
		delete[] verticesZ;
	}

	if (smoothedX) {
		delete[] smoothedX;
		delete[] smoothedY;
		delete[] smoothedZ;
	}
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

		if (verticesX) {
			delete[] verticesX;
			delete[] verticesY;
			delete[] verticesZ;
			delete[] smoothedX;
			delete[] smoothedY;
			delete[] smoothedZ;
		}

		verticesX = new double[paddedCount]();
		verticesY = new double[paddedCount]();
		verticesZ = new double[paddedCount]();
		decomposePointArray(referenceMeshVertexPositions, verticesX, verticesY, verticesZ, vertexCount);

		smoothedX = new double[paddedCount]();
		smoothedY = new double[paddedCount]();
		smoothedZ = new double[paddedCount]();

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
	decomposePointArray(meshVertexPositions, verticesX, verticesY, verticesZ, vertexCount);

	// Caculate the smoothed positions for the deformed mesh
	MPointArray meshSmoothedPositions{};
	averageSmoothing(meshVertexPositions, meshSmoothedPositions, smoothingIterationsValue, smoothWeightValue);

	// Apply the deltas
	MPointArray resultPositions{};
	resultPositions.setLength(vertexCount);

	std::vector<double> resultsX{};
	resultsX.resize(paddedCount);
	std::vector<double> resultsY{};
	resultsY.resize(paddedCount);
	std::vector<double> resultsZ{};
	resultsZ.resize(paddedCount);

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

	__m256d deltaX{};
	__m256d deltaY{};
	__m256d deltaZ{};

	double * deltaXPtr{ &deltasX[0] };
	double * deltaYPtr{ &deltasY[0] };
	double * deltaZPtr{ &deltasZ[0] };

	int* neighbourPtr{ &neighbours[0] };

	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4, resultPositionsPtr += 4, vertexPositionsPtr += 4, neighbourPtr += 13) {
		// resetting the delta vector
		/*deltaPtr[0] = 0.0;
		deltaPtr[1] = 0.0;
		deltaPtr[2] = 0.0;*/
		deltaX = _mm256_setzero_pd();
		deltaY = _mm256_setzero_pd();
		deltaZ = _mm256_setzero_pd();

		__m256d smoothedPositionsX = _mm256_load_pd(smoothedX + vertexIndex);
		__m256d smoothedPositionsY = _mm256_load_pd(smoothedY + vertexIndex);
		__m256d smoothedPositionsZ = _mm256_load_pd(smoothedZ + vertexIndex);

		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < DELTA_COUNT; ++neighbourIndex, ++neighbourPtr, deltaXPtr += 4, deltaYPtr += 4, deltaZPtr += 4) {
			// Calculate the vectors between the current vertex and two of its neighbours
			/*tangentPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			tangentPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			tangentPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];*/
			__m256d neighboursPositionX = _mm256_setr_pd(smoothedX[neighbourPtr[0]], smoothedX[neighbourPtr[0 + 4]], smoothedX[neighbourPtr[0 + 8]], smoothedX[neighbourPtr[0 + 12]]);
			__m256d neighboursPositionY = _mm256_setr_pd(smoothedY[neighbourPtr[0]], smoothedY[neighbourPtr[0 + 4]], smoothedY[neighbourPtr[0 + 8]], smoothedY[neighbourPtr[0 + 12]]);
			__m256d neighboursPositionZ = _mm256_setr_pd(smoothedZ[neighbourPtr[0]], smoothedZ[neighbourPtr[0 + 4]], smoothedZ[neighbourPtr[0 + 8]], smoothedZ[neighbourPtr[0 + 12]]);

			__m256d tangentX = _mm256_sub_pd(neighboursPositionX, smoothedPositionsX);
			__m256d tangentY = _mm256_sub_pd(neighboursPositionY, smoothedPositionsY);
			__m256d tangentZ = _mm256_sub_pd(neighboursPositionZ, smoothedPositionsZ);

			__m256d length = _mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(tangentZ, tangentZ), _mm256_add_pd(_mm256_mul_pd(tangentX, tangentX), _mm256_mul_pd(tangentY, tangentY))));
			__m256d factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);

			tangentX = _mm256_mul_pd(tangentX, factor);
			tangentY = _mm256_mul_pd(tangentY, factor);
			tangentZ = _mm256_mul_pd(tangentZ, factor);

			/*normalPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			normalPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			normalPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];*/
			
			neighboursPositionX = _mm256_setr_pd(smoothedX[neighbourPtr[1]], smoothedX[neighbourPtr[1 + 4]], smoothedX[neighbourPtr[1 + 8]], smoothedX[neighbourPtr[1 + 12]]);
			neighboursPositionY = _mm256_setr_pd(smoothedY[neighbourPtr[1]], smoothedY[neighbourPtr[1 + 4]], smoothedY[neighbourPtr[1 + 8]], smoothedY[neighbourPtr[1 + 12]]);
			neighboursPositionZ = _mm256_setr_pd(smoothedZ[neighbourPtr[1]], smoothedZ[neighbourPtr[1 + 4]], smoothedZ[neighbourPtr[1 + 8]], smoothedZ[neighbourPtr[1 + 12]]);

			__m256d normalX = _mm256_sub_pd(neighboursPositionX, smoothedPositionsX);
			__m256d normalY = _mm256_sub_pd(neighboursPositionY, smoothedPositionsY);
			__m256d normalZ = _mm256_sub_pd(neighboursPositionZ, smoothedPositionsZ);

			length = _mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(normalZ, normalZ), _mm256_add_pd(_mm256_mul_pd(normalX, normalX), _mm256_mul_pd(normalY, normalY))));
			factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);

			normalX = _mm256_mul_pd(normalX, factor);
			normalY = _mm256_mul_pd(normalY, factor);
			normalZ = _mm256_mul_pd(normalZ, factor);

			// Normalizes the two vectors.
			// Vector normalization is calculated as follows:
			// lenght of the vector -> sqrt(x*x + y*y + z*z)
		    // [x, y, z] / length
			//length = std::sqrt(tangentPtr[0] * tangentPtr[0] + tangentPtr[1] * tangentPtr[1] + tangentPtr[2] * tangentPtr[2]);

			//factor = 1.0 / length;
			//tangentPtr[0] *= factor;
			//tangentPtr[1] *= factor;
			//tangentPtr[2] *= factor;

			//length = std::sqrt(normalPtr[0] * normalPtr[0] + normalPtr[1] * normalPtr[1] + normalPtr[2] * normalPtr[2]);

			//factor = 1.0 / length;
			//normalPtr[0] *= factor;
			//normalPtr[1] *= factor;
			//normalPtr[2] *= factor;

			// Ensures  axis orthogonality through cross product.
			// Cross product is calculated in the following code as:
			// crossVector = [(y1 * z2 - z1 * y2), (z1 * x2 - x1 * z2), (x1 * y2 - y1 * x2)]
	/*		binormalPtr[0] = tangentPtr[1] * normalPtr[2] - tangentPtr[2] * normalPtr[1];
			binormalPtr[1] = tangentPtr[2] * normalPtr[0] - tangentPtr[0] * normalPtr[2];
			binormalPtr[2] = tangentPtr[0] * normalPtr[1] - tangentPtr[1] * normalPtr[0];

			normalPtr[0] = tangentPtr[1] * binormalPtr[2] - tangentPtr[2] * binormalPtr[1];
			normalPtr[1] = tangentPtr[2] * binormalPtr[0] - tangentPtr[0] * binormalPtr[2];
			normalPtr[2] = tangentPtr[0] * binormalPtr[1] - tangentPtr[1] * binormalPtr[0];*/

			__m256d binormalX = _mm256_sub_pd(_mm256_mul_pd(tangentY, normalZ), _mm256_mul_pd(tangentZ, normalY));
			__m256d binormalY = _mm256_sub_pd(_mm256_mul_pd(tangentZ, normalX), _mm256_mul_pd(tangentX, normalZ));
			__m256d binormalZ = _mm256_sub_pd(_mm256_mul_pd(tangentX, normalY), _mm256_mul_pd(tangentY, normalX));

			/*	normalPtr[0] = tangentPtr[1] * binormalPtr[2] - tangentPtr[2] * binormalPtr[1];
			normalPtr[1] = tangentPtr[2] * binormalPtr[0] - tangentPtr[0] * binormalPtr[2];
			normalPtr[2] = tangentPtr[0] * binormalPtr[1] - tangentPtr[1] * binormalPtr[0];*/
			normalX = _mm256_sub_pd(_mm256_mul_pd(tangentY, binormalZ), _mm256_mul_pd(tangentZ, binormalY));
			normalY = _mm256_sub_pd(_mm256_mul_pd(tangentZ, binormalX), _mm256_mul_pd(tangentX, binormalZ));
			normalZ = _mm256_sub_pd(_mm256_mul_pd(tangentX, binormalY), _mm256_mul_pd(tangentY, binormalX));

			__m256d deltaxX = _mm256_load_pd(deltaXPtr);
			__m256d deltaxY = _mm256_load_pd(deltaYPtr);
			__m256d deltaxZ = _mm256_load_pd(deltaZPtr);

			__m256d resultX = _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(tangentX, deltaxX), _mm256_mul_pd(tangentY, deltaxY)), _mm256_mul_pd(tangentZ, deltaxZ));
			__m256d resultY = _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(normalX, deltaxX), _mm256_mul_pd(normalY, deltaxY)), _mm256_mul_pd(normalZ, deltaxZ));
			__m256d resultZ = _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(binormalX, deltaxX), _mm256_mul_pd(binormalY, deltaxY)), _mm256_mul_pd(binormalZ, deltaxZ));

			deltaX = _mm256_add_pd(deltaX, resultX);
			deltaY = _mm256_add_pd(deltaY, resultY);
			deltaZ = _mm256_add_pd(deltaZ, resultZ);

			// Accumulate the displacement Vectors
			// TODO : Provide a custom matrix*vector implementation to remove the intermediate MVector and transform the MMatrix into a simpler double[4][4]
			/*MVector tangentSpaceDelta{ tangentSpaceMatrix * deltas[vertexIndex][neighbourIndex] };
			deltaPtr[0] += tangentSpaceDelta.x;
			deltaPtr[1] += tangentSpaceDelta.y;
			deltaPtr[2] += tangentSpaceDelta.z;*/
		}

		// Averaging the delta
		/*factor = (1.0 / DELTA_COUNT);
		deltaPtr[0] *= factor;
		deltaPtr[1] *= factor;
		deltaPtr[2] *= factor;*/

		__m256d factor = _mm256_set1_pd(1.0 / DELTA_COUNT);
		deltaX = _mm256_mul_pd(deltaX, factor);
		deltaY = _mm256_mul_pd(deltaY, factor);
		deltaZ = _mm256_mul_pd(deltaZ, factor);

		// Scaling the delta
		/*delta.normalize();*/
		/*deltaPtr[0] *= (deltaMagnitudes[vertexIndex] * deltaWeightValue);
		deltaPtr[1] *= (deltaMagnitudes[vertexIndex] * deltaWeightValue);
		deltaPtr[2] *= (deltaMagnitudes[vertexIndex] * deltaWeightValue);*/
		__m256d length = _mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(deltaZ, deltaZ), _mm256_add_pd(_mm256_mul_pd(deltaX, deltaX), _mm256_mul_pd(deltaY, deltaY))));
		factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);

		deltaX = _mm256_mul_pd(deltaX, factor);
		deltaY = _mm256_mul_pd(deltaY, factor);
		deltaZ = _mm256_mul_pd(deltaZ, factor);

		deltaX = _mm256_mul_pd(deltaX, _mm256_mul_pd(_mm256_load_pd(&deltaMagnitudes[vertexIndex]), _mm256_set1_pd(deltaWeightValue)));
		deltaY = _mm256_mul_pd(deltaY, _mm256_mul_pd(_mm256_load_pd(&deltaMagnitudes[vertexIndex]), _mm256_set1_pd(deltaWeightValue)));
		deltaZ = _mm256_mul_pd(deltaZ, _mm256_mul_pd(_mm256_load_pd(&deltaMagnitudes[vertexIndex]), _mm256_set1_pd(deltaWeightValue)));

		// Finding the final position
	/*	resultPositionsPtr[0] = smoothedPositionsPtr[vertexIndex * 4] + deltaPtr[0];
		resultPositionsPtr[1] = smoothedPositionsPtr[vertexIndex * 4 + 1] + deltaPtr[1];
		resultPositionsPtr[2] = smoothedPositionsPtr[vertexIndex * 4 + 2] + deltaPtr[2];
		resultPositionsPtr[3] = 1.0;*/
		__m256d resultPositionX = _mm256_add_pd(smoothedPositionsX, deltaX);
		__m256d resultPositionY = _mm256_add_pd(smoothedPositionsY, deltaY);
		__m256d resultPositionZ = _mm256_add_pd(smoothedPositionsZ, deltaZ);

		// We calculate the new definitive delta
		/*deltaPtr[0] = resultPositionsPtr[0] - vertexPositionsPtr[0];
		deltaPtr[1] = resultPositionsPtr[1] - vertexPositionsPtr[1];
		deltaPtr[2] = resultPositionsPtr[2] - vertexPositionsPtr[2];*/
		__m256d vertexX = _mm256_load_pd(verticesX + vertexIndex);
		__m256d vertexY = _mm256_load_pd(verticesY + vertexIndex);
		__m256d vertexZ = _mm256_load_pd(verticesZ + vertexIndex);

		deltaX = _mm256_sub_pd(resultPositionX, vertexX);
		deltaY = _mm256_sub_pd(resultPositionY, vertexY);
		deltaZ = _mm256_sub_pd(resultPositionZ, vertexZ);

		// Setting the weighted final position
		/*resultPositionsPtr[0] = vertexPositionsPtr[0] + (deltaPtr[0] * perVertexWeights[vertexIndex] * envelopeValue);
		resultPositionsPtr[1] = vertexPositionsPtr[1] + (deltaPtr[1] * perVertexWeights[vertexIndex] * envelopeValue);
		resultPositionsPtr[2] = vertexPositionsPtr[2] + (deltaPtr[2] * perVertexWeights[vertexIndex] * envelopeValue);*/
		__m128 globalWeightsF{ _mm_load_ps(&perVertexWeights[vertexIndex]) };
		globalWeightsF = _mm_mul_ps(globalWeightsF, _mm_set1_ps(envelopeValue));

		__m256d globalWeights = _mm256_cvtps_pd(globalWeightsF);

		resultPositionX = _mm256_add_pd(vertexX, _mm256_mul_pd(deltaX, globalWeights));
		resultPositionY = _mm256_add_pd(vertexY, _mm256_mul_pd(deltaY, globalWeights));
		resultPositionZ = _mm256_add_pd(vertexZ, _mm256_mul_pd(deltaZ, globalWeights));

		_mm256_store_pd(&resultsX[vertexIndex], resultPositionX);
		_mm256_store_pd(&resultsY[vertexIndex], resultPositionY);
		_mm256_store_pd(&resultsZ[vertexIndex], resultPositionZ);
	}

	composePointArray(&resultsX[0], &resultsY[0], &resultsZ[0], resultPositions, vertexCount);
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

MStatus DeltaMush::averageSmoothing(const MPointArray & verticesPositions, MPointArray & out_smoothedPositions, unsigned int iterations, double weight)
{
	unsigned int vertexCount{ verticesPositions.length() };
	out_smoothedPositions.setLength(vertexCount);

	// A copy is necessary to avoid losing the original data trough the computations while working iteratively on the smoothed positions
	double* verticesCopyX = new double[paddedCount];
	std::copy(verticesX, verticesX + paddedCount, verticesCopyX);

	double* verticesCopyY = new double[paddedCount];
	std::copy(verticesY, verticesY + paddedCount, verticesCopyY);

	double* verticesCopyZ = new double[paddedCount];
	std::copy(verticesZ, verticesZ + paddedCount, verticesCopyZ);

	//Declaring the data needed by the loop
	__m256d averageX;
	__m256d averageY;
	__m256d averageZ;

	__m256d weighVector{ _mm256_set1_pd(weight) };
	int* neighbourPtr{};

	for (unsigned int iterationIndex{ 0 }; iterationIndex < iterations; ++iterationIndex) {
		neighbourPtr = &neighbours[0];

		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4, neighbourPtr += 12) {
			averageX = _mm256_setzero_pd();
			averageY = _mm256_setzero_pd();
			averageZ = _mm256_setzero_pd();

			for (unsigned int neighbourIndex{ 0 }; neighbourIndex < MAX_NEIGHBOURS; ++neighbourIndex, ++neighbourPtr) {
				__m256d neighboursX = _mm256_setr_pd(verticesCopyX[neighbourPtr[0]], verticesCopyX[neighbourPtr[0 + 4]], verticesCopyX[neighbourPtr[0 + 8]], verticesCopyX[neighbourPtr[0 + 12]]);
				__m256d neighboursY = _mm256_setr_pd(verticesCopyY[neighbourPtr[0]], verticesCopyY[neighbourPtr[0 + 4]], verticesCopyY[neighbourPtr[0 + 8]], verticesCopyY[neighbourPtr[0 + 12]]);
				__m256d neighboursZ = _mm256_setr_pd(verticesCopyZ[neighbourPtr[0]], verticesCopyZ[neighbourPtr[0 + 4]], verticesCopyZ[neighbourPtr[0 + 8]], verticesCopyZ[neighbourPtr[0 + 12]]);

				averageX = _mm256_add_pd(averageX, neighboursX);
				averageY = _mm256_add_pd(averageY, neighboursY);
				averageZ = _mm256_add_pd(averageZ, neighboursZ);
			}

			// Divides the accumulated vector to average it
			__m256d averageFactorVec = _mm256_set1_pd(AVERAGE_FACTOR);

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

			_mm256_store_pd(smoothedX + vertexIndex, averageX);
			_mm256_store_pd(smoothedY + vertexIndex, averageY);
			_mm256_store_pd(smoothedZ + vertexIndex, averageZ);
		}

		std::swap(smoothedX, verticesCopyX);
		std::swap(smoothedY, verticesCopyY);
		std::swap(smoothedZ, verticesCopyZ);
	}

	//TODO make average smoothing save its smoothed position in smoothedX
	composePointArray(verticesCopyX, verticesCopyY, verticesCopyZ, out_smoothedPositions, vertexCount);

	delete[] verticesCopyX;
	delete[] verticesCopyY;
	delete[] verticesCopyZ;

	return MStatus::kSuccess;
}

MStatus DeltaMush::cacheDeltas(const MPointArray & vertexPositions, const MPointArray & smoothedPositions, unsigned int vertexCount)
{
	deltas.resize(paddedCount);
	deltaMagnitudes.resize(paddedCount);

	deltasX.resize(paddedCount * MAX_NEIGHBOURS * DELTA_COUNT);
	deltasY.resize(paddedCount * MAX_NEIGHBOURS * DELTA_COUNT);
	deltasZ.resize(paddedCount * MAX_NEIGHBOURS * DELTA_COUNT);

	decomposePointArray(smoothedPositions, smoothedX, smoothedY, smoothedZ, vertexCount);

	// Declare the data needed by the loop
	MVector delta{};
	double *deltaPtr{ &delta.x };
	__m256d deltaX{};
	__m256d deltaY{};
	__m256d deltaZ{};

	const double *vertexPositionsPtr{ &vertexPositions[0].x };
	const double *smoothedPositionsPtr{ &smoothedPositions[0].x };

	MMatrix tangentSpaceMatrix{};
	double* tangentPtr{ &tangentSpaceMatrix.matrix[0][0] };
	double* normalPtr{ &tangentSpaceMatrix.matrix[1][0] };
	double* binormalPtr{ &tangentSpaceMatrix.matrix[2][0] };

	int* neighbourPtr{ &neighbours[0] };

	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex += 4, vertexPositionsPtr += 4, neighbourPtr += 13) {
		/*delta[0] = vertexPositionsPtr[0] - smoothedPositionsPtr[vertexIndex * 4];
		delta[1] = vertexPositionsPtr[1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
		delta[2] = vertexPositionsPtr[2] - smoothedPositionsPtr[vertexIndex * 4 + 2];*/
		__m256d smoothedPositionsX = _mm256_load_pd(smoothedX + vertexIndex);
		__m256d smoothedPositionsY = _mm256_load_pd(smoothedY + vertexIndex);
		__m256d smoothedPositionsZ = _mm256_load_pd(smoothedZ + vertexIndex);

		deltaX = _mm256_sub_pd(_mm256_load_pd(verticesX + vertexIndex), smoothedPositionsX);
		deltaY = _mm256_sub_pd(_mm256_load_pd(verticesY + vertexIndex), smoothedPositionsY);
		deltaZ = _mm256_sub_pd(_mm256_load_pd(verticesZ + vertexIndex), smoothedPositionsZ);

		/*deltaMagnitudes[vertexIndex] = delta.length();*/
		// Calculate the lenght of the array : sqrt(x^2 + y^2 + z^2)
		__m256d length = _mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(deltaZ, deltaZ), _mm256_add_pd(_mm256_mul_pd(deltaX, deltaX), _mm256_mul_pd(deltaY, deltaY))));
		_mm256_store_pd(&deltaMagnitudes[vertexIndex], length);

		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < DELTA_COUNT; ++neighbourIndex, ++neighbourPtr) {
			// Calculate the vectors between the current vertex and two of its neighbours
			//tangentPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			//tangentPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			//tangentPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];
			__m256d neighboursPositionX = _mm256_setr_pd(smoothedX[neighbourPtr[0]], smoothedX[neighbourPtr[0 + 4]], smoothedX[neighbourPtr[0 + 8]], smoothedX[neighbourPtr[0 + 12]]);
			__m256d neighboursPositionY = _mm256_setr_pd(smoothedY[neighbourPtr[0]], smoothedY[neighbourPtr[0 + 4]], smoothedY[neighbourPtr[0 + 8]], smoothedY[neighbourPtr[0 + 12]]);
			__m256d neighboursPositionZ = _mm256_setr_pd(smoothedZ[neighbourPtr[0]], smoothedZ[neighbourPtr[0 + 4]], smoothedZ[neighbourPtr[0 + 8]], smoothedZ[neighbourPtr[0 + 12]]);

			__m256d tangentX = _mm256_sub_pd(neighboursPositionX, smoothedPositionsX);
			__m256d tangentY = _mm256_sub_pd(neighboursPositionY, smoothedPositionsY);
			__m256d tangentZ = _mm256_sub_pd(neighboursPositionZ, smoothedPositionsZ);

			length = _mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(tangentZ, tangentZ), _mm256_add_pd(_mm256_mul_pd(tangentX, tangentX), _mm256_mul_pd(tangentY, tangentY))));
			__m256d factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);

			tangentX = _mm256_mul_pd(tangentX, factor);
			tangentY = _mm256_mul_pd(tangentY, factor);
			tangentZ = _mm256_mul_pd(tangentZ, factor);

			neighboursPositionX = _mm256_setr_pd(smoothedX[neighbourPtr[1]], smoothedX[neighbourPtr[1 + 4]], smoothedX[neighbourPtr[1 + 8]], smoothedX[neighbourPtr[1 + 12]]);
			neighboursPositionY = _mm256_setr_pd(smoothedY[neighbourPtr[1]], smoothedY[neighbourPtr[1 + 4]], smoothedY[neighbourPtr[1 + 8]], smoothedY[neighbourPtr[1 + 12]]);
			neighboursPositionZ = _mm256_setr_pd(smoothedZ[neighbourPtr[1]], smoothedZ[neighbourPtr[1 + 4]], smoothedZ[neighbourPtr[1 + 8]], smoothedZ[neighbourPtr[1 + 12]]);

			__m256d normalX = _mm256_sub_pd(neighboursPositionX, smoothedPositionsX);
			__m256d normalY = _mm256_sub_pd(neighboursPositionY, smoothedPositionsY);
			__m256d normalZ = _mm256_sub_pd(neighboursPositionZ, smoothedPositionsZ);

			length = _mm256_sqrt_pd(_mm256_add_pd(_mm256_mul_pd(normalZ, normalZ), _mm256_add_pd(_mm256_mul_pd(normalX, normalX), _mm256_mul_pd(normalY, normalY))));
			factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);

			normalX = _mm256_mul_pd(normalX, factor);
			normalY = _mm256_mul_pd(normalY, factor);
			normalZ = _mm256_mul_pd(normalZ, factor);

			/*normalPtr[0] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4] - smoothedPositionsPtr[vertexIndex * 4];
			normalPtr[1] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 1] - smoothedPositionsPtr[vertexIndex * 4 + 1];
			normalPtr[2] = smoothedPositionsPtr[neighbours[vertexIndex * MAX_NEIGHBOURS + neighbourIndex + 1] * 4 + 2] - smoothedPositionsPtr[vertexIndex * 4 + 2];*/

			/*length = std::sqrt(tangentPtr[0] * tangentPtr[0] + tangentPtr[1] * tangentPtr[1] + tangentPtr[2] * tangentPtr[2]);

			factor = 1.0 / length;
			tangentPtr[0] *= factor;
			tangentPtr[1] *= factor;
			tangentPtr[2] *= factor;

			length = std::sqrt(normalPtr[0] * normalPtr[0] + normalPtr[1] * normalPtr[1] + normalPtr[2] * normalPtr[2]);

			factor = 1.0 / length;
			normalPtr[0] *= factor;
			normalPtr[1] *= factor;
			normalPtr[2] *= factor;*/

			// Ensures  axis orthogonality through cross product.
			// Cross product is calculated in the following code as:
			// crossVector = [(y1 * z2 - z1 * y2), (z1 * x2 - x1 * z2), (x1 * y2 - y1 * x2)]
			//binormalPtr[0] = tangentPtr[1] * normalPtr[2] - tangentPtr[2] * normalPtr[1];
			//binormalPtr[1] = tangentPtr[2] * normalPtr[0] - tangentPtr[0] * normalPtr[2];
			//binormalPtr[2] = tangentPtr[0] * normalPtr[1] - tangentPtr[1] * normalPtr[0];
			__m256d binormalX = _mm256_sub_pd(_mm256_mul_pd(tangentY, normalZ), _mm256_mul_pd(tangentZ, normalY));
			__m256d binormalY = _mm256_sub_pd(_mm256_mul_pd(tangentZ, normalX), _mm256_mul_pd(tangentX, normalZ));
			__m256d binormalZ = _mm256_sub_pd(_mm256_mul_pd(tangentX, normalY), _mm256_mul_pd(tangentY, normalX));

		/*	normalPtr[0] = tangentPtr[1] * binormalPtr[2] - tangentPtr[2] * binormalPtr[1];
			normalPtr[1] = tangentPtr[2] * binormalPtr[0] - tangentPtr[0] * binormalPtr[2];
			normalPtr[2] = tangentPtr[0] * binormalPtr[1] - tangentPtr[1] * binormalPtr[0];*/
			normalX = _mm256_sub_pd(_mm256_mul_pd(tangentY, binormalZ), _mm256_mul_pd(tangentZ, binormalY));
			normalY = _mm256_sub_pd(_mm256_mul_pd(tangentZ, binormalX), _mm256_mul_pd(tangentX, binormalZ));
			normalZ = _mm256_sub_pd(_mm256_mul_pd(tangentX, binormalY), _mm256_mul_pd(tangentY, binormalX));

			//Scaling the crossed vector back
			length = _mm256_add_pd(_mm256_mul_pd(normalZ, normalZ), _mm256_add_pd(_mm256_mul_pd(normalX, normalX), _mm256_mul_pd(normalY, normalY)));
			factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);
			normalX = _mm256_mul_pd(normalX, factor);
			normalY = _mm256_mul_pd(normalY, factor);
			normalZ = _mm256_mul_pd(normalZ, factor);

			length = _mm256_add_pd(_mm256_mul_pd(binormalZ, binormalZ), _mm256_add_pd(_mm256_mul_pd(binormalX, binormalX), _mm256_mul_pd(binormalY, binormalY)));
			factor = _mm256_div_pd(_mm256_set1_pd(1.0), length);
			binormalX = _mm256_mul_pd(binormalX, factor);
			binormalY = _mm256_mul_pd(binormalY, factor);
			binormalZ = _mm256_mul_pd(binormalZ, factor);

			// Calculate the displacement Vector
			/*deltas[vertexIndex][neighbourIndex] = tangentSpaceMatrix.inverse() * delta;*/
			__m256d resultX = _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(tangentX, deltaX), _mm256_mul_pd(normalX, deltaY)), _mm256_mul_pd(binormalX, deltaZ));
			__m256d resultY = _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(tangentY, deltaX), _mm256_mul_pd(normalY, deltaY)), _mm256_mul_pd(binormalY, deltaZ));
			__m256d resultZ = _mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(tangentZ, deltaX), _mm256_mul_pd(normalZ, deltaY)), _mm256_mul_pd(binormalZ, deltaZ));


			_mm256_store_pd(&deltasX[0] + (vertexIndex * 3) + (neighbourIndex * 4), resultX);
			_mm256_store_pd(&deltasY[0] + (vertexIndex * 3) + (neighbourIndex * 4), resultY);
			_mm256_store_pd(&deltasZ[0] + (vertexIndex * 3) + (neighbourIndex * 4), resultZ);
		}
	}

	int deltaIndex = 0;
	for (unsigned int vertexIndex = 0; vertexIndex < vertexCount; vertexIndex += 4, deltaIndex += 12) {
		deltas[vertexIndex].setLength(DELTA_COUNT);
		deltas[vertexIndex + 1].setLength(DELTA_COUNT);
		deltas[vertexIndex + 2].setLength(DELTA_COUNT);
		deltas[vertexIndex + 3].setLength(DELTA_COUNT);

		deltas[vertexIndex][0].x = deltasX[deltaIndex];
		deltas[vertexIndex][0].y = deltasY[deltaIndex];
		deltas[vertexIndex][0].z = deltasZ[deltaIndex];

		deltas[vertexIndex][1].x = deltasX[deltaIndex + 4];
		deltas[vertexIndex][1].y = deltasY[deltaIndex + 4];
		deltas[vertexIndex][1].z = deltasZ[deltaIndex + 4];

		deltas[vertexIndex][2].x = deltasX[deltaIndex + 8];
		deltas[vertexIndex][2].y = deltasY[deltaIndex + 8];
		deltas[vertexIndex][2].z = deltasZ[deltaIndex + 8];

		deltas[vertexIndex+1][0].x = deltasX[deltaIndex+1];
		deltas[vertexIndex+1][0].y = deltasY[deltaIndex+1];
		deltas[vertexIndex+1][0].z = deltasZ[deltaIndex+1];

		deltas[vertexIndex+1][1].x = deltasX[deltaIndex + 4+1];
		deltas[vertexIndex+1][1].y = deltasY[deltaIndex + 4+1];
		deltas[vertexIndex+1][1].z = deltasZ[deltaIndex + 4+1];

		deltas[vertexIndex+1][2].x = deltasX[deltaIndex + 8+1];
		deltas[vertexIndex+1][2].y = deltasY[deltaIndex + 8+1];
		deltas[vertexIndex+1][2].z = deltasZ[deltaIndex + 8+1];

		deltas[vertexIndex+2][0].x = deltasX[deltaIndex+2];
		deltas[vertexIndex+2][0].y = deltasY[deltaIndex+2];
		deltas[vertexIndex+2][0].z = deltasZ[deltaIndex+2];

		deltas[vertexIndex+2][1].x = deltasX[deltaIndex + 2 + 4];
		deltas[vertexIndex+2][1].y = deltasY[deltaIndex + 2 + 4];
		deltas[vertexIndex+2][1].z = deltasZ[deltaIndex +  2 + 4];

		deltas[vertexIndex+2][2].x = deltasX[deltaIndex + 2 +8];
		deltas[vertexIndex+2][2].y = deltasY[deltaIndex +2+ 8];
		deltas[vertexIndex+2][2].z = deltasZ[deltaIndex +2+ 8];

		deltas[vertexIndex+3][0].x = deltasX[deltaIndex+3];
		deltas[vertexIndex+3][0].y = deltasY[deltaIndex+3];
		deltas[vertexIndex+3][0].z = deltasZ[deltaIndex+3];

		deltas[vertexIndex+3][1].x = deltasX[deltaIndex + 4+3];
		deltas[vertexIndex+3][1].y = deltasY[deltaIndex + 4+3];
		deltas[vertexIndex+3][1].z = deltasZ[deltaIndex + 4+3];

		deltas[vertexIndex+3][2].x = deltasX[deltaIndex + 8+3];
		deltas[vertexIndex+3][2].y = deltasY[deltaIndex + 8+3];
		deltas[vertexIndex+3][2].z = deltasZ[deltaIndex + 8+3];
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