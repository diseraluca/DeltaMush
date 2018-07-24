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

MString DeltaMush::typeName{ "ldsDeltaMush" };
MTypeId DeltaMush::typeId{ 0xd1230a };

MObject DeltaMush::referenceMesh;
MObject DeltaMush::smoothingIterations;
MObject DeltaMush::smoothWeight;
MObject DeltaMush::deltaWeight;

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
	CHECK_MSTATUS(attributeAffects(smoothingIterations, outputGeom));
	CHECK_MSTATUS(attributeAffects(smoothWeight, outputGeom));
	CHECK_MSTATUS(attributeAffects(deltaWeight, outputGeom));

	MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer ldsDeltaMush weights");

	return MStatus::kSuccess;
}

MStatus DeltaMush::deform(MDataBlock & block, MItGeometry & iterator, const MMatrix & matrix, unsigned int multiIndex)
{
	MStatus status{};

	MPlug referenceMeshPlug{ thisMObject(), referenceMesh };
	if (!referenceMeshPlug.isConnected()) {
		MGlobal::displayWarning(this->name() + ": referenceMesh is not connected. Please connect a mesh");
		return MStatus::kUnknownParameter;
	}

	// Retrieves attributes values
	float envelopeValue{ block.inputValue(envelope).asFloat() };
	MObject referenceMeshValue{ block.inputValue(referenceMesh).asMesh() };
	int smoothingIterationsValue{ block.inputValue(smoothingIterations).asInt() };
	double smoothWeightValue{ block.inputValue(smoothWeight).asDouble() };
	double deltaWeightValue{ block.inputValue(deltaWeight).asDouble() };

	int vertexCount{ iterator.count(&status) };
	CHECK_MSTATUS_AND_RETURN_IT(status);

	// Retrieves the positions for the reference mesh
	MFnMesh referenceMeshFn{ referenceMeshValue };
	MPointArray referenceMeshVertexPositions{};
	referenceMeshVertexPositions.setLength(vertexCount);
	CHECK_MSTATUS_AND_RETURN_IT(referenceMeshFn.getPoints(referenceMeshVertexPositions));

	// Build the neighbours array 
	std::vector<MIntArray> referenceMeshNeighbours{};
	getNeighbours(referenceMeshValue, referenceMeshNeighbours, vertexCount);

	// Calculate the smoothed positions for the reference mesh
	MPointArray referenceMeshSmoothedPositions{};
	averageSmoothing(referenceMeshVertexPositions, referenceMeshSmoothedPositions, referenceMeshNeighbours, smoothingIterationsValue, smoothWeightValue);

	// Calculate the deltas
	MVectorArray deltas{};
	cacheDeltas(referenceMeshFn, referenceMeshVertexPositions, referenceMeshSmoothedPositions, deltas, vertexCount);

	MPointArray meshVertexPositions{};
	iterator.allPositions(meshVertexPositions);

	// Caculate the smoothed positions for the deformed mesh
	MPointArray meshSmoothedPositions{};
	averageSmoothing(meshVertexPositions, meshSmoothedPositions, referenceMeshNeighbours, smoothingIterationsValue, smoothWeightValue);

	// Apply the deltas
	MFnMesh inputGeomFn{ getInputGeom(block, multiIndex) };
	MPointArray resultPositions{};
	applyDeltas(inputGeomFn, meshSmoothedPositions, deltas, resultPositions, deltaWeightValue*envelopeValue, vertexCount);

	iterator.setAllPositions(resultPositions);

	return MStatus::kSuccess;
}

MStatus DeltaMush::getNeighbours(MObject & mesh, std::vector<MIntArray>& out_neighbours, unsigned int vertexCount) const
{
	out_neighbours.resize(vertexCount);

	MItMeshVertex meshVtxIt{ mesh };
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++, meshVtxIt.next()) {
		CHECK_MSTATUS_AND_RETURN_IT(meshVtxIt.getConnectedVertices(out_neighbours[vertexIndex]));
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::averageSmoothing(const MPointArray & verticesPositions, MPointArray & out_smoothedPositions, const std::vector<MIntArray>& neighbours, unsigned int iterations, double weight) const
{
	unsigned int vertexCount{ verticesPositions.length() };
	out_smoothedPositions.setLength(vertexCount);

	// A copy is necessary to avoid losing the original data trough the computations while working iteratively on the smoothed positions
	MPointArray verticesPositionsCopy{ verticesPositions };
	for (unsigned int iterationIndex{ 0 }; iterationIndex < iterations; iterationIndex++) {
		for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
			MVector averagePosition{ neighboursAveragePosition(verticesPositionsCopy, neighbours, vertexIndex) };
			MVector smoothedPosition{ ((averagePosition - verticesPositionsCopy[vertexIndex]) * weight) + verticesPositionsCopy[vertexIndex] };

			out_smoothedPositions[vertexIndex] = smoothedPosition;
		}

		verticesPositionsCopy.copy(out_smoothedPositions);
	}

	return MStatus::kSuccess;
}

MVector DeltaMush::neighboursAveragePosition(const MPointArray & verticesPositions, const std::vector<MIntArray>& neighbours, unsigned int vertexIndex) const
{
	unsigned int neighbourCount{ neighbours[vertexIndex].length() };

	MVector averagePosition{};
	for (unsigned int neighbourIndex{ 0 }; neighbourIndex < neighbourCount; neighbourIndex++) {
		averagePosition += verticesPositions[neighbours[vertexIndex][neighbourIndex]];
	}

	averagePosition /= neighbourCount;

	return averagePosition;
}

MStatus DeltaMush::cacheDeltas(const MFnMesh & meshFn, const MPointArray & vertexPositions, const MPointArray & smoothedPositions, MVectorArray & out_deltas, unsigned int vertexCount) const
{
	MFloatVectorArray normals{};
	meshFn.getVertexNormals(false, normals);

	MFloatVectorArray tangents{};
	meshFn.getTangents(tangents);

	out_deltas.setLength(vertexCount);
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		// We use the cross product to calculate the binormal to ensure the orthogonality of the axis
		MVector binormal = normals[vertexIndex] ^ tangents[vertexIndex];

		// Build Tangent Space Matrix
		MMatrix tangentSpaceMatrix{};
		buildTangentSpaceMatrix(tangentSpaceMatrix, tangents[vertexIndex], normals[vertexIndex], binormal, smoothedPositions[vertexIndex]);

		// Calculate the displacement Vector
		out_deltas[vertexIndex] = tangentSpaceMatrix.inverse() * vertexPositions[vertexIndex];
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::applyDeltas(const MFnMesh & meshFn, const MPointArray & smoothedPositions, MVectorArray & deltas, MPointArray & out_positions, double weight, unsigned int vertexCount) const
{
	MFloatVectorArray normals{};
	meshFn.getVertexNormals(false, normals);

	MFloatVectorArray tangents{};
	meshFn.getTangents(tangents);

	out_positions.setLength(vertexCount);
	for (unsigned int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		// We use the cross product to calculate the binormal to ensure the orthogonality of the axis
		MVector binormal = normals[vertexIndex] ^ tangents[vertexIndex];

		// Build Tangent Space Matrix
		MMatrix tangentSpaceMatrix{};
		buildTangentSpaceMatrix(tangentSpaceMatrix, tangents[vertexIndex], normals[vertexIndex], binormal, smoothedPositions[vertexIndex]);

		// Apply the tangent trasformation to the delta vector to find the final position
		out_positions[vertexIndex] = ((tangentSpaceMatrix * deltas[vertexIndex]) * weight) + smoothedPositions[vertexIndex];
	}

	return MStatus::kSuccess;
}

MStatus DeltaMush::buildTangentSpaceMatrix(MMatrix & out_TangetSpaceMatrix, const MVector & tangent, const MVector & normal, const MVector & binormal, const MVector & translation) const
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

	out_TangetSpaceMatrix[3][0] = translation.x;
	out_TangetSpaceMatrix[3][1] = translation.y;
	out_TangetSpaceMatrix[3][2] = translation.z;
	out_TangetSpaceMatrix[3][3] = 1.0;

	return MStatus::kSuccess;
}

MObject DeltaMush::getInputGeom(MDataBlock & block, unsigned int multiIndex)
{
	// We use outputValues to avoid the possibility of triggering the calculation of the node again
	MArrayDataHandle inputHandle{ block.outputArrayValue(input) };
	inputHandle.jumpToElement(multiIndex);
	MObject inputGeomValue{ inputHandle.outputValue().child(inputGeom).asMesh() };
	
	return inputGeomValue;
}
