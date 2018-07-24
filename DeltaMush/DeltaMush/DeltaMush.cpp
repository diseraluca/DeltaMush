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
#include <maya/MFnMesh.h>
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

	float envelopeValue{ block.inputValue(envelope).asFloat() };
	MObject referenceMeshValue{ block.inputValue(referenceMesh).asMesh() };
	int smoothingIterationsValue{ block.inputValue(smoothingIterations).asInt() };
	double smoothWeightValue{ block.inputValue(smoothWeight).asDouble() };
	double deltaWeightValue{ block.inputValue(deltaWeight).asDouble() };

	int vertexCount{ iterator.count(&status) };
	CHECK_MSTATUS_AND_RETURN_IT(status);

	MFnMesh referenceMeshFn{ referenceMeshValue };
	MPointArray referenceMeshVertexPositions{};
	referenceMeshVertexPositions.setLength(vertexCount);
	CHECK_MSTATUS_AND_RETURN_IT(referenceMeshFn.getPoints(referenceMeshVertexPositions));

	std::vector<MIntArray> referenceMeshNeighbours{};
	referenceMeshNeighbours.resize(vertexCount);

	// Get the neighbours for the reference mesh
	MItMeshVertex referenceMeshVtxIt{ referenceMeshValue };
	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++, referenceMeshVtxIt.next()) {
		CHECK_MSTATUS_AND_RETURN_IT(referenceMeshVtxIt.getConnectedVertices(referenceMeshNeighbours[vertexIndex]));
	}

	MPointArray referenceMeshSmoothedPositions{};
	referenceMeshSmoothedPositions.setLength(vertexCount);

	// Performs the smoothing
	averageSmoothing(referenceMeshVertexPositions, referenceMeshSmoothedPositions, referenceMeshNeighbours, smoothingIterationsValue, smoothWeightValue);

	MFloatVectorArray referenceMeshBinormals{};
	referenceMeshFn.getBinormals(referenceMeshBinormals);
	MFloatVectorArray referenceMeshTangents{};
	referenceMeshFn.getTangents(referenceMeshTangents);

	MVectorArray deltas{};
	deltas.setLength( vertexCount );

	//Calculate the deltas
	for ( int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		MVector vertexNormal{};
		referenceMeshFn.getVertexNormal(vertexIndex, false, vertexNormal);

		MVector binormal = vertexNormal ^ referenceMeshTangents[vertexIndex];

		// Build Tangent Space Matrix
		MMatrix tangentSpaceMatrix{};
		buildTangentSpaceMatrix(tangentSpaceMatrix, referenceMeshTangents[vertexIndex], vertexNormal, binormal, referenceMeshSmoothedPositions[vertexIndex]);

		// Calculate the displacementVector
		MVector delta{ tangentSpaceMatrix.inverse() * referenceMeshVertexPositions[vertexIndex] };
		deltas[vertexIndex] = delta;
	}

	MPointArray meshSmoothedPositions{};
	meshSmoothedPositions.setLength(vertexCount);
	MPointArray meshVertexPositions{};
	iterator.allPositions(meshVertexPositions);
	// Performs the smoothing
	averageSmoothing(meshVertexPositions, meshSmoothedPositions, referenceMeshNeighbours, smoothingIterationsValue, smoothWeightValue);

	//Get the input geom
	MArrayDataHandle inputHandle{ block.outputArrayValue(input) };
	inputHandle.jumpToElement(multiIndex);
	MObject inputGeomValue{ inputHandle.outputValue().child(inputGeom).asMesh() };
	MFnMesh inputGeomFn{ inputGeomValue };

	MFloatVectorArray inputGeomBinormals{};
	inputGeomFn.getBinormals(inputGeomBinormals);
	MFloatVectorArray inputGeomTangents{};
	inputGeomFn.getTangents(inputGeomTangents);

	MPointArray resultPositions{};
	resultPositions.setLength(vertexCount);

	// apply the delta
	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		MVector vertexNormal{};
		inputGeomFn.getVertexNormal(vertexIndex, false, vertexNormal);

		MVector binormal = vertexNormal ^ inputGeomTangents[vertexIndex];

		MMatrix tangentSpaceMatrix{};
		buildTangentSpaceMatrix(tangentSpaceMatrix, inputGeomTangents[vertexIndex], vertexNormal, binormal, meshSmoothedPositions[vertexIndex]);

		resultPositions[vertexIndex] += tangentSpaceMatrix * (deltas[vertexIndex] * deltaWeightValue);
	}

	iterator.setAllPositions(resultPositions);

	return MStatus::kSuccess;
}

MStatus DeltaMush::averageSmoothing(const MPointArray & verticesPositions, MPointArray & out_smoothedPositions, const std::vector<MIntArray>& neighbours, unsigned int iterations, double weight)
{
	unsigned int vertexCount{ verticesPositions.length() };
	out_smoothedPositions.setLength(vertexCount);

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

MVector DeltaMush::neighboursAveragePosition(const MPointArray & verticesPositions, const std::vector<MIntArray>& neighbours, unsigned int vertexIndex)
{
	unsigned int neighbourCount{ neighbours[vertexIndex].length() };

	MVector averagePosition{};
	for (unsigned int neighbourIndex{ 0 }; neighbourIndex < neighbourCount; neighbourIndex++) {
		averagePosition += verticesPositions[neighbours[vertexIndex][neighbourIndex]];
	}

	averagePosition /= neighbourCount;

	return averagePosition;
}

MStatus DeltaMush::buildTangentSpaceMatrix(MMatrix & out_TangetSpaceMatrix, const MVector & tangent, const MVector & normal, const MVector & binormal, const MVector & translation) const
{
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