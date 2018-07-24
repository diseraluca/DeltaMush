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
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshVertex.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MMatrix.h>

#include <vector>

MString DeltaMush::typeName{ "ldsDeltaMush" };
MTypeId DeltaMush::typeId{ 0xd1230a };

MObject DeltaMush::referenceMesh;
MObject DeltaMush::smoothingIterations;
MObject DeltaMush::smoothWeight;

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

	CHECK_MSTATUS(attributeAffects(referenceMesh, outputGeom));
	CHECK_MSTATUS(attributeAffects(smoothingIterations, outputGeom));
	CHECK_MSTATUS(attributeAffects(smoothWeight, outputGeom));

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
	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		MVector averagePosition{};

		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < referenceMeshNeighbours[vertexIndex].length(); neighbourIndex++) {
			averagePosition += referenceMeshVertexPositions[referenceMeshNeighbours[vertexIndex][neighbourIndex]];
		}

		averagePosition /= referenceMeshNeighbours[vertexIndex].length();
		MVector smoothedPosition = ((averagePosition - referenceMeshVertexPositions[vertexIndex] ) * smoothWeightValue) + referenceMeshVertexPositions[vertexIndex];

		referenceMeshSmoothedPositions[vertexIndex] = smoothedPosition;
	}

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

		// Build Tangent Space Matrix
		MMatrix tangentSpaceMatrix{};
		tangentSpaceMatrix[0][0] = referenceMeshTangents[vertexIndex].x;
		tangentSpaceMatrix[0][1] = referenceMeshTangents[vertexIndex].y;
		tangentSpaceMatrix[0][2] = referenceMeshTangents[vertexIndex].z;
		tangentSpaceMatrix[0][3] = 0.0;

		tangentSpaceMatrix[1][0] = vertexNormal.x;
		tangentSpaceMatrix[1][1] = vertexNormal.y;
		tangentSpaceMatrix[1][2] = vertexNormal.z;
		tangentSpaceMatrix[1][3] = 0.0;

		tangentSpaceMatrix[2][0] = referenceMeshBinormals[vertexIndex].x;
		tangentSpaceMatrix[2][1] = referenceMeshBinormals[vertexIndex].y;
		tangentSpaceMatrix[2][2] = referenceMeshBinormals[vertexIndex].z;
		tangentSpaceMatrix[2][3] = 0.0;

		tangentSpaceMatrix[3][0] = referenceMeshSmoothedPositions[vertexIndex].x;
		tangentSpaceMatrix[3][1] = referenceMeshSmoothedPositions[vertexIndex].y;
		tangentSpaceMatrix[3][2] = referenceMeshSmoothedPositions[vertexIndex].z;
		tangentSpaceMatrix[3][3] = 1.0;

		// Calculate the displacementVector
		MVector delta{ tangentSpaceMatrix.inverse() * referenceMeshVertexPositions[vertexIndex] };
		deltas[vertexIndex] = delta;
	}

	MPointArray meshSmoothedPositions{};
	meshSmoothedPositions.setLength(vertexCount);
	MPointArray meshVertexPositions{};
	iterator.allPositions(meshVertexPositions);
	// Performs the smoothing
	for (int vertexIndex{ 0 }; vertexIndex < vertexCount; vertexIndex++) {
		MVector averagePosition{};

		for (unsigned int neighbourIndex{ 0 }; neighbourIndex < referenceMeshNeighbours[vertexIndex].length(); neighbourIndex++) {
			averagePosition += meshVertexPositions[referenceMeshNeighbours[vertexIndex][neighbourIndex]];
		}

		averagePosition /= referenceMeshNeighbours[vertexIndex].length();
		MVector smoothedPosition = ((averagePosition - meshVertexPositions[vertexIndex]) * smoothWeightValue) + meshVertexPositions[vertexIndex];

		meshSmoothedPositions[vertexIndex] = smoothedPosition;
	}

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

		// Build Tangent Space Matrix
		MMatrix tangentSpaceMatrix{};
		tangentSpaceMatrix[0][0] = inputGeomTangents[vertexIndex].x;
		tangentSpaceMatrix[0][1] = inputGeomTangents[vertexIndex].y;
		tangentSpaceMatrix[0][2] = inputGeomTangents[vertexIndex].z;
		tangentSpaceMatrix[0][3] = 0.0;

		tangentSpaceMatrix[1][0] = vertexNormal.x;
		tangentSpaceMatrix[1][1] = vertexNormal.y;
		tangentSpaceMatrix[1][2] = vertexNormal.z;
		tangentSpaceMatrix[1][3] = 0.0;

		tangentSpaceMatrix[2][0] = inputGeomBinormals[vertexIndex].x;
		tangentSpaceMatrix[2][1] = inputGeomBinormals[vertexIndex].y;
		tangentSpaceMatrix[2][2] = inputGeomBinormals[vertexIndex].z;
		tangentSpaceMatrix[2][3] = 0.0;

		tangentSpaceMatrix[3][0] = meshSmoothedPositions[vertexIndex].x;
		tangentSpaceMatrix[3][1] = meshSmoothedPositions[vertexIndex].y;
		tangentSpaceMatrix[3][2] = meshSmoothedPositions[vertexIndex].z;
		tangentSpaceMatrix[3][3] = 1.0;

		resultPositions[vertexIndex] = tangentSpaceMatrix * deltas[vertexIndex];
	}

	iterator.setAllPositions(resultPositions);

	return MStatus::kSuccess;
}
