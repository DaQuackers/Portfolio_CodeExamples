#pragma once

#include <vector>
#include "../CollisionLibrary/CollisionLibrary.h"

struct EDMesh;
// TriangleSet
//
// Each leaf of the BVH will have a TriangleSet as its stored data.
// Actual data of the mesh is stored in EDMesh.
struct TriangleSet
{
	// Pointer to the EDMesh.
	const EDMesh* m_pMesh;

	// List of triangle indices.
	// Triangle indices map to every three vertex indices in the EDMesh.
	// Ex: If m_TriIndices[0] == 5, this triangle
	//		is composed of m_pMesh->m_VertIndices[15], m_pMesh->m_VertIndices[16], and m_pMesh->m_VertIndices[17]
	std::vector< unsigned int > m_TriIndices;

	TriangleSet(const EDMesh* mesh) : m_pMesh(mesh) {}

	TriangleSet(const TriangleSet& other)
	{
		if (this != &other)
		{
			m_pMesh = other.m_pMesh;
			m_TriIndices = other.m_TriIndices;
		}
	}

	TriangleSet(TriangleSet&& other)
	{
		if (this != &other)
		{
			m_pMesh = other.m_pMesh;
			m_TriIndices = std::move(other.m_TriIndices);
		}
	}

	static TriangleSet generate(const EDMesh* mesh);

	static TriangleSet* create(const EDMesh* mesh) { return new TriangleSet(generate(mesh)); }
};

namespace BVH
{
	// Specializations of CollectionMethods for BVH::Tree<TriangleSet>
	namespace CollectionMethods
	{
		AABB ComputeAABB(TriangleSet* collection);

		void PartitionCollection(TriangleSet* in, TriangleSet*& resultOne, TriangleSet*& resultTwo);

		size_t GetCount(TriangleSet* collection);

		void OnNodeDestruct(TriangleSet* collection);

		template<typename Collection>
		size_t GetStopCount();

		template<>
		size_t GetStopCount<TriangleSet>();
	}
}