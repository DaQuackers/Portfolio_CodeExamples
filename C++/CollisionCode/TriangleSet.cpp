#include "TriangleSet.h"
#include "EDMesh.h"
#include <assert.h>

TriangleSet TriangleSet::generate(const EDMesh* mesh)
{
	TriangleSet result(mesh);

	if (result.m_pMesh)
	{
		for (unsigned int i = 0, j = 0; i < mesh->m_VertIndices.size(); i += 3, ++j)
			result.m_TriIndices.push_back(j);
	}

	return result;
}

namespace BVH
{
	// Specializations of CollectionMethods for BVH::Tree<TriangleSet>
	namespace CollectionMethods
	{
		size_t GetCount(TriangleSet* collection)
		{
			// Get the current number of triangles in the TriangleSet "collection"
			return collection->m_TriIndices.size();
		}

		template<>
		size_t GetStopCount<TriangleSet>()
		{
			// Get number of triangles to stop splitting at
			return 1;
		}

		void OnNodeDestruct(TriangleSet* collection)
		{
			// Containing node is being destroyed, so delete the TriangleSet
			delete collection;
		}

		AABB ComputeAABB(TriangleSet* collection)
		{
			// TODO:
			// Calculate the AABB of the vertices of the TriangleSet and return the result.
			// See the comment above "m_TriIndices" in "TriangleSet.h"

			AABB result;
			vec3f *max = &result.max, *min = &result.min;
			*min = vec3f(INT_MAX, INT_MAX, INT_MAX);
			*max = vec3f(-INT_MAX, -INT_MAX, -INT_MAX);
			for (size_t i = 0; i < collection->m_TriIndices.size(); i++)
			{
				int index = collection->m_TriIndices[i];
				for (size_t j = 0; j < 3; j++)
				{
					vec3f point = collection->m_pMesh->m_Vertices[collection->m_pMesh->m_VertIndices[(index * 3) + j]];
					if (point.x > max->x) max->x = point.x;
					if (point.x < min->x) min->x = point.x;

					if (point.y > max->y) max->y = point.y;
					if (point.y < min->y) min->y = point.y;

					if (point.z > max->z) max->z = point.z;
					if (point.z < min->z) min->z = point.z;
				}
			}
			return result;
		}

		void PartitionCollection(TriangleSet* in, TriangleSet*& resultOne, TriangleSet*& resultTwo)
		{
			// TODO:
			// Partition the TriangleSet "in" into two sets.
			// Use the "Splitting Along Object Mean" method from lecture. "Bounding Volume Hierarchies", slides 45-51.
			AABB cntBound;
			cntBound.max = vec3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			cntBound.min = vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
			vec3f avg;
			avg.make_zero();
			for (size_t i = 0; i < in->m_TriIndices.size(); i++)
			{
				vec3f point = in->m_pMesh->m_Centroids[in->m_TriIndices[i]];
				avg += point;
				if (point.x > cntBound.max.x) cntBound.max.x = point.x;
				if (point.x < cntBound.min.x) cntBound.min.x = point.x;
														  
				if (point.y > cntBound.max.y) cntBound.max.y = point.y;
				if (point.y < cntBound.min.y) cntBound.min.y = point.y;
										  				  
				if (point.z > cntBound.max.z) cntBound.max.z = point.z;
				if (point.z < cntBound.min.z) cntBound.min.z = point.z;
			}
			vec3f bounds = cntBound.max - cntBound.min;
			int index = 0, max = bounds.array()[index];
			if (bounds.array()[1] > max)
			{
				index = 1;
			}
			if (bounds.array()[2] > max)
			{
				index = 2;
			}
			avg /= in->m_TriIndices.size();
			std::vector< unsigned int > tmpIndices;
			resultTwo = new TriangleSet(in->m_pMesh);
			for (size_t i = 0; i < in->m_TriIndices.size(); i++)
			{
				if (in->m_pMesh->m_Centroids[in->m_TriIndices[i]].array()[index] > avg.array()[index])
					resultTwo->m_TriIndices.push_back(in->m_TriIndices[i]);
				else
					tmpIndices.push_back(in->m_TriIndices[i]);
			}
			in->m_TriIndices.clear();
			in->m_TriIndices = tmpIndices;
			resultOne = in;
			// You will only need to allocate one new TriangleSet for resultTwo.
			// "in" can be reused for resultOne.
		}
	}
}
