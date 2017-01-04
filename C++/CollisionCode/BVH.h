#pragma once

#include <assert.h>
#include "../CollisionLibrary/CollisionLibrary.h"

namespace BVH
{
	template<typename Collection>
	class Tree;

	// Template prototypes for methods used during BVH construction.
	// These methods do not have templated definitions.
	// Provide template specializations for desired collection types.
	namespace CollectionMethods
	{
		template<typename Collection>
		AABB ComputeAABB(Collection* collection);

		template<typename Collection>
		void PartitionCollection(Collection* in, Collection*& resultOne, Collection*& resultTwo);

		template<typename Collection>
		size_t GetCount(Collection* collection);

		template<typename Collection>
		size_t GetStopCount();

		template<typename Collection>
		void OnNodeDestruct(Collection* collection);
	}

	// BVH Tree Hierarchy class.
	// Used to represent some collection of objects with a hierarchy of AABBs.
	template<typename Collection>
	class Tree
	{
	public:
		struct Node;

	private:

		// Root of the hierarchy.
		Node* root;

		// This templated method allows type "Func" to be
		// a function pointer, a lambda function, an std::function, or a functor.
		// It is assumed Func can match the signature "bool func(Node*)".
		template<typename Func>
		void traverse(Node* current, Func func)
		{
			// TODO:
			// Perform func on the current node.
			// If the result is true, recursively traverse
			// if appropriate for the node type.
			if (func(current) == true && current->type == BRANCH)
			{
				traverse(current->left, func);
				traverse(current->right, func);
			}
			// To see an example func, see EDApp.cpp (lines 367-377).
		}

		Tree(Node* root) : root{ root } {}

		static Node* constructTD(Collection* collection);

	public:

		Tree(void) : root{ nullptr } {}

		Tree(Tree&& other)
		{
			root = other.root;
			other.root = nullptr;
		}

		Tree& operator=(Tree&& other)
		{
			if (this != &other)
				std::swap(root, other.root);

			return *this;
		}

		~Tree(){ delete root; }

		static const int LEAF = 0;
		static const int BRANCH = 1;

		// Node type that composes the tree hierarchy.
		struct Node
		{
			// Bounding volume of this node.
			AABB bounds;
			
			// Node type. LEAF, or BRANCH.
			int type;

			// Parent to this node in the hierarchy.
			Node* parent;

			// Branch nodes have left/right children.
			// Leaf nodes have collections.
			union
			{
				struct
				{
					Node* left;
					Node* right;
				};

				Collection* collection;
			};

			~Node()
			{
				// TODO: Complete the following steps to clean up allocated memory and references.

				// If this node has children, clean them up.
				// If this node has a collection, invoke CollectionMethods::OnNodeDestruct
				if (type == BRANCH)
				{
					delete left;
					delete right;
				}
				else
					CollectionMethods::OnNodeDestruct(collection);
				// If this node has a parent, null the parent's dangling references.
			}
		};

		// Traverse the BVH hierarchy from the root.
		// This templated method allows type "Func" to be
		// a function pointer, a lambda function, an std::function, or a functor.
		// It is assumed Func can match the signature "bool func(Node*)".
		template<typename Func>
		void traverse(Func func){ if(root) traverse(root, func); }

		static Tree createTD(Collection* in){ return Tree{ constructTD(in) }; }
	};

	// Construct the tree using the Top-Down algorithm
	template<typename Collection>
	typename Tree<Collection>::Node* Tree<Collection>::constructTD(Collection* collection)
	{
		if (collection == nullptr)
			return nullptr;

		// Number of objects in the collection
		size_t count = CollectionMethods::GetCount(collection);

		assert(count > 0);

		// TODO: Complete the following steps of the Top-Down construction algorithm.

		// Create a new node and compute its bounds with CollectionMethods::ComputeAABB
		Node* nod = new Node();
		nod->bounds = CollectionMethods::ComputeAABB(collection);

		// If the number of objects in the collection is <= CollectionMethods::GetStopCount
		// then the node will be a leaf. Set its members appropriately.
		size_t stopcount = CollectionMethods::GetStopCount<Collection>();
		if (count <= stopcount)
		{
			nod->type = LEAF;
			nod->collection = collection;
		}
		// Otherwise, the node will be a branch.
		// Split the collection with CollectionMethods::PartitionCollection.
		// Set the nodes members appropriately.
		else
		{
			nod->type = BRANCH;
			Collection *result1, *result2;
			CollectionMethods::PartitionCollection(collection, result1, result2);
			nod->left = constructTD(result1);
			nod->right = constructTD(result2);
			nod->left->parent = nod;
			nod->right->parent = nod;
		}

		// Return the newly created node.

		return nod;
	}
}

