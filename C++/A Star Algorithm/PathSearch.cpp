#include "PathSearch.h"
#include <Windows.h>
//#define _CRTDBG_MAP_ALLOC
//#include <stdlib.h>
//#include <crtdbg.h>

#define heur_val 1.20
#if 1
#define cost_val 1
#else
#define cost_val 0
#endif

namespace fullsail_ai {
	namespace algorithms {
		int rows = 0, cols = 0;
		float Rad = 0.0f;
		void Adjacent(SearchNode* rhs, SearchNode* lhs)
		{
			SearchNode::Edge blegh;
			blegh.end = lhs;
			blegh.edgeDist = Rad * 2 * lhs->tile->getWeight();
			rhs->edges.push_back(blegh);
			rhs->tile->addLineTo(lhs->tile, 0xff00ffa5);
		}
		bool Complete = false;
		float distance(SearchNode* node, SearchNode* goal)
		{
			vec2f N, G, F;
			N.x = node->tile->getXCoordinate();  N.y = node->tile->getYCoordinate();
			G.x = goal->tile->getXCoordinate();  G.y = goal->tile->getYCoordinate();
			F.x = G.x - N.x; F.y = G.y - N.y;
			float len = F.dot(F);
			return sqrt(len);
		}
		bool isGreater(PlannerNode* const& lhs, PlannerNode* const& rhs)
		{
			return lhs->finalCost > rhs->finalCost;
		}
		PathSearch::PathSearch() : que2(&isGreater)
		{
		}

		PathSearch::~PathSearch()
		{
		}
		void PathSearch::initialize(TileMap* _tileMap)
		{
			//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
			rows = _tileMap->getRowCount();
			cols = _tileMap->getColumnCount();
			Rad = _tileMap->getTileRadius();
			for (size_t i = 0; i < cols; ++i)
			{
				for (size_t j = 0; j < rows; ++j)
				{
					Tile* tmp = _tileMap->getTile(j, i);
					if (tmp->getWeight() > 0){
						SearchNode* aNode = new SearchNode();
						aNode->tile = tmp;
						FullList[tmp] = aNode;
						VisitedList[aNode] = new PlannerNode();
						VisitedList[aNode]->ActNode = nullptr;
					}
				}
			}
			for (size_t i = 0; i < rows; ++i)
			{
				for (size_t j = 0; j < cols; j++)
				{
					Tile* tmp = _tileMap->getTile(i, j);
					if (FullList[tmp] != nullptr){
						int col; col = tmp->getColumn();
						int row; row = tmp->getRow();
						Tile* tile = _tileMap->getTile(row, col - 1);
						if (tile != nullptr && tile->getWeight() != 0)
						{
							Adjacent(FullList[tmp], FullList[tile]);
						}
						tile = _tileMap->getTile(row, col + 1);
						if (tile != nullptr && tile->getWeight() != 0)
						{
							Adjacent(FullList[tmp], FullList[tile]);
						}
						if (row % 2 == 0)
						{
							tile = _tileMap->getTile(row - 1, col);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}
							tile = _tileMap->getTile(row - 1, col - 1);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}
							tile = _tileMap->getTile(row + 1, col);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}
							tile = _tileMap->getTile(row + 1, col - 1);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}
						}
						else
						{
							tile = _tileMap->getTile(row - 1, col);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}
							tile = _tileMap->getTile(row - 1, col + 1);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}

							tile = _tileMap->getTile(row + 1, col);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}
							tile = _tileMap->getTile(row + 1, col + 1);
							if (tile != nullptr && tile->getWeight() != 0)
							{
								Adjacent(FullList[tmp], FullList[tile]);
							}
						}
					}
				}
			}
			themap = _tileMap;
		}

		void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
		{
			Tile* check = themap->getTile(startRow, startColumn);
			Start = FullList[check];
			check = themap->getTile(goalRow, goalColumn);
			Goal = FullList[check];
			themap->resetTileDrawing();
			PlannerNode *root = new PlannerNode;
			root->parent = nullptr;
			root->ActNode = Start;
			root->givenCost = 0;
			root->ActNode->heuristicCost = distance(Start, Goal);
			root->finalCost = root->ActNode->heuristicCost * heur_val;
			que2.push(root);
			VisitedList[Start] = root;
			Complete = false;
		}

		////////////////////////////////////////
		/*
			A* Search Algorithm Update Loop
		*/
		////////////////////////////////////////
		void PathSearch::update(long timeslice)
		{ 
			while (!que2.empty()){
				curr = que2.front();
				que2.pop();
				if (curr->ActNode == Goal)
				{
					Complete = true;
					return;
				}
				for (size_t c = 0; c < curr->ActNode->edges.size(); ++c)
				{
					succ = curr->ActNode->edges[c].end;
					succ->tile->setFill(0xff0000ff);
					tempcost = curr->givenCost + curr->ActNode->edges[c].edgeDist;
					node = VisitedList[succ];
					if (node->ActNode != nullptr)
					{
						if (tempcost < node->givenCost)
						{
							que2.remove(node);
							node->givenCost = tempcost;
							node->finalCost = cost_val*node->givenCost + node->ActNode->heuristicCost*heur_val;
							node->parent = curr;
							que2.push(node);
						}
					}
					else
					{
						node->ActNode = succ;
						node->givenCost = tempcost;
						node->ActNode->heuristicCost = distance(succ, Goal);
						node->finalCost = cost_val*node->givenCost + node->ActNode->heuristicCost*heur_val;
						node->parent = curr;
						que2.push(node);
					}
				}
				if (timeslice == 0)
					return;
			}
		}

		void PathSearch::exit()
		{
			auto iter = VisitedList.begin();
			while (iter != VisitedList.end())
			{
				iter->second->ActNode = nullptr;
				iter++;
			}
			que2.clear();
			curr = nullptr;
		}

		void PathSearch::shutdown()
		{
			auto iter = VisitedList.begin();
			auto ite2 = FullList.begin();
			int i = 0;
			while (iter != VisitedList.end())
			{
				delete iter->second;
				iter++; i++;
			}
			VisitedList.clear();
			i = 0;
			while (ite2 != FullList.end())
			{
				delete ite2->second;
				ite2++; i++;
			}
			FullList.clear();
		}

		bool PathSearch::isDone() const
		{
			return Complete;
		}

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			std::vector<Tile const*> temp;
			PlannerNode* send = curr;
			while (send != nullptr)
			{
				temp.push_back(send->ActNode->tile);
				send = send->parent;
			}
			return temp;
		}
	}
}

