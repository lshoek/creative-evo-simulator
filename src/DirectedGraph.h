#pragma once
#include "DirectedGraphNode.h"
#include "DirectedGraphConnection.h"
#include "ofLog.h"
#include <random>

class DirectedGraph
{
public:
	DirectedGraph();

	void initRandom();
	void initCurl();
	void unwrap();

	void addNode(GraphNode* node);
	void addConnection(GraphNode* parent, GraphNode* child, bool bIsTerminal);
	void addConnection(GraphNode* parent, GraphNode* child, GraphConnection::JointInfo info, bool bIsTerminal);

	int getNodeIndex(GraphNode* node);
	int getNumNodesUnwrapped();
	int getNumJointsUnwrapped();

	GraphNode* getRootNode();
	const std::vector<GraphNode*>& getNodes();

private:
	std::vector<GraphNode*> _nodes;

	GraphNode* _rootNode;
	std::default_random_engine _rng;

	void dfs(GraphNode* node);
	void dfsTraverse(GraphNode* node, std::vector<int> recursionLimits);

	int _numNodesUnwrapped = 0;
	int _numJointsUnwrapped = 0;
	bool _bTraversed = false;
};
