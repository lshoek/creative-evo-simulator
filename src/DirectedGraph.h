#pragma once
#include "DirectedGraphNode.h"
#include "DirectedGraphConnection.h"
#include "ofLog.h"
#include <random>

class DirectedGraph
{
public:
	DirectedGraph();
	DirectedGraph(const DirectedGraph& srcGraph);
	~DirectedGraph();

	void initRandom();
	void initCurl();
	void unfold();

	void addNode(GraphNode* node);
	void addConnection(GraphNode* parent, GraphNode* child, const GraphConnection::JointInfo& info);

	int getNumNodesUnwrapped();
	int getNumJointsUnwrapped();

	GraphNode* getRootNode();
	const std::vector<GraphNode*>& getNodes();

	void save();
	void load(uint32_t id);

private:
	int getNodeIndex(GraphNode* node);

	// Random & mutation
	GraphNode::PrimitiveInfo randomPrimitive(btScalar min, btScalar max);
	GraphConnection::JointInfo randomJoint();
	btVector3 randomPointOnSphere();

	std::vector<GraphNode*> _nodes;
	GraphNode* _rootNode;

	std::mt19937 _rng;
	std::random_device _rd;
	std::uniform_real_distribution<> _distrib;

	void dfs(GraphNode* node);
	void dfsTraverse(GraphNode* node, std::vector<int> recursionLimits);

	int _numNodesUnwrapped = 0;
	int _numJointsUnwrapped = 0;
	bool _bTraversed = false;
};
