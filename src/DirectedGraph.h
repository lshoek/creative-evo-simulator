#pragma once
#include "DirectedGraphNode.h"
#include "DirectedGraphConnection.h"
#include "ofLog.h"
#include <random>

class DirectedGraph
{
public:
	DirectedGraph();
	DirectedGraph(bool bInitRandom);
	DirectedGraph(const DirectedGraph& srcGraph);
	~DirectedGraph();

	void initRandom();
	void initCurl();
	void unfold();

	void addNode(GraphNode* node);
	void addConnection(GraphNode* parent, GraphNode* child, const GraphConnection::JointInfo& info);

	uint32_t getNumNodesUnfolded();
	uint32_t getNumEndNodesUnfolded();
	uint32_t getNumJointsUnfolded();

	GraphNode* getRootNode();
	const std::vector<GraphNode*>& getNodes();

	void print();
	void save();
	void load(std::string id);

private:
	int getNodeIndex(GraphNode* node);

	// Random & mutation
	GraphNode::PrimitiveInfo randomPrimitive(btScalar min, btScalar max, uint32_t minRecursions, uint32_t maxRecursions);
	GraphConnection::JointInfo randomJoint();
	btVector3 randomPointOnSphere();

	std::vector<GraphNode*> _nodes;
	GraphNode* _rootNode;

	std::mt19937 _rng;
	std::random_device _rd;
	std::uniform_real_distribution<> _distrib;

	void dfs(GraphNode* node, bool bPrint);
	void dfsTraverse(GraphNode* node, std::vector<int> recursionLimits, bool bPrint);

	uint32_t _numNodesUnfolded = 0;
	uint32_t _numEndNodesUnfolded = 0;
	uint32_t _numJointsUnfolded = 0;
	bool _bTraversed = false;
};
