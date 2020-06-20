#pragma once
#include "DirectedGraphNode.h"
#include "DirectedGraphConnection.h"
#include "ofLog.h"
#include <random>
#include <set>

class DirectedGraph
{
public:
	DirectedGraph();
	DirectedGraph(bool bInitRandom, bool bAxisAlignedAttachments);
	DirectedGraph(const DirectedGraph& srcGraph);
	~DirectedGraph();

	void initRandom(bool bAxisAlignedAttachments);
	void initPrefabStructure();
	void initCurl();
	void unfold();

	void addNode(GraphNode* node);
	void addConnection(GraphNode* parent, GraphNode* child, const GraphConnection::JointInfo& info);

	uint32_t getNumNodesUnfolded();
	uint32_t getNumEndNodesUnfolded();
	uint32_t getNumJointsUnfolded();
	std::string getName();

	GraphNode* getRootNode();
	const std::vector<GraphNode*>& getNodes();

	void print();
	void save();
	bool load(std::string id);

private:
	int getNodeIndex(GraphNode* node);
	std::vector<uint32_t> getIndices(bool connected);

	// Random & mutation
	GraphNode::PrimitiveInfo randomPrimitive(btScalar min, btScalar max, uint32_t minRecursions, uint32_t maxRecursions, bool bAxisAlignedAttachments);
	GraphConnection::JointInfo randomJoint(bool bAxisAlignedAttachments);
	btVector3 randomPointOnSphere();
	btVector3 randomAxis();

	std::vector<GraphNode*> _nodes;
	GraphNode* _rootNode;

	std::string _name = "";

	std::mt19937 _rng;
	std::random_device _rd;
	std::uniform_real_distribution<> _distrib;

	void dfs(GraphNode* node, bool bPrint);
	void dfsTraverse(GraphNode* node, std::vector<int> recursionLimits, bool bPrint);

	std::set<uint32_t> _connectedNodeIndices;

	uint32_t _numNodesUnfolded = 0;
	uint32_t _numEndNodesUnfolded = 0;
	uint32_t _numJointsUnfolded = 0;
	bool _bTraversed = false;
};
