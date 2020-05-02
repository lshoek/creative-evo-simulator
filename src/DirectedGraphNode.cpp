#include "DirectedGraphNode.h"
#include "DirectedGraphConnection.h"

GraphNode::GraphNode(std::string id, PrimitiveInfo info, bool isRoot, uint32_t recursionLimit) :
	id(id), primitiveInfo(info), _bIsRootNode(isRoot), _recursionLimit(recursionLimit) {}

GraphNode::GraphNode(std::string id, bool isRoot, uint32_t recursionLimit) :
	id(id), primitiveInfo(PrimitiveInfo()), _bIsRootNode(isRoot), _recursionLimit(recursionLimit) {}

GraphNode::GraphNode(const GraphNode& g) :
	id(g.id), primitiveInfo(g.primitiveInfo), _bIsRootNode(g._bIsRootNode), _recursionLimit(g._recursionLimit), _graphIndex(g._graphIndex)
{
	for (GraphConnection* c : g.conns) {
		addConnection(c->child, c->bIsTerminalFlag);
	}
}

void GraphNode::addConnection(GraphNode* child, GraphConnection::JointInfo info, bool bIsTerminal) {
	conns.push_back(new GraphConnection(this, child, info, bIsTerminal));
}

void GraphNode::addConnection(GraphNode* child, bool bIsTerminal) {
	conns.push_back(new GraphConnection(this, child, bIsTerminal));
}

uint32_t GraphNode::getRecursionLimit() {
	return _recursionLimit;
}

uint32_t GraphNode::getGraphIndex()
{
	return _graphIndex;
}

void GraphNode::setGraphIndex(uint32_t index)
{
	_graphIndex = index;
}

bool GraphNode::IsRootNode() 
{
	return _bIsRootNode;
}
