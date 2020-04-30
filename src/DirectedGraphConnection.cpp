#include "DirectedGraphConnection.h"
#include "DirectedGraphNode.h"

GraphConnection::GraphConnection(GraphNode* parent, GraphNode* child, JointInfo info, bool bIsTerminal) :
	parent(parent), child(child), bIsTerminalFlag(bIsTerminal)
{
	if (parent == child) {
		bIsRecurrent = true;
	}
	jointInfo = info;
}

GraphConnection::GraphConnection(GraphNode* parent, GraphNode* child, bool bIsTerminal) : 
	parent(parent), child(child), bIsTerminalFlag(bIsTerminal)
{
	if (parent == child) {
		bIsRecurrent = true;
	}
}
