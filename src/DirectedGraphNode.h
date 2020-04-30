#pragma once
#include "DirectedGraphConnection.h"
#include "SimDefines.h"
#include "bullet/btBulletCollisionCommon.h"
#include <vector>

class GraphNode
{
public:
	struct PrimitiveInfo {
		uint32_t primitiveType = PrimitiveType_Box;
		uint32_t jointType = JointType_Hinge;
		btVector3 dimensions = btVector3(1.0, 1.0, 1.0);
		bool bSymmetryFlag = true;

		PrimitiveInfo() {}
		PrimitiveInfo(uint32_t type, btVector3 dims, bool symmetry) :
			primitiveType(type), dimensions(dims), bSymmetryFlag(symmetry) {}
		PrimitiveInfo(const PrimitiveInfo& src) : 
			primitiveType(src.primitiveType), dimensions(src.dimensions), bSymmetryFlag(src.bSymmetryFlag) {}
	};
	GraphNode(std::string id, PrimitiveInfo info, bool isRoot, uint32_t recursionLimit);
	GraphNode(std::string id, bool isRoot, uint32_t recursionLimit);
	GraphNode(const GraphNode& g);

	void addConnection(GraphNode* child, GraphConnection::JointInfo info, bool bIsTerminal);
	void addConnection(GraphNode* child, bool bIsTerminal);

	uint32_t getRecursionLimit();
	uint32_t getGraphIndex();
	void setGraphIndex(uint32_t index);

	PrimitiveInfo primitiveInfo;

	std::string id;
	std::vector<GraphConnection*> conns;

	static constexpr btScalar minSize = 0.125;
	static constexpr btScalar maxSize = 1.5;
	
private:
	bool _bIsRootNode = false;
	uint32_t _recursionLimit = 1;
	uint32_t _graphIndex;
};
