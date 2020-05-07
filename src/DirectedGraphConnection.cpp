#include "DirectedGraphConnection.h"
#include "DirectedGraphNode.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

GraphConnection::GraphConnection(GraphNode* parent, GraphNode* child, const JointInfo& info) :
	parent(parent), child(child)
{
	if (parent == child) {
		bIsRecurrent = true;
	}
	jointInfo = info;
	jointInfo.fromIndex = parent->primitiveInfo.index;
	jointInfo.toIndex = child->primitiveInfo.index;
}



void to_json(json& j, const GraphConnection::JointInfo& info)
{
	j = json{
		{"childAnchorDir_x", double(info.childAnchorDir.x())},
		{"childAnchorDir_y", double(info.childAnchorDir.y())},
		{"childAnchorDir_z", double(info.childAnchorDir.z())},
		{"axis_x", int(info.axis.x())},
		{"axis_y", int(info.axis.y())},
		{"axis_z", int(info.axis.z())},
		{"scalingFactor", double(info.scalingFactor)},
		{"fromIndex", info.fromIndex},
		{"toIndex", info.toIndex},
	};
}

void from_json(const json& j, GraphConnection::JointInfo& info)
{
	double x, y, z, w;
	int ix, iy, iz;

	j.at("childAnchorDir_x").get_to(x);
	j.at("childAnchorDir_y").get_to(y);
	j.at("childAnchorDir_z").get_to(z);
	info.childAnchorDir = btVector3(x, y, z);

	j.at("axis_x").get_to(ix);
	j.at("axis_y").get_to(iy);
	j.at("axis_z").get_to(iz);
	info.axis = btVector3(ix, iy, iz);

	j.at("scalingFactor").get_to(info.scalingFactor);
	j.at("fromIndex").get_to(info.fromIndex);
	j.at("toIndex").get_to(info.toIndex);
}
