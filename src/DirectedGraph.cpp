#pragma once
#include "DirectedGraph.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include <random>

GraphNode::PrimitiveInfo randomPrimitive(btScalar min, btScalar max);
GraphConnection::JointInfo randomJoint(default_random_engine& rng);

// simple test graph for now 
DirectedGraph::DirectedGraph() 
{
    initRandom();
    //initCurl();
}

void DirectedGraph::initRandom()
{
    GraphNode* root = new GraphNode("a", randomPrimitive(GraphNode::maxSize*0.25, GraphNode::maxSize), true, 3);
    GraphNode* anotherNode = new GraphNode("b", randomPrimitive(GraphNode::minSize, GraphNode::maxSize), false, 1);
    GraphNode* endNode = new GraphNode("c", randomPrimitive(GraphNode::minSize, GraphNode::maxSize), false, 1);

    root->addConnection(root, randomJoint(_rng), false);
    root->addConnection(anotherNode, randomJoint(_rng), false);
    anotherNode->addConnection(endNode, randomJoint(_rng), true);

    _nodes.push_back(root);
    _nodes.push_back(anotherNode);
    _nodes.push_back(endNode);

    _rootNode = root;
}

// Verify that creature initialization works as it should
void DirectedGraph::initCurl()
{
    GraphNode::PrimitiveInfo primInfoTemplate{0, btVector3(0.5, 1.0, 2.0), true};
    GraphConnection::JointInfo jointInfoTemplate{};
    jointInfoTemplate.scalingFactor = 0.75;
    jointInfoTemplate.parentAnchorDir = btVector3(0.5, -0.5, 1.0).rotate(btVector3(0, 1, 0), SIMD_HALF_PI);
    jointInfoTemplate.childAnchorDir = btVector3(0.06125, 0.5, 1.0);

    GraphNode* root = new GraphNode("a", primInfoTemplate, true, 5);
    root->addConnection(root, jointInfoTemplate, false);
    _nodes.push_back(root);

    _rootNode = root;
}

GraphNode::PrimitiveInfo randomPrimitive(btScalar min, btScalar max)
{
    GraphNode::PrimitiveInfo info;

    info.dimensions = btVector3(
        ofRandom(max - min) + min,
        ofRandom(max - min) + min,
        ofRandom(max - min) + min
    );
    return info;
}

GraphConnection::JointInfo randomJoint(default_random_engine& rng)
{
    GraphConnection::JointInfo info;
    float ax = ofRandom(1.0f);

    std::normal_distribution<double> norm(0.0, 1.0);
    glm::vec3 fwd = glm::vec3(0, 0, 1);
    btQuaternion q = btQuaternion::getIdentity();

    // This is currently leads to lots of broken randomly initialized phenomes
    // A system is required for preventing infeasible morphologies to be placed in the world
    info.parentAnchorDir = SimUtils::glmToBullet(MathUtils::randomPointOnSphere());
    info.childAnchorDir = SimUtils::glmToBullet(MathUtils::randomPointOnSphere());

    info.reflection = btVector3(0, 0, 0);
    info.rotation = q.slerp(SimUtils::glmToBullet(glm::rotation(fwd, MathUtils::randomPointOnSphere())), norm(rng));
    info.axis = ax < 0.3333f ? btVector3(1, 0, 0) : ax < 0.6666f ? btVector3(0, 1, 0) : btVector3(0, 0, 1);
    info.scalingFactor = 0.75;

    return info;
}

void DirectedGraph::unwrap() 
{
    _bTraversed = false;
    _numNodesUnwrapped = 0;
    _numJointsUnwrapped = 0;

    for (GraphNode* gn : _nodes) {
        gn->setGraphIndex(getNodeIndex(gn));
    }
    dfs(_rootNode);
}

// Same node should not be registered more than once
void DirectedGraph::addNode(GraphNode* node)
{
    _nodes.push_back(node);
}

void DirectedGraph::addConnection(GraphNode* parent, GraphNode* child, bool bIsTerminal)
{
    parent->addConnection(child, bIsTerminal);
}

void DirectedGraph::addConnection(GraphNode* parent, GraphNode* child, GraphConnection::JointInfo info, bool bIsTerminal)
{
    parent->addConnection(child, info, bIsTerminal);
}

GraphNode* DirectedGraph::getRootNode()
{
    return _rootNode;
}

const std::vector<GraphNode*>& DirectedGraph::getNodes()
{
    return _nodes;
}

int DirectedGraph::getNodeIndex(GraphNode* node)
{
    std::vector<GraphNode*>::iterator it = std::find(_nodes.begin(), _nodes.end(), node);
    return std::distance(_nodes.begin(), it);
}

int DirectedGraph::getNumNodesUnwrapped()
{
    if (!_bTraversed) {
        dfs(_rootNode);
    }
    return _numNodesUnwrapped;
}

int DirectedGraph::getNumJointsUnwrapped()
{
    if (!_bTraversed) {
        dfs(_rootNode);
    }
    return _numJointsUnwrapped;
}

void DirectedGraph::dfs(GraphNode* node)
{
    std::vector<int> recursionLimits(_nodes.size());
    for (int i = 0; i < recursionLimits.size(); i++) {
        recursionLimits[i] = _nodes[i]->getRecursionLimit();
    }
    // todo: check if node is registered
    dfsTraverse(node, recursionLimits);
    _bTraversed = true;
}

void DirectedGraph::dfsTraverse(GraphNode* node, std::vector<int> recursionLimits)
{
    int index = getNodeIndex(node);
    recursionLimits[index]--;
    _numNodesUnwrapped++;

    for (GraphConnection* c : node->conns) {
        if (recursionLimits[getNodeIndex(c->child)] > 0) {
            _numJointsUnwrapped++;
            dfsTraverse(c->child, recursionLimits);
        }
    }
}
