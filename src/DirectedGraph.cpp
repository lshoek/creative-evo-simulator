#pragma once
#include "DirectedGraph.h"
#include "SimUtils.h"
#include "MathUtils.h"

GraphNode::PrimitiveInfo randomPrimitive(btScalar min, btScalar max);
GraphConnection::JointInfo randomJoint();

// simple test graph for now 
DirectedGraph::DirectedGraph() 
{
    initDefault();
    //initSnake();
}

void DirectedGraph::initDefault()
{
    GraphNode* root = new GraphNode("a", randomPrimitive(GraphNode::maxSize*0.25, GraphNode::maxSize), true, 3);
    GraphNode* anotherNode = new GraphNode("b", randomPrimitive(GraphNode::minSize, GraphNode::maxSize), false, 1);
    GraphNode* endNode = new GraphNode("c", randomPrimitive(GraphNode::minSize, GraphNode::maxSize), false, 1);

    root->addConnection(root, randomJoint(), false);
    root->addConnection(anotherNode, randomJoint(), false);
    anotherNode->addConnection(endNode, randomJoint(), true);

    _nodes.push_back(root);
    _nodes.push_back(anotherNode);
    _nodes.push_back(endNode);

    _rootNode = root;
}

void DirectedGraph::initSnake()
{
    GraphNode::PrimitiveInfo primInfoTemplate{0, btVector3(0.5, 0.5, 2.0), true};
    GraphConnection::JointInfo jointInfoTemplate{};

    GraphNode* root = new GraphNode("a", primInfoTemplate, true, 8);
    root->addConnection(root, randomJoint(), false);
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

GraphConnection::JointInfo randomJoint()
{
    GraphConnection::JointInfo info;
    float ax = ofRandom(1.0f);

    // This is currently leads to lots of broken randomly initialized phenomes
    // A system is required for preventing infeasible morphologies to be placed in the world
    info.parentAnchor = SimUtils::glmToBullet(MathUtils::randomPointOnSphere());

    info.reflection = btVector3(0, 0, 0);
    info.axis = ax < 0.3333f ? btVector3(1, 0, 0) : ax < 0.6666f ? btVector3(0, 1, 0) : btVector3(0, 0, 1);
    info.rotation = SimUtils::glmToBullet(glm::rotation(glm::vec3(0, 0, 1), MathUtils::randomPointOnSphere()));
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
