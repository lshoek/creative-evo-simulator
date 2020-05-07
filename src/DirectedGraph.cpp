#pragma once
#include "DirectedGraph.h"
#include "SimUtils.h"
#include "ofFileUtils.h"
#include "ofUtils.h"

#include "nlohmann/json.hpp"

using json = nlohmann::json;

#define NTRS_BODY_GENOME_DIR "output/genomes/morphology/"
#define NTRS_NODE_EXT ".node"
#define NTRS_CONN_EXT ".conn"

DirectedGraph::DirectedGraph() 
{
    std::random_device::result_type seed = _rd();
    //ofLog() << "DirectedGraph seed: " << seed;

    _rng = std::mt19937(seed);
    _distrib = std::uniform_real_distribution<>(0, 1);

    initRandom();
}

DirectedGraph::DirectedGraph(const DirectedGraph& srcGraph)
{
    // Build nodes and add indices accordingly
    _nodes.resize(srcGraph._nodes.size());
    for (GraphNode* n : srcGraph._nodes) {
        _nodes[n->getGraphIndex()] = new GraphNode(*n);
    }
    for (GraphNode* n : srcGraph._nodes) {
        GraphNode* targetNodePtr = _nodes[n->getGraphIndex()];
        for (GraphConnection* c : n->conns) {
            addConnection(targetNodePtr, _nodes[c->child->getGraphIndex()], c->jointInfo);
        }
    }
    for (GraphNode* n : _nodes) {
        if (n->IsRootNode()) {
            _rootNode = n;
            break;
        }
    }
};

void DirectedGraph::initRandom()
{
    // Build nodes
    GraphNode* root = new GraphNode(randomPrimitive(GraphNode::maxSize*0.25, GraphNode::maxSize), true);
    GraphNode* anotherNode = new GraphNode(randomPrimitive(GraphNode::minSize, GraphNode::maxSize), false);
    GraphNode* endNode = new GraphNode(randomPrimitive(GraphNode::minSize, GraphNode::maxSize), false);

    // Register indices in graph
    addNode(root);
    addNode(anotherNode);
    addNode(endNode);

    // Add connections
    root->addConnection(root, randomJoint());
    root->addConnection(anotherNode, randomJoint());
    anotherNode->addConnection(endNode, randomJoint());

    _rootNode = root;
}

// Verify that creature initialization works as it should
void DirectedGraph::initCurl()
{
    GraphNode::PrimitiveInfo primInfoTemplate{0, 5, btVector3(0.5, 1.0, 2.0)};
    GraphConnection::JointInfo jointInfoTemplate{};
    jointInfoTemplate.scalingFactor = 0.75;

    //jointInfoTemplate.parentAnchorDir = btVector3(0.5, -0.5, 1.0).rotate(btVector3(0, 1, 0), SIMD_HALF_PI);
    jointInfoTemplate.childAnchorDir = btVector3(0.25, 0.75, 1.0);

    GraphNode* root = new GraphNode(primInfoTemplate, true);
    root->addConnection(root, jointInfoTemplate);
    addNode(root);

    _rootNode = root;
}

GraphNode::PrimitiveInfo DirectedGraph::randomPrimitive(btScalar min, btScalar max)
{
    GraphNode::PrimitiveInfo info;

    info.dimensions = btVector3(
        _distrib(_rng) * (max - min) + min,
        _distrib(_rng) * (max - min) + min,
        _distrib(_rng) * (max - min) + min
    );
    info.parentAttachmentPlane = randomPointOnSphere();
    info.recursionLimit = int(_distrib(_rng)*5.0) + 1;
    return info;
}

GraphConnection::JointInfo DirectedGraph::randomJoint()
{
    GraphConnection::JointInfo info;
    float ax = _distrib(_rng);
    //std::normal_distribution<double> norm(0.0, 1.0);

    info.childAnchorDir = randomPointOnSphere();
    info.axis = ax < 0.3333f ? btVector3(1, 0, 0) : ax < 0.6666f ? btVector3(0, 1, 0) : btVector3(0, 0, 1);
    info.scalingFactor = 0.75;

    return info;
}

btVector3 DirectedGraph::randomPointOnSphere()
{
    btScalar theta = 2 * SIMD_PI * _distrib(_rng);
    btScalar phi = acos(1 - 2 * _distrib(_rng));
    btVector3 p(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
    return p;
}

void DirectedGraph::unfold() 
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
    node->setGraphIndex(getNodeIndex(node));
}

void DirectedGraph::addConnection(GraphNode* parent, GraphNode* child, const GraphConnection::JointInfo& info)
{
    parent->addConnection(child, info);
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

void DirectedGraph::save()
{
    const ofDirectory genomeDir = ofDirectory(ofToDataPath(NTRS_BODY_GENOME_DIR, true));
    const std::vector<ofFile> files = genomeDir.getFiles();

    std::string id = ofToString(genomeDir.getFiles().size());
    ofDirectory::createDirectory(genomeDir.getAbsolutePath() + '\\' + id);

    int nodeCount = 0;
    for (GraphNode* n : _nodes) {
        std::string path = genomeDir.getAbsolutePath() + '\\' + id + '\\' + ofToString(nodeCount) + NTRS_NODE_EXT;
        n->save(path);
        nodeCount++;
    }
}

void DirectedGraph::load(uint32_t id)
{
    const ofDirectory genomeDir = ofDirectory(ofToDataPath(NTRS_BODY_GENOME_DIR, true));
    std::string path = genomeDir.getAbsolutePath() + '\\' + ofToString(id);

    const std::vector<ofFile> files = ofDirectory(path).getFiles();

    _nodes.clear();
    for (ofFile f : files) {
        GraphNode* n = new GraphNode();
        n->load(f);
        addNode(n);
    }
}

DirectedGraph::~DirectedGraph()
{
    for (GraphNode* n : _nodes) {
        delete n;
    }
}
