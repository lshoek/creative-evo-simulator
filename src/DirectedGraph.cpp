#pragma once
#include "DirectedGraph.h"
#include "SimDefines.h"
#include "SimUtils.h"
#include "ofFileUtils.h"
#include "ofUtils.h"
#include <set>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

DirectedGraph::DirectedGraph() {}

DirectedGraph::DirectedGraph(bool bInitRandom, bool bAxisAlignedAttachments)
{
    std::random_device::result_type seed = _rd();
    //ofLog() << "DirectedGraph seed: " << seed;

    _rng = std::mt19937(seed);
    _distrib = std::uniform_real_distribution<>(0, 1);

    if (bInitRandom) {
        //initCurl();
        //initPrefabStructure();
        initRandom(bAxisAlignedAttachments);
    }
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
        for (GraphConnection* c : n->getConnections()) {
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

void DirectedGraph::initRandom(bool bAAAttachments)
{
    uint32_t minNumNodes = 4;
    uint32_t minNumConnections = minNumNodes;

    uint32_t numNodes = minNumNodes + int32_t(_distrib(_rng) * 4.0);
    uint32_t numConnections = minNumConnections + uint32_t(_distrib(_rng) * 4.0);
    std::vector<GraphNode*> nodes(numNodes);

    nodes[0] = new GraphNode(randomPrimitive(
        GraphNode::minSize, GraphNode::maxSize, 2, uint32_t(_distrib(_rng) * 3.0), bAAAttachments), true
    );
    _rootNode = nodes[0];
    addNode(_rootNode);

    uint32_t connectionCount = 0;
    for (uint32_t i = 1; i < numNodes; i++) {
        GraphNode* gn = new GraphNode(randomPrimitive(
            GraphNode::minSize, GraphNode::maxSize, 1, uint32_t(_distrib(_rng) * 3.0), bAAAttachments), true
        );
        addNode(gn);
        nodes[uint32_t(_distrib(_rng) * i)]->addConnection(gn, randomJoint(bAAAttachments));
        nodes[i] = gn;

        connectionCount++;
    }
    while (connectionCount < numConnections) {
        nodes[uint32_t(_distrib(_rng) * numNodes)]->addConnection(nodes[uint32_t(_distrib(_rng) * numNodes)], randomJoint(bAAAttachments));
        connectionCount++;
    }

    bool bFullyConnected = false;
    while (!bFullyConnected) {

        dfs(_rootNode, false);
        const std::vector<uint32_t>& loose = getIndices(false);
        const std::vector<uint32_t>& connected = getIndices(true);

        if (!loose.empty()) {
            GraphNode* connectedNode = _nodes[connected[uint32_t(_distrib(_rng) * connected.size())]];
            GraphNode* looseNode = _nodes[loose[uint32_t(_distrib(_rng) * loose.size())]];
            connectedNode->addConnection(looseNode, randomJoint(bAAAttachments));
        }
        else {
            bFullyConnected = true;
        }
    }

}

void DirectedGraph::initPrefabStructure()
{
    bool bAAAttachments = false;

    // Build nodes
    GraphNode* root = new GraphNode(randomPrimitive(GraphNode::maxSize * 0.25, GraphNode::maxSize, 3, 3, bAAAttachments), true);
    GraphNode* anotherNode = new GraphNode(randomPrimitive(GraphNode::minSize, GraphNode::maxSize, 1, 3, bAAAttachments), false);
    GraphNode* endNode = new GraphNode(randomPrimitive(GraphNode::minSize, GraphNode::maxSize, 1, 3, bAAAttachments), false);

    // Register indices in graph
    addNode(root);
    addNode(anotherNode);
    addNode(endNode);

    // Add connections
    root->addConnection(root, randomJoint(bAAAttachments));
    root->addConnection(anotherNode, randomJoint(bAAAttachments));
    root->addConnection(anotherNode, randomJoint(bAAAttachments));
    anotherNode->addConnection(endNode, randomJoint(bAAAttachments));

    _rootNode = root;
}

void DirectedGraph::initCurl()
{
    GraphNode::PrimitiveInfo primInfoTemplate{0, 1, btVector3(0.5, 1.0, 2.0)};

    GraphNode* a = new GraphNode(randomPrimitive(GraphNode::maxSize * 0.25, GraphNode::maxSize, 2, 2, true), true);
    GraphNode* b = new GraphNode(randomPrimitive(GraphNode::maxSize * 0.25, GraphNode::maxSize, 1, 1, true), true);
    GraphNode* c = new GraphNode(randomPrimitive(GraphNode::maxSize * 0.25, GraphNode::maxSize, 1, 1, true), true);
    GraphNode* d = new GraphNode(randomPrimitive(GraphNode::maxSize * 0.25, GraphNode::maxSize, 1, 1, true), true);
    GraphNode* e = new GraphNode(randomPrimitive(GraphNode::maxSize * 0.25, GraphNode::maxSize, 1, 1, true), true);
    a->addConnection(b, randomJoint(true));
    b->addConnection(c, randomJoint(true));
    c->addConnection(d, randomJoint(true));
    d->addConnection(e, randomJoint(true));
    addNode(a);
    addNode(b);
    addNode(c);
    addNode(d);
    addNode(e);
    _rootNode = a;
}

GraphNode::PrimitiveInfo DirectedGraph::randomPrimitive(
    btScalar min, btScalar max, uint32_t minRecursions, uint32_t maxRecursions, bool bAxisAlignedAttachments)
{
    GraphNode::PrimitiveInfo info;
    minRecursions = minRecursions > 0 ? minRecursions : 1;
    maxRecursions = maxRecursions > minRecursions ? maxRecursions : minRecursions;

    info.dimensions = btVector3(
        _distrib(_rng) * (max - min) + min,
        _distrib(_rng) * (max - min) + min,
        _distrib(_rng) * (max - min) + min
    );
    info.parentAttachmentPlane = bAxisAlignedAttachments ? (randomAxis() * (_distrib(_rng) > .5 ? -1. : 1.)) : randomPointOnSphere();
    info.recursionLimit = uint32_t(_distrib(_rng)*(maxRecursions-minRecursions)) + minRecursions;
    return info;
}

GraphConnection::JointInfo DirectedGraph::randomJoint(bool bAxisAlignedAttachments)
{
    GraphConnection::JointInfo info;
    
    info.childAnchorDir = (bAxisAlignedAttachments ? 
        (randomAxis() * (_distrib(_rng) > .5 ? -1. : 1.)) : 
        randomPointOnSphere()
    );
    info.axis = (randomAxis() * (_distrib(_rng) > .5 ? -1. : 1.));
    info.scalingFactor = (_distrib(_rng) * 0.25) + 0.75;
    return info;
}

btVector3 DirectedGraph::randomPointOnSphere()
{
    btScalar theta = 2 * SIMD_PI * _distrib(_rng);
    btScalar phi = acos(1 - 2 * _distrib(_rng));
    btVector3 p(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
    return p;
}

btVector3 DirectedGraph::randomAxis()
{
    float ax = _distrib(_rng);
    btVector3 axis = (ax < 0.3333f ?
        btVector3(1, 0, 0) : ax < 0.6666f ?
        btVector3(0, 1, 0) :
        btVector3(0, 0, 1)
    );
    return axis;
}

std::vector<uint32_t> DirectedGraph::getIndices(bool bConnected)
{
    if (bConnected) {
        return std::vector<uint32_t>(_connectedNodeIndices.begin(), _connectedNodeIndices.end());
    }
    std::vector<uint32_t> loose;
    for (uint32_t i = 1; i < _nodes.size(); i++) {
        if (_connectedNodeIndices.find(i) == _connectedNodeIndices.end()) {
            loose.push_back(i);
        }
    }
    return loose;
}

void DirectedGraph::unfold() 
{
    _bTraversed = false;
    dfs(_rootNode, false);
}

// Same node should not be registered more than once
void DirectedGraph::addNode(GraphNode* node)
{
    _nodes.push_back(node);
    node->setGraphIndex(getNodeIndex(node));
    _bTraversed = false;
}

void DirectedGraph::addConnection(GraphNode* parent, GraphNode* child, const GraphConnection::JointInfo& info)
{
    parent->addConnection(child, info);
    _bTraversed = false;
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

uint32_t DirectedGraph::getNumNodesUnfolded()
{
    if (!_bTraversed) {
        dfs(_rootNode, false);
    }
    return _numNodesUnfolded;
}

uint32_t DirectedGraph::getNumEndNodesUnfolded()
{
    if (!_bTraversed) {
        dfs(_rootNode, false);
    }
    return _numEndNodesUnfolded;
}

uint32_t DirectedGraph::getNumJointsUnfolded()
{
    if (!_bTraversed) {
        dfs(_rootNode, false);
    }
    return _numJointsUnfolded;
}

void DirectedGraph::dfs(GraphNode* node, bool bPrint)
{
    std::vector<int> recursionLimits(_nodes.size());
    for (int i = 0; i < recursionLimits.size(); i++) {
        recursionLimits[i] = _nodes[i]->getRecursionLimit();
    }
    _connectedNodeIndices.clear();

    _numNodesUnfolded = 0;
    _numEndNodesUnfolded = 0;
    _numJointsUnfolded = 0;

    // todo: check if node is registered
    dfsTraverse(node, recursionLimits, bPrint);
    _bTraversed = true;
}

void DirectedGraph::dfsTraverse(GraphNode* node, std::vector<int> recursionLimits, bool bPrint)
{
    int index = node->primitiveInfo.index;

    _connectedNodeIndices.insert(index);
    _numNodesUnfolded++;

    if (bPrint) {
        std::ostringstream ss;
        for (GraphConnection* c : node->getConnections()) {
            ss << c->child->primitiveInfo.index << ", ";
        }
        ofLog() << index << " [" << recursionLimits[index] << "] -> " << ss.str();
    }
    recursionLimits[index]--;

    if (node->getConnections().empty()) _numEndNodesUnfolded++;
    for (GraphConnection* c : node->getConnections()) {
        if (recursionLimits[c->child->primitiveInfo.index] > 0) {
            _numJointsUnfolded++;
            dfsTraverse(c->child, recursionLimits, bPrint);
        }
    }
}

void DirectedGraph::print()
{
    dfs(_rootNode, true);
}

void DirectedGraph::save()
{
    const ofDirectory genomeDir = ofDirectory(ofToDataPath(NTRS_BODY_GENOME_DIR, true));
    const std::vector<ofFile> files = genomeDir.getFiles();

    std::string id = ofToString(genomeDir.getFiles().size());
    ofDirectory::createDirectory(genomeDir.getAbsolutePath() + '\\' + id);

    int nodeCount = 0;
    int connCount = 0;
    for (GraphNode* n : _nodes) {
        std::string path = genomeDir.getAbsolutePath() + '\\' + id + '\\' + ofToString(nodeCount) + '.';
        n->save(path + NTRS_NODE_EXT);

        for (GraphConnection* c : n->getConnections()) {
            path = genomeDir.getAbsolutePath() + '\\' + id + '\\' + ofToString(connCount) + '.';
            c->save(path + NTRS_CONN_EXT);
            connCount++;
        }
        nodeCount++;
    }
}

void DirectedGraph::load(std::string id)
{
    const ofDirectory genomeDir = ofDirectory(ofToDataPath(NTRS_BODY_GENOME_DIR, true));
    std::string path = genomeDir.getAbsolutePath() + '\\' + id;

    std::vector<ofFile> files = ofDirectory(path).getFiles();

    _nodes.clear();
    for (ofFile& f : files) {
        f.changeMode(ofFile::ReadOnly, false);
        if (f.getExtension() == NTRS_NODE_EXT) {
            GraphNode* n = new GraphNode();
            n->load(f);
            addNode(n);

            if (n->primitiveInfo.index == 0) {
                n->setIsRootNode(true);
                _rootNode = n;
            }
        }
    }
    for (ofFile& f : files) {
        if (f.getExtension() == NTRS_CONN_EXT) {

            // This one is only made for the purpose of loading primitive info
            GraphConnection* c = new GraphConnection();
            c->load(f);

            for (GraphNode* n : _nodes) {
                if (n->primitiveInfo.index == c->jointInfo.fromIndex) {
                    n->addConnection(_nodes[c->jointInfo.toIndex], c->jointInfo);
                }
            }
            delete c;
        }
    }
}

DirectedGraph::~DirectedGraph()
{
    for (GraphNode* n : _nodes) {
        delete n;
    }
}
