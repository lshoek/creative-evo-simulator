#include "Simulator/SimCreature.h"
#include "Simulator/SimDefines.h"
#include "Utils/SimUtils.h"
#include "Utils/MathUtils.h"
#include "Utils/toolbox.h"
#include "Genome/DirectedGraph.h"

#define World2Loc SimUtils::b3RefFrameHelper::getTransformWorldToLocal
#define Loc2World SimUtils::b3RefFrameHelper::getTransformLocalToWorld

btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

SimCreature::SimCreature(btVector3 position, const std::shared_ptr<DirectedGraph>& graph, btDynamicsWorld* ownerWorld)
	: m_ownerWorld(ownerWorld)
{
	m_spawnPosition = position;
	m_bodyGenome = new DirectedGraph(*graph);
	m_bodyGenome->unfold();

	m_motorStrength = 0.025f * m_ownerWorld->getSolverInfo().m_numIterations;
	m_targetFrequency = 3;
	m_targetAccumulator = 0;

	m_bodyColor = ofColor::fromHsb(ofRandom(255), 0.7f * 255, 255, 255);

	buildPhenome(m_bodyGenome);
}

void SimCreature::buildPhenome(DirectedGraph* graph)
{
	m_numBodies = graph->getNumNodesUnfolded();
	m_numBrushes = graph->getNumBrushes();
	m_numJoints = m_numBodies - 1;
	m_numOutputs = m_numJoints + m_numBrushes;

	m_nodes.resize(m_numBodies);
	m_brushNodes.reserve(m_numBrushes);
	m_bodies.resize(m_numBodies);
	m_joints.reserve(m_numJoints);
	m_touchSensors.resize(m_numBodies);
	m_outputs.resize(m_numOutputs);

	std::vector<int> recursionLimits(graph->getNodes().size());
	for (int i = 0; i < recursionLimits.size(); i++) {
		recursionLimits[i] = graph->getNodes()[i]->getRecursionLimit();
	}

	int segmentIndex = 0;
	dfs(graph->getRootNode(), nullptr, nullptr, graph, btVector3(1., 1., 1.), 1.0, 0.0, recursionLimits, segmentIndex);
}

void SimCreature::dfs(
	GraphNode* graphNode, GraphConnection* incoming, SimNode* parentSimNode, DirectedGraph* graph, 
	btVector3 parentDims, btScalar cascadingScale, btScalar attachment, std::vector<int> recursionLimits, int& segmentIndex)
{
	bool bIsRootNode = (incoming == nullptr);

	if (!bIsRootNode) {
		cascadingScale *= incoming->jointInfo.scalingFactor;
	}
	recursionLimits[graphNode->getGraphIndex()]--;
	
	SimNode* simNodePtr = new SimNode(BodyTag, m_bodyColor, m_ownerWorld);
	simNodePtr->setCreatureOwner(this);

	if (!bIsRootNode) {
		segmentIndex++;

		btTransform parentWorldTrans = parentSimNode->getRigidBody()->getWorldTransform();

		btVector3 boxSizeParent = parentDims;
		btVector3 boxSize = graphNode->primitiveInfo.dimensions; // *cascadingScale;
		boxSize = btVector3(btMax(boxSize.x(), GraphNode::minSize), btMax(boxSize.y(), GraphNode::minSize), btMax(boxSize.z(), GraphNode::minSize));
		boxSize = btVector3(btMin(boxSize.x(), GraphNode::maxSize), btMin(boxSize.y(), GraphNode::maxSize), btMin(boxSize.z(), GraphNode::maxSize));
		
		btVector3 halfExtentsParent = boxSizeParent * 0.5;
		btVector3 halfExtents = boxSize * 0.5;

		// Calculate parent attachment point from plane
		btVector3 planeForward = incoming->parent->primitiveInfo.parentAttachmentPlane.normalize();
		btVector3 planeRight = SimUtils::minAbsDotAxis(planeForward);

		// Calculate local anchor points
		btVector3 parentAnchorNormalLocal = planeRight.rotate(planeForward, SIMD_2_PI * attachment).normalize();
		btVector3 childAnchorNormalLocal = incoming->jointInfo.childAnchorDir.normalize();

		// Calculate local anchor points
		btVector3 parentSurfaceNormalLocal, childSurfaceNormalLocal;
		btVector3 parentAnchorLocal = parentAnchorNormalLocal * abs(SimUtils::distToSurface(parentAnchorNormalLocal, halfExtentsParent, parentSurfaceNormalLocal));
		btVector3 childAnchorLocal = childAnchorNormalLocal * abs(SimUtils::distToSurface(childAnchorNormalLocal, halfExtents, childSurfaceNormalLocal));

		// Calculate world anchor point and the child origin in world
		btVector3 anchorWorld = parentWorldTrans * parentAnchorLocal;
		btVector3 parentAnchorNormalWorld = (parentWorldTrans.getBasis() * parentAnchorNormalLocal).normalized();

		btVector3 childOriginWorld = anchorWorld + parentAnchorNormalWorld * childAnchorLocal.length();
		btVector3 untransformedChildAnchorNormalWorld = (parentWorldTrans.getBasis() * childAnchorNormalLocal).normalized();

		// Rotation to align parent and child normals
		btQuaternion anchorAlignmentRot = SimUtils::glmToBullet(glm::rotation(
			SimUtils::bulletToGlm(untransformedChildAnchorNormalWorld),
			SimUtils::bulletToGlm(-parentAnchorNormalWorld)
		));
		btVector3 childAnchorWorld = childOriginWorld + btTransform(anchorAlignmentRot) * (parentWorldTrans.getBasis() * childAnchorLocal);
		btVector3 childAnchorNormalWorld = (childAnchorWorld - childOriginWorld).normalized();

		btScalar alignmentDot = btDot(parentAnchorNormalWorld, childAnchorNormalWorld);
		btScalar parentChildDistance = parentAnchorLocal.length() + childAnchorLocal.length();

		/// Verification
		//ofLog() << "VERIFY alignment: " << std::endl << "parentAnchor . childAnchor = " << alignmentDot << std::endl; // should be parallel but point in opposite directions (dot: -1.0)
		//ofLog() << "VERIFY anchors: " << std::endl << SimUtils::bulletToGlm(anchorWorld) << " | " << SimUtils::bulletToGlm(childAnchorWorld) << std::endl;
		//ofLog() << "VERIFY distance: " << std::endl << parentChildDistance << " | " << (childOriginWorld - parentWorldTrans.getOrigin()).length() << std::endl;

		// Build final child transform
		btTransform childWorldTrans = btTransform(btMatrix3x3::getIdentity(), childOriginWorld) * btTransform(anchorAlignmentRot) * btTransform(parentWorldTrans.getBasis());

		btCollisionShape* shape = new btBoxShape(halfExtents);
		btRigidBody* body = localCreateRigidBody(1.0f, childWorldTrans, shape);
		body->setUserPointer(simNodePtr);

		// Set up reference frames for axes
		btVector3 jointAxis = incoming->jointInfo.axis.normalize();
		btVector3 parentChildForward = (parentWorldTrans.getOrigin() - childWorldTrans.getOrigin());

		if (parentChildForward.length() < SIMD_EPSILON) {
			parentChildForward = UP;
		}
		parentChildForward.normalize();

		if (abs(btDot(jointAxis, parentChildForward)) > btScalar(1.) - SIMD_EPSILON) {
			jointAxis = SimUtils::minAbsDotAxis(jointAxis);
		}
		btVector3 rotAxis = (jointAxis.cross(parentChildForward)).normalize();
		btScalar theta = acos(jointAxis.dot(parentChildForward));
		btQuaternion qq = btQuaternion(rotAxis, theta);

		btTransform frameInWorld;
		frameInWorld.setIdentity();
		frameInWorld.setOrigin(anchorWorld);

		btTransform parentFrameInWorld;
		parentFrameInWorld.setIdentity();
		parentFrameInWorld.setOrigin(anchorWorld);
		parentFrameInWorld.setRotation(childWorldTrans.inverse().getRotation() * qq);

		btTransform childFrameInWorld;
		childFrameInWorld.setIdentity();
		childFrameInWorld.setOrigin(anchorWorld);
		childFrameInWorld.setRotation(childWorldTrans.inverse().getRotation() * qq);

		btHingeConstraint* joint = new btHingeConstraint(
			*parentSimNode->getRigidBody(), *body,
			parentWorldTrans.inverse() * parentFrameInWorld, 
			childWorldTrans.inverse() * childFrameInWorld
		);
		joint->setLimit(-SIMD_HALF_PI * btScalar(0.75), SIMD_HALF_PI * btScalar(0.75));
		joint->setDbgDrawSize(0.25f);
		joint->setEnabled(true);

		simNodePtr->setRigidBody(body);
		simNodePtr->setMesh(std::make_shared<ofMesh>(ofMesh::box(boxSize.x(), boxSize.y(), boxSize.z())));
		if (!m_bHasBrush && graphNode->primitiveInfo.brush != 0) {
			simNodePtr->setTag(BrushTag | BodyTag);
			simNodePtr->setInkColor(INK);
			m_brushNodes.push_back(simNodePtr);
			m_bHasBrush = true;
		}

		m_nodes[segmentIndex] = simNodePtr;
		m_bodies[segmentIndex] = body;
		m_bodyTouchSensorIndexMap.insert(btHashPtr(body), segmentIndex);
		m_joints.push_back(joint);

		parentDims = boxSize;
	}
	else { // ROOT BODY
		btScalar startHeight = GraphNode::maxSize * 2.0;
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(m_spawnPosition + btVector3(0, startHeight, 0));

		btVector3 boxSize = graphNode->primitiveInfo.dimensions;
		btVector3 halfExtents = boxSize * 0.5;

		btCollisionShape* shape = new btBoxShape(halfExtents);
		btRigidBody* body = localCreateRigidBody(1.0, trans, shape);
		body->setUserPointer(simNodePtr);

		simNodePtr->setRigidBody(body);
		simNodePtr->setMesh(std::make_shared<ofMesh>(ofMesh::box(boxSize.x(), boxSize.y(), boxSize.z())));

		m_nodes[segmentIndex] = simNodePtr;
		m_bodies[segmentIndex] = body;
		m_bodyTouchSensorIndexMap.insert(btHashPtr(body), segmentIndex);

		parentDims = boxSize;
	}

	int connectionIndex = 0;
	for (GraphConnection* c : graphNode->getConnections()) {
		if (recursionLimits[c->child->getGraphIndex()] > 0) {
			btScalar attachment = connectionIndex/float(graphNode->getConnections().size());
			dfs(c->child, c, simNodePtr, graph, parentDims, cascadingScale, attachment, recursionLimits, segmentIndex);
		}
		connectionIndex++;
	}
}

bool SimCreature::isAwaitingEffectorUpdate()
{
	return m_bAwaitingEffectorUpdate;
}

void SimCreature::updateTimeStep(double timeStep)
{
	if (!m_bAwaitingEffectorUpdate) {
		m_timeStep = timeStep;
		m_targetAccumulator += m_timeStep;
		if (m_targetAccumulator >= 1.0 / (double)m_targetFrequency) {
			m_targetAccumulator = 0;
			m_bAwaitingEffectorUpdate = true;
		}
	}
}

void SimCreature::updateOutputs(const std::vector<float>& outputs)
{
	//m_outputs = std::vector<double>(outputs.begin(), outputs.end());
	m_outputs = outputs;
	m_bAwaitingEffectorUpdate = false;
}

const std::vector<float>& SimCreature::getOutputs()
{
	return m_outputs;
}

void SimCreature::update()
{
	// update joints
	for (int i = 0; i < m_numJoints; i++)
	{
		btScalar targetAngle = 0;
		btHingeConstraint* joint = static_cast<btHingeConstraint*>(getJoints()[i]);

		targetAngle = m_outputs[i];

		btScalar targetLimitAngle = joint->getLowerLimit() + targetAngle * (joint->getUpperLimit() - joint->getLowerLimit());
		btScalar currentAngle = joint->getHingeAngle();
		btScalar angleError = targetLimitAngle - currentAngle;
		btScalar desiredAngularVel = 0;

		if (m_timeStep) {
			desiredAngularVel = angleError / m_timeStep;
		}
		else {
			desiredAngularVel = angleError / 0.0001f;
		}
		joint->enableAngularMotor(true, desiredAngularVel, m_motorStrength);
	}

	// update brushes
	float pressure = m_outputs[m_numOutputs - m_numBrushes] * 0.5f + 0.5f;
	for (int i = 0; i < m_numBrushes; i++) {
		m_brushNodes[i]->setBrushPressure(pressure);
	}

	// clear sensor signals after usage
	clearTouchSensors();
}

btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
{
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	btDefaultMotionState* motionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setDamping(0.05, 0.85);
	body->setDeactivationTime(0.8);
	body->setSleepingThresholds(0.5f, 0.5f);

	return body;
}

bool SimCreature::feasibilityCheck()
{
	addToWorld();
	m_ownerWorld->performDiscreteCollisionDetection();

	int numManifolds = m_ownerWorld->getDispatcher()->getNumManifolds();
	bool bIsFeasible = true;

	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contactManifold = m_ownerWorld->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* o1 = (btCollisionObject*)(contactManifold->getBody0());
		btCollisionObject* o2 = (btCollisionObject*)(contactManifold->getBody1());

		// Check for collision with self (excluding linked bodies)
		if (((SimNode*)o1->getUserPointer())->getCreaturePtr() == this && 
			((SimNode*)o2->getUserPointer())->getCreaturePtr() == this) {
			bIsFeasible = false;
		}
		// Make sure to remove all manifolds again
		contactManifold->clearManifold();	
	}
	removeFromWorld();

	return bIsFeasible;
}

void SimCreature::draw()
{
	for (SimNode* &n : m_nodes) {
		n->draw();
	}
}

void SimCreature::drawImmediate()
{
	for (SimNode*& n : m_nodes) {
		n->drawImmediate();
	}
}
const DirectedGraph& SimCreature::getBodyGenome()
{
	return *m_bodyGenome;
}

uint32_t SimCreature::getID()
{
	return m_id;
}

uint32_t SimCreature::getNumOutputs()
{
	return m_numOutputs;
}

uint32_t SimCreature::getNumJoints()
{
	return m_numJoints;
}

btTypedConstraint** SimCreature::getJoints()
{
	return &m_joints[0];
}

std::vector<float> SimCreature::getJointState()
{
	std::vector<float> jointState;
	jointState.reserve(m_numJoints);

	for (btTypedConstraint* joint : m_joints) {
		btHingeConstraint* j = static_cast<btHingeConstraint*>(joint);
		jointState.push_back((j->getHingeAngle() - j->getLowerLimit())/(j->getUpperLimit()-j->getLowerLimit()));
	}
	return jointState;
}

btRigidBody** SimCreature::getRigidBodies()
{
	return &m_bodies[0];
}

SimNode** SimCreature::getSimNodes()
{
	return &m_nodes[0];
}

void SimCreature::setSensorMode(SensorMode mode)
{
	_sensorMode = mode;
}

double SimCreature::getTouchSensor(int i)
{
	return m_touchSensors[i];
}

void SimCreature::setTouchSensor(void* bodyPointer)
{
	m_touchSensors[*m_bodyTouchSensorIndexMap.find(btHashPtr(bodyPointer))] = 1.0;
}

void SimCreature::clearTouchSensors()
{
	for (int i = 0; i < m_numBodies; i++) {
		m_touchSensors[i] = 0.0;
	}
}

void SimCreature::addToWorld()
{
	for (uint32_t i = 0; i < m_numJoints; i++) {
		m_ownerWorld->addConstraint(m_joints[i], true);
	}
	for (SimNode* node : m_nodes) {
		node->addToWorld();
	}
}

void SimCreature::removeFromWorld()
{
	for (uint32_t i = 0; i < m_numJoints; i++) {
		m_ownerWorld->removeConstraint(m_joints[i]);
	}
	for (SimNode* node : m_nodes) {
		node->removeFromWorld();
	}
}

btVector3 SimCreature::getSpawnPosition() const
{
	return m_spawnPosition;
}

btVector3 SimCreature::getCenterOfMassPosition() const
{
	btVector3 finalPosition(0, 0, 0);

	for (int i = 0; i < m_numBodies; i++) {
		finalPosition += m_bodies[i]->getCenterOfMassPosition();
	}
	finalPosition /= m_numBodies;
	return finalPosition;
}

void SimCreature::clearForces()
{
	for (int i = 0; i < m_numBodies; ++i) {
		m_bodies[i]->clearForces();
		m_bodies[i]->setAngularVelocity(btVector3(0, 0, 0));
		m_bodies[i]->setLinearVelocity(btVector3(0, 0, 0));
	}
}

void SimCreature::setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<ofMaterial> mtl, std::shared_ptr<ofTexture> tex)
{
	m_shader = shader;
	m_material = mtl;
	m_texture = tex;

	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i]->setShader(m_shader);
		m_nodes[i]->setMaterial(m_material);
		m_nodes[i]->setTexture(m_texture);
	}
}

void SimCreature::setShader(std::shared_ptr<ofShader> shader)
{
	m_shader = shader;

	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i]->setShader(m_shader);
	}
}

void SimCreature::setMaterial(std::shared_ptr<ofMaterial> mtl)
{
	m_material = mtl;

	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i]->setMaterial(m_material);
	}
}

void SimCreature::setLight(std::shared_ptr<ofLight> light)
{
	m_light = light;

	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i]->setLight(m_light);
	}
}

void SimCreature::setTexture(std::shared_ptr<ofTexture> tex)
{
	m_texture = tex;

	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i]->setTexture(m_texture);
	}
}

SimCreature::~SimCreature()
{
	removeFromWorld();

	for (auto &c : m_joints) {
		delete c;
	}
	for (auto &node : m_nodes) {
		delete node;
	}
	if (m_bodyGenome) delete m_bodyGenome;
}
