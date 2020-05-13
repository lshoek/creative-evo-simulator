#include "SimCreature.h"
#include "SimDefines.h"
#include "SimUtils.h"
#include "MathUtils.h"
#include "toolbox.h"

#include "DirectedGraph.h"

#define World2Loc SimUtils::b3RefFrameHelper::getTransformWorldToLocal
#define Loc2World SimUtils::b3RefFrameHelper::getTransformLocalToWorld

btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

SimCreature::SimCreature(btVector3 position, std::shared_ptr<DirectedGraph> graph, btDynamicsWorld* ownerWorld)
	: m_ownerWorld(ownerWorld), m_inEvaluation(false), m_evaluationTime(0), m_reaped(false)
{
	m_spawnPosition = position;
	m_morphologyGenome = new DirectedGraph(*graph);
	m_morphologyGenome->unfold();

	m_motorStrength = 0.025f * m_ownerWorld->getSolverInfo().m_numIterations;
	m_targetFrequency = 3.0f;
	m_targetAccumulator = 0;

	buildPhenome(m_morphologyGenome);
}

SimCreature::SimCreature(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld, bool bInit)
	: m_ownerWorld(ownerWorld), m_inEvaluation(false), m_evaluationTime(0), m_reaped(false)
{
	m_motorStrength = 0.025f * m_ownerWorld->getSolverInfo().m_numIterations;
	m_targetFrequency = 3.0f;
	m_targetAccumulator = 0;

	if (bInit) {
		initWalker(position, numLegs, ownerWorld);
	}
}

void SimCreature::buildPhenome(DirectedGraph* graph)
{
	m_numBodies = graph->getNumNodesUnfolded();
	m_numBrushes = graph->getNumEndNodesUnfolded();
	m_numJoints = m_numBodies - 1;
	m_numLegs = 0;

	randomizeSensoryMotorWeights();

	m_nodes.resize(m_numBodies);
	m_brushNodes.reserve(m_numBrushes);
	m_bodies.resize(m_numBodies);
	m_joints.reserve(m_numJoints);
	m_touchSensors.resize(m_numBodies);

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
	
	SimNode* simNodePtr = new SimNode(BodyTag, m_ownerWorld);
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
		ofLog() << "VERIFY alignment: " << std::endl << "parentAnchor . childAnchor = " << alignmentDot << std::endl; // should be parallel but point in opposite directions (dot: -1.0)
		ofLog() << "VERIFY anchors: " << std::endl << SimUtils::bulletToGlm(anchorWorld) << " | " << SimUtils::bulletToGlm(childAnchorWorld) << std::endl;
		ofLog() << "VERIFY distance: " << std::endl << parentChildDistance << " | " << (childOriginWorld - parentWorldTrans.getOrigin()).length() << std::endl;

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
		joint->setLimit(-SIMD_HALF_PI * btScalar(0.5), SIMD_HALF_PI * btScalar(0.5));
		joint->setDbgDrawSize(0.25f);
		joint->setEnabled(true);

		simNodePtr->setRigidBody(body);
		simNodePtr->setMesh(std::make_shared<ofMesh>(ofMesh::box(boxSize.x(), boxSize.y(), boxSize.z())));
		if (graphNode->primitiveInfo.bodyEnd != 0) {
			simNodePtr->setTag(BrushTag | BodyTag);
			simNodePtr->setInkColor(INK);
			m_brushNodes.push_back(simNodePtr);
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

#pragma region InitWalkerFunc
void SimCreature::initWalker(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld)
{
	// fixed for now
	bHasBallPointers = true;
	m_spawnPosition = position;

	// calculate indices
	m_numLegs = numLegs;
	m_numBrushes = m_numLegs;
	m_numBodies = 2 * m_numLegs + 1;
	m_numJoints = m_numBodies - 1;

	randomizeSensoryMotorWeights();

	// build meshes for rendering
	m_bodyMesh = std::make_shared<ofMesh>(ofMesh::sphere(gRootBodyRadius));
	m_foreLegMesh = std::make_shared<ofMesh>(ofMesh::cylinder(gForeLegRadius, gForeLegLength));
	m_legMesh = std::make_shared<ofMesh>(ofMesh::cylinder(gLegRadius, gLegLength));
	m_ballPointMesh = std::make_shared<ofMesh>(ofMesh::sphere(gForeLegRadius*0.9f));

	// Setup geometry
	m_nodes.reserve(m_numBodies);
	for (int i = 0; i < m_numBodies; i++) {
		m_nodes.push_back(new SimNode(BodyTag, m_ownerWorld));
	}
	//m_ballPointerNodes.resize(m_numBrushes);
	//for (int i = 0; i < m_ballPointerNodes.size(); i++) {
	//	m_ballPointerNodes[i] = new SimNode(BrushTag, ofColor::fromHex(0x33), m_ownerWorld);
	//	m_ballPointerNodes[i]->setMesh(m_ballPointMesh);
	//}

	// body
	m_shapes.resize(m_numBodies);
	m_ballPointerShapes.resize(m_numBrushes);

	m_shapes[0] = new btCapsuleShape(gRootBodyRadius, gRootBodyHeight); // root body capsule
	m_nodes[0]->setMesh(m_bodyMesh);

	// legs
	for (uint32_t i = 0; i < m_numLegs; i++)
	{
		m_shapes[1 + 2 * i] = new btCapsuleShape(gLegRadius, gLegLength); // leg  capsule
		m_shapes[2 + 2 * i] = new btCapsuleShape(gForeLegRadius, gForeLegLength); // fore leg capsule
		m_nodes[1 + 2 * i]->setMesh(m_legMesh);
		m_nodes[2 + 2 * i]->setMesh(m_foreLegMesh);

		m_ballPointerShapes[i] = new btSphereShape(gForeLegRadius*1.25);
	}

	// Setup rigid bodies
	btScalar rootHeightOffset = gForeLegLength;

	// Root body in global reference frame
	btTransform bodyOffsetTrans;
	bodyOffsetTrans.setIdentity();
	bodyOffsetTrans.setOrigin(position);

	// Root body in local reference frame
	btVector3 localRootBodyPos = btVector3(btScalar(0.), rootHeightOffset, btScalar(0.));

	btTransform trans;
	trans.setIdentity();
	trans.setOrigin(localRootBodyPos);

	btTransform originTrans = trans;

	m_bodies.resize(m_numBodies);
	m_ballPointerBodies.resize(m_numBrushes);

	m_bodies[0] = localCreateRigidBody(1.0, bodyOffsetTrans * trans, m_shapes[0]);
	m_ownerWorld->addRigidBody(m_bodies[0]);

	m_bodyRelativeTransforms.resize(m_numBodies);
	m_bodyRelativeTransforms[0] = btTransform::getIdentity();
	m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[0]), 0);

	m_joints.resize(m_numJoints);
	m_ballPointerJoints.resize(m_numBrushes);

	btHingeConstraint* hingeC;
	//btConeTwistConstraint* coneC;

	btTransform localA, localB, localC;

	// legs
	for (int i = 0; i < m_numLegs; i++)
	{
		float footAngle = SIMD_2_PI * i / m_numLegs;
		float footYUnitPosition = std::sin(footAngle);
		float footXUnitPosition = std::cos(footAngle);

		trans.setIdentity();
		btVector3 legCOM = btVector3(
			footXUnitPosition * (gRootBodyRadius + 0.5 * gLegLength), 
			rootHeightOffset,
			footYUnitPosition * (gRootBodyRadius + 0.5 * gLegLength)
		);
		trans.setOrigin(legCOM);

		// thigh
		btVector3 legDirection = (legCOM - localRootBodyPos).normalize();
		btVector3 kneeAxis = legDirection.cross(UP);
		trans.setRotation(btQuaternion(kneeAxis, SIMD_HALF_PI));
		m_bodies[1 + 2 * i] = localCreateRigidBody(1.0, bodyOffsetTrans * trans, m_shapes[1 + 2 * i]);
		m_bodyRelativeTransforms[1 + 2 * i] = trans;
		m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[1 + 2 * i]), 1 + 2 * i);

		// shin
		trans.setIdentity();
		trans.setOrigin(btVector3(
			footXUnitPosition * (gRootBodyRadius + gLegLength), 
			rootHeightOffset - 0.5 * gForeLegLength,
			footYUnitPosition * (gRootBodyRadius + gLegLength)
		));
		m_bodies[2 + 2 * i] = localCreateRigidBody(1.0, bodyOffsetTrans * trans, m_shapes[2 + 2 * i]);
		m_bodyRelativeTransforms[2 + 2 * i] = trans;
		m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[2 + 2 * i]), 2 + 2 * i);
		
		// ball pointers
		trans.setIdentity();
		trans.setOrigin(btVector3(
			footXUnitPosition * (gRootBodyRadius + gLegLength),
			rootHeightOffset - gForeLegLength,
			footYUnitPosition * (gRootBodyRadius + gLegLength)
		));
		m_ballPointerBodies[i] = localCreateRigidBody(1.0, bodyOffsetTrans * trans, m_ballPointerShapes[i]);
		m_ballPointerBodies[i]->setCollisionFlags(m_ballPointerBodies[i]->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);

		// hip joints
		localA.setIdentity();
		localB.setIdentity();
		localA.getBasis().setEulerZYX(0, -footAngle, 0);
		localA.setOrigin(btVector3(footXUnitPosition * gRootBodyRadius, 0.0, footYUnitPosition * gRootBodyRadius));
		localB = World2Loc(m_bodies[1 + 2 * i]->getWorldTransform(), Loc2World(m_bodies[0]->getWorldTransform(), localA));
		hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[1 + 2 * i], localA, localB);
		hingeC->setLimit(-0.75 * SIMD_PI_4, SIMD_PI_8);
		//hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
		m_joints[2 * i] = hingeC;

		// knee joints
		localA.setIdentity();
		localB.setIdentity();
		localC.setIdentity();
		localA.getBasis().setEulerZYX(0, -footAngle, 0);
		localA.setOrigin(btVector3(footXUnitPosition * (gRootBodyRadius + gLegLength), 0.0, footYUnitPosition * (gRootBodyRadius + gLegLength)));
		localB = World2Loc(m_bodies[1 + 2 * i]->getWorldTransform(), Loc2World(m_bodies[0]->getWorldTransform(), localA));
		localC = World2Loc(m_bodies[2 + 2 * i]->getWorldTransform(), Loc2World(m_bodies[0]->getWorldTransform(), localA));
		hingeC = new btHingeConstraint(*m_bodies[1 + 2 * i], *m_bodies[2 + 2 * i], localB, localC);
		//hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
		hingeC->setLimit(-SIMD_PI_8, 0.2);
		m_joints[1 + 2 * i] = hingeC;

		localA.setIdentity();
		localA.setOrigin(btVector3(0, -gForeLegLength * 0.5, 0));
		m_ballPointerJoints[i] = new btFixedConstraint(*m_bodies[2 + 2 * i], *m_ballPointerBodies[i], localA, btTransform::getIdentity());
		
		// debug drawing
		//m_joints[2 * i]->setDbgDrawSize(1.0f);
		//m_joints[1 + 2 * i]->setDbgDrawSize(1.0f);

		m_ownerWorld->addRigidBody(m_bodies[1 + 2 * i]);  // add thigh bone
		m_ownerWorld->addConstraint(m_joints[2 * i], true);  // connect thigh bone with root

		m_ownerWorld->addRigidBody(m_ballPointerBodies[i]);
		m_ownerWorld->addConstraint(m_ballPointerJoints[i], true);
	}

	// Setup some damping on the m_bodies
	for (int i = 0; i < m_numBodies; ++i)
	{
		m_bodies[i]->setUserPointer(this);
		m_bodies[i]->setDamping(0.05, 0.85);
		m_bodies[i]->setDeactivationTime(0.8);
		m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
	}

	// pass rigid bodies to node object (so they can be rendered)
	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i]->setRigidBody(m_bodies[i]);
	}
	for (int i = 0; i < m_ballPointerNodes.size(); ++i) {
		m_ballPointerBodies[i]->setUserPointer(this);
		m_ballPointerNodes[i]->setRigidBody(m_ballPointerBodies[i]);
	}
	m_touchSensors.resize(m_numBodies);

	// Should not yet be in world
	removeFromWorld();
	bInitialized = true;
}
#pragma endregion

void SimCreature::update(double timeStep)
{
	if (!bIsDebugCreature)
	{
		m_targetAccumulator += timeStep;

		// motor update rate
		if (m_targetAccumulator >= 1.0f / ((double)m_targetFrequency))
		{
			m_targetAccumulator = 0;

			// activate network
			const std::vector<double> outputs = m_controlPolicyGenome->activate(m_touchSensors);

			// update joints
			for (int i = 0; i < m_numJoints; i++)
			{
				btScalar targetAngle = 0;
				btHingeConstraint* joint = static_cast<btHingeConstraint*>(getJoints()[i]);

				targetAngle = outputs[i];

				btScalar targetLimitAngle = joint->getLowerLimit() + targetAngle * (joint->getUpperLimit() - joint->getLowerLimit());
				btScalar currentAngle = joint->getHingeAngle();
				btScalar angleError = targetLimitAngle - currentAngle;
				btScalar desiredAngularVel = 0;

				if (timeStep) {
					desiredAngularVel = angleError / timeStep;
				}
				else {
					desiredAngularVel = angleError / 0.0001f;
				}
				joint->enableAngularMotor(true, desiredAngularVel, m_motorStrength);
			}

			// update brushes
			for (int i = 0; i < m_numBrushes; i++) {
				float pressure = outputs[m_numJoints + i] * 0.5f + 0.5f;
				m_brushNodes[i]->setBrushPressure(pressure);
			}
		}

		// clear sensor signals after usage
		clearTouchSensors();
	}
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

//initialize random weights
void SimCreature::randomizeSensoryMotorWeights()
{
	m_sensoryMotorWeights.resize(m_numBodies * m_numJoints);
	for (int i = 0; i < m_numBodies; i++) {
		for (int j = 0; j < m_numJoints; j++) {
			m_sensoryMotorWeights[i + j * m_numBodies] = (double)ofRandom(1.0f) * 2.0f - 1.0f;
		}
	}
}

void SimCreature::draw()
{
	for (SimNode* &n : m_nodes) {
		n->draw();
	}
	for (SimNode* &n : m_ballPointerNodes) {
		n->draw();
	}
}

void SimCreature::drawImmediate()
{
	for (SimNode*& n : m_nodes) {
		n->drawImmediate();
	}
	for (SimNode*& n : m_ballPointerNodes) {
		n->drawImmediate();
	}
}

btTypedConstraint** SimCreature::getJoints()
{
	return &m_joints[0];
}

btRigidBody** SimCreature::getRigidBodies()
{
	return &m_bodies[0];
}

SimNode** SimCreature::getSimNodes()
{
	return &m_nodes[0];
}

void SimCreature::setControlPolicyGenome(const GenomeBase& genome)
{
	m_controlPolicyGenome = new GenomeBase(genome);
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

btScalar* SimCreature::getSensoryMotorWeights()
{
	return &m_sensoryMotorWeights[0];
}

void SimCreature::addToWorld()
{
	int i;
	// add all bodies and shapes
	for (i = 0; i < m_numBodies; ++i) {
		m_ownerWorld->addRigidBody(m_bodies[i]);
	}

	// add all constraints
	// important! If you add constraints back, you must set bullet physics to disable collision between constrained bodies
	for (i = 0; i < m_numJoints; ++i) {
		m_ownerWorld->addConstraint(m_joints[i], true);
	}
}

void SimCreature::removeFromWorld()
{
	// Remove all constraints
	for (int i = 0; i < m_numJoints; ++i) {
		m_ownerWorld->removeConstraint(m_joints[i]);
	}

	// Remove all bodies
	for (int i = 0; i < m_numBodies; ++i) {
		m_ownerWorld->removeRigidBody(m_bodies[i]);
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
	for (int i = 0; i < m_ballPointerNodes.size(); ++i) {
		m_ballPointerNodes[i]->setShader(m_shader);
		m_ballPointerNodes[i]->setMaterial(m_material);
		m_ballPointerNodes[i]->setTexture(m_texture);
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

btScalar SimCreature::getEvaluationTime() const
{
	return m_evaluationTime;
}

void SimCreature::setEvaluationTime(btScalar evaluationTime)
{
	m_evaluationTime = evaluationTime;
}

bool SimCreature::isInEvaluation() const
{
	return m_inEvaluation;
}

void SimCreature::setInEvaluation(bool inEvaluation)
{
	m_inEvaluation = inEvaluation;
}

bool SimCreature::isReaped() const
{
	return m_reaped;
}

void SimCreature::setReaped(bool reaped)
{
	m_reaped = reaped;
}

int SimCreature::getIndex() const
{
	return m_index;
}

SimCreature::~SimCreature()
{
	removeFromWorld();

	// Delete all joints/constraints
	for (int i = 0; i < m_numJoints; ++i) {
		delete m_joints[i];
		m_joints[i] = 0;
	}
	// Delete all nodes and their corresponding bodies and shapes
	for (int i = 0; i < m_numBodies; ++i) {
		delete m_nodes[i];
		m_nodes[i] = 0;
	}
	// Clear brush pointers to deleted nodes
	for (int i = 0; i < m_numBrushes; ++i) {
		m_brushNodes[i] = 0;
	}
	if (m_controlPolicyGenome) delete m_controlPolicyGenome;
	if (m_morphologyGenome) delete m_morphologyGenome;
}
