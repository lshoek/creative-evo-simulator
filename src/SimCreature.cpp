#include "SimCreature.h"
#include "SimUtils.h"
#include "toolbox.h"

#define World2Loc SimUtils::b3RefFrameHelper::getTransformWorldToLocal
#define Loc2World SimUtils::b3RefFrameHelper::getTransformLocalToWorld

btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform, btCollisionShape* shape);

SimCreature::SimCreature(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bInit)
	: m_ownerWorld(ownerWorld), m_inEvaluation(false), m_evaluationTime(0), m_reaped(false)
{
	m_motorStrength = 0.05f * m_ownerWorld->getSolverInfo().m_numIterations;
	m_motorStrength = 1.0f;

	m_targetFrequency = 6.0f;
	m_targetAccumulator = 0;

	if (bInit) {
		init(position, numLegs, ownerWorld, positionOffset);
	}
}

void SimCreature::init(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
{
	// fixed for now
	bHasBallPointers = true;

	// calculate indices
	m_numLegs = numLegs;
	m_numBallPointers = m_numLegs;
	m_numBodyParts = 2 * m_numLegs + 1;
	m_numJoints = m_numBodyParts - 1;

	randomizeSensoryMotorWeights();

	btVector3 vUp = SimUtils::glmToBullet(up);

	// build meshes for rendering
	m_bodyMesh = std::make_shared<ofMesh>(ofMesh::sphere(gRootBodyRadius));
	m_foreLegMesh = std::make_shared<ofMesh>(ofMesh::cylinder(gForeLegRadius, gForeLegLength));
	m_legMesh = std::make_shared<ofMesh>(ofMesh::cylinder(gLegRadius, gLegLength));
	m_ballPointMesh = std::make_shared<ofMesh>(ofMesh::sphere(gLegRadius));

	// Setup geometry
	m_nodes.reserve(m_numBodyParts);
	for (int i = 0; i < m_numBodyParts; i++) {
		m_nodes.push_back(new SimNode(BodyTag));
	}
	m_ballPointerNodes.resize(m_numBallPointers);
	for (int i = 0; i < m_ballPointerNodes.size(); i++) {
		m_ballPointerNodes[i] = new SimNode(BrushTag, ofColor::fromHex(0x33));
		m_ballPointerNodes[i]->setMesh(m_ballPointMesh);
	}

	// body
	m_shapes.resize(m_numBodyParts);
	m_ballPointerShapes.resize(m_numBallPointers);

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
	btScalar rootAboveGroundHeight = gForeLegLength;
	btTransform bodyOffset;
	bodyOffset.setIdentity();
	bodyOffset.setOrigin(positionOffset);

	// root body
	btVector3 localRootBodyPosition = btVector3(position.x(), rootAboveGroundHeight, position.z()); // root body position in local reference frame
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(localRootBodyPosition);

	btTransform originTransform = transform;

	m_bodies.resize(m_numBodyParts);
	m_ballPointerBodies.resize(m_numBallPointers);

	m_bodies[0] = localCreateRigidBody(1.0, bodyOffset * transform, m_shapes[0]);
	m_ownerWorld->addRigidBody(m_bodies[0]);

	m_bodyRelativeTransforms.resize(m_numBodyParts);
	m_bodyRelativeTransforms[0] = btTransform::getIdentity();
	m_bodies[0]->setUserPointer(this);
	m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[0]), 0);

	m_joints.resize(m_numJoints);
	m_ballPointerJoints.resize(m_numBallPointers);

	btHingeConstraint* hingeC;
	//btConeTwistConstraint* coneC;

	btTransform localA, localB, localC;

	// legs
	for (int i = 0; i < m_numLegs; i++)
	{
		float footAngle = 2 * SIMD_PI * i / m_numLegs;  // legs are uniformly distributed around the root body
		float footYUnitPosition = std::sin(footAngle); // y position of the leg on the unit circle
		float footXUnitPosition = std::cos(footAngle); // x position of the leg on the unit circle

		transform.setIdentity();
		btVector3 legCOM = btVector3(
			footXUnitPosition * (gRootBodyRadius + 0.5 * gLegLength), 
			rootAboveGroundHeight, 
			footYUnitPosition * (gRootBodyRadius + 0.5 * gLegLength)
		);
		transform.setOrigin(legCOM);

		// thigh
		btVector3 legDirection = (legCOM - localRootBodyPosition).normalize();
		btVector3 kneeAxis = legDirection.cross(vUp);
		transform.setRotation(btQuaternion(kneeAxis, SIMD_HALF_PI));
		m_bodies[1 + 2 * i] = localCreateRigidBody(1.0, bodyOffset * transform, m_shapes[1 + 2 * i]);
		m_bodyRelativeTransforms[1 + 2 * i] = transform;
		m_bodies[1 + 2 * i]->setUserPointer(this);
		m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[1 + 2 * i]), 1 + 2 * i);

		// shin
		transform.setIdentity();
		transform.setOrigin(btVector3(
			footXUnitPosition * (gRootBodyRadius + gLegLength), 
			rootAboveGroundHeight - 0.5 * gForeLegLength, 
			footYUnitPosition * (gRootBodyRadius + gLegLength)
		));
		m_bodies[2 + 2 * i] = localCreateRigidBody(1.0, bodyOffset * transform, m_shapes[2 + 2 * i]);
		m_bodyRelativeTransforms[2 + 2 * i] = transform;
		m_bodies[2 + 2 * i]->setUserPointer(this);
		m_bodyTouchSensorIndexMap.insert(btHashPtr(m_bodies[2 + 2 * i]), 2 + 2 * i);
		
		// ball pointers
		transform.setIdentity();
		transform.setOrigin(btVector3(
			footXUnitPosition * (gRootBodyRadius + gLegLength),
			rootAboveGroundHeight - gForeLegLength,
			footYUnitPosition * (gRootBodyRadius + gLegLength)
		));
		m_ballPointerBodies[i] = localCreateRigidBody(1.0, bodyOffset * transform, m_ballPointerShapes[i]);
		m_ballPointerBodies[i]->setUserPointer(this);
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

		//if (nnWalkersDemo->detectCollisions())
		if (false)
		{  // if thigh bone causes collision, remove it again
			m_ownerWorld->removeRigidBody(m_bodies[1 + 2 * i]);
			m_ownerWorld->removeConstraint(m_joints[2 * i]);  // disconnect thigh bone from root
		}
		else
		{
			m_ownerWorld->addRigidBody(m_bodies[2 + 2 * i]);         // add shin bone
			m_ownerWorld->addConstraint(m_joints[1 + 2 * i], true);  // connect shin bone with thigh

			//if (nnWalkersDemo->detectCollisions())
			if (false)
			{  // if shin bone causes collision, remove it again
				m_ownerWorld->removeRigidBody(m_bodies[2 + 2 * i]);
				m_ownerWorld->removeConstraint(m_joints[1 + 2 * i]);  // disconnect shin bone from thigh
			}
		}
	}

	// Setup some damping on the m_bodies
	for (int i = 0; i < m_numBodyParts; ++i)
	{
		m_bodies[i]->setDamping(0.05, 0.85);
		m_bodies[i]->setDeactivationTime(0.8);
		m_bodies[i]->setSleepingThresholds(0.5f, 0.5f);
	}

	// pass rigid bodies to node object (so they can be rendered)
	for (int i = 0; i < m_nodes.size(); ++i) {
		m_nodes[i]->setRigidBody(m_bodies[i]);
	}
	for (int i = 0; i < m_ballPointerNodes.size(); ++i) {
		m_ballPointerBodies[i]->setUserIndex(BrushTag);
		m_ballPointerNodes[i]->setRigidBody(m_ballPointerBodies[i]);
	}
	m_touchSensors.resize(m_numBodyParts);

	removeFromWorld(); // it should not yet be in the world
	bInitialized = true;
}

// use this for debug only
void SimCreature::initSnake(btVector3 position, unsigned int numNodes, float boxExtents, float distPct, bool bRandomSize)
{
	m_nodes.resize(numNodes);

	glm::vec3 start = glm::vec3(position.x(), boxExtents, position.z());
	glm::vec3 size = glm::vec3(boxExtents);
	glm::vec3 dir = right;
	glm::vec3 cur, prev = start;
	glm::quat lookatRot;

	float maxDistBetweenNodes = ofLerp(0, sqrt((boxExtents * 2) * (boxExtents * 2) * 3), distPct);

	cur = start;
	for (int i = 0; i < numNodes; i++)
	{
		char id[32];
		sprintf_s(id, "testNode_%d", i);

		lookatRot = tb::quatDiff(right, glm::rotation(right, dir) * glm::rotate(right, 0.125f, up));
		dir = glm::normalize(lookatRot * right);

		prev = cur;
		cur = prev + dir * maxDistBetweenNodes;

		if (bRandomSize) {
			size = glm::vec3(ofRandom(boxExtents / 2, boxExtents), boxExtents, ofRandom(boxExtents / 2, boxExtents));
		}
		m_nodes[i] = new SimNode(AnonymousTag);
		m_nodes[i]->initBox(cur, size, 1.0f);
		m_nodes[i]->setRotation(lookatRot);

		m_ownerWorld->addRigidBody(m_nodes[i]->getRigidBody());

		// constraints
		btVector3 ax = ofRandom(1.0f) > 0.5f ? btVector3(0, 0, 1) : btVector3(0, 1, 0);
		if (i > 0) {
			btVector3 pivotInA(0, 0, 0);
			btVector3 pivotInB =
				m_nodes[i - 1]->getRigidBody()->getWorldTransform().inverse()(m_nodes[i]->getRigidBody()->getWorldTransform()(pivotInA));

			btTransform frameInA, frameInB = btTransform::getIdentity();
			frameInA.setOrigin(pivotInA);
			frameInB.setOrigin(pivotInB);

			btHingeConstraint* joint = new btHingeConstraint(
				*m_nodes[i]->getRigidBody(), *m_nodes[i - 1]->getRigidBody(), pivotInA, pivotInB, ax, ax);

			joint->setLimit(SIMD_HALF_PI * -0.5f, SIMD_HALF_PI * 0.5f);
			m_ownerWorld->addConstraint(joint);
		}
	}
	// disables collision on linked nodes
	for (int i = 1; i < numNodes - 1; i++) {
		m_nodes[i]->getRigidBody()->setIgnoreCollisionCheck(m_nodes[i - 1]->getRigidBody(), true);
		m_nodes[i]->getRigidBody()->setIgnoreCollisionCheck(m_nodes[i + 1]->getRigidBody(), true);
	}
	bIsDebugCreature = true;
	bInitialized = true;
}

void handleCollisions(btDynamicsWorld* worldPtr)
{
	int numManifolds = worldPtr->getDispatcher()->getNumManifolds();

	for (int i=0; i < numManifolds; i++)
	{
		btPersistentManifold* contactManifold = worldPtr->getDispatcher()->getManifoldByIndexInternal(i);
		btCollisionObject* o1 = (btCollisionObject*)(contactManifold->getBody0());
		btCollisionObject* o2 = (btCollisionObject*)(contactManifold->getBody1());

		for (int j = 0; j < contactManifold->getNumContacts(); j++)
		{
			// collision with something other than itself
			if ((o1->getUserIndex() == BodyTag && o2->getUserIndex() != BodyTag) ||
				(o1->getUserIndex() != BodyTag && o2->getUserIndex() == BodyTag)) 
			{
				SimCreature* creaturePtr = nullptr;

				// brushtag should always have a simcreature as user pointer
				if (o1->getUserIndex() == BodyTag) {
					creaturePtr = (SimCreature*)o1->getUserPointer();
					creaturePtr->setTouchSensor(o1);
				}
				else if (o2->getUserIndex() == BodyTag) {
					creaturePtr = (SimCreature*)o2->getUserPointer();
					creaturePtr->setTouchSensor(o2);
				}
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				if (worldPtr->getDebugDrawer() != NULL) {
					worldPtr->getDebugDrawer()->drawSphere(pt.getPositionWorldOnA(), 10.0, btVector3(1., 0., 0.));
				}
			}
			// check for canvas
			if ((o1->getUserIndex() == BodyTag && o2->getUserIndex() == CanvasTag) ||
				(o1->getUserIndex() == CanvasTag && o2->getUserIndex() == BodyTag))
			{
				SimCanvasNode* canvasPtr = nullptr;

				// brushtag should always have a simcreature as user pointer
				if (o1->getUserIndex() == CanvasTag) {
					canvasPtr = (SimCanvasNode*)o1->getUserPointer();
				}
				else if (o2->getUserIndex() == CanvasTag) {
					canvasPtr = (SimCanvasNode*)o2->getUserPointer();
				}
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				canvasPtr->addBrushStroke(pt.getPositionWorldOnA(), pt.getAppliedImpulse());
			}
		}
		//contactManifold->clearManifold();	
	}
}

void SimCreature::update(double timeStep)
{
	if (!bIsDebugCreature)
	{
		handleCollisions(m_ownerWorld);

		m_time += timeStep;
		m_targetAccumulator += timeStep;

		// motor update rate
		if (m_targetAccumulator >= 1.0f / ((double)m_targetFrequency))
		{
			m_targetAccumulator = 0;

			for (int i = 0; i < 2 * m_numLegs; i++)
			{
				btScalar targetAngle = 0;
				btHingeConstraint* joint = static_cast<btHingeConstraint*>(getJoints()[i]);

				// neural network movement
				// accumulate sensor inputs with weights
				for (int j = 0; j < m_numJoints; j++) {
					targetAngle += getSensoryMotorWeights()[i + j * m_numBodyParts] * getTouchSensor(i);
				}
				// apply the activation function
				targetAngle = (std::tanh(targetAngle) + 1.0f) * 0.5f;

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

	return body;
}

//initialize random weights
void SimCreature::randomizeSensoryMotorWeights()
{
	m_sensoryMotorWeights.resize(m_numBodyParts * m_numJoints);
	for (int i = 0; i < m_numBodyParts; i++) {
		for (int j = 0; j < m_numJoints; j++) {
			m_sensoryMotorWeights[i + j * m_numBodyParts] = (double)ofRandom(1.0f) * 2.0f - 1.0f;
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

void SimCreature::setTouchSensor(void* bodyPointer)
{
	m_touchSensors[*m_bodyTouchSensorIndexMap.find(btHashPtr(bodyPointer))] = true;
}

void SimCreature::clearTouchSensors()
{
	for (int i = 0; i < m_numBodyParts; i++) {
		m_touchSensors[i] = false;
	}
}

bool SimCreature::getTouchSensor(int i)
{
	return m_touchSensors[i];
}

btScalar* SimCreature::getSensoryMotorWeights()
{
	return &m_sensoryMotorWeights[0];
}

void SimCreature::addToWorld()
{
	int i;
	// add all bodies and shapes
	for (i = 0; i < m_numBodyParts; ++i) {
		m_ownerWorld->addRigidBody(m_bodies[i]);
	}

	// add all constraints
	// important! If you add constraints back, you must set bullet physics to disable collision between constrained bodies
	for (i = 0; i < m_numJoints; ++i) {
		m_ownerWorld->addConstraint(m_joints[i], true);
	}
	m_startPosition = getPosition();
}

void SimCreature::removeFromWorld()
{
	int i;

	// Remove all constraints
	for (i = 0; i < m_numJoints; ++i) {
		m_ownerWorld->removeConstraint(m_joints[i]);
	}

	// Remove all bodies
	for (i = 0; i < m_numBodyParts; ++i) {
		m_ownerWorld->removeRigidBody(m_bodies[i]);
	}
}

btVector3 SimCreature::getPosition() const
{
	btVector3 finalPosition(0, 0, 0);

	for (int i = 0; i < m_numBodyParts; i++) {
		finalPosition += m_bodies[i]->getCenterOfMassPosition();
	}
	finalPosition /= m_numBodyParts;
	return finalPosition;
}

btScalar SimCreature::getDistanceFitness() const
{
	btScalar distance = 0;
	distance = (getPosition() - m_startPosition).length2();

	return distance;
}

btScalar SimCreature::getFitness() const
{
	return getDistanceFitness();  // for now it is only distance
}

void SimCreature::resetAt(const btVector3& position)
{
	btTransform resetPosition(btQuaternion::getIdentity(), position);
	for (int i = 0; i < m_numBodyParts; ++i) {
		m_bodies[i]->setWorldTransform(resetPosition * m_bodyRelativeTransforms[i]);
		if (m_bodies[i]->getMotionState()) {
			m_bodies[i]->getMotionState()->setWorldTransform(resetPosition * m_bodyRelativeTransforms[i]);
		}
		m_bodies[i]->clearForces();
		m_bodies[i]->setAngularVelocity(btVector3(0, 0, 0));
		m_bodies[i]->setLinearVelocity(btVector3(0, 0, 0));
	}

	clearTouchSensors();
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
	// Remove all constraints
	for (int i = 0; i < m_numJoints; ++i) {
		m_ownerWorld->removeConstraint(m_joints[i]);
		delete m_joints[i];
		m_joints[i] = 0;
	}
	for (int i = 0; i < m_numBallPointers; ++i) {
		m_ownerWorld->removeConstraint(m_ballPointerJoints[i]);
		delete m_ballPointerJoints[i];
		m_ballPointerJoints[i] = 0;
	}
	// Remove all nodes and their corresponding bodies and shapes
	for (int i = 0; i < m_nodes.size(); ++i) {
		m_ownerWorld->removeRigidBody(m_nodes[i]->getRigidBody());
		delete m_nodes[i];
		m_nodes[i] = 0;
	}
	for (int i = 0; i < m_ballPointerNodes.size(); ++i) {
		m_ownerWorld->removeRigidBody(m_ballPointerNodes[i]->getRigidBody());
		delete m_ballPointerNodes[i];
		m_ballPointerNodes[i] = 0;
	}
}
