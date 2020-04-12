#pragma once
#include "bullet/btBulletCollisionCommon.h"
#include "SimNode.h"
#include "SimCanvasNode.h"
#include "ofGraphics.h"
#include "ofMaterial.h"
#include "ofMesh.h"

#define DRAW_INTERPENETRATIONS false

class SimCreature
{
public:
	SimCreature(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bInit);
	~SimCreature();

	void init(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld, const btVector3& positionOffset);
	void initSnake(btVector3 position, unsigned int numNodes, float boxExtents, float distPct, bool bRandomSize);

	void update(double timeStep);
	void draw();

	void randomizeSensoryMotorWeights();
	btTypedConstraint** getJoints();
	btRigidBody** getRigidBodies();
	SimNode** getSimNodes();

	void setTouchSensor(void* bodyPointer);
	void clearTouchSensors();
	bool getTouchSensor(int i);
	btScalar* getSensoryMotorWeights();

	void addToWorld();
	void removeFromWorld();

	btVector3 getPosition() const;
	btScalar getDistanceFitness() const;
	btScalar getFitness() const;

	void resetAt(const btVector3& position);

	btScalar getEvaluationTime() const;
	void setEvaluationTime(btScalar evaluationTime);
	bool isInEvaluation() const;
	void setInEvaluation(bool inEvaluation);
	bool isReaped() const;
	void setReaped(bool reaped);
	int getIndex() const;

	void setShader(ofShader* shader);
	void setMaterial(ofMaterial* mtl);
	void setTexture(ofTexture* tex);
	void setAppearance(ofShader* shader, ofMaterial* mtl, ofTexture* tex);

	float m_motorStrength;
	float m_targetFrequency;

private:
	bool bInitialized = false;
	bool bHasBallPointers = false;

	btDynamicsWorld* m_ownerWorld;

	// standard rigid bodies and shapes for creature
	std::vector<btCollisionShape*> m_shapes;
	std::vector<btRigidBody*> m_bodies;
	std::vector<SimNode*> m_nodes;
	std::vector<btTransform> m_bodyRelativeTransforms;
	std::vector<btTypedConstraint*> m_joints;

	// ball pointers are collision shapes that detect contact with a canvas
	std::vector <btCollisionShape*> m_ballPointerShapes;
	std::vector <btRigidBody*> m_ballPointerBodies;
	std::vector<SimNode*> m_ballPointerNodes;
	std::vector<btFixedConstraint*> m_ballPointerJoints;

	ofShader* m_shader;
	ofTexture* m_texture;
	ofMaterial* m_material;

	ofMesh* m_legMesh;
	ofMesh* m_foreLegMesh;
	ofMesh* m_bodyMesh;
	ofMesh* m_ballPointMesh;

	uint32_t m_numLegs;
	uint32_t m_numBodyParts;
	uint32_t m_numJoints;
	uint32_t m_numBallPointers;

	// body part hash, touch sensor index
	btHashMap<btHashPtr, int> m_bodyTouchSensorIndexMap;
	std::vector<bool> m_touchSensors;

	std::vector<btScalar> m_sensoryMotorWeights;

	int m_index = 0;
	double m_time = 0.0;

	bool m_inEvaluation;	
	bool m_reaped;	

	btScalar m_targetAccumulator;
	btScalar m_evaluationTime;
	btVector3 m_startPosition;

	btScalar gRootBodyRadius = 0.25f;
	btScalar gRootBodyHeight = 0.1f;
	btScalar gLegRadius = 0.1f;
	btScalar gLegLength = 0.45f;
	btScalar gForeLegLength = 0.75f;
	btScalar gForeLegRadius = 0.08f;

	const glm::vec3 right = glm::vec3(1, 0, 0);
	const glm::vec3 up = glm::vec3(0, 1, 0);
	const glm::vec3 fwd = glm::vec3(0, 0, 1);

	bool bIsDebugCreature = false;
};
