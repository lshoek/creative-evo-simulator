#pragma once
#include "bullet/btBulletCollisionCommon.h"
#include "SimNode.h"
#include "SimCanvasNode.h"
#include "ofGraphics.h"
#include "ofMaterial.h"
#include "ofMesh.h"
#include "GenomeBase.h"
#include "DirectedGraph.h"

#define DRAW_INTERPENETRATIONS false

class SimCreature
{
public:
	// Initialization from a graph genome
	SimCreature(btVector3 position, const DirectedGraph& graph, btDynamicsWorld* ownerWorld);
	// Spawns a walker with a specified number of legs
	SimCreature(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld, bool bInit);
	~SimCreature();

	void initWalker(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld);
	void initSnake(btVector3 position, unsigned int numNodes, float boxExtents, float distPct, bool bRandomSize);

	void update(double timeStep);
	void draw();

	void randomizeSensoryMotorWeights();
	btTypedConstraint** getJoints();
	btRigidBody** getRigidBodies();
	SimNode** getSimNodes();

	// Makes a deep copy of the genome and stores/uses it until the simulation instance is finished.
	void setControlPolicyGenome(const GenomeBase& genome);

	void setTouchSensor(void* bodyPointer);
	void clearTouchSensors();
	double getTouchSensor(int i);
	btScalar* getSensoryMotorWeights();

	void addToWorld();
	void removeFromWorld();

	btVector3 getSpawnPosition() const;
	btVector3 getCenterOfMassPosition() const;

	void resetAt(const btVector3& position);

	btScalar getEvaluationTime() const;
	void setEvaluationTime(btScalar evaluationTime);
	bool isInEvaluation() const;
	void setInEvaluation(bool inEvaluation);
	bool isReaped() const;
	void setReaped(bool reaped);
	int getIndex() const;

	void setShader(std::shared_ptr<ofShader> shader);
	void setMaterial(std::shared_ptr<ofMaterial> mtl);
	void setLight(std::shared_ptr<ofLight> light);
	void setTexture(std::shared_ptr<ofTexture> tex);
	void setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<ofMaterial> mtl, std::shared_ptr<ofTexture> tex);

	float m_motorStrength;
	float m_targetFrequency;

private:
	void buildPhenome(DirectedGraph* graph);
	void dfs(
		GraphNode* graphNode, GraphConnection* incoming, SimNode* parentSimNode, DirectedGraph* graph, 
		btVector3 parentDims, btScalar cascadingScale, std::vector<int> recursionLimits, int& segmentIndex
	);

	bool bInitialized = false;
	bool bHasBallPointers = false;

	btDynamicsWorld* m_ownerWorld;

	GenomeBase* m_controlPolicyGenome;
	DirectedGraph* m_morphologyGenome;

	btVector3 m_spawnPosition;

	// standard rigid bodies and shapes for creature
	std::vector<btCollisionShape*> m_shapes;
	std::vector<btRigidBody*> m_bodies;
	std::vector<SimNode*> m_nodes;
	std::vector<btTransform> m_bodyRelativeTransforms;
	std::vector<btTypedConstraint*> m_joints;

	// ball pointers are collision shapes that detect contact with a canvas
	std::vector<btCollisionShape*> m_ballPointerShapes;
	std::vector<btRigidBody*> m_ballPointerBodies;
	std::vector<SimNode*> m_ballPointerNodes;
	std::vector<btFixedConstraint*> m_ballPointerJoints;

	std::shared_ptr<ofShader> m_shader;
	std::shared_ptr<ofTexture> m_texture;
	std::shared_ptr<ofMaterial> m_material;
	std::shared_ptr<ofLight> m_light;

	std::shared_ptr<ofMesh> m_segmentMesh;
	std::shared_ptr<ofMesh> m_legMesh;
	std::shared_ptr<ofMesh> m_foreLegMesh;
	std::shared_ptr<ofMesh> m_bodyMesh;
	std::shared_ptr<ofMesh> m_ballPointMesh;

	uint32_t m_numBodyParts;
	uint32_t m_numJoints;
	uint32_t m_numLegs;
	uint32_t m_numBallPointers;

	// body part hash, touch sensor index
	btHashMap<btHashPtr, int> m_bodyTouchSensorIndexMap;
	std::vector<double> m_touchSensors;

	std::vector<btScalar> m_sensoryMotorWeights;

	int m_index = 0;

	bool m_inEvaluation;	
	bool m_reaped;	

	btScalar m_targetAccumulator;
	btScalar m_evaluationTime;

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
