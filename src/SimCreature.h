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
	SimCreature(btVector3 position, std::shared_ptr<DirectedGraph> graph, btDynamicsWorld* ownerWorld);
	// Spawns a walker with a specified number of legs
	SimCreature(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld, bool bInit);
	~SimCreature();

	void update(double timeStep);
	void draw();
	void drawImmediate();

	bool feasibilityCheck();

	btTypedConstraint** getJoints();
	btRigidBody** getRigidBodies();
	SimNode** getSimNodes();

	// Makes a deep copy of the genome and stores/uses it until the simulation instance is finished.
	void setControlPolicyGenome(const GenomeBase& genome);

	enum SensorMode { Touch, Canvas };
	SensorMode _sensorMode;

	void setSensorMode(SensorMode mode);

	double getTouchSensor(int i);
	void setTouchSensor(void* bodyPointer);
	void clearTouchSensors();

	void setCanvasSensors(const unsigned char* canvasSensors, double t);

	void addToWorld();
	void removeFromWorld();

	btVector3 getSpawnPosition() const;
	btVector3 getCenterOfMassPosition() const;

	void clearForces();

	btScalar getEvaluationTime() const;
	uint64_t getActivationMillis() const;

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
	void initWalker(btVector3 position, uint32_t numLegs, btDynamicsWorld* ownerWorld);
	void buildPhenome(DirectedGraph* graph);
	void dfs(
		GraphNode* graphNode, GraphConnection* incoming, SimNode* parentSimNode, DirectedGraph* graph, 
		btVector3 parentDims, btScalar cascadingScale, btScalar attachment, std::vector<int> recursionLimits, int& segmentIndex
	);

	bool bInitialized = false;
	bool bHasBallPointers = false;

	btDynamicsWorld* m_ownerWorld;

	GenomeBase* m_controlPolicyGenome = NULL;
	DirectedGraph* m_morphologyGenome = NULL;

	btVector3 m_spawnPosition;

	// standard rigid bodies and shapes for creature
	std::vector<btRigidBody*> m_bodies;
	std::vector<SimNode*> m_nodes;
	std::vector<SimNode*> m_brushNodes;
	std::vector<btTypedConstraint*> m_joints;

	std::vector<btCollisionShape*> m_shapes;
	std::vector<btTransform> m_bodyRelativeTransforms;

	std::shared_ptr<ofShader> m_shader;
	std::shared_ptr<ofTexture> m_texture;
	std::shared_ptr<ofMaterial> m_material;
	std::shared_ptr<ofLight> m_light;

	std::shared_ptr<ofMesh> m_segmentMesh;
	std::shared_ptr<ofMesh> m_legMesh;
	std::shared_ptr<ofMesh> m_foreLegMesh;
	std::shared_ptr<ofMesh> m_bodyMesh;
	std::shared_ptr<ofMesh> m_ballPointMesh;

	uint32_t m_numBodies;
	uint32_t m_numJoints;
	uint32_t m_numLegs;
	uint32_t m_numBrushes;

	uint64_t m_activationMillis = 0;

	// body part hash, touch sensor index
	btHashMap<btHashPtr, int> m_bodyTouchSensorIndexMap;
	std::vector<double> m_touchSensors;
	std::vector<double> m_canvasSensors;

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

	const ofColor INK = ofColor::fromHex(0x333333);

	const btVector3 FORWARD = btVector3(1, 0, 0);
	const btVector3 UP = btVector3(0, 1, 0);
	const btVector3 RIGHT = btVector3(0, 0, 1);
	const btVector3 AXES[3] = { FORWARD, UP, RIGHT };

	bool bIsDebugCreature = false;
};
