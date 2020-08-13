#pragma once
#include "btBulletCollisionCommon.h"
#include "Simulator/SimNode.h"
#include "Simulator/SimCanvasNode.h"
#include "ofGraphics.h"
#include "ofMaterial.h"
#include "ofMesh.h"
#include "Genome/DirectedGraph.h"

class SimCreature
{
public:
	// Initialization from a graph genome
	SimCreature(btVector3 position, const std::shared_ptr<DirectedGraph>& graph, btDynamicsWorld* ownerWorld);

	~SimCreature();

	bool isAwaitingEffectorUpdate();
	void updateTimeStep(double timeStep);
	void update();

	void updateOutputs(const std::vector<float>& outputs);
	const std::vector<float>& getOutputs();

	void draw();
	void drawImmediate();

	bool feasibilityCheck();

	uint32_t getID();
	uint32_t getNumOutputs();
	uint32_t getNumJoints();
	const DirectedGraph& getBodyGenome();

	btTypedConstraint** getJoints();
	std::vector<float> getJointState();

	btRigidBody** getRigidBodies();
	SimNode** getSimNodes();

	enum SensorMode { Touch, Canvas };
	SensorMode _sensorMode;

	void setSensorMode(SensorMode mode);

	double getTouchSensor(int i);
	void setTouchSensor(void* bodyPointer);
	void clearTouchSensors();

	void addToWorld();
	void removeFromWorld();

	btVector3 getSpawnPosition() const;
	btVector3 getCenterOfMassPosition() const;
	btQuaternion getRootNodeRotation() const;

	void clearForces();

	void setShader(std::shared_ptr<ofShader> shader);
	void setMaterial(std::shared_ptr<ofMaterial> mtl);
	void setLight(std::shared_ptr<ofLight> light);
	void setTexture(std::shared_ptr<ofTexture> tex);
	void setAppearance(std::shared_ptr<ofShader> shader, std::shared_ptr<ofMaterial> mtl, std::shared_ptr<ofTexture> tex);

	float m_motorStrength;
	uint32_t m_targetFrequency;

private:
	void buildPhenome(DirectedGraph* graph);
	void dfs(
		GraphNode* graphNode, GraphConnection* incoming, SimNode* parentSimNode, DirectedGraph* graph, 
		btVector3 parentDims, btScalar cascadingScale, btScalar attachment, std::vector<int> recursionLimits, int& segmentIndex
	);

	bool bInitialized = false;

	btDynamicsWorld* m_ownerWorld;
	DirectedGraph* m_bodyGenome = NULL;
	uint32_t m_id = 0;

	// standard rigid bodies and shapes for creature
	std::vector<btRigidBody*> m_bodies;
	std::vector<btCollisionShape*> m_shapes;
	std::vector<SimNode*> m_nodes;
	std::vector<SimNode*> m_brushNodes;
	std::vector<btTypedConstraint*> m_joints;

	SimNode* m_rootNode;

	std::shared_ptr<ofShader> m_shader;
	std::shared_ptr<ofTexture> m_texture;
	std::shared_ptr<ofMaterial> m_material;
	std::shared_ptr<ofLight> m_light;

	uint32_t m_numBodies;
	uint32_t m_numJoints;
	uint32_t m_numBrushes;
	uint32_t m_numOutputs;

	uint64_t m_activationMillis = 0;

	// body part hash, touch sensor index
	btHashMap<btHashPtr, int> m_bodyTouchSensorIndexMap;
	std::vector<double> m_touchSensors;
	std::vector<double> m_canvasSensors;

	// output neuron activations
	std::vector<float> m_outputs;

	btVector3 m_spawnPosition;
	btScalar m_targetAccumulator;
	btScalar m_timeStep;

	bool m_bAwaitingEffectorUpdate = false;
	bool m_bHasBrush = false;

	ofColor m_bodyColor;

	const ofColor INK = ofColor::fromHex(0x333333);
	const btVector3 FORWARD = btVector3(1, 0, 0);
	const btVector3 UP = btVector3(0, 1, 0);
	const btVector3 RIGHT = btVector3(0, 0, 1);
	const btVector3 AXES[3] = { FORWARD, UP, RIGHT };
};
