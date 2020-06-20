#pragma once
#include "SimCreature.h"
#include "SimWorld.h"

class SimInstance 
{
public:
    SimInstance(int id, int generation, SimWorld* world, SimCreature* crtr, SimCanvasNode* canv, btScalar duration);
    ~SimInstance();

    void updateTimeStep(double timeStep);
    void update();
    void abort();

    bool isEffectorUpdateRequired();
    bool isAborted();
    bool isFinished();

    int getID();
    btScalar getElapsedTime();
    btScalar getDuration();

    SimWorld* getWorld();
    SimCreature* getCreature();
    SimCanvasNode* getCanvas();

private:
    SimWorld* _world;
    SimCreature* _creature;
    SimCanvasNode* _canvas;

    int _instanceId;
    int _generation;
    btScalar _elapsed, _duration;

    bool _bIsAwaitingOutputUpdate = false;
    bool _bIsAborted = false;
    bool _bIsFinished = false;
};
