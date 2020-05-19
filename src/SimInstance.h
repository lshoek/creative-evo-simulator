#pragma once
#include "SimCreature.h"

class SimInstance 
{
public:
    SimInstance(int id, SimCreature* crtr, SimCanvasNode* canv, btScalar startTime, btScalar duration);
    ~SimInstance();

    void abort();
    bool IsAborted();

    int getID();
    btScalar getStartTime();
    btScalar getDuration();

    SimCreature* getCreature();
    SimCanvasNode* getCanvas();

private:
    int _instanceId;
    btScalar _startTime, _duration;

    btDynamicsWorld* _world;
    SimCreature* _creature;
    SimCanvasNode* _canvas;

    bool _bIsAborted = false;
};
