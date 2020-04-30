#pragma once
#include "SimCreature.h"

class SimInstance 
{
public:
    SimInstance(int id, SimCreature* crtr, SimCanvasNode* canv, float startTime, float duration);
    ~SimInstance();

    void abort();
    bool IsAborted();

    int getID();
    int getStartTime();
    int getDuration();

    SimCreature* getCreature();
    SimCanvasNode* getCanvas();

private:
    int _instanceId;
    float _startTime, _duration;

    btDynamicsWorld* _world;
    SimCreature* _creature;
    SimCanvasNode* _canvas;

    bool _bIsAborted = false;
};
