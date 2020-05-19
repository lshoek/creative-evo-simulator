#include "SimInstance.h"

SimInstance::SimInstance(int id, SimCreature* crtr, SimCanvasNode* canv, btScalar startTime, btScalar duration) :
	_instanceId(id), _world(canv->_ownerWorld), _creature(crtr), _canvas(canv), _startTime(startTime), _duration(duration) {}

void SimInstance::abort()
{
	_bIsAborted = true;
}

bool SimInstance::IsAborted() 
{
	return _bIsAborted;
}

int SimInstance::getID()
{
	return _instanceId;
}

btScalar SimInstance::getStartTime()
{
	return _startTime;
}

btScalar SimInstance::getDuration()
{
	return _duration;
}

SimCreature* SimInstance::getCreature()
{
	return _creature;
}

SimCanvasNode* SimInstance::getCanvas()
{
	return _canvas;
}

SimInstance::~SimInstance() 
{
	delete _creature;
	delete _canvas;
}
