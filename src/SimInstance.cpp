#include "SimInstance.h"

SimInstance::SimInstance(int id, SimCreature* crtr, SimCanvasNode* canv, float startTime, float duration) :
	_instanceId(id), _world(canv->_ownerWorld), _creature(crtr), _canvas(canv), _startTime(startTime), _duration(duration) {}

int SimInstance::getID()
{
	return _instanceId;
}

int SimInstance::getStartTime()
{
	return _startTime;
}

int SimInstance::getDuration()
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
