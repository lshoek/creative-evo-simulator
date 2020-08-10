#include "SimInstance.h"
#include "SimDefines.h"

SimInstance::SimInstance(int id, int generation, SimWorld* world, SimCreature* crtr, SimCanvasNode* canv, btScalar duration) :
	_instanceId(id), _generation(generation), _world(world), _creature(crtr), _canvas(canv), _duration(duration), _elapsed(0) 
{
	_world->setSimInstance(this);
}

void SimInstance::updateTimeStep(double timeStep)
{
	if (!_bIsFinished) {
		// prevent updates if an effector vector is required
		if (!_creature->isAwaitingEffectorUpdate()) {
			_creature->updateTimeStep(timeStep);
			_world->getBtWorld()->stepSimulation(timeStep, 1, FixedTimeStep);
			_elapsed += timeStep;
		}
		if (_elapsed >= _duration) {
			_bIsFinished = true;
		}
	}
}

void SimInstance::update() 
{
	_creature->update();
	_canvas->update();
}

void SimInstance::terminate()
{
	_bIsTerminated = true;
}

bool SimInstance::isTerminated() 
{
	return _bIsTerminated;
}

bool SimInstance::isFinished()
{
	return _bIsFinished;
}

bool SimInstance::isEffectorUpdateRequired()
{
	return _creature->isAwaitingEffectorUpdate();
}

int SimInstance::getID()
{
	return _instanceId;
}

int SimInstance::getGeneration()
{
	return _generation;
}

btScalar SimInstance::getElapsedTime()
{
	return _elapsed;
}

btScalar SimInstance::getDuration()
{
	return _duration;
}

SimWorld* SimInstance::getWorld()
{
	return _world;
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
	delete _world;
}
