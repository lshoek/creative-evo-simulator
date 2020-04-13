#pragma once
#include "FitnessFunc.h"
#include "GenomeBase.h"
#include "SimulationManager.h"

class SimFitnessFunc : FitnessFunc
{
public:
	virtual void init(SimulationManager* _manager) { _sim = _manager; }
	virtual double evaluate(GenomeBase& genome) override = 0;

	virtual void setSimulationManager(SimulationManager* _manager) { _sim = _manager; }
	virtual SimulationManager* getSimulationManager() { return _sim; }

protected:
	SimulationManager* _sim;

	double _maxSimulationSeconds = 60.0;
};
