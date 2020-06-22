#pragma once
#include "FitnessFunc.h"
#include "GenomeBase.h"
#include "SimulationManager.h"

typedef int isim_ticket;

class SimFitnessFunc
{
public:
	virtual void init(SimulationManager* _manager) { _sim = _manager; }

	virtual isim_ticket queueEval(GenomeBase& genome, bool bMultiEval) = 0;
	virtual double awaitEval(isim_ticket id) = 0;

	virtual void setSimulationManager(SimulationManager* _manager) { _sim = _manager; }
	virtual SimulationManager* getSimulationManager() { return _sim; }

protected:
	SimulationManager* _sim;

	double _maxSimulationSeconds = 60.0;
};
