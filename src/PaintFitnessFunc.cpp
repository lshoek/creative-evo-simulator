#pragma once
#include "PaintFitnessFunc.h"
#include "SimSharedData.h"

double PaintFitnessFunc::evaluate(GenomeBase& genome)
{
    genome.buildPhenotype();
    
    // run an instance of a simulation and acquire the fitness result
    // a neural network phenotype (or cppn) should be passed on to the sim
    int id = _sim->queueSimulationInstance(genome, 10.0f);

    // automatically unregisters itself when out of scope
    SimResult simres;
    ofEventListener listener = _sim->onSimulationInstanceFinished.newListener([&simres](SimResult res) {
        simres = res;
    });
    
    // wait for simulation to return sim ticket
    while (simres.instanceId != id) {
        ofSleepMillis(1000);
    }

    return simres.fitness;
}
