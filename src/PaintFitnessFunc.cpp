#pragma once
#include "PaintFitnessFunc.h"
#include "SimSharedData.h"

isim_ticket PaintFitnessFunc::queueEval(GenomeBase& genome)
{
    genome.buildPhenotype();
    
    // run an instance of a simulation and acquire the fitness result
    // a neural network phenotype (or cppn) should be passed on to the sim
    int id = _sim->queueSimulationInstance(genome, 10.0f);

    ofLog() << "started : " << id;
    return id;
}

double PaintFitnessFunc::awaitEval(isim_ticket id)
{
    // automatically unregisters itself when out of scope
    SimResult simres;
    ofEventListener listener = _sim->onSimulationInstanceFinished.newListener([&simres, id](SimResult r) {
        if (r.instanceId == id) {
            simres = r;
        }
    });

    // wait for simulation to return sim ticket
    while (simres.instanceId < 0) {
        ofSleepMillis(100);
    }
    ofLog() << "done : " << id;
    return simres.fitness;
}
