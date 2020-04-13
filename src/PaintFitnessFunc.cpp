#pragma once
#include "PaintFitnessFunc.h"

double PaintFitnessFunc::evaluate(GenomeBase& genome)
{
    genome.buildPhenotype();
    
    // run an instance of a simulation and acquire the fitness result
    // a neural network phenotype (or cppn) should be passed on to the sim
    // something like this
    // _sim->runSimulationInstance(genome.getNN());

    return 1.0;
}
