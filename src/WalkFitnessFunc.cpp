#pragma once
#include "WalkFitnessFunc.h"

double WalkFitnessFunc::evaluate(GenomeBase& genome)
{
    genome.buildPhenotype();
    return 1.0;
}
