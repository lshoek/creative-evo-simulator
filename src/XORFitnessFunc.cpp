#pragma once
#include "XORFitnessFunc.h"

double activate(GenomeBase& genome, double input_a, double input_b)
{
    std::vector<double> inputs;
    inputs.resize(3);

    inputs[0] = input_a;
    inputs[1] = input_b;

    // bias
    inputs[2] = 1.0; 

    std::vector<double> output = genome.activate(inputs);
    return output[0];
}

double xorTest(GenomeBase& genome)
{
    genome.buildPhenotype();

    double errSum = 0;
    errSum += abs(activate(genome, 0.0, 0.0) - 0.0);
    errSum += abs(activate(genome, 0.0, 1.0) - 1.0);
    errSum += abs(activate(genome, 1.0, 0.0) - 1.0);
    errSum += abs(activate(genome, 1.0, 1.0) - 0.0);

    double fitness = (4.0 - errSum) * (4.0 - errSum);
    return fitness;
}

double XORFitnessFunc::evaluate(GenomeBase& genome)
{
    return xorTest(genome);
}
