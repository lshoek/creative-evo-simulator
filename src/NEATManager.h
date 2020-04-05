#pragma once
#include "MultiNEAT.h"
#include "ofMain.h"

#include "GenomeBase.h"
#include "XORFitnessFunc.h"

class NEATManager : public ofThread
{
public:
	void setup(bool threaded);
    void exit();

    void startEvolution();
    void evolutionLoop();

    bool tick();
    bool evaluatePopulation();

    NEAT::Population* getPopulation();
    const NEAT::Parameters& getParams();

    double getBestFitness();
    double getTargetFitness();
    const std::vector<double>& getFitnessResults();

private:
    virtual void threadedFunction() override;

    XORFitnessFunc xorFitnessFunc;
    FitnessFunc* fitnessFuncPtr;

    NEAT::Population* population;
    NEAT::Parameters params;
    NEAT::Substrate substrate;

    int maxNumGenerations;
    double targetFitness;
    double bestFitness;

    bool bHasFitnessFunc;
    bool bThreaded;

    NEAT::Genome deadGenome;
    NEAT::Genome offspringGenome;
    GenomeBase* offspringGenomeBasePtr;

    int lastDeadId;
    int lastOffspringId;

    std::vector<double> fitnessResults;

    uint64_t totalTimeMs = 0;
};
