#pragma once
#include "MultiNEAT.h"
#include "ofMain.h"

#include "GenomeBase.h"
#include "PaintFitnessFunc.h"
#include "SimulationManager.h"
#include "SimSharedData.h"

class NEATManager : public ofThread
{
public:
	void setup(SimulationManager* sim);
    void draw();
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

    int getNumGeneration();
    float getPctGenEvaluated();

    ofEvent<void> onNewBestFound;

    void setMaxParallelEvals(int max);

private:
    virtual void threadedFunction() override;

    PaintFitnessFunc paintFitnessFunc;
    SimFitnessFunc* fitnessFuncPtr;

    NEAT::Population* population;
    NEAT::Parameters params;
    NEAT::Substrate substrate;

    int maxNumGenerations;
    int maxParallelEvals = 1;

    double targetFitness;
    double bestFitness;

    float pctGenEvaluated = 0.0f;

    bool bThreaded = true;
    bool bHasFitnessFunc;

    NEAT::Genome deadGenome;
    NEAT::Genome offspringGenome;

    GenomeBase* offspringGenomeBasePtr;
    GenomeBase* bestGenomeBasePtr;

    int lastDeadId;
    int lastOffspringId;

    std::vector<double> fitnessResults;

    uint64_t totalTimeMs = 0;
};
