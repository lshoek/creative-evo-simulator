#pragma once
#include "MultiNEAT.h"
#include "ofMain.h"

#include "GenomeBase.h"
#include "XORFitnessFunc.h"

class NEATManager : public ofThread
{
public:
	void setup(bool threaded);
    void draw();
    void exit();

    void startEvolution();
    void evolutionLoop();
    void rtTick(bool, int&, int&);

    void evaluatePopulation();
    void replaceGenomeIds(unsigned int oldId, unsigned int newId);

    NEAT::Population* getPopulation();
    const NEAT::Parameters& getParams();

private:
    virtual void threadedFunction() override;

    XORFitnessFunc xorFitnessFunc;
    FitnessFunc* fitnessFuncPtr;

    NEAT::Population* population;
    NEAT::Parameters params;

    int maxNumGenerations;
    double targetFitness;
    double bestFitness;

    bool bHasFitnessFunc;
    bool bFirstEval;
    bool bThreaded;

    std::vector<unsigned int> genomeIds;

    NEAT::Genome deadGenome;
    NEAT::Genome offspringGenome;
    GenomeBase* offspringGenomeBasePtr;

    int lastKilledId;
    int lastOffspringId;

    std::vector<double> fitnessResults;
};
