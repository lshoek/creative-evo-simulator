#include "NEATManager.h"
#include "NEATUtils.h"
#include "toolbox.h"

void NEATManager::setup(bool threaded)
{
	// create a fitness function
	fitnessFuncPtr = &xorFitnessFunc;

	// load neat params from config file
	std::ifstream paramFile("data/config/params_hyperneat_xor_test.conf", std::ifstream::in);
	params.Load(paramFile);
	paramFile.close();

	params.PopulationSize = 200;
	params.NeuronRecursionLimit = 8;

	// create substrate
	substrate = NEATUtils::CreateSubstrate(3);
	substrate.PrintInfo();

	// create cppn genome using substrate config
	NEAT::Genome templateGenome(0,
		substrate.GetMinCPPNInputs(), 0,
		substrate.GetMinCPPNOutputs(), false,
		NEAT::ActivationFunction::TANH,
		NEAT::ActivationFunction::TANH,
		0, params, 0
	);
	offspringGenomeBasePtr = new GenomeBase(templateGenome);

	// init population and set genome ids that belong to the population
	population = new NEAT::Population(templateGenome, params, true, 1.0, 0);

	maxNumGenerations = 200;
	fitnessResults.reserve(maxNumGenerations);

	targetFitness = 15.99;
	bThreaded = threaded;
}

void NEATManager::startEvolution()
{
	totalTimeMs = ofGetElapsedTimeMillis();

	if (bThreaded) {
		startThread();
	}
	else {
		evolutionLoop();
	}
}

void NEATManager::threadedFunction()
{
	evolutionLoop();
}

void NEATManager::evolutionLoop()
{
	bool bTargetReached = false;

	while ((population->m_Generation < maxNumGenerations || maxNumGenerations == -1) &&
		(bestFitness < targetFitness || targetFitness == -1))
	{
		bool bNewBest = false;

		bNewBest = evaluatePopulation();
		bNewBest |= tick();

		if (bNewBest) {
			GenomeBase g(population->GetBestGenome());
			g.buildHyperNEATPhenotype(substrate);
		}

		fitnessResults.push_back(bestFitness);

		// check if converged
		if (bestFitness >= targetFitness && targetFitness != -1) {
			bTargetReached = true;
			break;
		}
		// no solution found -> advance to next generation
		population->Epoch();
	}

	totalTimeMs = ofGetElapsedTimeMillis() - totalTimeMs;
	ofLog() << (bTargetReached ? "Target Reached" : "Evolution Stopped");
	ofLog() << "time: " << totalTimeMs / float(1000) << "s";
}

bool NEATManager::tick()
{
	bool bNewBest = false;

	// replace worst genome with new and return it
	offspringGenome = *population->Tick(deadGenome);

	// get access to genome's network, substrate and params
	offspringGenomeBasePtr->setGenome(offspringGenome);

	// evaluate the newly created offspring in Tick()
	double f = fitnessFuncPtr->evaluate(*offspringGenomeBasePtr);
	offspringGenomeBasePtr->getGenome().SetFitness(f);
	offspringGenomeBasePtr->getGenome().SetEvaluated();

	bNewBest = f > bestFitness;
	bestFitness = bNewBest ? f : bestFitness;

	return bNewBest;
}

bool NEATManager::evaluatePopulation()
{
	double f_best = -DBL_MAX;
	bool bNewBest = false;

	for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
		for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {

			GenomeBase g(population->m_Species[i].m_Individuals[j]);
			double f = fitnessFuncPtr->evaluate(g);

			population->m_Species[i].m_Individuals[j].SetFitness(f);
			population->m_Species[i].m_Individuals[j].SetEvaluated();

			f_best = (f > f_best) ? f : f_best;
		}
		population->m_Species[i].CalculateAverageFitness();
	}
	bNewBest = f_best > bestFitness;
	bestFitness = bNewBest ? f_best : bestFitness;

	ofLog() << population->m_Generation << " : " << f_best;
	return bNewBest;
}

NEAT::Population* NEATManager::getPopulation()
{
	return population;
}

const NEAT::Parameters& NEATManager::getParams()
{
	return params;
}

double NEATManager::getBestFitness()
{
	return bestFitness;
}

double NEATManager::getTargetFitness()
{
	return targetFitness;
}

const std::vector<double>& NEATManager::getFitnessResults()
{
	return fitnessResults;
}

void NEATManager::exit()
{
	if (bThreaded) {
		waitForThread();
	}
	delete population;
	delete offspringGenomeBasePtr;
}
