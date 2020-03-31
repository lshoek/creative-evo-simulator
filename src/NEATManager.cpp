#include "NEATManager.h"
#include "toolbox.h"
#include <boost/assign/std.hpp>

using namespace boost::assign;

void NEATManager::setup(bool threaded)
{
	// create a fitness function
	fitnessFuncPtr = &xorFitnessFunc;

	// set parameters to NEAT defaults
	params = NEAT::Parameters();
	params.NeuronRecursionLimit = 8;

	std::ifstream paramFile("data/config/params_hyperneat_xor_test.conf", std::ifstream::in);
	params.Load(paramFile);
	paramFile.close();

	// substrate
	std::vector<std::vector<double>> substrateInputs(3);
	std::vector<std::vector<double>> substrateHidden;
	std::vector<std::vector<double>> substrateOutputs(1);

	substrateInputs[0] += -1.0, -1.0, 0.0;
	substrateInputs[1] += 1.0, -1.0, 0.0;
	substrateInputs[2] += 0.0, -1.0, 0.0;
	substrateOutputs[0] += 0.0, 1.0, 0.0;

	NEAT::Substrate substrate = NEAT::Substrate(
		substrateInputs, 
		substrateHidden, 
		substrateOutputs
	);

	substrate.m_allow_input_hidden_links = true;
	substrate.m_allow_input_output_links = false;
	substrate.m_allow_hidden_output_links = true;
	substrate.m_allow_hidden_hidden_links = false;
	substrate.m_hidden_nodes_activation = NEAT::ActivationFunction::SIGNED_SIGMOID;
	substrate.m_output_nodes_activation = NEAT::ActivationFunction::UNSIGNED_SIGMOID;
	substrate.m_max_weight_and_bias = 8.0;

	// initial net
	int numInputs = 3;
	int numHidden = 0;
	int numOutputs = 2;

	// init genotype
	NEAT::Genome templateGenome(
		0, numInputs, numHidden, numOutputs,
		NEAT::UNSIGNED_SIGMOID,
		NEAT::UNSIGNED_SIGMOID,
		params
	);

	// init population and set genome ids
	population = new NEAT::Population(templateGenome, params, true, 1.0, 0);
	for (unsigned int i = 0; i < params.PopulationSize; ++i) {
		genomeIds.push_back(i);
	}

	maxNumGenerations = 500;
	fitnessResults.reserve(maxNumGenerations);

	targetFitness = 15.999;
	bFirstEval = true;
	bThreaded = threaded;
}

void NEATManager::startEvolution()
{
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
	while ((population->m_Generation < maxNumGenerations || maxNumGenerations == -1) &&
		(bestFitness < targetFitness || targetFitness == -1))
	{
		bool bEvalInTick = bFirstEval; // prevent double eval in first frame
		if (bEvalInTick) {
			rtTick(true, lastKilledId, lastOffspringId);
		}
		else {
			evaluatePopulation();
		}
		fitnessResults.push_back(bestFitness);
		population->Epoch();
	}
	ofLog() << ((bestFitness >= targetFitness && targetFitness != -1) ? "Target Reached" : "Evolution Stopped");
}

void NEATManager::rtTick(bool eval, int& deceased_id, int& new_baby_id)
{
	if (bFirstEval) {
		bFirstEval = false;
		if (eval) {
			evaluatePopulation();
		}
		// sustain best offspring
		offspringGenome = *population->Tick(deadGenome);
		offspringGenomeBasePtr = new GenomeBase(offspringGenome);
	}
	else {
		// sustain best offspring
		offspringGenome = *population->Tick(deadGenome);
		offspringGenomeBasePtr->setGenome(offspringGenome);
	}
	if (eval) {
		double f = fitnessFuncPtr->evaluate(*offspringGenomeBasePtr);
		offspringGenomeBasePtr->getGenome().SetFitness(f);
		bestFitness = (f > bestFitness) ? f : bestFitness;
	}
	lastKilledId = deadGenome.GetID();
	lastOffspringId = offspringGenome.GetID();
	replaceGenomeIds(lastKilledId, lastOffspringId);
}

void NEATManager::evaluatePopulation()
{
	double f_best = -DBL_MAX;

	for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
		for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {
			GenomeBase g(population->m_Species[i].m_Individuals[j]);
			double f = fitnessFuncPtr->evaluate(g);
			population->m_Species[i].m_Individuals[j].SetFitness(f);
			population->m_Species[i].m_Individuals[j].m_Evaluated = true;
			f_best = (f > f_best) ? f : f_best;
		}
		population->m_Species[i].CalculateAverageFitness();
	}
	bestFitness = (f_best > bestFitness) ? f_best : bestFitness;
	ofLog() << population->m_Generation << " : " << f_best;
}

void NEATManager::replaceGenomeIds(unsigned int oldId, unsigned int newId)
{
	int pos = find(genomeIds.begin(), genomeIds.end(), oldId) - genomeIds.begin();
	if (pos > params.PopulationSize) {
		ofLog(OF_LOG_ERROR) << " Genome ID: " << oldId << " not found!!";
	}
	else {
		genomeIds[pos] = newId;
	}
	std::sort(genomeIds.begin(), genomeIds.end());
}

void NEATManager::draw()
{
	ofPolyline poly;
	poly.setClosed(false);

	glm::vec3 pos;
	double f = 0;
	int n = fitnessResults.size();

	// draw line
	ofSetHexColor(0xffffff);
	for (int i = 0; i < n; i++) {
		f = fitnessResults[i];

		pos = glm::vec3(i * ((double)ofGetWidth() / (double)n), tb::getHeight(f), 0);
		poly.addVertex(pos);
	}
	poly.draw();

	// mark improvements
	ofFill();
	for (int i=0; i < fitnessResults.size(); i++) {
		f = fitnessResults[i];
		pos = glm::vec3(i * ((double)ofGetWidth() / (double)n), tb::getHeight(f), 0);

		if (i > 0) {
			if (fitnessResults[i - 1] < f) ofSetHexColor(0xff0088);
			else ofSetHexColor(0xffffff);

			ofCircle(pos, 4.0f);
		}
	}
	ofNoFill();

	ofSetHexColor(0xffffff);
	pos += glm::vec3(-52.0f, 24.0f, 0);
	if (n!=0) ofDrawBitmapString(fitnessResults[n-1], pos);
}

NEAT::Population* NEATManager::getPopulation()
{
	return population;
}

const NEAT::Parameters& NEATManager::getParams()
{
	return params;
}

void NEATManager::exit()
{
	if (bThreaded) {
		waitForThread();
	}
	delete population;
	delete offspringGenomeBasePtr;
}
