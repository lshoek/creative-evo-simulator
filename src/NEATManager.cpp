#include "NEATManager.h"
#include "NEATUtils.h"
#include "toolbox.h"
#include <boost/thread.hpp>

void NEATManager::setup(SimulationManager* sim, bool threaded)
{
	// create a fitness function
	paintFitnessFunc.init(sim);
	fitnessFuncPtr = &paintFitnessFunc;

	// load neat params from config file
	std::ifstream paramFile("data/config/params_hyperneat_xor_test.conf", std::ifstream::in);
	params.Load(paramFile);
	paramFile.close();

	params.PopulationSize = 4;
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
	bestGenomeBasePtr = new GenomeBase(templateGenome);

	// init population and set genome ids that belong to the population
	population = new NEAT::Population(templateGenome, params, true, 1.0, 0);

	maxNumGenerations = 500;
	fitnessResults.reserve(maxNumGenerations);

	targetFitness = 1.0;
	bThreaded = threaded;
}

void NEATManager::draw()
{
	glm::ivec2 rect = glm::ivec2(512, 512);

	ofPushMatrix();
	ofScale(rect.x/2, rect.y/2, 0);
	ofTranslate(rect.x/2, rect.y/2, 0);

	const std::vector<NEAT::Neuron>& neurons = bestGenomeBasePtr->getNN().m_neurons;
	const std::vector<NEAT::Connection>& conns = bestGenomeBasePtr->getNN().m_connections;

	std::vector<glm::vec3> ncoords;
	ncoords.resize(neurons.size());

	for (int i = 0; i < neurons.size(); i++) {
		if (neurons[i].m_substrate_coords.size() > 0) {
			ncoords[i] = glm::vec3(neurons[i].m_substrate_coords[0], neurons[i].m_substrate_coords[1], neurons[i].m_substrate_coords[2]);
		}
	}
	for (const NEAT::Connection& c : conns) {
		if (c.m_source_neuron_idx != c.m_target_neuron_idx) {
			ofDrawLine(ncoords[c.m_source_neuron_idx], ncoords[c.m_target_neuron_idx]);
		}
		else { /* handle recurrent connection */ }
	}
	ofFill();
	ofSetHexColor(0xff0088);

	for (glm::vec3& v : ncoords) {
		ofDrawCircle(v, 0.05f);
	}

	//ofColor typecol;
	//for (int i=0; i < neurons.size(); i++) {
	//	if (neurons[i].m_type == NEAT::INPUT)		typecol = ofColor::fromHex(0xff0088);
	//	else if (neurons[i].m_type == NEAT::HIDDEN) typecol = ofColor::fromHex(0x88ff00);
	//	else if (neurons[i].m_type == NEAT::OUTPUT) typecol = ofColor::fromHex(0x0088ff);

	//	ofSetColor(typecol);
	//	ofDrawCircle(ncoords[i], 0.05f);
	//}

	ofNoFill();
	ofSetHexColor(0xffffff);

	ofPopMatrix();
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
			bestGenomeBasePtr->setGenome(population->GetBestGenome());
			
			//bestGenomeBasePtr->buildHyperNEATPhenotype(substrate);
			bestGenomeBasePtr->buildPhenotype();
			onNewBestFound.notify();
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
	// start evaluation threads
	//boost::thread_group threadGroup;

	for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
		for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {

			// activate simulation here
			GenomeBase g(population->m_Species[i].m_Individuals[j]);

			//threadGroup.create_thread(boost::bind(&FitnessFunc::evaluate, &g));
			double f = fitnessFuncPtr->evaluate(g);

			population->m_Species[i].m_Individuals[j].SetFitness(f);
			population->m_Species[i].m_Individuals[j].SetEvaluated();
			//f_best = (f > f_best) ? f : f_best;
		}
		//population->m_Species[i].CalculateAverageFitness();
	}
	// join evaluation threads

	double f_best = -DBL_MAX;
	bool bNewBest = false;

	for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
		for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {
			double f = population->m_Species[i].m_Individuals[j].GetFitness();
			f_best = (f > f_best) ? f : f_best;
		}
		population->m_Species[i].CalculateAverageFitness();
	}
	bNewBest = f_best > bestFitness;
	bestFitness = bNewBest ? f_best : bestFitness;

	ofLog() << "gen_" << population->m_Generation << ": " << f_best;
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
	delete bestGenomeBasePtr;
	delete offspringGenomeBasePtr;
}
