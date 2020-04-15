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
	std::ifstream paramFile("data/config/params_paint.conf", std::ifstream::in);
	params.Load(paramFile);
	paramFile.close();

	params.NeuronRecursionLimit = 8;

	// create substrate
	substrate = NEATUtils::CreateSubstrate(3);
	//substrate.PrintInfo();

	// create cppn genome using substrate config
	//NEAT::Genome templateGenome(0,
	//	substrate.GetMinCPPNInputs(), 0,
	//	substrate.GetMinCPPNOutputs(), false,
	//	NEAT::ActivationFunction::TANH,
	//	NEAT::ActivationFunction::TANH,
	//	0, params, 0
	//);

	MorphologyInfo info = sim->getWalkerMorphologyInfo();
	NEAT::Genome templateGenome(
		0, info.numSensors, 0, info.numJoints, false,
		NEAT::ActivationFunction::TANH,
		NEAT::ActivationFunction::TANH,
		0, params, 0
	);
	ofLog() << "genome" <<
		"\ninputs:" << templateGenome.NumInputs() <<
		"\noutputs:" << templateGenome.NumOutputs() << "\n";

	offspringGenomeBasePtr = new GenomeBase(templateGenome);
	bestGenomeBasePtr = new GenomeBase(templateGenome);

	// init population and set genome ids that belong to the population
	population = new NEAT::Population(templateGenome, params, true, 1.0, 0);

	maxNumGenerations = 100;
	fitnessResults.reserve(maxNumGenerations);

	maxSimultaneousEvaluations = 4;
	targetFitness = 1.0;

	bThreaded = threaded;
	bThreadedEvaluation = false;
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
	if (bThreadedEvaluation) 
	{
		std::vector<GenomeBase> tempGenomes;
		std::vector<std::thread> evalThreads;
		tempGenomes.reserve(population->NumGenomes());
		evalThreads.reserve(maxSimultaneousEvaluations);

		for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
			for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {
				tempGenomes.push_back(GenomeBase(population->m_Species[i].m_Individuals[j]));
			}
		}

		int index = 0;
		for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
			for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {
				pctGenEvaluated = index / float(population->NumGenomes());
				evalThreads.push_back(std::thread([this, &tempGenomes, i, j, index]
				{
					double f = fitnessFuncPtr->evaluate(tempGenomes[index]);
					population->m_Species[i].m_Individuals[j].SetFitness(f);
					population->m_Species[i].m_Individuals[j].SetEvaluated();
				}));
				if (evalThreads.size() >= maxSimultaneousEvaluations) {
					for (std::thread& t : evalThreads) {
						t.join();
					}
					evalThreads.clear();
				}
				index++;
			}
		}
		// Really make sure all evaluations are finished
		for (std::thread& t : evalThreads) {
			t.join();
		}
	}
	else
	{
		int index = 0;
		for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
			for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {
				pctGenEvaluated = index / float(population->NumGenomes());

				GenomeBase g(population->m_Species[i].m_Individuals[j]);
				double f = fitnessFuncPtr->evaluate(g);
				population->m_Species[i].m_Individuals[j].SetFitness(f);
				population->m_Species[i].m_Individuals[j].SetEvaluated();
				index++;
			}
		}
	}

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

int NEATManager::getNumGeneration()
{
	return population->m_Generation;
}

float NEATManager::getPctGenEvaluated()
{
	return pctGenEvaluated;
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
