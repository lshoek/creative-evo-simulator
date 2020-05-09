#include "EvoManager.h"
#include "EvoUtils.h"
#include "toolbox.h"
#include <boost/thread.hpp>

void EvoManager::setup(SimulationManager* sim)
{
	// create a fitness function
	paintFitnessFunc.init(sim);
	fitnessFuncPtr = &paintFitnessFunc;

	// load neat params from config file
	std::ifstream paramFile("data/config/params_paint.conf", std::ifstream::in);
	params.Load(paramFile);
	paramFile.close();

	ofLog() << "Initializing population... (" << params.PopulationSize << " genomes)";

	// get morphology info to build network
	NEAT::Genome templateGenome;
	if (sim->bUseBodyGenomes) {
		std::shared_ptr<DirectedGraph> graphPtr = sim->getBodyGenome();
		templateGenome = NEAT::Genome(0,
			graphPtr->getNumNodesUnwrapped(), graphPtr->getNumNodesUnwrapped(), graphPtr->getNumJointsUnwrapped(), false,
			NEAT::ActivationFunction::TANH,
			NEAT::ActivationFunction::TANH,
			1, params, 1
		);
	} 
	else {
		MorphologyInfo info = sim->getWalkerBodyInfo();
		templateGenome = NEAT::Genome(0,
			info.numSensors, info.numSensors, info.numJoints, false,
			NEAT::ActivationFunction::TANH,
			NEAT::ActivationFunction::TANH,
			1, params, 1
		);
	}

	// create substrate
	substrate = EvoUtils::CreateSubstrate(3);
	//substrate.PrintInfo();

	// create cppn genome using substrate config
	//NEAT::Genome templateGenome(0,
	//	substrate.GetMinCPPNInputs(), 0,
	//	substrate.GetMinCPPNOutputs(), false,
	//	NEAT::ActivationFunction::TANH,
	//	NEAT::ActivationFunction::TANH,
	//	0, params, 0
	//);

	// Fully-connected CTRNN constructor causes stackoverflow exception if not checked for loops!
	//NEAT::Genome templateGenome(0, 
	//	info.numSensors, info.numSensors, info.numJoints,
	//	NEAT::ActivationFunction::TANH,
	//	NEAT::ActivationFunction::TANH,
	//	params
	//);

	ofLog() << "template_genome" <<
		"\ninputs:" << templateGenome.NumInputs() <<
		"\noutputs:" << templateGenome.NumOutputs() <<
		"\nneurons:" << templateGenome.NumNeurons() << 
		"\nlinks:" << templateGenome.NumLinks();

	offspringGenomeBasePtr = new GenomeBase(templateGenome);
	bestGenomeBasePtr = new GenomeBase(templateGenome);

	// init population and set genome ids that belong to the population
	population = new NEAT::Population(templateGenome, params, true, 1.0, 0);

	maxNumGenerations = 100;
	fitnessResults.reserve(maxNumGenerations);

	maxParallelEvals = 1;
	targetFitness = 1.0;

	ofLog() << "Initializing population: Done!";
}

void EvoManager::draw()
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

// Export results to some file
void EvoManager::report()
{
	ofLog() << 
		"*** Report ***" << std::endl <<
		"Generations: " << population->GetGeneration() << std::endl <<
		"Num Evaluations: " << population->m_NumEvaluations << std::endl <<
		"Highest Fitness: " << population->GetBestFitnessEver() << std::endl <<
		"Num Genomes: " << population->NumGenomes() << std::endl <<
		"Num Species: " << population->m_Species.size() << std::endl;
}

void EvoManager::startEvolution()
{
	bEvolutionActive = true;
	totalTimeMs = ofGetElapsedTimeMillis();

	if (bThreaded) {
		startThread();
	}
	else {
		evolutionLoop();
	}
}

void EvoManager::stopEvolution()
{
	bStopSimulationQueued = true;
	ofLog() << "Waiting for generation to finish...";
}

void EvoManager::threadedFunction()
{
	evolutionLoop();
}

void EvoManager::evolutionLoop()
{
	bool bTargetReached = false;

	while ((population->m_Generation < maxNumGenerations || maxNumGenerations == -1) &&
		(bestFitness < targetFitness || targetFitness == -1))
	{
		bool bNewBest = false;

		bNewBest = evaluatePopulation();
		bNewBest |= tick();

		// Prematurely evolution loop if a stop is queued
		if (bStopSimulationQueued) {
			break;
		}

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

	bStopSimulationQueued = false;
	bEvolutionActive = false;
	onEvolutionStopped.notify();
}

bool EvoManager::tick()
{
	// Prematurely exit evolution loop if a stop is queued
	if (bStopSimulationQueued) {
		return bestFitness;
	}

	bool bNewBest = false;

	// replace worst genome with new and return it
	offspringGenome = *population->Tick(deadGenome);

	// get access to genome's network, substrate and params
	offspringGenomeBasePtr->setGenome(offspringGenome);

	// evaluate the newly created offspring in Tick()
	int id = fitnessFuncPtr->queueEval(*offspringGenomeBasePtr, false);
	double f = fitnessFuncPtr->awaitEval(id);

	offspringGenomeBasePtr->getGenome().SetFitness(f);
	offspringGenomeBasePtr->getGenome().SetEvaluated();

	bNewBest = f > bestFitness;
	bestFitness = bNewBest ? f : bestFitness;

	return bNewBest;
}

bool EvoManager::evaluatePopulation()
{
	bool bBreak = false;

	if (maxParallelEvals > 1)
	{
		std::vector<GenomeBase> tempGenomes;
		std::vector<std::thread> evalThreads;
		std::mutex populationMutex;

		tempGenomes.reserve(population->NumGenomes());
		evalThreads.reserve(maxParallelEvals);

		for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
			for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {
				tempGenomes.push_back(GenomeBase(population->m_Species[i].m_Individuals[j]));
			}
		}

		int index = 0;
		for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
			for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {

				pctGenEvaluated = index / float(population->NumGenomes());
				GenomeBase gb = GenomeBase(population->m_Species[i].m_Individuals[j]);
				gb.buildPhenotype();

				int id = fitnessFuncPtr->queueEval(gb, true);

				evalThreads.push_back(std::thread([this, &populationMutex, id, i, j, index]
				{
					double f = fitnessFuncPtr->awaitEval(id);

					std::lock_guard<std::mutex> guard(populationMutex);
					population->m_Species[i].m_Individuals[j].SetFitness(f);
					population->m_Species[i].m_Individuals[j].SetEvaluated();
				}));
				if (evalThreads.size() >= maxParallelEvals || index >= population->NumGenomes()-1) {
					for (std::thread& t : evalThreads) {
						t.join();
					}
					evalThreads.clear();
				}
				index++;

				// Prematurely exit evolution loop if a stop is queued
				if (bStopSimulationQueued) {
					bBreak = true;
					break;
				}
			}
			if (bBreak) break;
		}
	}
	else
	{
		int index = 0;
		for (unsigned int i = 0; i < population->m_Species.size(); ++i) {
			for (unsigned int j = 0; j < population->m_Species[i].m_Individuals.size(); ++j) {

				pctGenEvaluated = index / float(population->NumGenomes());
				GenomeBase gb(population->m_Species[i].m_Individuals[j]);
				gb.buildPhenotype();

				int id = fitnessFuncPtr->queueEval(gb, false);
				double f = fitnessFuncPtr->awaitEval(id);

				population->m_Species[i].m_Individuals[j].SetFitness(f);
				population->m_Species[i].m_Individuals[j].SetEvaluated();
				index++;

				// Prematurely exit evolution loop if a stop is queued
				if (bStopSimulationQueued) {
					bBreak = true;
					break;
				}
			}
			if (bBreak) break;
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

bool EvoManager::isEvolutionActive()
{
	return bEvolutionActive;
}

NEAT::Population* EvoManager::getPopulation()
{
	return population;
}

const NEAT::Parameters& EvoManager::getParams()
{
	return params;
}

double EvoManager::getBestFitness()
{
	return bestFitness;
}

double EvoManager::getTargetFitness()
{
	return targetFitness;
}

const std::vector<double>& EvoManager::getFitnessResults()
{
	return fitnessResults;
}

int EvoManager::getNumGeneration()
{
	return population->m_Generation;
}

float EvoManager::getPctGenEvaluated()
{
	return pctGenEvaluated;
}

void EvoManager::setMaxParallelEvals(int max)
{
	maxParallelEvals = max;
}

void EvoManager::exit()
{
	if (bThreaded) {
		waitForThread();
	}
	delete population;
	delete bestGenomeBasePtr;
	delete offspringGenomeBasePtr;
}
