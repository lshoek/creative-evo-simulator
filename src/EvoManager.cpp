#include "EvoManager.h"
#include "EvoUtils.h"
#include "toolbox.h"
#include <boost/thread.hpp>

void EvoManager::setup(SimulationManager* sim)
{
	_simManagerPtr = sim;

	// create a fitness function
	paintFitnessFunc.init(sim);
	fitnessFuncPtr = &paintFitnessFunc;

	// load neat params from config file
	std::string path = "bin/data/config/params_paint.conf";

	std::ifstream paramFile(path.c_str(), std::ifstream::in);
	params.Load(paramFile);
	paramFile.close();

	ofLog() << "Initializing population... (" << params.PopulationSize << " genomes)";

	// get morphology info to build network
	NEAT::Genome templateGenome;
	if (sim->bUseBodyGenomes) {

		std::shared_ptr<DirectedGraph> graphPtr = sim->getBodyGenome();
		glm::ivec2 canvasInputSize = sim->getCanvasNeuronInputResolution();
		uint32_t numBrushToggles = graphPtr->getNumEndNodesUnfolded();

		if (sim->bCanvasInputNeurons) {
			templateGenome = NEAT::Genome(0,
				canvasInputSize.x * canvasInputSize.y + 1,
				(canvasInputSize.x * canvasInputSize.y)/8,
				graphPtr->getNumJointsUnfolded() + numBrushToggles,
				false,
				NEAT::ActivationFunction::TANH,
				NEAT::ActivationFunction::TANH,
				1, params, 1
			);
		}
		else {
			templateGenome = NEAT::Genome(0,
				graphPtr->getNumNodesUnfolded(),
				graphPtr->getNumNodesUnfolded(),
				graphPtr->getNumJointsUnfolded() + numBrushToggles,
				false,
				NEAT::ActivationFunction::TANH,
				NEAT::ActivationFunction::TANH,
				1, params, 1
			);
		}

	} 
	else {
		MorphologyInfo info = sim->getWalkerBodyInfo();
		templateGenome = NEAT::Genome(0,
			info.numSensors, info.numSensors, info.numActuators, false,
			NEAT::ActivationFunction::TANH,
			NEAT::ActivationFunction::TANH,
			1, params, 1
		);
	}

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

	targetFitness = 1.0;

	ofLog() << "Initializing population: Done!";
}

void EvoManager::draw()
{

}

// Export results to some file
void EvoManager::report()
{
	std::string sessionDirName = _simManagerPtr->getUniqueSimId();
	ofDirectory sessionDir(NTRS_SIMS_DIR + sessionDirName);
	sessionDir.create(false);

	std::string reportFilePath = sessionDir.getAbsolutePath() + "/REPORT.txt";
	std::string populationFilePath = sessionDir.getAbsolutePath() + "/POPULATION";
	std::string bestGenomeFilePath = sessionDir.getAbsolutePath() + "/BEST_GENOME";

	ofFile f(reportFilePath, ofFile::WriteOnly, false);

	std::string evalTypeStr = "";
	switch (_simManagerPtr->evaluationType) {
	case SimulationManager::EvaluationType::Coverage:
		evalTypeStr = "Coverage";
		break;
	case SimulationManager::EvaluationType::CircleCoverage:
		evalTypeStr = "CircleCoverage";
		break;
	case SimulationManager::EvaluationType::InverseCircleCoverage:
		evalTypeStr = "InverseCircleCoverage";
		break;
	}

	f << "*** Report ***" << std::endl <<
		"Evaluation Type: " << evalTypeStr <<
		"Fitness: " << population->GetBestFitnessEver() << '/' << targetFitness << std::endl <<
		"Generations: " << population->GetGeneration() << '/' << maxNumGenerations << std::endl <<
		"Total Evaluations: " << population->m_NumEvaluations << std::endl <<
		"Total genomes per population: " << population->NumGenomes() << std::endl <<
		"Total species: " << population->m_Species.size() << std::endl << std::endl <<
		"";

	// Overall best genome per species
	// Artifact IDs
	// Control policy genomes
	// Body genomes
	// For each species: highest fitness at each GEN

	for (int i = 0; i < fitnessResults.size(); i++) {
		f << "Highest Fitness at GEN" << i << " : " << fitnessResults[i] << std::endl;
	}

	ofLog() << f.getFileBuffer();
	f.close();

	population->Save(populationFilePath.c_str());
	population->GetBestGenome().Save(bestGenomeFilePath.c_str());
}

void EvoManager::startEvolution()
{
	bEvolutionActive = true;

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

	// Evolution has stopped
	report();

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
