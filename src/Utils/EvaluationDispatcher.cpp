#include "Utils/EvaluationDispatcher.h"
#include "Simulator/SimDefines.h"

EvaluationDispatcher::EvaluationDispatcher() { }

void EvaluationDispatcher::setup(EvaluationType type, uint32_t width, uint32_t height)
{
	if (type == Coverage) _evaluator = std::make_unique<CoverageEvaluator>();
	else if (type == CircleCoverage) _evaluator = std::make_unique<CircleCoverageEvaluator>();
	else if (type == InverseCircleCoverage) _evaluator = std::make_unique<InverseCircleCoverageEvaluator>();
	else if (type == OrderlyCoverage) _evaluator = std::make_unique<OrderlyCoverageEvaluator>();
	else if (type == Aesthetics) _evaluator = std::make_unique<AestheticEvaluator>();
	else _evaluator = std::make_unique<CoverageEvaluator>();

	_evaluator->setup(width, height);

	ofAddListener(ofEvents().update, this, &EvaluationDispatcher::update);
	startThread();
	bSetup = true;
}

void EvaluationDispatcher::queue(cv::Mat image, int generation, int id)
{
	ArtifactEntry entry{image};
	entry.generation = generation;
	entry.id = id;

	ofLog() << "Queued " << generation << ":" << id << " for evaluation";
	_evalQueue.send(entry);
}

void EvaluationDispatcher::queueResponse()
{
	ArtifactEntry entry{};
	entry.response = 1;
	_evalQueue.send(entry);
}

void EvaluationDispatcher::threadedFunction() 
{
	ArtifactEntry entry;
	while (_evalQueue.receive(entry)) {
		if (entry.response != 1) {
			double fitness = _evaluator->evaluate(entry.image);
			entry.fitness = fitness;
		}
		_updateQueue.send(entry);
	}
}

void EvaluationDispatcher::update(ofEventArgs& a) 
{
	ArtifactEntry entry;
	if (_updateQueue.tryReceive(entry)) {
		if (entry.response != 1) {
			ofLog() << "Finished evaluating " << entry.generation << ":" << entry.id << "  f:" << entry.fitness;
			_fitnessQueue.push_back(entry.fitness);
		}
		else {
			onFitnessResponseReady.notify(_fitnessQueue);
			_fitnessQueue.clear();
		}
	}
}

EvaluationDispatcher::~EvaluationDispatcher()
{
	if (bSetup) {
		_evalQueue.close();
		_updateQueue.close();
		waitForThread(true);
		ofRemoveListener(ofEvents().update, this, &EvaluationDispatcher::update);
	}
}
