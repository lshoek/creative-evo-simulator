#pragma once
#include "ofThread.h"
#include "ofThreadChannel.h"
#include "Artifact/EvaluationType.h"
#include "Artifact/SimpleEvaluators.h"
#include "Artifact/AestheticEvaluator.h"
#include "Artifact/OrderlyCoverageEvaluator.h"

class EvaluationDispatcher : public ofThread 
{
public:
    ofEvent<const std::vector<double>&> onFitnessResponseReady;

    EvaluationDispatcher();
    ~EvaluationDispatcher();

    void setup(EvaluationType type, uint32_t width, uint32_t height);
    void queue(cv::Mat image, int generation, int id);
    void queueResponse();

private:
    void update(ofEventArgs& a);
    virtual void threadedFunction();

    std::unique_ptr<EvaluatorBase> _evaluator;

    struct ArtifactEntry
    {
        ArtifactEntry() {}
        ArtifactEntry(cv::Mat im) : image(im) {}

        cv::Mat image;
        double fitness = -1.0;
        int generation;
        int id;
        int response = 0;
    };
    ofThreadChannel<ArtifactEntry> _evalQueue;
    ofThreadChannel<ArtifactEntry> _updateQueue;

    std::vector<double> _fitnessQueue;
    bool bSetup = false;
};
