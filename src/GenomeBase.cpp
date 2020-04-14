#include "GenomeBase.h"

void GenomeBase::buildPhenotype()
{
    m_genome.BuildPhenotype(m_network);
}

void GenomeBase::buildHyperNEATPhenotype(NEAT::Substrate substrate)
{
    m_genome.BuildHyperNEATPhenotype(m_network, substrate);
}

void GenomeBase::buildESHyperNEATPhenotype(NEAT::Substrate substrate, NEAT::Parameters params)
{
    m_genome.BuildESHyperNEATPhenotype(m_network, substrate, params);
}

const std::vector<double> GenomeBase::activate(std::vector<double> inputs) 
{
    m_network.Flush();
    m_genome.CalculateDepth();
    m_network.Input(inputs);

    //unsigned int networkDepth = m_network.CalculateNetworkDepth();
    //for (unsigned int i = 0; i < networkDepth; i++) {

    for (unsigned int i = 0; i < m_genome.GetDepth(); i++) {
        m_network.Activate();
    }
    return m_network.Output();
}

void GenomeBase::updateFitness(double fitness)
{
    m_genome.SetFitness(fitness);
    m_genome.SetEvaluated();
}

void GenomeBase::setGenome(NEAT::Genome genome)
{ 
    m_genome = genome; 
}

NEAT::Genome& GenomeBase::getGenome()
{ 
    return m_genome; 
}

NEAT::NeuralNetwork& GenomeBase::getNN()
{ 
    return m_network; 
}

