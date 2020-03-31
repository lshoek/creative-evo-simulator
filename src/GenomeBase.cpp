#include "GenomeBase.h"

void GenomeBase::buildNetwork() 
{
    m_genome.BuildPhenotype(m_network);
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
