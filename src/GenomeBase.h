#pragma once
#include "Genome.h"

class GenomeBase
{
public:
	GenomeBase(NEAT::Genome genome)
	{
		m_genome = genome;
	};
	~GenomeBase() {};

	void buildPhenotype();
	void buildHyperNEATPhenotype(NEAT::Substrate substrate);
	void buildESHyperNEATPhenotype(NEAT::Substrate substrate, NEAT::Parameters params);
	
	const std::vector<double> activate(std::vector<double> inputs);

	void setGenome(NEAT::Genome genome);
	NEAT::Genome& getGenome();
	NEAT::NeuralNetwork& getNN();
	//NEAT::Substrate& getSubstrate() { return m_substrate; }

private:
	NEAT::Genome m_genome;
	NEAT::NeuralNetwork m_network;
	//NEAT::Substrate m_substrate;
};
