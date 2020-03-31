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

	void buildNetwork();
	const std::vector<double> activate(std::vector<double> inputs);

	void setGenome(NEAT::Genome genome) { m_genome = genome; }

	NEAT::Genome& getGenome() { return m_genome; }
	NEAT::NeuralNetwork& getNN() { return m_network; }

private:
	NEAT::Genome m_genome;
	NEAT::NeuralNetwork m_network;
};
