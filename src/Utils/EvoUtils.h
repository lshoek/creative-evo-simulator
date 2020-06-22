#pragma once
#include "MultiNEAT.h"
#include <boost/assign/std.hpp>

using namespace boost::assign;

static class EvoUtils 
{
private:
	static std::vector<std::vector<double>> sheetSpace(double start, double stop, int dim, double z) {
		std::vector<double> linspace;
		std::vector<std::vector<double>> space;
		linspace.reserve(dim);
		space.reserve(dim * dim);

		for (int i = 0; i < dim; i++) {
			linspace.push_back(ofLerp(start, stop, i / float(dim>1?dim-1:1)));
		}

		for (int i = 0; i < dim; i++) {
			for (int j = 0; j < dim; j++) {
				std::vector<double> v;
				v += linspace[i], linspace[j], z;
				space.push_back(v);
			}
		}
		return space;
	}

public:
	static NEAT::Substrate CreateSubstrate(int dim)
	{
		// substrate config
		std::vector<std::vector<double>> substrateInputs = sheetSpace(-1, 1, dim, -1);
		std::vector<std::vector<double>> substrateOutputs = sheetSpace(-1, 1, dim, -1);
		std::vector<std::vector<double>> substrateHidden;

		NEAT::Substrate substrate = NEAT::Substrate(
			substrateInputs,
			substrateHidden,
			substrateOutputs
		);

		substrate.m_allow_input_hidden_links = true;
		substrate.m_allow_input_output_links = false;
		substrate.m_allow_hidden_output_links = true;
		substrate.m_allow_hidden_hidden_links = false;
		substrate.m_hidden_nodes_activation = NEAT::ActivationFunction::SIGNED_SIGMOID;
		substrate.m_output_nodes_activation = NEAT::ActivationFunction::UNSIGNED_SIGMOID;
		substrate.m_max_weight_and_bias = 8.0;

		return substrate;
	}
};
