#pragma once
#include "GenomeBase.h"

class FitnessFunc
{
public:
	virtual double evaluate(GenomeBase& genome) = 0;
};
