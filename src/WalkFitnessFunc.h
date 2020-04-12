#pragma once
#include "FitnessFunc.h"

class WalkFitnessFunc : public FitnessFunc
{
public:
	virtual double evaluate(GenomeBase& genome) override;
};
