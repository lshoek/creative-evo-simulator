#pragma once
#include "FitnessFunc.h"

class XORFitnessFunc : public FitnessFunc
{
public:
	virtual double evaluate(GenomeBase& genome) override;
};
