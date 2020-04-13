#pragma once
#include "SimFitnessFunc.h"

class PaintFitnessFunc : public SimFitnessFunc
{
public:
	virtual double evaluate(GenomeBase& genome) override;
};
