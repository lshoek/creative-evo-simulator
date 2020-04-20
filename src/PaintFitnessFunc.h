#pragma once
#include "SimFitnessFunc.h"

class PaintFitnessFunc : public SimFitnessFunc
{
public:
	virtual isim_ticket queueEval(GenomeBase& genome) override;
	virtual double awaitEval(isim_ticket id) override;
};
