#pragma once
#include "ofThread.h"
#include "ofEvents.h"

class Scheduler : public ofThread
{
public:
	ofEvent<void> tick;
	long waitTimeMs = 10000;

	void Scheduler::setup(float waitSeconds) {
		waitTimeMs = long(1000 * waitSeconds);
	};

private:
	void Scheduler::threadedFunction() 
	{
		while (isThreadRunning()) {
			sleep(waitTimeMs);
			tick.notify();
		}
	}
};
