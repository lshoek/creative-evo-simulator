#pragma once
#include "ofThread.h"
#include "ofEvents.h"

class Scheduler : public ofThread
{
public:
	ofEvent<void> tick;
	long waitTimeMs = 10000;

	void Scheduler::setup(float f) {
		waitTimeMs = long(1000 * f);
	};

private:
	void Scheduler::threadedFunction() 
	{
		sleep(waitTimeMs);
		ofNotifyEvent(tick);
		waitForThread();
	}
};
