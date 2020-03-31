#pragma once

#include "ofMain.h"
#include "NEATManager.h"

#include "MultiNEAT.h"
#include "GenomeBase.h"
#include "XORFitnessFunc.h"

class ofApp : public ofBaseApp
{
public:
	void setup();
	void update();
    void draw();
    void exit();

	void windowResized(int w, int h);	

    NEATManager neatManager;
};
