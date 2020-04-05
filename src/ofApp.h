#pragma once

#include "ofMain.h"
#include "NEATManager.h"
#include "SimulationManager.h"
#include "ofxGrabCam.h"
#include "ofxImGui.h"
#include "ofxIniSettings.h"

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
	void keyPressed(int key);

    NEATManager neatManager;
	SimulationManager simulationManager;

	ofRectangle previewRect;
	ofRectangle viewRect;
	ofFbo frameFbo;

	ofxGrabCam cam;
	ofxImGui::Gui gui;
	ofxIniSettings settings;

	int focusNodeIndex = 0;

	bool bEvolve = true;
	bool bSimulate = true;
	bool bCameraSnapFocus = true;
	bool bDebugGrid = true;
};
