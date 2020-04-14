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

	void startSimulation();
	void startEvolution();

	void windowResized(int w, int h);
	void keyPressed(int key);

	void queueRenderEvent();

private:
    NEATManager neatManager;
	SimulationManager simulationManager;

	ofEventListener renderEventQueuedListener;

	ofRectangle previewRect;
	ofRectangle viewRect;

	ofFbo frameFbo;
	ofFbo cppnFbo;

	ofxGrabCam cam;
	ofxImGui::Gui gui;
	ofxIniSettings settings;

	bool bRenderEventQueued = false;

	bool bEvolve = true;
	bool bSimulate = true;
	bool bCameraSnapFocus = true;
	bool bDebugGrid = true;
	bool bGui = true;
};
