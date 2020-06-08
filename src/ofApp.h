#pragma once

#include "ofMain.h"
#include "EvoManager.h"
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

	void imGui();

	void initSimulation();
	void startEvolution();
	void stopEvolution();

	void windowResized(int w, int h);
	void keyPressed(int key);

	void queueRenderEvent();

private:
    EvoManager evoManager;
	SimulationManager simulationManager;

	ofEventListener renderEventQueuedListener;
	ofEventListener evolutionStoppedListener;

	ofRectangle previewRect;
	ofRectangle viewRect;

	ofFbo frameFbo;
	ofFbo cppnFbo;

	ofxImGui::Gui gui;
	ofxIniSettings settings;

	uint64_t perf_update = 0;
	uint64_t perf_draw = 0;

	// Gui toggles
	bool bWindow = true;
	bool bMetaOverlay = true;

	// App toggles
	bool bRenderEventQueued = false;
	bool bEvolve = true;
	bool bSimulate = true;
	bool bDraw = true;
	bool bShadows = true;
	bool bGui = true;

	struct GuiFileItem
	{
		GuiFileItem(const char* fname) : fileName(fname) {}
		std::string fileName;

		const char* getRawFileName() const
		{
			return fileName.c_str();
		}
	};
	std::vector<GuiFileItem> genomesOnDisk;
};
