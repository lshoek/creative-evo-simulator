#pragma once

#include "ofMain.h"
#include "Simulator/SimulationManager.h"
#include "ofxGrabCam.h"
#include "ofxImGui.h"
#include "ofxIniSettings.h"

class ofApp : public ofBaseApp
{
public:
	void setup();
	void update();
    void draw();
    void exit();

	void imGui();

	void initSim();
	void start();
	void stop();

	void windowResized(int w, int h);
	void keyPressed(int key);

private:
	SimulationManager simulationManager;

	ofEventListener renderEventQueuedListener;
	ofEventListener evolutionStoppedListener;

	ofRectangle previewRect;
	ofRectangle viewRect;

	ofFbo frameFbo;

	ofxImGui::Gui gui;
	ofxIniSettings settings;

	uint64_t perf_update = 0;
	uint64_t perf_draw = 0;

	// Gui toggles
	bool bWindow = true;
	bool bMetaOverlay = true;

	// App toggles
	bool bEvolve = true;
	bool bSimulate = true;
	bool bDraw = true;
	bool bShadows = true;
	bool bGui = true;
	bool bLockFrameRate = true;

	struct GuiFileItem
	{
		GuiFileItem(const char* fname) : fileName(fname) {}
		std::string fileName;

		const char* getRawFileName() const {
			return fileName.c_str();
		}
	};
	std::vector<GuiFileItem> genomesOnDisk;
};
