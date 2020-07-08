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
	bool bMonitor = true;
	bool bMetaOverlay = true;

	// App toggles
	bool bEvolve = true;
	bool bSimulate = true;
	bool bDraw = true;
	bool bShadows = true;
	bool bGui = true;
	bool bLockFrameRate = true;

	using EvalType = SimulationManager::EvaluationType;
	std::string evalTypeStr(EvalType evalType)
	{
		switch (evalType) {
			case SimulationManager::Coverage: return "Coverage"; break;
			case SimulationManager::CircleCoverage: return "CircleCoverage"; break;
			case SimulationManager::InverseCircleCoverage: return "InverseCircleCoverage"; break;
			case SimulationManager::Aesthetics: return "Aesthetics"; break;
			default: return "NA";
		}
	}
	EvalType evalType(std::string evalTypeStr)
	{
		if (evalTypeStr == "Coverage") return EvalType::Coverage;
		else if (evalTypeStr == "CircleCoverage") return EvalType::CircleCoverage;
		else if (evalTypeStr == "InverseCircleCoverage") return EvalType::InverseCircleCoverage;
		else if (evalTypeStr == "Aesthetics") return EvalType::Aesthetics;
	}

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
