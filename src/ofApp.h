#pragma once

#include "ofMain.h"
#include "Simulator/SimulationManager.h"
#include "Artifact/EvaluationType.h"
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
	ofRectangle windowRect;

	ofFbo frameFbo;
	ofFboSettings frameFboSettings;

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
	bool bwindowRenderResolution = false;

	std::string evalTypeStr(EvaluationType evalType)
	{
		if (evalType == EvaluationType::Coverage) return "Coverage";
		if (evalType == EvaluationType::CircleCoverage) return "CircleCoverage";
		if (evalType == EvaluationType::InverseCircleCoverage) return "InverseCircleCoverage";
		if (evalType == EvaluationType::OrderlyCoverage) return "OrderlyCoverage";
		if (evalType == EvaluationType::Aesthetics) return "Aesthetics";
		else return "NA";
	}

	EvaluationType evalType(std::string evalTypeStr)
	{
		if (evalTypeStr == "Coverage") return EvaluationType::Coverage;
		else if (evalTypeStr == "CircleCoverage") return EvaluationType::CircleCoverage;
		else if (evalTypeStr == "InverseCircleCoverage") return EvaluationType::InverseCircleCoverage;
		else if (evalTypeStr == "OrderlyCoverage") return EvaluationType::OrderlyCoverage;
		else if (evalTypeStr == "Aesthetics") return EvaluationType::Aesthetics;
		else return EvaluationType::Coverage;
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
