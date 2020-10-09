#pragma once

#include "ofMain.h"
#include "Simulator/SimulationManager.h"
#include "Artifact/EvaluationType.h"
#include "ofxImGui.h"
#include "ofxIniSettings.h"
#include "ofxFFmpegRecorder.h"
#include "ofxFastFboReader.h"

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
	ofRectangle renderRect;
	ofRectangle renderRectFlipped;

	ofFbo frameFbo;
	ofFbo guiFbo;
	ofFboSettings frameFboSettings;

	ofxImGui::Gui gui;
	ofxIniSettings settings;

	ofxFFmpegRecorder recorder;
	ofxFastFboReader fboReader;
	ofFbo recordFbo;
	ofPixels recordPixels;

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
	bool bFullScreen = true;
	bool bLockFrameRate = true;
	bool bAutoCam = false;
	bool bFirstFrame = false;
	bool bRecording = false;
	bool bwindowRenderResolution = false;

	const std::string FFMPEG_PATH = "c:/ffmpeg/bin/ffmpeg.exe";
	const std::string RECORDINGS_DIR = "recordings/";
	const std::string RECORDINGS_EXT = ".mp4";

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
