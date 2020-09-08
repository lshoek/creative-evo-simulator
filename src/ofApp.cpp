#include "ofApp.h"
#include "Simulator/SimDefines.h"
#include "Simulator/SimNode.h"
#include "Utils/OFUtils.h"
#include "Utils/MathUtils.h"
#include "Utils/FixedQueue.h"

#define APP_ID "neatures v0.2"
#define CONSOLE_SIZE 600
#define CONSOLE_MARGIN 24

void ofApp::setup()
{
	ofEnableAntiAliasing();
	ofEnableSmoothing();
	ofSetWindowTitle(APP_ID);
	ofSetVerticalSync(false);
	ofDisableArbTex();
	ofSetWindowPosition(32, 128);
	ofNoFill();

	ofSetLogLevel("ofThread", OF_LOG_VERBOSE);
	ofEnableGLDebugLog();
	ofLogToConsole();
	//ofLog() << "Num threads: " << std::thread::hardware_concurrency();

	uint32_t frameRate = (bLockFrameRate) ? 60 : 0;
	ofSetFrameRate(frameRate);

	int seed = GetTickCount();
	ofLog() << "seed: " << seed;
	ofSeedRandom(seed);

	settings = ofxIniSettings("settings.ini");
	bDraw = settings.get("mode.draw", true);
	bMonitor = settings.get("mode.monitor", true);

	HWND hwnd = GetConsoleWindow();
	MoveWindow(
		hwnd, ofGetScreenWidth() - CONSOLE_SIZE, CONSOLE_MARGIN, 
		CONSOLE_SIZE, CONSOLE_SIZE, TRUE
	);

	bwindowRenderResolution = settings.get("rendering.window", true);
	int renderWidth = settings.get("rendering.width", ofGetWindowWidth());
	int renderHeight = settings.get("rendering.height", ofGetWindowHeight());

	windowRect = ofRectangle(0, 0, ofGetWindowWidth(), ofGetWindowHeight());

	frameFboSettings.width = bwindowRenderResolution ? windowRect.width : renderWidth;
	frameFboSettings.height = bwindowRenderResolution ? windowRect.height : renderHeight;
	frameFboSettings.internalformat = GL_RGBA;
	frameFboSettings.useDepth = true;
	frameFboSettings.numSamples = 4;
	frameFbo.allocate(frameFboSettings);

	gui.setup();

	initSim();
}

void ofApp::initSim() 
{
	if (!simulationManager.isInitialized()) {
		simulationManager.bDebugDraw = settings.get("mode.debugdraw", true);
		simulationManager.bAutoLoadGenome = settings.get("genome.autoload", true);
		simulationManager.bAxisAlignedAttachments = settings.get("genome.axis_aligned_attachments", false);
		simulationManager.bFeasibilityChecks = settings.get("genome.feasibility_checks", true);
		simulationManager.bCanvasSensors = settings.get("sensors.type", "canvas").compare("canvas") == 0;
		simulationManager.bSaveArtifactsToDisk = settings.get("canvas.save", true);

		SimulationManager::SimSettings simSettings;
		simSettings.evalType = evalType(settings.get("eval.type", "Coverage"));
		simSettings.canvasSize = settings.get("canvas.size", 4.0f);
		simSettings.canvasResolution = settings.get("canvas.resolution", 256);
		simSettings.canvasConvResolution = settings.get("canvas.resolution_conv", 64);
		simSettings.canvasViewSize = settings.get("canvas.viewsize", 1.0f);
		simSettings.canvasMargin = settings.get("canvas.margin", 4.0f);
		simSettings.maxParallelSims = settings.get("evolution.max_parallel_sims", 1);
		simSettings.genomeFile = settings.get("genome.id", "0");
		simSettings.host = settings.get("controller.host", "localhost");
		simSettings.outPort = settings.get("controller.port", 1024);
		simSettings.inPort = settings.get("controller.port_in", 1025);

		simulationManager.init(simSettings);
	}
}

void ofApp::start()
{
	// Start simulator first so it can await evaluation requests
	if (!simulationManager.isSimulationActive()) {
		simulationManager.startSimulation();
	}
}

void ofApp::stop()
{
	// Then abort the sim instances that are currently active
	if (simulationManager.isSimulationActive()) {
		simulationManager.terminateSimInstances();
		simulationManager.stopSimulation();
	}
}

void ofApp::update()
{
	if (simulationManager.isInitialized()) {
		uint64_t start = ofGetElapsedTimeMillis();
		simulationManager.update();
		simulationManager.lateUpdate();
		perf_update = ofGetElapsedTimeMillis() - start;
	}
}

void ofApp::draw()
{
	if (bDraw) {
		uint64_t start = ofGetElapsedTimeMillis();

		ofBackground(0x000000); //0x222222
		ofSetHexColor(0xffffff);

		glEnable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (bSimulate)
		{
			simulationManager.shadowPass();

			frameFbo.begin();
			ofClear(0, 0);
			simulationManager.draw();
			frameFbo.end();

			frameFbo.draw(windowRect);
		}
		glDisable(GL_BLEND);
		glDisable(GL_DEPTH_TEST);

		perf_draw = ofGetElapsedTimeMillis() - start;
	}
	if (bGui) {
		imGui();
	}
}

void ofApp::imGui()
{
	gui.begin();
	{
		ImVec2 windowSize(200, 540);
		ImVec2 overlaySize(240, 300);
		ImVec2 margin(0, 4);
		ImVec2 menuBarSize = ImVec2(0, 0);
		uint32_t offset = 24;

		ImGuiWindowFlags flags =
			ImGuiWindowFlags_NoTitleBar |
			ImGuiWindowFlags_AlwaysAutoResize |
			ImGuiWindowFlags_NoSavedSettings |
			ImGuiWindowFlags_NoFocusOnAppearing |
			ImGuiWindowFlags_NoNav;

		if (ImGui::BeginMainMenuBar())
		{
			menuBarSize = ImGui::GetWindowSize();
			if (ImGui::BeginMenu("Genome"))
			{
				/* disclaimer: I coded this really fast leave me alone*/
				if (ImGui::BeginMenu("Load")) {
					const std::vector<ofFile> files = ofDirectory(ofToDataPath(NTRS_BODY_GENOME_DIR, true)).getFiles();
					std::vector<std::shared_ptr<GuiFileItem>> items;
					for (ofFile f : files) {
						if (f.isDirectory()) {
							std::string fname = f.getFileName();
							items.push_back(std::make_unique<GuiFileItem>(fname.c_str()));
						}
					}
					for (std::shared_ptr<GuiFileItem> i : items) {
						if (ImGui::MenuItem(i->getRawFileName(), NULL, false)) {
							simulationManager.loadGenomeFromDisk(i->fileName);
						}
					}
					ImGui::EndMenu();
				}
				if (ImGui::MenuItem("Save", NULL, false)) {
					simulationManager.getSelectedGenome()->save();
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Generate", NULL, false)) {
					simulationManager.generateRandomGenome();
				}
				ImGui::InputInt("Min Nodes", &simulationManager.genomeGenMinNumNodes);
				ImGui::InputInt("Min Connections", &simulationManager.genomeGenMinNumConns);
				if (ImGui::MenuItem("Feasibility Checks", NULL, simulationManager.bFeasibilityChecks)) {
					simulationManager.bFeasibilityChecks = !simulationManager.bFeasibilityChecks;
				}
				if (ImGui::MenuItem("Axis Aligned Attachments", NULL, simulationManager.bAxisAlignedAttachments)) {
					simulationManager.bAxisAlignedAttachments = !simulationManager.bAxisAlignedAttachments;
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Simulation"))
			{
				if (ImGui::MenuItem("Start", "e", simulationManager.isSimulationActive())) {
					start();
				}
				if (ImGui::MenuItem("Stop", NULL, false)) {
					stop();
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Shift Camera Focus", "c", false)) {
					simulationManager.shiftFocus();
				}
				if (ImGui::MenuItem("Terminate Activate Simulations", "x", false)) {
					simulationManager.terminateSimInstances();
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Window"))
			{
				if (ImGui::MenuItem("Monitor", "m", bMonitor)) {
					bMonitor = !bMonitor;
				}
				if (ImGui::MenuItem("Meta Overlay", "o", bMetaOverlay)) {
					bMetaOverlay = !bMetaOverlay;
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Settings"))
			{
				if (ImGui::MenuItem("Save Artifacts", NULL, simulationManager.bSaveArtifactsToDisk)) {
					simulationManager.bSaveArtifactsToDisk = !simulationManager.bSaveArtifactsToDisk;
				}
				if (ImGui::MenuItem("Lock Frame Rate", NULL, bLockFrameRate)) {
					bLockFrameRate = !bLockFrameRate;
					uint32_t frameRate = (bLockFrameRate) ? 60 : 0;
					ofSetFrameRate(frameRate);
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Debug"))
			{
				ImGui::SliderFloat("Light Intensity", &simulationManager.lightIntensity, 0.0f, 5000.0f);
				if (ImGui::MenuItem("Debug Renderer", "d", simulationManager.bDebugDraw)) {
					simulationManager.bDebugDraw = !simulationManager.bDebugDraw;
				}
				if (ImGui::MenuItem("Debug Light", "l", simulationManager.bMouseLight)) {
					simulationManager.bMouseLight = !simulationManager.bMouseLight;
				}
				if (ImGui::MenuItem("View Lightspace Depth", NULL, simulationManager.bViewLightSpaceDepth)) {
					simulationManager.bViewLightSpaceDepth = !simulationManager.bViewLightSpaceDepth;
				}
				if (ImGui::MenuItem("View Canvas Evaluation Mask", NULL, simulationManager.bViewCanvasEvaluationMask)) {
					simulationManager.bViewCanvasEvaluationMask = !simulationManager.bViewCanvasEvaluationMask;
				}
				if (ImGui::MenuItem("Canvas Sensors", NULL, simulationManager.bCanvasSensors)) {
					simulationManager.bCanvasSensors = !simulationManager.bCanvasSensors;
				}
				if (ImGui::MenuItem("Reload Shaders", "r", false)) {
					simulationManager.loadShaders();
				}
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}

		// Overlay
		if (bMetaOverlay) {
			ImGui::SetNextWindowPos(ImVec2(ofGetWindowWidth() - overlaySize.x - offset, menuBarSize.y + offset));
			ImGui::SetNextWindowSize(overlaySize);
			ImGui::SetNextWindowBgAlpha(0.5f);

			if (ImGui::Begin("Overlay", NULL, flags | ImGuiWindowFlags_NoScrollbar)) {
				ImGui::Text("Meta");
				ImGui::Separator();
				if (simulationManager.isInitialized()) {
					ImGui::Text(&simulationManager.getUniqueSimId()[0]);
				}
				ImGui::Dummy(margin);

				glm::vec3 cpos = simulationManager.getCamera()->getPosition();
				ImGui::Text("cam: (%.02f, %.02f, %.02f)", cpos.x, cpos.y, cpos.z);
				ImGui::Text("update: %dms", perf_update);
				ImGui::Text("draw: %dms", perf_draw);
				ImGui::Text("fps: %.02f", ofGetFrameRate());
				if (simulationManager.isInitialized()) {
					ImGui::Text("timesteps: %d", simulationManager.getTimeStepsPerUpdate());
				}
				ImGui::Text("dbgdraw: %s", simulationManager.bDebugDraw ? "on" : "off");
				ImGui::Dummy(margin);

				if (simulationManager.getSelectedGenome()) {
					ImGui::Text("Creature");
					ImGui::Separator();
					ImGui::Text("id: %s", simulationManager.getSelectedGenome()->getName().c_str());
					ImGui::Text("nodes: %d", simulationManager.getSelectedGenome()->getNumNodesUnfolded());
					ImGui::Text("joints: %d", simulationManager.getSelectedGenome()->getNumJointsUnfolded());
					ImGui::Text("brushes: %d", simulationManager.getSelectedGenome()->getNumBrushes());
					ImGui::Text("viewsize: %.4f", simulationManager.getSettings().canvasViewSize);
					ImGui::Dummy(margin);
				}
				ImGui::Text("Eval");
				ImGui::Separator();
				ImGui::Text("func: %s", evalTypeStr(simulationManager.getEvaluationType()).c_str());
				if (simulationManager.bStoreLastArtifact && simulationManager.getPrevArtifactTexture() != nullptr) {
					ImGui::Image(
						(void*)(intptr_t)simulationManager.getPrevArtifactTexture()->getTextureData().textureID,
						ImVec2(windowSize.x - margin.x, windowSize.x - margin.x)
					);
				}
			}	
			ImGui::End();

		}
		ImGui::SetNextWindowPos(ImVec2(0, ofGetWindowHeight() - offset));
		ImGui::SetNextWindowSize(ImVec2(ofGetWindowWidth(), offset));
		ImGui::SetNextWindowBgAlpha(0.5f);

		if (ImGui::Begin("Footer", NULL, flags | ImGuiWindowFlags_NoScrollbar)) {
			ImGui::Text("%s", simulationManager.getStatus().c_str());
		}
		ImGui::End();

		if (bMonitor) {
			ImGui::SetNextWindowPos(ImVec2(0, menuBarSize.y));
			ImGui::SetNextWindowSize(ImVec2(240, ofGetWindowHeight()-(menuBarSize.y+offset)));
			ImGui::SetNextWindowBgAlpha(0.5f);
			ImGui::Begin("Monitor", NULL, flags);

			if (bSimulate) {
				if (simulationManager.isInitialized()) {
					ImGui::Text("Simulation Speed:");
					ImGui::SliderInt("##SimSpeed", (int*)&simulationManager.simulationSpeed, 0, 32);
					ImGui::Separator();
					if (simulationManager.isSimulationInstanceActive()) {
						ImGui::Text("Elapsed time:");
						ImGui::Text(simulationManager.getFocusInfo().c_str());
						ImGui::Separator();
						if (simulationManager.getFocusCanvas()) {
							ImGui::Text("Creature Artifact:");
							ImGui::Image(
								(void*)(intptr_t)simulationManager.getFocusCanvas()->getPaintMapRGBA()->getTexture().getTextureData().textureID, 
								ImVec2(windowSize.x-margin.x, windowSize.x-margin.x)
							);
							ImGui::Text("Neural Input:");
							ImGui::Image(
								(void*)(intptr_t)simulationManager.getFocusCanvas()->getViewMap()->getTexture().getTextureData().textureID, 
								ImVec2(windowSize.x-margin.x, windowSize.x-margin.x)
							);
							ImGui::Separator();
						}
						if (simulationManager.getFocusCreature()) {
							const auto hist = simulationManager.getCPGBuffer();
							ImGui::Text("CPG:");
							ImGui::PlotLines("##CPG", &hist[0], hist.size(), 0, ofToString(hist[0], 2).c_str(), 0, 1.f, ImVec2(windowSize.x-margin.x, windowSize.x/4));
							ImGui::Separator();
							ImGui::Text("Effectors:");
							ImGui::TextWrapped(ofToString(simulationManager.getFocusCreature()->getOutputs()).c_str());
							ImGui::Separator();
						}
						if (simulationManager.bDebugDraw) {
							ImGui::Text("Global Motor Strength:");
							ImGui::SliderFloat("##GMStrength", &simulationManager.getFocusCreature()->m_motorStrength, 0, 1);
							ImGui::Separator();
						}
					}
				}
			}
			ImGui::End();
		}
	}
	gui.end();
}

void ofApp::windowResized(int w, int h)
{
	windowRect = ofRectangle(0, 0, w, h);
	previewRect = ofRectangle(w*0.75f, h*0.75f, w*0.2f, h*0.2f);

	if (bwindowRenderResolution) {
		frameFboSettings.width = windowRect.width;
		frameFboSettings.height = windowRect.height;
		frameFbo.allocate(frameFboSettings);
	}
}

void ofApp::keyPressed(int key)
{
	if (bSimulate) {
		if (key == 'e') {
			start();
		}
		if (key == 'g') {
			bGui = !bGui;
		}
		if (key == 'm') {
			bMonitor = !bMonitor;
		}
		if (key == 'o') {
			bMetaOverlay = !bMetaOverlay;
		}
		if (simulationManager.isInitialized()) {
			if (key == 'd') {
				simulationManager.bDebugDraw = !simulationManager.bDebugDraw;
			}
			if (key == 'D') {
				bDraw = !bDraw;
			}
			if (key == 'r') {
				simulationManager.loadShaders();
			}
			if (key == 'l') {
				simulationManager.bMouseLight = !simulationManager.bMouseLight;
			}
			if (key == 'c') {
				simulationManager.shiftFocus();
			}
			if (key == 'x') {
				simulationManager.terminateSimInstances();
			}
		}
	}
}

void ofApp::exit()
{
	stop();
}
