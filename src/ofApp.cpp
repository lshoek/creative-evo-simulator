#include "ofApp.h"
#include "Simulator/SimDefines.h"
#include "Simulator/SimNode.h"
#include "Utils/toolbox.h"
#include "Utils/MathUtils.h"

#define APP_ID "neatures v0.2"
#define CONSOLE_SIZE 600
#define CONSOLE_MARGIN 24

void ofApp::setup()
{
	ofSetWindowTitle(APP_ID);
	ofSetVerticalSync(false);
	ofDisableArbTex();
	ofSetWindowPosition(32, 128);
	ofBackground(0x222222);
	ofNoFill();

	ofSetLogLevel("ofThread", OF_LOG_VERBOSE);
	ofEnableGLDebugLog();
	ofLogToConsole();
	//ofLog() << "Num threads: " << std::thread::hardware_concurrency();

	uint32_t frameRate = (bLockFrameRate) ? 60 : 0;
	ofSetFrameRate(frameRate);

	uint64_t seed = GetTickCount();
	ofLog() << "seed: " << seed;
	ofSeedRandom(seed);

	settings = ofxIniSettings("settings.ini");
	bDraw = settings.get("mode.draw", true);
	bWindow = settings.get("mode.window", true);

	HWND hwnd = GetConsoleWindow();
	MoveWindow(
		hwnd, ofGetScreenWidth() - CONSOLE_SIZE, CONSOLE_MARGIN, 
		CONSOLE_SIZE, CONSOLE_SIZE, TRUE
	);
	viewRect = ofRectangle(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
	frameFbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA);

	gui.setup();

	initSim();
}

void ofApp::initSim() 
{
	if (!simulationManager.isInitialized()) {
		simulationManager.bDebugDraw = settings.get("mode.debugdraw", true);
		simulationManager.bAutoLoadGenome = settings.get("genome.autoload", true);
		simulationManager.bCameraSnapFocus = settings.get("mode.snapfocus", true);
		simulationManager.bAxisAlignedAttachments = settings.get("genome.axis_aligned_attachments", false);
		simulationManager.bUseBodyGenomes = settings.get("genome.body_genomes", true);
		simulationManager.bFeasibilityChecks = settings.get("genome.feasibility_checks", true);
		simulationManager.bCanvasSensors = settings.get("sensors.type", "canvas").compare("canvas") == 0;
		simulationManager.bSaveArtifactsToDisk = settings.get("canvas.save", true);

		SimulationManager::SimSettings simSettings;
		simSettings.evalType = SimulationManager::Coverage;
		simSettings.canvasSize = settings.get("canvas.size", 256);
		simSettings.canvasSizeConv = settings.get("canvas.size_conv", 128);
		simSettings.maxParallelSims = settings.get("evolution.max_parallel_sims", 1);
		simSettings.genomeFile = settings.get("genome.id", "0");
		simSettings.host = settings.get("io.host", "localhost");
		simSettings.inPort = settings.get("io.port_in", 1025);
		simSettings.outPort = settings.get("io.port_out", 1024);

		simulationManager.init(simSettings);
	}
}

// Note: the simulator and evolution module don't wait for each other to finish, 
// which can cause problems on certain user-input.
void ofApp::start()
{
	std::string uniqueSimId = ofGetTimestampString("%Y%m%d_%H%M%S_%i");

	// Start simulator first so it can await evaluation requests
	if (!simulationManager.isSimulationActive()) {
		simulationManager.startSimulation(uniqueSimId, SimulationManager::EvaluationType::InverseCircleCoverage);
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

		ofBackground(0x222222);
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

			frameFbo.draw(viewRect);
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
		ImVec2 windowSize(240, 540);
		ImVec2 overlaySize(240, 140);
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
							simulationManager.loadBodyGenomeFromDisk(i->fileName);
						}
					}
					ImGui::EndMenu();
				}
				if (ImGui::MenuItem("Save", NULL, false)) {
					simulationManager.getBodyGenome()->save();
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Generate", NULL, false)) {
					simulationManager.generateRandomBodyGenome();
				}
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
				if (ImGui::MenuItem("Monitor", "w", bWindow)) {
					bWindow = !bWindow;
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
				if (ImGui::MenuItem("Debug Renderer", "d", simulationManager.bDebugDraw)) {
					simulationManager.bDebugDraw = !simulationManager.bDebugDraw;
				}
				if (ImGui::MenuItem("Debug Light", NULL, simulationManager.bMouseLight)) {
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

			if (ImGui::Begin("Meta", NULL, flags | ImGuiWindowFlags_NoScrollbar)) {
				ImGui::Text("meta");
				ImGui::Separator();
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

				//ImGui::Text("Creature Name");
				//ImGui::Separator();
				//if (simulationManager.getFocusCreature() != nullptr) {
				//}
				//ImGui::InputFloat3("light:", &simulationManager.lightPosition[0], 2);
			}	
			ImGui::End();

			ImGui::SetNextWindowPos(ImVec2(0, ofGetWindowHeight() - offset));
			ImGui::SetNextWindowSize(ImVec2(ofGetWindowWidth(), offset));
			ImGui::SetNextWindowBgAlpha(0.5f);

			if (ImGui::Begin("Footer", NULL, flags | ImGuiWindowFlags_NoScrollbar)) {
				ImGui::Text("%s", simulationManager.getStatus().c_str());
			}
			ImGui::End();
		}

		if (bWindow) {
			ImGui::SetNextWindowPos(ImVec2(0, menuBarSize.y));
			ImGui::SetNextWindowSize(ImVec2(240, ofGetWindowHeight()-(menuBarSize.y+offset)));
			ImGui::SetNextWindowBgAlpha(0.5f);
			ImGui::Begin("Monitor", NULL, flags);

			if (bSimulate) {
				if (simulationManager.isInitialized()) {
					ImGui::Text("Simulation Speed:");
					ImGui::SliderInt("<", (int*)&simulationManager.simulationSpeed, 0, 16);
					ImGui::Separator();
					if (simulationManager.isSimulationInstanceActive()) {
						ImGui::Text("Elapsed time:");
						ImGui::Text(simulationManager.getFocusInfo().c_str());
						ImGui::Separator();
						if (simulationManager.getFocusCanvas() != nullptr) {
							ImGui::Text("Creature Artifact:");
							ImGui::Image((void*)(intptr_t)simulationManager.getFocusCanvas()->getCanvasFbo()->getTexture().getTextureData().textureID, ImVec2(windowSize.x, windowSize.x));
							ImGui::Text("Neural Input:");
							ImGui::Image((void*)(intptr_t)simulationManager.getFocusCanvas()->getConvFbo()->getTexture().getTextureData().textureID, ImVec2(windowSize.x, windowSize.x));
							ImGui::Separator();
							ImGui::Text("Effectors:");
							ImGui::TextWrapped(ofToString(simulationManager.getFocusCreature()->getOutputs()).c_str());
						}
						if (simulationManager.bDebugDraw) {
							if (simulationManager.getFocusCreature() != nullptr) {
								ImGui::Text("Global Motor Strength:");
								ImGui::SliderFloat("<<", &simulationManager.getFocusCreature()->m_motorStrength, 0, 1);
								ImGui::Separator();
							}
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
	viewRect = ofRectangle(0, 0, w, h);
	frameFbo.allocate(w, h, GL_RGBA8);
	previewRect = ofRectangle(w*0.75f, h*0.75f, w*0.2f, h*0.2f);
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
		if (key == 'w') {
			bWindow = !bWindow;
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
