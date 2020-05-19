#include "ofApp.h"
#include "SimDefines.h"
#include "SimNode.h"
#include "toolbox.h"
#include "MathUtils.h"

#define APP_ID "neatures v0.01"
#define CONSOLE_SIZE 600
#define CONSOLE_MARGIN 24

void ofApp::setup()
{
	ofSetWindowTitle(APP_ID);
	ofSetFrameRate(60);
	ofSetVerticalSync(false);
	ofDisableArbTex();
	ofSetWindowPosition(32, 128);
	ofBackground(0x222222);
	ofNoFill();

	ofSetLogLevel("ofThread", OF_LOG_VERBOSE);
	ofEnableGLDebugLog();
	ofLogToConsole();
	//ofLog() << "Num threads: " << std::thread::hardware_concurrency();

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
	cppnFbo.allocate(512, 512, GL_RGB);

	gui.setup();

	initSimulation();
}

void ofApp::initSimulation() 
{
	if (!simulationManager.isInitialized()) {

		uint32_t canvasSize = settings.get("canvas.size", 128);
		uint32_t canvasNeuralInputSize = settings.get("genome.canvas_neural_input_size", 32);

		simulationManager.bDebugDraw = settings.get("mode.debugdraw", true);
		simulationManager.bCameraSnapFocus = settings.get("mode.snapfocus", true);
		simulationManager.bAxisAlignedAttachments = settings.get("genome.axis_aligned_attachments", false);
		simulationManager.bUseBodyGenomes = settings.get("genome.body_genomes", true);
		simulationManager.bFeasibilityChecks = settings.get("genome.feasibility_checks", true);
		simulationManager.bCanvasInputNeurons = settings.get("genome.canvas_neural_input", false);
		simulationManager.bSaveArtifactsToDisk = settings.get("canvas.save", true);

		simulationManager.setMaxParallelSims(settings.get("evolution.max_parallel_evals", 4));
		simulationManager.setCanvasNeuronInputResolution(canvasNeuralInputSize, canvasNeuralInputSize);
		simulationManager.init(canvasSize, canvasSize);
	}
}

// Note: the simulator and evolution module don't wait for each other to finish, 
// which can cause problems on certain user-input.
void ofApp::startEvolution()
{
	std::string uniqueSimId = ofGetTimestampString("%Y%m%d_%H%M%S_%i");

	// Start simulator first so it can await evaluation requests
	if (!simulationManager.isSimulationActive()) {
		simulationManager.startSimulation(uniqueSimId, SimulationManager::EvaluationType::CircleCoverage);
	}
	// Then call the evolution loop
	if (!evoManager.isEvolutionActive()) {
		evoManager.setup(&simulationManager);
		evoManager.setMaxParallelEvals(settings.get("evolution.max_parallel_evals", 4));
		evoManager.startEvolution();
		renderEventQueuedListener = evoManager.onNewBestFound.newListener([this] {
			bRenderEventQueued = true;
			renderEventQueuedListener.unsubscribe();
		});
		evolutionStoppedListener = evoManager.onEvolutionStopped.newListener([this] {
			evoManager.report();
			evolutionStoppedListener.unsubscribe();
		});
	}
}

void ofApp::stopEvolution()
{
	// First queue a simulation stop
	if (evoManager.isEvolutionActive()) {
		evoManager.stopEvolution();
	}
	// Then abort the sim instances that are currently active
	if (simulationManager.isSimulationActive()) {
		simulationManager.abortSimInstances();
		simulationManager.stopSimulation();
	}
}

void ofApp::update() 
{
	if (simulationManager.isInitialized()) {
		simulationManager.updateTime();
		simulationManager.lateUpdate();
	}
}

void ofApp::draw()
{
	if (bDraw) {
		ofBackground(0x222222);
		ofSetHexColor(0xffffff);

		glEnable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (bRenderEventQueued) {
			cppnFbo.begin();
			ofClear(0, 1.0f);
			evoManager.draw();
			cppnFbo.end();
			bRenderEventQueued = false;
		}
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
	}
	if (bGui) {
		imGui();
	}
}

void ofApp::imGui()
{
	gui.begin();
	{
		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("Main"))
			{
				if (ImGui::MenuItem("Monitor Window", "w", bWindow)) {
					bWindow = !bWindow;
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Evolution"))
			{
				if (ImGui::MenuItem("Start", "e", simulationManager.isSimulationActive())) {
					startEvolution();
				}
				if (ImGui::MenuItem("Stop", NULL, false)) {
					stopEvolution();
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Generate Genome", NULL, false)) {
					simulationManager.generateRandomBodyGenome();
				}
				if (ImGui::MenuItem("Feasibility Checks", NULL, simulationManager.bFeasibilityChecks)) {
					simulationManager.bFeasibilityChecks = !simulationManager.bFeasibilityChecks;
				}
				if (ImGui::MenuItem("Axis Aligned Attachments", NULL, simulationManager.bAxisAlignedAttachments)) {
					simulationManager.bAxisAlignedAttachments = !simulationManager.bAxisAlignedAttachments;
				}
				if (ImGui::MenuItem("Input Canvas Neurons", NULL, simulationManager.bCanvasInputNeurons)) {
					simulationManager.bCanvasInputNeurons = !simulationManager.bCanvasInputNeurons;
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Save Genome", NULL, false)) {
					simulationManager.getBodyGenome()->save();
				}
				/* disclaimer: I coded this really fast leave me alone*/
				if (ImGui::BeginMenu("Load Genome")) {
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
				if (ImGui::MenuItem("Save Artifacts", NULL, simulationManager.bSaveArtifactsToDisk)) {
					simulationManager.bSaveArtifactsToDisk = !simulationManager.bSaveArtifactsToDisk;
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Shift Camera Focus", "c", false)) {
					simulationManager.shiftFocus();
				}
				if (ImGui::MenuItem("Skip Generation", "z", false)) {
					simulationManager.abortSimInstances();
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Debug"))
			{
				if (ImGui::MenuItem("Debug Renderer", "d", simulationManager.bDebugDraw)) {
					simulationManager.bDebugDraw = !simulationManager.bDebugDraw;
				}
				if (ImGui::MenuItem("Shadows", NULL, simulationManager.bShadows)) {
					simulationManager.bShadows = !simulationManager.bShadows;
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
				if (ImGui::MenuItem("Reload Shaders", "r", false)) {
					simulationManager.loadShaders();
				}
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}

		if (bWindow)
		{
			ImVec2 size(240, 540);
			ImGui::SetNextWindowSize(size);
			ImGui::SetNextWindowBgAlpha(0.75f);
			ImGui::Begin("Monitor");

			if (bSimulate) {
				if (simulationManager.isInitialized()) {
					ImGui::Text("Simulation Time:");
					ImGui::Text("%.02f", simulationManager.getSimulationTime());
					ImGui::Separator();
					ImGui::Text("Simulation Speed:");
					ImGui::SliderInt("<", (int*)&simulationManager.simulationSpeed, 0, 16);
					ImGui::Separator();
					if (simulationManager.isSimulationInstanceActive()) {
						if (simulationManager.getFocusCanvas() != nullptr) {
							ImGui::Text("Creature Artifact:");
							ImGui::Image((void*)(intptr_t)simulationManager.getFocusCanvas()->getCanvasFbo()->getTexture().getTextureData().textureID, ImVec2(size.x, size.x));
							ImGui::Text("Neural Input:");
							ImGui::Image((void*)(intptr_t)simulationManager.getFocusCanvas()->getCanvasNeuralInputRawFbo()->getTexture().getTextureData().textureID, ImVec2(size.x, size.x));
							ImGui::Separator();
						}
						if (simulationManager.getFocusCreature() != nullptr) {
							ImGui::Text("Activation time:");
							ImGui::Text("%dms", simulationManager.getFocusCreature()->getActivationMillis());
							ImGui::Text("Global Motor Strength:");
							ImGui::SliderFloat("<<", &simulationManager.getFocusCreature()->m_motorStrength, 0, 1);
							ImGui::Separator();
						}
					}
					glm::vec3 cpos = simulationManager.getCamera()->getPosition();
					ImGui::Text("Camera Position:");
					ImGui::Text("(%.02f, %.02f, %.02f)", cpos.x, cpos.y, cpos.z);
					ImGui::Separator();
					ImGui::Text("Light Position:");
					ImGui::InputFloat3("<<<", &simulationManager.lightPosition[0], 2);
					ImGui::Separator();
					ImGui::Text("Debug Draw: %s", simulationManager.bDebugDraw ? "ON" : "OFF");
					ImGui::Separator();
				}
			}
			if (evoManager.isEvolutionActive()) {
				std::vector<float> fitnessFloats(
					evoManager.getFitnessResults().begin(),
					evoManager.getFitnessResults().end()
				);
				if (!fitnessFloats.empty()) {
					ImGui::Text("Best Fitness per Generation:");
					ImGui::PlotLines(
						"Fitness", &fitnessFloats[0], fitnessFloats.size(), 0, 0,
						0, evoManager.getTargetFitness(), ImVec2(size.x, 96)
					);
					ImGui::Separator();
				}
				ImGui::Text("Current Generation: %d", evoManager.getNumGeneration());
				ImGui::Text("Evaluated: %.02f%%", evoManager.getPctGenEvaluated() * 100.0f);
				ImGui::Text("Best Fitness: %.03f/%.03f", evoManager.getBestFitness(), evoManager.getTargetFitness());
				ImGui::Separator();
			}
			ImGui::Text("FPS: %.02f", ofGetFrameRate());
			ImGui::End();
		}
	}
	gui.end();
}

void ofApp::queueRenderEvent()
{
	bRenderEventQueued = true;
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
			startEvolution();
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
			if (key == 'z') {
				simulationManager.abortSimInstances();
			}
		}
	}
}

void ofApp::exit()
{
	stopEvolution();
	evoManager.exit();
}
