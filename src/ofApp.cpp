#include "ofApp.h"
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

	settings = ofxIniSettings("settings.ini");
	bEvolve = settings.get("mode.evolve", true);
	bSimulate = settings.get("mode.simulate", true);
	bDebugGrid = settings.get("mode.debuggrid", true);
	bDraw = settings.get("mode.draw", true);

	HWND hwnd = GetConsoleWindow();
	MoveWindow(
		hwnd, ofGetScreenWidth() - CONSOLE_SIZE, CONSOLE_MARGIN, 
		CONSOLE_SIZE, CONSOLE_SIZE, TRUE
	);
	viewRect = ofRectangle(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
	frameFbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA);
	cppnFbo.allocate(512, 512, GL_RGB);

	gui.setup();

	if (bSimulate) {
		startSimulation();
	}
	if (bEvolve) {
		startEvolution();
	}
}

void ofApp::startSimulation()
{
	simulationManager.init();
	simulationManager.setMaxParallelSims(settings.get("evo.max_parallel_evals", 4));
	simulationManager.setMorphologyGenomeModeEnabled(settings.get("mode.body_genomes", true));
	simulationManager.bDebugDraw = settings.get("mode.debugdraw", true);
	simulationManager.bCameraSnapFocus = settings.get("mode.snapfocus", true);
	simulationManager.startSimulation();
}

void ofApp::startEvolution()
{
	neatManager.setup(&simulationManager);
	neatManager.setMaxParallelEvals(settings.get("evo.max_parallel_evals", 4));
	neatManager.startEvolution();
	renderEventQueuedListener = neatManager.onNewBestFound.newListener([this] {
		bRenderEventQueued = true;
	});
}

void ofApp::update() 
{
	if (bSimulate && simulationManager.isInitialized()) {
		simulationManager.updateTime();
	}
}

void ofApp::draw()
{
	if (bDraw) {
		ofBackground(0x222222);
		ofSetHexColor(0xffffff);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		if (bRenderEventQueued) {
			cppnFbo.begin();
			ofClear(0, 1.0f);
			neatManager.draw();
			cppnFbo.end();
			bRenderEventQueued = false;
		}
		if (bSimulate)
		{
			frameFbo.begin();
			ofClear(0, 0);

			simulationManager.draw();

			frameFbo.end();
			frameFbo.draw(viewRect);
		}
		glDisable(GL_BLEND);
	}
	if (bGui) {
		imGui();
	}
}

void ofApp::imGui()
{
	gui.begin();
	{
		ImVec2 size(240, 640);
		ImGui::SetNextWindowPos(ImVec2(24, 24));
		ImGui::SetNextWindowSize(size);
		ImGui::SetNextWindowBgAlpha(0.75f);

		ImGui::Begin(APP_ID);

		if (bEvolve) {
			std::vector<float> fitnessFloats(
				neatManager.getFitnessResults().begin(),
				neatManager.getFitnessResults().end()
			);
			if (!fitnessFloats.empty()) {
				ImGui::PlotLines(
					"Fitness", &fitnessFloats[0], fitnessFloats.size(), 0, 0,
					0, neatManager.getTargetFitness(), ImVec2(size.x, 96)
				);
			}
			//ImGui::Image((void*)(intptr_t)cppnFbo.getTexture().getTextureData().textureID, ImVec2(size.x, size.x));
			ImGui::Text("current_gen: %d", neatManager.getNumGeneration());
			ImGui::Text("evaluated: %.02f%%", neatManager.getPctGenEvaluated() * 100.0f);
			ImGui::Text("best_fitness: %.03f/%.03f", neatManager.getBestFitness(), neatManager.getTargetFitness());
			ImGui::Separator();
		}
		if (bSimulate) {
			glm::vec3 cpos = simulationManager.getCamera()->getPosition();
			ImGui::Text("cam_pos: (%.02f, %.02f, %.02f)", cpos.x, cpos.y, cpos.z);
			ImGui::Separator();
			if (simulationManager.isInitialized()) {
				ImGui::SliderFloat3("light_pos", &simulationManager.lightPosition[0], -20.0f, 20.0f);
				ImGui::Separator();
				ImGui::Text("sim_time: %.02f", simulationManager.getSimulationTime());
				ImGui::SliderInt("sim_speed", (int*)&simulationManager.simulationSpeed, 0, 16);
				if (simulationManager.isSimulationInstanceActive()) {
					ImGui::SliderFloat("motor_strength", &simulationManager.getFocusCreature()->m_motorStrength, 0, 1);
					ImGui::SliderFloat("target_freq", &simulationManager.getFocusCreature()->m_targetFrequency, 1, 60);
					ImGui::Image((void*)(intptr_t)simulationManager.getCanvasFbo()->getTexture().getTextureData().textureID, ImVec2(size.x, size.x));
					ImGui::Separator();
				}
				ImGui::Text("dbg_draw: %s", simulationManager.bDebugDraw ? "on" : "off");
			}
		}
		ImGui::Text("fps: %.02f", ofGetFrameRate());
		ImGui::End();
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
			if (!bEvolve) {
				startEvolution();
				bEvolve = true;
			}
		}
		if (key == 't') {
			simulationManager.initTestEnvironment();
		}
		if (key == 'g') {
			bGui = !bGui;
		}
		if (simulationManager.isInitialized()) {
			if (key == 'd') {
				simulationManager.bDebugDraw = !simulationManager.bDebugDraw;
			}
			if (key == 'D') {
				bDraw = !bDraw;
			}
			//if (key == 's') {
			//	simulationManager.saveCanvas();
			//}
			if (key == 'r') {
				simulationManager.loadShaders();
			}
			if (key == 'c') {
				simulationManager.shiftFocus();
			}
			if (key == 'z') {
				simulationManager.nextSimulation();
			}
		}
	}
}

void ofApp::exit()
{
	if (bSimulate) {
		neatManager.exit();
	}
}
