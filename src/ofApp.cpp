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
	ofSetVerticalSync(true);
	ofDisableArbTex();
	ofSetFrameRate(60);
	ofSetWindowPosition(32, 128);
	ofSetLogLevel("ofThread", OF_LOG_VERBOSE);
	ofLogToConsole();
	ofNoFill();

	settings = ofxIniSettings("settings.ini");
	bEvolve = settings.get("mode.evolve", true);
	bSimulate = settings.get("mode.simulate", true);
	bDebugGrid = settings.get("mode.debuggrid", true);
	bCameraSnapFocus = settings.get("mode.snapfocus", true);

	HWND hwnd = GetConsoleWindow();
	MoveWindow(
		hwnd, ofGetScreenWidth() - CONSOLE_SIZE, CONSOLE_MARGIN, 
		CONSOLE_SIZE, CONSOLE_SIZE, TRUE
	);
	viewRect = ofRectangle(0, 0, ofGetWindowWidth(), ofGetWindowHeight());
	frameFbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA);
	cppnFbo.allocate(512, 512, GL_RGB);

	gui.setup();

	cam.setNearClip(0.01f);
	cam.setFarClip(1000.0f);
	cam.setPosition(8.0f, 8.0f, 4.0f);
	cam.lookAt(glm::vec3(0));

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
	simulationManager.bDebugDraw = settings.get("mode.debugdraw", true);
}

void ofApp::startEvolution()
{
	neatManager.setup(&simulationManager, true);
	neatManager.startEvolution();
	renderEventQueuedListener = neatManager.onNewBestFound.newListener([this] {
		bRenderEventQueued = true;
	});
}

void ofApp::update()
{
	if (bSimulate) {
		simulationManager.update(1/60.0);

		if (bCameraSnapFocus && simulationManager.isSimulationInstanceActive()) {
			cam.lookAt(simulationManager.getFocusOrigin());
		}
	}
}

void ofApp::draw()
{
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

		cam.begin();
		simulationManager.draw();
		cam.end();

		frameFbo.end();
		frameFbo.draw(viewRect);
	}
	glDisable(GL_BLEND);

	if (bGui) 
	{
		gui.begin();
		{
			ImVec2 size(240, 640);
			ImGui::SetNextWindowPos(ImVec2(24, 24));
			ImGui::SetNextWindowSize(size);
			ImGui::SetNextWindowBgAlpha(0.5f);

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
				ImGui::Image((void*)(intptr_t)cppnFbo.getTexture().getTextureData().textureID, ImVec2(size.x, size.x));
				ImGui::Text("fitness: %.03f/%.03f", neatManager.getBestFitness(), neatManager.getTargetFitness());
				ImGui::Separator();
			}
			if (bSimulate) {
				ImGui::Text("cam_pos: x:%.02f, y:%.02f, z:%.02f", cam.getPosition().x, cam.getPosition().y, cam.getPosition().z);
				if (simulationManager.isInitialized()) {
					ImGui::SliderFloat3("light_dir", &simulationManager.lightDirection[0], -1.0f, 1.0f);
					ImGui::SliderFloat3("light_pos", &simulationManager.lightPosition[0], -100.0f, 100.0f);
					if (simulationManager.isSimulationInstanceActive()) {
						ImGui::SliderFloat("motor_strength", &simulationManager.getFocusCreature()->m_motorStrength, 0, 50);
						ImGui::SliderFloat("target_freq", &simulationManager.getFocusCreature()->m_targetFrequency, 1, 60);
						ImGui::Image((void*)(intptr_t)simulationManager.getCanvasFbo()->getTexture().getTextureData().textureID, ImVec2(size.x, size.x));
					}
					ImGui::Text("dbg_draw: %s", simulationManager.bDebugDraw ? "on" : "off");
				}
			}
			ImGui::Text("fps: %.02f", ofGetFrameRate());
			ImGui::End();
		}
		gui.end();
	}
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
				simulationManager.bDraw = !simulationManager.bDraw;
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
		}
	}
}

void ofApp::exit()
{
	if (bSimulate) {
		neatManager.exit();
	}
}
