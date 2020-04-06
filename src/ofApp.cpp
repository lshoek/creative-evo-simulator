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
	frameFbo.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGBA8);
	gui.setup();

	cam.setNearClip(0.01f);
	cam.setFarClip(1000.0f);
	cam.setPosition(0.0f, 40.0f, 24.0f);
	cam.lookAt(glm::vec3(0));

	if (bSimulate) {
		simulationManager.init();
		simulationManager.bDebugDraw = settings.get("mode.debugdraw", true);
	}
	if (bEvolve) {
		neatManager.setup(true);
		neatManager.startEvolution();
	}
}

void ofApp::update()
{
	float delta = ofGetLastFrameTime();

	if (bSimulate) {
		simulationManager.update(delta);

		if (bCameraSnapFocus) {
			cam.lookAt(simulationManager.getNodes()[focusNodeIndex]->getPosition());
		}
	}
}

void ofApp::draw()
{
	ofBackground(0x222222);
	ofSetHexColor(0xffffff);

	glEnable(GL_DEPTH_TEST);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
	glDisable(GL_DEPTH_TEST);

	ofSetHexColor(0xffffff);
	gui.begin();
	{
		ImVec2 size(240, 240);
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
			ImGui::Text("fitness: %.03f/%.03f", neatManager.getBestFitness(), neatManager.getTargetFitness());
			ImGui::Separator();
		}
		if (bSimulate) {
			ImGui::SliderFloat3("light_dir", &simulationManager.lightDirection[0], -1.0f, 1.0f);
			ImGui::SliderFloat3("light_pos", &simulationManager.lightPosition[0], -100.0f, 100.0f);
			ImGui::Text("#nodes: %d", simulationManager.getNumNodes());
			ImGui::Text("cam_pos: x:%.02f, y:%.02f, z:%.02f", cam.getPosition().x, cam.getPosition().y, cam.getPosition().z);
			ImGui::Text("dbg_draw: %s", simulationManager.bDebugDraw ? "on" : "off");
		}
		ImGui::Text("fps: %.02f", ofGetFrameRate());
		ImGui::End();
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
		if (key == 'd') {
			simulationManager.bDebugDraw = !simulationManager.bDebugDraw;
		}
		if (key == 'D') {
			simulationManager.bDraw = !simulationManager.bDraw;
		}
		if (key == 's') {
			simulationManager.loadShaders();
		}
		if (key == 'r') {
			simulationManager.reset();
		}
		if (key == 'f') {
			simulationManager.applyForce(true);
		}
		if (key == 'F') {
			simulationManager.applyForce(false);
		}
		if (key == 'c') {
			focusNodeIndex = (focusNodeIndex+1)%simulationManager.getNodes().size();
		}
	}
}

void ofApp::exit()
{
	if (bSimulate) {
		neatManager.exit();
	}
}
