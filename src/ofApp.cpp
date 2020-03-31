#include "ofApp.h"
#include "toolbox.h"

#define APP_ID "neatures v0.01"
#define CONSOLE_SIZE 600
#define CONSOLE_MARGIN 24

void ofApp::setup()
{
	ofSetWindowTitle(APP_ID);
	ofSetVerticalSync(true);
	ofSetFrameRate(60);
	ofSetWindowPosition(32, 128);
	ofSetLogLevel("ofThread", OF_LOG_VERBOSE);
	ofLogToConsole();
	ofNoFill();

	HWND hwnd = GetConsoleWindow();
	MoveWindow(
		hwnd, ofGetScreenWidth() - CONSOLE_SIZE, CONSOLE_MARGIN, 
		CONSOLE_SIZE, CONSOLE_SIZE, TRUE
	);

	neatManager.setup(true);
	neatManager.startEvolution();
}

void ofApp::update()
{
	//
}

void ofApp::draw()
{
	ofBackground(0x22);
	neatManager.draw();
}

void ofApp::windowResized(int w, int h)
{
	//
}

void ofApp::exit()
{
	neatManager.exit();
}
