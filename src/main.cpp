#include "ofMain.h"
#include "ofApp.h"
#include "ofAppGLFWWindow.h"

int main()
{
	ofGLFWWindowSettings settings;
	settings.setGLVersion(4, 5);
	settings.setSize(1280, 720);
	ofCreateWindow(settings);

	return ofRunApp(std::make_shared<ofApp>());
}
