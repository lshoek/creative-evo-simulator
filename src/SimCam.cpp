#include "SimCam.h"
#include "SimUtils.h"
#include "toolbox.h"

void SimCam::setup()
{
	ofAddListener(ofEvents().mouseMoved, this, &SimCam::mouseMoved);
	ofAddListener(ofEvents().mouseDragged, this, &SimCam::mouseDragged);
}

void SimCam::update()
{
	if (bHasFocusNode) 
	{
		glm::vec3 focus = SimUtils::bulletToGlm(focusNodePtr->getPosition());
		forward = glm::normalize(focus - ofCamera::getPosition());

		// update distance to target
		if (bLMouseDragged) {
			glm::vec3 target = ofCamera::getPosition() + forward * diff.y;
			if (glm::dot(focus - target, forward) > 0) {
				glm::vec3 nodeToTarget = target - focus;
				nodeToTarget = glm::clamp(nodeToTarget, minDistToNode, 1000.0f);

				glm::vec3 result = focus + nodeToTarget;
				ofCamera::setPosition(result);
			}
		}
		// update dir
		forward = glm::normalize(focus - ofCamera::getPosition());
		glm::vec3 right = glm::cross(glm::vec3(0, 1, 0), forward);
		glm::vec3 up = glm::cross(forward, right);
		ofCamera::lookAt(focus, up);
	}
	bLMouseDragged = false;
	bRMouseDragged = false;
}

void SimCam::setFocusNode(SimNode* node)
{
	focusNodePtr = node;
	bHasFocusNode = true;
}

void SimCam::mouseMoved(ofMouseEventArgs& mouse)
{
	prevMouse = curMouse;
	curMouse = glm::vec3(mouse.x, mouse.y, 0);
}

void SimCam::mouseDragged(ofMouseEventArgs& mouse)
{
	prevMouse = curMouse;
	curMouse = glm::vec3(mouse.x, mouse.y, 0);

	rawDiff = tb::lim(curMouse - prevMouse, 0, maxDiffLength);
	diff = rawDiff * mouseDamping;

	if (mouse.button == 0) bLMouseDragged = true;
	if (mouse.button > 0) bRMouseDragged = true;
}
