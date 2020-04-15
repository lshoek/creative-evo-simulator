#pragma once

struct SimResult {
    double fitness = -1.0;
    int instanceId = -1;
};
struct MorphologyInfo {
    int numSensors = 0;
    int numJoints = 0;
    MorphologyInfo(int nSensors, int nJoints) : 
        numSensors(nSensors), numJoints(nJoints) {}
};
