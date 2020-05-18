#pragma once

struct SimResult {
    double fitness = -1.0;
    int instanceId = -1;
};

struct MorphologyInfo {
    int numSensors = 0;
    int numActuators = 0;
    MorphologyInfo(int nSensors, int nActuators) : 
        numSensors(nSensors), numActuators(nActuators) {}
};
