#pragma once

/// Global paths
const std::string NTRS_SIMS_DIR = "output/sims/";
const std::string NTRS_BODY_GENOME_DIR = "output/genomes/morphology/";
const std::string NTRS_OUTPUT_DIR = "output/";

/// Prefixes
const std::string NTRS_ARTIFACTS_PREFIX = "artifacts/";

/// Extensions
const std::string NTRS_NODE_EXT = "node";
const std::string NTRS_CONN_EXT = "conn";

/// Collision detection tags
const uint32_t  AnonymousTag =	1 << 0;
const uint32_t  BodyTag =		1 << 1;
const uint32_t  JointTag =		1 << 2;
const uint32_t  BrushTag =		1 << 3;
const uint32_t  TerrainTag =	1 << 4;
const uint32_t  CanvasTag =		1 << 5;
const uint32_t  BoundsTag =		1 << 6;

/// GraphNode Primitive types
const uint32_t PrimitiveType_Box = 0;
const uint32_t PrimitiveType_Cylinder = 1;

/// GraphNode Joint types
const uint32_t JointType_Fixed = 0;
const uint32_t JointType_Hinge = 1;
const uint32_t JointType_Twist = 2;

/// OSC Message Protocol
const std::string OSC_FRAME_START = "/frame/start/";
const std::string OSC_FRAME_PART = "/frame/part/";
const std::string OSC_FRAME_END = "/frame/end/";
const std::string OSC_FRAME_INFO = "/frame/info/";

const std::string OSC_FRAME = "frame";
const std::string OSC_START = "start";
const std::string OSC_PART = "part";
const std::string OSC_END = "end";
const std::string OSC_INFO = "info";
