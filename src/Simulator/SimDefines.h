#pragma once

/// Global paths
const std::string NTRS_SIMS_DIR = "sims/";
const std::string NTRS_BODY_GENOME_DIR = "genomes/";

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

/// Simulation
const double FIXED_TIMESTEP = 1.0 / 60.0;
const double FIXED_TIMESTEP_MILLIS = (1.0 / 60.0) * 1000.0;
