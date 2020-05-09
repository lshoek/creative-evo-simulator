#pragma once

/// Global paths
#define NTRS_BODY_GENOME_DIR "output/genomes/morphology/"
#define NTRS_NODE_EXT "node"
#define NTRS_CONN_EXT "conn"

/// Collision detection tags
#define AnonymousTag	1<<0
#define BodyTag			1<<1
#define JointTag		1<<2
#define BrushTag		1<<3
#define TerrainTag		1<<4
#define CanvasTag		1<<5
#define BoundsTag		1<<6

/// GraphNode Primitive types
#define PrimitiveType_Box 0
#define PrimitiveType_Cylinder 1

/// GraphNode Joint types
#define JointType_Fixed 0
#define JointType_Hinge 1
#define JointType_Twist 2
