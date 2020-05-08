#pragma once

/// Global paths
#define NTRS_BODY_GENOME_DIR "output/genomes/morphology/"
#define NTRS_NODE_EXT "node"
#define NTRS_CONN_EXT "conn"

/// Collision detection tags
#define AnonymousTag 0
#define BodyTag 1
#define JointTag 2
#define BrushTag 3
#define TerrainTag 4
#define CanvasTag 5
#define BoundsTag 6

/// GraphNode Primitive types
#define PrimitiveType_Box 0
#define PrimitiveType_Cylinder 1

/// GraphNode Joint types
#define JointType_Fixed 0
#define JointType_Hinge 1
#define JointType_Twist 2
