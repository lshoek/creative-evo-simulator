#version 450

#define inv(x) 1.0-x
#define BRUSH_COORD_BUF_MAXSIZE 8

struct BrushCoord {
	vec2 coord;
	float impulse;
	float enabled;
};
layout(std430, binding=0) buffer brushCoordBuffer {
	BrushCoord brush_coords[];
};

uniform float alpha = 1.0;
uniform int brush_coords_bufsize = 0;

//const float thickness = 0.0;
//const float min_thickness = 0.005;
//const float thickness = 0.005;
//const float impulse_mult = 0.125/2.0;

const float fade = 0.0125;

in vec2 texcoord_varying;

out float fragColor;
 
void main(void)
{
	vec2 st = texcoord_varying;
	float pct = 0.0;

	for (int i=0; i<BRUSH_COORD_BUF_MAXSIZE; i++) 
	{
		BrushCoord b = brush_coords[i];

		if (i < brush_coords_bufsize)
		{
			float dist = min(distance(b.coord, st), 1.0);
			float dist_inv = inv(dist);

			//float diam = min_thickness + thickness * b.impulse * impulse_mult;
			//float diam = min_thickness + thickness * impulse_mult;
			//float diam = thickness;

			float result = dist_inv * smoothstep(fade, 0.0, dist);
			pct = max(result, pct);
		}
	}
	fragColor = max(pct, 0.0);
}
