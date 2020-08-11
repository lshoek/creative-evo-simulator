#version 450

#define inv(x) 1.0-x
#define BRUSH_COORD_BUF_MAXSIZE 16

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

const float fademin = 1/128.0;
const float fademax = fademin+1/128.0;

in vec2 texcoord_varying;
out float fragColor;
 
void main()
{
	vec2 st = texcoord_varying;
	float pct = 0.0;

	for (int i=0; i<BRUSH_COORD_BUF_MAXSIZE; i++) 
	{
		BrushCoord b = brush_coords[i];

		if (i < brush_coords_bufsize)
		{
			float dist = distance(b.coord, st);
			float result = smoothstep(fademax, fademin, dist);
			pct = max(result, pct);
		}
	}
	fragColor = max(pct, 0.0);
}
