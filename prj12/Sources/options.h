#pragma once

#define PICKED_X -1
#define PICKED_Y -1

#define RENDER_SUBIMAGE false
#define XSTART 50
#define YSTART 50
#define XEND 75
#define YEND 75

#define SHOW_NORMALS false
#define TOTAL_BOUNCES 7
#define MAX_SAMPLES 1
#define MIN_SAMPLES 1
#define MAX_THRESHOLD .01
#define MIN_THRESHOLD .000001
#define USE_RAY_DIFFERENTIALS false
#define TEXTURE_SAMPLE_COUNT 32
#define RECONSTRUCTION_FILTER_SIZE	1.5

#define SPHERE_BIAS .001
#define PLANE_BIAS .001
#define TRIANGLE_BIAS .00008

#define DISABLE_SHADOWS false
#define ENABLE_SOFT_SHADOWS false
#define EARLY_SHADOW_TERMINATION true
#define MAX_SHADOW_SAMPLES 256
#define MIN_SHADOW_SAMPLES 64
#define TOTAL_THREADS 8

#define INDIRECT_SAMPLES 1

#define GAMMA 1.0

#define SHOW_VIEWPORT true

#define SCENE "italianStillLife.xml"