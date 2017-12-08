#pragma once

#define PICKED_X -1
#define PICKED_Y -1

#define R_SQUARED_FALLOFF true

#define RENDER_SUBIMAGE false
#define XSTART 50
#define YSTART 50
#define XEND 75
#define YEND 75

#define SHOW_NORMALS false
#define TOTAL_BOUNCES 6
#define MAX_SAMPLES 64
#define MIN_SAMPLES 4
#define MAX_THRESHOLD .01
#define MIN_THRESHOLD .000001
#define USE_RAY_DIFFERENTIALS true // BROKEN when using indirect illumination 
#define TEXTURE_SAMPLE_COUNT 32
#define RECONSTRUCTION_FILTER_SIZE	1.5
#define USE_HALTON false

#define SPHERE_BIAS .001
#define PLANE_BIAS .001
#define TRIANGLE_BIAS .00008

#define DISABLE_SHADOWS false
#define ENABLE_SOFT_SHADOWS true // THIS MESSES UP HALTON
#define EARLY_SHADOW_TERMINATION false // BROKEN due to transparency
#define MAX_SHADOW_SAMPLES 16
#define MIN_SHADOW_SAMPLES 8
#define TOTAL_THREADS 8

#define INDIRECT_SAMPLES 0

#define USE_IRRADIANCE_CACHE false
#define COLOR_THRESHOLD .01
#define NORMAL_THRESHOLD .9
#define Z_THRESHOLD 5.0
#define MIN_LEVEL -5
#define MAX_LEVEL 0

#define GAMMA 2.2

#define SHOW_VIEWPORT true

#define SCENE "GICornell.xml"