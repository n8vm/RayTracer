#pragma once

#define PICKED_X -1
#define PICKED_Y -1
#define SHOW_NORMALS false
#define TOTAL_BOUNCES 4
#define MAX_SAMPLES 128
#define MIN_SAMPLES 64
#define MAX_THRESHOLD .01
#define MIN_THRESHOLD .0
#define USE_RAY_DIFFERENTIALS true
#define TEXTURE_SAMPLE_COUNT 32
#define RECONSTRUCTION_FILTER_SIZE 2.0

#define SPHERE_BIAS .001
#define PLANE_BIAS .0001
#define TRIANGLE_BIAS .000008

#define DISABLE_SHADOWS false
#define TOTAL_THREADS 10

#define SCENE "DepthOfField.xml"