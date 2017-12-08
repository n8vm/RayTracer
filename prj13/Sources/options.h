#pragma once
/* General Settings */
#define USE_HALTON true
#define TOTAL_THREADS 5
#define SHOW_VIEWPORT true
#define SCENE "photonCornell.xml"

/* Camera Settings */
#define SHOW_NORMALS false
#define TOTAL_BOUNCES 10
#define MAX_SAMPLES 8
#define MIN_SAMPLES 8
#define MAX_THRESHOLD .01
#define MIN_THRESHOLD .000001
#define SPHERE_BIAS .001
#define PLANE_BIAS .001
#define TRIANGLE_BIAS .00008
#define RENDER_SUBIMAGE false
#define XSTART 0
#define YSTART 400
#define XEND 300
#define YEND 800
#define PICKED_X -1
#define PICKED_Y -1
#define GAMMA 2.2

/* Texture Settings */
#define USE_RAY_DIFFERENTIALS false // BROKEN when using indirect illumination 
#define TEXTURE_SAMPLE_COUNT 32
#define RECONSTRUCTION_FILTER_SIZE	1.5

/* Light Settings */
#define R_SQUARED_FALLOFF true
#define DISABLE_SHADOWS false
#define ENABLE_SOFT_SHADOWS false // THIS MESSES UP HALTON
#define EARLY_SHADOW_TERMINATION false // BROKEN due to transparency
#define MAX_SHADOW_SAMPLES 16
#define MIN_SHADOW_SAMPLES 8

#define DIRECT_TYPE IlluminationType::PATH_TRACING
#define INDIRECT_TYPE IlluminationType::PATH_TRACING
#define INITIAL_PATH_TRACES 2

// Path Tracing Settings
#define INDIRECT_SAMPLES 8


// Irradiance caching
#define USE_IRRADIANCE_CACHE true
#define IRRADIANCE_DIRECT_TYPE IlluminationType::PHOTON
#define COLOR_THRESHOLD .025
#define NORMAL_THRESHOLD .0001
#define Z_THRESHOLD 2.0
#define MIN_LEVEL -4
#define MAX_LEVEL 0

// Photon Mapping Settings 
/*		Photon mapping is very quick compared to path tracing, but is more difficult to get to converge.
			Irradiance maps are helpful to smooth photon mapping out*/
#define USE_PHOTON_MAPPING false
#define USE_CAUSTIC_REFRACTIONS true
#define USE_CAUSTIC_REFLECTIONS true

#define USE_CACHED_PHOTON_MAP true
#define USE_CACHED_REFRACTIONS true
#define USE_CACHED_REFLECTIONS true

#define SAVE_DIRECT_PHOTON true

/* Specifies how many photons are taken into consideration when approximating irradiance. 
		More samples means smoother (& more blurry) result, and may slow down rendering. */
#define MAX_PHOTON_SAMPLES 1000
#define MAX_CAUSTIC_REFLECTION_SAMPLES 4
#define MAX_CAUSTIC_REFRACTION_SAMPLES 4

/* More bounces are more realistic, but take more time and memory */
#define PHOTON_BOUNCES 10

/* Only used if not using cached photon map */
#define TOTAL_PHOTONS 10000000
#define TOTAL_REFRACTION_PHOTONS 10000000
#define TOTAL_REFLECTION_PHOTONS 10000000

// Higher values are slower, lower values are more noisy. This is scene dependent
#define PHOTON_SPHERE_RADIUS 10. 
#define REFRACTION_SPHERE_RADIUS 0.1
#define REFLECTION_SPHERE_RADIUS 0.1

/* Controls the brightness of the photon maps */
#define PHOTON_SCALE (4.0 * M_PI)
#define CAUSTIC_REFRACTION_SCALE (/*4.0 * M_PI*/.8)
#define CAUSTIC_REFLECTION_SCALE (/*4.0 * M_PI*/0.8)
