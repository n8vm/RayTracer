#pragma once
/* General Settings */
#define USE_HALTON true
<<<<<<< Updated upstream
#define TOTAL_THREADS 8
=======
#define TOTAL_THREADS 12
>>>>>>> Stashed changes
#define SHOW_VIEWPORT false
#define SCENE "competition2.xml"

/* Camera Settings */
#define SHOW_NORMALS false
#define TOTAL_BOUNCES 4
#define MAX_SAMPLES 1
#define MIN_SAMPLES 1
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
#define PICKED_X 403
#define PICKED_Y 301
#define GAMMA 2.2

/* Texture Settings */
#define USE_RAY_DIFFERENTIALS false // BROKEN when using indirect illumination 
#define TEXTURE_SAMPLE_COUNT 32
#define RECONSTRUCTION_FILTER_SIZE	1.5

/* Light Settings */
#define R_SQUARED_FALLOFF true
#define DISABLE_SHADOWS false
#define ENABLE_SOFT_SHADOWS true // THIS MESSES UP HALTON
#define EARLY_SHADOW_TERMINATION false // BROKEN due to transparency
#define MAX_SHADOW_SAMPLES 16
#define MIN_SHADOW_SAMPLES 8

#define DIRECT_TYPE IlluminationType::PATH_TRACING
#define INDIRECT_TYPE IlluminationType::PHOTON
#define INITIAL_PATH_TRACES 0

// Path Tracing Settings
#define INDIRECT_SAMPLES 32

// Irradiance caching
#define USE_IRRADIANCE_CACHE false
#define IRRADIANCE_DIRECT_TYPE IlluminationType::PHOTON
#define COLOR_THRESHOLD .025
#define NORMAL_THRESHOLD .0001
#define Z_THRESHOLD 2.0
#define MIN_LEVEL -4
#define MAX_LEVEL 0

// Photon Mapping Settings 
/*		Photon mapping is very quick compared to path tracing, but is more difficult to get to converge.
			Irradiance maps are helpful to smooth photon mapping out*/
#define USE_SPPM true // Stochastic Progressive Photon Mapping
#define USE_PHOTON_MAPPING true
#define USE_CAUSTIC_REFRACTIONS false
#define USE_CAUSTIC_REFLECTIONS true

#define USE_CACHED_PHOTON_MAP true
#define USE_CACHED_REFRACTIONS true
#define USE_CACHED_REFLECTIONS true

// Include direct illumination
#define SAVE_DIRECT_PHOTON false

// determines how the radius shrinks over time
#define PHOTON_ALPHA .666

/* Specifies how many photons are taken into consideration when approximating irradiance. 
		More samples means smoother (& more blurry) result, and may slow down rendering. */
#define MAX_PHOTON_SAMPLES 10000
#define MAX_CAUSTIC_REFLECTION_SAMPLES 10000
#define MAX_CAUSTIC_REFRACTION_SAMPLES 10000

/* More bounces are more realistic, but take more time and memory */
#define PHOTON_BOUNCES 10

/* Only used if not using cached photon map */
#define TOTAL_PHOTONS 1000000
#define TOTAL_REFRACTION_PHOTONS 10000
#define TOTAL_REFLECTION_PHOTONS 25000

// Higher values are slower, lower values are more noisy. This is scene dependent
#define PHOTON_SPHERE_RADIUS .25 
#define REFRACTION_SPHERE_RADIUS .25
#define REFLECTION_SPHERE_RADIUS .25

/* Controls the brightness of the photon maps */
#define PHOTON_SCALE (8.0 * M_PI)
#define CAUSTIC_REFRACTION_SCALE (8.0 * M_PI)
#define CAUSTIC_REFLECTION_SCALE (8.0 * M_PI)
