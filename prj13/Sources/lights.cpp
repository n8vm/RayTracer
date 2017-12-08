#include "lights.h"
#include "render.h"
#include "options.h"
#include <time.h>
#include "cyCodeBase/cyMatrix.h"

extern Node rootNode;
extern LightList lights;
extern TexturedColor environment;

extern HaltonIDX haltonIDX[TOTAL_THREADS];

float GenLight::Shadow(Rays rays, int tid, float t_max) {
	if (DISABLE_SHADOWS) return 1;
	HitInfo hInfo = HitInfo(tid);
	hInfo.shadow = EARLY_SHADOW_TERMINATION;
	
	Rays shadowRays = rays;
	while (true) {
		bool hit = Trace(shadowRays, rootNode, hInfo, HIT_FRONT_AND_BACK, t_max);
		if (!hit) return 1;

		/* Make sure the thing we hit isn't transparent */
		if (hInfo.node->GetMaterial()->isTransparent(hInfo)) {
			shadowRays = Rays(rays, hInfo.z + .001);
		}
		else return 0;
	}
}

Color PointLight::Illuminate(const Point3 &p, const Point3 &N, int tid) const {
	if (MAX_SHADOW_SAMPLES == 0) return intensity;

	Ray initialRay = Ray(p, position - p);
	float dist = (R_SQUARED_FALLOFF) ? initialRay.dir.LengthSquared() : 1.0;
	if (dist < 1.0) dist = 1.0;

	if (ENABLE_SOFT_SHADOWS && size > 0.0) {
		cyPoint3f from = cyPoint3f(0, 0, 1); 
		cyPoint3f to = initialRay.dir.GetNormalized();
		cyMatrix4f transform = cyMatrix4f::MatrixTrans(position) * cyMatrix4f::MatrixRotation(from, to);

		int totalSamples = MIN_SHADOW_SAMPLES;
	
		float shadowSum = 0;
		for (int i = 0; i < totalSamples; ++i) {
			int idx = haltonIDX[tid].SSidx();
			float t = 2 * M_PI*Halton(idx, 2);
			float u = Halton(idx, 3) + Halton(idx, 5);
			float r = (u > 1.0) ? 2 - u : u;
			float dpx = r*cos(t) * size * 1.0;
			float dpy = r*sin(t) * size * 1.0;
		

			cyPoint4f point = cyPoint4f(dpx, dpy, 0, 1);
			cyPoint3f result = cyPoint3f(transform * point);

			Ray toLight;
			toLight.p = initialRay.p;
			toLight.dir = result - p;

			shadowSum += Shadow({ toLight, Ray(), Ray() }, tid, 1);

			if ( (i == (MIN_SHADOW_SAMPLES - 1)) && (shadowSum > 0.0) && (shadowSum < MIN_SHADOW_SAMPLES)) {
				totalSamples = MAX_SHADOW_SAMPLES;
			}
		}
		return (shadowSum / (float)totalSamples) * intensity * (1.0 / dist);
	}
	else {
		return Shadow( Rays(initialRay, Ray(), Ray() ), tid, 1.0) *intensity * (1.0 / dist);
	}
}

/* Note, does not take size into account. */
Ray PointLight::GenerateRandomPhoton(int tid) const {
	double theta = 2 * M_PI * (rand() / (float)RAND_MAX);
	double phi = acos(1 - 2 * (rand() / (float)RAND_MAX));
	
	return Ray(position, cyPoint3f(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi)));
}