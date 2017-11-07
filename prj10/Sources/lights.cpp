#include "lights.h"
#include "render.h"
#include "options.h"
#include <time.h>
#include "cyCodeBase/cyMatrix.h"
extern Node rootNode;


float GenLight::Shadow(Rays rays, float t_max) {
	if (DISABLE_SHADOWS) return 1;
	HitInfo hInfo = HitInfo();
	hInfo.shadow = EARLY_SHADOW_TERMINATION;
	Trace(rays, rootNode, hInfo, HIT_FRONT_AND_BACK, t_max);

	if ((hInfo.node != nullptr) && hInfo.z <= t_max) {
		return 0.;
	}
	return 1;
}

Color PointLight::Illuminate(const Point3 &p, const Point3 &N) const {
	if (MAX_SHADOW_SAMPLES == 0) return intensity;

	Ray initialRay = Ray(p, position - p);

	if (ENABLE_SOFT_SHADOWS) {

		int random = rand() % 100;

		cyPoint3f from = cyPoint3f(0, 0, 1); 
		cyPoint3f to = initialRay.dir.GetNormalized();
		cyMatrix4f transform = cyMatrix4f::MatrixTrans(position) * cyMatrix4f::MatrixRotation(from, to);

		int totalSamples = MIN_SHADOW_SAMPLES;
	
		float shadowSum = 0;
		for (int i = 0; i < totalSamples; ++i) {
			float t = 2 * M_PI*Halton(i + random, 2);
			float u = Halton(i + random, 3) + Halton(i + random, 3);
			float r = (u > 1.0) ? 2 - u : u;
			float dpx = r*cos(t) * size * 2.0;
			float dpy = r*sin(t) * size * 2.0;
		

			cyPoint4f point = cyPoint4f(dpx, dpy, 0, 1);
			cyPoint3f result = cyPoint3f(transform * point);

			Ray toLight;
			toLight.p = initialRay.p;
			toLight.dir = result - p;

			shadowSum += Shadow({ toLight, Ray(), Ray() }, 1);

			if ( (i == (MIN_SHADOW_SAMPLES - 1)) && (shadowSum > 0.0) && (shadowSum < MIN_SHADOW_SAMPLES)) {
				totalSamples = MAX_SHADOW_SAMPLES;
			}
		}
		return (shadowSum / (float)totalSamples) * intensity;
	}
	else {
		return Shadow({ initialRay, Ray(), Ray() }, 1) *intensity;
	}
}
