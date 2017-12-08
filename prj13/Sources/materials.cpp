#include "materials.h"
#include "render.h"
#include "lights.h"
#include <math.h>

extern Camera camera;
extern RenderImage renderImage;
extern Node rootNode;
extern MaterialList materials;
extern LightList lights;
extern TexturedColor background;
extern TexturedColor environment;
extern cy::PhotonMap *photonMap;
extern cy::PhotonMap *refractionMap;
extern cy::PhotonMap *reflectionMap;

extern HaltonIDX haltonIDX[TOTAL_THREADS];

#define E 2.71828182845904523536f

Rays refract(const Rays &rays/*const cyPoint3f &I*/, const cyPoint3f &N, const cyPoint3f &P, const float &ior, bool front, bool &internRefl) {
	// T = nI + (nc1 - c2)N
	// n = iorV / iorT
	// c1 = N % I
	// c2 = sqrt(1 - pow(n,2) * (1 - pow(c1, 2))), if imaginary => internal reflection
	
	float cosi = MAX(-1.f, MIN(N % rays.mainRay.dir, 1.f));
	float iorV = 1.f, iorT = ior;

	cyPoint3f N_ = N;
	if (front) {
		// Hitting a front face, make sure NdotI is positive
		cosi = -cosi;
	}
	else {
		// Hitting back face, reverse surface normal, and swap iors
		N_ = -N;
		std::swap(iorV, iorT);
	}

	float n = iorV / iorT;
	float c2squared = 1.f - n * n * (1.f - cosi * cosi);

	Rays Ts;
	if (c2squared < 0.f) {
		internRefl = true;
	}
	else { 
		cyPoint3f temp = (n * cosi - sqrtf(c2squared)) * N_;
		internRefl = false;
		Ts.mainRay.p = P;
		Ts.mainRay.dir = n * rays.mainRay.dir + temp;
		Ts.mainRay.dir_inv = invertPoint(Ts.mainRay.dir);
		
		Ts.difx.p = P;
		Ts.difx.dir = n * rays.difx.dir + temp;
		//Ts.difx.dir_inv = invertPoint(Ts.difx.dir);

		Ts.dify.p = P;
		Ts.dify.dir = n * rays.dify.dir + temp;
		//Ts.dify.dir_inv = invertPoint(Ts.dify.dir);

	}

	return Ts;
}

Ray refract(const Ray &ray, const cyPoint3f &N, const cyPoint3f &P, const float &ior, bool front, bool &internRefl) {
	// T = nI + (nc1 - c2)N
	// n = iorV / iorT
	// c1 = N % I
	// c2 = sqrt(1 - pow(n,2) * (1 - pow(c1, 2))), if imaginary => internal reflection

	float cosi = MAX(-1.f, MIN(N % ray.dir, 1.f));
	float iorV = 1.f, iorT = ior;

	cyPoint3f N_ = N;
	if (front) {
		// Hitting a front face, make sure NdotI is positive
		cosi = -cosi;
	}
	else {
		// Hitting back face, reverse surface normal, and swap iors
		N_ = -N;
		std::swap(iorV, iorT);
	}

	float n = iorV / iorT;
	float c2squared = 1.f - n * n * (1.f - cosi * cosi);

	Ray Ts;
	if (c2squared < 0.f) {
		internRefl = true;
	}
	else {
		cyPoint3f temp = (n * cosi - sqrtf(c2squared)) * N_;
		internRefl = false;
		Ts.p = P;
		Ts.dir = n * ray.dir + temp;
		Ts.dir_inv = invertPoint(Ts.dir);
	}

	return Ts;
}

void frensel(const cyPoint3f &I, const cyPoint3f &N, const float &ior, bool front, float &kr) {
	float cosi = MAX(-1.f, MIN(N % I, 1.f));
	float iorV = 1.f, iorT = ior;

	if (!front) std::swap(iorV, iorT);
	float sint = iorV / iorT * sqrtf(MAX(0.f, 1.f - cosi * cosi));
	if (sint >= 1.f) kr = 1.f; // Total internal reflection.
	else {
		float cost = sqrtf(MAX(0.f, 1.f - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((iorT * cosi) - (iorV * cost)) / ((iorT * cosi) + (iorV * cost));
		float Rp = ((iorV * cosi) - (iorT * cost)) / ((iorV * cosi) + (iorT * cost));
		kr = (Rs * Rs + Rp * Rp) / 2.f;
	}
}

cyPoint3f getAltered(const cyPoint3f &originalDir, const float radius, int haltonIdx, bool hemisphere = false) {
	float x, y, z;
	do {
		x = Halton(haltonIdx, 2) * radius - (radius * 0.5);
		y = Halton(haltonIdx, 3) * radius - (radius * 0.5);
		z = Halton(haltonIdx, 5) * radius - (radius * 0.5);
	} while (((x * x) + (y * y) + (z * z)) > (radius * radius));

	cyPoint3f alter = cyPoint3f(x, y, z);

	/* If we're using a hemisphere to alter our ray, and the ray dir is going 
		a different direction than the alter ray, that alter ray is outside the hemisphere. 
		To fix, just negate the alter ray. */
	if (hemisphere && ((alter % originalDir) < 0)) alter *= -1;

	return (originalDir + alter).GetNormalized();
}

cyPoint3f getPointInSphere() {
	int random = rand() >> 1;
	float Radius = 1.0;

	float Angle = Halton(random, 2) * M_PI * 2.0;
	float Dist = Radius * sqrt(Halton(random, 3));
	cyPoint3f output;
	output.x = Dist * cos(Angle);
	output.y = Dist * sin(Angle);
	output.z = sqrt(Radius * Radius - Dist * Dist);
	if (Halton(random, 5) < 0.5) {
		output.z = -output.z;
	}
	return output;
}

cyPoint3f getPointInHemisphere(cyPoint3f normal) {
	cyPoint3f result = getPointInSphere();
	if ((result % normal)<0) {
		result = -1.0 * result;
	}
	return result;
}

cyPoint3f CosineSampleHemisphere(cyPoint3f N, int idx)
{
	cyPoint3f from = cyPoint3f(0, 0, 1);
	cyPoint3f to = N;
	cyMatrix4f transform = cyMatrix4f::MatrixRotation(from, to);

	float t = 2 * M_PI*Halton(idx, 2);
	float u = Halton(idx, 3) + Halton(idx, 5);
	float r = (u > 1.0) ? 2 - u : u;
	float dpx = r*cos(t);
	float dpy = r*sin(t);
	float dpz = sqrt(1.0 - (dpx * dpx) - (dpy * dpy));
	
	cyPoint4f point = cyPoint4f(dpx, dpy, dpz, 1);
	cyPoint3f result = cyPoint3f(transform * point);
	return result;
}

cyPoint3f diffuseReflection(cyPoint3f N) {
	//cyPoint3f dir;
	//do {
	//	do {
	//		dir.x = 2.0 * (rand() / (float)RAND_MAX) - 1.0;
	//		dir.y = 2.0 * (rand() / (float)RAND_MAX) - 1.0;
	//		dir.z = 2.0 * (rand() / (float)RAND_MAX) - 1.0;
	//	} while ((dir.x*dir.x + dir.y*dir.y + dir.z*dir.z) > 1.0);
	//	//dir = (dir - .5) * 2.0;
	//	dir.Normalize();
	//} while ((dir % N) <= 0.0);
	//return dir;


	cyPoint3f from = cyPoint3f(0, 0, 1);
	cyPoint3f to = N;
	cyMatrix4f transform = cyMatrix4f::MatrixRotation(from, to);

	float t = 2 * M_PI*(rand() / (float)RAND_MAX);
	float u = (rand() / (float)RAND_MAX) + (rand() / (float)RAND_MAX);
	float r = (u > 1.0) ? 2 - u : u;
	float dpx = r*cos(t);
	float dpy = r*sin(t);
	float dpz = sqrt(1.0 - (dpx * dpx) - (dpy * dpy));

	cyPoint4f point = cyPoint4f(dpx, dpy, dpz, 1);
	cyPoint3f result = cyPoint3f(transform * point);
	return result;
}

cyPoint3f getRandomRay(const cyPoint3f& hemisphereDirection) {
	float radius = 0.5;
	float x, y, z;
	int i = 0;
	int random = rand() >> 1;
	do {
		i++;
		x = (2.0 * Halton(random + i, 2) - 1.0) * radius;
		y = (2.0 * Halton(random + i, 3) - 1.0) * radius;
		z = (2.0 * Halton(random + i, 5) - 1.0) * radius;
	} while (((x * x) + (y * y) + (z * z)) > (radius * radius));

	cyPoint3f alter = cyPoint3f(x, y, z).GetNormalized();
	cyPoint3f dir = hemisphereDirection.GetNormalized();

	/* If we're using a hemisphere to alter our ray, and the ray dir is going
	a different direction than the alter ray, that alter ray is outside the hemisphere.
	To fix, just negate the alter ray. */
	if ((alter % dir) < 0.0) alter *= -1.0;
	return alter;
}

Ray reflect(const Ray &ray, const cyPoint3f &N, const cyPoint3f &P) {
	Ray newRay;
	auto temp = (2.f *  N * (ray.dir % N));
	newRay.p = P;
	newRay.dir = ray.dir - temp;
	return newRay;
}

Rays reflect(const Rays &rays, const cyPoint3f &N, const cyPoint3f &P) {
	Rays newRays;
	
	auto temp = (2.f *  N * (rays.mainRay.dir % N));

	newRays.mainRay.p = newRays.difx.p = newRays.dify.p = P;

	newRays.mainRay.dir = rays.mainRay.dir - temp;
	newRays.mainRay.dir_inv = invertPoint(newRays.mainRay.dir);
	newRays.difx.dir = rays.difx.dir - temp;
	//newRays.difx.dir_inv = invertPoint(newRays.difx.dir);
	newRays.dify.dir = rays.dify.dir - temp;
	//newRays.dify.dir_inv = invertPoint(newRays.dify.dir);

	return newRays;
}

Color getAbsorption(const HitInfo &hInfo, const Rays &rays, Color absorption) {
	Color absorped = Color::White();

	/* Compute how much light is absorped by the material. */
	if (!hInfo.front && absorption != Color::Black()) {
		float h = (hInfo.p - rays.mainRay.p).Length();
		absorped.r = pow(E, -h * absorption.r);
		absorped.g = pow(E, -h * absorption.g);
		absorped.b = pow(E, -h * absorption.b);
	}

	return absorped;
}





Color MtlBlinn::ShadeDirect(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, IlluminationType directType, IlluminationType indirectType) const {
	Color directCol = cyColor::Black();
	ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
	ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(hInfo.uvw, hInfo.duvw) : specular.Sample(hInfo.uvw);
	
	/* If the object we hit is transparent, we need to continue tracing. */
	if (dMat.a <= 0.5) {
		HitInfo transpInfo = HitInfo(hInfo.tid);
		Rays newRays = Rays(rays, hInfo.z + .1);
		bool hit = Trace(newRays, rootNode, transpInfo, HIT_FRONT_AND_BACK);
		if (hit) {
			const Material *hitMat = transpInfo.node->GetMaterial();
			directCol = hitMat->Shade(newRays, transpInfo, lights, bounceCount, directType, indirectType);
		}
		return directCol;
	}

	/* For each direct light in the scene */
	if (bounceCount >= 0)
		for (int i = 0; i < lights.size(); i++) {
			Light *l = lights.at(i);

			/* Try to illuminate this point using the light */
			Color li = l->Illuminate(hInfo.p, hInfo.N, hInfo.tid);

			/* Always add ambient */
			if (l->IsAmbient() && ((INDIRECT_SAMPLES==0) || bounceCount==0)) { 
				directCol += Color(dMat * ColorA(li));
			}
			else {
				/* Do Blinn */
				cyPoint3f ldir = -l->Direction(hInfo.p);
				float dot = (hInfo.N.GetNormalized() % ldir);

				/* If surface is facing the light */
				if (dot >= 0) {
					cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
					directCol += Color(ColorA(li) * dot * (dMat + (sMat * pow(hInfo.N % H, glossiness))));
				}
			}
		}

	return directCol;
}

Color MtlBlinn::ShadeIndirectPathTracing(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, IlluminationType directType, IlluminationType indirectType) const {
	Color indirectCol = Color::Black();

#if INDIRECT_SAMPLES > 0
	//if (bounceCount <= 0) {
	//	indirectCol = Color(environment.SampleEnvironment(hInfo.N));
	//	return indirectCol;
	//}

	int samples = 1;
	if (bounceCount == (TOTAL_BOUNCES - 1)) samples = INDIRECT_SAMPLES;
		
	for (int i = 0; i < samples; ++i) {

		Rays fromPoint = Rays();
		int idx = haltonIDX[hInfo.tid].GIidx(bounceCount);
		fromPoint.mainRay = Ray(hInfo.p, (CosineSampleHemisphere(hInfo.N, idx)).GetNormalized());

		HitInfo fromPointInfo = HitInfo(hInfo.tid);
		bool hit = Trace(fromPoint, rootNode, fromPointInfo, HIT_FRONT_AND_BACK);

		if (!hit) {
			float dot = (hInfo.N.GetNormalized() % fromPoint.mainRay.dir);
			indirectCol += Color(environment.SampleEnvironment(fromPoint.mainRay.dir));
			continue;
		}
		Color intensity = fromPointInfo.node->GetMaterial()->Shade(fromPoint, fromPointInfo, lights, bounceCount, directType, indirectType);
		cyPoint3f ldir = (fromPointInfo.p - hInfo.p).GetNormalized();

		/* Geometry term */
		float dot = (hInfo.N.GetNormalized() % ldir);

		/* If surface is facing the light */
		if (dot >= 0) {
#if USE_RAY_DIFFERENTIALS
			indirectCol += Color(ColorA(intensity) * dot * (diffuse.Sample(hInfo.uvw, hInfo.duvw)));
#else
			indirectCol += Color(ColorA(intensity) * dot * (diffuse.Sample(hInfo.uvw)));
#endif
			dot++;
		}
	}
#endif
	
#if INDIRECT_SAMPLES != 0
	return indirectCol / (float)samples;
#else
	return indirectCol;
#endif
}

Color MtlBlinn::ShadeReflection(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &fkr, Color absorped, IlluminationType directType, IlluminationType indirectType) const {
	/* If the material has no reflection, return black */
	if (bounceCount <= 0 || (reflection.GetColor() == cyColorA::Black() && fkr <= 0)) 	
		return cyColor::Black();
	
	/* Compute the reflection along the surface */
	int idx = haltonIDX[hInfo.tid].Reflidx(bounceCount);
	cyPoint3f N = getAltered(hInfo.N, reflectionGlossiness, idx);
	Rays reflRays = reflect(rays, N, hInfo.p);

	/* Trace that reflection through the scene */
	HitInfo reflInfo = HitInfo(hInfo.tid);
	Trace(reflRays, rootNode, reflInfo, HIT_FRONT_AND_BACK);

#if USE_RAY_DIFFERENTIALS
	ColorA temp = (reflection.Sample(reflInfo.uvw, reflInfo.duvw) + fkr) * ColorA(absorped);
#else
	ColorA temp = (reflection.Sample(reflInfo.uvw) + fkr) * ColorA(absorped);
#endif

	/* If we didn't hit something, sample the environment and return. */
	if (reflInfo.node == NULL)
		return Color(environment.SampleEnvironment(reflRays.mainRay.dir.GetNormalized()) * temp);

	/* Sample the reflective material */
	const Material *hitMat = reflInfo.node->GetMaterial();
	return hitMat->Shade(reflRays, reflInfo, lights, bounceCount, directType, indirectType) * Color(temp);
}

Color MtlBlinn::ShadeRefraction(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &fkr, Color absorped, IlluminationType directType, IlluminationType indirectType) const {
	Color refrCol = Color::Black();

	/* If the material has refraction */
	if (refraction.GetColor() != cyColorA::Black()) {

		/* Refract the ray through the volume */
		bool internallyReflected = false;
		int idx = haltonIDX[hInfo.tid].Refridx(bounceCount);
		cyPoint3f N = getAltered(hInfo.N, refractionGlossiness, idx);
		Rays refrRays = refract(rays, N, hInfo.p, ior, hInfo.front, internallyReflected);

		/* Also compute how much of the ray was reflected due to the frensel effect. */
		frensel(rays.mainRay.dir, N, ior, hInfo.front, fkr);

		/* If a ray is internally reflected, it does not refract, so we don't trace it's refraction. */
		if (!internallyReflected) {
			HitInfo refrInfo = HitInfo(hInfo.tid);
			Trace(refrRays, rootNode, refrInfo, HIT_FRONT_AND_BACK);

			/* If we hit something */
			if (refrInfo.node != NULL) {
				const Material *hitMat = refrInfo.node->GetMaterial();
#if USE_RAY_DIFFERENTIALS
				Color finalRefr = (internallyReflected) ? Color::Black() : Color((refraction.Sample(refrInfo.uvw, refrInfo.duvw) - fkr) * ColorA(absorped));
#else
				Color finalRefr = (internallyReflected) ? Color::Black() : Color((refraction.Sample(refrInfo.uvw) - fkr) * ColorA(absorped));
#endif
				finalRefr.r = MAX(finalRefr.r, 0); finalRefr.g = MAX(finalRefr.g, 0); finalRefr.b = MAX(finalRefr.b, 0);
				refrCol += hitMat->Shade(refrRays, refrInfo, lights, bounceCount, directType, indirectType) * finalRefr;
			}

			/* Else sample from the environment map */
			else {
#if USE_RAY_DIFFERENTIALS
				refrCol += Color(environment.SampleEnvironment(refrRays.mainRay.dir.GetNormalized()) * (refraction.Sample(refrInfo.uvw, refrInfo.duvw) - fkr) * ColorA(absorped));
#else
				refrCol += Color(environment.SampleEnvironment(refrRays.mainRay.dir.GetNormalized()) * (refraction.Sample(refrInfo.uvw) - fkr) * ColorA(absorped));
#endif
			}
		}
	}

	return refrCol;
}

Color MtlBlinn::ShadeIndirectPhotonMapping(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const {
	Color photon_col = Color::Black();
	if (USE_PHOTON_MAPPING) {
		cyPoint3f direction;
		Color illumination;
		photonMap->EstimateIrradiance<MAX_PHOTON_SAMPLES>(illumination, direction, PHOTON_SPHERE_RADIUS, hInfo.p);

		cyPoint3f ldir = direction;
		float dot = (hInfo.N.GetNormalized() % ldir.GetNormalized());

		ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
		ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(hInfo.uvw, hInfo.duvw) : specular.Sample(hInfo.uvw);

		cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
		photon_col = Color(ColorA(illumination) * (dMat + sMat * pow(hInfo.N % H, glossiness)));
	}
	return photon_col;
}

Color MtlBlinn::ShadeCausticRefraction(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const {
	Color refr_photon_col = Color::Black();
	if (USE_CAUSTIC_REFRACTIONS) {
		cyPoint3f direction;
		Color illumination;
		refractionMap->EstimateIrradiance<MAX_CAUSTIC_REFRACTION_SAMPLES>(illumination, direction, REFRACTION_SPHERE_RADIUS, hInfo.p);

		cyPoint3f ldir = direction;

		ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
		ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(hInfo.uvw, hInfo.duvw) : specular.Sample(hInfo.uvw);

	//	cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
		refr_photon_col = Color(ColorA(illumination) * (dMat/* + sMat * pow(hInfo.N % H, glossiness)*/));
	}
	return refr_photon_col;
}

Color MtlBlinn::ShadeCausticReflection(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const {
	Color refl_photon_col = Color::Black();
	if (USE_CAUSTIC_REFLECTIONS) {
		cyPoint3f direction;
		Color illumination;
		reflectionMap->EstimateIrradiance<MAX_CAUSTIC_REFLECTION_SAMPLES>(illumination, direction, REFLECTION_SPHERE_RADIUS, hInfo.p);

		cyPoint3f ldir = direction;

		ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
		ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(hInfo.uvw, hInfo.duvw) : specular.Sample(hInfo.uvw);

	//	cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
		refl_photon_col = Color(ColorA(illumination) * (dMat/* + sMat * pow(hInfo.N % H, glossiness)*/));
	}
	return refl_photon_col;
}

Color MtlBlinn::Shade(const Rays &rays, const HitInfo &hInfo, const LightList &lights, 
	int bounceCount, IlluminationType directType, IlluminationType indirectType) const
{
	if (bounceCount <= 0) return Color::Black();

	Color directColor = Color::Black();
	Color indirectColor = Color::Black();
	Color reflectionColor = Color::Black();
	Color refractionColor = Color::Black();
	Color absorped = getAbsorption(hInfo, rays, absorption);
	float frenselkr = 0.f;

	bounceCount--;

	/* L(x, wo) = Lemmitted(x, wo) + integral( fs(wi, wo) * L(x', wi) * (N dot wi) * V(x, x') dwi ) */
	refractionColor = ShadeRefraction(rays, hInfo, lights, bounceCount, frenselkr, absorped, directType, indirectType);
	reflectionColor = ShadeReflection(rays, hInfo, lights, bounceCount, frenselkr, absorped, directType, indirectType);

	if (directType == PATH_TRACING)
		directColor = ShadeDirect(rays, hInfo, lights, bounceCount, directType, indirectType);

	if (indirectType != NO_ILLUMINATION) {
		if (indirectType == PATH_TRACING || (TOTAL_BOUNCES - bounceCount < INITIAL_PATH_TRACES))
			//(USE_PHOTON_MAPPING && bounceCount == TOTAL_BOUNCES && INDIRECT_SAMPLES != 0) || !USE_PHOTON_MAPPING)
			indirectColor = ShadeIndirectPathTracing(rays, hInfo, lights, bounceCount, directType, indirectType);
		else if (indirectType == PHOTON) {
			indirectColor = ShadeIndirectPhotonMapping(rays, hInfo, lights, bounceCount);
			indirectColor += ShadeCausticReflection(rays, hInfo, lights, bounceCount);
			indirectColor += ShadeCausticRefraction(rays, hInfo, lights, bounceCount);
		}
	}
	
	return (directColor + indirectColor + reflectionColor + refractionColor);
}

bool MtlBlinn::isTransparent(const HitInfo &hInfo) const {
	ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
	return (dMat.a < .5);
}

extern int recordedPhotons;

float getProbability(Color power, Color matColor) {
	return MAX(power.r * matColor.r, MAX(power.g * matColor.g, power.b * matColor.b)) / MAX(power.r, MAX(power.g, power.g));
}

#include <thread>
#include <mutex>
std::mutex photonMutex;

// if this method returns true, a new photon with the given direction and color will be traced
bool MtlBlinn::BouncePhoton(BounceInfo &bInfo, const HitInfo &hInfo) const {
	/* Assume for now we're not recording a photon */
	bInfo.photonRecorded = false;

	/* Kill off dark photons */
	if (bInfo.power.Gray() < .001) return false;

	Color diff = Color(diffuse.GetColor());
	Color spec = Color(reflection.GetColor());
	Color refr = Color(refraction.GetColor());

	float pd = getProbability(bInfo.power, diff);
	float ps = getProbability(bInfo.power, spec);
	float pr = getProbability(bInfo.power, refr);

	float r = (rand() / (float)RAND_MAX) * (pd + ps + pr);
	float outof = MAX(pd + ps + pr, 1.0);
	pd /= outof; ps /= outof; pr /= outof; // correction for irregular material properties.

	/* DIFFUSE */
	if (r < pd) {
		/* If we're GI and either this isn't the first bounce or we're saving direct light, 
		OR we're CAUSTIC REFRACTIONS and the last ray was refractive
		OR we're CAUSTIC REFLECTIONS and the last ray was reflective
		*/
		if ( (bInfo.mapType == MapType::GLOBAL_ILLUMINATION && bInfo.bounceType == BounceType::DIFFUSE || (bInfo.bounceType == BounceType::NONE && SAVE_DIRECT_PHOTON))
			|| (bInfo.mapType == MapType::CAUSTIC_REFRACTIONS && bInfo.bounceType == BounceType::REFRACTIVE)
			|| (bInfo.mapType == MapType::CAUSTIC_REFLECTIONS && bInfo.bounceType == BounceType::REFLECTIVE)) {
			photonMutex.lock();
			bInfo.map->AddPhoton(hInfo.p, (bInfo.ray.p - hInfo.p).GetNormalized(), bInfo.power);
			photonMutex.unlock();
			bInfo.photonRecorded = true;
		}

		bInfo.power = (bInfo.power * diff) / pd;
		bInfo.ray.p = hInfo.p;
		int idx = haltonIDX[hInfo.tid].PMidx();
		bInfo.ray.dir = diffuseReflection(hInfo.N.GetNormalized());
		bInfo.bounceType = BounceType::DIFFUSE;
		return true;
	}
	/* REFLECTIVE */
	else if (/*r >= pd && */r < ps + pd) {
		bInfo.power = (bInfo.power * spec) / ps;
		bInfo.ray = reflect(bInfo.ray, hInfo.N, hInfo.p);
		bInfo.bounceType = BounceType::REFLECTIVE;
		return true;
	}
	/* REFRACTIVE */
	else if (/*r >= ps + pd && */r < ps + pd + pr) {
		bool internRefl;
		bInfo.power = (bInfo.power * refr) / pr;
		bInfo.ray = refract(bInfo.ray, hInfo.N, hInfo.p, ior, hInfo.front, internRefl);
		bInfo.bounceType = BounceType::REFRACTIVE;
		return true;
	}
	else {
		bInfo.bounceType = BounceType::ABSORPED;
		return false;
	}
}
