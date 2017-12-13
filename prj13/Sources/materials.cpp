#include "materials.h"
#include "render.h"
#include "lights.h"
#include <math.h>
#include "trace.h"

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

Color getAbsorption(const HitInfo &hInfo, const Ray &ray, Color absorption) {
	Color absorped = Color::White();

	/* Compute how much light is absorped by the material. */
	if (!hInfo.front && absorption != Color::Black()) {
		float h = (hInfo.p - ray.p).Length();
		absorped.r = pow(E, -h * absorption.r);
		absorped.g = pow(E, -h * absorption.g);
		absorped.b = pow(E, -h * absorption.b);
	}

	return absorped;
}

Color MtlBlinn::ShadeDirect(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const {
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
			directCol = hitMat->Shade(newRays, transpInfo, lights, bounceCount, statistics, directType, indirectType);
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

Color MtlBlinn::ShadeIndirectPathTracing(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const {
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
		Color intensity = fromPointInfo.node->GetMaterial()->Shade(fromPoint, fromPointInfo, lights, bounceCount, statistics, directType, indirectType);
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

Color MtlBlinn::ShadeReflection(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &fkr, Color absorped, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const {
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
	return hitMat->Shade(reflRays, reflInfo, lights, bounceCount, statistics, directType, indirectType) * Color(temp);
}

Color MtlBlinn::ShadeRefraction(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &fkr, Color absorped, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const {
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
				Color finalRefr = (internallyReflected) ? Color::Black() : Color((refraction.Sample(refrInfo.uvw)/* - fkr*/) * ColorA(absorped));
#endif
				finalRefr.r = MAX(finalRefr.r, 0); finalRefr.g = MAX(finalRefr.g, 0); finalRefr.b = MAX(finalRefr.b, 0);
				refrCol += hitMat->Shade(refrRays, refrInfo, lights, bounceCount, statistics, directType, indirectType) * finalRefr;
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

Color MtlBlinn::ShadeIndirectPhotonMapping(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics) const {
	Color photon_col = Color::Black();
	if (USE_PHOTON_MAPPING) {

		if (ContainsDiffuse()) {
			ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
			ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(hInfo.uvw, hInfo.duvw) : specular.Sample(hInfo.uvw);

			if (USE_SPPM) {
				/* Saving photon color elsewhere for post normalization*/
				Color photon_col = Color::Black();
				statistics.totalHits++;
				cyPoint3f direction;
				Color tm = Color::Black();
				int nx = statistics.SharedLocalPhotonCount; int mx = 0;
				photonMap->EstimateProgressiveIrradiance<MAX_PHOTON_SAMPLES>(tm, direction, statistics.R, nx, mx, hInfo.p);
				direction.Normalize();
				if (mx > 0) {
					statistics.additionalPhotons += mx;
					statistics.additionalFlux += tm;
				}
				photon_col = ((statistics.SharedLocalPhotonCount == 0) ? tm : statistics.totalFlux) * Color(dMat);

				float area = (float)M_PI*statistics.R*statistics.R;
				photon_col = (photon_col * PHOTON_SCALE) / (statistics.totalPhotons * area);
				statistics.additionalIndirectContribution += photon_col;
			}
			else {
				cyPoint3f direction;
				Color illumination;
				photonMap->EstimateIrradiance<MAX_PHOTON_SAMPLES>(illumination, direction, PHOTON_SPHERE_RADIUS, hInfo.p);

				cyPoint3f ldir = direction;
				float dot = (hInfo.N.GetNormalized() % ldir.GetNormalized());

				cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
				photon_col = Color(ColorA(illumination) * (dMat + sMat * pow(hInfo.N % H, glossiness)));
			}
		}
	}
	return photon_col;
}

Color MtlBlinn::ShadeCausticRefraction(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics) const {
	Color photon_col = Color::Black();
	if (USE_CAUSTIC_REFRACTIONS) {

		if (ContainsDiffuse()) {
			ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
			ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(hInfo.uvw, hInfo.duvw) : specular.Sample(hInfo.uvw);

			if (USE_SPPM) {
				/* Saving photon color elsewhere for post normalization*/
				Color photon_col = Color::Black();
				statistics.totalRefrHits++;
				cyPoint3f direction;
				Color tm = Color::Black();
				int nx = statistics.SharedLocalRefractivePhotonCount; int mx = 0;
				refractionMap->EstimateProgressiveIrradiance<MAX_CAUSTIC_REFRACTION_SAMPLES>(tm, direction, statistics.RRefr, nx, mx, hInfo.p);
				direction.Normalize();
				if (mx > 0) {
					statistics.additionalRefractivePhotons += mx;
					statistics.additionalRefractiveFlux += tm;
				}
				photon_col = ((statistics.SharedLocalRefractivePhotonCount == 0) ? tm : statistics.totalRefractiveFlux) * Color(dMat);

				float area = (float)M_PI*statistics.RRefr*statistics.RRefr;
				photon_col = (photon_col * CAUSTIC_REFRACTION_SCALE) / (statistics.totalRefractivePhotons * area);
				statistics.additionalIndirectRefractiveContribution += photon_col;
			}
			else {
				cyPoint3f direction;
				Color illumination;
				reflectionMap->EstimateIrradiance<MAX_CAUSTIC_REFRACTION_SAMPLES>(illumination, direction, REFRACTION_SPHERE_RADIUS, hInfo.p);

				cyPoint3f ldir = direction;
				float dot = (hInfo.N.GetNormalized() % ldir.GetNormalized());

				cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
				photon_col = Color(ColorA(illumination) * (dMat + sMat * pow(hInfo.N % H, glossiness)));
			}
		}
	}
	return photon_col;
}

Color MtlBlinn::ShadeCausticReflection(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, PixelStatistics &statistics) const {
	Color photon_col = Color::Black();
	if (USE_CAUSTIC_REFLECTIONS) {

		if (ContainsDiffuse()) {
			ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(hInfo.uvw, hInfo.duvw) : diffuse.Sample(hInfo.uvw);
			ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(hInfo.uvw, hInfo.duvw) : specular.Sample(hInfo.uvw);

			if (USE_SPPM) {
				/* Saving photon color elsewhere for post normalization*/
				Color photon_col = Color::Black();
				statistics.totalReflHits++;
				cyPoint3f direction;
				Color tm = Color::Black();
				int nx = statistics.SharedLocalReflectivePhotonCount; int mx = 0;
				reflectionMap->EstimateProgressiveIrradiance<MAX_CAUSTIC_REFLECTION_SAMPLES>(tm, direction, statistics.RRefl, nx, mx, hInfo.p);
				direction.Normalize();
				if (mx > 0) {
					statistics.additionalReflectivePhotons += mx;
					statistics.additionalReflectiveFlux += tm;
				}
				photon_col = ((statistics.SharedLocalReflectivePhotonCount == 0) ? tm : statistics.totalReflectiveFlux) * Color(dMat);

				float area = (float)M_PI*statistics.RRefl*statistics.RRefl;
				photon_col = (photon_col * CAUSTIC_REFLECTION_SCALE) / (statistics.totalReflectivePhotons * area);
				statistics.additionalIndirectReflectiveContribution += photon_col;
			}
			else {
				cyPoint3f direction;
				Color illumination;
				reflectionMap->EstimateIrradiance<MAX_CAUSTIC_REFLECTION_SAMPLES>(illumination, direction, REFLECTION_SPHERE_RADIUS, hInfo.p);

				cyPoint3f ldir = direction;
				float dot = (hInfo.N.GetNormalized() % ldir.GetNormalized());

				cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
				photon_col = Color(ColorA(illumination) * (dMat + sMat * pow(hInfo.N % H, glossiness)));
			}
		}
	}
	return photon_col;
}

Color MtlBlinn::Shade(const Rays &rays, const HitInfo &hInfo, const LightList &lights, 
	int bounceCount, PixelStatistics &statistics, IlluminationType directType, IlluminationType indirectType) const
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
	refractionColor = ShadeRefraction(rays, hInfo, lights, bounceCount, frenselkr, absorped, statistics, directType, indirectType);
	reflectionColor = ShadeReflection(rays, hInfo, lights, bounceCount, frenselkr, absorped, statistics, directType, indirectType);

	if (directType == PATH_TRACING)
		directColor = ShadeDirect(rays, hInfo, lights, bounceCount, statistics, directType, indirectType);

	if (indirectType != NO_ILLUMINATION) {
		if (indirectType == PATH_TRACING || (TOTAL_BOUNCES - bounceCount < INITIAL_PATH_TRACES))
			//(USE_PHOTON_MAPPING && bounceCount == TOTAL_BOUNCES && INDIRECT_SAMPLES != 0) || !USE_PHOTON_MAPPING)
			indirectColor = ShadeIndirectPathTracing(rays, hInfo, lights, bounceCount, statistics, directType, indirectType);
		else if (indirectType == PHOTON) {
			indirectColor = ShadeIndirectPhotonMapping(rays, hInfo, lights, bounceCount, statistics);
			indirectColor += ShadeCausticReflection(rays, hInfo, lights, bounceCount, statistics);
			indirectColor += ShadeCausticRefraction(rays, hInfo, lights, bounceCount, statistics);
		}
	}
	
	return (directColor + indirectColor + reflectionColor + refractionColor);
}

Color MtlBlinn::ShadePoint(HitInfo &info, PixelStatistics &stats) const {
	ColorA dMat = (USE_RAY_DIFFERENTIALS) ? diffuse.Sample(info.uvw, info.duvw) : diffuse.Sample(info.uvw);
	ColorA sMat = (USE_RAY_DIFFERENTIALS) ? specular.Sample(info.uvw, info.duvw) : specular.Sample(info.uvw);
	
	Color photon_col = Color::Black(), illumination;
	/*if (USE_PHOTON_MAPPING) {
		cyPoint3f direction;
		Color tm = Color::Black();
		int nx = stats.SharedLocalPhotonCount; int mx = 0;
		photonMap->EstimateProgressiveIrradiance<MAX_PHOTON_SAMPLES>(tm, direction, stats.R, nx, mx, info.p);
		direction.Normalize();
		if (mx > 0) {
			stats.additionalPhotons += mx;
			stats.additionalFlux += tm;
		}
		photon_col = ((stats.SharedLocalPhotonCount == 0) ? tm : stats.totalFlux) * Color(dMat);
	}

	float area = (float)M_PI*stats.R*stats.R;
	photon_col = (photon_col * PHOTON_SCALE) / (stats.totalPhotons * area);*/
	
	stats.additionalIndirectContribution += photon_col;

	///////////////////////////////////////////

	Color directCol = cyColor::Black();

	/* For each direct light in the scene */
	for (int i = 0; i < lights.size(); i++) {
		Light *l = lights.at(i);

		/* Try to illuminate this point using the light */
		Color li = l->Illuminate(info.p, info.N, info.tid);

		/* Always add ambient, although with photon mapping this shouldn't be a thing */
		if (l->IsAmbient()) {
			directCol += Color(dMat * ColorA(li));
		}
		else {
			/* Do Blinn */
			cyPoint3f ldir = -l->Direction(info.p);
			float dot = (info.N.GetNormalized() % ldir);

			/* If surface is facing the light */
			if (dot > 0 && !isnan(dot)) {
				cyPoint3f H = ((ldir + -info.d) / (ldir + -info.d).Length()).GetNormalized();
				directCol += Color(ColorA(li) * dot * (dMat + (sMat * pow(info.N % H, glossiness))));
			}
		}
	}
	stats.additionalDirectContribution += directCol * info.pointInfluence;

	return (directCol + photon_col) * (info.pointInfluence);
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
	Color absorped = getAbsorption(hInfo, bInfo.ray, absorption);

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
		*/ // I BROKE THIS! FIX IT
		if ( (bInfo.mapType == MapType::GLOBAL_ILLUMINATION && (bInfo.bounceType == BounceType::DIFFUSE || bInfo.bounceType == BounceType::NONE) )
			|| (bInfo.mapType == MapType::CAUSTIC_REFRACTIONS && bInfo.bounceType == BounceType::REFRACTIVE)
			|| (bInfo.mapType == MapType::CAUSTIC_REFLECTIONS && bInfo.bounceType == BounceType::REFLECTIVE)) {
			photonMutex.lock();
			if (SAVE_DIRECT_PHOTON || bInfo.directHit == true) {
				bInfo.map->AddPhoton(hInfo.p, (bInfo.ray.p - hInfo.p).GetNormalized(), bInfo.power);
				bInfo.photonRecorded = true;
			}
			bInfo.directHit = true;
			photonMutex.unlock();
			
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
		bInfo.power = (bInfo.power * refr * absorped) / pr;
		bInfo.ray = refract(bInfo.ray, hInfo.N, hInfo.p, ior, hInfo.front, internRefl);
		bInfo.bounceType = BounceType::REFRACTIVE;
		return true;
	}
	else {
		bInfo.bounceType = BounceType::ABSORPED;
		return false;
	}
}

bool MtlBlinn::containsNonspecular() const {
	if (Color(diffuse.GetColor()) != Color::Black()) return true;
}

void MtlBlinn::GetDiffuseHits(std::vector<HitInfo>& hits, Ray ray, HitInfo hInfo, int bounceCount) const
{
	if (bounceCount <= 0) return;
	bounceCount--;

	if (ContainsDiffuse()) {
		if (isTransparent(hInfo)) {
			HitInfo info = HitInfo(hInfo.tid);
			Ray newRay = Ray(ray, info.z);
			bool hit = Trace(newRay, rootNode, info, HIT_FRONT_AND_BACK);
			info.pointInfluence = hInfo.pointInfluence;
			if (hit) info.node->GetMaterial()->GetDiffuseHits(hits, newRay, info, bounceCount);
		}
		else {
			hInfo.p0 = ray.p;
			hits.push_back(hInfo);
		}
	}
	Color absorped = getAbsorption(hInfo, ray, absorption);
	
	float fkr;
	if (ContainsRefraction()) {
		HitInfo refrinfo = HitInfo(hInfo.tid);
		Ray refrRay;
		bool refrhit = TraceRefraction(ray, refrRay, hInfo, refrinfo, fkr);
		refrinfo.pointInfluence = hInfo.pointInfluence;

		HitInfo reflinfo = HitInfo(hInfo.tid);
		Ray reflRay;
		bool reflhit = TraceReflection(ray, reflRay, hInfo, reflinfo, HIT_FRONT_AND_BACK);
		reflinfo.pointInfluence = hInfo.pointInfluence;


		refrinfo.pointInfluence *= Color(refraction.Sample(hInfo.uvw) - fkr) * absorped;
		reflinfo.pointInfluence *= (Color(reflection.Sample(hInfo.uvw)) + fkr);
		if (!hInfo.front) reflinfo.pointInfluence *= absorped;
		
		if (refrhit) {
			refrinfo.node->GetMaterial()->GetDiffuseHits(hits, refrRay, refrinfo, bounceCount);
		}

		if (reflhit) {
			reflinfo.node->GetMaterial()->GetDiffuseHits(hits, reflRay, reflinfo, bounceCount);
		}
	}
	else if (ContainsReflection()) {
		HitInfo info = HitInfo(hInfo.tid);
		Ray reflRay;
		bool hit = TraceReflection(ray, reflRay, hInfo, info, HIT_FRONT_AND_BACK);
		info.pointInfluence = hInfo.pointInfluence;
		if (hit) {
			//info.pointInfluence *= (Color(reflection.Sample(hInfo.uvw)));
			if (info.pointInfluence.Gray() > .001)
				info.node->GetMaterial()->GetDiffuseHits(hits, reflRay, info, bounceCount);
		}
	}
}