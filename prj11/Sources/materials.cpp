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

cyPoint3f getAltered(const cyPoint3f &originalDir, const float radius, bool hemisphere = false) {
	int random = rand() >> 1;

	float x, y, z;
	do {
		x = Halton(random, 2) * radius - (radius * 0.5);
		y = Halton(random, 3) * radius - (radius * 0.5);
		z = Halton(random, 5) * radius - (radius * 0.5);
	} while (((x * x) + (y * y) + (z * z)) > (radius * radius));

	cyPoint3f alter = cyPoint3f(x, y, z);

	/* If we're using a hemisphere to alter our ray, and the ray dir is going 
		a different direction than the alter ray, that alter ray is outside the hemisphere. 
		To fix, just negate the alter ray. */
	if (hemisphere && ((alter % originalDir) < 0)) alter *= -1;

	return (originalDir + alter).GetNormalized();
}

Rays getAltered(const Rays& originalRays, const float radius, bool hemisphere = false) {
	int random = rand() >> 1;

	float x, y, z;
	do {
		int random = rand() >> 1;
		x = (2.0 * Halton(random, 2) - 1.0) * radius;
		y = (2.0 * Halton(random, 3) - 1.0) * radius;
		z = (2.0 * Halton(random, 5) - 1.0) * radius;
	} while (((x * x) + (y * y) + (z * z)) > (radius * radius));

	cyPoint3f alter = cyPoint3f(x, y, z);

	/* If we're using a hemisphere to alter our ray, and the ray dir is going
	a different direction than the alter ray, that alter ray is outside the hemisphere.
	To fix, just negate the alter ray. */
	if (hemisphere && ((alter % originalRays.mainRay.dir) < 0)) alter *= -1;


	Rays newRays;
	newRays.difx = Ray(originalRays.difx.p, (originalRays.difx.dir + alter).GetNormalized());
	newRays.dify = Ray(originalRays.dify.p, (originalRays.dify.dir + alter).GetNormalized());
	newRays.mainRay = Ray(originalRays.mainRay.p, (originalRays.mainRay.dir + alter).GetNormalized());

	return newRays;
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

cyPoint3f CosineSampleHemisphere(cyPoint3f N)
{
	cyPoint3f from = cyPoint3f(0, 0, 1);
	cyPoint3f to = N;
	cyMatrix4f transform = cyMatrix4f::MatrixRotation(from, to);

	int i = rand() >> 1;
	float t = 2 * M_PI*Halton(i, 2);
	float u = Halton(i, 3) + Halton(i, 5);
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

Color MtlBlinn::ShadeDirect(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const {
	Color directCol = cyColor::Black();
	
	/* For each direct light in the scene */
	if (bounceCount >= 0)
		for (int i = 0; i < lights.size(); i++) {
			Light *l = lights.at(i);

			/* Try to illuminate this point using the light */
			Color li = l->Illuminate(hInfo.p, hInfo.N);

			/* Always add ambient */
			if (l->IsAmbient()) { directCol += diffuse.Sample(hInfo.uvw, hInfo.duvw) * li; }
			else {
				/* Do Blinn */
				cyPoint3f ldir = -l->Direction(hInfo.p);
				float dot = (hInfo.N.GetNormalized() % ldir);

				/* If surface is facing the light */
				if (dot >= 0) {
					cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
					directCol += li * dot * (diffuse.Sample(hInfo.uvw, hInfo.duvw) + (specular.Sample(hInfo.uvw, hInfo.duvw) * pow(hInfo.N % H, glossiness)));
				}
			}
		}

	return directCol;
}

//Color MtlBlinn::ShadeIndirect(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const {
//	Color indirectCol = Color::Black();
//	
//	/* Get random ray from surface point */
//	Rays indirectRays = Rays();
//	indirectRays.mainRay = Ray(hInfo.p, getPointInHemisphere(hInfo.N));
//	indirectRays.difx = Ray();
//	indirectRays.dify = Ray();
//
//	if (bounceCount > 0) {
//		/* Trace that ray */
//		HitInfo indirectInfo = HitInfo();
//		Trace(indirectRays, rootNode, indirectInfo, HIT_FRONT_AND_BACK);
//
//		/* If we hit something*/
//		if (indirectInfo.node != NULL) {
//			/* Illumate the indirecly hit point */
//			const Material *hitMat = indirectInfo.node->GetMaterial();
//			Color lightColor = hitMat->Shade(indirectRays, indirectInfo, lights, bounceCount - 1);
//
//			/* Treat that point as a light source */
//			PointLight l = PointLight();
//			l.SetPosition(indirectInfo.p);
//			l.SetIntensity(lightColor);
//
//			/* Shade using that point light */
//			Color li = l.Illuminate(hInfo.p, hInfo.N.GetNormalized());
//
//			/* Do Blinn */
//			cyPoint3f ldir = (indirectInfo.p - hInfo.p).GetNormalized();
//			float dot = (hInfo.N.GetNormalized() % ldir);
//
//			/* If surface is facing the light */
//			if (dot >= 0) {
//				indirectCol = li * dot * (diffuse.Sample(indirectInfo.uvw, indirectInfo.duvw));
//				//cyPoint3f H = ((ldir + -indirectRays.mainRay.dir) / (ldir + -indirectRays.mainRay.dir).Length()).GetNormalized();
//				//indirectCol = li * dot * (diffuse.Sample(indirectInfo.uvw, indirectInfo.duvw) + (specular.Sample(indirectInfo.uvw, indirectInfo.duvw) * pow(indirectInfo.N % H, glossiness)));
//			}
//		}
//		else {
//			indirectCol = environment.SampleEnvironment(rays.mainRay.dir.GetNormalized());
//		}
//	}
//	else {
//		indirectCol = environment.SampleEnvironment(indirectRays.mainRay.dir.GetNormalized());
//	}
//	return indirectCol;
//}

Color MtlBlinn::ShadeIndirect(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const {
	Color indirectCol = Color::Black();

#if INDIRECT_SAMPLES > 0
	if (bounceCount == 0) return environment.SampleEnvironment(hInfo.N);

	int samples = 1;
	if (bounceCount == TOTAL_BOUNCES) samples = INDIRECT_SAMPLES;
		
	bounceCount--;
	for (int i = 0; i < samples; ++i) {
		if (bounceCount >= 0) {
			Rays fromPoint = Rays();
			fromPoint.mainRay = Ray(hInfo.p, (CosineSampleHemisphere(hInfo.N)).GetNormalized());

			HitInfo fromPointInfo = HitInfo();
			Trace(fromPoint, rootNode, fromPointInfo, HIT_FRONT_AND_BACK);

			if (fromPointInfo.node == NULL) {
				indirectCol += environment.SampleEnvironment(fromPoint.mainRay.dir);
				continue;
			}
			Color intensity = fromPointInfo.node->GetMaterial()->Shade(fromPoint, fromPointInfo, lights, bounceCount);
			cyPoint3f ldir = (fromPointInfo.p - hInfo.p).GetNormalized();

			/* Do Blinn */
			float dot = (hInfo.N.GetNormalized() % ldir);

			/* If surface is facing the light */
			if (dot >= 0) {
				//cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
				indirectCol += intensity * dot * (diffuse.Sample(hInfo.uvw, hInfo.duvw));// +(specular.Sample(hInfo.uvw, hInfo.duvw) * pow(hInfo.N % H, glossiness)));
			}

		}
	}

	//indirectCol /= (float)samples;
#endif

	return indirectCol;
}

Color MtlBlinn::ShadeReflection(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &fkr, Color absorped) const {
	/* If the material has no reflection, return black */
	if (bounceCount <= 0 || (reflection.GetColor() == cyColor::Black() && fkr <= 0)) 	
		return cyColor::Black();
	
	/* Compute the reflection along the surface */
	cyPoint3f N = getAltered(hInfo.N, reflectionGlossiness);
	Rays reflRays = reflect(rays, N, hInfo.p);
	bounceCount--;

	/* Trace that reflection through the scene */
	HitInfo reflInfo = HitInfo();
	Trace(reflRays, rootNode, reflInfo, HIT_FRONT_AND_BACK);

	Color temp = (reflection.Sample(reflInfo.uvw, reflInfo.duvw) + fkr) * absorped;
		
	/* If we didn't hit something, sample the environment and return. */
	if (reflInfo.node == NULL)
		return environment.SampleEnvironment(reflRays.mainRay.dir.GetNormalized()) * temp;

	/* Sample the reflective material */
	const Material *hitMat = reflInfo.node->GetMaterial();
	return hitMat->Shade(reflRays, reflInfo, lights, bounceCount) * temp;
}

Color MtlBlinn::ShadeRefraction(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount, float &fkr, Color absorped) const {
	Color refrCol = Color::Black();

	/* If the material has refraction */
	if (refraction.GetColor() != cyColor::Black()) {

		/* Refract the ray through the volume */
		bool internallyReflected = false;
		cyPoint3f N = getAltered(hInfo.N, refractionGlossiness);
		Rays refrRays = refract(rays, N, hInfo.p, ior, hInfo.front, internallyReflected);
		bounceCount--;

		/* Also compute how much of the ray was reflected due to the frensel effect. */
		frensel(rays.mainRay.dir, N, ior, hInfo.front, fkr);

		/* If a ray is internally reflected, it does not refract, so we don't trace it's refraction. */
		if (!internallyReflected) {
			HitInfo refrInfo = HitInfo();
			Trace(refrRays, rootNode, refrInfo, HIT_FRONT_AND_BACK);

			/* If we hit something */
			if (refrInfo.node != NULL) {
				const Material *hitMat = refrInfo.node->GetMaterial();
				Color finalRefr = (internallyReflected) ? Color::Black() : (refraction.Sample(refrInfo.uvw, refrInfo.duvw) - fkr) * absorped;
				finalRefr.r = MAX(finalRefr.r, 0); finalRefr.g = MAX(finalRefr.g, 0); finalRefr.b = MAX(finalRefr.b, 0);
				refrCol += hitMat->Shade(refrRays, refrInfo, lights, bounceCount) * finalRefr;
			}

			/* Else sample from the environment map */
			else {
				refrCol += environment.SampleEnvironment(refrRays.mainRay.dir.GetNormalized()) * (refraction.Sample(refrInfo.uvw, refrInfo.duvw) - fkr) * absorped;
			}
		}
	}

	return refrCol;
}

Color MtlBlinn::Shade(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const
{
	if (bounceCount <= 0) return Color::Black();
	Color directCol = Color::Black(), indirectCol = Color::Black(), reflCol = Color::Black(), refrCol = Color::Black();
	Color absorped = getAbsorption(hInfo, rays, absorption);
	float frenselkr = 0.f;
		
	/* L(x, wo) = Lemmitted(x, wo) + integral( fs(wi, wo) * L(x', wi) * (N dot wi) * V(x, x') dwi ) */
	refrCol = ShadeRefraction(rays, hInfo, lights, bounceCount, frenselkr, absorped);
	reflCol = ShadeReflection(rays, hInfo, lights, bounceCount, frenselkr, absorped);
	directCol = ShadeDirect(rays, hInfo, lights, bounceCount);
	indirectCol = ShadeIndirect(rays, hInfo, lights, bounceCount);
	
	return (directCol + indirectCol + reflCol + refrCol);
}
