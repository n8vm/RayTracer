#include "materials.h"
#include "render.h"
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
	
	float cosi = MAX(-1.f, MIN(N % rays.centerRay.dir, 1.f));
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

		Ts.centerRay.p = P;
		Ts.centerRay.dir = n * rays.centerRay.dir + temp;

		Ts.difx.p = P;
		Ts.difx.dir = n * rays.difx.dir + temp;

		Ts.dify.p = P;
		Ts.dify.dir = n * rays.dify.dir + temp;
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

Rays reflect(const Rays &rays, const cyPoint3f &N, const cyPoint3f &P) {
	Rays newRays;
	
	newRays.mainRay.dir = rays.mainRay.dir - (2.f *  N * (rays.mainRay.dir % N));
	newRays.mainRay.p = P;

	newRays.centerRay.dir = rays.centerRay.dir - (2.f *  N * (rays.centerRay.dir % N));
	newRays.centerRay.p = P;

	newRays.difx.dir = rays.difx.dir - (2.f *  N * (rays.difx.dir % N));
	newRays.difx.p = P;

	newRays.dify.dir = rays.dify.dir - (2.f *  N * (rays.dify.dir % N));
	newRays.dify.p = P;
	return newRays;
}

Color MtlBlinn::Shade(const Rays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const
{
	Color origCol, reflCol, refrCol;
	origCol.SetBlack();
	reflCol.SetBlack();
	refrCol.SetBlack();
	
	if (bounceCount > 0) {
		float frenselkr = 0.f;
		/* If the material has refraction */
		if (refraction.GetColor() != cyColor::Black()) {
			/* Compute how much light is absorped by the material. */
			Color absorped = cyColor::White();
			if (!hInfo.front) {
				float h = (hInfo.p - rays.mainRay.p).Length();
				absorped.r = pow(E, -h * absorption.r);
				absorped.g = pow(E, -h * absorption.g);
				absorped.b = pow(E, -h * absorption.b);
			}

			/* Refract the ray through the volume */
			bool internallyReflected;
			Rays refrRays = refract(rays, hInfo.N, hInfo.p, ior, hInfo.front, internallyReflected);
			
			/* Also compute how much of the ray was reflected due to the frensel effect. */
			frensel(rays.mainRay.dir, hInfo.N, ior, hInfo.front, frenselkr);

			/* If a ray is internally reflected, it does not refract, so we don't trace it's refraction. */
			if (!internallyReflected) {
				HitInfo refrInfo;
				Trace(refrRays, rootNode, refrInfo, HIT_FRONT_AND_BACK);

				/* If we hit something */
				if (refrInfo.node != NULL) {
					const Material *hitMat = refrInfo.node->GetMaterial();
					Color finalRefr = (refraction.Sample(refrInfo.uvw, refrInfo.duvw) - frenselkr) * absorped;
					finalRefr.r = MAX(finalRefr.r, 0);
					finalRefr.g = MAX(finalRefr.g, 0);
					finalRefr.b = MAX(finalRefr.b, 0);
					refrCol += hitMat->Shade(refrRays, refrInfo, lights, bounceCount - 1) * finalRefr;
				}

				/* Else sample from the environment map */
				else {
					refrCol += environment.SampleEnvironment(refrRays.mainRay.dir.GetNormalized()) * (refraction.Sample(refrInfo.uvw, refrInfo.duvw) - frenselkr);
				}
			}
		}

		/* If the material has reflection */
		if (reflection.GetColor() != cyColor::Black() || frenselkr != 0) {
			/* Compute the reflection along the surface */
			Rays reflRays = reflect(rays, hInfo.N, hInfo.p);
	
			/* Trace that reflection through the scene */
			HitInfo reflInfo;
			Trace(reflRays, rootNode, reflInfo, HIT_FRONT_AND_BACK);

			/* If we hit something. */
			if (reflInfo.node != NULL) {
				const Material *hitMat = reflInfo.node->GetMaterial();
				reflCol += hitMat->Shade(reflRays, reflInfo, lights, bounceCount - 1) * (reflection.Sample(reflInfo.uvw, reflInfo.duvw) + frenselkr);
			}

			/* else sample from the environment map */
			else {
				reflCol += environment.SampleEnvironment(reflRays.mainRay.dir.GetNormalized()) * (reflection.Sample(reflInfo.uvw, reflInfo.duvw) + frenselkr);
			}
		}
	}

	/* For each light in the scene */
	if (hInfo.front)
	{
		for (int i = 0; i < lights.size(); i++) {
			Light *l = lights.at(i);

			/* Try to illuminate this point using the light */
			Color li = l->Illuminate(hInfo.p, hInfo.N);

			/* Always add ambient */
			if (l->IsAmbient()) { origCol += diffuse.Sample(hInfo.uvw, hInfo.duvw) * li; } // REPLACE GETCOLOR
			else {
				/* Do Blinn */
				cyPoint3f ldir = -l->Direction(hInfo.p);
				float dot = (hInfo.N.GetNormalized() % ldir);

				/* If surface is facing the light */
				if (dot >= 0) {
					cyPoint3f H = ((ldir + -rays.mainRay.dir) / (ldir + -rays.mainRay.dir).Length()).GetNormalized();
					origCol += li * dot * (diffuse.Sample(hInfo.uvw, hInfo.duvw) + (specular.Sample(hInfo.uvw, hInfo.duvw) * pow(hInfo.N % H, glossiness))); // REPLACE GETCOLOR
				}
			}
		}
	}

	return origCol + reflCol + refrCol;
}
