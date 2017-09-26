#include "materials.h"
#include "render.h"

extern Camera camera;
extern RenderImage renderImage;
extern Node rootNode;
extern MaterialList materials;
extern LightList lights;

#define E 2.71828182845904523536f

Ray refract(const cyPoint3f &I, const cyPoint3f &N, const cyPoint3f &P, const float &ior, bool front, bool &internRefl/*, float &shlicks*/) {
	// T = nI + (nc1 - c2)N
	// n = iorV / iorT
	// c1 = N % I
	// c2 = sqrt(1 - pow(n,2) * (1 - pow(c1, 2))), if imaginary => internal reflection
	
	float cosi = MAX(-1.f, MIN(N % I, 1.f));
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

	//float r0 = pow(((iorV - iorT) / (iorV + iorT)), 2);
	//shlicks = (iorV == iorT) ? 0 : r0 + ((1 - r0) * pow(1 - cosi, 5));
	float n = iorV / iorT;
	float c2squared = 1.f - n * n * (1.f - cosi * cosi);

	Ray T;
	if (c2squared < 0.f) {
		internRefl = true;
	}
	else { 
		internRefl = false;
		T.p = P;
		T.dir = n * I + (n * cosi - sqrtf(c2squared)) * N_;
	}
	return T;
}

Ray refract2(const cyPoint3f &I, const cyPoint3f &N, const cyPoint3f &P, const float &ior, bool front, bool &internRefl/*, float &shlicks*/) {
	cyPoint3f S = N ^ ((N ^ -I) / (N ^ -I).Length());

	// T = -N*cos(theta2) + s*sin(theta2)
	float cosi = MAX(-1.f, MIN(N % I, 1.f));
	cyPoint3f N_ = N;
	float iorV = 1.f, iorT = ior;
	if (front) {
		// Hitting a front face, make sure NdotI is positive
		cosi = -cosi;
	}
	else {
		// Hitting back face, reverse surface normal, and swap iors
		N_ = -N;
		std::swap(iorV, iorT);
	}
	//sin(theta2) = (n1 / n2) * sin(theta1) = (n1 / n2) * sqrt(1 - cos^2(theta1)) = (n1 / n2) * sqrt(1 - (N % I) ^2)
	float eta = iorV / iorT;
	float sinTheta2 = eta * sqrt(1 - pow(cosi, 2));
	float cosTheta2 = sqrt(1 - pow(sinTheta2, 2));

	float c2squared = 1.f - eta * eta * (1.f - cosi * cosi);
	Ray T;
	if (c2squared < 0.f) {
		internRefl = true;
	}
	else {
		internRefl = false;
		cyPoint3f TDir = (-N_ * cosTheta2) + (S * sinTheta2);
		T.dir = TDir;
		T.p = P;
	}

	return T;
}


void frensel(const cyPoint3f &I, const cyPoint3f &N, const float &ior, bool front, float &kr) {
	float cosi = MAX(-1.f, MIN(N % I, 1.f));
	float iorV = 1.f, iorT = ior;

	if (!front) std::swap(iorV, iorT);
	float sint = iorV / iorT * std::sqrtf(MAX(0.f, 1.f - cosi * cosi));
	if (sint >= 1.f) kr = 1.f; // Total internal reflection.
	else {
		float cost = std::sqrtf(MAX(0.f, 1.f - sint * sint));
		cosi = fabsf(cosi);
		float Rs = ((iorT * cosi) - (iorV * cost)) / ((iorT * cosi) + (iorV * cost));
		float Rp = ((iorV * cosi) - (iorT * cost)) / ((iorV * cosi) + (iorT * cost));
		kr = (Rs * Rs + Rp * Rp) / 2.f;
	}
}

Ray reflect(const cyPoint3f &I, const cyPoint3f &N, const cyPoint3f &P) {
	Ray refl;
	refl.dir = I - (2.f *  N * (I % N));
	refl.p = P;
	return refl;
}

Color MtlBlinn::Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount) const
{
	Color c;
	c.SetBlack();
	Color origCol, reflCol, refrCol;
	origCol.SetBlack();
	reflCol.SetBlack();
	refrCol.SetBlack();
	
	if (bounceCount > 0) {
		// Compute refraction
		float kr = 0.f;
		bool internallyReflected;
		/* If the material has refraction */
		if (refraction.r > 0 && refraction.g > 0 && refraction.b > 0) {
			Color absorped;
			absorped.r = absorped.g = absorped.b = 1.0f;
			if (!hInfo.front) {
				float h = (hInfo.p - ray.p).Length();
				//absorped = absorption * h;
				//absPercentage = absorption * h;
				absorped.r = pow(E, -h * absorption.r);
				absorped.g = pow(E, -h * absorption.g);
				absorped.b = pow(E, -h * absorption.b);
			}

			frensel(ray.dir, hInfo.N, ior, hInfo.front, kr);
			Ray T = refract(ray.dir, hInfo.N, hInfo.p, ior, hInfo.front, internallyReflected);
			if (!internallyReflected) {
				HitInfo refrHInfo;
				Trace(T, rootNode, refrHInfo, HIT_FRONT_AND_BACK);

				if (refrHInfo.node != NULL) {
					const Material *hitMat = refrHInfo.node->GetMaterial();
					Color finalRefr = ((refraction * absorped) - kr);
					finalRefr.r = MAX(finalRefr.r, 0);
					finalRefr.g = MAX(finalRefr.g, 0);
					finalRefr.b = MAX(finalRefr.b, 0);
					refrCol += hitMat->Shade(T, refrHInfo, lights, bounceCount - 1) * finalRefr;
				}
			}
		}

		/* If the material has reflection */
		if ((reflection.r > 0 && reflection.g > 0 && reflection.b > 0) || kr != 0) {
			/* Compute reflection */
			Ray R = reflect(ray.dir, hInfo.N, hInfo.p);

			/* Trace that ray through the scene */
			HitInfo reflHInfo;
			Trace(R, rootNode, reflHInfo, HIT_FRONT_AND_BACK);

			/* If we hit something. */
			if (reflHInfo.node != NULL) {
				const Material *hitMat = reflHInfo.node->GetMaterial();
				reflCol += hitMat->Shade(R, reflHInfo, lights, bounceCount - 1) * (reflection + kr);
			}
		}
	}

	/* material color */

	/* For each light in the scene */
	if (hInfo.front)
	{
		for (int i = 0; i < lights.size(); i++) {
			Light *l = lights.at(i);

			/* Try to illuminate this point using the light */
			Color li = l->Illuminate(hInfo.p, hInfo.N);

			/* Always add ambient */
			if (l->IsAmbient()) { origCol += diffuse * li; }
			else {
				/* Do Blinn */
				cyPoint3f ldir = -l->Direction(hInfo.p);
				float dot = (hInfo.N.GetNormalized() % ldir);

				/* If surface is facing the light */
				if (dot >= 0) {
					cyPoint3f H = ((ldir + -ray.dir) / (ldir + -ray.dir).Length()).GetNormalized();
					origCol += li * dot * (diffuse + (specular * pow(hInfo.N % H, glossiness)));
				}
			}
		}
	}

	return origCol + reflCol + refrCol;
}
