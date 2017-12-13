#pragma once

extern Node rootNode;

/* RAY TRACE */
inline bool Trace(Rays rays, Node &n, HitInfo &info, int hitSide = HIT_FRONT, float t_max = BIGFLOAT) {
	bool hit = false;

	/* transform the main ray from parent space to child space */
	Rays transformedRays = n.ToNodeCoords(rays);

	/* If this node has an obj */
	if (n.GetNodeObj()) {
		/* If we hit that object's bounding box, test it*/
		Box bb = n.GetNodeObj()->GetBoundBox();
		if (bb.IntersectRay(transformedRays.mainRay, MIN(info.z, t_max))) {
			float oldZ = info.z;
			n.GetNodeObj()->IntersectRay(transformedRays, info, hitSide);

			/* If the z value decreased, this node is the closest node that the ray hit */
			if (info.z < oldZ) {
				info.node = &n;
				hit = true;
				info.d = transformedRays.mainRay.dir;
				/* Shadow rays can quit tracing at this point. */
				if (info.shadow && info.z < t_max)
					return true;
			}
		}
	}

	/* If the given ray doesn't hit the child bounding box, quit. */
	const Box &bb = n.GetChildBoundBox();
	if (!bb.IntersectRay(rays.mainRay, t_max)) return hit;

	Node *lastHitNode = 0;
	/* Recurse through the children */
	for (int i = 0; i < n.GetNumChild(); ++i) {
		const Node *previous = info.node;
		if (!Trace(transformedRays, *n.GetChild(i), info, hitSide, MIN(info.z, t_max))) continue;
		/* If this is a shadow trace and we hit something, return. */
		else if (info.shadow && info.z < t_max) return true;
		else if (info.z < t_max) hit = true;
		if (hit) lastHitNode = n.GetChild(i);
	}
	/* Transform hit data from child space to parent space */
	if (lastHitNode)
		lastHitNode->FromNodeCoords(info);
	return hit;
}

inline bool Trace(Ray ray, Node &n, HitInfo &info, int hitSide = HIT_FRONT, float t_max = BIGFLOAT) {
	return Trace(Rays(ray, Ray(), Ray()), n, info, hitSide, t_max);
}

#ifndef E
	#define E 2.71828182845904523536f
#endif 

inline void frensel(const cyPoint3f &I, const cyPoint3f &N, const float &ior, bool front, float &kr) {
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

inline Rays refract(const Rays &rays/*const cyPoint3f &I*/, const cyPoint3f &N, const cyPoint3f &P, const float &ior, bool front, bool &internRefl) {
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

inline Ray refract(const Ray &ray, const cyPoint3f &N, const cyPoint3f &P, const float &ior, bool front, bool &internRefl) {
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

inline Ray reflect(const Ray &ray, const cyPoint3f &N, const cyPoint3f &P) {
	Ray newRay;
	auto temp = (2.f *  N * (ray.dir % N));
	newRay.p = P;
	newRay.dir = ray.dir - temp;
	return newRay;
}

inline Rays reflect(const Rays &rays, const cyPoint3f &N, const cyPoint3f &P) {
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

inline bool TraceReflection(Ray oldRay, Ray &reflRay, HitInfo hitInfo, HitInfo &reflHitInfo, int hitside = HIT_FRONT_AND_BACK) {
	reflRay = reflect(oldRay, hitInfo.N, hitInfo.p);

	/* Trace that reflection through the scene */
	reflHitInfo = HitInfo(hitInfo.tid);
	return Trace(reflRay, rootNode, reflHitInfo, HIT_FRONT_AND_BACK);
}

inline bool TraceRefraction(Ray oldRay, Ray &refrRay, HitInfo &hitInfo, HitInfo &refrHitInfo, float &fkr, int hitside = HIT_FRONT_AND_BACK) {
	auto mat = hitInfo.node->GetMaterial();
	float ior = mat->GetIndexOfRefraction();
	bool internallyReflected;
	refrRay = refract(oldRay, hitInfo.N, hitInfo.p, ior, hitInfo.front, internallyReflected);

	/* Also compute how much of the ray was reflected due to the frensel effect. */
	if (!internallyReflected) {
		frensel(oldRay.dir, hitInfo.N, ior, hitInfo.front, fkr);
		refrHitInfo = HitInfo(hitInfo.tid);
		return Trace(refrRay, rootNode, refrHitInfo, HIT_FRONT_AND_BACK);
	}
	return false;
}
